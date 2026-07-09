"""
센서 모델 — IMU + GPS + 기압계
===============================

실기체에서 상태를 직접 관측할 수 없으므로,
센서로 측정 → 노이즈/바이어스 포함 → EKF로 추정.

센서 모델링 목적:
  1. 파이썬 시뮬에서 "완벽 상태 가정"을 깨고 현실적 검증
  2. EKF 추정기 개발·튜닝
  3. 제어기가 노이즈에 버티는지 확인 (특히 INDI)

센서 스펙 (일반적인 MEMS 기준):
  ┌─────────────┬──────────────────┬───────────┐
  │ 센서        │ 노이즈 (σ)       │ 주파수    │
  ├─────────────┼──────────────────┼───────────┤
  │ 가속도계    │ 0.02 m/s²        │ 1 kHz     │
  │ 자이로      │ 0.001 rad/s      │ 1 kHz     │
  │ GPS 위치    │ 1.5 m            │ 10 Hz     │
  │ GPS 속도    │ 0.5 m/s          │ 10 Hz     │
  │ 기압계      │ 0.5 m            │ 25 Hz     │
  └─────────────┴──────────────────┴───────────┘
"""

import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass, field


@dataclass
class SensorData:
    """한 타임스텝의 센서 데이터."""
    t: float

    # IMU (매 스텝)
    acc_body: np.ndarray     # 가속도계 [m/s²] (비력, 동체 프레임)
    gyro_body: np.ndarray    # 자이로 [rad/s] (동체 프레임)

    # GPS (gps_valid가 True일 때만 유효)
    gps_valid: bool = False
    gps_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))   # [m] (관성)
    gps_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))   # [m/s] (관성)

    # 기압계 (baro_valid가 True일 때만 유효)
    baro_valid: bool = False
    baro_alt: float = 0.0    # [m] (z-up 고도)


class IMUSensor:
    """
    IMU 센서 모델 — 가속도계 + 자이로.

    가속도계가 측정하는 것 (비력, specific force):
      f = R^T @ (a_inertial + g_inertial)
      여기서 a_inertial = dv/dt (관성 가속도), g = [0, 0, g] (z-up)

    왜 중력이 더해지나:
      가속도계는 "중력을 뺀 가속도"가 아니라 "중력에 대항하는 힘"을 측정.
      정지 시 가속도계 = +g (위를 가리킴). 자유낙하 시 = 0.

    자이로가 측정하는 것:
      ω_body = 동체 각속도 (그대로)

    바이어스 모델:
      bias = bias_init + random_walk (v1에서는 상수)
    """

    def __init__(self, noise_acc=0.02, noise_gyro=0.001,
                 bias_acc=None, bias_gyro=None, seed=None):
        """
        Parameters
        ----------
        noise_acc : float
            가속도계 노이즈 표준편차 [m/s²].
        noise_gyro : float
            자이로 노이즈 표준편차 [rad/s].
        bias_acc : array(3) or None
            가속도계 바이어스 [m/s²]. None이면 랜덤 생성.
        bias_gyro : array(3) or None
            자이로 바이어스 [rad/s]. None이면 랜덤 생성.
        seed : int or None
            난수 시드 (재현성).
        """
        self.rng = np.random.default_rng(seed)
        self.noise_acc = noise_acc
        self.noise_gyro = noise_gyro

        # 바이어스: 지정 안 하면 현실적 범위에서 랜덤 생성
        if bias_acc is not None:
            self.bias_acc = np.array(bias_acc, dtype=float)
        else:
            self.bias_acc = self.rng.normal(0, 0.05, 3)  # ~0.05 m/s²

        if bias_gyro is not None:
            self.bias_gyro = np.array(bias_gyro, dtype=float)
        else:
            self.bias_gyro = self.rng.normal(0, 0.005, 3)  # ~0.005 rad/s

    def measure(self, x_true, acc_inertial, g=9.81):
        """
        진짜 상태 → IMU 측정값.

        Parameters
        ----------
        x_true : array(17)
            진짜 상태 벡터.
        acc_inertial : array(3)
            관성 프레임 가속도 [m/s²] (xdot[3:6]에서 추출).
        g : float
            중력 가속도.

        Returns
        -------
        acc_meas : array(3)
            가속도계 측정 (비력, 동체 프레임) [m/s²].
        gyro_meas : array(3)
            자이로 측정 (동체 프레임) [rad/s].
        """
        q = x_true[6:10]
        omega = x_true[10:13]

        # 회전 행렬: 동체→관성
        R = Rotation.from_quat(q).as_matrix()

        # 비력 = R^T @ (a + g), 여기서 g=[0,0,+g] (z-up에서 위 방향)
        #
        # 왜 +g인가:
        #   동역학에서 v_dot = [0,0,-g] + F/m 이므로
        #   a_inertial = v_dot = [0,0,-g] + F/m
        #   비력 = a_inertial - gravity_vec = a_inertial - [0,0,-g] = a_inertial + [0,0,+g]
        #   = F/m (순수 힘 가속도)
        #
        #   정지 시: a=0, 비력 = [0,0,+g] → 가속도계가 위를 가리킴 ✓
        specific_force = acc_inertial + np.array([0, 0, g])
        acc_body_true = R.T @ specific_force

        # 측정 = 진짜 + 바이어스 + 노이즈
        acc_meas = (acc_body_true
                    + self.bias_acc
                    + self.rng.normal(0, self.noise_acc, 3))
        gyro_meas = (omega
                     + self.bias_gyro
                     + self.rng.normal(0, self.noise_gyro, 3))

        return acc_meas, gyro_meas


class GPSSensor:
    """
    GPS 센서 모델 — 위치 + 속도.

    특성:
      - 저주파 (10 Hz): 매 스텝이 아니라 gps_period마다 출력
      - 높은 위치 노이즈 (~1.5m): IMU 대비 부정확
      - 속도 노이즈 (~0.5 m/s): 위치보다 정확 (도플러)
      - 지연: v1에서는 무시 (나중에 추가)
    """

    def __init__(self, dt_plant, gps_rate=10.0,
                 noise_pos=1.5, noise_vel=0.5, seed=None):
        """
        Parameters
        ----------
        dt_plant : float
            플랜트 시뮬레이션 스텝 [s].
        gps_rate : float
            GPS 출력 주파수 [Hz].
        noise_pos : float
            위치 노이즈 표준편차 [m].
        noise_vel : float
            속도 노이즈 표준편차 [m/s].
        """
        self.rng = np.random.default_rng(seed)
        self.noise_pos = noise_pos
        self.noise_vel = noise_vel

        # GPS 주기를 플랜트 스텝 수로 변환
        self.period_steps = max(1, int(round(1.0 / (gps_rate * dt_plant))))
        self._step_count = 0

    def measure(self, x_true):
        """
        진짜 상태 → GPS 측정값 (gps_rate마다).

        Returns
        -------
        (pos_meas, vel_meas) or None
            None이면 이번 스텝에서 GPS 출력 없음.
        """
        self._step_count += 1
        if self._step_count % self.period_steps != 0:
            return None

        pos_true = x_true[0:3]
        vel_true = x_true[3:6]

        pos_meas = pos_true + self.rng.normal(0, self.noise_pos, 3)
        vel_meas = vel_true + self.rng.normal(0, self.noise_vel, 3)

        return pos_meas, vel_meas

    def reset(self):
        self._step_count = 0


class SensorSuite:
    """
    센서 묶음 — IMU + GPS.

    한 번의 measure() 호출로 모든 센서 데이터를 SensorData로 묶어 반환.
    """

    def __init__(self, imu, gps, g=9.81):
        self.imu = imu
        self.gps = gps
        self.g = g

    def measure(self, t, x_true, acc_inertial):
        """
        모든 센서 측정 수행.

        Parameters
        ----------
        t : float
            현재 시각 [s].
        x_true : array(17)
            진짜 상태 벡터.
        acc_inertial : array(3)
            관성 가속도 (xdot[3:6]).

        Returns
        -------
        SensorData
        """
        acc_meas, gyro_meas = self.imu.measure(x_true, acc_inertial, self.g)

        gps_result = self.gps.measure(x_true)

        data = SensorData(
            t=t,
            acc_body=acc_meas,
            gyro_body=gyro_meas,
        )

        if gps_result is not None:
            data.gps_valid = True
            data.gps_pos = gps_result[0]
            data.gps_vel = gps_result[1]

        return data

    def reset(self):
        self.gps.reset()


def create_default_sensors(dt_plant=0.001, noise_level=1.0, seed=42):
    """
    기본 센서 세트 생성.

    Parameters
    ----------
    noise_level : float
        1.0 = 일반 MEMS, 0.5 = 저노이즈, 2.0 = 고노이즈.
        노이즈 스윕 비교에 사용.
    seed : int
        재현성 시드.
    """
    imu = IMUSensor(
        noise_acc=0.02 * noise_level,
        noise_gyro=0.001 * noise_level,
        seed=seed,
    )
    gps = GPSSensor(
        dt_plant=dt_plant,
        gps_rate=10.0,
        noise_pos=1.5 * noise_level,
        noise_vel=0.5 * noise_level,
        seed=seed + 1,
    )
    return SensorSuite(imu, gps)
