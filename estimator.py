"""
Error-State Kalman Filter (ESKF) — 15D 오차상태
=================================================

왜 "오차상태" EKF인가:
  일반 EKF는 상태를 직접 추정하지만, 쿼터니언은 4D인데 3자유도밖에 없어서
  공분산 행렬이 특이(singular)해짐. ESKF는 이 문제를 깔끔하게 해결:

  1. **명목(nominal) 상태**: IMU로 직접 적분 (비선형, 대신호)
     p, v, q → IMU 가속도/자이로로 전진 적분
  2. **오차(error) 상태**: 명목과 진짜의 차이 (선형, 소신호)
     δx = [δp, δv, δθ, δb_a, δb_g]  ← 15D (쿼터니언 4→각도 3)
  3. GPS 등 측정이 오면 오차상태를 보정, 명목에 주입, 오차 리셋

  결과: 쿼터니언을 3D 각도 오차로 다뤄서 공분산이 잘 정의됨.

구조:
  ┌────────────────────────────────────┐
  │ 매 스텝 (1 kHz, IMU rate)          │
  │  acc, gyro → 명목 적분             │
  │  F, Q → 오차 공분산 전파 P = FPF'+Q │
  ├────────────────────────────────────┤
  │ GPS 도착 시 (10 Hz)                │
  │  z = [pos, vel]_gps                │
  │  혁신 = z - h(명목)                │
  │  K = PH'(HPH'+R)^-1               │
  │  δx = K · 혁신                     │
  │  명목 ← 명목 + δx (주입)            │
  │  P ← (I-KH)P (리셋)               │
  └────────────────────────────────────┘
"""

import numpy as np
from scipy.spatial.transform import Rotation


def _skew(v):
    """3D 벡터 → 반대칭 행렬 [v]×."""
    return np.array([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0],
    ])


class ESKF:
    """
    Error-State Kalman Filter.

    오차상태 15D:
      idx 0:3   δp    위치 오차 [m]
      idx 3:6   δv    속도 오차 [m/s]
      idx 6:9   δθ    자세 오차 [rad] (소각도 회전 벡터)
      idx 9:12  δb_a  가속도계 바이어스 오차 [m/s²]
      idx 12:15 δb_g  자이로 바이어스 오차 [rad/s]

    명목상태:
      pos(3), vel(3), quat(4), bias_acc(3), bias_gyro(3) = 16D
    """

    DIM_ERR = 15   # 오차상태 차원
    DIM_NOM = 16   # 명목상태 차원

    def __init__(self, x0, g=9.81,
                 P0_pos=1.0, P0_vel=0.5, P0_att=0.01,
                 P0_ba=0.1, P0_bg=0.01,
                 Q_acc=0.02, Q_gyro=0.001,
                 Q_ba=1e-4, Q_bg=1e-5):
        """
        Parameters
        ----------
        x0 : array(17)
            초기 상태 (우리 상태 벡터). 여기서 명목상태 추출.
        g : float
            중력 가속도 [m/s²].
        P0_* : float
            초기 오차 공분산 대각 원소.
        Q_* : float
            프로세스 노이즈 대각 원소.
            Q_acc, Q_gyro: IMU 노이즈 (센서 스펙과 매칭).
            Q_ba, Q_bg: 바이어스 랜덤워크 (느리게 변하는 바이어스).
        """
        self.g = g
        self.g_vec = np.array([0, 0, -g])  # 중력 (관성, z-up)

        # ── 명목상태 초기화 ──
        self.pos = x0[0:3].copy()
        self.vel = x0[3:6].copy()
        self.q = x0[6:10].copy()          # scalar-last [x,y,z,w]
        self.bias_acc = np.zeros(3)        # 바이어스는 0에서 시작 (추정할 것)
        self.bias_gyro = np.zeros(3)

        # ── 오차 공분산 ──
        self.P = np.diag([
            P0_pos, P0_pos, P0_pos,        # δp
            P0_vel, P0_vel, P0_vel,        # δv
            P0_att, P0_att, P0_att,        # δθ
            P0_ba, P0_ba, P0_ba,           # δb_a
            P0_bg, P0_bg, P0_bg,           # δb_g
        ])

        # ── 프로세스 노이즈 (연속시간 → 이산: Q_d ≈ Q_c * dt) ──
        # dt는 predict()에서 적용
        self._Q_diag = np.array([
            0, 0, 0,                       # δp: 위치는 속도 통해 간접 (별도 노이즈 없음)
            Q_acc**2, Q_acc**2, Q_acc**2,  # δv: 가속도계 노이즈
            Q_gyro**2, Q_gyro**2, Q_gyro**2,  # δθ: 자이로 노이즈
            Q_ba**2, Q_ba**2, Q_ba**2,     # δb_a: 바이어스 드리프트
            Q_bg**2, Q_bg**2, Q_bg**2,     # δb_g: 바이어스 드리프트
        ])

    def update_gps(self, pos_meas, vel_meas, R_pos=1.5, R_vel=0.5):
        """
        GPS 측정 업데이트 — 위치 + 속도.

        Parameters
        ----------
        pos_meas : array(3)
            GPS 위치 측정 [m] (관성 프레임).
        vel_meas : array(3)
            GPS 속도 측정 [m/s] (관성 프레임).
        R_pos : float
            위치 측정 노이즈 표준편차 [m].
        R_vel : float
            속도 측정 노이즈 표준편차 [m/s].
        """
        # 측정 모델: z = [pos; vel] = h(nominal) + H @ δx + noise
        # H는 6×15: 위치와 속도만 관측
        #
        #        δp   δv   δθ   δba  δbg
        #  pos [ I    0    0    0    0  ]   → 위치 오차는 δp
        #  vel [ 0    I    0    0    0  ]   → 속도 오차는 δv
        H = np.zeros((6, self.DIM_ERR))
        H[0:3, 0:3] = np.eye(3)   # pos → δp
        H[3:6, 3:6] = np.eye(3)   # vel → δv

        # 측정 노이즈 공분산
        R = np.diag([R_pos**2]*3 + [R_vel**2]*3)

        # 혁신 (innovation): z - h(nominal)
        innovation = np.concatenate([
            pos_meas - self.pos,
            vel_meas - self.vel,
        ])

        # 칼만 게인
        S = H @ self.P @ H.T + R         # 6×6
        K = self.P @ H.T @ np.linalg.inv(S)  # 15×6

        # 오차상태 보정
        dx = K @ innovation               # 15D

        # 명목상태에 주입 (injection)
        self._inject(dx)

        # 공분산 업데이트 (Joseph form for numerical stability)
        IKH = np.eye(self.DIM_ERR) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

        self.P = 0.5 * (self.P + self.P.T)

    def _inject(self, dx):
        """
        오차상태를 명목상태에 주입.

        dx(15) = [δp, δv, δθ, δb_a, δb_g]
        """
        self.pos += dx[0:3]
        self.vel += dx[3:6]

        # 자세 오차 주입: q ← q ⊗ exp(δθ/2)
        dtheta = dx[6:9]
        angle = np.linalg.norm(dtheta)
        if angle > 1e-10:
            axis = dtheta / angle
            dq = np.zeros(4)
            dq[0:3] = axis * np.sin(angle / 2)
            dq[3] = np.cos(angle / 2)
        else:
            dq = np.array([*(dtheta / 2), 1.0])
            dq = dq / np.linalg.norm(dq)

        self.q = self._quat_mult(self.q, dq)
        self.q = self.q / np.linalg.norm(self.q)

        # 바이어스 보정
        self.bias_acc += dx[9:12]
        self.bias_gyro += dx[12:15]

    def update_accel(self, acc_body, R_acc=3.0):
        """
        가속도계 기반 자세(roll/pitch) 업데이트.

        원리:
          호버나 등속비행 시, 가속도계는 주로 중력을 측정:
            a_body ≈ R^T @ [0, 0, g]
          이 값을 명목 자세에서 예측한 값과 비교하면
          roll/pitch 오차를 보정할 수 있음.

        주의:
          급기동 시에는 가속도가 중력만이 아니므로
          R_acc를 크게 해서 신뢰도를 낮춤.

        Parameters
        ----------
        acc_body : array(3)
            가속도계 측정 (비력) [m/s²].
        R_acc : float
            가속도 측정 노이즈 표준편차 [m/s²].
            클수록 업데이트가 약해짐 (급기동 시 키울 것).
        """
        R = Rotation.from_quat(self.q).as_matrix()

        # 명목 상태에서 예상되는 가속도계 읽기 (= R^T @ g)
        g_body_expected = R.T @ np.array([0, 0, self.g])

        # 바이어스 보정된 측정
        acc_corrected = acc_body - self.bias_acc

        # 급기동 감지: 비력 크기가 g에서 벗어나면 신뢰도 감소
        acc_mag = np.linalg.norm(acc_corrected)
        deviation = abs(acc_mag - self.g) / self.g
        # deviation 0 = 중력만 = 높은 신뢰, deviation > 0.3 = 급기동 = 낮은 신뢰
        adaptive_R = R_acc * (1.0 + 10.0 * deviation)

        innovation = acc_corrected - g_body_expected

        # 야코비안: h(x) = R^T·g + b_a
        # ∂h/∂δθ = [R^T·g]×  (자세 오차 → 중력 방향 변화)
        # ∂h/∂δb_a = +I      (바이어스 오차 → 측정 예측 증가)
        H = np.zeros((3, self.DIM_ERR))
        H[:, 6:9] = _skew(g_body_expected)
        H[:, 9:12] = np.eye(3)

        R_meas = np.eye(3) * adaptive_R**2

        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ innovation

        self._inject(dx)
        IKH = np.eye(self.DIM_ERR) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R_meas @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def get_state(self):
        """
        추정 상태를 우리 상태 벡터 x(17)로 반환.

        x = [pos(3), vel(3), quat(4), omega(3), motors(4)]

        omega: 자이로 - 추정 바이어스 (마지막 predict()에서 사용한 값)
        motors: 추정 불가 → 0 반환 (호출측에서 채워야 함)
        """
        x = np.zeros(17)
        x[0:3] = self.pos
        x[3:6] = self.vel
        x[6:10] = self.q
        # 각속도는 저장된 가장 최근 바이어스 보정 자이로 사용
        x[10:13] = self._last_gyro_corrected if hasattr(self, '_last_gyro_corrected') \
                   else np.zeros(3)
        # 로터 속도는 추정기에서 모름 → 호출측이 채움
        x[13:17] = 0.0
        return x

    def get_estimated_biases(self):
        """추정된 바이어스 반환 (디버깅용)."""
        return self.bias_acc.copy(), self.bias_gyro.copy()

    def predict(self, acc_body, gyro_body, dt):
        """
        예측 단계 — IMU 데이터로 명목상태 적분 + 오차 공분산 전파.

        Parameters
        ----------
        acc_body : array(3)
            가속도계 측정 (비력, 동체 프레임) [m/s²].
        gyro_body : array(3)
            자이로 측정 (동체 프레임) [rad/s].
        dt : float
            시간 스텝 [s].
        """
        # ── 1. 바이어스 보정 ──
        self._last_gyro_corrected = gyro_body - self.bias_gyro
        acc_corrected = acc_body - self.bias_acc
        gyro_corrected = self._last_gyro_corrected

        # ── 2. 명목상태 적분 ──
        R = Rotation.from_quat(self.q).as_matrix()  # 동체→관성

        # 관성 가속도 = R @ 비력(동체) + 중력
        acc_inertial = R @ acc_corrected + self.g_vec

        self.pos = self.pos + self.vel * dt + 0.5 * acc_inertial * dt**2
        self.vel = self.vel + acc_inertial * dt

        # 쿼터니언 적분: q ← q ⊗ exp(ω·dt)
        angle = np.linalg.norm(gyro_corrected) * dt
        if angle > 1e-10:
            axis = gyro_corrected / np.linalg.norm(gyro_corrected)
            dq = np.zeros(4)
            dq[0:3] = axis * np.sin(angle / 2)
            dq[3] = np.cos(angle / 2)
        else:
            dq = np.array([*(gyro_corrected * dt / 2), 1.0])
            dq = dq / np.linalg.norm(dq)

        self.q = self._quat_mult(self.q, dq)
        self.q = self.q / np.linalg.norm(self.q)

        # ── 3. 오차상태 야코비안 F (15×15) ──
        #
        #        δp   δv       δθ          δb_a   δb_g
        # δp  [  I    I·dt     0            0      0   ]
        # δv  [  0    I    -R[a_c×]·dt     -R·dt   0   ]
        # δθ  [  0    0    I-[ω_c×]·dt      0    -I·dt ]
        # δba [  0    0        0            I      0   ]
        # δbg [  0    0        0            0      I   ]
        F = np.eye(self.DIM_ERR)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -R @ _skew(acc_corrected) * dt
        F[3:6, 9:12] = -R * dt
        F[6:9, 6:9] = np.eye(3) - _skew(gyro_corrected) * dt
        F[6:9, 12:15] = -np.eye(3) * dt

        # ── 4. 공분산 전파 ──
        Q = np.diag(self._Q_diag * dt)
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)  # 대칭성 강제

    @staticmethod
    def _quat_mult(p, q):
        """Hamilton product p ⊗ q, scalar-last [x,y,z,w]."""
        px, py, pz, pw = p
        qx, qy, qz, qw = q
        return np.array([
            pw*qx + px*qw + py*qz - pz*qy,
            pw*qy - px*qz + py*qw + pz*qx,
            pw*qz + px*qy - py*qx + pz*qw,
            pw*qw - px*qx - py*qy - pz*qz,
        ])
