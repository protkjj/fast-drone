"""
안전장치 — 직접 모터 제어 시 필수 보호 메커니즘
================================================

PX4의 ActuatorMotors 직접 제어는 PX4의 모든 안정화·보호를 끈다.
우리 제어기가 100% 책임지므로, 아래 보호가 없으면
NaN 1개 = 즉시 추락.

보호 레이어:
  1. NaN/Inf 감지 → 마지막 유효 출력 또는 호버 폴백
  2. 출력 변화율 제한 (Rate Limiter) → 급격한 명령 변화 방지
  3. 자세 제한 → 과도 틸트 시 감속/호버 전환
  4. 워치독 → 제어기 응답 없으면 호버 폴백
  5. 고도 하한 → 지면 충돌 방지

사용법:
  guard = SafetyGuard(n_max=1800, dt=0.01)
  u_safe = guard.check(u_raw, state)
  if guard.triggered:
      logger.warn(f"안전장치 발동: {guard.trigger_reason}")
"""

import numpy as np
from enum import Enum


class SafetyLevel(Enum):
    """안전 상태."""
    NOMINAL = 0      # 정상
    WARNING = 1      # 경고 (소프트 제한 발동)
    FAILSAFE = 2     # 폴백 (호버 모터 속도)
    EMERGENCY = 3    # 비상 (모터 정지 — 최후 수단)


class SafetyGuard:
    """
    제어 출력 안전장치.

    모든 제어기 출력이 이 클래스를 거쳐서 PX4로 전달.
    여러 보호 레이어를 순차 적용.
    """

    def __init__(self, n_max, dt, hover_rpm=None):
        """
        Parameters
        ----------
        n_max : float
            최대 모터 속도 [rad/s].
        dt : float
            제어 주기 [s].
        hover_rpm : float or None
            호버 모터 속도 [rad/s].
            None이면 n_max * 0.32 사용 (T/W~10 기체의 호버점 근사).
        """
        self.n_max = n_max
        self.dt = dt

        # 호버 폴백 속도
        if hover_rpm is None:
            # n_hov ≈ sqrt(mg / (4*k_T)) ≈ 572 for our vehicle
            hover_rpm = n_max * 0.32
        self.hover_cmd = np.full(4, hover_rpm)

        # ── 내부 상태 ──
        self._last_valid_u = self.hover_cmd.copy()
        self._watchdog_count = 0
        self._consecutive_nan = 0

        # ── 보호 파라미터 ──
        # 출력 변화율 제한: 최대 Δn/Δt [rad/s²]
        # 1800 rad/s를 0.1초에 변경 → 18000 rad/s²
        self.max_rate = 18000.0  # rad/s per second

        # 자세 제한: 최대 틸트 각도 [rad]
        self.max_tilt = np.radians(70.0)

        # 고도 하한 [m] (NWU z-up 기준)
        self.min_altitude = 1.0

        # NaN 허용 횟수: 이 횟수 초과 시 EMERGENCY
        self.max_consecutive_nan = 50  # 50 스텝 = 0.5초 @ 100Hz

        # 워치독: 제어기가 응답 안 하면 호버
        self.watchdog_limit = 100  # 100 스텝 = 1초 @ 100Hz

        # ── 출력 상태 ──
        self.level = SafetyLevel.NOMINAL
        self.trigger_reason = ""
        self.triggered = False

    def check(self, u_raw, state=None):
        """
        제어 출력 안전 검사 + 보정.

        Parameters
        ----------
        u_raw : array(4)
            제어기 원시 출력 [rad/s]. NaN 가능.
        state : array(17) or None
            현재 상태 벡터 (자세/고도 체크용).
            None이면 자세/고도 체크 생략.

        Returns
        -------
        u_safe : array(4)
            안전 처리된 모터 명령 [rad/s].
        """
        self.triggered = False
        self.level = SafetyLevel.NOMINAL
        self.trigger_reason = ""

        u = np.array(u_raw, dtype=float)

        # ── Layer 1: NaN/Inf 감지 ──
        u = self._check_nan(u)

        # ── Layer 2: 범위 클램핑 ──
        u = np.clip(u, 0.0, self.n_max)

        # ── Layer 3: 변화율 제한 ──
        u = self._rate_limit(u)

        # ── Layer 4: 자세/고도 체크 ──
        if state is not None:
            u = self._check_state(u, state)

        # ── 최종 기록 ──
        if self.level == SafetyLevel.NOMINAL:
            self._last_valid_u = u.copy()
            self._watchdog_count = 0

        return u

    def _check_nan(self, u):
        """NaN/Inf 감지 → 마지막 유효 출력 폴백."""
        if np.any(np.isnan(u)) or np.any(np.isinf(u)):
            self._consecutive_nan += 1
            self.triggered = True

            if self._consecutive_nan >= self.max_consecutive_nan:
                # 연속 NaN 한계 초과 → 호버 폴백
                self.level = SafetyLevel.FAILSAFE
                self.trigger_reason = (
                    f"연속 NaN {self._consecutive_nan}회 → 호버 폴백"
                )
                return self.hover_cmd.copy()
            else:
                # 일시적 NaN → 마지막 유효값 유지
                self.level = SafetyLevel.WARNING
                self.trigger_reason = (
                    f"NaN 감지 ({self._consecutive_nan}회) → 마지막 유효 출력"
                )
                return self._last_valid_u.copy()
        else:
            self._consecutive_nan = 0
            return u

    def _rate_limit(self, u):
        """출력 변화율 제한."""
        max_delta = self.max_rate * self.dt  # 한 스텝 최대 변화량
        delta = u - self._last_valid_u
        delta_clipped = np.clip(delta, -max_delta, max_delta)

        if not np.allclose(delta, delta_clipped):
            u_limited = self._last_valid_u + delta_clipped
            self.triggered = True
            if self.level.value < SafetyLevel.WARNING.value:
                self.level = SafetyLevel.WARNING
            self.trigger_reason += " | 변화율 제한 발동"
            return u_limited

        return u

    def _check_state(self, u, state):
        """자세/고도 기반 보호."""
        # 쿼터니언에서 틸트 각도 추출
        q = state[6:10]
        # 틸트 = body z-axis와 inertial -z의 각도
        # body z in inertial = R @ [0,0,1]
        # R의 3번째 열 = [2(qx*qz+qy*qw), 2(qy*qz-qx*qw), 1-2(qx²+qy²)]
        qx, qy, qz, qw = q
        bz_inertial = np.array([
            2*(qx*qz + qy*qw),
            2*(qy*qz - qx*qw),
            1 - 2*(qx**2 + qy**2),
        ])
        # 우리 좌표계: z-up, body z-down → 호버 시 bz_inertial = [0,0,-1]
        # cos(tilt) = -bz_inertial[2] (body z와 inertial -z의 내적)
        cos_tilt = -bz_inertial[2]
        cos_tilt = np.clip(cos_tilt, -1.0, 1.0)
        tilt = np.arccos(cos_tilt)

        if tilt > self.max_tilt:
            # 과도 틸트 → 호버 폴백
            self.triggered = True
            self.level = SafetyLevel.FAILSAFE
            self.trigger_reason += (
                f" | 틸트 {np.degrees(tilt):.0f}° > "
                f"한계 {np.degrees(self.max_tilt):.0f}°"
            )
            return self.hover_cmd.copy()

        # 고도 하한 체크
        altitude = state[2]  # NWU z-up
        if altitude < self.min_altitude:
            # 고도 너무 낮으면 추력 증가
            self.triggered = True
            if self.level.value < SafetyLevel.WARNING.value:
                self.level = SafetyLevel.WARNING
            self.trigger_reason += (
                f" | 저고도 경고 z={altitude:.1f}m"
            )
            # 호버 이상의 추력 보장
            u = np.maximum(u, self.hover_cmd)

        return u

    def reset(self):
        """상태 초기화."""
        self._last_valid_u = self.hover_cmd.copy()
        self._consecutive_nan = 0
        self._watchdog_count = 0
        self.level = SafetyLevel.NOMINAL
        self.trigger_reason = ""
        self.triggered = False
