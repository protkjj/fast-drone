"""
Hybrid + LQR 폴백 제어기
=========================

몬테카를로 결과: Hybrid가 60% 승률이지만 최악 21.67(LQR의 20배).
→ Hybrid를 기본으로, 이상 감지 시 LQR로 자동 전환.

감지 기준 (하나라도 걸리면 전환):
  1. NaN/Inf 출력
  2. 모터 명령 진동 (짧은 윈도우 내 분산 과다)
  3. 고도 오차 과대 (|z - z_ref| > 임계값)

전환 동작:
  Hybrid → LQR: 즉시 전환 (bumpless 아닌 hard switch)
  LQR → Hybrid 복귀: 쿨다운(2초) + 상태 안정 확인
  채터링 방지: 복귀 후 최소 1초는 Hybrid 유지

왜 bumpless transfer를 안 하나:
  Hybrid가 발산할 때의 출력은 이미 비정상이므로,
  그걸 기준으로 blend하면 LQR까지 오염됨.
  LQR은 안정적이라 hard switch 후 자체 수렴.
"""

import numpy as np


class HybridWithFallback:
    """
    ProperHybrid + ScheduledLQR 폴백.

    사용법:
        hybrid = ProperHybrid(vnmpc, P, dt=dt)
        lqr = ScheduledLQR(P, v_ref=..., z_ref=...)
        ctrl = HybridWithFallback(hybrid, lqr, z_ref=50.0)
        u = ctrl(t, x)
    """

    def __init__(self, hybrid_ctrl, lqr_ctrl, z_ref=50.0,
                 z_err_limit=5.0, var_window=50, var_limit=1e5,
                 cooldown_sec=2.0, min_hybrid_sec=1.0, dt=0.001):
        """
        Parameters
        ----------
        hybrid_ctrl : ProperHybrid
        lqr_ctrl : ScheduledLQR
        z_ref : float
            기준 고도 [m] (고도 오차 감지용).
        z_err_limit : float
            고도 오차 임계값 [m]. 이 이상이면 폴백.
        var_window : int
            모터 명령 분산 계산 윈도우 [스텝]. 50 = 50ms @ 1kHz.
        var_limit : float
            모터 명령 분산 임계값. 이 이상이면 진동으로 판단.
        cooldown_sec : float
            LQR 전환 후 Hybrid 복귀까지 최소 대기 시간 [s].
        min_hybrid_sec : float
            Hybrid 복귀 후 최소 유지 시간 [s] (채터링 방지).
        dt : float
            제어 주기 [s].
        """
        self.hybrid = hybrid_ctrl
        self.lqr = lqr_ctrl
        self.z_ref = z_ref

        # 감지 파라미터
        self.z_err_limit = z_err_limit
        self.var_window = var_window
        self.var_limit = var_limit

        # 타이밍
        self.cooldown_steps = int(cooldown_sec / dt)
        self.min_hybrid_steps = int(min_hybrid_sec / dt)
        self.dt = dt

        # 내부 상태
        self._using_hybrid = True
        self._switch_count = 0         # Hybrid→LQR 전환 횟수
        self._steps_since_switch = 0   # 마지막 전환 이후 스텝
        self._cmd_history = []         # 최근 모터 명령 (분산 계산용)

    def __call__(self, t, x):
        """제어기 호출."""
        if self._using_hybrid:
            u = self._run_hybrid(t, x)
            need_fallback = self._check_fallback(u, x)

            if need_fallback:
                # Hybrid → LQR 전환
                self._using_hybrid = False
                self._switch_count += 1
                self._steps_since_switch = 0
                self._cmd_history.clear()
                # LQR로 즉시 전환
                u = self.lqr(t, x)
        else:
            u = self.lqr(t, x)
            self._steps_since_switch += 1

            # 복귀 조건: 쿨다운 경과 + 상태 안정
            if self._steps_since_switch >= self.cooldown_steps:
                if self._check_stable(x):
                    self._using_hybrid = True
                    self._steps_since_switch = 0
                    self._cmd_history.clear()
                    # 복귀 시 Hybrid 내부 상태 리셋
                    if hasattr(self.hybrid, 'reset'):
                        self.hybrid.reset()

        return u

    def _run_hybrid(self, t, x):
        """Hybrid 실행 (예외 포착)."""
        try:
            u = self.hybrid(t, x)
            return np.array(u, dtype=float)
        except Exception:
            return np.full(4, float('nan'))

    def _check_fallback(self, u, x):
        """폴백 필요 여부 판단."""
        # 채터링 방지: 복귀 직후엔 전환하지 않음
        if self._steps_since_switch < self.min_hybrid_steps and self._switch_count > 0:
            return False

        # 1. NaN/Inf
        if np.any(np.isnan(u)) or np.any(np.isinf(u)):
            return True

        # 2. 고도 오차 과대
        z_err = abs(x[2] - self.z_ref)
        if z_err > self.z_err_limit:
            return True

        # 3. 모터 명령 진동 (분산 기반)
        self._cmd_history.append(u.copy())
        if len(self._cmd_history) > self.var_window:
            self._cmd_history.pop(0)

        if len(self._cmd_history) >= self.var_window:
            arr = np.array(self._cmd_history)
            var = np.mean(np.var(arr, axis=0))
            if var > self.var_limit:
                return True

        self._steps_since_switch += 1
        return False

    def _check_stable(self, x):
        """Hybrid 복귀 가능한 안정 상태인지 확인."""
        z_err = abs(x[2] - self.z_ref)
        vel_mag = np.linalg.norm(x[3:6])
        omega_mag = np.linalg.norm(x[10:13])

        # 고도 오차 < 2m, 각속도 < 1 rad/s
        return z_err < 2.0 and omega_mag < 1.0

    def reset(self):
        """전체 리셋."""
        self._using_hybrid = True
        self._switch_count = 0
        self._steps_since_switch = 0
        self._cmd_history.clear()
        if hasattr(self.hybrid, 'reset'):
            self.hybrid.reset()
        if hasattr(self.lqr, 'reset'):
            self.lqr.reset()
        # NMPC 타이밍 리셋
        if hasattr(self.hybrid, 'nmpc') and hasattr(self.hybrid.nmpc, '_last_t'):
            self.hybrid.nmpc._last_t = -np.inf

    @property
    def fallback_count(self):
        """폴백 발생 횟수."""
        return self._switch_count

    @property
    def active_controller(self):
        """현재 활성 제어기 이름."""
        return 'Hybrid' if self._using_hybrid else 'LQR'
