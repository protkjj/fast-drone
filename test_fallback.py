"""HybridWithFallback 전환 로직 검증 — NMPC 미수렴 트리거 포함.

스텁 기반 단위 테스트 4개 + 실제 Hybrid 스모크 1개 (순항 5s, 오탐 없음 확인).
실행: python3 test_fallback.py  (~20초, ScheduledLQR 빌드 + 5s 시뮬 포함)
"""
import numpy as np

from fallback_controller import HybridWithFallback


# ══════════════════════════════════════════════════
# 스텁 (Hybrid/LQR 흉내 — 전환 로직만 격리 검증)
# ══════════════════════════════════════════════════

class _StubNMPC:
    def __init__(self):
        self.consec_fail = 0


class _StubHybrid:
    """정상 모터명령 600을 내는 가짜 Hybrid. u_out/consec_fail만 조작."""

    def __init__(self):
        self.nmpc = _StubNMPC()
        self.u_out = np.full(4, 600.0)
        self.reset_count = 0

    def __call__(self, t, x):
        return self.u_out

    def reset(self):
        self.reset_count += 1
        self.nmpc.consec_fail = 0


class _StubLQR:
    def __call__(self, t, x):
        return np.full(4, 500.0)

    def reset(self):
        pass


def _hover_x(z=50.0):
    x = np.zeros(17)
    x[2] = z
    x[6] = 1.0
    x[13:17] = 572.0
    return x


def _make():
    hyb = _StubHybrid()
    ctrl = HybridWithFallback(hyb, _StubLQR(), z_ref=50.0)
    return hyb, ctrl


def test_no_spurious():
    """정상 상태 200스텝 — 전환 없어야 함."""
    print("=" * 55)
    print("TEST 1: 정상 상태에서 오탐 없음")
    hyb, ctrl = _make()
    x = _hover_x()
    for k in range(200):
        u = ctrl(k * 0.001, x)
    assert ctrl.fallback_count == 0, f"오탐! count={ctrl.fallback_count}"
    assert ctrl.active_controller == 'Hybrid'
    print("  200스텝 후 count=0, active=Hybrid  PASS\n")


def test_nmpc_fail_trigger():
    """연속 미수렴 2회 → 유지, 3회(=limit) → LQR 전환."""
    print("=" * 55)
    print("TEST 2: NMPC 연속 미수렴 트리거 (limit=3)")
    hyb, ctrl = _make()
    x = _hover_x()

    hyb.nmpc.consec_fail = 2
    u = ctrl(0.0, x)
    assert ctrl.active_controller == 'Hybrid', "2회에 조기 전환!"
    print("  consec_fail=2 → Hybrid 유지 ✓")

    hyb.nmpc.consec_fail = 3
    u = ctrl(0.001, x)
    assert ctrl.active_controller == 'LQR', "3회에 미전환!"
    assert np.allclose(u, 500.0), "전환 스텝에 LQR 출력이 아님"
    assert ctrl.fallback_count == 1
    print("  consec_fail=3 → LQR 전환 + LQR 출력 반환 ✓  PASS\n")
    return ctrl, hyb


def test_recovery_resets(ctrl, hyb):
    """쿨다운(2s) 후 안정 상태면 Hybrid 복귀 + reset 호출로 카운터 초기화."""
    print("=" * 55)
    print("TEST 3: 쿨다운 후 복귀 + 카운터 리셋")
    x = _hover_x()          # 안정 상태 (z 오차 0, omega 0)
    for k in range(2100):   # cooldown_steps=2000 초과
        ctrl(0.002 + k * 0.001, x)
    assert ctrl.active_controller == 'Hybrid', "복귀 실패"
    assert hyb.reset_count >= 1, "복귀 시 hybrid.reset() 미호출"
    assert hyb.nmpc.consec_fail == 0
    # 복귀 후 유지 (카운터가 리셋됐으므로 재전환 없어야)
    for k in range(100):
        ctrl(2.5 + k * 0.001, x)
    assert ctrl.active_controller == 'Hybrid'
    assert ctrl.fallback_count == 1, "복귀 후 재전환 발생 (채터링)"
    print("  2s 쿨다운 후 Hybrid 복귀, reset 호출, 재전환 없음  PASS\n")


def test_nan_immediate():
    """NaN 출력은 미수렴 카운터와 무관하게 즉시 전환 (기존 동작 회귀 확인)."""
    print("=" * 55)
    print("TEST 4: NaN 즉시 전환 (회귀)")
    hyb, ctrl = _make()
    hyb.u_out = np.full(4, np.nan)
    u = ctrl(0.0, _hover_x())
    assert ctrl.active_controller == 'LQR'
    assert np.allclose(u, 500.0)
    print("  NaN → 1스텝 만에 LQR  PASS\n")


def test_smoke_real_hybrid():
    """실제 ProperHybrid+ScheduledLQR, 70m/s 순항 5s — 새 트리거의 오탐 없음."""
    print("=" * 55)
    print("TEST 5: 실제 Hybrid 스모크 (순항 5s, 오탐 검사)")
    from vehicle_params import vehicle_params as P
    from dynamics import AxialDronePlant
    from trim import find_trim
    from controller import ScheduledLQR
    from hybrid_comparison import VirtualNMPC, ProperHybrid

    plant = AxialDronePlant(P, dt=0.001)
    trim = find_trim(P, 70.0)
    x0 = trim['state'].copy()
    x0[2] = 50.0
    T_trim = float(np.sum(P['k_T'] * trim['control']**2))

    vn = VirtualNMPC(P, v_ref=[70, 0, 0], z_ref=50.0, T_ref=T_trim)
    hyb = ProperHybrid(vn, P, dt=plant.dt)
    lqr = ScheduledLQR(P, v_ref=[70, 0, 0], z_ref=50.0)
    ctrl = HybridWithFallback(hyb, lqr, z_ref=50.0)

    ts, xs, us = plant.simulate(x0, ctrl, 5.0)

    z_err = abs(xs[-1, 2] - 50.0)
    print(f"  5s 후: z오차={z_err:.3f}m, fallback_count={ctrl.fallback_count}, "
          f"last_status={vn.last_status}, consec_fail={vn.consec_fail}")
    assert ctrl.fallback_count == 0, "순항에서 오탐 전환!"
    assert z_err < 1.0, f"순항 고도 오차 과대: {z_err}"
    print("  PASS\n")


if __name__ == '__main__':
    print("\nHybridWithFallback 전환 로직 검증\n")
    test_no_spurious()
    c, h = test_nmpc_fail_trigger()
    test_recovery_resets(c, h)
    test_nan_immediate()
    test_smoke_real_hybrid()
    print("=" * 55)
    print("ALL TESTS PASSED")
    print("=" * 55)
