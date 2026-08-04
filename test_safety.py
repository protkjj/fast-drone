"""SafetyGuard 검증 — 최후 폴백 회전 감쇠(리뷰 Issue 2 근본 개선) 포함.

safety.py는 ros2 패키지에 있지만 상대 import가 없어 직접 import 가능.
실행: python3 test_safety.py
"""
import sys
import os

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl'))
from safety import SafetyGuard, SafetyLevel          # noqa: E402

from vehicle_params import vehicle_params as P       # noqa: E402
from dynamics import compute_allocation_matrix       # noqa: E402

F_TO_TM, TM_TO_F = compute_allocation_matrix(P)
HOVER = float(np.sqrt(P['mass'] * P['g'] / (4 * P['k_T'])))
RESCUE = dict(TM_to_f=TM_TO_F, k_T=P['k_T'],
              J_diag=np.array([P['Ixx'], P['Iyy'], P['Izz']]),
              k_omega=5.0)


def _state(z=50.0, omega=(0.0, 0.0, 0.0), tilt_deg=0.0):
    x = np.zeros(17)
    x[2] = z
    # 호버 자세(qx=1) + 필요시 y축 틸트
    th = np.radians(tilt_deg) / 2
    x[6], x[8] = np.cos(th), np.sin(th)   # Rx(180°)⊗Ry(tilt): [cos,0,sin,0]
    x[10:13] = omega
    x[13:17] = HOVER
    return x


def _achieved_TM(guard_out):
    f = P['k_T'] * np.asarray(guard_out)**2
    return F_TO_TM @ f


def _force_failsafe(g, state):
    """연속 NaN 한계 초과로 FAILSAFE 폴백 유도."""
    out = None
    for _ in range(g.max_consecutive_nan + 1):
        out = g.check(np.full(4, np.nan), state=state)
    assert g.level == SafetyLevel.FAILSAFE
    return out


def test_rescue_damps_rotation():
    print("TEST 1: 폴백 회전 감쇠 — 롤 ω에 반대 모멘트")
    g = SafetyGuard(n_max=P['n_max'], dt=0.01, hover_rpm=HOVER,
                    attitude_rescue=RESCUE)
    g.max_rate = 1e9                     # rate limiter 간섭 제거 (폴백 성분 검증)
    out = _force_failsafe(g, _state(omega=(5.0, 0.0, 0.0)))
    TM = _achieved_TM(out)
    expect_Mx = -RESCUE['k_omega'] * P['Ixx'] * 5.0     # -0.5 N·m
    assert abs(TM[1] - expect_Mx) < 0.05, f"Mx={TM[1]} (기대 {expect_Mx})"
    assert abs(TM[0] - P['mass'] * P['g']) < 1.0, f"T={TM[0]}"
    assert np.std(out) > 0.1, "차동 없음 (제로모멘트?)"
    print(f"  Mx={TM[1]:+.3f} (기대 {expect_Mx:+.3f}), T={TM[0]:.1f}N  PASS\n")


def test_rescue_downgrades_on_nan_omega():
    print("TEST 2: ω 비유한 → 제로모멘트 호버로 강등")
    g = SafetyGuard(n_max=P['n_max'], dt=0.01, hover_rpm=HOVER,
                    attitude_rescue=RESCUE)
    g.max_rate = 1e9
    out = _force_failsafe(g, _state(omega=(np.nan, 0.0, 0.0)))
    assert np.allclose(out, HOVER), f"강등 실패: {out}"
    print("  4모터 동일 호버로 강등 ✓  PASS\n")


def test_no_rescue_backcompat():
    print("TEST 3: attitude_rescue 미설정 — 기존 제로모멘트 동작 보존")
    g = SafetyGuard(n_max=P['n_max'], dt=0.01, hover_rpm=HOVER)
    g.max_rate = 1e9
    out = _force_failsafe(g, _state(omega=(5.0, 0.0, 0.0)))
    assert np.allclose(out, HOVER)
    print("  PASS\n")


def test_tilt_fallback_uses_rescue():
    print("TEST 4: 과도 틸트 폴백도 감쇠 사용")
    g = SafetyGuard(n_max=P['n_max'], dt=0.01, hover_rpm=HOVER,
                    attitude_rescue=RESCUE)
    g.max_rate = 1e9
    out = g.check(np.full(4, HOVER), state=_state(tilt_deg=80.0,
                                                  omega=(0.0, 3.0, 0.0)))
    assert g.level == SafetyLevel.FAILSAFE
    TM = _achieved_TM(out)
    assert TM[2] < -0.1, f"피치 감쇠 모멘트 없음: My={TM[2]}"
    print(f"  My={TM[2]:+.2f} (ω_y=3 반대)  PASS\n")


def test_low_altitude_preserves_differential():
    print("TEST 5: 저고도 부스트 — 차동(모멘트) 보존")
    g = SafetyGuard(n_max=P['n_max'], dt=0.01, hover_rpm=HOVER)
    g.max_rate = 1e9
    u_in = np.array([400.0, 500.0, 600.0, 700.0])
    out = g.check(u_in, state=_state(z=0.5))
    d_in = u_in - u_in.mean()
    d_out = out - out.mean()
    assert np.allclose(d_in, d_out, atol=1e-6), "차동 변형됨"
    assert out.mean() >= HOVER - 1e-6, "호버 평균 미달"
    print(f"  차동 보존 ✓, 평균 {out.mean():.0f} ≥ 호버 {HOVER:.0f}  PASS\n")


if __name__ == '__main__':
    print("\nSafetyGuard 검증 (회전 감쇠 폴백 포함)\n" + "=" * 55)
    test_rescue_damps_rotation()
    test_rescue_downgrades_on_nan_omega()
    test_no_rescue_backcompat()
    test_tilt_fallback_uses_rescue()
    test_low_altitude_preserves_differential()
    print("=" * 55)
    print("ALL TESTS PASSED")
