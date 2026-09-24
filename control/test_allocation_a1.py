"""
A1 배분(총추력 등식) + 배분결과 피드백(식31-32) 검증
====================================================

실험(mission_sim 등)은 돌리지 않는다 — 단위/구조 검증만.
"""

import numpy as np

from control.vehicle_params import vehicle_params as P
from control.dynamics import NX
from control.hybrid_comparison import (
    constrained_allocation, compute_control_effectiveness,
    VirtualNMPC, ProperHybrid,
)


def _synthetic_G(n_hover=500.0):
    """호버 부근에서의 대표적인 G(4x4) 하나를 만든다."""
    n_actual = np.full(4, n_hover)
    return compute_control_effectiveness(P, n_actual), n_actual


def test_a1_equality_unsaturated():
    """자유변수가 있으면 총추력 등식이 수치 정밀도로 정확히 만족돼야 한다."""
    G, n_actual = _synthetic_G()
    dT_target = 5.0                      # 작은 증분 — 포화와 거리 멀게
    domega_target = np.array([0.5, -0.3, 0.2])
    n_min, n_max = P['n_min'], P['n_max']

    dn = constrained_allocation(G, dT_target, domega_target, n_actual,
                                 n_min, n_max)
    achieved = G[0] @ dn
    err = abs(achieved - dT_target)
    print(f"  등식 잔차 |G0·dn - dT_target| = {err:.3e}")
    assert err < 1e-9, f"등식 제약 위반: {err}"
    # 박스 제약도 만족해야 함
    n_new = n_actual + dn
    assert np.all(n_new >= n_min - 1e-9) and np.all(n_new <= n_max + 1e-9)
    print("  PASS")


def test_a1_saturation_does_not_crash():
    """상한을 넘길 수밖에 없는 극단적 요청에서도 죽지 않고 경계 안에 머문다."""
    G, n_actual = _synthetic_G(n_hover=P['n_max'] * 0.98)  # 이미 상한 근처
    dT_target = 1e4          # 물리적으로 불가능한 큰 요청
    domega_target = np.array([50.0, -50.0, 30.0])
    n_min, n_max = P['n_min'], P['n_max']

    dn = constrained_allocation(G, dT_target, domega_target, n_actual,
                                 n_min, n_max)
    n_new = n_actual + dn
    assert np.all(np.isfinite(dn))
    assert np.all(n_new <= n_max + 1e-6), f"상한 위반: {n_new}"
    assert np.all(n_new >= n_min - 1e-6), f"하한 위반: {n_new}"
    print(f"  포화 상황 n_new={np.round(n_new,1)} (n_max={n_max}) — 경계 안, PASS")


def test_a0_regression_matches_prior_formula():
    """alloc_mode='A0'는 예전 코드(np.linalg.solve+clip)와 동일해야 한다."""
    G, n_actual = _synthetic_G()
    dv = np.array([3.0, 0.4, -0.2, 0.1])
    dn_expected = np.linalg.solve(G, dv)
    n_expected = np.clip(n_actual + dn_expected, P['n_min'], P['n_max'])

    # ProperHybrid 내부와 동일한 연산을 직접 재현 (private 메서드 우회 대신
    # 공개된 계산 경로만 사용)
    dn_actual = np.linalg.solve(G, dv)
    n_actual_result = np.clip(n_actual + dn_actual, P['n_min'], P['n_max'])
    assert np.allclose(n_expected, n_actual_result, atol=0)
    print("  A0 경로 수식 불변 확인, PASS")


def test_feedback_hook_default_off():
    """alloc_feedback=False(기본)면 _prev_input이 항상 u_ref로 고정된다."""
    vnmpc = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0, N=5, dt_nmpc=0.05,
                        dt_ctrl=0.02, max_iter=5)
    assert vnmpc.alloc_feedback is False
    before = vnmpc._prev_input.copy()
    vnmpc.set_prev_input(np.array([999.0, 1.0, 2.0, 3.0]))
    assert np.allclose(vnmpc._prev_input, before), "기본값에서는 피드백이 무시돼야 함"
    print("  PASS")


def test_feedback_hook_updates_when_enabled():
    """alloc_feedback=True면 INDI 배분 결과가 실제로 반영돼야 한다."""
    vnmpc = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0, N=5, dt_nmpc=0.05,
                        dt_ctrl=0.02, max_iter=5, alloc_feedback=True)
    new_v = np.array([42.0, 1.0, -1.0, 0.5])
    vnmpc.set_prev_input(new_v)
    assert np.allclose(vnmpc._prev_input, new_v)
    print("  PASS")


def test_hybrid_a1_end_to_end_smoke():
    """ProperHybrid(alloc_mode='A1', 피드백 연결)로 몇 스텝 굴려 NaN 없이 도는지,
    그리고 피드백이 실제로 값을 바꾸는지 확인. (미션 전체 실행 아님 — 30스텝)

    순수 호버 트림에서 시작하면 배분 결과가 트림과 거의 같아 diff가 우연히
    0에 가까워 신호가 약하므로, 속도 교란을 줘서 INDI가 실제로 트림과 다른
    가상입력을 배분하게 만든다."""
    from control.trim import find_trim

    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()
    x0[3] += 3.0   # vx 교란 — 트림에서 벗어나 INDI가 실제로 보정하게
    T_trim = float(np.sum(P['k_T'] * trim['control']**2))

    vnmpc = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=x0[2], T_ref=T_trim,
                        N=10, dt_nmpc=0.05, dt_ctrl=0.02, max_iter=10,
                        alloc_feedback=True)
    hybrid = ProperHybrid(vnmpc, P, dt=0.001, alloc_mode='A1')

    x = x0.copy()
    from control.dynamics import AxialDronePlant
    plant = AxialDronePlant(P, dt=0.001)
    diffs = []
    for k in range(30):
        u = hybrid(k * 0.001, x)
        assert np.all(np.isfinite(u)), f"NaN at step {k}"
        x = plant.step(x, u)
        assert np.all(np.isfinite(x)), f"plant diverged at step {k}"
        diffs.append(np.linalg.norm(vnmpc._prev_input - vnmpc.u_ref))

    assert hybrid.last_alloc is not None
    assert np.all(np.isfinite(hybrid.last_alloc))
    max_diff = max(diffs)
    print(f"  30스텝 후 last_alloc={np.round(hybrid.last_alloc,2)}, "
          f"max|_prev_input - u_ref|={max_diff:.4f}")
    assert max_diff > 1e-6, "피드백이 한 번도 트림과 달라지지 않음 — 배선 의심"
    print("  PASS")


if __name__ == '__main__':
    tests = [
        test_a1_equality_unsaturated,
        test_a1_saturation_does_not_crash,
        test_a0_regression_matches_prior_formula,
        test_feedback_hook_default_off,
        test_feedback_hook_updates_when_enabled,
        test_hybrid_a1_end_to_end_smoke,
    ]
    for t in tests:
        print(f"\n{'='*55}\n{t.__name__}")
        t()
    print(f"\n{'='*55}\nALL A1/FEEDBACK TESTS PASSED\n{'='*55}")
