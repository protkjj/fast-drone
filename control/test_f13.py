"""
F13(로터추력 직접입력 NMPC+INDI) 단위 검증 — 미션 전체 실행 아님.
"""

import numpy as np

from control.vehicle_params import vehicle_params as P
from control.trim import find_trim
from control.dynamics import AxialDronePlant
from control.nmpc_f13 import RotorThrustNMPC13, F13VirtualAdapter, build_f13_controller


def test_single_solve_feasible():
    """1회 솔브 결과가 f∈[0,f_max]^4 안에 들어오고 차원이 맞는지."""
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()

    nmpc = RotorThrustNMPC13(P, v_ref=[0, 0, 0], z_ref=x0[2], N=10,
                             dt_nmpc=0.05, dt_ctrl=0.02, max_iter=20)
    f = nmpc(0.0, x0)
    print(f"  f = {np.round(f,3)}  (f_max={nmpc.f_max:.1f})")
    assert f.shape == (4,)
    assert np.all(np.isfinite(f))
    assert np.all(f >= -1e-6) and np.all(f <= nmpc.f_max + 1e-6)
    print("  PASS")


def test_hover_trim_converges():
    """호버 트림에서 솔브가 성공하고, 4개 로터 추력 합이 대략 mg에 가까운지."""
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()
    mg = P['mass'] * P['g']

    nmpc = RotorThrustNMPC13(P, v_ref=[0, 0, 0], z_ref=x0[2], N=15,
                             dt_nmpc=0.05, dt_ctrl=0.02, max_iter=30)
    f = nmpc(0.0, x0)
    T_sum = np.sum(f)
    print(f"  sum(f)={T_sum:.2f} N vs mg={mg:.2f} N, status={nmpc.last_status}")
    assert nmpc.last_status in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
    assert abs(T_sum - mg) / mg < 0.15, "호버 트림에서 총추력이 mg와 너무 다름"
    print("  PASS")


def test_adapter_conversion_matches_allocation_matrix():
    """F13VirtualAdapter의 f→[T,ω̇] 변환이 compute_allocation_matrix와 일관되는지."""
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()

    adapter = F13VirtualAdapter(P, v_ref=[0, 0, 0], z_ref=x0[2], N=5,
                                dt_nmpc=0.05, dt_ctrl=0.02, max_iter=10)
    vc = adapter(0.0, x0)
    assert vc.shape == (4,)
    assert np.all(np.isfinite(vc))
    T_total = vc[0]
    print(f"  adapter 출력 [T,ω̇] = {np.round(vc,3)}")
    assert T_total > 0
    print("  PASS")


def test_f13_hybrid_end_to_end_smoke():
    """build_f13_controller로 몇 스텝 굴려 NaN 없이 도는지 (미션 실행 아님, 25스텝)."""
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()
    x0[3] += 2.0  # 속도 교란

    ctrl = build_f13_controller(P, v_ref=[0, 0, 0], z_ref=x0[2], dt=0.001,
                                N=10, dt_nmpc=0.05, dt_ctrl=0.02, max_iter=15)
    plant = AxialDronePlant(P, dt=0.001)
    x = x0.copy()
    for k in range(25):
        u = ctrl(k * 0.001, x)
        assert np.all(np.isfinite(u)), f"NaN at step {k}"
        assert np.all(u >= P['n_min'] - 1e-6) and np.all(u <= P['n_max'] + 1e-6)
        x = plant.step(x, u)
        assert np.all(np.isfinite(x)), f"plant diverged at step {k}"
    print(f"  25스텝 후 z={x[2]:.3f}, vx={x[3]:.3f} — 발산 없음")
    print("  PASS")


if __name__ == '__main__':
    tests = [
        test_single_solve_feasible,
        test_hover_trim_converges,
        test_adapter_conversion_matches_allocation_matrix,
        test_f13_hybrid_end_to_end_smoke,
    ]
    for t in tests:
        print(f"\n{'='*55}\n{t.__name__}")
        t()
    print(f"\n{'='*55}\nALL F13 TESTS PASSED\n{'='*55}")
