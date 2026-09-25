"""
BatteryModel + NMPCController(electrical_constraints) 검증.
⚠ 논문 v5.3 범위 밖 — 전류·전압·배터리는 모델에서 제외되고 고정 회전수
한계로 대체된다(논문 132행·608행). 실기 쪽 자산으로 남겨 두는 테스트다.
아래 식번호는 이 모듈 자체의 전력수지 식이며 논문 식번호가 아니다.
"""

import numpy as np

from control.vehicle_params import vehicle_params as P
from control.battery import BatteryModel
from control.nmpc import NMPCController
from control.trim import find_trim
from control.dynamics import AxialDronePlant


def test_battery_hover_steady_state():
    """호버 부근 정상상태에서 V_b<V_oc, I_b>0, SOC가 단조 감소해야 한다."""
    n_hov = np.sqrt(P['mass'] * P['g'] / (4 * P['k_T']))
    n_vec = np.full(4, n_hov)
    batt = BatteryModel(P, s0=1.0)

    V_b, I_b, s, I_i = batt.step(n_vec, n_vec, dt=0.001)  # 명령=실제(정상상태)
    print(f"  n_hover={n_hov:.1f}, V_b={V_b:.3f}, I_b={I_b:.3f}, s={s:.6f}, I_i={np.round(I_i,3)}")
    assert 0 < V_b < P['V_oc']
    assert I_b > 0
    assert s < 1.0
    assert np.all(I_i >= 0)
    print("  PASS")


def test_battery_hand_calc_matches_formula():
    """수렴 후(고정점 반복 후) 결과가 식(13) 전력수지를 직접 만족하는지 손검산."""
    n_vec = np.full(4, 500.0)
    batt = BatteryModel(P, s0=1.0)
    V_b, I_b, s, I_i = batt.step(n_vec, n_vec, dt=0.001)

    # 손계산: 같은 V_b·I_i로 전력수지가 실제로 맞는지 역산
    P_e_check = np.sum((P['k_e']*n_vec + P['R_m']*I_i) * I_i) / P['eta_esc']
    I_b_check = P_e_check / V_b
    V_b_check = P['V_oc'] - P['R_b'] * I_b_check
    print(f"  V_b={V_b:.5f} vs 역산 V_b_check={V_b_check:.5f}")
    assert abs(V_b - V_b_check) < 1e-4, "고정점 반복이 수렴하지 않음"
    print("  PASS")


def test_battery_soc_depletes_over_time():
    """여러 스텝 반복하면 SOC가 계속 줄어야 한다(방전만, 충전 없음)."""
    n_vec = np.full(4, 600.0)
    batt = BatteryModel(P, s0=1.0)
    s_prev = 1.0
    for _ in range(100):
        _, _, s, _ = batt.step(n_vec, n_vec, dt=0.01)
        assert s <= s_prev + 1e-12
        s_prev = s
    print(f"  100스텝(1s) 후 SOC={s_prev:.6f} (<1.0)")
    assert s_prev < 1.0
    print("  PASS")


def test_m17_electrical_constraints_default_off_unaffected():
    """electrical_constraints=False(기본)면 기존과 동일하게 잘 풀려야 한다(회귀)."""
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()
    nmpc = NMPCController(P, v_ref=[0, 0, 0], z_ref=x0[2], u_ref=trim['control'],
                          N=10, dt_nmpc=0.05, dt_ctrl=0.02)
    u = nmpc(0.0, x0)
    assert nmpc._solve_log[-1] == 'Solve_Succeeded'
    print(f"  기본(E0) 정상 수렴, u={np.round(u,1)}")
    print("  PASS")


def test_m17_electrical_constraints_on_solves_and_respects_bounds():
    """electrical_constraints=True(E1)로도 수렴하고, 결과가 물리적으로 합당한지."""
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()
    nmpc = NMPCController(P, v_ref=[0, 0, 0], z_ref=x0[2], u_ref=trim['control'],
                          N=10, dt_nmpc=0.05, dt_ctrl=0.02,
                          electrical_constraints=True)
    u = nmpc(0.0, x0)
    print(f"  E1 status={nmpc._solve_log[-1]}, u={np.round(u,1)}, V_b={nmpc.V_b}")
    assert nmpc._solve_log[-1] in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
    assert np.all(np.isfinite(u))
    assert np.all(u >= P['n_min'] - 1e-6) and np.all(u <= P['n_max'] + 1e-6)
    print("  PASS")


def test_m17_e0_e1_ablation_smoke():
    """표6의 E0↔E1 — 같은 시나리오에서 두 결과가 (보통) 달라야 어블레이션 의미가 있다."""
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()
    x0[3] += 3.0

    n0 = NMPCController(P, v_ref=[0, 0, 0], z_ref=x0[2], u_ref=trim['control'],
                        N=10, dt_nmpc=0.05, dt_ctrl=0.02,
                        electrical_constraints=False)
    n1 = NMPCController(P, v_ref=[0, 0, 0], z_ref=x0[2], u_ref=trim['control'],
                        N=10, dt_nmpc=0.05, dt_ctrl=0.02,
                        electrical_constraints=True)
    n1.V_b = 30.0  # 방전된 배터리 가정 — 전압제약이 실제로 걸리게

    u0 = n0(0.0, x0)
    u1 = n1(0.0, x0)
    print(f"  E0 u={np.round(u0,1)}  E1(V_b=30V) u={np.round(u1,1)}")
    assert np.all(np.isfinite(u1))
    print("  PASS (수렴/유한성만 확인 — 실제 수치 비교는 본 실험 몫)")


if __name__ == '__main__':
    tests = [
        test_battery_hover_steady_state,
        test_battery_hand_calc_matches_formula,
        test_battery_soc_depletes_over_time,
        test_m17_electrical_constraints_default_off_unaffected,
        test_m17_electrical_constraints_on_solves_and_respects_bounds,
        test_m17_e0_e1_ablation_smoke,
    ]
    for t in tests:
        print(f"\n{'='*55}\n{t.__name__}")
        t()
    print(f"\n{'='*55}\nALL BATTERY/E0-E1 TESTS PASSED\n{'='*55}")
