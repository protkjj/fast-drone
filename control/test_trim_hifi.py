"""표 기반 트림 솔버 검증 — 2026-09-25 밤 작업.

여기서 지키려는 것: (1) 단순화 모델의 트림이 실제 물리에서도 트림인지
구분할 수 있는지, (2) 저속가지 경계가 research 의 독립 계산과 맞는지
(교차검증), (3) 고속가지가 진짜로 존재하는지와 그 경계가 어디인지.
"""
import numpy as np
import pytest

from control.trim_hifi import default_seeds, find_trim_hifi, required_motor_current
from control.vehicle_params import load_selected_params

P = load_selected_params()


def test_hover_trim_matches_simple_model():
    """J=0 이라 단순화 모델과 정확히 같은 해가 나와야 한다."""
    r = find_trim_hifi(P, 0.0)
    assert r['converged']
    n = r['state'][13:17]
    n_hov = np.sqrt(P['mass']*P['g']/(4*P['k_T']))
    np.testing.assert_allclose(n, n_hov, rtol=1e-4)


def test_low_branch_boundary_matches_research_independent_value():
    """research 가 독립적으로 보고한 19.63 m/s 와 하이파이 솔버가 맞는지.

    이게 이 솔버 전체의 신뢰성 근거다 — 서로 다른 코드베이스, 같은 물리.
    """
    assert find_trim_hifi(P, 19.5)['converged']
    assert not find_trim_hifi(P, 20.0)['converged']


def test_gap_is_empty_between_20_and_80():
    """저속가지·고속가지 사이 간극에서 트림이 없어야 한다(정상상태, 가속 아님)."""
    for V in (25.0, 40.0, 60.0, 78.0, 80.0):
        assert not find_trim_hifi(P, V)['converged'], f"V={V} 에서 예상 밖 트림"


def test_high_branch_exists_and_covers_85():
    """목표 85 m/s 를 포함하는 고속가지가 실제로 있어야 한다."""
    for V in (82.0, 85.0, 90.0, 95.0):
        r = find_trim_hifi(P, V)
        assert r['converged'], f"V={V} 트림 실패"
        assert r['residual_norm'] < 1e-6


def test_high_branch_reopens_near_81_not_68():
    """2026-09-25 정정: 단순화 모델 기준(J_max 결함)으로는 67.99 로 잘못
    나왔었다. 실제(표 기반) 재개방은 81 근방이다 — 정확한 값은
    control/night_hifi_branch.py 가 이분탐색으로 낸다. 여기선 굵은 경계만."""
    assert not find_trim_hifi(P, 80.5)['converged']
    assert find_trim_hifi(P, 81.5)['converged']


def test_simplified_model_trim_can_be_unphysical_at_high_advance_ratio():
    """단순화 모델(J_max)로 찾은 '트림'이 실제 표로는 트림이 아닐 수 있다는
    것 자체를 재현한다 — 이 모듈이 필요해진 이유의 회귀 시험.
    """
    from control.dynamics import AxialDronePlant
    from control.dynamics_hifi import AxialDronePlantHighFidelity
    from control.trim import find_trim

    tr = find_trim(P, 70.0, guess=[np.radians(88.0),
                   np.sqrt(P['mass']*P['g']/(4*P['k_T']))*1.6, 0.0], quiet=True)
    assert tr['converged']
    simple_residual = AxialDronePlant(P, dt=0.001).evaluate_xdot(
        tr['state'], tr['control'])[3]
    hifi_residual = AxialDronePlantHighFidelity(P, dt=0.001).evaluate_xdot(
        tr['state'], tr['control'])[3]
    assert abs(simple_residual) < 1e-6         # 단순화 모델 기준으로는 트림
    assert abs(hifi_residual) > 1.0            # 실제 물리로는 전혀 트림이 아님


def test_required_current_uncapped_exceeds_capped_on_high_branch():
    """85 m/s 근방에서 실제 요구 전류가 40A 가정을 넘는지(research 교차검증).

    research 독립 보고값: 83.3 m/s 에서 45.61A(한계 39.95A 대비 +14.2%).
    """
    r = find_trim_hifi(P, 85.0)
    assert r['converged']
    n = r['state'][13:17]
    V_axial = 85.0   # 이 트림은 거의 수평(θ≈87.6°)이라 축방향 유입 ≈ V
    result = required_motor_current(P, n, V_axial)
    i_req_rear = result['I_uncapped'][2]
    assert i_req_rear > P['I_lim']              # 40A 가정을 실제로 넘는다
    assert 40.0 < i_req_rear < 55.0              # research 45.61A 근방 규모
