"""연구용 경로가 웹의 body +x 기하 및 실제 플랜트 미분과 일치하는지 검사."""
import copy

import casadi as ca
import numpy as np
import pytest

from control.dynamics import AxialDronePlant, build_dynamics, _rotor_forces_moments
from control.hybrid_comparison import build_virtual_dynamics, compute_control_effectiveness
from control.vehicle_params import rocket_params, selected_params, vehicle_params


@pytest.mark.parametrize('params', [vehicle_params, rocket_params])
def test_hover_state_points_thrust_up(params):
    x = AxialDronePlant.hover_state(params)
    f, _, _ = build_dynamics(params)
    derivative = np.asarray(f(x, x[13:17])).ravel()
    np.testing.assert_allclose(derivative, 0, atol=1e-8)


@pytest.mark.parametrize('params', [vehicle_params, rocket_params])
def test_virtual_translation_matches_physical_plant(params):
    # 같은 총추력에서 두 모델은 병진 운동과 쿼터니언 미분이 같아야 한다.
    p = copy.deepcopy(params)
    x = AxialDronePlant.hover_state(p)
    x[3:6] = [25, 2, 8]
    x[10:13] = [0.1, 0.2, -0.3]
    x[13:17] = [800, 850, 780, 900]
    from scipy.spatial.transform import Rotation
    velocity_body = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]
    force, _ = _rotor_forces_moments(ca.DM(velocity_body), ca.DM(x[13:17]), ca.DM(x[10:13]), p)
    total = np.linalg.norm(np.asarray(ca.DM(force)))
    actual, _, _ = build_dynamics(p)
    virtual, _, _ = build_virtual_dynamics(p)
    np.testing.assert_allclose(np.asarray(virtual(x[:13], [total, 0, 0, 0])).ravel()[:10],
                               np.asarray(actual(x, x[13:17])).ravel()[:10], atol=1e-8)


@pytest.mark.parametrize('params', [vehicle_params, rocket_params])
@pytest.mark.parametrize('inflow,rotors', [(0, [0, 800, 900, 1000]), (65, [1, 200, 900, 1200])])
def test_effectiveness_matches_rotor_wrench_derivative(params, inflow, rotors):
    p = copy.deepcopy(params)
    velocity = np.array([inflow, 0, 0]) if p.get('thrust_axis') == 'x' else np.array([0, 0, -inflow])
    n = ca.SX.sym('n', 4)
    force, moment = _rotor_forces_moments(ca.DM(velocity), n, ca.DM.zeros(3), p)
    total = force[0] if p.get('thrust_axis') == 'x' else -force[2]
    inertia = ca.DM([p['Ixx'], p['Iyy'], p['Izz']])
    wrench = ca.vertcat(total, moment / inertia)
    derivative = ca.Function('effectiveness', [n], [ca.jacobian(wrench, n)])
    expected = np.asarray(derivative(rotors))
    actual = compute_control_effectiveness(p, np.asarray(rotors, dtype=float), velocity)
    np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=1e-8)


# ── find_trim 축 지원 (2026-09-24) ─────────────────────────────────────
# trim.py 의 build_trim_state 가 z 축 호버 자세를 하드코딩하고 있어서
# thrust_axis='x' 에서는 트림이 잡히지 않았다. 축을 갈랐고, 그게 계속
# 유지되는지 여기서 지킨다.

@pytest.mark.parametrize('params', [vehicle_params, rocket_params])
def test_hover_trim_residual_is_zero_on_both_axes(params):
    from control.trim import find_trim
    trim = find_trim(params, 0.0, quiet=True)
    assert trim['converged'], trim['why']
    # 잔차는 [v̇_x, v̇_z, ω̇_y] 세 항 전부다.
    assert trim['residual'] < 1e-6
    # 호버는 로터가 균일해야 한다(차동이 붙으면 자세가 기운다).
    np.testing.assert_allclose(trim['control'], trim['control'][0], rtol=1e-9)


@pytest.mark.parametrize('params', [vehicle_params, rocket_params])
def test_cruise_trim_residual_is_zero_on_both_axes(params):
    from control.trim import find_trim
    trim = find_trim(params, 20.0, quiet=True)
    assert trim['converged'], trim['why']
    assert trim['residual'] < 1e-6


def test_x_axis_angle_of_attack_is_measured_from_the_thrust_axis():
    """x 축에 z 축 받음각 식을 쓰면 호버에서 -90° 가 나와 오독된다."""
    from control.trim import find_trim
    # 정지에서는 받음각이 정의되지 않는다 — 숫자를 지어내면 안 된다.
    hover = find_trim(rocket_params, 0.0, quiet=True)
    assert hover['alpha'] is None
    forward = find_trim(rocket_params, 20.0, quiet=True)
    # 호버(추력축 수직)에서 눕히는 중이므로 받음각은 90°에서 줄어든다.
    assert 0 < np.degrees(forward['alpha']) < 90


def test_selected_profile_trims_at_low_speed_and_correctly_fails_at_high_speed():
    """확정 설계는 20 m/s 위에 트림이 **없는 것이 정답**이다.

    research/TRIM_CAUSE_AND_LEVERS.md: 요구 피치모멘트 |cp|·W·cos θ 가 가용
    a·T 를 넘어 한쪽 로터쌍이 음추력을 요구한다(경계 19.63 m/s). 여기서
    고속이 통과하기 시작하면 계수가 물리에서 벗어났다는 신호다.
    """
    if selected_params is None:
        pytest.skip('research/profiles/selected.json 없음 — control/ 단독 실행')
    from control.trim import find_trim
    low = find_trim(selected_params, 10.0, quiet=True)
    assert low['converged'], low['why']
    assert low['residual'] < 1e-5
    high = find_trim(selected_params, 30.0, quiet=True)
    assert not high['converged']


def test_selected_profile_matches_the_research_trim_attitude():
    """같은 기체를 다른 코드베이스로 풀면 같은 자세가 나와야 한다.

    받음각은 법선력 평형(CN_alpha·CN_cross)만으로 정해지고 C_A0 근사는 추력
    크기에만 들어가므로, 이 값은 근사와 무관하게 일치해야 한다.
    """
    if selected_params is None:
        pytest.skip('research/profiles/selected.json 없음 — control/ 단독 실행')
    from control.trim import find_trim
    # research/trim_envelope.py 가 같은 프로파일에서 낸 값(문서화된 표).
    expected = {5.0: 86.849, 10.0: 74.238, 15.0: 51.061}
    guess = None
    for speed, reference in expected.items():
        trim = find_trim(selected_params, speed, guess=guess, quiet=True)
        assert trim['converged'], f'V={speed}: {trim["why"]}'
        guess = trim['guess']
        assert abs(np.degrees(trim['alpha']) - reference) < 0.01, \
            f'V={speed}: alpha {np.degrees(trim["alpha"]):.3f} != research {reference}'
