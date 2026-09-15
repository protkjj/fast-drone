"""연구용 경로가 웹의 body +x 기하 및 실제 플랜트 미분과 일치하는지 검사."""
import copy

import casadi as ca
import numpy as np
import pytest

from control.dynamics import AxialDronePlant, build_dynamics, _rotor_forces_moments
from control.hybrid_comparison import build_virtual_dynamics, compute_control_effectiveness
from control.vehicle_params import rocket_params, vehicle_params


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
