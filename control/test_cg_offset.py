"""CG 편차(`control/uncertainty.py::cg_offset_params`) — kj 결정(2026-09-28)의 시험 항목 그대로.

  1) 편차 0 = 기존 플랜트와 완전히 같다(파라미터 dict 자체가 nominal과 값까지 같다, 궤적도 비트 동일).
  2) 편차를 주면 호버에서 추력이 만드는 모멘트가 해석값(편차 × 추력)과 맞는다.
"""
import numpy as np
import pytest

from control.uncertainty import cg_offset_params, cg_offset_arm_fraction, CG_OFFSET_AXES
from models.team_light.control.dynamics import AxialDronePlant
from models.team_light.control.vehicle_params import vehicle_params


@pytest.fixture
def hover_state():
    return AxialDronePlant.hover_state(vehicle_params)


def test_zero_offset_gives_the_same_parameters(hover_state):
    offset_params = cg_offset_params(vehicle_params, [0.0, 0.0, 0.0])
    for key in ('nose_cp', 'rotor_positions'):
        np.testing.assert_array_equal(offset_params[key], vehicle_params[key])
    for a, b in zip(offset_params['body_strips'], vehicle_params['body_strips']):
        np.testing.assert_array_equal(a['position'], b['position'])
    for a, b in zip(offset_params['drag_elements'], vehicle_params['drag_elements']):
        np.testing.assert_array_equal(a['position'], b['position'])


def test_zero_offset_gives_a_bit_identical_trajectory(hover_state):
    plant_nominal = AxialDronePlant(vehicle_params, dt=0.002)
    plant_offset = AxialDronePlant(cg_offset_params(vehicle_params, [0.0, 0.0, 0.0]), dt=0.002)
    u = hover_state[13:17]
    x_nominal = plant_nominal.step(hover_state.copy(), u.copy())
    x_offset = plant_offset.step(hover_state.copy(), u.copy())
    np.testing.assert_array_equal(x_nominal, x_offset)


@pytest.mark.parametrize('axis, sign', [('y', +1.0), ('z', +1.0)])
def test_offset_moment_at_hover_matches_the_analytic_value(hover_state, axis, sign):
    """호버(추력 ≈ 무게)에서 대칭 배치 로터의 순 모멘트는 0인데, CG를 옮기면 모든 로터 위치가
    반대로 옮겨져 M ≈ -offset × (mass·g·axis_thrust)만큼 생긴다(axis_thrust = body +x).
      y 오프셋 dy → M_z = +mass·g·dy
      z 오프셋 dz → M_y = -mass·g·dz
    """
    arm = float(np.linalg.norm(np.asarray(vehicle_params['rotor_positions'])[0, 1:3]))
    dy_or_dz = sign*0.1*arm
    offset = [0.0, dy_or_dz, 0.0] if axis == 'y' else [0.0, 0.0, dy_or_dz]

    plant_nominal = AxialDronePlant(vehicle_params, dt=0.002)
    plant_offset = AxialDronePlant(cg_offset_params(vehicle_params, offset), dt=0.002)
    u = hover_state[13:17]
    xd_nominal = plant_nominal.evaluate_xdot(hover_state, u)
    xd_offset = plant_offset.evaluate_xdot(hover_state, u)
    dw = xd_offset[10:13] - xd_nominal[10:13]

    mass, g = vehicle_params['mass'], vehicle_params['g']
    J = np.array([vehicle_params['Ixx'], vehicle_params['Iyy'], vehicle_params['Izz']])
    M_analytic = np.zeros(3)
    if axis == 'y':
        M_analytic[2] = mass*g*dy_or_dz
    else:
        M_analytic[1] = -mass*g*dy_or_dz
    # 실측 상대오차는 기계정밀도(~1e-15) 수준이다(호버 트림이 추력=무게로 거의 완벽히 맞아서) —
    # rtol은 그보다 훨씬 느슨하게 둬서 트림 정밀도가 바뀌어도 시험이 안 부서지게 한다.
    np.testing.assert_allclose(dw, M_analytic/J, rtol=1e-3)


def test_cg_offset_arm_fraction_matches_direct_call(hover_state):
    arm = float(np.linalg.norm(np.asarray(vehicle_params['rotor_positions'])[0, 1:3]))
    via_helper = cg_offset_arm_fraction(vehicle_params, 'y', fraction=0.1)
    via_direct = cg_offset_params(vehicle_params, [0.0, 0.1*arm, 0.0])
    np.testing.assert_array_equal(via_helper['rotor_positions'], via_direct['rotor_positions'])


def test_unknown_axis_is_rejected():
    with pytest.raises(ValueError, match='axis'):
        cg_offset_arm_fraction(vehicle_params, 'x')


def test_offset_shifts_every_cg_referenced_position_consistently():
    """네 그룹(코·body_strips·drag_elements·rotor_positions) 전부 같은 벡터만큼 옮겨야 한다."""
    offset = [0.0, 0.01, 0.0]
    p = cg_offset_params(vehicle_params, offset)
    np.testing.assert_allclose(np.asarray(p['nose_cp']) - np.asarray(vehicle_params['nose_cp']),
                               [-o for o in offset])
    np.testing.assert_allclose(np.asarray(p['rotor_positions']) - np.asarray(vehicle_params['rotor_positions']),
                               np.tile([-o for o in offset], (4, 1)))
    for a, b in zip(p['body_strips'], vehicle_params['body_strips']):
        np.testing.assert_allclose(np.asarray(a['position']) - np.asarray(b['position']), [-o for o in offset])
    for a, b in zip(p['drag_elements'], vehicle_params['drag_elements']):
        np.testing.assert_allclose(np.asarray(a['position']) - np.asarray(b['position']), [-o for o in offset])
