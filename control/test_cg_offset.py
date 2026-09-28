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


# ── 경기장 배선(2026-09-28 d2): extra_params의 cg_offset_axis·cg_offset_arm_fraction ──

class _StopAtPlant(Exception):
    pass


class _FakeFactory:
    """run_trial이 플랜트를 짓기 직전까지만 필요한 최소 factory — 제어기는 만들지 않는다."""
    def __init__(self, p):
        self.p = p
        self.dt = 0.002

    def make(self, label, v, z):
        return object()

    def solver_of(self, ctrl):
        return None


def _truth_seen_by_plant(monkeypatch, extra_params):
    """run_trial이 build_plant에 넘기는 truth를 가로채 돌려준다(거기서 시행을 멈춘다)."""
    from control import validation_suite as suite
    from control.mission_profiles import MissionProfile
    seen = {}

    def spy(truth, dt):
        seen['truth'] = truth
        raise _StopAtPlant

    monkeypatch.setattr(suite, 'build_plant', spy)
    # 이 시험은 truth 배선만 본다 — 트림 선택(CG면 6자유도, 아니면 벤더)은 test_arena_trim.py가
    # 본다. 트림 탐색과 섞이지 않게 둘 다 명목 트림으로 대신한다.
    from models.team_light.control.trim import find_trim as vendor_trim
    nominal_trim = lambda truth, V: vendor_trim(dict(vehicle_params), V)
    monkeypatch.setattr(suite, 'find_trim', nominal_trim)
    monkeypatch.setattr(suite, 'find_trim_6dof', nominal_trim)
    factory = _FakeFactory(dict(vehicle_params))
    case = dict(case_id='cg', factors={}, extra_params=extra_params)
    with pytest.raises(_StopAtPlant):
        suite.run_trial(factory, 'X', MissionProfile(20.0, 20.0, (1.0, 1.0, 1.0, 1.0, 1.0)), case, None)
    return seen['truth'], factory


@pytest.mark.parametrize('axis', ['y', 'z'])
def test_run_trial_applies_cg_offset_to_the_plant_only(monkeypatch, axis):
    truth, factory = _truth_seen_by_plant(
        monkeypatch, dict(cg_offset_axis=axis, cg_offset_arm_fraction=0.1))
    expected = cg_offset_arm_fraction(vehicle_params, axis, 0.1)
    np.testing.assert_array_equal(truth['rotor_positions'], expected['rotor_positions'])
    np.testing.assert_array_equal(truth['nose_cp'], expected['nose_cp'])
    moved = np.asarray(truth['rotor_positions']) - np.asarray(vehicle_params['rotor_positions'])
    assert np.all(moved[:, CG_OFFSET_AXES[axis]] < 0)          # +편차 → 위치벡터는 −방향으로
    # 제어기가 받는 명목값은 그대로다(run_trial 자신도 이걸 검사한다 — 여기선 위치까지 직접 본다)
    np.testing.assert_array_equal(factory.p['rotor_positions'], vehicle_params['rotor_positions'])


def test_run_trial_without_cg_keys_matches_the_old_truth(monkeypatch):
    """키가 없으면 무게중심 함수를 안 부른다 — truth가 옛 경로(perturb_params + update)와 같다."""
    from control.uncertainty import perturb_params
    from control.validation_suite import parameter_hash
    truth, _ = _truth_seen_by_plant(monkeypatch, {})
    old = perturb_params(dict(vehicle_params), {})
    assert parameter_hash(truth) == parameter_hash(old)


@pytest.mark.parametrize('extra, message', [
    (dict(cg_offset_axis='y'), 'give both'),
    (dict(cg_offset_axis='x', cg_offset_arm_fraction=0.1), 'cg_offset_axis'),
    (dict(extra_force_start_s=3.001), 'multiple'),
    (dict(extra_moment_duration_s=-0.002), 'multiple'),
])
def test_config_rejects_bad_cg_and_window_keys(extra, message):
    from copy import deepcopy
    from control.arena import load_config, validate_config
    config = deepcopy(load_config())
    config['scenarios'][0]['extra_params'] = extra
    with pytest.raises(ValueError, match=message):
        validate_config(config)


def test_config_accepts_the_table7_keys():
    from copy import deepcopy
    from control.arena import load_config, validate_config
    config = deepcopy(load_config())
    config['scenarios'][0]['extra_params'] = dict(
        cg_offset_axis='z', cg_offset_arm_fraction=0.1,
        extra_force_world=[0.0, 8.06, 0.0], extra_force_start_s=3.0, extra_force_duration_s=5.0,
        extra_moment_body=[0.0, 0.234, 0.0], extra_moment_start_s=3.0, extra_moment_duration_s=5.0)
    validate_config(config)
