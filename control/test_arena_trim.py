"""경기장 6자유도 트림(`control/arena_trim.py`) — kj 결정(2026-09-28)의 시험 항목.

  1) 명목 기체에서 벤더 find_trim과 같은 해(20·85 m/s는 비트 동일, 호버는 θ가 경계 π/2에 붙어 1e-9 안).
  2) 벤더가 실패하는 CG 편차(y: 전 속도, z: 호버)에서 트림을 잡는다 — 전제(벤더 실패)도 같이 확인한다.
  3) 조건 1(롤 0)이 없으면 조건 2(옆미끄럼 0)로 넘어가고, 그 해는 옆미끄럼이 0이다.
  4) 둘 다 없으면 'vehicle limit' — 제어기 실패와 따로 분류할 근거.
  5) run_trial은 CG 키가 있을 때만 6자유도 트림을 쓴다(기본 경로는 벤더 그대로).
"""
import numpy as np
import pytest

from control import arena_trim
from control.arena_trim import find_trim_6dof, CONDITIONS
from control.uncertainty import cg_offset_arm_fraction, perturb_params
from models.team_light.control.trim import find_trim
from models.team_light.control.vehicle_params import vehicle_params


@pytest.fixture
def params():
    return dict(vehicle_params)


@pytest.mark.parametrize('V', [20.0, 85.0])
def test_nominal_cruise_matches_vendor_trim_bit_for_bit(params, V):
    vendor, ours = find_trim(params, V), find_trim_6dof(params, V)
    assert ours['condition'] == 'roll0'
    assert np.array_equal(ours['state'], vendor['state'])


def test_nominal_hover_matches_vendor_analytic_hover(params):
    """호버는 벤더가 해석해(θ=π/2 정확히)를 주고, 여기선 θ 상한 경계에서 멈춰 1e-11 차이가 난다."""
    vendor, ours = find_trim(params, 0.0), find_trim_6dof(params, 0.0)
    np.testing.assert_allclose(ours['state'], vendor['state'], rtol=0, atol=1e-9)
    assert ours['residual'] < 1e-8


@pytest.mark.parametrize('axis, V', [('y', 0.0), ('y', 20.0), ('y', 85.0), ('z', 0.0)])
def test_cg_offset_trims_that_the_planar_vendor_search_cannot_represent(params, axis, V):
    p = cg_offset_arm_fraction(params, axis, 0.1)
    assert not find_trim(p, V, strict=False)['valid']      # 전제: 벤더는 못 잡는다
    ours = find_trim_6dof(p, V)
    assert ours['valid'] and ours['condition'] in CONDITIONS
    assert ours['residual'] < 1e-8
    assert np.all(ours['control'] <= p['n_max']) and np.all(ours['control'] >= p['n_min'])


def test_cg_y_at_cruise_needs_asymmetric_left_right_rotors(params):
    """벤더가 표현 못 하는 이유 그 자체 — 같은 z쌍 안에서도 회전수가 달라야 한다."""
    p = cg_offset_arm_fraction(params, 'y', 0.1)
    n = find_trim_6dof(p, 20.0)['control']
    positive_z = np.asarray(p['rotor_positions'])[:, 2] > 0
    assert np.ptp(n[positive_z]) > 1.0 and np.ptp(n[~positive_z]) > 1.0


def test_falls_back_to_zero_sideslip_when_zero_roll_fails(params, monkeypatch):
    """조건 1을 강제로 실패시키면 조건 2의 해를 돌려주고, 그 해는 옆미끄럼 0이다."""
    real_solve = arena_trim._solve

    def roll0_fails(p, V, condition, guess):
        result = real_solve(p, V, condition, guess)
        if condition == 'roll0':
            result['valid'] = False
        return result

    monkeypatch.setattr(arena_trim, '_solve', roll0_fails)
    p = cg_offset_arm_fraction(params, 'z', 0.1)
    ours = find_trim_6dof(p, 20.0)
    assert ours['condition'] == 'sideslip0'
    assert [a['condition'] for a in ours['attempts']] == ['roll0', 'sideslip0']
    assert abs(ours['sideslip']) < 1e-9


def test_sideslip0_on_nominal_is_the_vendor_solution(params):
    vendor = find_trim(params, 85.0)
    guess = np.r_[vendor['theta'], 0.0, vendor['control']]
    ours = arena_trim._solve(params, 85.0, 'sideslip0', guess)
    assert ours['valid']
    np.testing.assert_allclose(ours['state'], vendor['state'], rtol=0, atol=1e-9)


def test_no_trim_under_either_condition_is_a_vehicle_limit(params):
    # 호버 최대추력/무게 = 7.04 → 추력 계수 0.1배면 0.70이라 트림이 있을 수 없다. (처음엔 질량 3배
    # 85 m/s로 짰는데 트림이 있었다 — 받음각 18°의 동체 양력이 무게의 약 40%를 받쳐, 벤더도 같은 해를 낸다.)
    weak = perturb_params(params, {'thrust': 0.1})
    with pytest.raises(ValueError, match='vehicle limit'):
        find_trim_6dof(weak, 0.0)
    result = find_trim_6dof(weak, 0.0, strict=False)
    assert result['condition'] is None and not result['valid']
    assert [a['condition'] for a in result['attempts']] == list(CONDITIONS)


class _Stop(Exception):
    pass


@pytest.mark.parametrize('extra, expected', [
    ({}, 'vendor'),
    (dict(cg_offset_axis='y', cg_offset_arm_fraction=0.1), '6dof'),
])
def test_run_trial_uses_6dof_trim_only_for_cg_cases(monkeypatch, params, extra, expected):
    from control import validation_suite as suite
    from control.mission_profiles import MissionProfile
    called = []
    monkeypatch.setattr(suite, 'find_trim', lambda *a, **k: called.append('vendor') or (_ for _ in ()).throw(_Stop))
    monkeypatch.setattr(suite, 'find_trim_6dof', lambda *a, **k: called.append('6dof') or (_ for _ in ()).throw(_Stop))

    class Factory:
        p, dt = params, 0.002

    with pytest.raises(_Stop):
        suite.run_trial(Factory(), 'X', MissionProfile(20.0, 20.0, (1.0,)*5),
                        dict(case_id='t', factors={}, extra_params=extra), None)
    assert called == [expected]
