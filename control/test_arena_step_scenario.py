"""새 시나리오 타입 'step'(설계점검과 같은 StepProfile) — 스키마 검사와 프로필 생성만 본다.

kj 결정(2026-09-27 저녁): 튜닝 집합이 "본시험"만 감싸는 게 아니라 "모든 평가 축(본시험 + 설계점검)"을
감싸야 한다. CPID 고도-계단 정착 실패(순수 z-추종 신호 없음)가 원인이었다 — 이 테스트는 그 신호가
스키마에 제대로 들어왔는지 확인한다. 실제 폐루프 결과(성공·실패)는 여기서 안 본다(느리다) — 그건
`control/test_arena_fairness.py`의 전체 스위트가 본다.
"""
import numpy as np
import pytest

from control.arena import build_scenarios, validate_config, DEFAULT_CONFIG, load_config
from control.mission_profiles import StepProfile


@pytest.fixture(scope='module')
def config():
    return load_config(DEFAULT_CONFIG)


def _minimal_config(config, scenario):
    """config를 복제하지 않고 tuning.scenarios만 이 시나리오 하나로 바꾼 얕은 뷰."""
    import copy
    cfg = copy.deepcopy(config)
    cfg['tuning']['scenarios'] = [scenario]
    return cfg


def test_step_axis_altitude_builds_a_step_profile_with_the_right_size(config):
    scenario = dict(id='t_step', type='step', speed=14.0, axis='altitude', size=0.7,
                    durations_s=[8.0])
    (s,) = build_scenarios(config, scenarios=[scenario])
    assert isinstance(s.profile, StepProfile)
    assert s.profile.V == 14.0 and s.profile.dz == 0.7 and s.profile.dv == 0.0
    assert s.profile.T_total == 8.0 and s.profile.ramp_s == 1.0
    assert s.meta['axis'] == 'altitude' and s.meta['size'] == 0.7
    assert s.window == (0.0, 8.0)


def test_step_axis_speed_builds_a_step_profile_with_the_right_size(config):
    scenario = dict(id='t_step', type='step', speed=0.0, axis='speed', size=-1.3,
                    durations_s=[8.0])
    (s,) = build_scenarios(config, scenarios=[scenario])
    assert s.profile.V == 0.0 and s.profile.dv == -1.3 and s.profile.dz == 0.0


def test_step_reference_before_and_after_the_transition(config):
    """StepProfile 자체(스위트 규약)를 다시 확인 — get_ref가 계단 전/후 트림·목표를 낸다."""
    scenario = dict(id='t_step', type='step', speed=14.0, axis='altitude', size=0.7,
                    durations_s=[8.0])
    (s,) = build_scenarios(config, scenarios=[scenario])
    v_before, z_before, name_before = s.profile.get_ref(0.1)
    v_after, z_after, name_after = s.profile.get_ref(7.9)
    assert name_before == '트림' and z_before == pytest.approx(20.0)   # 경기장 기본 고도
    assert name_after == '계단' and z_after == pytest.approx(20.7)
    np.testing.assert_array_equal(v_before, [14.0, 0.0, 0.0])
    np.testing.assert_array_equal(v_after, [14.0, 0.0, 0.0])           # 고도 계단은 속도를 안 건드린다


@pytest.mark.parametrize('bad', [
    dict(id='t', type='step', speed=0.0, axis='lateral', size=1.0, durations_s=[8.0]),   # axis 오타
])
def test_validate_config_rejects_unknown_step_axis(config, bad):
    cfg = _minimal_config(config, bad)
    with pytest.raises(ValueError, match='axis'):
        validate_config(cfg)


def test_validate_config_accepts_the_new_tune3_scenarios_already_in_the_config(config):
    """configs/arena.json에 실제로 커밋된 tune3_step_* 8개가 스키마를 통과하는지."""
    ids = [s['id'] for s in config['tuning']['scenarios'] if s['type'] == 'step']
    assert len(ids) == 8 and all(i.startswith('tune3_step_') for i in ids)
    validate_config(config)   # 이미 load_config에서 통과했지만, 실패 시 원인을 이 테스트가 바로 가리키게


def test_step_scenario_ids_stay_disjoint_from_main_test_and_tune2(config):
    step_ids = {s['id'] for s in config['tuning']['scenarios'] if s['type'] == 'step'}
    other_tuning_ids = {s['id'] for s in config['tuning']['scenarios'] if s['type'] != 'step'}
    main_ids = {s['id'] for s in config['scenarios']}
    assert not (step_ids & other_tuning_ids) and not (step_ids & main_ids)
