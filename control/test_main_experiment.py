"""본 실험 캠페인(`control/main_experiment.py`, `configs/main_experiment.json`) — kj 결정(2026-09-28) 시험.

  1) arena.json은 바뀌지 않는다(파일 바이트·정규화 해시가 spec 기록과 같다).
  2) 가드: 튜닝 해시가 비면 거부, 기준 해시가 다르면 거부, 기록 파일 sha256이 맞으면 통과. --run은 가드에서 멈춘다.
  3) 묶음 구성: 참조 15, 표7 기준 5, 표7 17×5, 돌풍 8, 임무 V_H 1, 사다리(V13×nominal 중복 제외).
  4) 규칙 → 숫자: 0.7g·1.4g × 명목 질량, 호버 최대 피치 모멘트 × 25%·50%, 면내 항력 계수 = V_H 보정값.
  5) 사다리 파생 설정은 controllers.V13의 세 항목만 다르다.
  6) 기체 한계: 트림이 없으면 vehicle_limit, CG 행은 6자유도 트림으로 판정.
  7) 참조 묶음 ρ는 목표의 0.3% 안(ramp_round_s=dt), 표7 ρ=1.0 기준은 평가 창이 참조 묶음과 다르다(같은 표에 두지 않는 근거).
"""
from copy import deepcopy
import hashlib
import json

import numpy as np
import pytest

from control import main_experiment as me
from control.arena import ROOT, load_config, config_sha256, build_scenarios
from control.arena_factory import ArenaFactory
from control.validation_suite import baseline_params


@pytest.fixture(scope='module')
def spec():
    return me.load_spec()


@pytest.fixture(scope='module')
def base():
    return load_config()


@pytest.fixture(scope='module')
def native():
    return baseline_params()


@pytest.fixture(scope='module')
def batches(spec, base, native):
    return {b.name: b for b in me.build_batches(spec, base, native)}


def test_arena_json_is_untouched_by_the_main_experiment(spec, base, native):
    path = ROOT/'configs'/'arena.json'
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    me.build_batches(spec, base, native)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    assert config_sha256(load_config()) == spec['base_config']['config_sha256']


def test_guard_refuses_while_tuning_hashes_are_empty(spec):
    problems = me.guard_problems(spec)
    assert any('run_dir is empty' in p for p in problems)
    assert sum('sha256 is empty' in p for p in problems) == len(spec['controllers'])


def test_guard_refuses_a_wrong_base_hash(spec):
    bad = deepcopy(spec)
    bad['base_config']['config_sha256'] = '0'*64
    assert any('base config hash' in p for p in me.guard_problems(bad))


def test_guard_passes_matching_record_hashes_and_catches_a_changed_file(spec, tmp_path):
    run_dir = tmp_path/'final'
    run_dir.mkdir()
    good = deepcopy(spec)
    good['tuned']['run_dir'] = str(run_dir)          # 절대경로 — root/절대경로 = 절대경로
    for label in spec['controllers']:
        (run_dir/f'{label}.record.json').write_text(json.dumps({'controller': label}), encoding='utf-8')
        good['tuned']['records'][label]['sha256'] = hashlib.sha256(
            (run_dir/f'{label}.record.json').read_bytes()).hexdigest()
    assert me.guard_problems(good) == []
    (run_dir/'V13.record.json').write_text('{"controller": "V13", "edited": true}', encoding='utf-8')
    assert any(p.startswith('V13: record sha256') for p in me.guard_problems(good))


def test_run_stops_at_the_guard_before_any_trial(spec, tmp_path, monkeypatch):
    import control.validation_suite as suite
    monkeypatch.setattr(suite, 'run_trial', lambda *a, **k: pytest.fail('a trial ran despite the guard'))
    with pytest.raises(SystemExit, match='refusing to run'):
        me.run(spec, 'reference', tmp_path)
    assert not any(tmp_path.iterdir())


def test_batch_composition(batches, spec):
    count = {name: len(b.config['scenarios']) for name, b in batches.items()}
    assert count['reference'] == 15
    assert count['table7_base'] == 5
    assert count['table7'] == 17*5
    assert count['gust'] == 8
    assert count['mission'] == 1 and batches['mission'].config['scenarios'][0]['speed'] == 'V_H'
    assert count['ladder:V13-0'] == count['ladder:V13-1'] == count['ladder:V13-2'] == 15*4
    assert count['ladder:V13'] == 15*3                   # nominal은 참조 묶음 결과를 쓴다
    ids = [s['id'] for b in batches.values() for s in b.config['scenarios']]
    assert len(ids) == len(set(ids))
    assert not any('brake_VL' in i for i in ids)          # 제동@V_L 제외(kj 결정)
    assert all(b.controllers == ['V13'] for n, b in batches.items() if n.startswith('ladder'))


def test_rules_become_the_agreed_numbers(spec, base, native):
    from control.arena_plant_wrench import reference_inplane_drag_coefficient, hover_max_pitch_moment
    rows = {r['name']: r for r in spec['table7']['rows']}
    _, force = me.row_values(rows['force_0.7g'], spec, native, base)
    np.testing.assert_allclose(force['extra_force_world'], [0.0, 0.7*native['mass']*native['g'], 0.0])
    assert abs(force['extra_force_world'][1] - 8.062) < 1e-3 and force['extra_force_start_s'] == 3.0
    _, moment = me.row_values(rows['moment_50'], spec, native, base)
    np.testing.assert_allclose(moment['extra_moment_body'], [0.0, 0.5*hover_max_pitch_moment(native), 0.0])
    assert abs(moment['extra_moment_body'][1] - 0.468) < 1e-3 and moment['extra_moment_duration_s'] == 5.0
    _, drag = me.row_values(rows['inplane_drag_1x'], spec, native, base)
    assert drag['rotor_inplane_drag_coeff'] == reference_inplane_drag_coefficient(native, 85.0)
    assert len(spec['table7']['rows']) == 17


def test_ladder_variants_change_only_the_three_v13_flags(batches, base):
    for name, b in batches.items():
        if not name.startswith('ladder'):
            continue
        config = deepcopy(b.config)
        for k in ('alloc_mode', 'alloc_feedback', 'time_align'):
            config['controllers']['V13'][k] = base['controllers']['V13'][k]
        config['scenarios'] = base['scenarios']
        assert config == base, name


def test_ladder_v13_flags_match_the_agreed_mapping(batches):
    got = {b.variant: b.config['controllers']['V13'] for b in batches.values() if b.variant}
    assert (got['V13-0']['alloc_mode'], got['V13-0']['alloc_feedback'], got['V13-0']['time_align']) == ('A0', False, 'S0')
    assert (got['V13-1']['alloc_mode'], got['V13-1']['alloc_feedback'], got['V13-1']['time_align']) == ('A1', False, 'S0')
    assert (got['V13-2']['alloc_mode'], got['V13-2']['alloc_feedback'], got['V13-2']['time_align']) == ('A1', True, 'S0')
    assert (got['V13']['alloc_mode'], got['V13']['alloc_feedback'], got['V13']['time_align']) == ('A1', True, 'S1')


def test_missing_trim_is_a_vehicle_limit_not_a_controller_failure(base, native):
    config = deepcopy(base)
    config['scenarios'] = [dict(id='weak', type='cruise', speed='V_H', times_s=[3.0, 5.0, 7.0],
                                perturbation={'thrust': 0.1})]
    scenario = build_scenarios(config, ArenaFactory(base, native).cp, native)[0]
    status, detail = me.trim_status(native, scenario)
    assert status == 'vehicle_limit' and not detail[85.0]['valid']


def test_cg_rows_are_judged_with_the_6dof_trim(batches, native, base):
    cp = ArenaFactory(base, native).cp
    config = deepcopy(batches['table7'].config)
    config['scenarios'] = [s for s in config['scenarios'] if s['id'] == 'main_t7_cruise_VL__cg_y_0.1']
    scenario = build_scenarios(config, cp, native)[0]
    status, detail = me.trim_status(native, scenario)
    assert status is None and detail[20.0]['method'] == '6dof' and detail[20.0]['condition'] == 'roll0'


def test_reference_rho_is_on_target_and_table7_windows_differ(batches, base, native):
    cp = ArenaFactory(base, native).cp
    ref = {s.id: s for s in build_scenarios(batches['reference'].config, cp, native)}
    for s in ref.values():
        assert abs(s.meta['rho_actual']/s.meta['rho_target'] - 1) < 3e-3, s.id
    t7 = {s.id: s for s in build_scenarios(batches['table7_base'].config, cp, native)}
    short = t7['main_t7_accel_VL_rho1'], ref['main_ref_accel_VL_rho1']
    assert short[0].window != short[1].window and short[0].window[1] >= 15.0
    assert short[0].profile.T_total >= 15.0 and short[1].profile.T_total < 15.0


@pytest.mark.parametrize('scenario, message', [
    (dict(id='x', type='cruise', speed='V_L', times_s=[3.0, 5.0, 7.0], ramp_round_s=0.002), 'reference-only'),
    (dict(id='x', type='reference', **{'from': 0.0}, to='V_L', rho=1.0, lead_s=3.0, tail_s=5.0,
          ramp_round_s=0.003), 'reference-only'),
    (dict(id='x', type='cruise', speed='V_L', times_s=[3.0, 5.0, 7.0], evaluation_start_s=20.0), 'inside'),
])
def test_config_rejects_misused_new_scenario_keys(base, scenario, message):
    from control.arena import validate_config
    config = deepcopy(base)
    config['scenarios'] = [scenario]
    with pytest.raises(ValueError, match=message):
        validate_config(config)
