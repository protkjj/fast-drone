"""control/tuning_analysis.py 단위 테스트 — 합성 기록만 쓴다(시뮬레이션 없음).

합성 기록은 arena_tune.py가 쓰는 형식(<제어기>.record.json + <제어기>.jsonl)을
그대로 흉내 낸다. 기댓값은 코드 출력이 아니라 손으로 계산한 값이다.

실행: python3 -m pytest control/test_tuning_analysis.py -q
"""
import json

import numpy as np
import pytest

from control.arena import load_config, DEFAULT_CONFIG
from control.arena_factory import ARENA_LABELS, NMPC_LABELS
import control.tuning_analysis as ta


@pytest.fixture(scope='module')
def config():
    return load_config(DEFAULT_CONFIG)


def _names(config, label):
    return config['tuning']['parameters']['NMPC' if label in NMPC_LABELS else label]


def write_run(run_dir, config, label, objectives, *, budget=None, failed=None, exponents=None,
              complete=True, cache_hits=()):
    """arena_tune 형식의 합성 기록 한 벌을 쓴다.

    objectives[k] = k번째 평가의 목적함수. failed[k] = 그 평가에서 실패한 시나리오가 있는가.
    exponents[k]를 안 주면 평가마다 다른 좌표(첫 좌표만 k)를 쓴다 — 0번은 항상 사전값(전부 0).
    cache_hits에 든 k는 scenarios=None(캐시 적중)으로 기록한다.
    """
    names = _names(config, label)
    n = len(objectives)
    budget = n if budget is None else budget
    failed = failed or [False]*n
    if exponents is None:
        exponents = [[float(k)] + [0.0]*(len(names) - 1) for k in range(n)]
    scenario_ids = [s['id'] for s in config['tuning']['scenarios']]
    prior = {name: 1.0 for name in names}
    with (run_dir/f'{label}.jsonl').open('w', encoding='utf-8') as stream:
        for k, (obj, e) in enumerate(zip(objectives, exponents)):
            scenarios = None if k in cache_hits else [
                dict(id=sid, failed=(failed[k] and j == 0), score=obj) for j, sid in enumerate(scenario_ids)]
            entry = dict(index=k, exponents=e, values={nm: 2.0**x for nm, x in zip(names, e)},
                         objective=obj, cache_hit=k in cache_hits, scenarios=scenarios)
            stream.write(json.dumps(entry) + '\n')
    best = int(np.argmin(objectives))
    record = dict(label='PILOT', controller=label, budget=budget, spent=n, parameter_names=names,
                  prior=prior, scenario_ids=scenario_ids,
                  main_scenario_ids=[s['id'] for s in config['scenarios']],
                  objective=config['tuning']['objective'], search=config['tuning']['search'],
                  seeds=dict(tuning_range=config['seeds']['tuning'], main_range=config['seeds']['main'],
                             used=[]),
                  status='complete' if complete else 'running')
    if complete:
        record.update(prior_objective=objectives[0], best_objective=objectives[best],
                      best_exponents=exponents[best],
                      best_values={nm: 2.0**x for nm, x in zip(names, exponents[best])})
    (run_dir/f'{label}.record.json').write_text(json.dumps(record), encoding='utf-8')


def write_all(run_dir, config, n=6, **overrides):
    """5종 모두 평가 n회·완료. overrides[label] = write_run 키워드."""
    for label in ARENA_LABELS:
        kwargs = dict(objectives=[10.0 - k for k in range(n)])
        kwargs.update(overrides.get(label, {}))
        write_run(run_dir, config, label, **kwargs)


# ── 곡선 계산 ───────────────────────────────────────────────────────

def test_learning_curve_is_running_minimum():
    assert ta.learning_curve([5, 7, 3, 4, 1, 2]) == [5, 5, 3, 3, 1, 1]


def test_first_reach_counts_from_one_and_returns_none_if_never():
    curve = [5, 5, 3, 3, 1, 1]
    assert ta.first_reach(curve, 5) == 1
    assert ta.first_reach(curve, 3.5) == 3
    assert ta.first_reach(curve, 0.5) is None
    assert ta.first_reach(curve, 5, start=2) == 3      # start 이전은 보지 않는다


def test_three_90pct_definitions_match_hand_calculation(config):
    # 앞의 두 평가 실패(벌점 1000). 손계산:
    #   A: 1000 − 0.9·(1000 − 2) = 101.8 → 3회째(10)
    #   B: 2/0.9 = 2.22            → 7회째(2)     (2.5는 아직 밖)
    #   C: f₀=10(3회째), 10 − 0.9·8 = 2.8 → 6회째(2.5)
    objectives = [1000.0, 1000.0, 10.0, 8.0, 3.0, 2.5, 2.0]
    failed = [True, True, False, False, False, False, False]
    evaluations = [dict(index=k, exponents=[float(k)], objective=o,
                        scenarios=[dict(id='s', failed=f)]) for k, (o, f) in enumerate(zip(objectives, failed))]
    m = ta.controller_metrics('GSLQR', evaluations, config)
    assert m['first_objective'] == 1000.0 and m['first_failed'] is True
    assert m['first_failed_scenarios'] == ['s']
    assert m['best_objective'] == 2.0 and m['best_at_evaluation'] == 7
    assert m['failed_evaluations'] == 2
    assert m['first_nonfailed_evaluation'] == 3
    assert m['evals_to_90pct_improvement'] == 3
    assert m['evals_to_within_best_over_0p9'] == 7
    assert m['evals_to_90pct_improvement_from_first_nonfailed'] == 6
    assert m['curve'] == [1000, 1000, 10, 8, 3, 2.5, 2]


def test_definition_c_never_counts_before_first_nonfailed(config):
    # 인위적 경우: 실패한 평가(2회째)가 첫 비실패 평가(3회째)보다 목적함수가 작다.
    # C는 첫 비실패부터 센다 → 임계 50 − 0.9·(50 − 1) = 5.9 이지만 3회째 이전은 보지 않는다.
    objectives, failed = [1000.0, 2.0, 50.0, 1.0], [True, True, False, False]
    evaluations = [dict(index=k, exponents=[float(k)], objective=o, scenarios=[dict(id='s', failed=f)])
                   for k, (o, f) in enumerate(zip(objectives, failed))]
    m = ta.controller_metrics('CPID', evaluations, config)
    assert m['first_nonfailed_evaluation'] == 3
    assert m['evals_to_90pct_improvement_from_first_nonfailed'] == 3


def test_no_improvement_reaches_at_first_evaluation(config):
    evaluations = [dict(index=k, exponents=[float(k)], objective=o, scenarios=[dict(id='s', failed=False)])
                   for k, o in enumerate([4.0, 5.0, 6.0])]
    m = ta.controller_metrics('CPID', evaluations, config)
    assert m['evals_to_90pct_improvement'] == 1
    assert m['evals_to_within_best_over_0p9'] == 1
    assert m['evals_to_90pct_improvement_from_first_nonfailed'] == 1


def test_all_failed_leaves_nonfailed_columns_empty(config):
    evaluations = [dict(index=k, exponents=[float(k)], objective=1000.0, scenarios=[dict(id='s', failed=True)])
                   for k in range(3)]
    m = ta.controller_metrics('V13', evaluations, config)
    assert m['first_nonfailed_evaluation'] is None
    assert m['evals_to_90pct_improvement_from_first_nonfailed'] is None


def test_cache_hit_inherits_failure_of_first_real_evaluation():
    evaluations = [dict(index=0, exponents=[0.0], objective=1000.0, scenarios=[dict(id='s', failed=True)]),
                   dict(index=1, exponents=[1.0], objective=3.0, scenarios=[dict(id='s', failed=False)]),
                   dict(index=2, exponents=[0.0], objective=1000.0, scenarios=None)]
    assert ta.evaluation_failures(evaluations) == [True, False, True]


def test_cache_hit_without_earlier_evaluation_is_refused():
    with pytest.raises(ta.AnalysisRefused, match='cache hit'):
        ta.evaluation_failures([dict(index=0, exponents=[0.0], objective=1.0, scenarios=None)])


def test_parameter_and_operating_point_counts_come_from_config(config):
    ev = [dict(index=0, exponents=[0.0], objective=1.0, scenarios=[])]
    for label in ARENA_LABELS:
        m = ta.controller_metrics(label, ev, config)
        assert m['n_parameters'] == len(_names(config, label))
        expected = len(config['controllers']['GSLQR']['V_table_m_s']) if label == 'GSLQR' else None
        assert m['gslqr_operating_points'] == expected


# ── 공정성 게이트(예산) ─────────────────────────────────────────────

def test_equal_complete_budgets_are_accepted(tmp_path, config):
    write_all(tmp_path, config)
    analysis, _ = ta.analyze(tmp_path, config)
    assert analysis['label'] == 'PILOT' and analysis['evaluations_per_controller'] == 6
    assert [c['controller'] for c in analysis['controllers']] == list(ARENA_LABELS)


@pytest.mark.parametrize('truncate', [False, True])
def test_budget_mismatch_is_refused_even_with_truncation(tmp_path, config, truncate):
    write_all(tmp_path, config, GSLQR=dict(objectives=[9.0 - k for k in range(8)]))
    # 'between controllers'는 이 도구 자체의 게이트 문구다(I-4 검사기의 문구와 구분).
    with pytest.raises(ta.AnalysisRefused, match='budgets differ between controllers'):
        ta.analyze(tmp_path, config, truncate_to_common=truncate)


def test_incomplete_run_is_refused_in_strict_mode(tmp_path, config):
    write_all(tmp_path, config, M17=dict(objectives=[9.0, 8.0, 7.0], budget=6, complete=False))
    with pytest.raises(ta.AnalysisRefused, match='I-4'):
        ta.analyze(tmp_path, config)


def test_truncate_to_common_cuts_every_controller_and_labels_partial(tmp_path, config):
    write_all(tmp_path, config, M17=dict(objectives=[9.0, 8.0, 7.0], budget=6, complete=False))
    analysis, runs = ta.analyze(tmp_path, config, truncate_to_common=True)
    assert analysis['label'] == ta.PARTIAL
    assert all(c['evaluations'] == 3 for c in analysis['controllers'])
    v13 = next(c for c in analysis['controllers'] if c['controller'] == 'V13')
    assert v13['best_objective'] == 8.0          # 10, 9, 8 까지만 — 뒤의 7..5는 안 본다
    written = ta.write_outputs(analysis, runs, config, tmp_path/'out')
    assert not (tmp_path/'out'/'sensitivity_plan.json').exists()
    assert (tmp_path/'out'/'learning_curves.png') in written


def test_other_i4_violations_still_refused_when_truncating(tmp_path, config):
    write_all(tmp_path, config, CPID=dict(objectives=[9.0, 8.0], budget=6, complete=False))
    record_path = tmp_path/'V13.record.json'
    record = json.loads(record_path.read_text())
    record['scenario_ids'] = ['mission_VH']                  # 본시험 시나리오로 튜닝
    record_path.write_text(json.dumps(record))
    with pytest.raises(ta.AnalysisRefused, match='main-test'):
        ta.analyze(tmp_path, config, truncate_to_common=True)


def test_spent_not_matching_jsonl_is_refused(tmp_path, config):
    write_all(tmp_path, config)
    with (tmp_path/'F13.jsonl').open('a') as stream:                  # 줄 하나 더(기록 불일치)
        stream.write(json.dumps(dict(index=6, exponents=[9.0] + [0.0]*5, objective=1.0,
                                     cache_hit=False, scenarios=[])) + '\n')
    with pytest.raises(ta.AnalysisRefused, match='spent=6'):
        ta.analyze(tmp_path, config)


def test_index_gap_in_jsonl_is_refused(tmp_path, config):
    write_all(tmp_path, config)
    lines = (tmp_path/'CPID.jsonl').read_text().splitlines()
    (tmp_path/'CPID.jsonl').write_text('\n'.join(lines[:2] + lines[3:]) + '\n')
    with pytest.raises(ta.AnalysisRefused, match='indices'):
        ta.load_run(tmp_path)


def test_record_summary_disagreeing_with_log_is_refused(tmp_path, config):
    write_all(tmp_path, config)
    record_path = tmp_path/'GSLQR.record.json'
    record = json.loads(record_path.read_text())
    record['best_objective'] = 0.1
    record_path.write_text(json.dumps(record))
    with pytest.raises(ta.AnalysisRefused, match='best_objective'):
        ta.analyze(tmp_path, config)


# ── 민감도 실행 계획 ────────────────────────────────────────────────

def test_sensitivity_plan_is_one_at_a_time_half_and_double(tmp_path, config):
    # CPID: 평가 1(=인덱스 1)에서 최선. 그 좌표의 첫 파라미터 +1(×2)을 튜닝 중에 이미 평가함.
    names = _names(config, 'CPID')
    z = [0.0]*len(names)
    exps = [z, [1.0] + z[1:], [2.0] + z[1:], [0.0, 1.0] + z[2:]]
    write_all(tmp_path, config, CPID=dict(objectives=[5.0, 1.0, 4.0, 3.0, 6.0, 7.0],
                                          exponents=exps + [[3.0] + z[1:], [4.0] + z[1:]]))
    analysis, runs = ta.analyze(tmp_path, config)
    plan = ta.sensitivity_plan(runs, config)
    assert plan['status'].startswith('PLAN_ONLY') and plan['scenario_set'] == 'tuning'
    assert set(plan['scenario_ids']).isdisjoint(s['id'] for s in config['scenarios'])
    expected_runs = sum(2*len(_names(config, label)) for label in ARENA_LABELS)
    assert len(plan['runs']) == expected_runs
    cpid = {(r['parameter'], r['factor']): r for r in plan['runs'] if r['controller'] == 'CPID'}
    first = names[0]
    best_value = 2.0**1.0                                   # prior 1.0 × 2^1
    assert cpid[(first, 0.5)]['values'][first] == pytest.approx(best_value*0.5)
    assert cpid[(first, 2.0)]['values'][first] == pytest.approx(best_value*2.0)
    assert cpid[(first, 0.5)]['exponents'][0] == 0.0 and cpid[(first, 2.0)]['exponents'][0] == 2.0
    # 좌표 [0,…]=사전값(목적 5.0), [2,…]=목적 4.0 은 튜닝 로그에 있다 → 참고값으로 붙는다
    assert cpid[(first, 0.5)]['tuning_log_objective'] == 5.0
    assert cpid[(first, 2.0)]['tuning_log_objective'] == 4.0
    assert cpid[(names[1], 0.5)]['tuning_log_objective'] is None
    # 다른 파라미터는 최종값 그대로
    assert cpid[(names[1], 2.0)]['values'][first] == pytest.approx(best_value)


# ── 명령행 ──────────────────────────────────────────────────────────

def test_cli_writes_outputs_and_returns_2_on_refusal(tmp_path, config, capsys):
    write_all(tmp_path, config)
    assert ta.main(['--run-dir', str(tmp_path)]) == 0
    out = tmp_path/'analysis'
    for name in ('summary.csv', 'summary.md', 'analysis.json', 'learning_curves.png',
                 'sensitivity_plan.json'):
        assert (out/name).exists(), name
    header = (out/'summary.csv').read_text().splitlines()[0]
    assert 'to90_A_improve' in header and 'first_nonfailed' in header

    bad = tmp_path/'bad'
    bad.mkdir()
    write_all(bad, config, CPID=dict(objectives=[1.0, 2.0], budget=99, complete=False))
    assert ta.main(['--run-dir', str(bad)]) == 2
    assert 'REFUSED' in capsys.readouterr().out
