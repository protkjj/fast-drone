"""arena_tuning_report — 합성 튜닝 기록으로 재생·상수 벌점·실패 평가 분류를 확인한다(빠름, NLP 없음).

합성 기록은 `arena_tune.tune_controller`와 같은 형식이다: 평가 1회 = 한 줄, 캐시 적중이면 scenarios=None.
사례는 둘이다 — 'out'은 늘 실패(상수 벌점), 'in'은 목적함수 f(e)를 점수로 내고 절벽 조건이면 실패한다.
"""
import numpy as np
import pytest

from control.arena_tune import compass_search
from control.arena_tuning_report import analyse, ReplayMismatch

SEARCH = dict(method='compass_log_multiplier', initial_step=2.0, min_step=1.05)
PENALTY = 1000.0


def synthetic_run(f, n, budget, cliff=lambda e: False, omega=lambda e, failed: 11.0 if failed else 0.5):
    names = [f'p{i}' for i in range(n)]
    prior = {name: 1.0 + i for i, name in enumerate(names)}
    log, cache = [], {}

    def evaluate(e):
        e = tuple(float(x) for x in e)
        hit = e in cache
        failed_in = bool(cliff(np.asarray(e)))
        score_in = PENALTY if failed_in else float(f(np.asarray(e)))
        objective = cache[e] if hit else (PENALTY + score_in)/2
        cache[e] = objective
        scenarios = None if hit else [
            dict(id='out', failed=True, score=PENALTY, max_omega=9.0, stop_reason='divergence', paper_reasons=[]),
            dict(id='in', failed=failed_in, score=score_in, max_omega=float(omega(np.asarray(e), failed_in)),
                 stop_reason=None, paper_reasons=['paper_state_limit'] if failed_in else [])]
        log.append(dict(index=len(log), exponents=list(e),
                        values={k: prior[k]*2.0**x for k, x in zip(names, e)},
                        objective=objective, cache_hit=hit, scenarios=scenarios))
        return objective

    best_e, best, info = compass_search(n, budget, evaluate, SEARCH['initial_step'], SEARCH['min_step'])
    record = dict(controller='SYN', parameter_names=names, prior=prior, search=SEARCH, budget=budget,
                  status='complete', best_exponents=list(best_e), best_objective=best)
    return record, log


def test_monotone_objective_never_reduces_the_step_and_finds_the_cliff():
    # f는 p0이 클수록 좋아지고, p0 지수 3(×8)에서 'in'이 실패한다. 12회면 세 번째 폴 도중에 예산이 끝난다.
    record, log = synthetic_run(lambda e: 2.0**(-e[0]), n=2, budget=12, cliff=lambda e: e[0] >= 3)
    out = analyse(record, log)
    assert out['constant_failures'] == ['out']
    assert out['signal_scenarios'] == ['in']
    assert out['search']['step_reduced'] is False
    assert out['best_exponents'] == [2.0, 0.0]
    assert out['best_index'] == 5 and out['search']['evaluations_after_best'] == 6
    assert [x['evaluation'] for x in out['failures']] == [9]
    move = out['failures'][0]['move']
    assert (move['parameter'], move['factor'], move['center_value'], move['value']) == ('p0', 2.0, 4.0, 8.0)
    assert out['failures'][0]['failed'][0]['id'] == 'in'
    assert out['signal_mean'] == dict(prior=1.0, best=0.25)
    assert out['best_signal_omega'] == 0.5
    peak = out['peak_signal_omega']                        # 신호 사례의 실패 평가도 |ω|에 들어간다
    assert (peak['max_omega'], peak['evaluation'], peak['failed']) == (11.0, 9, True)
    assert (peak['move']['parameter'], peak['move']['factor']) == ('p0', 2.0)


def test_large_omega_without_failure_is_the_search_peak():
    # CPID Kd_att ×0.5 사례(평가 98: |ω| 12.9 rad/s, 실패 아님)와 같은 모양 — p1을 줄이면 흔들리지만 실패는 아니다.
    record, log = synthetic_run(lambda e: 2.0**(-e[0]), n=2, budget=12,
                                omega=lambda e, failed: 12.9 if e[1] < 0 else 0.5)
    out = analyse(record, log)
    assert out['failures'] == []
    assert out['best_signal_omega'] == 0.5
    peak = out['peak_signal_omega']
    assert (peak['max_omega'], peak['evaluation'], peak['failed']) == (12.9, 4, False)
    assert (peak['move']['parameter'], peak['move']['factor'], peak['move']['value']) == ('p1', 0.5, 1.0)


def test_converging_objective_reduces_the_step():
    record, log = synthetic_run(lambda e: (e[0] - 0.5)**2 + e[1]**2, n=2, budget=40)
    out = analyse(record, log)
    assert out['search']['step_reduced'] is True
    assert out['search']['smallest_step_exponent'] < 1.0
    assert out['best_exponents'] == [0.5, 0.0]
    assert out['failures'] == []
    assert out['best_objective'] == min(e['objective'] for e in log)


def test_single_fresh_evaluation_cannot_name_constant_failures():
    record, log = synthetic_run(lambda e: 1.0, n=2, budget=1)
    record['status'] = 'running'
    out = analyse(record, log)
    assert out['constant_failures'] is None
    assert out['signal_scenarios'] == ['out', 'in']


def test_tampered_log_is_refused():
    record, log = synthetic_run(lambda e: 2.0**(-e[0]), n=2, budget=12)
    log[3]['exponents'] = [0.0, 2.0]
    with pytest.raises(ReplayMismatch):
        analyse(record, log)


def test_complete_record_with_a_different_best_is_refused():
    record, log = synthetic_run(lambda e: 2.0**(-e[0]), n=2, budget=12)
    record['best_exponents'] = [1.0, 0.0]
    with pytest.raises(ReplayMismatch):
        analyse(record, log)
