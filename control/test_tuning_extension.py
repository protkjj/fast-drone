"""튜닝 예산 연장 판정(control/tuning_extension.py) — kj 규칙(2026-09-28 밤) 시험. 합성 기록으로 본다."""
import json

import pytest

from control.tuning_extension import verdict, decide


def _log(scores_per_eval, cache_at=()):
    """scores_per_eval[i] = [(score, failed), ...]. 목적함수 = 시나리오 점수 평균(벌점 1000 포함)."""
    log = []
    for i, scores in enumerate(scores_per_eval):
        scen = [dict(id=f's{j}', score=(1000.0 if f else v), failed=f) for j, (v, f) in enumerate(scores)]
        entry = dict(index=i, exponents=[float(i)], objective=sum(s['score'] for s in scen)/len(scen),
                     cache_hit=False, scenarios=scen)
        log.append(entry)
    for i in cache_at:                      # 캐시 적중: 앞 평가와 같은 지수, 점수 없음
        log[i] = dict(log[i - 1], index=i, cache_hit=True, scenarios=None)
    return log


def _flat(n, value, failed=(False, False)):
    return [[(value, failed[0]), (value, failed[1])] for _ in range(n)]


def test_continuous_improvement_above_one_percent_is_improvement():
    v = verdict(_log(_flat(90, 0.5) + _flat(30, 0.49)))            # 2% 개선
    assert v['continuous_rate'] == pytest.approx(0.02) and v['improved']


def test_small_improvement_is_not():
    assert not verdict(_log(_flat(90, 0.5) + _flat(30, 0.4975)))['improved']      # 0.5%


def test_penalty_does_not_hide_continuous_improvement():
    """보완 전 규칙의 맹점: 벌점이 섞이면 목적함수 개선율이 거의 0이지만 연속 성분은 좋아졌다."""
    v = verdict(_log(_flat(90, 0.5, failed=(False, True)) + _flat(30, 0.45, failed=(False, True))))
    assert (v['objective_at'][0] - v['objective_at'][1])/v['objective_at'][0] < 0.001
    assert v['continuous_rate'] == pytest.approx(0.1) and v['improved']


def test_fewer_failures_counts_as_improvement_even_if_continuous_worse():
    v = verdict(_log(_flat(90, 0.5, failed=(False, True)) + [[(0.6, False), (0.7, False)]]*30))
    assert v['fewer_failures'] and v['improved'] and v['failures_at'] == [1, 0]


def test_cache_hit_never_becomes_the_best_entry():
    """캐시 적중은 앞선 원래 평가와 목적함수가 같고 최선은 같은 값이면 앞 번호를 고르므로, 최선은 늘 점수가
    있는 원래 평가다(도구의 '캐시 적중이면 원래 계산을 찾는' 경로는 방어용이라 여기서 도달하지 않는다)."""
    log = _log(_flat(90, 0.5) + _flat(29, 0.49) + _flat(1, 0.49))
    log[119] = dict(log[118], index=119, cache_hit=True, scenarios=None)
    v = verdict(log)
    assert v['continuous_at'][1] == pytest.approx(0.49)


def test_decision_waits_for_all_five_and_extends_on_any(tmp_path):
    def write(label, rows):
        (tmp_path/f'{label}.jsonl').write_text('\n'.join(json.dumps(e) for e in _log(rows)), encoding='utf-8')
    for label in ('V13', 'M17', 'F13', 'GSLQR', 'CPID'):
        write(label, _flat(90, 0.5) + _flat(30, 0.45 if label == 'F13' else 0.4999))
    assert decide([tmp_path])['extend_to_180'] is True
    write('M17', _flat(60, 0.5))
    d = decide([tmp_path])
    assert d['extend_to_180'] is None and d['incomplete'] == ['M17']


def test_no_controller_improving_ends_at_120(tmp_path):
    for label in ('V13', 'M17', 'F13', 'GSLQR', 'CPID'):
        (tmp_path/f'{label}.jsonl').write_text('\n'.join(json.dumps(e) for e in _log(_flat(120, 0.5))),
                                                encoding='utf-8')
    assert decide([tmp_path])['extend_to_180'] is False
