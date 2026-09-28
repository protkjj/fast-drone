"""튜닝 평가의 시나리오 병렬화(--scenario-workers)가 결과를 바꾸지 않는지 — 순차(N=1)와 비트 동일.

비교 대상은 평가가 남기는 모든 것이다: 목적함수, 시나리오별 항목 전체(궤적 sha256 포함, 순서 포함),
그리고 tune_controller가 쓰는 <제어기>.jsonl 파일의 바이트.
시나리오 18개 전부를 쓴다(CPID는 실패=벌점 사례가 섞여 있어 실패 경로도 함께 확인된다).
무거운 NMPC(V13·F13·M17)는 여기서 돌리지 않는다 — 튜닝 경로 실측은 NEXT_STEPS b에서 한다.
"""
import json
import os

import pytest

from control.arena import load_config
from control.arena_factory import ArenaFactory, ControllerModel
from control.arena_tune import Evaluator, parameter_space, tune_controller
from models.team_light.control.baseline_v2 import baseline_params

QUICK = os.environ.get('ARENA_QUICK') == '1'
slow = pytest.mark.skipif(QUICK, reason='ARENA_QUICK=1: heavy closed-loop check skipped')


@pytest.fixture(scope='module')
def setup():
    config = load_config()
    native = baseline_params()
    return config, native, ControllerModel(native)


def _run(setup, label, overrides, workers):
    config, native, model = setup
    evaluator = Evaluator(config, native, model, label, scenario_workers=workers)
    try:
        return evaluator(overrides)
    finally:
        evaluator.close()


def _shifted_overrides(setup, label):
    """사전값이 아닌 후보(첫 좌표 ×2) — overrides가 작업자에게 제대로 넘어가는지 보려고."""
    config, native, model = setup
    gains = ArenaFactory(config, native, model=model).gains
    names, prior, apply = parameter_space(config, label, gains)
    values = dict(prior, **{names[0]: prior[names[0]]*2.0})
    return apply(values)


@slow
@pytest.mark.parametrize('label', ['CPID', 'GSLQR'])
@pytest.mark.parametrize('candidate', ['prior', 'shifted'])
def test_parallel_evaluation_is_bit_identical(setup, label, candidate):
    overrides = {} if candidate == 'prior' else _shifted_overrides(setup, label)
    seq_obj, seq_scores = _run(setup, label, overrides, workers=1)
    par_obj, par_scores = _run(setup, label, overrides, workers=3)
    assert par_obj == seq_obj
    assert [e['id'] for e in par_scores] == [e['id'] for e in seq_scores]
    assert json.dumps(par_scores, sort_keys=True, default=str) == \
        json.dumps(seq_scores, sort_keys=True, default=str)
    # 비교가 헛돌지 않았는지: 실제로 궤적이 나온 사례가 있어야 한다.
    assert any(e.get('trajectory_sha256') for e in seq_scores)


@slow
def test_shifted_candidate_actually_differs(setup):
    """위 시험의 'shifted'가 사전값과 같은 결과를 냈다면 overrides 전달을 확인하지 못한 것이다."""
    prior_obj, _ = _run(setup, 'GSLQR', {}, workers=1)
    shifted_obj, _ = _run(setup, 'GSLQR', _shifted_overrides(setup, 'GSLQR'), workers=2)
    assert shifted_obj != prior_obj


@slow
def test_tune_controller_log_is_byte_identical(setup, tmp_path):
    config, native, model = setup
    logs = {}
    for workers in (1, 2):
        run_dir = tmp_path/f'w{workers}'
        record = tune_controller(config, 'GSLQR', 2, run_dir, native=native, model=model,
                                 scenario_workers=workers)
        assert record['scenario_workers'] == workers
        logs[workers] = (run_dir/'GSLQR.jsonl').read_bytes()
    assert logs[1] == logs[2]


def test_scenario_workers_must_be_positive(setup):
    config, native, model = setup
    with pytest.raises(ValueError):
        Evaluator(config, native, model, 'GSLQR', scenario_workers=0)
