"""arena_tune_repro의 합격 판정(rtol + 판정 일치) — 합성 기록으로 경계를 확인한다(시뮬레이션 없음)."""
import copy

import pytest

from control.arena import load_config
from control.arena_tune_repro import tolerance_problems


def _entry(id_, score=0.5, failed=False, at_limit=0.0):
    return dict(id=id_, failed=failed, stop_reason=None, paper_reasons=[], score=score,
                window_rmse_velocity=score*0.6, window_rmse_z=score*0.4, max_omega=3.0,
                trajectory_sha256='a'*64,
                integrators=dict(steps=2000, at_limit_fraction=at_limit, channels={}))


@pytest.fixture(scope='module')
def config():
    return load_config()


@pytest.fixture
def reference():
    return [_entry('s1', 0.5), _entry('s2', 1.2)]


def _check(new, reference, config, rtol=1e-3):
    objective = sum(e['score'] for e in new)/len(new)
    ref_objective = sum(e['score'] for e in reference)/len(reference)
    return tolerance_problems(new, reference, objective, ref_objective, config, rtol)


def test_identical_passes(reference, config):
    assert _check(copy.deepcopy(reference), reference, config) == []


def test_hash_difference_alone_passes(reference, config):
    new = copy.deepcopy(reference)
    new[0]['trajectory_sha256'] = 'b'*64          # 비트 불일치는 기록만 한다
    assert _check(new, reference, config) == []


@pytest.mark.parametrize('relative, ok', [(5e-4, True), (2e-3, False)])
def test_numeric_rtol_boundary(reference, config, relative, ok):
    new = copy.deepcopy(reference)
    new[1]['max_omega'] *= 1 + relative
    assert (_check(new, reference, config) == []) is ok


def test_verdict_flip_fails(reference, config):
    new = copy.deepcopy(reference)
    new[0]['failed'] = True
    assert any('failed' in p for p in _check(new, reference, config))


def test_integrator_flag_change_fails(reference, config):
    new = copy.deepcopy(reference)
    new[0]['integrators']['at_limit_fraction'] = 0.5
    assert any('integrator' in p for p in _check(new, reference, config))


def test_order_change_fails(reference, config):
    assert _check(list(reversed(reference)), reference, config) == ['scenario ids or order differ']
