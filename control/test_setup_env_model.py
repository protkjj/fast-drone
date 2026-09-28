"""scripts/setup_env.py의 제어기 모델 검사(2026-09-28 밤, kj) — 해시 정확 일치 대신 적합 계수 rtol 1e-8 비교.

리눅스 실측: 설정 해시·커밋·스레드는 같았는데 제어기 모델 sha가 달라 A.3이 FAIL했다(2582197d… → 10b40c8c…).
계수 적합(fit_lumped_aero·fit_cp_schedule)의 부동소수 차이 때문이다. 판정은 계수 비교로 하고 sha는 기록만 한다.
"""
from copy import deepcopy
import importlib.util
import json

import pytest

from control.arena import ROOT


@pytest.fixture(scope='module')
def setup_env():
    spec = importlib.util.spec_from_file_location('setup_env', ROOT/'scripts'/'setup_env.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def reference(setup_env):
    return json.loads(setup_env.MODEL_REFERENCE.read_text(encoding='utf-8'))


@pytest.fixture(scope='module')
def cp():
    from control.arena_factory import ControllerModel
    from control.validation_suite import baseline_params
    return ControllerModel(baseline_params()).cp


def _perturbed(reference, key, rel):
    ref = deepcopy(reference)
    value = ref['coefficients'][key]
    ref['coefficients'][key] = [v*(1 + rel) for v in value] if isinstance(value, list) else value*(1 + rel)
    return ref


def test_reference_file_matches_this_platform_exactly(setup_env, reference, cp):
    ok, rows = setup_env.compare_model(cp, reference)
    assert ok and all(r[3] == 'OK' for r in rows)


@pytest.mark.parametrize('key', ['C_Na', 'x_cp', 'C_mq', 'x_cp_poly'])
def test_difference_beyond_rtol_fails_and_within_passes(setup_env, reference, cp, key):
    assert not setup_env.compare_model(cp, _perturbed(reference, key, 1e-7))[0]
    assert setup_env.compare_model(cp, _perturbed(reference, key, 1e-9))[0]


def test_polynomial_length_mismatch_fails(setup_env, reference, cp):
    ref = deepcopy(reference)
    ref['coefficients']['x_cp_poly'] = ref['coefficients']['x_cp_poly'][:-1]
    assert not setup_env.compare_model(cp, ref)[0]


def test_model_sha_is_recorded_not_judged(setup_env):
    ok, rows = setup_env.check_config(str(ROOT/'configs'/'arena_v2.json'), None, '0'*64)
    sha_rows = [r for r in rows if r[0].startswith('controller_model sha256')]
    assert ok and sha_rows and sha_rows[0][3] == 'INFO'
