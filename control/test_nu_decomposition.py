"""원고 식(29)·(37) 계측(control/validation_suite.py nu_decomposition, ProperHybrid.probe) — kj 결정(2026-09-28 밤).

  1) 계산: r_ν = ν_alloc − ν_d, ε_ν = ν_act − ν_alloc을 D_ν로 나눈 성분별·노름의 적분·최댓값. ν_alloc이 없는 스텝은 뺀다.
  2) D_ν = diag(명목 m·g, 100, 100, 100) — V13 NMPC 비용함수(식 15)의 정규화와 같은 값.
  3) ν_act(플랜트 참 맵, ω=0)는 플랜트 호버 트림에서 총추력 = m·g, 각가속도 ≈ 0.
  4) 적용 범위: 가상입력 인터페이스가 있는 V13 사다리·F13만, 나머지는 applicable=False.
기록만 하고 제어는 그대로라는 주장은 옛 ProperHybrid와의 궤적 해시 대조와 arena_tune_repro로 확인했다(보고서 25절).
"""
import numpy as np
import pytest

from control.arena import load_config
from control.arena_factory import ArenaFactory
from control.validation_suite import (baseline_params, nu_decomposition, nu_scale, actual_input_function,
                                      run_trial)


@pytest.fixture(scope='module')
def native():
    return baseline_params()


@pytest.fixture(scope='module')
def factory(native):
    return ArenaFactory(load_config(), native)


def test_decomposition_arithmetic_and_skipped_steps():
    scale = np.array([10.0, 100.0, 100.0, 100.0])
    nu_d = np.array([[10.0, 0, 0, 0], [12.0, 10.0, 0, 0], [11.0, 0, 0, 0]])
    nu_alloc = np.array([[10.0, 0, 0, 0], [11.0, 0.0, 0, 0], [np.nan]*4])
    nu_act = np.array([[9.0, 0, 0, 0], [11.0, 20.0, 0, 0], [0.0, 0, 0, 0]])
    d = nu_decomposition(nu_d, nu_alloc, nu_act, 0.5, scale)
    assert d['steps'] == 3 and d['steps_without_alloc'] == 1
    r, e = d['allocation_residual'], d['realization_error']
    np.testing.assert_allclose(r['max'], [0.1, 0.1, 0, 0])
    np.testing.assert_allclose(r['integral'], [0.05, 0.05, 0, 0])
    np.testing.assert_allclose(e['max'], [0.1, 0.2, 0, 0])
    assert r['norm_max'] == pytest.approx(np.hypot(0.1, 0.1))
    assert r['components'] == ['thrust', 'alpha_x', 'alpha_y', 'alpha_z']


def test_scale_is_the_nmpc_input_normalization(factory):
    from control.hybrid_comparison import VirtualNMPC
    scale = nu_scale(factory.cp)
    assert scale[0] == pytest.approx(factory.cp['mass']*factory.cp['g']) and list(scale[1:]) == [100.0]*3
    nmpc = VirtualNMPC(factory.cp, v_ref=[0.0, 0.0, 0.0], z_ref=20.0, cost_spec='paper')
    assert nmpc.T_ref == pytest.approx(scale[0])


def test_actual_input_at_plant_hover_trim_is_weight(native):
    from models.team_light.control.trim import find_trim
    from scipy.spatial.transform import Rotation
    tr = find_trim(native, 0.0)
    x = tr['state']
    vb = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]
    nu = np.array(actual_input_function(native)(vb, x[13:17])).ravel()
    assert nu[0] == pytest.approx(native['mass']*native['g'], rel=1e-9)
    assert np.abs(nu[1:]).max() < 1e-9


@pytest.mark.parametrize('label, applicable', [('V13', True), ('F13', True), ('GSLQR', False)])
def test_applicability_per_controller(factory, label, applicable):
    from control.mission_profiles import StepProfile
    from control.validation_metrics import Acceptance
    profile = StepProfile(0.0, 20.0, 0.0, 0.3, duration=0.4, ramp_s=0.2)
    metrics, result, _ = run_trial(factory, label, profile, dict(case_id='nu', factors={}),
                                   Acceptance(**load_config()['acceptance']))
    d = metrics['nu_decomposition']
    assert d['applicable'] is applicable
    if applicable:
        assert result['nu_d'].shape == result['nu_alloc'].shape == result['nu_act'].shape == (len(result['us']), 4)
        assert d['steps'] == len(result['us']) and d['steps_without_alloc'] >= 1      # 첫 스텝은 초기화 경로
    else:
        assert 'nu_d' not in result
