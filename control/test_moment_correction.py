"""제어기 모델 모멘트 보정항(control/moment_correction.py, control/dynamics.py, configs/arena_v2.json) —
kj 결정(2026-09-28 밤) 시험. I-10 확장 관문(±20° 부호·상대오차)은 test_arena_fairness.py의 I-10 절에 있다.

  1) 키가 없으면 보정 함수를 부르지 않는다(기본 경로 비트 동일 — test_plant·repro가 전체를 본다).
  2) 보정은 M_y·M_z만 바꾼다(힘·M_x는 비트 동일).
  3) 명목 트림 86점(1 m/s 격자)에서 보정항 ≤ 1e-15 N·m(정의상 0, 부동소수 잔여만).
  4) 적합은 결정적이다(다시 적합하면 파일과 같다).
  5) arena_v2.json은 controller_model.moment_correction 절만 arena.json과 다르다.
  6) 파일 sha256이 설정과 다르면 팩토리가 거부한다.
  7) 배선: M17·F13·GSLQR은 보정 모델, V13·CPID·명목 트림·a_avail은 기존 모델.
  8) 플랜트는 받음각 0°에서 축대칭(피치·요 기울기 크기 같음) — 적합 계수가 두 축에서 다른 이유는
     트림 받음각이 0이 아니어서다(보고서 23절).
"""
from copy import deepcopy
import json

import casadi as ca
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from control import dynamics
from control.arena import ROOT, load_config
from control.arena_factory import ArenaFactory, moment_corrected_params
from control.moment_correction import load, fit, DEFAULT_FILE
from control.validation_suite import baseline_params

V2 = ROOT/'configs'/'arena_v2.json'


@pytest.fixture(scope='module')
def native():
    return baseline_params()


@pytest.fixture(scope='module')
def factory(native):
    return ArenaFactory(load_config(), native)


@pytest.fixture(scope='module')
def factory_v2(native, factory):
    return ArenaFactory(load_config(V2), native, model=factory.model)


def _moment_fn(cp):
    v, w = ca.SX.sym('v', 3), ca.SX.sym('w', 3)
    return ca.Function('aero', [v, w], list(dynamics._body_aerodynamics(v, w, cp)))


def test_without_the_key_the_correction_is_never_built(factory, monkeypatch):
    monkeypatch.setattr(dynamics, '_moment_correction', lambda *a: pytest.fail('correction built'))
    _moment_fn(factory.cp)
    assert factory.cp_pred is factory.cp


def test_correction_changes_only_pitch_and_yaw_moment(factory_v2):
    old, new = _moment_fn(factory_v2.cp), _moment_fn(factory_v2.cp_pred)
    for vb in ([40.0, 3.0, 12.0], [85.0, -5.0, -9.0], [20.0, 8.0, 15.0]):
        w = [0.3, -0.2, 0.1]
        F0, M0 = (np.array(t).ravel() for t in old(vb, w))
        F1, M1 = (np.array(t).ravel() for t in new(vb, w))
        assert np.array_equal(F0, F1) and M0[0] == M1[0]
        assert M0[1] != M1[1] and M0[2] != M1[2]


def test_correction_vanishes_at_every_nominal_trim(factory_v2):
    old, new = _moment_fn(factory_v2.cp), _moment_fn(factory_v2.cp_pred)
    for V in range(0, 86):
        tr = factory_v2.model.trim(float(V))
        vb = Rotation.from_quat(tr['state'][6:10]).as_matrix().T @ tr['state'][3:6]
        d = np.array(new(vb, [0, 0, 0])[1]).ravel() - np.array(old(vb, [0, 0, 0])[1]).ravel()
        assert np.abs(d).max() <= 1e-15, (V, d)


def test_fit_is_deterministic_and_matches_the_file(native, factory):
    doc = fit(native, factory.cp, factory.model.trim)
    on_disk = load(DEFAULT_FILE)
    for key in ('pitch', 'yaw'):
        assert doc[key]['coeffs'] == on_disk[key]['coeffs']
    assert doc['alpha_ref'] == on_disk['alpha_ref']


def test_arena_v2_differs_only_in_the_moment_correction_section():
    base, v2 = load_config(), load_config(V2)
    stripped = deepcopy(v2)
    del stripped['controller_model']['moment_correction']
    assert stripped == base
    assert v2['controller_model']['moment_correction']['applies_to'] == ['M17', 'F13', 'GSLQR']


def test_factory_refuses_a_moment_file_with_another_hash(factory):
    config = load_config(V2)
    config['controller_model']['moment_correction']['sha256'] = '0'*64
    with pytest.raises(ValueError, match='sha256 differs'):
        moment_corrected_params(config, factory.cp)


def test_wiring_gives_the_corrected_model_only_to_m17_f13_gslqr(factory_v2, monkeypatch):
    import control.nmpc, control.nmpc_f13, control.controller, control.hybrid_comparison
    from control.mission_profiles import MissionProfile
    seen = {}

    class Stop(Exception):
        pass

    def spy(label):
        def f(params, *a, **k):
            seen[label] = params
            raise Stop
        return f

    monkeypatch.setattr(control.nmpc, 'NMPCController', spy('M17'))
    monkeypatch.setattr(control.nmpc_f13, 'build_f13_controller', spy('F13'))
    monkeypatch.setattr(control.controller, 'ScheduledLQR', spy('GSLQR'))
    monkeypatch.setattr(control.controller, 'CascadedPID', spy('CPID'))
    monkeypatch.setattr(control.hybrid_comparison, 'VirtualNMPC', spy('V13'))
    profile = MissionProfile(20.0, 20.0, (1.0,)*5)
    for label in ('M17', 'F13', 'GSLQR', 'CPID', 'V13'):
        with pytest.raises(Stop):
            factory_v2.make_for_profile(label, profile)
    for label in ('M17', 'F13', 'GSLQR'):
        assert 'moment_correction' in seen[label], label
    for label in ('V13', 'CPID'):
        assert seen[label] is factory_v2.cp and 'moment_correction' not in seen[label], label
    assert 'moment_correction' not in factory_v2.model.cp          # 명목 트림·a_avail이 쓰는 모델


def test_plant_is_axisymmetric_at_zero_incidence(native):
    from control.kh_adapter import _build_aero_function
    f = _build_aero_function(native)
    for V in (20.0, 85.0):
        h = np.radians(1.0)
        my = lambda a: np.array(f([V*np.cos(a), 0, V*np.sin(a)], [0, 0, 0])[1]).ravel()[1]
        mz = lambda b: np.array(f([V*np.cos(b), V*np.sin(b), 0], [0, 0, 0])[1]).ravel()[2]
        slope_y, slope_z = (my(h) - my(-h))/(2*h), (mz(h) - mz(-h))/(2*h)
        assert slope_y == pytest.approx(-slope_z, rel=1e-9)
