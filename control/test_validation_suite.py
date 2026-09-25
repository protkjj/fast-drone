"""Regression tests for scenario separation and honest robustness accounting."""
import json
from copy import deepcopy

import numpy as np
import pytest

from control.mission_profiles import MissionProfile, GustProfile
from control.validation_metrics import Acceptance, evaluate
from control.validation_suite import (build_parser, make_plan, gust_wind, run_trial,
                                      summarize, json_safe, main)
from control.uncertainty import perturb_params, sweep_cases, monte_carlo_cases, DEFAULT_RANGES
from models.team_light.control.baseline_v2 import baseline_params, parameter_hash
from models.team_light.control.dynamics import AxialDronePlant
from models.team_light.control.trim import find_trim
from models.team_light.control.run_baseline_comparison import Factory


def synthetic(profile, dt=.01):
    ts = np.arange(round(profile.T_total/dt)+1)*dt
    vr, zr = profile.compute_refs(ts)
    xs = np.zeros((len(ts), 17))
    xs[:, 2], xs[:, 3:6], xs[:, 9] = zr, vr, 1.
    return dict(ts=ts, xs=xs, us=np.zeros((len(ts)-1, 4)), v_refs=vr, z_refs=zr)


def test_baseline_boundaries_and_reference_continuity():
    profile = MissionProfile()
    assert profile.T_total == 40.
    assert profile.cruise_start == 9 and profile.decel_start == 12
    assert [p[0] for p in profile.phases] == ['초기호버', '가속', '순항', '감속', '호버링']
    ts = np.array([-1., 0., 1., 9., 12., 37., 40., 100.])
    vr, zr = profile.compute_refs(ts)
    np.testing.assert_allclose(vr[:, 0], [0, 0, 0, 70, 70, 0, 0, 0])
    np.testing.assert_allclose(zr, 50.)
    for i, t in enumerate(ts):
        v, z, _ = profile.get_ref(t)
        np.testing.assert_allclose(vr[i], v)
        assert zr[i] == z
    for _, t, _ in profile.get_phase_boundaries():
        np.testing.assert_allclose(profile.get_ref(t-1e-5)[0], profile.get_ref(t+1e-5)[0], atol=1e-8)


@pytest.mark.parametrize('scenario', ['baseline', 'sweep', 'mc'])
def test_non_gust_stages_never_inject_wind(scenario):
    args = build_parser().parse_args(['--scenario', scenario, '--trials', '2'])
    profile, cases, _, _ = make_plan(args)
    assert profile.gust_interval is None
    assert all(gust_wind(profile, case) is None for case in cases)


def test_gust_timing_and_cruise_initial_state():
    p = GustProfile()
    assert p.T_total == 4.5 and p.get_ref(0)[0][0] == 70
    wind = gust_wind(p, dict(gust_direction='lateral', gust_peak=10))
    np.testing.assert_allclose(wind(1.5), [0, 10, 0])
    for t in (0, .99, 1, 2, 3):
        np.testing.assert_allclose(wind(t), 0, atol=1e-10)


def test_gust_recovery_excludes_pulse_and_requires_lateral_recovery():
    p = GustProfile(settle=3., gust_duration=1., recovery=6.)
    r = synthetic(p)
    # Lateral upset with perfect vx and z is still an unsettled gust response.
    r['xs'][(r['ts'] >= 3.1) & (r['ts'] < 6), 4] = 2.
    m = evaluate(r, p)
    assert m['pre_gust_ready'] and m['recovered']
    assert m['recovery_seconds'] == pytest.approx(2.)
    r['xs'][r['ts'] >= 9.8, 4] = 2.
    m = evaluate(r, p)
    assert not m['recovered'] and m['recovery_seconds'] is None
    assert not m['passed']


def test_initial_transient_is_not_gust_success():
    p = GustProfile(settle=3., gust_duration=1., recovery=6.)
    r = synthetic(p)
    r['xs'][r['ts'] < 3, 3] += 1.
    m = evaluate(r, p)
    assert not m['pre_gust_ready'] and not m['passed']
    assert m['recovery_seconds'] == 0  # stable after the pulse, but invalid setup


def test_failure_criteria_detect_nonfinite_peak_and_sustained_spin():
    p = MissionProfile(durations=[.2]*5)
    r = synthetic(p)
    r['xs'][:21, 10] = 26.
    assert 'omega_sustained' in evaluate(r, p)['failure_reasons']
    r['xs'][:, 10] = 0
    r['xs'][5, 10] = 36.
    assert 'omega_peak' in evaluate(r, p)['failure_reasons']
    r['xs'][5, 10] = np.inf
    assert 'nonfinite' in evaluate(r, p)['failure_reasons']
    assert json_safe(evaluate(r, p))['max_omega'] is None


def test_early_stop_with_small_rmse_is_failure():
    p = MissionProfile()
    r = synthetic(p)
    for key in ('ts', 'xs', 'v_refs', 'z_refs'):
        r[key] = r[key][:10]
    r['us'] = r['us'][:9]
    m = evaluate(r, p)
    assert m['rmse_z'] == 0 and not m['passed']
    assert 'incomplete' in m['failure_reasons']


def test_sweep_modes_and_monte_carlo_reproducibility():
    ranges = {'mass': [.8, 1.2], 'motor_tau': [.5, 1.5]}
    assert len(sweep_cases(ranges)) == 5
    assert len(sweep_cases(ranges, mode='grid')) == 9
    a = monte_carlo_cases(ranges, 6, 9)
    assert a == monte_carlo_cases(dict(reversed(list(ranges.items()))), 6, 9)
    assert a != monte_carlo_cases(ranges, 6, 10)
    assert all(set(c['factors']) == set(ranges) for c in a)
    for case in a:
        for key, value in case['factors'].items():
            assert ranges[key][0] <= value <= ranges[key][1]


def test_unperturbed_truth_preserves_the_frozen_parameter_hash():
    nominal = baseline_params()
    assert parameter_hash(perturb_params(nominal, {})) == parameter_hash(nominal)
    assert parameter_hash(perturb_params(nominal, {k: 1. for k in DEFAULT_RANGES})) == parameter_hash(nominal)


def test_completed_mission_requires_final_hover_settling():
    profile = MissionProfile()
    r = synthetic(profile)
    r['xs'][r['ts'] > profile.T_total-.2, 4] = 1.
    metrics = evaluate(r, profile)
    assert not metrics['final_settled']
    assert 'final_hover_not_settled' in metrics['failure_reasons']


@pytest.mark.parametrize('factor', list(DEFAULT_RANGES)+['aero'])
def test_every_uncertainty_factor_changes_actual_dynamics_only(factor):
    nominal = baseline_params()
    frozen = parameter_hash(nominal)
    truth = perturb_params(nominal, {factor: 1.2})
    x = find_trim(nominal, 70.)['state'].copy()
    x[3:6] += [1., 2., 3.]
    x[10:13] = [.3, .4, .5]
    x[13:17] *= [1.02, .98, 1.01, .99]
    command = x[13:17]*1.05
    a = AxialDronePlant(nominal).evaluate_xdot(x, command)
    b = AxialDronePlant(truth).evaluate_xdot(x, command)
    assert np.linalg.norm(a-b) > 1e-5, factor
    assert parameter_hash(nominal) == frozen
    truth['prop_curve']['knots'][0][1] = 999.
    assert parameter_hash(nominal) == frozen


def test_failed_trials_remain_in_mc_denominator():
    rows = [dict(controller='Split', passed=False, failure_reasons=['setup_error'],
                 stop_reason='error') for _ in range(4)]
    s = summarize(rows, 'mc')['Split']
    assert s['failure_rate'] == 1. and s['trials'] == 4
    assert s['complete_metric_trials'] == 0
    assert s['failure_rate_wilson95'][1] == pytest.approx(1.)


@pytest.mark.parametrize('args', [
    ['--speed', '86'], ['--trials', '0'], ['--durations', '0', '1', '1', '1', '1'],
    ['--durations', '.003', '1', '1', '1', '1'], ['--scenario', 'gust', '--gust-settle', '.1'],
])
def test_invalid_experiment_rejected_before_run(args):
    with pytest.raises(ValueError):
        make_plan(build_parser().parse_args(args))


def test_dry_run_saves_replayable_mc_plan(tmp_path):
    out = main(['--scenario', 'mc', '--trials', '3', '--seed', '12',
                '--dry-run', '--output', str(tmp_path)])
    manifest = json.loads((out/'manifest.json').read_text(encoding='utf-8'))
    cases = json.loads((out/'cases.json').read_text(encoding='utf-8'))
    assert manifest['status'] == 'planned'
    assert cases == monte_carlo_cases(DEFAULT_RANGES, 3, 12)
    assert not (out/'trials.jsonl').exists()


def test_real_gust_trial_starts_at_cruise_and_repeated_trials_are_independent():
    factory = Factory(baseline_params())
    p = GustProfile(settle=.5, gust_duration=.1, recovery=.5)
    case = dict(factors={}, gust_direction='vertical', gust_peak=.2)
    a, trace, _ = run_trial(factory, 'GS-LQR', p, case, Acceptance())
    b, repeated, _ = run_trial(factory, 'GS-LQR', p, case, Acceptance())
    assert a['pre_gust_ready'] and b['pre_gust_ready']
    assert trace['xs'][0, 3] == 70 and trace['xs'][0, 2] == 50
    assert trace['ts'][-1] == pytest.approx(1.1)
    np.testing.assert_array_equal(trace['xs'], repeated['xs'])
    np.testing.assert_array_equal(trace['us'], repeated['us'])


def test_historical_timeline_is_explicit():
    from control.mission_sim import LegacyMissionProfile
    legacy = LegacyMissionProfile()
    assert legacy.T_total == 65 and legacy.cruise_start == 28
    assert MissionProfile().T_total == 40
