"""Timing diagnostics must not hide late excursions or incomplete runs."""
import numpy as np
import pytest

from control.timing_study import (RampProfile, settling_time, domain_breakdown,
                                   numerical_timing_eligible)
from control.validation_suite import build_parser, make_plan
from control.mission_profiles import MissionProfile, GustProfile
from models.team_light.control.baseline_v2 import baseline_params
from models.team_light.control.trim import find_trim


def trace_at_reference(profile, dt=.002):
    ts = np.arange(round(profile.T_total/dt)+1)*dt
    vr, zr = profile.compute_refs(ts)
    x = np.zeros((len(ts), 17))
    x[:, 2], x[:, 3:6], x[:, 9] = zr, vr, 1.
    return dict(ts=ts, xs=x, v_refs=vr, z_refs=zr)


def test_ramps_start_at_requested_equilibrium_and_end_at_target():
    for a, b in [(0, 70), (70, 0)]:
        profile = RampProfile(a, b, 8.)
        assert profile.T_total == 15.
        assert profile.get_ref(0)[0][0] == a
        assert profile.get_ref(9)[0][0] == b
        assert profile.get_ref(5)[0][0] == pytest.approx(35.)
        assert profile.gust_interval is None


@pytest.mark.parametrize('duration', [0, -1, np.nan, np.inf, .003])
def test_invalid_screening_duration_is_rejected(duration):
    with pytest.raises(ValueError):
        RampProfile(0, 70, duration)


def test_settling_requires_stable_tail_and_full_observation():
    p = RampProfile(0, 70, 8.)
    r = trace_at_reference(p)
    assert settling_time(r, 9., 15.) == 0.
    r['xs'][(r['ts'] >= 9) & (r['ts'] < 10.2), 4] = .2
    assert settling_time(r, 9., 15.) == pytest.approx(1.2)
    r['xs'][r['ts'] > 14.5, 10] = .1
    assert settling_time(r, 9., 15.) is None
    r = {k: v[:500] for k, v in r.items()}
    assert settling_time(r, 9., 15.) is None


def test_domain_diagnostic_retains_small_reverse_flow():
    p = baseline_params()
    x = find_trim(p, 70.)['state']
    trace = dict(xs=np.tile(x, (3, 1)), ts=np.array([0, .002, .004]))
    good = domain_breakdown(trace, p)
    assert good['rpm_below_data']['fraction'] == 0
    assert good['reverse_flow']['fraction'] == 0
    trace['xs'][1:, 3:6] *= -1e-6
    bad = domain_breakdown(trace, p)
    assert bad['reverse_flow']['fraction'] == 1.


def test_cli_and_python_defaults_generate_identical_optimized_references():
    for scenario, expected in [('baseline', MissionProfile()), ('gust', GustProfile())]:
        actual, *_ = make_plan(build_parser().parse_args(['--scenario', scenario]))
        assert actual.phases == expected.phases
        assert actual.T_total == expected.T_total


def test_numerical_timing_screen_does_not_erase_domain_failure():
    row = dict(metrics=dict(tracking_pass=True, passed=False,
                            optimizer_failures=0, max_z_error=.4, max_velocity_error=3.,
                            motor_saturation_fraction=0, failure_reasons=['propulsion_model_domain']),
               analysis=dict(settling_seconds={'initial': 0., 'settle': .2}))
    assert numerical_timing_eligible(row)
    assert not row['metrics']['passed']
    row['metrics']['max_velocity_error'] = 3.6
    assert not numerical_timing_eligible(row)
    row['metrics']['max_velocity_error'] = 3.
    row['metrics']['optimizer_failures'] = 1
    assert not numerical_timing_eligible(row)
