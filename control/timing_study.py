"""Reproducible segment-duration experiments; never retune the controller.

Standalone ramps are screening experiments only. A recommended mission must
also be rerun end-to-end. Every failed candidate remains in the output.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
import json
import hashlib
import os

import numpy as np
from scipy.spatial.transform import Rotation

from control.mission_profiles import (MissionProfile, GustProfile,
                                      DEFAULT_MISSION_DURATIONS, DEFAULT_GUST_TIMES)
from control.validation_metrics import Acceptance
from control.validation_suite import run_trial, write_json, plot_traces
from models.team_light.control.baseline_v2 import baseline_params, parameter_hash
from models.team_light.control.run_baseline_comparison import Factory
from models.team_light.control.propeller_curve import positive_thrust_j_limit


class RampProfile(MissionProfile):
    def __init__(self, v0, v1, duration, lead=1., tail=6., altitude=50.):
        intervals = np.asarray([lead, duration, tail], dtype=float)
        if not np.isfinite(intervals).all() or np.any(intervals <= 0):
            raise ValueError('ramp intervals must be finite and positive')
        if not np.allclose(intervals/.002, np.round(intervals/.002), atol=1e-8, rtol=0):
            raise ValueError('ramp intervals must be multiples of 0.002 seconds')
        if not np.isfinite([v0, v1]).all() or min(v0, v1) < 0:
            raise ValueError('speeds must be finite and nonnegative')
        super().__init__(max(v0, v1), altitude)
        self.phases = [
            ('initial', 0., lead, v0, v0, altitude, altitude),
            ('ramp', lead, duration, v0, v1, altitude, altitude),
            ('settle', lead+duration, tail, v1, v1, altitude, altitude),
        ]
        self.T_total = lead+duration+tail


def settling_time(trace, start, end, z=.05, velocity=.1, omega=.05, hold=1.):
    """Earliest time inside a tighter observation band through the segment end."""
    ts, xs = trace['ts'], trace['xs']
    if ts[-1] < end-1e-8:
        return None
    use = (ts >= start-1e-9) & (ts <= end+1e-9)
    times = ts[use]
    stable = (np.abs(xs[use, 2]-trace['z_refs'][use]) <= z) & (
        np.linalg.norm(xs[use, 3:6]-trace['v_refs'][use], axis=1) <= velocity) & (
        np.linalg.norm(xs[use, 10:13], axis=1) <= omega)
    suffix = np.logical_and.accumulate(stable[::-1])[::-1]
    candidates = np.flatnonzero(suffix & (end-times >= hold-1e-9))
    return float(max(0., times[candidates[0]]-start)) if len(candidates) else None


def domain_breakdown(trace, p):
    x, t = trace['xs'][1:], trace['ts'][1:]
    if not len(x) or not np.isfinite(x).all():
        return {}
    w = trace.get('wind', np.zeros((len(x), 3)))
    vb = Rotation.from_quat(x[:, 6:10]).inv().apply(x[:, 3:6]-w)
    rpm = x[:, 13:17]*60/(2*np.pi)
    j = np.maximum(vb[:, :1], 0)/(x[:, 13:17]*p['D_prop']/(2*np.pi)+1e-8)
    low_rpm = p['prop_curve']['assumed_rpm_working_range'][0]
    masks = {'rpm_below_data': (rpm < low_rpm-1e-6).any(axis=1),
             'advance_ratio_above_data': (j > positive_thrust_j_limit(p)+1e-8).any(axis=1),
             'reverse_flow': vb[:, 0] < -1e-8}
    out = {'minimum_axial_air_speed': float(vb[:, 0].min()),
           'minimum_rotor_rpm': float(rpm.min())}
    for name, mask in masks.items():
        q = t[mask]
        out[name] = dict(fraction=float(mask.mean()),
                         first=float(q[0]) if len(q) else None,
                         last=float(q[-1]) if len(q) else None)
    return out


def analyze(trace, profile, params):
    phases = {}
    for name, start, end in profile.get_phase_boundaries():
        phase = next(p for p in profile.phases if p[0] == name)
        if phase[3] == phase[4]:
            phases[name] = settling_time(trace, start, end)
    return dict(settling_seconds=phases, domain=domain_breakdown(trace, params))


def numerical_timing_eligible(row, max_z=.5, max_velocity=3.5):
    """Engineering screening budget, separate from strict model-domain validity.

    0.5 m = 1% of 50 m altitude; 3.5 m/s = 5% of 70 m/s cruise.
    Passing this filter never overrides metrics['passed'] or domain flags.
    """
    m = row['metrics']
    return bool(m['tracking_pass'] and m['optimizer_failures'] == 0 and
                m['max_z_error'] <= max_z and m['max_velocity_error'] <= max_velocity and
                m['motor_saturation_fraction'] == 0 and
                all(v is not None for v in row['analysis']['settling_seconds'].values()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study', choices=['accel', 'decel', 'mission', 'gust'], required=True)
    parser.add_argument('--times', type=float, nargs='+', default=[15., 25., 40.])
    parser.add_argument('--durations', type=float, nargs=5, default=list(DEFAULT_MISSION_DURATIONS))
    parser.add_argument('--gust-times', type=float, nargs=3, default=list(DEFAULT_GUST_TIMES))
    parser.add_argument('--controllers', nargs='+', choices=['GS-LQR', 'NMPC', 'Split'], default=['Split'])
    parser.add_argument('--output', type=Path, default=Path('results/validation'))
    args = parser.parse_args(argv)
    out = args.output/('timing_'+args.study+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True, exist_ok=False)
    p = baseline_params()
    limits = Acceptance()
    root = Path(__file__).resolve().parents[1]
    sources = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
               for folder in ('control', 'models/team_light/control') for path in (root/folder).glob('*.py')}
    write_json(out/'manifest.json', dict(arguments=vars(args), parameter_hash=parameter_hash(p),
                                         source_sha256=sources,
                                         environment={k: os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS')},
                                         acceptance=asdict(limits), status='running',
                                         observation=dict(z=.05, velocity=.1, omega=.05, hold=1.)))
    factory = Factory(p)
    rows = []
    for duration in (args.times if args.study in ('accel', 'decel') else [None]):
        if args.study in ('accel', 'decel'):
            profile = RampProfile(0. if args.study == 'accel' else 70.,
                                  70. if args.study == 'accel' else 0., duration)
            cases = [dict(case_id=f'{args.study}_{duration:g}', factors={})]
        elif args.study == 'mission':
            profile = MissionProfile(durations=args.durations)
            cases = [dict(case_id='mission', factors={})]
        else:
            profile = GustProfile(settle=args.gust_times[0], gust_duration=args.gust_times[1],
                                  recovery=args.gust_times[2])
            cases = [dict(case_id='control', factors={}, gust_peak=0.)] + [
                dict(case_id=f'gust_{d}', factors={}, gust_direction=d, gust_peak=2.)
                for d in ('vertical', 'lateral')]
        for case in cases:
            for name in args.controllers:
                print(f'Running {case["case_id"]} {name}; total={profile.T_total:g}s; {out}', flush=True)
                metrics, trace, log = run_trial(factory, name, profile, case, limits)
                row = dict(case=case, controller=name, phases=profile.phases,
                           metrics=metrics, analysis=analyze(trace, profile, p))
                rows.append(row)
                tag = case['case_id']+'_'+name
                write_json(out/(tag+'.json'), row)
                write_json(out/(tag+'.solver.json'), log)
                np.savez_compressed(out/(tag+'.npz'), **{k: v for k, v in trace.items() if k != 'error'})
                write_json(out/'summary.json', rows)
                print(json.dumps(dict(controller=name, tracking=metrics['tracking_pass'],
                                      duration=metrics['simulated_seconds'],
                                      failures=metrics['optimizer_failures'],
                                      reasons=metrics['failure_reasons'],
                                      analysis=row['analysis']), ensure_ascii=False), flush=True)
        plot_traces(out, cases, args.controllers, profile)
    manifest = json.loads((out/'manifest.json').read_text(encoding='utf-8'))
    manifest.update(status='complete', recorded_trials=len(rows))
    write_json(out/'manifest.json', manifest)
    return out


if __name__ == '__main__':
    main()
