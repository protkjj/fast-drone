"""Four independent validation stages using the pinned team aircraft.

python -m control.mission_sim --scenario baseline|gust|sweep|mc
Results describe the tested model and sampled conditions, not a proof for all
uncertainties. No parameter changes are disclosed to the controller.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
from importlib.metadata import version
import subprocess
from time import perf_counter

import numpy as np
from scipy.spatial.transform import Rotation

from control.mission_profiles import (MissionProfile, GustProfile, DEFAULT_GUST_TIMES,
                                      DEFAULT_MISSION_DURATIONS)
from control.validation_metrics import Acceptance, evaluate
from control.uncertainty import (DEFAULT_RANGES, validate_ranges, perturb_params,
                                 sweep_cases, monte_carlo_cases)
from models.team_light.control.baseline_v2 import baseline_params, parameter_hash
from models.team_light.control.dynamics import AxialDronePlant
from models.team_light.control.trim import find_trim
from models.team_light.control.run_baseline_comparison import Factory, LABELS, DT, solver_of
from models.team_light.control.propeller_curve import domain_status

ROOT = Path(__file__).resolve().parents[1]


def json_safe(value):
    """Strict JSON: invalid numeric metrics remain null, with failure reasons."""
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def write_json(path, value):
    path.write_text(json.dumps(json_safe(value), ensure_ascii=False,
                               indent=2, allow_nan=False)+'\n', encoding='utf-8')


def gust_wind(profile, case):
    if not isinstance(profile, GustProfile) or case.get('gust_peak', 0) == 0:
        return None
    start, end = profile.gust_interval
    axis = {'lateral': 1, 'vertical': 2}[case['gust_direction']]

    def wind(t):
        w = np.zeros(3)
        if start <= t <= end:
            w[axis] = .5*case['gust_peak']*(1-np.cos(2*np.pi*(t-start)/(end-start)))
        return w
    return wind


def run_trial(factory, label, profile, case, limits):
    """Fresh controller state per trial; nominal gains/solver may be cached."""
    nominal_hash = parameter_hash(factory.p)
    truth = perturb_params(factory.p, case['factors'])
    initial_v, initial_z, _ = profile.get_ref(0.)
    trim = find_trim(truth, float(initial_v[0]))
    x = trim['state'].copy()
    x[2] = initial_z
    # make() resets the NMPC solution/timing/history and creates a new INDI
    # inner loop. Cruise references and trim thrust are set before its first call.
    ctrl = factory.make(label, float(initial_v[0]), initial_z)
    solver = solver_of(ctrl)
    plant = AxialDronePlant(truth, dt=DT)
    wind = gust_wind(profile, case)
    ts, xs, us, winds = [0.], [x.copy()], [], []
    outside, reason = 0, None
    started = perf_counter()
    for k in range(round(profile.T_total/DT)):
        t = k*DT
        v, z, _ = profile.get_ref(t)
        w = wind(t) if wind else np.zeros(3)
        try:
            factory.update(ctrl, v, z)
            u = np.asarray(ctrl(t, x), dtype=float)
            if u.shape != (4,) or not np.all(np.isfinite(u)):
                reason = 'nonfinite or malformed motor command'
                break
            if np.any(u < truth['n_min']-1e-6) or np.any(u > truth['n_max']+1e-6):
                reason = 'command outside physical motor bounds'
                break
            if solver is not None and solver.consec_fail >= 5:
                reason = '5 consecutive optimizer failures'
                break
            xn = plant.step(x, u, w)
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            reason = f'{type(exc).__name__}: {exc}'
            break
        us.append(u.copy())
        winds.append(w)
        xs.append(xn.copy())
        ts.append((k+1)*DT)
        x = xn
        if not np.all(np.isfinite(x)):
            reason = 'nonfinite plant state'
            break
        vb = Rotation.from_quat(x[6:10]).as_matrix().T@(x[3:6]-w)
        outside += not domain_status(truth, x[13:17], vb)['inside_assumed_domain']
        if np.linalg.norm(x[10:13]) > limits.max_omega or abs(x[2]-z) > 50:
            reason = 'body-rate or altitude divergence stop'
            break
    ts, xs = np.asarray(ts), np.asarray(xs)
    vr, zr = profile.compute_refs(ts)
    result = dict(ts=ts, xs=xs, us=np.asarray(us).reshape(-1, 4),
                  v_refs=vr, z_refs=zr, wind=np.asarray(winds).reshape(-1, 3), error=reason)
    metrics = evaluate(result, profile, limits, truth['n_max'])
    log = deepcopy(solver.solve_log) if solver is not None else []
    metrics.update(tracking_pass=metrics['passed'],
                   model_domain_valid=outside == 0 and len(us) > 0 and bool(np.all(np.isfinite(xs))),
                   prop_domain_outside_fraction=outside/max(len(us), 1),
                   optimizer_calls=len(log),
                   optimizer_failures=sum(not entry['accepted'] for entry in log),
                   wall_seconds=perf_counter()-started, stop_reason=reason,
                   truth_parameter_sha256=parameter_hash(truth))
    if outside:
        metrics['passed'] = False
        metrics['failure_reasons'].append('propulsion_model_domain')
    if parameter_hash(factory.p) != nominal_hash:
        raise AssertionError('controller nominal parameters mutated during trial')
    return metrics, result, log


def summarize(rows, scenario):
    """Failures stay in the denominator; RMSE distributions use completions."""
    summary = {}
    for label in sorted({r['controller'] for r in rows}):
        selected = [r for r in rows if r['controller'] == label]
        count = len(selected)
        failures = sum(not r['passed'] for r in selected)
        record = dict(trials=count, passed=count-failures, failed=failures,
                      failure_rate=failures/count,
                      tracking_passed=sum(r.get('tracking_pass', False) for r in selected),
                      domain_valid=sum(r.get('model_domain_valid', False) for r in selected))
        # Wilson interval only for the independent random trials, not OAT/grid.
        if scenario == 'mc':
            z, p = 1.959963984540054, failures/count
            den = 1+z*z/count
            center = (p+z*z/(2*count))/den
            half = z*np.sqrt(p*(1-p)/count+z*z/(4*count*count))/den
            record['failure_rate_wilson95'] = [max(0., center-half), min(1., center+half)]
        complete = [r for r in selected if not r.get('stop_reason') and
                    'incomplete' not in r['failure_reasons'] and r.get('rmse_velocity') is not None]
        record['complete_metric_trials'] = len(complete)
        for key in ('rmse_z', 'rmse_velocity', 'max_omega'):
            values = [r[key] for r in complete if np.isfinite(r[key])]
            if values:
                record[key] = dict(mean=float(np.mean(values)), std=float(np.std(values)),
                                   p95=float(np.percentile(values, 95)), worst=float(max(values)))
        summary[label] = record
    return summary


def write_report(out, manifest, rows):
    summary = summarize(rows, manifest['scenario'])
    lines = [f'# Validation: {manifest["scenario"]}', '',
             f'Model: `{manifest["params"]["profile_id"]}`; planned duration: {manifest["duration"]:g} s.',
             '', 'Pass requires tracking/safety criteria AND propulsion model domain validity.',
             'Early-stop RMSE describes only the recorded prefix, not the full mission.', '',
             '| Case | Controller | Simulated s | Tracking | Model domain | Pass | Reasons |',
             '|---|---|---:|---|---|---|---|']
    for r in rows:
        lines.append(f'| {r["case_id"]} | {r["controller"]} | {r["simulated_seconds"]:.3f} | '
                     f'{r.get("tracking_pass", False)} | {r.get("model_domain_valid", False)} | '
                     f'{r["passed"]} | {", ".join(r["failure_reasons"])} |')
    lines += ['', '## Aggregate', '', '```json',
              json.dumps(json_safe(summary), indent=2), '```', '',
              'Uncertainty ranges are exploratory assumptions unless replaced with measured tolerances.',
              'Monte Carlo uses independent uniform factors; its interval is conditional on this distribution.',
              'No claim about untested conditions or real flight follows from these results.', '']
    (out/'REPORT.md').write_text('\n'.join(lines), encoding='utf-8')


def plot_traces(out, cases, labels, profile):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for case in cases:
        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        plotted = False
        for label in labels:
            path = out/f'{case["case_id"]}_{label}.npz'
            if not path.exists():
                continue
            with np.load(path) as trace:
                ts, xs = trace['ts'], trace['xs']
                display = 'NMPC + INDI (Split)' if label == 'Split' else label
                legend = display+(' [stopped]' if ts[-1] < profile.T_total-1e-8 else '')
                axes[0].plot(ts, xs[:, 3], label=legend)
                axes[1].plot(ts, np.linalg.norm(xs[:, 3:6]-trace['v_refs'], axis=1), label=legend)
                axes[2].plot(ts, xs[:, 2]-trace['z_refs'], label=legend)
                axes[3].plot(ts, np.linalg.norm(xs[:, 10:13], axis=1), label=legend)
                plotted = True
        if plotted:
            grid = np.linspace(0, profile.T_total, 1000)
            refs, _ = profile.compute_refs(grid)
            axes[0].plot(grid, refs[:, 0], 'k--', label='Reference')
            for ax, title in zip(axes, ['Forward speed [m/s]', 'Velocity error norm [m/s]',
                                       'Altitude error [m]', 'Body rate norm [rad/s]']):
                ax.set_ylabel(title)
                ax.grid(alpha=.25)
                ax.legend()
                ax.set_xlim(0, profile.T_total)
                if profile.gust_interval is not None and case.get('gust_peak', 0):
                    ax.axvspan(*profile.gust_interval, color='red', alpha=.12)
                for _, start, _ in profile.get_phase_boundaries():
                    ax.axvline(start, color='grey', alpha=.2)
            axes[-1].set_xlabel('Time [s]')
            fig.suptitle(f'Team light rocket | {case["case_id"]}')
            fig.tight_layout()
            fig.savefig(out/f'{case["case_id"]}.png', dpi=140)
        plt.close(fig)


def build_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', choices=['baseline', 'gust', 'sweep', 'mc'], default='baseline')
    p.add_argument('--controllers', nargs='+', choices=LABELS, default=['GS-LQR', 'Split'])
    p.add_argument('--speed', type=float, default=70.)
    p.add_argument('--altitude', type=float, default=50.)
    p.add_argument('--durations', type=float, nargs=5, default=list(DEFAULT_MISSION_DURATIONS),
                   metavar=('HOVER1', 'ACCEL', 'CRUISE', 'DECEL', 'HOVER2'))
    p.add_argument('--gust-settle', type=float, default=DEFAULT_GUST_TIMES[0])
    p.add_argument('--gust-duration', type=float, default=DEFAULT_GUST_TIMES[1])
    p.add_argument('--gust-recovery', type=float, default=DEFAULT_GUST_TIMES[2])
    p.add_argument('--gust-peak', type=float, default=2., help='m/s; signed peak allowed')
    p.add_argument('--gust-directions', nargs='+', choices=['vertical', 'lateral'],
                   default=['vertical', 'lateral'])
    p.add_argument('--ranges', type=Path, help='JSON mapping factor -> [min, max] multipliers')
    p.add_argument('--sweep-mode', choices=['oat', 'grid'], default='oat')
    p.add_argument('--points', type=int, default=3)
    p.add_argument('--trials', type=int, default=100)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--criteria', type=Path, help='JSON overrides for Acceptance fields')
    p.add_argument('--output', type=Path, default=ROOT/'results'/'validation', help='parent output directory')
    p.add_argument('--dry-run', action='store_true', help='save configuration/cases without simulation')
    p.add_argument('--save-traces', action='store_true', help='also save every sweep/MC trajectory')
    return p


def make_plan(args):
    if not np.isfinite(args.speed) or not 0 <= args.speed <= 85:
        raise ValueError('speed must be within the checked nominal trim range 0..85 m/s')
    if not np.isfinite(args.gust_peak):
        raise ValueError('gust peak must be finite')
    if args.trials < 1 or args.points < 2 or args.seed < 0:
        raise ValueError('trials >= 1, points >= 2 and seed >= 0 are required')
    ranges = validate_ranges(json.loads(args.ranges.read_text(encoding='utf-8'))
                             if args.ranges else deepcopy(DEFAULT_RANGES))
    limits = Acceptance(**(json.loads(args.criteria.read_text(encoding='utf-8')) if args.criteria else {}))
    if args.scenario == 'gust':
        profile = GustProfile(args.speed, args.altitude, args.gust_settle,
                              args.gust_duration, args.gust_recovery)
        if args.gust_settle < limits.recovery_hold or args.gust_recovery < limits.recovery_hold:
            raise ValueError('gust settling/recovery windows must cover recovery_hold')
        cases = [dict(case_id='cruise_control', factors={}, gust_peak=0.)]
        cases += [dict(case_id=f'gust_{direction}', factors={}, gust_direction=direction,
                       gust_peak=args.gust_peak) for direction in dict.fromkeys(args.gust_directions)]
    else:
        profile = MissionProfile(args.speed, args.altitude, args.durations)
        if args.scenario == 'sweep':
            if args.sweep_mode == 'grid' and args.points**len(ranges) > 100000:
                raise ValueError('grid exceeds 100000 cases; select fewer factors or use MC')
            cases = sweep_cases(ranges, args.points, args.sweep_mode)
        elif args.scenario == 'mc':
            cases = monte_carlo_cases(ranges, args.trials, args.seed)
        else:
            cases = [dict(case_id='nominal', factors={})]
    for _, start, duration, *_ in profile.phases:
        if abs(duration/DT-round(duration/DT)) > 1e-7:
            raise ValueError(f'phase durations must be multiples of {DT} seconds')
    return profile, cases, ranges, limits


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        profile, cases, ranges, limits = make_plan(args)
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    labels = list(dict.fromkeys(args.controllers))
    out = args.output/('run_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True, exist_ok=False)
    params = baseline_params()
    sources = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
               for folder in ('control', 'models/team_light/control') for p in (ROOT/folder).glob('*.py')}
    try:
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = None
    manifest = dict(scenario=args.scenario, controllers=labels, arguments=vars(args),
                    phases=profile.phases, duration=profile.T_total, gust_interval=profile.gust_interval,
                    params=params, parameter_sha256=parameter_hash(params),
                    uncertainty_ranges=ranges, sampling='independent uniform' if args.scenario == 'mc' else args.scenario,
                    criteria=asdict(limits), dt=DT, git_revision=revision,
                    source_sha256=sources, python=platform.python_version(),
                    packages={name: version(name) for name in ('numpy', 'scipy', 'casadi', 'matplotlib')},
                    controller_settings=dict(N=15, dt_prediction=.04, dt_control=.04, max_iter=80,
                                             future_reference_preview=False),
                    case_count=len(cases), expected_trials=len(cases)*len(labels),
                    status='planned' if args.dry_run else 'running')
    write_json(out/'manifest.json', manifest)
    write_json(out/'cases.json', cases)
    print(f'Results: {out}', flush=True)
    profile.print_profile()
    print(f'{len(cases)} cases x {len(labels)} controllers', flush=True)
    if args.dry_run:
        return out
    factory = Factory(params)
    rows = []
    for case in cases:
        for label in labels:
            print(f'Running {case["case_id"]} / {label}', flush=True)
            try:
                metrics, result, log = run_trial(factory, label, profile, case, limits)
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                metrics = dict(passed=False, tracking_pass=False, failure_reasons=['setup_error'],
                               stop_reason=f'{type(exc).__name__}: {exc}', simulated_seconds=0.)
                result, log = None, []
            row = dict(case, controller=label, **metrics)
            rows.append(row)
            tag = f'{case["case_id"]}_{label}'
            with (out/'trials.jsonl').open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(json_safe(row), ensure_ascii=False, allow_nan=False)+'\n')
            if log:
                write_json(out/(tag+'.solver.json'), log)
            if result is not None and (args.save_traces or args.scenario in ('baseline', 'gust')):
                np.savez_compressed(out/(tag+'.npz'), **{k: v for k, v in result.items() if k != 'error'})
            write_json(out/'summary.json', summarize(rows, args.scenario))
            print(f'  pass={row["passed"]}, t={row["simulated_seconds"]:g}s, '
                  f'reasons={row["failure_reasons"]}', flush=True)
    manifest.update(status='complete', recorded_trials=len(rows))
    write_json(out/'manifest.json', manifest)
    write_report(out, manifest, rows)
    if args.scenario in ('baseline', 'gust'):
        plot_traces(out, cases, labels, profile)
    print(json.dumps(json_safe(summarize(rows, args.scenario)), indent=2), flush=True)
    return out


if __name__ == '__main__':
    main()
