"""E3 analysis of D1·D6 isolation trials (no new simulation).

Usage: python3 e_analyze.py <tree_root> <e_iso_dir> <out_md>
Domain check replicates control/validation_suite.py:264-265: after each step, truth rotor speed and
truth airspeed (wind of that step) through propeller_curve.domain_status with nominal parameters.
"""
import glob
import json
import os
import sys

import numpy as np

WINDOWS = ((0.0, 0.5), (0.5, 3.0), (3.0, 1e9))


def main():
    root, base, out = sys.argv[1:4]
    sys.path.insert(0, root)
    from scipy.spatial.transform import Rotation
    from models.team_light.control.baseline_v2 import baseline_params
    from models.team_light.control.propeller_curve import domain_status
    p = baseline_params()
    rows = []
    for trial_dir in sorted(glob.glob(os.path.join(base, '*', '*_s*'))):
        if not os.path.isdir(trial_dir):
            continue
        group = os.path.basename(os.path.dirname(trial_dir))
        ctrl, seed = os.path.basename(trial_dir).rsplit('_s', 1)
        jl = glob.glob(os.path.join(trial_dir, 'arena_*', 'trials.jsonl'))
        npz = glob.glob(os.path.join(trial_dir, 'arena_*', '*.npz'))
        if not jl or not npz:
            rows.append(dict(group=group, ctrl=ctrl, seed=int(seed), status='missing'))
            continue
        verdict = json.loads(open(jl[0], encoding='utf-8').readline())
        d = np.load(npz[0], allow_pickle=False)
        ts, xs, wind = d['ts'], d['xs'], d['wind']
        n = len(wind)
        inside = np.ones(n, dtype=bool)
        for k in range(n):
            x = xs[k+1]
            vb = Rotation.from_quat(x[6:10]).as_matrix().T @ (x[3:6]-wind[k])
            inside[k] = domain_status(p, x[13:17], vb)['inside_assumed_domain']
        t_end = ts[1:n+1]
        rpm_min = xs[1:n+1, 13:17].min(axis=1)*60/(2*np.pi)
        first = float(t_end[~inside][0]) if (~inside).any() else None
        row = dict(group=group, ctrl=ctrl, seed=int(seed), status='ok',
                   passed=verdict.get('passed'), tracking=verdict.get('tracking_pass'),
                   domain=verdict.get('model_domain_valid'), reasons=verdict.get('failure_reasons'),
                   outside_frac_recorded=verdict.get('prop_domain_outside_fraction'),
                   outside_frac_recomputed=float((~inside).mean()), first_exit_s=first)
        for lo, hi in WINDOWS:
            m = (t_end > lo) & (t_end <= hi)
            key = f'{lo:g}-{hi:g}' if hi < 1e8 else f'{lo:g}-'
            row[f'exits_{key}'] = int((~inside & m).sum())
            row[f'minrpm_{key}'] = float(rpm_min[m].min()) if m.any() else None
        row['trim_rpm_min'] = float(xs[0, 13:17].min()*60/(2*np.pi))
        row['first_cmd_rpm_min'] = float(np.min(d['us'][0])*60/(2*np.pi)) if len(d['us']) else None
        if 'indi_path' in d.files:
            paths = [str(s) for s in d['indi_path']]
            row['indi_fallback_steps'] = sum(s.startswith('fallback') for s in paths)
            row['indi_first_path'] = paths[0] if paths else None
        if 'xs_est' in d.files and first is not None:
            k = int(np.argmax(~inside)) + 1
            e = d['xs_est'][k]-xs[k]
            att = Rotation.from_quat(d['xs_est'][k][6:10]).inv()*Rotation.from_quat(xs[k][6:10])
            row['est_err_at_first_exit'] = dict(pos=float(np.linalg.norm(e[0:3])), vel=float(np.linalg.norm(e[3:6])),
                                                att_deg=float(np.degrees(att.magnitude())),
                                                omega=float(np.linalg.norm(e[10:13])),
                                                rotor_rpm=float(np.abs(e[13:17]).max()*60/(2*np.pi)))
        rows.append(row)
    with open(out.replace('.md', '.json'), 'w', encoding='utf-8') as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)
    lines = ['| group | ctrl | seed | pass | track | domain | first exit s | exits 0-0.5 / 0.5-3 / 3- | min RPM 0-0.5 / 0.5-3 / 3- | 1st cmd min RPM | INDI fallback steps |',
             '|---|---|---:|---|---|---|---:|---|---|---:|---:|']
    for r in rows:
        if r['status'] != 'ok':
            lines.append(f"| {r['group']} | {r['ctrl']} | {r['seed']} | MISSING |||||||||")
            continue
        fmt = lambda v: '—' if v is None else f'{v:.0f}'
        lines.append(f"| {r['group']} | {r['ctrl']} | {r['seed']} | {r['passed']} | {r['tracking']} | {r['domain']} | "
                     f"{'—' if r['first_exit_s'] is None else f'{r['first_exit_s']:.3f}'} | "
                     f"{r['exits_0-0.5']} / {r['exits_0.5-3']} / {r['exits_3-']} | "
                     f"{fmt(r['minrpm_0-0.5'])} / {fmt(r['minrpm_0.5-3'])} / {fmt(r['minrpm_3-'])} | "
                     f"{fmt(r['first_cmd_rpm_min'])} | {r.get('indi_fallback_steps', '—')} |")
    with open(out, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines)+'\n')
    print(f'{len(rows)} trials -> {out}')


if __name__ == '__main__':
    main()
