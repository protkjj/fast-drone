"""임무 ρ(t) — 참조 가속이 가용 가·감속을 넘는 구간(ρ > 1)과 나머지 구간의 지표를 나눈다. **SMOKE**(결론 없음).

kj 결정(2026-09-26 저녁): 임무 감속 프로필(15/15/15 s, 확정 사항)은 그대로 둔다. 대신 임무 ρ(t)를
기록하고, ρ > 1 구간과 나머지 구간의 지표를 따로 보고한다. 모든 제어기가 같은 참조를 받는다.
ρ > 1이란 명목 모델로도 낼 수 없는 가·감속을 요구하는 구간이라는 뜻이다.

  ρ(t)     |a_ref(t)| / a_avail(v_ref(t), 방향)
           참조 프로필의 ρ(논문 §5.5)와 같은 정의를 시각마다 쓴 것이다.
  a_ref    경기장 참조 창과 같은 전진 차분이다(NMPC 예측 격자의 첫 간격 Δt_pred).
  a_avail  명목 모델이 추력 ≥ 0·역유입 없이 그 속도·방향으로 낼 수 있는 최대 가·감속이다
           (arena.acceleration_table, 0~85 m/s를 5 m/s 격자로 잡고 선형 보간 — 참조 프로필과 같은 간격).
  구간     평가 창(임무 3 s부터) ∩ 시행이 살아 있던 구간을 ρ > 1과 나머지로 가른다.

스모크 실행 폴더의 궤적(npz)을 읽기만 하고 다시 시뮬레이션하지 않는다. 같은 (사례, 제어기)가
여러 실행에 있으면 뒤에 준 실행을 쓴다.

실행: python -m control.arena_mission_rho --runs <스모크 실행 폴더 …> [--out results/arena]
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from control.arena import load_config, config_sha256, build_scenarios, acceleration_table, DEFAULT_CONFIG, ROOT

GRID = np.arange(0.0, 85.0 + 1e-9, 5.0)


def rho_series(profile, ts, step, tables):
    """시각마다 (ρ, a_ref, v_ref). a_ref = 0이면 ρ = 0이다."""
    v = np.array([profile.get_ref(t)[0][0] for t in ts])
    a = (np.array([profile.get_ref(t + step)[0][0] for t in ts]) - v)/step
    avail = np.where(a < 0, np.interp(v, GRID, tables['brake']), np.interp(v, GRID, tables['accel']))
    with np.errstate(divide='ignore', invalid='ignore'):
        rho = np.where(a == 0.0, 0.0, np.abs(a)/avail)
    return rho, a, v


def spans(ts, mask):
    edges = np.flatnonzero(np.diff(np.r_[False, mask, False].astype(int)))
    return [(float(ts[a]), float(ts[b - 1])) for a, b in zip(edges[::2], edges[1::2])]


def part_metrics(xs, v_refs, z_refs, mask, dt, total_seconds):
    flown = float(mask.sum()*dt)
    if not mask.any():
        return dict(seconds=0.0, of_seconds=total_seconds, rmse_v=None, rmse_z=None, max_z_error=None,
                    max_omega=None)
    ev, ez = xs[mask, 3:6] - v_refs[mask], xs[mask, 2] - z_refs[mask]
    return dict(seconds=flown, of_seconds=total_seconds,
                rmse_v=float(np.sqrt(np.mean(np.sum(ev**2, axis=1)))), rmse_z=float(np.sqrt(np.mean(ez**2))),
                max_z_error=float(np.max(np.abs(ez))),
                max_omega=float(np.max(np.linalg.norm(xs[mask, 10:13], axis=1))))


def analyse(config, runs, factory):
    step, dt = float(config['nmpc_common']['dt_pred_s']), float(config['plant']['dt_s'])
    tables = {name: acceleration_table(factory.cp, GRID, sign)[1] for name, sign in (('accel', 1.0), ('brake', -1.0))}
    trials = {}
    for run in runs:                                   # 뒤에 준 실행이 앞의 같은 (사례, 제어기)를 덮는다
        for line in (Path(run)/'trials.jsonl').read_text(encoding='utf-8').splitlines():
            row = json.loads(line)
            if not row.get('skipped'):
                trials[(row['scenario_id'], row['controller'])] = (Path(run), row)
    out = []
    for s in build_scenarios(config, factory.cp, factory.p):
        if s.type != 'mission':
            continue
        grid = np.arange(0.0, s.profile.T_total, dt)
        rho_full, _, _ = rho_series(s.profile, grid, step, tables)
        window = (grid >= s.window[0]) & (grid <= s.window[1])
        high_total = float(np.sum(window & (rho_full > 1.0))*dt)
        rest_total = float(np.sum(window & (rho_full <= 1.0))*dt)
        entry = dict(case=s.id, rho_peak=float(np.max(rho_full)), rho_gt1_spans=spans(grid, window & (rho_full > 1.0)),
                     rho_gt1_seconds=high_total, rest_seconds=rest_total, controllers=[])
        for label in config['controllers']:
            hit = trials.get((s.id, label))
            if hit is None:
                continue
            run, row = hit
            data = np.load(run/f'{s.id}_{label}.npz')
            ts, xs, v_refs, z_refs = data['ts'], data['xs'], data['v_refs'], data['z_refs']
            rho, _, _ = rho_series(s.profile, ts, step, tables)
            live = (ts >= s.window[0]) & (ts <= s.window[1])
            entry['controllers'].append(dict(
                controller=label, run=run.name, stop_reason=row['stop_reason'],
                simulated_seconds=row['simulated_seconds'], trajectory_sha256=row['trajectory_sha256'],
                rho_gt1=part_metrics(xs, v_refs, z_refs, live & (rho > 1.0), dt, high_total),
                rest=part_metrics(xs, v_refs, z_refs, live & (rho <= 1.0), dt, rest_total)))
        out.append(entry)
    return out, tables


def plot(config, factory, tables, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    step, dt = float(config['nmpc_common']['dt_pred_s']), float(config['plant']['dt_s'])
    s = next(s for s in build_scenarios(config, factory.cp, factory.p) if s.type == 'mission')
    ts = np.arange(0.0, s.profile.T_total, dt)
    rho, a, v = rho_series(s.profile, ts, step, tables)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(ts, rho, lw=1.2, label='rho(t) = |a_ref| / a_avail')
    ax.axhline(1.0, color='k', lw=0.8, ls='--')
    ax.fill_between(ts, 0, np.minimum(rho, 13), where=rho > 1.0, color='tab:red', alpha=0.2,
                    label='rho > 1 (reference beyond nominal authority)')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('rho [-]')
    ax.set_ylim(0, 13)
    ax2 = ax.twinx()
    ax2.plot(ts, v, color='tab:gray', lw=0.8, label='v_ref')
    ax2.set_ylabel('v_ref [m/s]')
    ax.set_title(f'{s.id} (same reference for all mission cases): rho(t) and reference speed')
    lines = ax.get_legend_handles_labels()
    lines2 = ax2.get_legend_handles_labels()
    ax.legend(lines[0] + lines2[0], lines[1] + lines2[1], loc='upper left', fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def write_markdown(path, data):
    m = data['meta']
    f = lambda x, spec='.3g': '—' if x is None else format(x, spec)
    tuned = m.get('tuned')
    lines = [f"# 임무 ρ(t) — ρ > 1 구간과 나머지 구간({'튜닝값 ' if tuned else ''}SMOKE)", '',
             '**SMOKE** — kj 결정(2026-09-26 저녁): 임무 감속 프로필은 유지하고 ρ > 1 구간을 분리해 보고한다. 우위 결론 없음.', '',
             f'재현: `{m["command"]}` · 설정 sha256 `{m["config_sha256"][:12]}` · git `{m["git_revision"]}` '
             f'dirty={m["git_dirty"]} · 스모크 실행 {", ".join(m["runs"])}', '']
    if tuned:
        lines += [f"게인: **튜닝값** — `{tuned['run_dir']}` 기록의 최선값. 기록 sha256: "
                  + ', '.join(f'{name} `{sha[:12]}`' for name, sha in tuned['record_sha256'].items()), '']
    lines += ['ρ(t) = |a_ref| / a_avail(v_ref, 방향). a_ref는 경기장 창과 같은 0.05 s 전진 차분이다. a_avail은 명목 모델이 '
             '추력 ≥ 0·역유입 없이 낼 수 있는 최대 가·감속(참조 프로필 ρ와 같은 정의)이다. 구간은 평가 창(3 s부터) 안에서 '
             '나눈다. 칸의 "비행 s / 전체 s"는 그 구간 중 시행이 멈추기 전까지 난 시간이다.', '',
             '![mission rho](mission_rho.png)', '']
    for e in data['cases']:
        spans_text = ', '.join(f'{a:.1f}–{b:.1f}' for a, b in e['rho_gt1_spans']) or '없음'
        lines += [f"## {e['case']}", '',
                  f"ρ > 1 구간: {spans_text} s(평가 창 안 {e['rho_gt1_seconds']:.1f} s, 나머지 {e['rest_seconds']:.1f} s), "
                  f"ρ 최대 {e['rho_peak']:.2f}", '',
                  '| 제어기 | 정지·시간 s | ρ > 1: 비행 s / 전체 s | RMSE v | RMSE z | 최대 고도오차 | \\|ω\\|max | '
                  '나머지: 비행 s / 전체 s | RMSE v | RMSE z | 최대 고도오차 | \\|ω\\|max |',
                  '|---|---|---|---:|---:|---:|---:|---|---:|---:|---:|---:|']
        for c in e['controllers']:
            h, r = c['rho_gt1'], c['rest']
            stop = '끝까지' if c['stop_reason'] is None else f"{c['simulated_seconds']:.2f} 정지"
            lines.append(f"| {c['controller']} | {stop} | {h['seconds']:.1f} / {h['of_seconds']:.1f} | {f(h['rmse_v'])} | "
                         f"{f(h['rmse_z'])} | {f(h['max_z_error'])} | {f(h['max_omega'])} | "
                         f"{r['seconds']:.1f} / {r['of_seconds']:.1f} | {f(r['rmse_v'])} | {f(r['rmse_z'])} | "
                         f"{f(r['max_z_error'])} | {f(r['max_omega'])} |")
        lines.append('')
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    from control.arena_factory import ArenaFactory
    from control.validation_suite import write_json, git_state
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--runs', nargs='+', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    tuned_runs = []
    for run in args.runs:
        manifest = json.loads((run/'manifest.json').read_text(encoding='utf-8'))
        if manifest['config_sha256'] != config_sha256(config):
            raise SystemExit(f'{run}: smoke run used config {manifest["config_sha256"][:12]}, '
                             f'not the current {config_sha256(config)[:12]}')
        tuned_runs.append(manifest.get('tuned'))
    # 한 표 안에서 게인 출처가 섞이면 안 된다 — 사전값 실행과 튜닝값 실행, 또는 다른 튜닝 기록끼리
    if len({None if t is None else t['run_dir'] for t in tuned_runs}) > 1:
        raise SystemExit('runs mix gain sources (untuned vs tuned, or different tuning run-dirs)')
    extra = {} if tuned_runs[0] is None else dict(tuned=dict(
        run_dir=tuned_runs[0]['run_dir'],
        record_sha256={name: t['record_sha256'] for block in tuned_runs for name, t in block['controllers'].items()}))
    factory = ArenaFactory(config)
    cases, tables = analyse(config, args.runs, factory)
    revision, dirty = git_state()
    data = dict(meta=dict(label='SMOKE', command='python -m control.arena_mission_rho --runs '
                          + ' '.join(str(r) for r in args.runs)
                          + ('' if args.out == ROOT/'results'/'arena' else f' --out {args.out}'),
                          created_utc=datetime.now(timezone.utc).isoformat(), config_sha256=config_sha256(config),
                          git_revision=revision, git_dirty=dirty, runs=[r.name for r in args.runs], **extra),
                a_avail=dict(speeds=GRID.tolist(), **{k: v.tolist() for k, v in tables.items()}), cases=cases)
    args.out.mkdir(parents=True, exist_ok=True)
    write_json(args.out/'mission_rho.json', data)
    plot(config, factory, tables, args.out/'mission_rho.png')
    write_markdown(args.out/'MISSION_RHO.md', data)
    print((args.out/'MISSION_RHO.md').read_text(encoding='utf-8'))
    return data


if __name__ == '__main__':
    main()
