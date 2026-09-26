"""튜닝 경로 점검 — 최선값이 바뀐 평가마다 그 값으로 본시험 사례를 돌려, 어디서부터 무너지는지 본다. **PILOT**.

튜닝값 CPID가 튜닝 집합(저속 측풍 3·5 m/s)보다 센 본시험 사례(V_L 측풍 10 m/s)에서 실패한 것을 보고
만들었다(보고서 12절). 튜닝 목적함수가 나아지는 길(평가 0 = 사전값, 그 뒤 최선값이 갱신된 평가)을
따라가며 본시험 사례의 판정·창 RMSE·|ω|max를 나란히 놓는다. 튜닝 기록과 설정은 읽기만 한다.

본시험 사례를 튜닝 값 **고르기**에 쓰지 않는다 — 이미 끝난 튜닝의 경로를 사후에 점검할 뿐이다
(튜닝·본시험 분리, I-4). 이 표로 게인을 다시 고르면 본시험을 튜닝에 쓴 것이 된다.

실행: python -m control.arena_tuning_path_check --run-dir results/arena/tuning/main120 --controller CPID
        --cases gust_lateral_p10_VL gust_vertical_m5_VL        → results/arena/TUNING_PATH_<제어기>.md, .json
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from control.arena import load_config, config_sha256, build_scenarios, excluded_from, DEFAULT_CONFIG, ROOT


def improvements(log):
    """평가 0(사전값)과, 누적 최선이 엄밀히 좋아진 평가들(나침반 탐색이 중심을 옮긴 후보들)."""
    picked, best = [], None
    for entry in log:
        if best is None or entry['objective'] < best - 1e-12:
            picked.append(entry)
            best = entry['objective']
    return picked


def run(config, run_dir, label, case_ids):
    import control.validation_suite as suite
    from control.arena_factory import ArenaFactory
    from control.arena_tune import parameter_space
    from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
    from models.team_light.control.baseline_v2 import baseline_params
    record = json.loads((Path(run_dir)/f'{label}.record.json').read_text(encoding='utf-8'))
    if record.get('config_sha256') != config_sha256(config):
        raise SystemExit(f'{label}: tuned with config {str(record.get("config_sha256"))[:12]}, '
                         f'not the current {config_sha256(config)[:12]}')
    log = [json.loads(line) for line in (Path(run_dir)/f'{label}.jsonl').read_text(encoding='utf-8').splitlines()
           if line]
    native = baseline_params()
    base = ArenaFactory(config, native)
    scenarios = build_scenarios(config, base.cp, native, only=case_ids)
    refused = [f'{s.id}: {why}' for s in scenarios for why in [excluded_from(config, label, s)] if why]
    if refused:
        raise SystemExit(f'{label} is excluded from ' + '; '.join(refused))
    _, _, apply = parameter_space(config, label, base.gains)
    limits, paper = Acceptance(**config['acceptance']), PaperCriteria(**config['paper_criteria'])
    rows = []
    for entry in improvements(log):
        factory = ArenaFactory(config, native, overrides=apply(entry['values']))
        for s in scenarios:
            print(f"{label} evaluation {entry['index']} on {s.id}", flush=True)
            try:                                       # 스모크(run_arena)와 같은 처리 — 한 시행이 도구를 죽이지 않게
                metrics, result, solve_log = suite.run_trial(factory, label, s.profile, s.cases[0], limits)
                metrics.update(paper_evaluate(result, s.profile, paper, window=s.window, solve_log=solve_log,
                                              n_max=native['n_max']))
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                metrics = dict(passed=False, paper_failed=True, stop_reason=f'{type(exc).__name__}: {exc}')
            rows.append(dict(evaluation=entry['index'], tuning_objective=entry['objective'],
                             values=entry['values'], case=s.id,
                             **{k: metrics.get(k) for k in ('passed', 'paper_failed', 'stop_reason',
                                                            'window_rmse_velocity', 'window_rmse_z',
                                                            'max_omega', 'trajectory_sha256')}))
    return record, rows


def write_markdown(path, data):
    m = data['meta']
    f = lambda x, spec='.3g': '—' if x is None else format(x, spec)
    cases = m['cases']
    lines = [f"# 튜닝 경로 점검 — {m['controller']}(PILOT)", '',
             '**PILOT** — 이미 끝난 튜닝의 경로를 본시험 사례로 사후 점검한다. 게인을 다시 고르는 데 쓰지 않는다'
             '(쓰면 본시험을 튜닝에 쓴 것이 된다). 우위 결론 없음.', '',
             f"재현: `{m['command']}` · 설정 sha256 `{m['config_sha256'][:12]}` · git `{m['git_revision']}` "
             f"dirty={m['git_dirty']} · 튜닝 기록 `{m['run_dir']}`(튜닝 git `{str(m['tuning_git_revision'])[:8]}`)", '',
             '행 = 평가 0(사전값)과 튜닝 목적함수의 최선이 갱신된 평가. 칸 = 판정(통과/실패) · 창 RMSE v / z · '
             '\\|ω\\|max(rad/s).', '',
             '| 평가 | 튜닝 목적함수 | 바뀐 값(사전값 대비 배수) | ' + ' | '.join(cases) + ' |',
             '|---:|---:|---|' + '---|'*len(cases)]
    prior = data['prior']
    by = {(r['evaluation'], r['case']): r for r in data['rows']}
    for evaluation in dict.fromkeys(r['evaluation'] for r in data['rows']):
        first = by[(evaluation, cases[0])]
        changed = ', '.join(f"{k} ×{v/prior[k]:.4g}" for k, v in first['values'].items() if v != prior[k]) or '사전값'
        cells = []
        for c in cases:
            r = by[(evaluation, c)]
            verdict = '통과' if r['passed'] and not r['paper_failed'] else ('**실패**' + (' ·정지' if r['stop_reason'] else ''))
            cells.append(f"{verdict} · {f(r['window_rmse_velocity'])} / {f(r['window_rmse_z'])} · {f(r['max_omega'])}")
        lines.append(f"| {evaluation} | {first['tuning_objective']:.6g} | {changed} | " + ' | '.join(cells) + ' |')
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    from control.validation_suite import write_json, git_state
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--controller', required=True)
    parser.add_argument('--cases', nargs='+', required=True)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    tuning_ids = {s['id'] for s in config['tuning']['scenarios']}
    if tuning_ids & set(args.cases):
        parser.error('give main-test cases; tuning cases are already in the tuning log')
    record, rows = run(config, args.run_dir, args.controller, args.cases)
    revision, dirty = git_state()
    data = dict(meta=dict(label='PILOT', controller=args.controller, cases=list(args.cases),
                          command=f'python -m control.arena_tuning_path_check --run-dir {args.run_dir} '
                                  f'--controller {args.controller} --cases {" ".join(args.cases)}',
                          created_utc=datetime.now(timezone.utc).isoformat(), config_sha256=config_sha256(config),
                          git_revision=revision, git_dirty=dirty, run_dir=str(args.run_dir),
                          tuning_git_revision=record.get('git_revision'), tuning_status=record.get('status')),
                prior=record['prior'], rows=rows)
    args.out.mkdir(parents=True, exist_ok=True)
    write_json(args.out/f'tuning_path_{args.controller}.json', data)
    write_markdown(args.out/f'TUNING_PATH_{args.controller}.md', data)
    print((args.out/f'TUNING_PATH_{args.controller}.md').read_text(encoding='utf-8'))
    return data


if __name__ == '__main__':
    main()
