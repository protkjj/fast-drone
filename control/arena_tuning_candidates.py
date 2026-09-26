"""공통 튜닝 집합의 저속 사례 고르기 — kj 결정 6-10(2026-09-26). 결과는 **PILOT**.

문제("CPID 고정 벌점"): 옛 튜닝 집합 5개 중 CPID 설계 영역(0~20 m/s) 안의 사례는 측풍 3 m/s @10
하나뿐이었다. 나머지 4개에서 CPID는 게인과 무관하게 벌점(1000)을 받아, 24회 PILOT에서 목적함수가
800.019 → 800.011로만 움직였다(튜닝 신호가 거의 없음). kj 결정: 공통 튜닝 집합에 저속 사례를
더한다. 전 제어기가 같은 집합을 쓴다.

선택 규칙(실행 전에 계획서에 적음):
  후보마다 세기 사다리 [1단계, 2단계]를 둔다. 사전 게인 CPID(경기장 설정 그대로 — 기울기 45°)가
  1단계를 실패 없이 끝내면 채택한다. 실패하면 2단계(돌풍 −2 m/s, ρ 절반)로 내리고, 그래도
  실패하면 뺀다. 실패 = 튜닝 목적함수와 같은 정의(stop_reason 또는 paper_failed).
  CPID로 고르는 이유: 저속 사례를 넣는 목적이 CPID에 튜닝 신호를 주는 것이기 때문이다.
  다른 네 종은 이 속도가 설계 영역 안이다.

--controllers 로 고른 사례에서 다른 제어기(사전 게인)도 돌려 본다. 선택에는 쓰지 않는 정보다.
새 사례에서 어느 제어기가 사전 게인으로 실패하면, 그 제어기도 튜닝에서 같은 고정 벌점 문제를
겪는다는 뜻이라 보고서의 '결정 필요' 후보가 된다.

실행: python -m control.arena_tuning_candidates [--controllers V13 M17 F13 GSLQR] [--out results/arena]
"""
import argparse
from datetime import datetime, timezone
from pathlib import Path

from control.arena import load_config, config_sha256, build_scenarios, DEFAULT_CONFIG, ROOT
from control.arena_factory import ArenaFactory, ARENA_LABELS

# 사다리의 1단계가 configs/arena.json tuning.scenarios에 들어간 것과 같아야 한다(main이 대조한다).
LADDERS = (
    ('측풍 @15', (
        dict(id='tune_gust_lateral_p5_V15', type='gust', speed=15.0, direction='lateral', peak_m_s=5.0,
             times_s=[1.0, 1.0, 2.0]),
        dict(id='tune_gust_lateral_p3_V15', type='gust', speed=15.0, direction='lateral', peak_m_s=3.0,
             times_s=[1.0, 1.0, 2.0]))),
    ('수직풍 @20', (
        dict(id='tune_gust_vertical_p3_V20', type='gust', speed=20.0, direction='vertical', peak_m_s=3.0,
             times_s=[1.0, 1.0, 2.0]),
        dict(id='tune_gust_vertical_p1_V20', type='gust', speed=20.0, direction='vertical', peak_m_s=1.0,
             times_s=[1.0, 1.0, 2.0]))),
    ('가속 0→15', (
        {'id': 'tune_ramp_0_15_rho0.1', 'type': 'reference', 'from': 0.0, 'to': 15.0, 'rho': 0.1,
         'lead_s': 0.5, 'tail_s': 1.0},
        {'id': 'tune_ramp_0_15_rho0.05', 'type': 'reference', 'from': 0.0, 'to': 15.0, 'rho': 0.05,
         'lead_s': 0.5, 'tail_s': 1.0})),
)
SELECTOR = 'CPID'


def evaluate(config, factory, label, spec):
    """사례 하나를 튜닝 목적함수(arena_tune.Evaluator)와 같은 방식으로 돌린다."""
    import control.validation_suite as suite
    from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
    scenario = build_scenarios(config, factory.cp, factory.p, scenarios=[spec])[0]
    row, result, log = suite.run_trial(factory, label, scenario.profile, scenario.cases[0],
                                       Acceptance(**config['acceptance']))
    paper = paper_evaluate(result, scenario.profile, PaperCriteria(**config['paper_criteria']),
                           window=scenario.window, solve_log=log, n_max=factory.p['n_max'])
    failed = bool(row['stop_reason']) or paper['paper_failed']
    penalty = float(config['tuning']['objective']['failure_penalty'])
    score = (penalty if failed or paper['window_rmse_velocity'] is None
             else paper['window_rmse_velocity'] + paper['window_rmse_z'])
    integ = row.get('integrators') or {}
    return dict(controller=label, id=spec['id'], duration_s=float(scenario.profile.T_total),
                simulated_seconds=row['simulated_seconds'], failed=failed, stop_reason=row['stop_reason'],
                paper_reasons=paper['paper_reasons'], window_rmse_velocity=paper['window_rmse_velocity'],
                window_rmse_z=paper['window_rmse_z'], max_omega=row['max_omega'], score=float(score),
                at_limit_fraction=integ.get('at_limit_fraction'),
                trajectory_sha256=row['trajectory_sha256'])


def select(config, factory):
    """미리 정한 규칙: 사다리마다 실패 없이 끝낸 첫 단계를 고른다(없으면 뺀다)."""
    picks = []
    for name, ladder in LADDERS:
        tried, chosen = [], None
        for spec in ladder:
            tried.append(evaluate(config, factory, SELECTOR, spec))
            print(_line(tried[-1]), flush=True)
            if not tried[-1]['failed']:
                chosen = spec
                break
        picks.append(dict(ladder=name, tried=tried, chosen=chosen))
    return picks


def _line(r):
    return (f"{r['controller']:5s} {r['id']:26s} T={r['duration_s']:5.2f}s failed={r['failed']} "
            f"stop={r['stop_reason']} paper={r['paper_reasons']} v {r['window_rmse_velocity']} "
            f"z {r['window_rmse_z']} |w|max {r['max_omega']:.3g}")


def config_agreement(config, picks):
    """고른 사례가 configs/arena.json tuning.scenarios에 그대로 들어가 있는지."""
    tuning = {s['id']: s for s in config['tuning']['scenarios']}
    return {p['chosen']['id']: tuning.get(p['chosen']['id']) == p['chosen']
            for p in picks if p['chosen'] is not None}


def write_markdown(path, data):
    m = data['meta']
    fmt = lambda x, spec='.4g': '—' if x is None else format(x, spec)
    lines = ['# 공통 튜닝 집합 — 저속 사례 고르기(PILOT)', '',
             '**PILOT** — kj 결정 6-10: 공통 튜닝 집합에 저속 사례를 더한다. 우위·열위 결론 없음.', '',
             f'재현: `{m["command"]}` · 설정 sha256 `{m["config_sha256"][:12]}` · git `{m["git_revision"]}` '
             f'dirty={m["git_dirty"]}', '',
             '선택 규칙(미리 정함): 사전 게인 CPID(경기장 설정 — 기울기 45°)가 1단계를 실패 없이 끝내면 채택, '
             '실패하면 2단계(돌풍 −2 m/s, ρ 절반), 그래도 실패하면 뺀다. 실패 = stop_reason 또는 paper_failed'
             '(튜닝 목적함수와 같은 정의). 점수 = 창 RMSE v + z(실패면 벌점 '
             f'{data["failure_penalty"]:g}).', '',
             '| 사다리 | 제어기 | 사례 | 길이 s | 실패 | 정지 | 논문 사유 | 창 RMSE v | 창 RMSE z | 점수 | '
             '\\|ω\\|max | 적분 한계 도달 % |',
             '|---|---|---|---:|---|---|---|---:|---:|---:|---:|---:|']
    rows = [(p['ladder'], r) for p in data['picks'] for r in p['tried']]
    rows += [(next(p['ladder'] for p in data['picks'] if p['chosen'] and p['chosen']['id'] == r['id']), r)
             for r in data['others']]
    for ladder, r in rows:
        lim = '—' if r['at_limit_fraction'] is None else f'{100*r["at_limit_fraction"]:.2f}'
        lines.append(f'| {ladder} | {r["controller"]} | {r["id"]} | {r["duration_s"]:.2f} | {r["failed"]} | '
                     f'{r["stop_reason"] or "끝까지"} | {", ".join(r["paper_reasons"]) or "—"} | '
                     f'{fmt(r["window_rmse_velocity"])} | {fmt(r["window_rmse_z"])} | {fmt(r["score"])} | '
                     f'{r["max_omega"]:.3g} | {lim} |')
    lines += ['', '## 선택', '']
    for p in data['picks']:
        pick = p['chosen']['id'] if p['chosen'] else '뺌(두 단계 모두 실패)'
        lines.append(f'- {p["ladder"]}: {pick}')
    lines += ['', '설정 대조(고른 사례가 `configs/arena.json` tuning.scenarios에 같은 명세로 있는가): '
              + ', '.join(f'{k} {v}' for k, v in data['config_agreement'].items())]
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    from control.validation_suite import write_json, environment_fingerprint, git_state
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    parser.add_argument('--controllers', nargs='*', default=[], choices=[c for c in ARENA_LABELS if c != SELECTOR],
                        help='also run these controllers (prior gains) on the chosen cases; information only')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    factory = ArenaFactory(config)
    picks = select(config, factory)
    others = []
    for label in args.controllers:
        for p in picks:
            if p['chosen'] is not None:
                others.append(evaluate(config, factory, label, p['chosen']))
                print(_line(others[-1]), flush=True)
    revision, dirty = git_state()
    command = 'python -m control.arena_tuning_candidates' + (
        ' --controllers ' + ' '.join(args.controllers) if args.controllers else '')
    data = dict(meta=dict(label='PILOT', command=command, created_utc=datetime.now(timezone.utc).isoformat(),
                          config_sha256=config_sha256(config), git_revision=revision, git_dirty=dirty,
                          controller_model_sha256=factory.controller_model_sha256,
                          environment=environment_fingerprint()),
                failure_penalty=float(config['tuning']['objective']['failure_penalty']),
                picks=picks, others=others, config_agreement=config_agreement(config, picks))
    args.out.mkdir(parents=True, exist_ok=True)
    write_json(args.out/'tuning_candidates.json', data)
    write_markdown(args.out/'TUNING_CANDIDATES.md', data)
    print((args.out/'TUNING_CANDIDATES.md').read_text(encoding='utf-8'))
    return data


if __name__ == '__main__':
    main()
