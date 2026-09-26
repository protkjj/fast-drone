"""스모크 참조(JSON) → 요약 문서(markdown). 결과는 **SMOKE**(파이프라인 확인, 우위 결론 없음).

예전 요약(results/arena/SMOKE_2026-09-26.md)은 셸에서 손으로 만들었다(heredoc이 백틱을 먹은 사고도
있었다, 1cb7132). 이 도구가 같은 표를 참조 파일에서 다시 만든다. 이전 참조를 주면 사례·제어기마다
전후를 나란히 놓고, 진단 V4 결과를 주면 M17 궤적 해시가 V4와 같은지도 본다.

실행: python -m control.arena_smoke_summary --out results/arena/SMOKE_<날짜>.md
        [--reference results/arena/smoke_reference.json] [--previous <옛 참조.json>]
        [--v4 results/arena/m17_diagnosis/V4.json]
"""
import argparse
import json
from pathlib import Path

from control.arena import ROOT


def _fmt(x, spec='.3g'):
    return '—' if x is None else format(x, spec)


def _solver(r):
    return '—' if r.get('optimizer_calls') in (None, 0) else f"{r['optimizer_failures']}/{r['optimizer_calls']}"


def _limit(r):
    integ = r.get('integrators') or {}
    return '—' if not integ else f"{100*integ['at_limit_fraction']:.1f}"


def rows_table(reference):
    lines = ['| 사례 | 제어기 | 시뮬 s | 스위트 판정 | 추종 | 도메인 | 논문 실패 | 창 RMSE v | 창 RMSE z | '
             '\\|ω\\|max | 솔버실패 | 적분 한계 % | 정지·사유 |',
             '|---|---|---:|---|---|---|---|---:|---:|---:|---:|---:|---|']
    for r in reference['rows']:
        if r.get('skipped'):
            lines.append(f"| {r['scenario_id']} | {r['controller']} | — | 제외 | — | — | — | — | — | — | — | — | "
                         f"{r.get('skip_reason', '')} |")
            continue
        lines.append(f"| {r['scenario_id']} | {r['controller']} | {r['simulated_seconds']:.2f} | {r['passed']} | "
                     f"{r['tracking_pass']} | {r['model_domain_valid']} | {r['paper_failed']} | "
                     f"{_fmt(r['window_rmse_velocity'])} | {_fmt(r['window_rmse_z'])} | {_fmt(r['max_omega'])} | "
                     f"{_solver(r)} | {_limit(r)} | {r['stop_reason'] or ''} |")
    return lines


def comparison_table(reference, previous):
    old = {(r['scenario_id'], r['controller']): r for r in previous['rows']}
    lines = ['| 사례 | 제어기 | 해시 동일 | 시뮬 s 전 → 후 | 창 RMSE v 전 → 후 | 창 RMSE z 전 → 후 | '
             '\\|ω\\|max 전 → 후 | 논문 실패 전 → 후 | 정지 전 → 후 |',
             '|---|---|---|---|---|---|---|---|---|']
    for r in reference['rows']:
        b = old.get((r['scenario_id'], r['controller']))
        if b is None or r.get('skipped') or b.get('skipped'):
            state = '새 사례' if b is None else '제외'
            lines.append(f"| {r['scenario_id']} | {r['controller']} | {state} | | | | | | |")
            continue
        lines.append(f"| {r['scenario_id']} | {r['controller']} | {b['trajectory_sha256'] == r['trajectory_sha256']} | "
                     f"{b['simulated_seconds']:.2f} → {r['simulated_seconds']:.2f} | "
                     f"{_fmt(b['window_rmse_velocity'])} → {_fmt(r['window_rmse_velocity'])} | "
                     f"{_fmt(b['window_rmse_z'])} → {_fmt(r['window_rmse_z'])} | "
                     f"{_fmt(b['max_omega'])} → {_fmt(r['max_omega'])} | {b['paper_failed']} → {r['paper_failed']} | "
                     f"{b['stop_reason'] or '끝까지'} → {r['stop_reason'] or '끝까지'} |")
    gone = sorted(set(old) - {(r['scenario_id'], r['controller']) for r in reference['rows']})
    if gone:
        lines += ['', '이전 참조에만 있는 행: ' + ', '.join(f'{s}/{c}' for s, c in gone)]
    return lines


def v4_check(reference, v4):
    rows = {(r['scenario_id'], r['controller']): r for r in reference['rows']}
    lines = ['| 사례 | 진단 V4 해시 | 스모크 M17 해시 | 동일 |', '|---|---|---|---|']
    for case, entry in v4['cases'].items():
        r = rows.get((case, 'M17'))
        mine = None if r is None else r['trajectory_sha256']
        lines.append(f"| {case} | `{entry['trajectory_sha256'][:12]}` | `{(mine or '—')[:12]}` | "
                     f"{mine == entry['trajectory_sha256']} |")
    return lines


def summarize(reference, previous=None, v4=None, previous_label='이전 참조', reference_path=None):
    env = reference.get('environment', {})
    tuned = reference.get('tuned')
    counts = dict(trials=sum(not r.get('skipped') for r in reference['rows']),
                  skipped=sum(bool(r.get('skipped')) for r in reference['rows']))
    # 사전값 참조는 verify가 대조하는 이름 그대로 적는다. 튜닝값 참조는 그 파일을 덮어쓸 수 없으므로
    # (validation_suite --tuned가 막는다) 실제 경로를 적는다.
    out_path = reference_path if tuned else 'results/arena/smoke_reference.json'
    lines = [f"# {'튜닝값 스모크' if tuned else '스모크'} 결과 — {reference['created_utc'][:10]} (SMOKE)", '',
             '**SMOKE** — 경기장 파이프라인이 모든 사례를 끝까지 도는지, 어디서 멈추는지 보는 실행이다. '
             '성능 비교가 아니며 우위·열위 결론을 내리지 않는다.', '',
             f"재현: `{reference['command']} --reference-out {out_path}` · "
             f"요약: `python -m control.arena_smoke_summary`", '']
    if tuned:
        lines += [f"게인: **튜닝값** — `{tuned['run_dir']}` 기록의 최선값(같은 예산 튜닝, PILOT). 기록 sha256: "
                  + ', '.join(f"{name} `{t['record_sha256'][:12]}`" for name, t in tuned['controllers'].items()), '']
    lines += [f"설정 sha256 `{reference['config_sha256'][:12]}` · git `{str(reference.get('git_revision'))[:8]}` "
             f"dirty={reference.get('git_dirty')} · {env.get('system', '')} {env.get('machine', '')} · "
             f"{env.get('cpu', '')} · Python {env.get('python', '')}", '',
             f"시행 {counts['trials']}개" + (f" + 설계 영역 밖 제외 {counts['skipped']}개(kj 결정 2026-09-26: "
                                           'CPID는 설계 영역 0~20 m/s 안의 V_L 사례만)' if counts['skipped'] else '')
             + '.', '',
             '열: 스위트 판정 = 추종·안전(Acceptance) AND 추진 모델 도메인 · 추종 = Acceptance만 · 도메인 = 추진 '
             '모델 가정 범위(매우 엄격) · 논문 실패 = §5.10 · 창 RMSE = §5.8 평가창 · 솔버실패 = 실패/호출 · '
             '적분 한계 % = GSLQR·CPID 적분기가 한계에 닿은 스텝 비율', '']
    lines += rows_table(reference)
    if previous is not None:
        lines += ['', f'## {previous_label} 대비', '',
                  f"{previous_label}: 설정 sha256 `{previous['config_sha256'][:12]}` · git "
                  f"`{str(previous.get('git_revision'))[:8]}`. 해시가 다르면 궤적이 바뀐 것이다.", '']
        lines += comparison_table(reference, previous)
    if v4 is not None:
        lines += ['', '## M17 — 진단 V4(명령 하한 = 양추력 회전수)와 같은 궤적인가', '']
        lines += v4_check(reference, v4)
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--reference', type=Path, default=ROOT/'results'/'arena'/'smoke_reference.json')
    parser.add_argument('--previous', type=Path, help='older smoke reference to compare with')
    parser.add_argument('--previous-label', default='이전 참조')
    parser.add_argument('--v4', type=Path, help='results/arena/m17_diagnosis/V4.json')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    load = lambda p: None if p is None else json.loads(Path(p).read_text(encoding='utf-8'))
    path = args.reference.resolve()
    shown = str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)
    text = summarize(load(args.reference), load(args.previous), load(args.v4), args.previous_label,
                     reference_path=shown)
    args.out.write_text(text, encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
