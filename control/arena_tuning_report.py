"""튜닝 기록 요약 — `arena_tune`이 남긴 기록을 **읽기만** 해서 사례별 점수와 탐색 상태를 낸다. **PILOT**(우위 결론 없음).

`arena_tune --summarize`(I-4·적분기 1% 규칙)를 보완한다. 학습 곡선과 '90% 도달' 정의(A/B/C)는
`protkjj/arena-analysis` 브랜치의 `control/tuning_analysis.py` 몫이라 여기서는 하지 않는다.

  사례별 점수     사전값(평가 0)과 최선 평가의 사례별 점수·실패·|ω|max.
  상수 벌점 사례  새로 돌린 모든 평가(2회 이상)에서 실패한 사례다. 목적함수에 같은 벌점을 더할 뿐 탐색에
                 신호를 주지 않는다(예: CPID의 설계 영역 밖 사례). 이 사례를 뺀 '신호 사례' 평균을 따로 적는다.
  탐색 상태      기록된 목적함수로 `arena_tune.compass_search`를 그대로 다시 돌려(재생) 폴마다 중심·보폭을
                 얻는다. 탐색은 결정적이라 재생한 지수가 로그와 하나라도 다르면 기록이 어긋난 것이다(거부).
                 보폭이 한 번도 줄지 않았으면 수렴하기 전에 예산이 끝났다는 뜻이다.
  실패 평가      상수 벌점 사례 밖에서 실패한 평가다. 폴 중심에서 어느 파라미터를 몇 배 바꿨는지,
                 새로 실패한 사례와 그 |ω|max를 적는다(최선값 옆의 실패 절벽을 보려고).

돌고 있는 튜닝의 기록도 읽는다(그 제어기는 PARTIAL로 표시한다). 기록 파일은 바꾸지 않는다.

실행: python -m control.arena_tuning_report --run-dir results/arena/tuning/main120
      → results/arena/TUNING_main120.md, results/arena/tuning_main120.json
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from control.arena import load_config, config_sha256, integrator_limit_flags, DEFAULT_CONFIG, ROOT
from control.arena_tune import compass_search, check_tuning_records, _evaluation_entries


class ReplayMismatch(RuntimeError):
    """기록이 탐색 재생과 다르다 — 다른 코드·설정으로 만든 기록이거나 줄이 빠졌다."""


def load_run(run_dir, labels):
    """{제어기: (record, log)}. 기록이 없는 제어기는 뺀다."""
    runs = {}
    for label in labels:
        record_path, log_path = Path(run_dir)/f'{label}.record.json', Path(run_dir)/f'{label}.jsonl'
        if record_path.exists() and log_path.exists():
            log = [json.loads(line) for line in log_path.read_text(encoding='utf-8').splitlines() if line]
            runs[label] = (json.loads(record_path.read_text(encoding='utf-8')), log)
    return runs


def replay(record, log):
    """기록된 목적함수로 나침반 탐색을 다시 돌린다.

    반환: (평가마다 (폴 번호, 폴 중심, 보폭 지수) — 평가 0은 시작점이라 (None, None, None),
           탐색 정보, 최선 지수, 최선 값)
    """
    calls = []

    def evaluate(e):
        i = len(calls)
        if i >= len(log) or tuple(log[i]['exponents']) != tuple(e):
            logged = log[i]['exponents'] if i < len(log) else 'nothing'
            raise ReplayMismatch(f"{record['controller']}: evaluation {i} replays as {list(e)}, log has {logged}")
        calls.append(e)
        return log[i]['objective']

    search = record['search']
    center, best, info = compass_search(len(record['parameter_names']), len(log), evaluate,
                                        initial_step=float(search['initial_step']),
                                        min_step=float(search['min_step']))
    owner = [(None, None, None)]
    for k, poll in enumerate(info['polls']):
        owner += [(k, poll['center'], poll['step_exponent'])]*poll['evaluated']
    return owner, info, center, best


def analyse(record, log):
    """한 제어기의 기록을 요약한다(파일은 읽지 않는다 — 테스트가 합성 기록을 넣는다)."""
    label, names = record['controller'], record['parameter_names']
    owner, info, center, best = replay(record, log)
    if record.get('status') == 'complete' and (
            [float(v) for v in center] != [float(v) for v in record['best_exponents']]
            or best != record['best_objective']):
        raise ReplayMismatch(f'{label}: replayed best {list(center)} = {best} differs from the record '
                             f"{record['best_exponents']} = {record['best_objective']}")
    fresh = [e for e in log if e['scenarios'] is not None]
    by_id = lambda e: {s['id']: s for s in e['scenarios']}
    ids = [s['id'] for s in log[0]['scenarios']]
    constant = ([i for i in ids if all(by_id(e)[i]['failed'] for e in fresh)] if len(fresh) >= 2 else None)
    signal = [i for i in ids if i not in (constant or [])]
    best_index = next(e['index'] for e in log if e['objective'] == best)
    prior, top = by_id(log[0]), by_id(log[best_index])
    signal_mean = lambda e: float(np.mean([by_id(e)[i]['score'] for i in signal])) if signal else None

    def move_of(e):
        """폴 중심에서 바꾼 것. 폴 후보는 중심에서 한 좌표만 바꾼다. 평가 0(시작점)은 None."""
        _, c, _ = owner[e['index']]
        if c is None:
            return None
        delta = np.asarray(e['exponents'], dtype=float) - np.asarray(c, dtype=float)
        j = int(np.argmax(np.abs(delta)))
        return dict(parameter=names[j], factor=float(2.0**delta[j]),
                    center_value=float(record['prior'][names[j]]*2.0**c[j]), value=e['values'][names[j]])

    failures = []
    for e in fresh:
        new = [s for s in e['scenarios'] if s['failed'] and s['id'] not in (constant or [])]
        if new:
            failures.append(dict(evaluation=e['index'], objective=e['objective'], move=move_of(e),
                                 failed=[dict(id=s['id'], max_omega=s.get('max_omega'),
                                              stop_reason=s.get('stop_reason'),
                                              paper_reasons=s.get('paper_reasons')) for s in new]))

    s0 = float(np.log2(float(record['search']['initial_step'])))
    steps = [p['step_exponent'] for p in info['polls']]
    # |ω|는 신호 사례만 본다(상수 벌점 사례는 늘 발산이라 탐색 정보가 없다). 실패로 안 잡힌 큰 흔들림도 드러낸다.
    omegas = [(s['max_omega'], e['index'], s['id'], s['failed']) for e in fresh for s in e['scenarios']
              if s['id'] in signal and s.get('max_omega') is not None]
    peak = max(omegas, key=lambda item: item[0]) if omegas else None     # 같은 값이면 먼저 나온 평가
    best_omegas = [top[i]['max_omega'] for i in signal if top[i].get('max_omega') is not None]
    return dict(
        controller=label, status=record.get('status'), spent=len(log), budget=record['budget'],
        fresh_evaluations=len(fresh), git_revision=record.get('git_revision'), git_dirty=record.get('git_dirty'),
        config_sha256=record.get('config_sha256'),
        prior_objective=log[0]['objective'], best_objective=best, best_index=best_index,
        best_exponents=[float(v) for v in center],
        parameters=[dict(name=n, prior=float(record['prior'][n]), best=float(record['prior'][n]*2.0**x),
                         factor=float(2.0**x)) for n, x in zip(names, center)],
        constant_failures=constant, signal_scenarios=signal,
        signal_mean=dict(prior=signal_mean(log[0]), best=signal_mean(log[best_index])),
        scenarios=[dict(id=i, constant_failure=i in (constant or []),
                        prior=dict(score=prior[i]['score'], failed=prior[i]['failed'],
                                   max_omega=prior[i].get('max_omega')),
                        best=dict(score=top[i]['score'], failed=top[i]['failed'], max_omega=top[i].get('max_omega')))
                   for i in ids],
        search=dict(initial_step_exponent=s0, last_step_exponent=steps[-1] if steps else s0,
                    smallest_step_exponent=min(steps) if steps else s0,
                    step_reduced=bool(steps) and min(steps) < s0, polls=len(steps), restarts=info['restarts'],
                    evaluations_after_best=len(log) - 1 - best_index),
        failures=failures,
        best_signal_omega=max(best_omegas) if best_omegas else None,
        peak_signal_omega=None if peak is None else dict(max_omega=peak[0], evaluation=peak[1], case=peak[2],
                                                         failed=peak[3], move=move_of(log[peak[1]])))


def _move_text(mv):
    return '시작점' if mv is None else \
        f"{mv['parameter']} ×{mv['factor']:.4g}({mv['center_value']:.4g} → {mv['value']:.4g})"


def write_markdown(path, data):
    m = data['meta']
    f = lambda x, spec='.4g': '—' if x is None else format(x, spec)
    lines = [f"# 튜닝 기록 요약 — {m['run_dir_name']}(PILOT)", '',
             '**PILOT** — 튜닝 기록을 읽기만 한다. 우위 결론 없음. 돌고 있는 제어기는 **PARTIAL**이다.', '',
             f"재현: `{m['command']}` · 설정 sha256 `{m['config_sha256'][:12]}` · 요약 시점 git `{m['git_revision']}` "
             f"dirty={m['git_dirty']} · {m['created_utc'][:16]}Z", '',
             f"I-4(`arena_tune.check_tuning_records`): 위반 {len(m['i4_violations'])}건"
             + (' — ' + '; '.join(m['i4_violations']) if m['i4_violations'] else ''), '',
             '- 목적함수 = 튜닝 사례 점수의 평균(작을수록 좋다). 사례 점수 = 창 RMSE v + 창 RMSE z, 실패면 벌점 1000.',
             '- 상수 벌점 사례 = 새로 돌린 모든 평가에서 실패한 사례(탐색 신호 없음). 신호 사례 평균은 그 사례를 뺀 평균이다.',
             '- 보폭은 log₂ 배수 지수다(1 = ×2). 보폭이 한 번도 줄지 않았으면 수렴 전에 예산이 끝난 것이다.',
             '- \\|ω\\|max(rad/s)는 신호 사례만 본다. "탐색 전체"는 새로 돈 모든 평가 중 최대다 — 실패로 안 잡힌 큰 흔들림도 '
             '여기 드러난다(실패 기준: 20 rad/s가 0.1 s 이상 등, `configs/arena.json`).', '',
             '| 제어기 | 상태 | 평가(새로 돈 것) | 목적함수 사전 → 최선 | 신호 사례 평균 사전 → 최선 | 상수 벌점 사례 | '
             '최선 평가(뒤에 남은 평가) | 마지막 폴 보폭 · 축소 | 상수 밖 실패 평가 | 신호 사례 \\|ω\\|max 최선 / 탐색 전체 |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for c in data['controllers']:
        s = c['search']
        status = 'complete' if c['status'] == 'complete' else f"**PARTIAL**({c['status']})"
        constant = '판단 불가(평가 1회)' if c['constant_failures'] is None else (str(len(c['constant_failures'])) + '개')
        peak = c['peak_signal_omega']
        lines.append(f"| {c['controller']} | {status} | {c['spent']}/{c['budget']}({c['fresh_evaluations']}) | "
                     f"{f(c['prior_objective'])} → {f(c['best_objective'])} | "
                     f"{f(c['signal_mean']['prior'])} → {f(c['signal_mean']['best'])} | {constant} | "
                     f"{c['best_index']}({s['evaluations_after_best']}) | {s['last_step_exponent']:g} · "
                     f"{'있음' if s['step_reduced'] else '없음'} | {len(c['failures'])} | "
                     f"{f(c['best_signal_omega'], '.3g')} / "
                     f"{'—' if peak is None else format(peak['max_omega'], '.3g') + '(평가 ' + str(peak['evaluation']) + ')'} |")
    lines.append('')
    for c in data['controllers']:
        changed = [p for p in c['parameters'] if p['factor'] != 1.0]
        lines += [f"## {c['controller']}", '',
                  f"튜닝 시작 git `{c['git_revision']}` dirty={c['git_dirty']} · 설정 sha256 `{(c['config_sha256'] or '')[:12]}`"
                  + ('' if c['config_sha256'] == data['meta']['config_sha256'] else ' **(지금 설정과 다름)**'), '',
                  '최선값(평가 ' + str(c['best_index']) + '): '
                  + (', '.join(f"{p['name']} {p['prior']:.4g} → {p['best']:.4g}(×{p['factor']:.4g})" for p in changed)
                     or '사전값 그대로') + ('. 나머지는 사전값 그대로' if changed else ''), '']
        if c['constant_failures']:
            lines += ['상수 벌점 사례: ' + ', '.join(c['constant_failures']), '']
        lines += ['| 사례 | 사전 점수 | 최선 점수 | 사전 \\|ω\\|max | 최선 \\|ω\\|max | 비고 |', '|---|---:|---:|---:|---:|---|']
        for s in c['scenarios']:
            note = '상수 벌점' if s['constant_failure'] else ('최선에서 실패' if s['best']['failed'] else '')
            lines.append(f"| {s['id']} | {f(s['prior']['score'])} | {f(s['best']['score'])} | "
                         f"{f(s['prior']['max_omega'], '.3g')} | {f(s['best']['max_omega'], '.3g')} | {note} |")
        lines.append('')
        peak = c['peak_signal_omega']
        if peak is not None:
            lines += [f"신호 사례 \\|ω\\|max: 최선 평가 {f(c['best_signal_omega'], '.3g')} rad/s, 탐색 전체 "
                      f"{peak['max_omega']:.3g} rad/s(평가 {peak['evaluation']}, {peak['case']}, "
                      f"{_move_text(peak['move'])}, {'실패' if peak['failed'] else '실패 아님'})", '']
        if c['failures']:
            lines += ['상수 벌점 사례 밖에서 실패한 평가:', '',
                      '| 평가 | 폴 중심에서 바꾼 것 | 목적함수 | 새로 실패한 사례(\\|ω\\|max, 사유) |', '|---:|---|---:|---|']
            for x in c['failures']:
                cases = '; '.join(f"{y['id']}({f(y['max_omega'], '.3g')}, "
                                  f"{y['stop_reason'] or ', '.join(y['paper_reasons'] or [])})" for y in x['failed'])
                lines.append(f"| {x['evaluation']} | {_move_text(x['move'])} | {f(x['objective'])} | {cases} |")
            lines.append('')
        flags = [g for g in data['meta']['integrator_flags'] if g['controller'] == c['controller']]
        if flags:
            fractions = [g['at_limit_fraction'] for g in flags]
            cases = sorted({g['case'] for g in flags})
            channels = sorted({ch for g in flags for ch in g['channels']})
            lines += [f"적분기 1% 규칙: {len(flags)}건 — 사례 {', '.join(cases)}, 평가 "
                      f"{', '.join(str(g['evaluation']) for g in flags)}, 비율 {min(fractions):.3g}~{max(fractions):.3g}, "
                      f"채널 {', '.join(channels)}"
                      + (' (모두 상수 벌점 사례 — 목적함수에 영향 없음)'
                         if c['constant_failures'] and set(cases) <= set(c['constant_failures']) else ''), '']
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    from control.validation_suite import write_json, git_state
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    runs = load_run(args.run_dir, list(config['controllers']))
    if not runs:
        raise SystemExit(f'no tuning records in {args.run_dir}')
    controllers = [analyse(record, log) for record, log in runs.values()]
    _, flags = integrator_limit_flags(_evaluation_entries(args.run_dir), config)
    revision, dirty = git_state()
    name = args.run_dir.name
    data = dict(meta=dict(label='PILOT', command=f'python -m control.arena_tuning_report --run-dir {args.run_dir}',
                          created_utc=datetime.now(timezone.utc).isoformat(), config_sha256=config_sha256(config),
                          git_revision=revision, git_dirty=dirty, run_dir=str(args.run_dir), run_dir_name=name,
                          i4_violations=check_tuning_records([r for r, _ in runs.values()], config),
                          integrator_flags=flags),
                controllers=controllers)
    args.out.mkdir(parents=True, exist_ok=True)
    write_json(args.out/f'tuning_{name}.json', data)
    write_markdown(args.out/f'TUNING_{name}.md', data)
    print((args.out/f'TUNING_{name}.md').read_text(encoding='utf-8'))
    return data


if __name__ == '__main__':
    main()
