"""스모크 실행 합치기 — 같은 커밋·설정·환경에서 제어기를 나눠 돌린 실행들을 참조 하나로 합친다. **SMOKE**.

M17 NLP가 무거워서 스모크를 제어기별로 나눠 병렬로 돌릴 때 쓴다(예: `--only-controllers M17`과 나머지 4종).
합치기 전에 확인하고, 하나라도 어긋나면 거부한다.
  - 실행마다 status complete이고, 기록한 행 수가 기대 행 수와 같다
  - 설정 sha256, git 리비전, 환경 지문, 소스 해시, 파라미터 해시, 제어기 모델 해시가 모두 같고 git dirty가 아니다
  - 게인 출처가 같다(모두 사전값, 또는 모두 같은 튜닝 run-dir)
  - 사례 목록이 같고 사례마다 case가 하나다
  - 제어기가 실행끼리 겹치지 않고, 합치면 설정의 제어기 전부다
행은 설정의 사례 순서 → 제어기 순서로 놓고(한 번에 돌린 스모크와 같은 순서), 어느 실행에서 왔는지 남긴다.

실행: python -m control.arena_reference_merge --runs <실행 폴더 A> <실행 폴더 B> --out <참조.json>
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

# 실행끼리 같아야 하는 manifest 항목 — 다르면 합친 참조의 행들이 서로 다른 조건에서 나온 것이다
SAME = ('config_sha256', 'git_revision', 'git_dirty', 'environment', 'source_sha256', 'parameter_sha256',
        'controller_model_sha256', 'scenarios')


class MergeRefused(ValueError):
    """합치면 안 되는 실행들이다."""


def load(run):
    run = Path(run)
    return (json.loads((run/'manifest.json').read_text(encoding='utf-8')),
            json.loads((run/'arena_reference.json').read_text(encoding='utf-8')))


def merge(runs):
    """[(이름, manifest, reference)] → 합친 참조(dict). 어긋나면 MergeRefused."""
    if len(runs) < 2:
        raise MergeRefused('give at least two runs to merge')
    problems = []
    for name, manifest, _ in runs:
        if manifest.get('status') != 'complete' or manifest.get('recorded_trials') != manifest.get('expected_trials'):
            problems.append(f"{name}: not complete ({manifest.get('status')}, "
                            f"{manifest.get('recorded_trials')}/{manifest.get('expected_trials')} trials)")
        if manifest.get('git_dirty') is not False:
            problems.append(f'{name}: git dirty={manifest.get("git_dirty")}')
    first = runs[0][1]
    for key in SAME:
        differ = [name for name, manifest, _ in runs[1:] if manifest.get(key) != first.get(key)]
        if differ:
            problems.append(f'{key} differs: {runs[0][0]} vs {", ".join(differ)}')
    sources = {None if m.get('tuned') is None else m['tuned']['run_dir'] for _, m, _ in runs}
    if len(sources) > 1:
        problems.append(f'gain sources differ: {sorted(map(str, sources))}')
    if any(len(s['cases']) != 1 for s in first['scenarios']):
        problems.append('a scenario has more than one case (row order would be ambiguous)')
    order = list(first['config']['controllers'])
    seen = {}
    for name, manifest, _ in runs:
        for label in manifest['controllers']:
            if label in seen:
                problems.append(f'{label} ran in both {seen[label]} and {name}')
            seen[label] = name
    missing = [label for label in order if label not in seen]
    if missing:
        problems.append(f'controllers not covered by any run: {missing}')
    if problems:
        raise MergeRefused('; '.join(problems))

    scenario_index = {s['id']: i for i, s in enumerate(first['scenarios'])}
    rows = sorted((dict(row, merged_from=name) for name, _, reference in runs for row in reference['rows']),
                  key=lambda r: (scenario_index[r['scenario_id']], order.index(r['controller'])))
    tuned = None
    if sources != {None}:
        tuned = dict(run_dir=next(iter(sources)),
                     controllers={label: t for _, m, _ in runs for label, t in m['tuned']['controllers'].items()})
    reference = runs[0][2]
    merged = dict(
        label=reference['label'], created_utc=datetime.now(timezone.utc).isoformat(),
        command=' + '.join(f"{r['command']} --only-controllers {' '.join(m['controllers'])}" for _, m, r in runs),
        merged_from=[dict(run=name, controllers=m['controllers'], created_utc=r['created_utc']) for name, m, r in runs],
        config_sha256=first['config_sha256'], git_revision=first['git_revision'], git_dirty=first['git_dirty'],
        parameter_sha256=first['parameter_sha256'], controller_model_sha256=first['controller_model_sha256'],
        environment=first['environment'], source_sha256=first['source_sha256'], scenarios=first['scenarios'],
        controller_settings={label: s for _, _, r in runs for label, s in r['controller_settings'].items()},
        rows=rows)
    if tuned is not None:
        merged['tuned'] = tuned
    return merged


def main(argv=None):
    from control.validation_suite import write_json
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--runs', nargs='+', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        merged = merge([(run.name, *load(run)) for run in args.runs])
    except MergeRefused as exc:
        parser.error(str(exc))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.out, merged)
    print(f"merged {len(merged['rows'])} rows from {', '.join(r.name for r in args.runs)} → {args.out}")
    return merged


if __name__ == '__main__':
    main()
