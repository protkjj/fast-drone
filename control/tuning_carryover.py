"""튜닝 기록 승계 — arena.json으로 튜닝한 V13·CPID 기록을 arena_v2.json(모멘트 보정 절만 다름)에서 쓰기 위한
증명과 표(kj 결정 2026-09-28 밤, 보고서 23절).

모멘트 보정은 M17·F13·GSLQR에만 들어간다. V13·CPID는 같은 명목 모델·같은 튜닝 시나리오를 보므로 튜닝
결과가 비트 동일해야 한다 — 그 주장을 **다시 계산해서** 증명한 기록만 승계한다:
  - 평가 0과 최종 최선 평가를 arena_v2에서 arena_tune_repro로 다시 계산해 둘 다 비트 동일(목적함수·
    시나리오별 전 필드·궤적 sha256)일 때만 표에 넣는다.
  - 표에는 기록 파일 sha256, 두 설정의 정규화 해시, 모멘트 모델 파일 sha256, 증명 결과를 적는다.
  - 규칙(record_hash_problems): 표에 있는 기록(파일 sha256까지 일치)만 arena.json 해시를 허용하고, 나머지는
    현재 설정(arena_v2) 해시여야 한다. `arena_tune --summarize`(I-4)와 `tuned_overrides`가 같이 쓴다.

    python3 -m control.tuning_carryover --controller CPID --run-dir results/arena/tuning/retune_v3
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

from control.arena import ROOT, DEFAULT_CONFIG, load_config, config_sha256

DEFAULT_TABLE = ROOT/'configs'/'tuning_carryover.json'
DEFAULT_TARGET = ROOT/'configs'/'arena_v2.json'
SCHEMA = 'tuning_carryover/1'


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_table(path=DEFAULT_TABLE):
    path = Path(path)
    if not path.exists():
        return None
    doc = json.loads(path.read_text(encoding='utf-8'))
    if doc.get('schema') != SCHEMA:
        raise ValueError(f'{path}: not a carry-over table')
    return doc


def record_hash_problems(records, config, table):
    """records: [(record dict, record 파일 경로)]. 해시 규칙 위반 목록(빈 목록이면 통과)."""
    now = config_sha256(config)
    problems = []
    for record, path in records:
        if record.get('config_sha256') == now:
            continue
        label = record['controller']
        entry = (table or {}).get('entries', {}).get(label)
        if entry is None:
            problems.append(f'{label}: tuned with config {str(record.get("config_sha256"))[:12]}, '
                            f'not {now[:12]}, and not in the carry-over table')
            continue
        checks = {
            'record file sha256': _sha(path) == entry['record_sha256'],
            'record config is the table base config': record.get('config_sha256') == table['base_config_sha256'],
            'current config is the table target config': now == table['target_config_sha256'],
            'both proofs bit-identical': all(p.get('bit_identical') for p in entry['proofs'].values()),
        }
        problems += [f'{label}: carry-over check failed — {name}' for name, ok in checks.items() if not ok]
    return problems


def prove(label, run_dir, target=DEFAULT_TARGET, table_path=DEFAULT_TABLE, scenario_workers=1):
    """평가 0과 최종 최선 평가를 target 설정에서 다시 계산. 둘 다 비트 동일이면 표에 넣는다."""
    from control.arena_tune_repro import reproduce, best_index, carryover_allowed
    from control.validation_suite import git_state
    revision, dirty = git_state()
    if dirty:
        # 증명은 커밋된 코드에서만 만든다 — 어느 코드가 비트 동일을 보였는지 표에 남기려고(kj 2026-09-28).
        raise SystemExit('refusing to prove on a dirty working tree — commit first')
    config = load_config(target)
    if carryover_allowed(config, label) is None:
        raise SystemExit(f'{label}: not eligible for carry-over under {target}')
    run_dir = Path(run_dir)
    record_path = run_dir/f'{label}.record.json'
    record = json.loads(record_path.read_text(encoding='utf-8'))
    if record.get('status') != 'complete':
        raise SystemExit(f'{label}: tuning not complete ({record.get("status")}) — prove after it finishes')
    proofs = {}
    for name, index in (('evaluation_0', 0), ('best', best_index(run_dir, label))):
        r = reproduce(config, label, run_dir, index, scenario_workers, carryover=True)
        proofs[name] = dict(index=r['index'], objective=r['objective'], passed=r['passed'],
                            bit_identical=r['bit_identical'], differing_fields=r['differing_fields'][:20],
                            model_sha256_equal=r['model_sha256_equal'], wall_s=r['wall_s'])
        print(f"{label} {name} (index {r['index']}): bit_identical={r['bit_identical']} "
              f"objective={r['objective']}", flush=True)
    ok = all(p['bit_identical'] and p['model_sha256_equal'] for p in proofs.values())
    if not ok:
        print(f'{label}: NOT carried over — proofs are not bit-identical', flush=True)
        return proofs, False
    base = load_config(DEFAULT_CONFIG)
    moment = config['controller_model']['moment_correction']
    table = load_table(table_path) or dict(schema=SCHEMA, entries={})
    table.update(base_config=str(DEFAULT_CONFIG.relative_to(ROOT)), base_config_sha256=config_sha256(base),
                 target_config=str(Path(target).resolve().relative_to(ROOT)),
                 target_config_sha256=config_sha256(config), moment_model_sha256=moment['sha256'])
    table['entries'][label] = dict(record=str(record_path.resolve().relative_to(ROOT)),
                                   record_sha256=_sha(record_path), record_git_revision=record.get('git_revision'),
                                   proved_at_revision=revision, proved_git_dirty=dirty,
                                   proved_utc=datetime.now(timezone.utc).isoformat(), proofs=proofs)
    Path(table_path).write_text(json.dumps(table, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    print(f'{label}: carried over → {table_path}', flush=True)
    return proofs, True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--controller', required=True, choices=('V13', 'CPID'))
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--target', type=Path, default=DEFAULT_TARGET)
    parser.add_argument('--table', type=Path, default=DEFAULT_TABLE)
    parser.add_argument('--scenario-workers', type=int, default=1)
    args = parser.parse_args(argv)
    _, ok = prove(args.controller, args.run_dir, args.target, args.table, args.scenario_workers)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
