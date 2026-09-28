"""본 실험 분산 실행 — 조각 배분, 이어 돌리기, 합치기, 교차 확인(kj 결정 2026-09-28, 보고서 22절).

  python -m control.main_distributed make-shards --n 4 --out shards.json
  python -m control.main_distributed run-shard  --shards shards.json --index 0 --output runs
  python -m control.main_distributed status     --shards shards.json --output runs
  python -m control.main_distributed merge      --shards shards.json --inputs runs_A runs_B --out merged
  python -m control.main_distributed cross-select  --merged merged --out cross_shards.json
  python -m control.main_distributed cross-compare --cross-shards cross_shards.json --merged merged --rerun runs_X

규칙(kj)
  - 시행 하나 = (묶음, 시나리오, 제어기). ID는 '묶음/시나리오/제어기'. 한 시행은 한 컴퓨터에서 끝까지 돈다.
  - 조각은 예상 시간 기준으로 고르게 나눈다(긴 시행부터 가장 덜 찬 조각에 — LPT).
  - 이어 돌리기: 시행마다 결과 파일 하나(trials/<ID>.json)를 **임시 파일 → 이름 바꾸기**로 쓴다. 이름이
    바뀐 파일만 '끝남'이다. 다시 실행하면 해시가 같은 끝난 파일은 건너뛰고, 끊긴 시행(임시 파일)은
    지우고 처음부터 다시 돈다. 해시가 다른 끝난 파일이 있으면 섞지 않고 멈춘다.
  - 모든 행에 설정·튜닝 해시와 컴퓨터 지문(호스트 이름 포함)을 적는다.
  - 합치기: 누락·중복·모르는 시행·해시 불일치·지문 누락·플랫폼(기본 Windows — 맥 결과와 섞지 않는다)·
    git 리비전 불일치를 검사한다. 하나라도 있으면 합치지 않는다.
  - 교차 확인: 합친 결과에서 돌린 시행의 5%를 고정 시드로 뽑아 **다른 컴퓨터**에서 다시 돌리고,
    판정 일치 + rtol 1e-3을 본다(arena_tune_repro와 같은 규칙).
"""
import argparse
from datetime import datetime, timezone
import glob
import json
import math
import os
from pathlib import Path
import platform
import sys

import numpy as np

from control.arena import ROOT, load_config, config_sha256, build_scenarios, excluded_from
from control import main_experiment as me

SHARDS_SCHEMA = 'main_shards/1'
VERDICT_KEYS = ('skipped', 'skip_reason', 'passed', 'paper_failed', 'paper_reasons', 'stop_reason',
                'failure_reasons')
NUMERIC_KEYS = ('window_rmse_velocity', 'window_rmse_z', 'window_max_omega', 'max_omega')


def machine_fingerprint():
    """environment_fingerprint(튜닝 기록에도 쓰여 건드리지 않는다) + 호스트 이름."""
    from control.validation_suite import environment_fingerprint
    return dict(environment_fingerprint(), hostname=platform.node())


def safe_name(trial_id):
    return trial_id.replace('/', '__').replace(':', '_')


def _utc():
    return datetime.now(timezone.utc).isoformat()


def _write_atomic(path, text):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(text, encoding='utf-8')
    os.replace(tmp, path)            # 같은 디렉터리 안 이름 바꾸기 — Windows·맥 모두 원자적


# ══════════════════════════════════════════════════════════════════
# 실행 문맥(묶음·팩토리·시나리오를 필요할 때 한 번씩 만든다)
# ══════════════════════════════════════════════════════════════════

class Campaign:
    def __init__(self, spec, spec_path, root=ROOT, with_tuning=True):
        from control.validation_suite import baseline_params
        self.spec, self.spec_path, self.root = spec, Path(spec_path), root
        self.spec_sha256 = me._sha256_file(spec_path)
        self.base = load_config(root/spec['base_config']['path'])
        self.base_sha256 = config_sha256(self.base)
        self.native = baseline_params()
        self.batches = {b.name: b for b in me.build_batches(spec, self.base, self.native)}
        self.tuned_sha = {label: spec['tuned']['records'].get(label, {}).get('sha256', '')
                          for label in spec['controllers']}
        self.overrides, self.used = None, None
        if with_tuning:
            from control.arena_design_check import tuned_overrides
            self.overrides, self.used = tuned_overrides(self.base, root/spec['tuned']['run_dir'],
                                                        spec['controllers'])
        self._factories, self._scenarios, self._trim = {}, {}, {}

    def factory(self, batch_name):
        from control.arena_factory import ArenaFactory
        if batch_name not in self._factories:
            self._factories[batch_name] = ArenaFactory(self.batches[batch_name].config, self.native,
                                                       overrides=self.overrides)
        return self._factories[batch_name]

    def scenarios(self, batch_name):
        if batch_name not in self._scenarios:
            from control.arena_factory import ArenaFactory
            cp = ArenaFactory(self.base, self.native).cp       # a_avail은 명목 모델(튜닝과 무관)
            self._scenarios[batch_name] = {s.id: s for s in build_scenarios(
                self.batches[batch_name].config, cp, self.native)}
        return self._scenarios[batch_name]

    def trials(self, only_batches=None):
        """전 시행 목록과 예상 시간(맥 실측 × 1). 건너뛸 시행(설계 영역·기체 한계)도 행으로 남긴다."""
        rates = self.spec['timing_mac_wall_per_sim_s']
        out = []
        for name, batch in self.batches.items():
            if only_batches and name not in only_batches:
                continue
            for sid, s in self.scenarios(name).items():
                for label in batch.controllers:
                    est = 0.0 if excluded_from(batch.config, label, s) else s.profile.T_total*float(rates[label])
                    out.append(dict(trial_id=f'{name}/{sid}/{label}', batch=name, scenario_id=sid,
                                    controller=label, variant=batch.variant, est_seconds=est))
        return out

    def hashes_for(self, label):
        return dict(spec_sha256=self.spec_sha256, base_config_sha256=self.base_sha256,
                    tuned_record_sha256=self.tuned_sha.get(label, ''))

    def run_trial(self, trial):
        """시행 하나 → 결과 행(dict)과 궤적(dict 또는 None)."""
        from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
        from control.validation_suite import run_trial as suite_run
        batch = self.batches[trial['batch']]
        s = self.scenarios(trial['batch'])[trial['scenario_id']]
        label = trial['controller']
        if s.id not in self._trim:
            self._trim[s.id] = me.trim_status(self.native, s)
        status, detail = self._trim[s.id]
        row = dict(trial_id=trial['trial_id'], batch=trial['batch'], scenario_id=s.id, scenario_type=s.type,
                   controller=label, variant=batch.variant, case=s.cases[0], window=list(s.window),
                   derived_config_sha256=config_sha256(batch.config), **self.hashes_for(label),
                   trim_detail={f'{v:g}': d for v, d in detail.items()},
                   authority_boundary=me.authority_boundary(detail, self.spec, self.base))
        reason = excluded_from(batch.config, label, s)
        if reason or status:
            row.update(skipped=True, skip_reason=reason or 'vehicle_limit')
            return row, None
        limits = Acceptance(**batch.config['acceptance'])
        paper = PaperCriteria(**batch.config['paper_criteria'])
        try:
            metrics, result, log = suite_run(self.factory(trial['batch']), label, s.profile, s.cases[0], limits)
            metrics.update(paper_evaluate(result, s.profile, paper, window=s.window, solve_log=log,
                                          n_max=self.native['n_max']))
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            metrics = dict(passed=False, failure_reasons=['setup_error'], stop_reason=f'{type(exc).__name__}: {exc}')
            result = None
        row.update(skipped=False, skip_reason=None, **metrics)
        return row, result


# ══════════════════════════════════════════════════════════════════
# 조각 만들기
# ══════════════════════════════════════════════════════════════════

def make_shards(spec_path, n, out, only_batches=None, root=ROOT):
    spec = me.load_spec(spec_path)
    problems = me.guard_problems(spec, root)
    if problems:
        raise SystemExit('refusing to make shards:\n  - ' + '\n  - '.join(problems))
    camp = Campaign(spec, spec_path, root, with_tuning=False)
    trials = camp.trials(only_batches)
    loads = [0.0]*n
    assigned = [[] for _ in range(n)]
    # LPT: 긴 시행부터, 지금 가장 덜 찬 조각에. 동률은 조각 번호가 작은 쪽 — 결정적이다.
    for t in sorted(trials, key=lambda t: (-t['est_seconds'], t['trial_id'])):
        k = min(range(n), key=lambda i: (loads[i], i))
        assigned[k].append(t['trial_id'])
        loads[k] += t['est_seconds']
    doc = dict(schema=SHARDS_SCHEMA, created_utc=_utc(), spec_path=str(Path(spec_path)),
               spec_sha256=camp.spec_sha256, base_config_sha256=camp.base_sha256,
               tuned_record_sha256=camp.tuned_sha, only_batches=only_batches, n_shards=n,
               trials={t['trial_id']: t for t in trials},
               shards=[dict(index=i, est_hours_mac=loads[i]/3600.0, trials=assigned[i]) for i in range(n)])
    _write_atomic(Path(out), json.dumps(doc, ensure_ascii=False, indent=1))
    return doc


def load_shards(path):
    doc = json.loads(Path(path).read_text(encoding='utf-8'))
    if doc.get('schema') != SHARDS_SCHEMA:
        raise ValueError(f'{path}: not a shards file')
    return doc


# ══════════════════════════════════════════════════════════════════
# 조각 실행(이어 돌리기)
# ══════════════════════════════════════════════════════════════════

def run_shard(shards_path, index, output, root=ROOT, max_trials=None):
    doc = load_shards(shards_path)
    spec_path = Path(doc['spec_path'])
    spec_path = spec_path if spec_path.is_absolute() else root/spec_path
    spec = me.load_spec(spec_path)
    problems = me.guard_problems(spec, root)
    if me._sha256_file(spec_path) != doc['spec_sha256']:
        problems.append('spec file changed since the shards were made')
    if problems:
        raise SystemExit('refusing to run the shard:\n  - ' + '\n  - '.join(problems))
    camp = Campaign(spec, spec_path, root)
    if camp.base_sha256 != doc['base_config_sha256'] or camp.tuned_sha != doc['tuned_record_sha256']:
        raise SystemExit('refusing to run the shard: hashes differ from the shards file')
    out = Path(output)/f'shard{index}'
    (out/'trials').mkdir(parents=True, exist_ok=True)
    for stale in glob.glob(str(out/'trials'/'*.tmp')):
        os.remove(stale)                   # 끊긴 시행의 흔적 — 그 시행은 처음부터 다시 돈다
        print(f'removed interrupted trial file {Path(stale).name}', flush=True)
    from control.validation_suite import json_safe, git_state
    revision, dirty = git_state()
    machine = machine_fingerprint()
    ran = 0
    for trial_id in doc['shards'][index]['trials']:
        trial = doc['trials'][trial_id]
        path = out/'trials'/(safe_name(trial_id)+'.json')
        if path.exists():
            done = json.loads(path.read_text(encoding='utf-8'))
            if {k: done.get(k) for k in camp.hashes_for(trial['controller'])} != camp.hashes_for(trial['controller']):
                raise SystemExit(f'{path}: finished with different hashes — refusing to mix results')
            continue
        if max_trials is not None and ran >= max_trials:
            break
        started = _utc()
        row, result = camp.run_trial(trial)
        if result is not None:
            npz = out/'trials'/(safe_name(trial_id)+'.npz')
            with open(str(npz)+'.tmp', 'wb') as stream:
                np.savez_compressed(stream, **{k: v for k, v in result.items() if k != 'error'})
            os.replace(str(npz)+'.tmp', npz)
        row.update(shard=index, machine=machine, git_revision=revision, git_dirty=dirty,
                   started_utc=started, finished_utc=_utc())
        _write_atomic(path, json.dumps(json_safe(row), ensure_ascii=False, allow_nan=False))
        ran += 1
        print(f"[{index}] {trial_id}: {'SKIP '+row['skip_reason'] if row['skipped'] else row.get('passed')}",
              flush=True)
    return out


def status(shards_path, output):
    doc = load_shards(shards_path)
    lines = []
    for shard in doc['shards']:
        folder = Path(output)/f"shard{shard['index']}"/'trials'
        done = {p.stem for p in folder.glob('*.json')} if folder.exists() else set()
        todo = [t for t in shard['trials'] if safe_name(t) not in done]
        left = sum(doc['trials'][t]['est_seconds'] for t in todo)/3600.0
        lines.append(f"shard {shard['index']}: {len(shard['trials'])-len(todo)}/{len(shard['trials'])} done, "
                     f"~{left:.2f} h left (mac estimate)")
    return lines


# ══════════════════════════════════════════════════════════════════
# 합치기
# ══════════════════════════════════════════════════════════════════

def collect(inputs):
    rows = {}
    duplicates = []
    for folder in inputs:
        for path in sorted(glob.glob(str(Path(folder)/'**'/'trials'/'*.json'), recursive=True)):
            row = json.loads(Path(path).read_text(encoding='utf-8'))
            if row['trial_id'] in rows:
                duplicates.append(f"{row['trial_id']}: {rows[row['trial_id']][1]} and {path}")
                continue
            rows[row['trial_id']] = (row, path)
    return rows, duplicates


def merge(shards_path, inputs, out, require_platform='Windows'):
    doc = load_shards(shards_path)
    rows, duplicates = collect(inputs)
    problems = [f'duplicate {d}' for d in duplicates]
    expected = set(doc['trials'])
    problems += [f'missing {t}' for t in sorted(expected - set(rows))]
    problems += [f'unknown {t}' for t in sorted(set(rows) - expected)]
    revisions = set()
    for trial_id, (row, path) in sorted(rows.items()):
        if trial_id not in expected:
            continue
        want = dict(spec_sha256=doc['spec_sha256'], base_config_sha256=doc['base_config_sha256'],
                    tuned_record_sha256=doc['tuned_record_sha256'].get(row['controller'], ''))
        for key, value in want.items():
            if row.get(key) != value:
                problems.append(f'{trial_id}: {key} mismatch')
        machine = row.get('machine') or {}
        if not machine.get('hostname') or not machine.get('system'):
            problems.append(f'{trial_id}: machine fingerprint missing')
        elif require_platform and machine['system'] != require_platform:
            problems.append(f"{trial_id}: platform {machine['system']} != required {require_platform}")
        revisions.add((row.get('git_revision'), row.get('git_dirty')))
    if len(revisions) > 1:
        problems.append(f'rows come from different code states: {sorted(map(str, revisions))}')
    report = dict(created_utc=_utc(), shards_file=str(shards_path), inputs=[str(i) for i in inputs],
                  require_platform=require_platform, expected=len(expected), found=len(rows),
                  problems=problems)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    _write_atomic(out/'merge_report.json', json.dumps(report, ensure_ascii=False, indent=1))
    if problems:
        return report
    with open(out/'merged.jsonl.tmp', 'w', encoding='utf-8') as stream:
        for trial_id in sorted(rows):
            stream.write(json.dumps(rows[trial_id][0], ensure_ascii=False)+'\n')
    os.replace(out/'merged.jsonl.tmp', out/'merged.jsonl')
    return report


def load_merged(folder):
    return [json.loads(line) for line in (Path(folder)/'merged.jsonl').read_text(encoding='utf-8').splitlines()]


# ══════════════════════════════════════════════════════════════════
# 교차 확인
# ══════════════════════════════════════════════════════════════════

def cross_select(shards_path, merged, out, fraction=0.05, seed=20260928):
    """돌린 시행(건너뛴 것 제외) 중 fraction을 고정 시드로 뽑아 조각 1개짜리 shards 파일로 쓴다."""
    doc = load_shards(shards_path)
    ran = sorted(r['trial_id'] for r in load_merged(merged) if not r.get('skipped'))
    k = max(1, math.ceil(fraction*len(ran)))
    picked = sorted(np.random.default_rng(seed).choice(ran, size=k, replace=False).tolist())
    cross = dict(doc, created_utc=_utc(), n_shards=1,
                 trials={t: doc['trials'][t] for t in picked},
                 shards=[dict(index=0, est_hours_mac=sum(doc['trials'][t]['est_seconds'] for t in picked)/3600.0,
                              trials=picked)],
                 cross_check=dict(of_merged=str(merged), fraction=fraction, seed=seed, population=len(ran)))
    _write_atomic(Path(out), json.dumps(cross, ensure_ascii=False, indent=1))
    return cross


def _close(a, b, rtol):
    if a is None or b is None:
        return a == b
    return abs(a - b) <= rtol*max(abs(a), abs(b)) + 1e-9


def cross_compare(cross_shards, merged, rerun, rtol=1e-3, allow_same_machine=False):
    cross = load_shards(cross_shards)
    original = {r['trial_id']: r for r in load_merged(merged)}
    again, duplicates = collect([rerun])
    results, problems = [], [f'duplicate {d}' for d in duplicates]
    for trial_id in cross['shards'][0]['trials']:
        if trial_id not in again:
            problems.append(f'{trial_id}: not rerun')
            continue
        a, b = original[trial_id], again[trial_id][0]
        same_host = a['machine']['hostname'] == b['machine']['hostname']
        if same_host and not allow_same_machine:
            problems.append(f'{trial_id}: rerun on the same computer ({a["machine"]["hostname"]})')
        verdict = [k for k in VERDICT_KEYS if a.get(k) != b.get(k)]
        numeric = [k for k in NUMERIC_KEYS if not _close(a.get(k), b.get(k), rtol)]
        ok = not verdict and not numeric
        results.append(dict(trial_id=trial_id, passed=ok, verdict_mismatch=verdict, numeric_mismatch=numeric,
                            bit_identical=a.get('trajectory_sha256') == b.get('trajectory_sha256'),
                            hosts=[a['machine']['hostname'], b['machine']['hostname']]))
        if not ok:
            problems.append(f'{trial_id}: verdict {verdict} numeric {numeric}')
    return dict(rtol=rtol, allow_same_machine=allow_same_machine, checked=len(results),
                passed=not problems and len(results) == len(cross['shards'][0]['trials']),
                problems=problems, results=results)


# ══════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('make-shards')
    p.add_argument('--spec', type=Path, default=me.DEFAULT_SPEC)
    p.add_argument('--n', type=int, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--only-batches', nargs='*', help='시험용 — 일부 묶음만')
    p = sub.add_parser('run-shard')
    p.add_argument('--shards', type=Path, required=True)
    p.add_argument('--index', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--max-trials', type=int, help='시험용 — 이번 실행에서 새로 돌릴 시행 수 상한')
    p = sub.add_parser('status')
    p.add_argument('--shards', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('merge')
    p.add_argument('--shards', type=Path, required=True)
    p.add_argument('--inputs', type=Path, nargs='+', required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--require-platform', default='Windows', help="본 실험은 Windows. 맥 시험만 'Darwin'")
    p = sub.add_parser('cross-select')
    p.add_argument('--shards', type=Path, required=True)
    p.add_argument('--merged', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--fraction', type=float, default=0.05)
    p.add_argument('--seed', type=int, default=20260928)
    p = sub.add_parser('cross-compare')
    p.add_argument('--cross-shards', type=Path, required=True)
    p.add_argument('--merged', type=Path, required=True)
    p.add_argument('--rerun', type=Path, required=True)
    p.add_argument('--rtol', type=float, default=1e-3)
    p.add_argument('--allow-same-machine', action='store_true', help='맥 시험 전용 — 본 실험에서는 쓰지 않는다')
    args = parser.parse_args(argv)

    if args.cmd == 'make-shards':
        doc = make_shards(args.spec, args.n, args.out, args.only_batches)
        for s in doc['shards']:
            print(f"shard {s['index']}: {len(s['trials'])} trials, ~{s['est_hours_mac']:.2f} h (mac estimate)")
    elif args.cmd == 'run-shard':
        run_shard(args.shards, args.index, args.output, max_trials=args.max_trials)
    elif args.cmd == 'status':
        print('\n'.join(status(args.shards, args.output)))
    elif args.cmd == 'merge':
        report = merge(args.shards, args.inputs, args.out, args.require_platform)
        print(json.dumps({k: report[k] for k in ('expected', 'found')}, ensure_ascii=False))
        if report['problems']:
            print('MERGE REFUSED:\n  - ' + '\n  - '.join(report['problems'][:50]))
            return 1
        print('MERGED')
    elif args.cmd == 'cross-select':
        cross = cross_select(args.shards, args.merged, args.out, args.fraction, args.seed)
        print(f"picked {len(cross['shards'][0]['trials'])} of {cross['cross_check']['population']} trials")
    elif args.cmd == 'cross-compare':
        report = cross_compare(args.cross_shards, args.merged, args.rerun, args.rtol, args.allow_same_machine)
        print(json.dumps(report, ensure_ascii=False, indent=1))
        print('CROSS-CHECK PASS' if report['passed'] else 'CROSS-CHECK FAIL')
        return 0 if report['passed'] else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
