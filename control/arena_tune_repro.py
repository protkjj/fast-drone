"""튜닝 기록 재현 검사 — 기록(<제어기>.jsonl)의 평가 하나를 지금 코드·환경에서 다시 계산해 비교한다.

쓰임:
  - 코드가 바뀐 뒤에도 튜닝 경로 결과가 그대로인지(tune-final → tune-final-2)
  - 다른 컴퓨터(학교)가 이 기록을 재현하는지 — 튜닝을 나눠 돌리기 전에 확인
  - --scenario-workers N 으로 병렬 경로가 순차 기록과 같은지

  python3 -m control.arena_tune_repro --controller V13 \
      --run-dir results/arena/tuning/retune_v3 --index 0 --scenario-workers 3

합격 기준(kj 결정 2026-09-28, 다른 컴퓨터는 비트 일치가 아닐 수 있다 — `scripts/verify_arena.py`와 같은 규칙):
  - 판정 일치: 시나리오 id·순서, failed, stop_reason, paper_reasons, 적분기 1% 규칙 판정이 정확히 같다
  - 수치: 목적함수와 시나리오별 score·window_rmse_velocity·window_rmse_z·max_omega가
    |a−b| ≤ rtol·max(|a|,|b|) + 1e-9 (rtol 기본 1e-3)
비트 일치 여부(궤적 sha256 포함 전 필드)와 제어기 모델 sha256 일치 여부는 **기록만** 한다.
합격이면 0, 아니면 1로 끝난다. 기록이 캐시 적중(scenarios 없음)이면 비교할 수 없어 멈춘다.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

from control.arena import load_config, config_sha256, integrator_limit_flags, DEFAULT_CONFIG
from control.arena_factory import ArenaFactory, ControllerModel, ARENA_LABELS
from control.arena_tune import Evaluator, parameter_space


def _rss_sampler(evaluator, peaks, stop):
    """작업자 프로세스들의 RSS 최댓값(KiB)을 3초마다 모은다(병렬일 때만 의미가 있다)."""
    while not stop.is_set():
        pool = evaluator._pool
        pids = [str(p) for p in list(getattr(pool, '_processes', None) or {})]
        if pids:
            try:
                out = subprocess.run(['ps', '-o', 'pid=,rss=', '-p', ','.join(pids)],
                                     capture_output=True, text=True).stdout
            except OSError:              # Windows에는 ps가 없다 — 메모리는 기록 없이 넘어간다
                return
            total = 0
            for line in out.splitlines():
                if line.strip():
                    pid, rss = line.split()
                    peaks[pid] = max(peaks.get(pid, 0), int(rss))
                    total += int(rss)
            peaks['sum'] = max(peaks.get('sum', 0), total)
        stop.wait(3.0)


VERDICT_KEYS = ('id', 'failed', 'stop_reason', 'paper_reasons')
NUMERIC_KEYS = ('score', 'window_rmse_velocity', 'window_rmse_z', 'max_omega')


def _close(a, b, rtol):
    if a is None or b is None:
        return a == b
    return abs(a - b) <= rtol*max(abs(a), abs(b)) + 1e-9


def tolerance_problems(scores, reference, objective, reference_objective, config, rtol):
    """판정 일치 + 수치 rtol 검사. 문제 목록(빈 목록이면 합격)."""
    problems = []
    if [e['id'] for e in scores] != [e['id'] for e in reference]:
        return ['scenario ids or order differ']
    if not _close(objective, reference_objective, rtol):
        problems.append(f'objective: {reference_objective:.6g} -> {objective:.6g}')
    for new, ref in zip(scores, reference):
        for key in VERDICT_KEYS:
            if new.get(key) != ref.get(key):
                problems.append(f"{ref['id']} {key}: {ref.get(key)!r} -> {new.get(key)!r}")
        for key in NUMERIC_KEYS:
            if not _close(new.get(key), ref.get(key), rtol):
                problems.append(f"{ref['id']} {key}: {ref.get(key)} -> {new.get(key)}")
    flagged = [sorted(f['case'] for f in integrator_limit_flags(side, config)[1])
               for side in (scores, reference)]
    if flagged[0] != flagged[1]:
        problems.append(f'integrator 1% flags: {flagged[1]} -> {flagged[0]}')
    return problems


def best_index(run_dir, label):
    """최종 최선 평가의 번호. 최선이 캐시 적중(시나리오 없음)이면 같은 지수를 처음 실제로 계산한 평가."""
    lines = [json.loads(l) for l in (Path(run_dir)/f'{label}.jsonl').read_text(encoding='utf-8').splitlines() if l]
    best = min(lines, key=lambda e: (e['objective'], e['index']))
    if best.get('scenarios'):
        return best['index']
    for e in lines:
        if e.get('scenarios') and e['exponents'] == best['exponents']:
            return e['index']
    raise ValueError(f'{label}: no computed evaluation with the best exponents')


def carryover_allowed(config, label):
    """승계 검증 모드: 이 설정이 기준 설정(arena.json)과 controller_model.moment_correction 절만 다르고
    label이 보정 대상이 아니면 기준 설정 해시를 돌려준다(그 해시의 기록을 이 설정에서 다시 계산해도 된다)."""
    from copy import deepcopy
    from control.arena import DEFAULT_CONFIG
    spec = config['controller_model'].get('moment_correction')
    if spec is None or label in spec['applies_to']:
        return None
    base = load_config(DEFAULT_CONFIG)
    stripped = deepcopy(config)
    del stripped['controller_model']['moment_correction']
    return config_sha256(base) if stripped == base else None


def reproduce(config, label, run_dir, index, scenario_workers, rtol=1e-3, carryover=False):
    from control.validation_suite import json_safe
    from models.team_light.control.baseline_v2 import baseline_params
    run_dir = Path(run_dir)
    record = json.loads((run_dir/f'{label}.record.json').read_text(encoding='utf-8'))
    if index == 'best':
        index = best_index(run_dir, label)
    lines = (run_dir/f'{label}.jsonl').read_text(encoding='utf-8').splitlines()
    ref = json.loads(lines[index])
    if ref['index'] != index:
        raise ValueError(f'line {index} holds evaluation {ref["index"]}')
    if not ref.get('scenarios'):
        raise ValueError(f'evaluation {index} is a cache hit — nothing to compare; pick another index')
    allowed = {config_sha256(config)}
    if carryover:
        base_sha = carryover_allowed(config, label)
        if base_sha is None:
            raise ValueError(f'{label}: carry-over check needs a config that differs from arena.json only in '
                             'controller_model.moment_correction, and a controller it does not apply to')
        allowed.add(base_sha)
    if record['config_sha256'] not in allowed:
        raise ValueError('config sha256 differs from the record — this check needs the same config')

    native = baseline_params()
    model = ControllerModel(native)
    names, prior, apply = parameter_space(config, label, ArenaFactory(config, native, model=model).gains)
    values = {n: prior[n]*2.0**e for n, e in zip(names, ref['exponents'])}
    evaluator = Evaluator(config, native, model, label, scenario_workers=scenario_workers)
    peaks, stop = {}, threading.Event()
    sampler = threading.Thread(target=_rss_sampler, args=(evaluator, peaks, stop), daemon=True)
    sampler.start()
    started = time.time()
    try:
        objective, scores = evaluator(apply(values))
    finally:
        stop.set()
        evaluator.close()
    wall = time.time() - started
    # 기록과 같은 직렬화를 거쳐 비교한다(기록은 json_safe → JSON 문자열로 저장됐다).
    scores = json.loads(json.dumps(json_safe(scores), ensure_ascii=False, allow_nan=False))
    diffs = [(a['id'], key) for a, b in zip(scores, ref['scenarios'])
             for key in sorted(set(a) | set(b)) if a.get(key) != b.get(key)]
    if [e['id'] for e in scores] != [e['id'] for e in ref['scenarios']]:
        diffs.append(('<order>', 'scenario ids differ'))
    problems = tolerance_problems(scores, ref['scenarios'], objective, ref['objective'], config, rtol)
    return dict(controller=label, index=index, scenario_workers=scenario_workers, rtol=rtol,
                config_sha256=config_sha256(config), record_config_sha256=record['config_sha256'],
                passed=not problems, tolerance_problems=problems,
                bit_identical=objective == ref['objective'] and not diffs,
                record_git_revision=record.get('git_revision'),
                model_sha256_equal=model.sha256 == record['controller_model_sha256'],
                objective=objective, reference_objective=ref['objective'],
                objective_equal=objective == ref['objective'], differing_fields=diffs,
                scenarios=len(scores), wall_s=round(wall, 1),
                peak_worker_rss_mib={k: round(v/1024) for k, v in peaks.items()})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--controller', required=True, choices=ARENA_LABELS)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--index', default='0', help="평가 번호 또는 'best'(최종 최선 평가)")
    parser.add_argument('--carryover', action='store_true',
                        help='승계 검증: arena.json으로 튜닝한 기록을 arena_v2(모멘트 절만 다름)에서 다시 계산')
    parser.add_argument('--scenario-workers', type=int, default=1)
    parser.add_argument('--rtol', type=float, default=1e-3)
    args = parser.parse_args(argv)
    index = args.index if args.index == 'best' else int(args.index)
    result = reproduce(load_config(args.config), args.controller, args.run_dir, index,
                       args.scenario_workers, rtol=args.rtol, carryover=args.carryover)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if result['passed']:
        print(f"PASS (rtol {args.rtol:g} + identical verdicts); bit-identical: {result['bit_identical']}")
    else:
        print(f"FAIL — {len(result['tolerance_problems'])} problem(s) beyond rtol {args.rtol:g} "
              'or differing verdicts. Do not start tuning on this computer; send this output to kj.')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
