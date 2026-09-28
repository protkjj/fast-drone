"""튜닝 기록 재현 검사 — 기록(<제어기>.jsonl)의 평가 하나를 지금 코드·환경에서 다시 계산해 비트 비교한다.

쓰임:
  - 코드가 바뀐 뒤에도 튜닝 경로 결과가 그대로인지(tune-final → tune-final-2)
  - 다른 컴퓨터(학교)가 이 기록을 재현하는지 — 튜닝을 나눠 돌리기 전에 확인
  - --scenario-workers N 으로 병렬 경로가 순차 기록과 같은지

  python3 -m control.arena_tune_repro --controller V13 \
      --run-dir results/arena/tuning/retune_v3 --index 0 --scenario-workers 3

일치하면 0, 다르면 1로 끝난다. 기록이 캐시 적중(scenarios 없음)이면 비교할 수 없어 멈춘다.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

from control.arena import load_config, config_sha256, DEFAULT_CONFIG
from control.arena_factory import ArenaFactory, ControllerModel, ARENA_LABELS
from control.arena_tune import Evaluator, parameter_space


def _rss_sampler(evaluator, peaks, stop):
    """작업자 프로세스들의 RSS 최댓값(KiB)을 3초마다 모은다(병렬일 때만 의미가 있다)."""
    while not stop.is_set():
        pool = evaluator._pool
        pids = [str(p) for p in list(getattr(pool, '_processes', None) or {})]
        if pids:
            out = subprocess.run(['ps', '-o', 'pid=,rss=', '-p', ','.join(pids)],
                                 capture_output=True, text=True).stdout
            total = 0
            for line in out.splitlines():
                if line.strip():
                    pid, rss = line.split()
                    peaks[pid] = max(peaks.get(pid, 0), int(rss))
                    total += int(rss)
            peaks['sum'] = max(peaks.get('sum', 0), total)
        stop.wait(3.0)


def reproduce(config, label, run_dir, index, scenario_workers):
    from control.validation_suite import json_safe
    from models.team_light.control.baseline_v2 import baseline_params
    run_dir = Path(run_dir)
    record = json.loads((run_dir/f'{label}.record.json').read_text(encoding='utf-8'))
    lines = (run_dir/f'{label}.jsonl').read_text(encoding='utf-8').splitlines()
    ref = json.loads(lines[index])
    if ref['index'] != index:
        raise ValueError(f'line {index} holds evaluation {ref["index"]}')
    if not ref.get('scenarios'):
        raise ValueError(f'evaluation {index} is a cache hit — nothing to compare; pick another index')
    if record['config_sha256'] != config_sha256(config):
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
    return dict(controller=label, index=index, scenario_workers=scenario_workers,
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
    parser.add_argument('--index', type=int, default=0)
    parser.add_argument('--scenario-workers', type=int, default=1)
    args = parser.parse_args(argv)
    result = reproduce(load_config(args.config), args.controller, args.run_dir, args.index,
                       args.scenario_workers)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    ok = result['objective_equal'] and not result['differing_fields'] and result['model_sha256_equal']
    print('REPRODUCED' if ok else 'MISMATCH')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
