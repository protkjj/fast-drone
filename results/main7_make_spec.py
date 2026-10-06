"""tune7 본 실험 설정(spec)을 PC별 제어기 묶음으로 만든다 — 겹치기 실행(kj 결정 2026-10-06).

학교 PC마다 자기가 튜닝한 제어기만 맡는다(180회 연장이 끝나면 그 PC가 바로 본 실험을 시작).
PC끼리 기록을 옮기지 않아도 되게 하려는 것이다. 설정끼리는 **제어기 목록·기록 해시·사다리 유무만** 다르고
나머지(시나리오, 시드, 기준 설정)는 같다 — 시행 하나는 (시나리오, 제어기, 시드) 단위로 독립이라 나중에 합쳐 쓴다.

사다리 묶음은 V13 기록으로 돈다. V13이 없는 설정에 사다리를 남기면 튜닝 안 된 V13이 조용히 돌기 때문에
V13이 없는 설정에서는 사다리 변형을 비운다.

설정 파일은 configs/ 밖(results/main7/)에 쓴다. configs/*.json은 실행 코드 해시(source_hashes)에 들어간다.
control/·configs/는 건드리지 않는다(튜닝 기록이 그 해시를 고정한다).

    python school/main7_make_spec.py --root <fds> --controllers V13 CPID --out <fds>/results/main7/part.spec.json

기록 파일이 없거나 complete·180회가 아니면 거부한다.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = 'configs/main_experiment_sensor_candidate_v6.json'
BASE = 'configs/arena_tune7.json'
BASE_SHA = 'e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801'
RUN_DIR = 'results/arena/tuning/tune7_final'
BUDGET = 180
CONTROLLERS = ('V13', 'M17', 'F13', 'GSLQR', 'CPID')
SENSOR_SEEDS = list(range(1000, 1020))          # 비교 묶음 20개(kj 결정 B, 2026-10-01)
LADDER_SEEDS = list(range(1000, 1010))          # 사다리 10개, 비교 시드의 부분집합


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def make_spec(labels, root=ROOT, run_dir=RUN_DIR, budget=BUDGET):
    labels = [l for l in CONTROLLERS if l in labels]
    spec = deepcopy(json.loads((root/TEMPLATE).read_text(encoding='utf-8')))
    spec['purpose'] = (f'tune7 main experiment part ({", ".join(labels)}). Parts differ only in controllers, '
                       'record hashes and the ladder (only the part with V13).')
    spec['base_config'] = dict(path=BASE, config_sha256=BASE_SHA)
    spec['controllers'] = labels
    records = {}
    for label in labels:
        path = root/run_dir/f'{label}.record.json'
        if not path.exists():
            raise SystemExit(f'{path}: missing (download the final record first)')
        record = json.loads(path.read_text(encoding='utf-8'))
        if record.get('status') != 'complete' or record.get('spent') != budget or record.get('budget') != budget:
            raise SystemExit(f"{label}: record is not complete at {budget} "
                             f"(status {record.get('status')}, {record.get('spent')}/{record.get('budget')})")
        records[label] = dict(sha256=sha256_file(path), tag='tune-final-7', budget=budget)
    spec['tuned'] = dict(run_dir=run_dir, records=records,
                         note=f'Final tune7 records (budget {budget}; 180 after the pre-registered extension).')
    spec['sensor_seeds'] = SENSOR_SEEDS
    spec['ladder_sensor_seeds'] = LADDER_SEEDS
    spec['sensor_seed_note'] = 'kj decision B (2026-10-01): comparison 1000-1019, ladder 1000-1009. Fixed before any main result.'
    if 'V13' not in labels:
        spec['ladder'] = dict(spec['ladder'], variants={},
                              note='Ladder runs only in the part with V13 (it uses the V13 record).')
    spec['part'] = dict(controllers=labels)
    return spec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--controllers', nargs='+', choices=CONTROLLERS, required=True)
    parser.add_argument('--root', type=Path, default=ROOT, help='fds checkout (default: this file\'s repo)')
    parser.add_argument('--records', default=RUN_DIR)
    parser.add_argument('--budget', type=int, default=BUDGET, help='180 for the real run; other values only for dry runs')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    spec = make_spec(args.controllers, args.root, args.records, args.budget)
    out = args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(spec, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    print(f'wrote {out} ({", ".join(spec["controllers"])}) sha256 {sha256_file(out)}')


if __name__ == '__main__':
    main()
