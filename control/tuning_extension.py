"""튜닝 예산 연장 판정(kj 규칙, 센서 재튜닝 전에 확정 — 2026-09-28 밤). 기록을 **읽기만** 한다.

규칙(보완판)
  - 제어기마다 평가 90회 시점의 최선과 120회 시점의 최선(목적함수 기준 누적 최선)을 비교한다.
  - 연속 성분 C = 그 최선 평가에서 **실패하지 않은 시나리오 점수(창 RMSE 속도+고도)의 평균** — 실패 벌점(1000)을 뺀 값.
    개선율 = (C₉₀ − C₁₂₀) / C₉₀.
  - 두 시점 사이에 최선 게인의 실패 시나리오 수가 줄었으면 개선율과 상관없이 '개선 있음'.
  - 다섯 제어기 중 하나라도 (개선율 > 1%) 또는 (실패 수 감소)면 → 다섯 모두 180회까지 이어서 돌린다.
보완 전 규칙(최선 목적함수의 개선율)은 벌점이 섞이면 개선율이 작게 보여 실패 벌점이 있는 제어기의 미수렴을
못 잡았다(CPID retune_v3: 333.59 → 333.59, 0.00%) — 그래서 연속 성분과 실패 수로 나눴다(보고서 25절).

최선이 캐시 적중(시나리오 점수 없음)이면 같은 지수를 처음 실제로 계산한 평가의 점수를 쓴다.

    python3 -m control.tuning_extension --run-dirs results/arena/tuning/<폴더> [--at 90 120]
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

THRESHOLD = 0.01


def load_log(path):
    return [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]


def best_entry_at(log, k):
    """평가 k회까지(1-based) 목적함수 최선 평가와 그 시나리오 점수(캐시 적중이면 원래 계산)."""
    head = log[:k]
    best = min(head, key=lambda e: (e['objective'], e['index']))
    if best.get('scenarios'):
        return best, best['scenarios']
    for e in log:
        if e.get('scenarios') and e['exponents'] == best['exponents']:
            return best, e['scenarios']
    raise ValueError(f"evaluation {best['index']}: no computed scenarios for its exponents")


def continuous_part(scenarios):
    ok = [s['score'] for s in scenarios if not s['failed']]
    return (float(np.mean(ok)) if ok else None), sum(bool(s['failed']) for s in scenarios)


def verdict(log, at=(90, 120), threshold=THRESHOLD):
    k0, k1 = at
    if len(log) < k1:
        raise ValueError(f'only {len(log)} evaluations, need {k1}')
    (b0, s0), (b1, s1) = best_entry_at(log, k0), best_entry_at(log, k1)
    (c0, f0), (c1, f1) = continuous_part(s0), continuous_part(s1)
    rate = None if (c0 is None or c1 is None or c0 == 0) else (c0 - c1)/c0
    fewer_failures = f1 < f0
    improved = bool(fewer_failures or (rate is not None and rate > threshold))
    return dict(best_index_at=[b0['index'], b1['index']], objective_at=[b0['objective'], b1['objective']],
                continuous_at=[c0, c1], failures_at=[f0, f1], continuous_rate=rate,
                fewer_failures=fewer_failures, failures_increased=f1 > f0, improved=improved)


def decide(run_dirs, controllers=('V13', 'M17', 'F13', 'GSLQR', 'CPID'), at=(90, 120)):
    rows = {}
    for label in controllers:
        for folder in run_dirs:
            path = Path(folder)/f'{label}.jsonl'
            if path.exists():
                log = load_log(path)
                if len(log) < at[1]:
                    rows[label] = dict(run_dir=str(folder), incomplete=True, evaluations=len(log))
                else:
                    rows[label] = dict(verdict(log, at), run_dir=str(folder), incomplete=False)
                break
    missing = [c for c in controllers if c not in rows]
    incomplete = [c for c, r in rows.items() if r['incomplete']]
    # 규칙은 다섯 제어기를 모두 본다 — 하나라도 미완료·기록 없음이면 판정을 내리지 않는다(None = 보류)
    extend = None if (missing or incomplete) else any(r['improved'] for r in rows.values())
    return dict(rule='extend all to 180 if any controller: continuous-part improvement > 1% '
                     f'between evaluations {at[0]} and {at[1]}, or fewer failed scenarios',
                controllers=rows, missing=missing, incomplete=incomplete, extend_to_180=extend)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-dirs', type=Path, nargs='+', required=True)
    parser.add_argument('--at', type=int, nargs=2, default=[90, 120])
    args = parser.parse_args(argv)
    result = decide(args.run_dirs, at=tuple(args.at))
    print('| 제어기 | 최선(90) | 최선(120) | 연속 성분 C₉₀ → C₁₂₀ | 개선율 | 실패 수 | 개선 있음 |')
    print('|---|---:|---:|---|---:|---|---|')
    for label, r in result['controllers'].items():
        if r['incomplete']:
            print(f"| {label} | 미완료({r['evaluations']}회) | | | | | |")
            continue
        c0, c1 = r['continuous_at']
        rate = '—' if r['continuous_rate'] is None else f"{100*r['continuous_rate']:.2f}%"
        print(f"| {label} | {r['objective_at'][0]:.4f} | {r['objective_at'][1]:.4f} | "
              f"{c0 if c0 is None else round(c0, 4)} → {c1 if c1 is None else round(c1, 4)} | {rate} | "
              f"{r['failures_at'][0]} → {r['failures_at'][1]} | {'예' if r['improved'] else '아니오'} |")
    if result['missing']:
        print(f"기록 없음: {result['missing']}")
    if result['extend_to_180'] is None:
        print('\n판정: 보류 — 다섯 제어기의 120회 기록이 모두 있어야 한다 '
              f"(기록 없음 {result['missing']}, 미완료 {result['incomplete']})")
    else:
        print(f"\n판정: {'다섯 모두 180회까지 이어서' if result['extend_to_180'] else '120회에서 끝냄'}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
