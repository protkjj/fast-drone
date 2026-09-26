"""M17 양추력 하한의 형태 비교 — kj 결정(2026-09-26 저녁)의 근거. 결과는 **PILOT**(우위 결론 없음).

kj는 처음에 (나) '노드별 하한 U_k ≥ floor(X_k)'를 골랐다(F13의 노드별 f ≥ 0과 대칭).
적용 전 확인에서 이 정확한 부등식 제약이 이미 통과하던 V_H 돌풍 두 사례를 새로 실패시켰다
(IPOPT 반복 상한 30 초과). 임무 실패는 어느 형태로도 고쳐지지 않았다. 그래서 kj는 (A) '측정 상태
하한 유지'로 결정했다. 이 도구가 그 근거 표를 재현한다. M17만 돌리고, 경기장 설정 위에서 하한
형태만 바꾼다.

  per_state            경기장 설정(rotor_floor='positive_thrust'). 측정 상태 하나로 모든 노드의 lbx를 올린다.
  per_node             노드별 부등식 제약 U_k ≥ floor(X_k)(rotor_floor='positive_thrust_per_node'). kj 결정 (나).
  per_node_maxiter100  per_node + IPOPT 반복 상한 100(M17만). I-3 대칭을 깨므로 참고용이다.
  predicted_box        부등식 없이, 풀이마다 lbx를 노드별로 올린다.
                       노드 0 = 측정 상태, 노드 k ≥ 1 = 직전 계획(웜스타트)의 예측 상태.

변형마다 새 프로세스로 돌린다. M17 NLP가 ~2 GB이기 때문이다.

실행: python -m control.arena_m17_floor_variants <변형> <사례>   → results/arena/m17_floor_variants/<변형>__<사례>.json
      python -m control.arena_m17_floor_variants --aggregate     → results/arena/M17_FLOOR_VARIANTS.md
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import weakref

import numpy as np

from control.arena import load_config, config_sha256, build_scenarios, DEFAULT_CONFIG, ROOT

VARIANTS = ('per_state', 'per_node', 'per_node_maxiter100', 'predicted_box')
OUT = ROOT/'results'/'arena'/'m17_floor_variants'


class PredictedBox:
    """풀이마다 U 칸 하한을 노드별 양추력 회전수로 올린다. 노드 0은 측정 상태, k ≥ 1은 웜스타트(직전 계획) 상태다."""

    def __init__(self, solver, nmpc):
        self._solver, self._nmpc = solver, weakref.ref(nmpc)

    def stats(self):
        return self._solver.stats()

    def __call__(self, **kw):
        from control.nmpc import positive_thrust_rate_floor
        nmpc = self._nmpc()
        nx, nu, N = nmpc.nx, nmpc.nu, nmpc.N
        stride = nx + nu
        x0 = np.asarray(kw['x0'], dtype=float).ravel()
        lbx = np.array(kw['lbx'], dtype=float).ravel()
        x_meas = np.asarray(kw['p'], dtype=float).ravel()[:nx]
        for k in range(N):
            X = x_meas if k == 0 else x0[k*stride:k*stride + nx]
            sl = slice(k*stride + nx, k*stride + stride)
            lbx[sl] = np.maximum(lbx[sl], positive_thrust_rate_floor(nmpc.p, X))
        return self._solver(**dict(kw, lbx=lbx))


def variant_config(config, variant):
    cfg = deepcopy(config)
    m17 = cfg['controllers']['M17']
    if variant == 'per_state':
        m17['rotor_floor'] = 'positive_thrust'
    elif variant in ('per_node', 'per_node_maxiter100'):
        m17['rotor_floor'] = 'positive_thrust_per_node'
        if variant == 'per_node_maxiter100':
            cfg['nmpc_common']['max_iter'] = 100          # 이 도구는 M17만 짓는다
    elif variant == 'predicted_box':
        m17.pop('rotor_floor', None)
    else:
        raise ValueError(f'unknown variant {variant!r}')
    return cfg


def run(config, variant, case_id):
    import control.validation_suite as suite
    from control.arena_factory import ArenaFactory
    from control.validation_metrics import Acceptance
    cfg = variant_config(config, variant)
    factory = ArenaFactory(cfg)
    scenario = build_scenarios(cfg, factory.cp, factory.p, only=[case_id])[0]
    if variant == 'predicted_box':
        real = factory.make_for_profile

        def make(label, profile, case=None):
            ctrl = real(label, profile, case)
            ctrl.nmpc.solver = PredictedBox(ctrl.nmpc.solver, ctrl.nmpc)
            return ctrl
        factory.make_for_profile = make
    row, _, log = suite.run_trial(factory, 'M17', scenario.profile, scenario.cases[0],
                                  Acceptance(**cfg['acceptance']))
    failures = [e for e in log if not e['accepted']]
    return dict(variant=variant, case=case_id, stop_reason=row['stop_reason'],
                simulated_seconds=row['simulated_seconds'], T_total=float(scenario.profile.T_total),
                solves=len(log), failures=len(failures),
                max_iter_seen=max((e['iter_count'] or 0) for e in log) if log else None,
                max_omega=row['max_omega'], trajectory_sha256=row['trajectory_sha256'],
                config_sha256=config_sha256(cfg))


def aggregate(out=OUT):
    from control.validation_suite import git_state
    records = [json.loads(p.read_text(encoding='utf-8')) for p in sorted(Path(out).glob('*.json'))]
    cases = list(dict.fromkeys(r['case'] for r in records))
    by = {(r['variant'], r['case']): r for r in records}
    revision, dirty = git_state()
    lines = ['# M17 양추력 하한 형태 비교(PILOT)', '',
             '**PILOT** — kj 결정(2026-09-26 저녁)의 근거다. M17만 돌렸고 우위 결론은 없다. 결정은 **(A) 측정 상태 하한 '
             '유지**다. 노드별 정확한 부등식은 V_H 돌풍 두 사례를 새로 실패시켰고, 임무 실패는 어느 형태로도 '
             '고쳐지지 않았다(ρ > 1 구간이 원인 — MISSION_RHO.md).', '',
             '재현: `python -m control.arena_m17_floor_variants <변형> <사례>`, 표는 '
             f'`python -m control.arena_m17_floor_variants --aggregate` · git `{revision}` dirty={dirty}', '',
             '칸 = 끝까지 또는 정지 시각 · 실패 풀이/전체 · 반복 최대 · |ω|max(rad/s). IPOPT 반복 상한은 per_node_maxiter100만 100, '
             '나머지는 경기장 공통값 30이다.', '',
             '| 변형 | ' + ' | '.join(cases) + ' |', '|---|' + '---|'*len(cases)]
    for v in VARIANTS:
        cells = []
        for c in cases:
            r = by.get((v, c))
            if r is None:
                cells.append('—')
                continue
            end = '끝까지' if r['stop_reason'] is None else f"{r['simulated_seconds']:.2f} s 정지"
            cells.append(f"{end} · {r['failures']}/{r['solves']} · {r['max_iter_seen']} · {r['max_omega']:.2g}")
        lines.append(f'| {v} | ' + ' | '.join(cells) + ' |')
    Path(out).parent.joinpath('M17_FLOOR_VARIANTS.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))


def main(argv=None):
    from control.validation_suite import write_json, environment_fingerprint, git_state
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('variant', nargs='?', choices=VARIANTS)
    parser.add_argument('case', nargs='?')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--aggregate', action='store_true')
    args = parser.parse_args(argv)
    if args.aggregate:
        return aggregate()
    if not (args.variant and args.case):
        parser.error('give <variant> <case>, or --aggregate')
    record = run(load_config(args.config), args.variant, args.case)
    revision, dirty = git_state()
    record.update(created_utc=datetime.now(timezone.utc).isoformat(), git_revision=revision, git_dirty=dirty,
                  environment=environment_fingerprint(),
                  command=f'python -m control.arena_m17_floor_variants {args.variant} {args.case}')
    OUT.mkdir(parents=True, exist_ok=True)
    write_json(OUT/f'{args.variant}__{args.case}.json', record)
    print(json.dumps({k: record[k] for k in ('variant', 'case', 'stop_reason', 'simulated_seconds', 'failures',
                                             'solves', 'max_iter_seen', 'max_omega')}))
    return record


if __name__ == '__main__':
    main()
