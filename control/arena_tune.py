"""같은 예산 튜닝 — kj 작업지시서(2026-09-25) 작업 E. 결과는 항상 **PILOT**.

논문 §5.3: "모든 제어기는 독립된 튜닝 시나리오에서 같은 튜닝 예산을 받으며,
본시험 난수는 튜닝에 사용하지 않는다." 이 스크립트가 그 규칙을 코드로 강제한다.

과거 튜닝(research/tune_gains.py)에서 찾은 불공정 요인과 여기서의 처리:
  - baseline·시작점 평가가 예산 밖이었다      → 첫 평가(사전값)부터 예산에 넣는다
  - 재방문 후보를 다시 돌려 예산을 썼다        → 캐시하되 **똑같이 계상**한다
    (계산은 아끼고 예산 규칙은 모든 제어기에 같게)
  - GSLQR만 불안정 후보를 무료로 거를 수 있었다 → 무료 사전선별 없음
  - 예산이 남은 채 수렴하면 평가 횟수가 달라진다 → 보폭을 되돌려 재시작, 예산을
    **정확히** 다 쓴다(I-4: 평가 횟수가 같아야 한다)
  - 벽시계 의존                               → 솔버는 반복 상한만, 결과에 시간 없음

탐색: 사전값 대비 log₂ 배수 좌표에서의 나침반(compass) 탐색. 한 번의 폴링은 좌표마다
+s·-s 두 후보(2n개)를 전부 평가하고, 가장 좋은 후보가 현재보다 좋으면 옮긴다.
아니면 보폭 s를 반으로(배수 2 → √2 → 2^¼ → 2^⅛), 2^⅛ 다음은 재시작(s=1).
좌표가 이진 분수라 캐시 키가 정확하다.

실행 예(제어기별 병렬 프로세스, 같은 run-dir):
  python -m control.arena_tune --controllers V13 --budget 24 --run-dir results/arena/tuning/pilot24
  python -m control.arena_tune --summarize --run-dir results/arena/tuning/pilot24
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from control.arena import (load_config, config_sha256, build_scenarios, integrator_limit_flags,
                           DEFAULT_CONFIG, ROOT)
from control.arena_factory import ArenaFactory, ControllerModel, ARENA_LABELS, NMPC_LABELS
from control.nmpc_common import PAPER_COST_WEIGHTS
from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate

LABEL = 'PILOT'


# ══════════════════════════════════════════════════════════════════
# 파라미터 공간 — 제어기별 이름·사전값·적용 방법
# ══════════════════════════════════════════════════════════════════

def _gslqr_groups(g):
    """gslqr.json의 14개 대각 가중치를 9개 튜닝 그룹으로 묶는다(구조가 맞는지 확인)."""
    q = [float(v) for v in g['Q_diag']]
    if not (q[1] == q[2] and q[4] == q[5] == q[6] and q[7] == q[8] == q[9]
            and q[10] == q[11] == q[12] == q[13]):
        raise ValueError('gslqr.json Q_diag does not have the grouped structure assumed here')
    q_iz, q_ivx = (float(v) for v in g['Q_integral'])
    return dict(q_z=q[0], q_v_h=q[1], q_v_z=q[3], q_phi=q[4], q_omega=q[7], q_n=q[10],
                r=float(g['R_scale']), q_iz=q_iz, q_ivx=q_ivx)


def parameter_space(config, label, factory_gains):
    """(이름 목록, 사전값 dict, 값 dict → factory overrides 함수)."""
    names = config['tuning']['parameters']['NMPC' if label in NMPC_LABELS else label]
    if label in NMPC_LABELS:
        prior = dict(PAPER_COST_WEIGHTS, Q_z=float(config['nmpc_common']['Q_z']))

        def apply(values):
            weights = {k: values[k] for k in PAPER_COST_WEIGHTS}
            return {label: dict(cost_weights=weights, Q_z=values['Q_z'])}
    elif label == 'GSLQR':
        prior = _gslqr_groups(factory_gains['GSLQR'])

        def apply(values):
            v = values
            return {'GSLQR': dict(
                Q_diag=[v['q_z'], v['q_v_h'], v['q_v_h'], v['q_v_z']] + [v['q_phi']]*3
                       + [v['q_omega']]*3 + [v['q_n']]*4,
                R_scale=v['r'], Q_integral=[v['q_iz'], v['q_ivx']])}
    else:
        g = factory_gains['CPID']
        prior = dict(Kp_vel=g['Kp_vel'], Ki_vel=g['Ki_vel'], Kp_z=g['Kp_z'], Kd_z=g['Kd_z'],
                     Ki_z=g['Ki_z'], Kp_att=1.0, Kd_att=1.0)   # 벡터 게인은 사전값 전체에 곱하는 배수

        def apply(values):
            v = values
            return {'CPID': dict(Kp_vel=v['Kp_vel'], Ki_vel=v['Ki_vel'], Kp_z=v['Kp_z'],
                                 Kd_z=v['Kd_z'], Ki_z=v['Ki_z'],
                                 Kp_att=[v['Kp_att']*a for a in g['Kp_att']],
                                 Kd_att=[v['Kd_att']*a for a in g['Kd_att']])}
    missing = [n for n in names if n not in prior]
    if missing:
        raise ValueError(f'{label}: tuning parameters {missing} have no prior value')
    return list(names), {n: float(prior[n]) for n in names}, apply


# ══════════════════════════════════════════════════════════════════
# 목적함수 — 모든 제어기에 같은 식
# ══════════════════════════════════════════════════════════════════

class Evaluator:
    """튜닝 시나리오 전체를 돌려 목적함수 하나를 낸다(시간·난수 무관)."""

    def __init__(self, config, native, model, label):
        self.config, self.native, self.model, self.label = config, native, model, label
        self.scenarios = build_scenarios(config, model.cp, native,
                                         scenarios=config['tuning']['scenarios'])
        self.limits = Acceptance(**config['acceptance'])
        self.paper = PaperCriteria(**config['paper_criteria'])
        self.penalty = float(config['tuning']['objective']['failure_penalty'])

    def __call__(self, overrides):
        import control.validation_suite as suite
        factory = ArenaFactory(self.config, self.native, overrides=overrides, model=self.model)
        scores = []
        for scenario in self.scenarios:
            entry = dict(id=scenario.id)
            try:
                row, result, log = suite.run_trial(factory, self.label, scenario.profile,
                                                   scenario.cases[0], self.limits)
                paper = paper_evaluate(result, scenario.profile, self.paper, window=scenario.window,
                                       solve_log=log, n_max=self.native['n_max'])
                failed = bool(row['stop_reason']) or paper['paper_failed']
                entry.update(stop_reason=row['stop_reason'], paper_reasons=paper['paper_reasons'],
                             window_rmse_velocity=paper['window_rmse_velocity'],
                             window_rmse_z=paper['window_rmse_z'], max_omega=row['max_omega'],
                             trajectory_sha256=row['trajectory_sha256'],
                             integrators=row.get('integrators'))
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                failed = True
                entry.update(stop_reason=f'{type(exc).__name__}: {exc}')
            value = (self.penalty if failed or entry.get('window_rmse_velocity') is None
                     else entry['window_rmse_velocity'] + entry['window_rmse_z'])
            entry.update(failed=failed, score=float(value))
            scores.append(entry)
        return float(np.mean([e['score'] for e in scores])), scores


# ══════════════════════════════════════════════════════════════════
# 탐색 — 결정적 나침반 탐색, 예산을 정확히 다 쓴다
# ══════════════════════════════════════════════════════════════════

def compass_search(n, budget, evaluate, initial_step=2.0, min_step=1.05):
    """log₂ 배수 좌표 e(길이 n)에서 탐색. evaluate(e) → 목적함수(작을수록 좋음).

    evaluate는 캐시·로그를 책임진다(여기선 '요청'만 센다). 반환: (최선 e, 최선값, 이력).
    """
    s0 = float(np.log2(initial_step))
    s_min = float(np.log2(min_step))
    center = tuple([0.0]*n)
    best = evaluate(center)
    spent, s, restarts, history = 1, s0, 0, []
    while spent < budget:
        poll = []
        for i in range(n):
            for sign in (+1.0, -1.0):
                if spent >= budget:
                    break
                cand = list(center)
                cand[i] += sign*s
                cand = tuple(cand)
                poll.append((evaluate(cand), cand))
                spent += 1
        history.append(dict(center=center, step_exponent=s, evaluated=len(poll)))
        if poll:
            value, cand = min(poll, key=lambda item: item[0])
            if value < best - 1e-12:
                center, best = cand, value
                continue
        s /= 2.0
        if s < s_min:
            s, restarts = s0, restarts + 1         # 예산이 남으면 보폭을 되돌려 계속
    return center, best, dict(spent=spent, restarts=restarts, polls=history)


# ══════════════════════════════════════════════════════════════════
# 실행·기록·재개
# ══════════════════════════════════════════════════════════════════

def _environment():
    from control.validation_suite import environment_fingerprint, git_state
    revision, dirty = git_state()
    return dict(environment=environment_fingerprint(), git_revision=revision, git_dirty=dirty)


def _write_json(path, value):
    from control.validation_suite import json_safe
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(json_safe(value), ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                   encoding='utf-8')
    tmp.replace(path)


def tune_controller(config, label, budget, run_dir, native=None, model=None):
    """한 제어기를 예산만큼 튜닝. 같은 run_dir에 로그가 있으면 이어서 한다."""
    from models.team_light.control.baseline_v2 import baseline_params
    native = native if native is not None else baseline_params()
    model = model if model is not None else ControllerModel(native)
    gains = ArenaFactory(config, native, model=model).gains
    names, prior, apply = parameter_space(config, label, gains)
    evaluator = Evaluator(config, native, model, label)
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path, record_path = run_dir/f'{label}.jsonl', run_dir/f'{label}.record.json'
    logged = ([json.loads(line) for line in log_path.read_text(encoding='utf-8').splitlines() if line]
              if log_path.exists() else [])
    cache, counter = {}, {'index': 0}
    search = config['tuning']['search']
    record = dict(label=LABEL, controller=label, budget=int(budget), spent=0,
                  parameter_names=names, prior=prior,
                  scenario_ids=[s.id for s in evaluator.scenarios],
                  main_scenario_ids=[s['id'] for s in config['scenarios']],
                  objective=config['tuning']['objective'], search=search,
                  seeds=dict(tuning_range=config['seeds']['tuning'], main_range=config['seeds']['main'],
                             used=[]),
                  config_sha256=config_sha256(config),
                  controller_model_sha256=model.sha256, status='running', **_environment())

    def values_of(e):
        return {name: prior[name]*2.0**exp for name, exp in zip(names, e)}

    def evaluate(e):
        index = counter['index']
        counter['index'] += 1
        if index < len(logged):                    # 재개: 로그를 그대로 재생한다
            entry = logged[index]
            if tuple(entry['exponents']) != tuple(e):
                raise RuntimeError(f'{label}: resume log diverges at evaluation {index} — '
                                   'different config/code; start a new run-dir')
            cache.setdefault(tuple(e), entry['objective'])
            return entry['objective']
        hit = tuple(e) in cache
        if hit:
            objective, scores = cache[tuple(e)], None
        else:
            objective, scores = evaluator(apply(values_of(e)))
            cache[tuple(e)] = objective
        entry = dict(index=index, exponents=list(e), values=values_of(e), objective=objective,
                     cache_hit=hit, scenarios=scores)
        with log_path.open('a', encoding='utf-8') as stream:
            from control.validation_suite import json_safe
            stream.write(json.dumps(json_safe(entry), ensure_ascii=False, allow_nan=False) + '\n')
        record.update(spent=index + 1)
        _write_json(record_path, record)
        print(f'  [{label}] eval {index + 1}/{budget} objective={objective:.6g}'
              f'{" (cache)" if hit else ""}', flush=True)
        return objective

    best_e, best, info = compass_search(len(names), int(budget), evaluate,
                                        initial_step=float(search['initial_step']),
                                        min_step=float(search['min_step']))
    prior_objective = cache[tuple([0.0]*len(names))]
    record.update(spent=info['spent'], restarts=info['restarts'], best_exponents=list(best_e),
                  best_values=values_of(best_e), best_objective=best,
                  prior_objective=prior_objective, status='complete',
                  finished_utc=datetime.now(timezone.utc).isoformat())
    _write_json(record_path, record)
    return record


def check_tuning_records(records, config):
    """불변식 I-4 — 위반 목록(빈 목록이면 통과)."""
    bad = []
    if not records:
        return ['no tuning records']
    budgets = {r['controller']: r['budget'] for r in records}
    if len(set(budgets.values())) != 1:
        bad.append(f'budgets differ: {budgets}')
    for r in records:
        if r.get('status') != 'complete':
            bad.append(f"{r['controller']}: status {r.get('status')}")
        if r['spent'] != r['budget']:
            bad.append(f"{r['controller']}: spent {r['spent']} != budget {r['budget']}")
        if set(r['scenario_ids']) & {s['id'] for s in config['scenarios']}:
            bad.append(f"{r['controller']}: tuning used main-test scenarios")
        lo_t, hi_t = r['seeds']['tuning_range']
        lo_m, hi_m = config['seeds']['main']
        if not (hi_m < lo_t or hi_t < lo_m):
            bad.append(f"{r['controller']}: tuning seed range overlaps main-test seeds")
        if any(lo_m <= s <= hi_m or not lo_t <= s <= hi_t for s in r['seeds']['used']):
            bad.append(f"{r['controller']}: used a seed outside the tuning range")
    for key in ('scenario_ids', 'objective', 'search'):
        if len({json.dumps(r[key], sort_keys=True) for r in records}) != 1:
            bad.append(f'{key} differs between controllers')
    return bad


def _evaluation_entries(run_dir):
    """튜닝 로그(<label>.jsonl)의 시나리오별 항목을 평평하게 모은다(적분기 1% 규칙 검사용).
    캐시 적중 평가는 새로 돌리지 않았으므로 항목이 없다."""
    entries = []
    for path in sorted(Path(run_dir).glob('*.jsonl')):
        controller = path.stem
        for line in path.read_text(encoding='utf-8').splitlines():
            if not line:
                continue
            evaluation = json.loads(line)
            for score in evaluation.get('scenarios') or []:
                entries.append(dict(score, controller=controller, evaluation=evaluation['index']))
    return entries


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--controllers', nargs='+', choices=ARENA_LABELS)
    parser.add_argument('--budget', type=int, help='evaluations per controller (default: config)')
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--summarize', action='store_true',
                        help='collect <label>.record.json files, check I-4, write summary.json')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    if args.summarize:
        records = [json.loads(p.read_text(encoding='utf-8'))
                   for p in sorted(args.run_dir.glob('*.record.json'))]
        violations = check_tuning_records(records, config)
        threshold, flags = integrator_limit_flags(_evaluation_entries(args.run_dir), config)
        summary = dict(label=LABEL, run_dir=str(args.run_dir), i4_violations=violations,
                       integrator_limit=dict(threshold=threshold, flagged=len(flags), flags=flags),
                       controllers={r['controller']: dict(
                           budget=r['budget'], spent=r['spent'], status=r['status'],
                           prior_objective=r.get('prior_objective'),
                           best_objective=r.get('best_objective'),
                           best_values=r.get('best_values')) for r in records})
        _write_json(args.run_dir/'summary.json', summary)
        print(json.dumps(summary, indent=2, ensure_ascii=False, default=str))
        return 0 if not violations else 1
    if not args.controllers:
        parser.error('--controllers is required unless --summarize')
    budget = args.budget or int(config['tuning']['budget'])
    from models.team_light.control.baseline_v2 import baseline_params
    native = baseline_params()
    model = ControllerModel(native)
    for label in args.controllers:
        print(f'== {LABEL} tuning {label}: budget {budget} ==', flush=True)
        tune_controller(config, label, budget, args.run_dir, native=native, model=model)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
