"""M17 솔버 실패 진단 — kj 결정(2026-09-26, 결정 필요 6-18). 결과는 **PILOT**. 진단 전용이라
경기장 설정·코드는 바꾸지 않는다.

스모크에서 M17만 V_H 외란·제동·감속에서 IPOPT 반복 상한(30)에 연속으로 걸려 멈췄다
(|ω| ≤ 0.43, 비행은 조용했다). 같은 사례에서 V13·F13은 반복 5회 안팎이었다. 가설은 셋이다.

  H1  회전수 명령 한계가 활성이다. 85 m/s 트림 회전수 여유가 9%뿐이다.
  H2  NLP 안의 추진 곡선이 C1(PCHIP)이다. 매듭마다 2차 미분이 튀어 정확한 헤시안이 불연속이다.
      85 m/s 트림의 로터 3·4는 매듭 J=1.2094에서 0.004 거리다. NLP에 곡선을 품은 것은 M17뿐이다.
  H3  입력 편차 비용의 기준이 호버 회전수(≈0.38·n_max)다(논문 식 17). 85 m/s 트림은 0.86~0.91·n_max라
      멀고, 그 아래 0.813·n_max부터는 추력 0 평탄 구간이다.

변형(각각 새 프로세스로 돌린다 — M17 NLP 하나가 ~1.9 GiB이고 곡선 패치가 프로세스 전역이다):
  V0  기준 설정. 참조(results/arena/smoke_reference.json) 궤적 해시를 비트 재현하는지 먼저 본다.
  V1  max_iter 100. 반복 예산 부족인지(어렵지만 풀리는 문제인지) 본다.
  V2  M17 NLP만 같은 매듭의 C2 곡선(scipy CubicSpline)으로 바꾼다. 플랜트는 원래 곡선이다
      (플랜트 모듈은 곡선 함수를 import 때 묶는다 — 패치 전후 플랜트 xdot 비트 동일로 확인).
  V3  입력 편차 기준 = 시작 속도의 명목 모델 트림 회전수(논문 식 17의 호버 대신).
  V4  (사후 추가, 2026-09-26 — V0~V3이 모두 같은 시각에 멈춘 뒤 정함) M17 명령 하한 = 실측 축방향
      속도에서 추력이 0이 되는 회전수(팀 `propeller_curve.supported_rate_bounds`의 하한과 같은 개념).
      실패 풀이의 계획이 전부 추력 0 평탄 구간(기울기 0, fmax 꺾임)에 들어갔다는 관찰(H4)을 가른다.
모든 변형에 솔버 프록시를 달아 풀이마다 U 칸의 한계 활성(H1)과 마지막 반복의 inf_pr·inf_du를
기록한다. 프록시는 읽기만 해서 결과에 영향이 없다(V0가 참조 해시를 재현하는지로 확인).

판정 규칙(미리 정함, 결과를 보기 전에 계획서에 적었다):
  H1 지지  V0 실패 풀이의 절반 이상에서 U 한계가 활성이고, V1로도 멈춘다
  H2 지지  V0가 멈춘 사례를 V2가 끝까지 간다(곡선 차이 ≤ 2%, 넘으면 '교란됨')
  H3 지지  V0가 멈춘 사례를 V3가 끝까지 간다
  반복 부족 V1이 끝까지 간다
  H4 지지  V0가 멈춘 사례를 V4가 끝까지 간다(사후 가설 — 실행 전에 이 규칙을 적었다)
  여럿이 함께 성립할 수 있다. H2·H3이면 경기장 수정은 kj 결정 뒤에 한다(kj 선택).

실행: python -m control.arena_m17_diagnosis                 # 전 변형(하위 프로세스) + 집계
      python -m control.arena_m17_diagnosis --variant V2    # 한 변형만(집계 없이 JSON만)
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import weakref

import numpy as np

from control.arena import ROOT

LABEL = 'PILOT'
CASES = ('gust_lateral_p10_VH', 'gust_vertical_m5_VH', 'ref_brake_VH_VL_rho1')
VARIANTS = ('V0', 'V1', 'V2', 'V3', 'V4')
ZERO_THRUST_MARGIN = 1e-3           # V4 하한 = 양추력 회전수 × (1 + 이 값)
OUT = ROOT/'results'/'arena'/'m17_diagnosis'
OPERATING_J = (0.40, 1.28)          # V_L~V_H 트림의 운용 J(0.41~1.272)를 감싼다
# 한계 근접 판정: n_max 대비 상대 거리. 처음엔 1e-6이었는데, V0 측풍 한 건을 돌려 보니 실패 풀이의
# 계획 명령이 n_max에 1.2e-5까지 붙고도 '활성'으로 안 잡혔다 — IPOPT 내점법 반복점은 한계에
# 정확히 닿지 않는다(bound_push). 그래서 1e-4로 정했다(결과 하나를 본 뒤의 조정, 보고서에 명기).
BOUND_TOL = 1e-4


class SolverProxy:
    """IPOPT 솔버를 감싸 풀이마다 U 칸의 한계 활성과 마지막 반복의 잔차를 기록한다.

    `_solve`와 SolverMonitor가 둘 다 `nmpc.solver(...)`·`nmpc.solver.stats()`를 부르므로 두 가지를
    그대로 넘긴다. NMPC는 약한 참조로 잡는다(NMPC → 프록시 → NMPC 순환을 만들지 않게).
    """

    def __init__(self, solver, nmpc, raise_lower_bound=False):
        from models.team_light.control.propeller_curve import positive_thrust_j_limit, uses_curve
        self._solver = solver
        self._nmpc = weakref.ref(nmpc)
        self.records = []
        self.j_limit = positive_thrust_j_limit(nmpc.p) if uses_curve(nmpc.p) else None
        self.raise_lower_bound = raise_lower_bound      # V4: 명령 하한을 양추력 회전수로

    def stats(self):
        return self._solver.stats()

    def __call__(self, **kw):
        nmpc = self._nmpc()
        if self.raise_lower_bound and self.j_limit:
            # V4(진단 전용): 실측 축방향 속도에서 추력이 0이 되는 회전수보다 낮은 명령을 계획하지 못하게 한다
            from scipy.spatial.transform import Rotation
            nx, nu, N = nmpc.nx, nmpc.nu, nmpc.N
            x_m = np.asarray(kw['p'], dtype=float).ravel()[:nx]
            va = max(float((Rotation.from_quat(x_m[6:10]).as_matrix().T @ x_m[3:6])[0]), 0.0)
            floor = min(2*np.pi*va/(nmpc.p['D_prop']*self.j_limit)*(1 + ZERO_THRUST_MARGIN),
                        0.999*float(nmpc.p['n_max']))
            lbx = np.array(kw['lbx'], dtype=float).ravel()
            stride = nx + nu
            for k in range(N):
                sl = slice(k*stride + nx, k*stride + nx + nu)
                lbx[sl] = np.maximum(lbx[sl], floor)
            kw = dict(kw, lbx=lbx)
        sol = self._solver(**kw)
        stats = self._solver.stats()
        x = np.asarray(sol['x'], dtype=float).ravel()
        lbx = np.asarray(kw['lbx'], dtype=float).ravel()
        ubx = np.asarray(kw['ubx'], dtype=float).ravel()
        nx, nu, N = nmpc.nx, nmpc.nu, nmpc.N
        stride = nx + nu
        idx = np.concatenate([np.arange(k*stride + nx, k*stride + nx + nu) for k in range(N)])
        n_max = float(nmpc.p['n_max'])
        u = x[idx]
        upper = (ubx[idx] - u) <= BOUND_TOL*n_max
        lower = (u - lbx[idx]) <= BOUND_TOL*n_max
        # 계획 명령이 추력 0 평탄 구간(J > 양추력 한계)에 들어간 칸 수 — 실측 축방향 속도로 근사
        x_meas = np.asarray(kw['p'], dtype=float).ravel()[:nx]
        from scipy.spatial.transform import Rotation
        va = max(float((Rotation.from_quat(x_meas[6:10]).as_matrix().T @ x_meas[3:6])[0]), 0.0)
        n_zero = 2*np.pi*va/(nmpc.p['D_prop']*self.j_limit) if self.j_limit else 0.0
        iterations = stats.get('iterations') or {}
        inf_pr = iterations.get('inf_pr') or [None]
        inf_du = iterations.get('inf_du') or [None]
        self.records.append(dict(
            t=float(nmpc._t_now), status=stats.get('return_status'), iter_count=stats.get('iter_count'),
            active_upper=int(upper.sum()), active_lower=int(lower.sum()),
            first_step_active=int(upper[:nu].sum() + lower[:nu].sum()),
            zero_thrust_slots=int((u < n_zero).sum()),
            u_min_frac=float(u.min()/n_max), u_max_frac=float(u.max()/n_max),
            inf_pr=None if inf_pr[-1] is None else float(inf_pr[-1]),
            inf_du=None if inf_du[-1] is None else float(inf_du[-1])))
        return sol


def install_c2_curve():
    """M17 NLP가 쓸 추진 곡선만 C2(같은 매듭의 CubicSpline)로 바꾼다. 원래 함수를 돌려준다.

    `control/dynamics.py`는 곡선 함수를 **호출 때** 모듈에서 가져오고, 플랜트 모듈
    (`models/team_light/control/dynamics.py`)은 **import 때** 묶는다. 그래서 모듈 속성만 바꾸면
    제어기 쪽 기호 모델만 바뀐다. `_splines` 캐시(수치 경로 포함)는 건드리지 않는다.
    """
    import casadi as ca
    from scipy.interpolate import CubicSpline
    import models.team_light.control.dynamics  # noqa: F401 — 플랜트가 원래 함수를 먼저 묶게 한다
    from models.team_light.control import propeller_curve as pc

    cache = {}

    def c2_splines(p):
        key = pc._key(p)
        if key not in cache:
            rows = np.asarray(key)
            cache[key] = tuple(CubicSpline(rows[:, 0], rows[:, k]) for k in (1, 2))
        return cache[key]

    def symbolic_force_torque_c2(p, rate, axial_velocity):
        n_rps = ca.fmax(rate, 0.)/(2*ca.pi)
        j = ca.fmax(axial_velocity, 0.)/(n_rps*p['D_prop'] + 1e-8)
        ct, cp = c2_splines(p)
        return (p['rho']*n_rps**2*p['D_prop']**4*pc._symbolic_spline(ct, j),
                p['rho']*n_rps**2*p['D_prop']**5/(2*ca.pi)*pc._symbolic_spline(cp, j))

    original = pc.symbolic_force_torque
    pc.symbolic_force_torque = symbolic_force_torque_c2
    return original, c2_splines


def curve_difference(native, c2_splines):
    """운용 J 범위에서 C2 곡선이 PCHIP과 얼마나 다른가(상대·절대, Ct·Cp)."""
    from models.team_light.control import propeller_curve as pc
    ct, cp = pc._splines(pc._key(native))
    ct2, cp2 = c2_splines(native)
    J = np.linspace(*OPERATING_J, 2001)
    out = {}
    for name, a, b in (('Ct', ct, ct2), ('Cp', cp, cp2)):
        va, vb = a(J), b(J)
        rel = np.abs(vb - va)/np.maximum(np.abs(va), 1e-12)
        out[name] = dict(max_relative=float(rel.max()), max_absolute=float(np.abs(vb - va).max()),
                         J_at_max_relative=float(J[int(np.argmax(rel))]))
    return out


def _factory_class(variant):
    from control.arena_factory import ArenaFactory, ArenaController, trim_warm_start

    class DiagnosisFactory(ArenaFactory):
        """M17을 만들 때 솔버 프록시를 달고, V3이면 입력 편차 기준을 트림 회전수로 둔다."""

        last_proxy = None

        def _build_m17(self, window, v0, z0):
            if variant != 'V3':
                ctrl = super()._build_m17(window, v0, z0)
            else:
                from control.nmpc import NMPCController
                u_trim = self._trim_input('M17', v0)
                nmpc = NMPCController(self.cp, v_ref=v0, z_ref=z0, u_ref=u_trim,
                                      **self._nmpc_kwargs('M17', window))
                settings = self._nmpc_settings('M17', nmpc, u_ref='controller-model trim at v0')
                ctrl = ArenaController('M17', nmpc, window, nmpc=nmpc, settings=settings,
                                       warm_start=lambda x: trim_warm_start(nmpc, x, u_trim))
            proxy = SolverProxy(ctrl.nmpc.solver, ctrl.nmpc, raise_lower_bound=(variant == 'V4'))
            ctrl.nmpc.solver = proxy
            type(self).last_proxy = proxy
            return ctrl

    return DiagnosisFactory


def _zero_thrust_fraction(native, result):
    """로터가 추력 0 평탄 구간(J > 양추력 한계)에 들어간 스텝의 비율(플랜트 기준, 바람 포함)."""
    from scipy.spatial.transform import Rotation
    from models.team_light.control.propeller_curve import positive_thrust_j_limit
    xs, winds = result['xs'][1:], result['wind']
    if not len(winds):
        return 0.0
    j_limit = positive_thrust_j_limit(native)
    hits = 0
    for x, w in zip(xs, winds):
        va = max(float((Rotation.from_quat(x[6:10]).as_matrix().T @ (x[3:6] - w))[0]), 0.0)
        n_rps = np.maximum(x[13:17], 1e-9)/(2*np.pi)
        hits += bool(np.any(va/(n_rps*native['D_prop']) > j_limit))
    return hits/len(winds)


def _summarize(records, max_iter):
    failed = [r['status'] not in ('Solve_Succeeded', 'Solved_To_Acceptable_Level') for r in records]
    fails = [r for r, bad in zip(records, failed) if bad]
    succeeded = [r for r, bad in zip(records, failed) if not bad]
    iters = np.array([r['iter_count'] or 0 for r in records]) if records else np.zeros(1)
    run, longest = 0, 0
    for bad in failed:
        run = run + 1 if bad else 0
        longest = max(longest, run)
    active = [r for r in fails if r['active_upper'] + r['active_lower'] > 0]
    return dict(
        solves=len(records), failures=len(fails), longest_failure_run=longest,
        first_failure_t=fails[0]['t'] if fails else None,
        iter_median=float(np.median(iters)), iter_p95=float(np.percentile(iters, 95)),
        iter_max=int(iters.max()), at_iteration_limit=int((iters >= max_iter).sum()),
        failures_with_active_bound=len(active),
        failures_with_zero_thrust_plan=int(sum(r['zero_thrust_slots'] > 0 for r in fails)),
        succeeded_zero_thrust_fraction=(float(np.mean([r['zero_thrust_slots'] > 0 for r in succeeded]))
                                        if succeeded else None),
        failure_statuses=sorted({r['status'] for r in fails}),
        failures_active_upper_mean=float(np.mean([r['active_upper'] for r in fails])) if fails else None,
        failures_active_lower_mean=float(np.mean([r['active_lower'] for r in fails])) if fails else None,
        failures_first_step_active=int(sum(r['first_step_active'] > 0 for r in fails)),
        failures_u_min_frac=float(min(r['u_min_frac'] for r in fails)) if fails else None,
        failures_u_max_frac=float(max(r['u_max_frac'] for r in fails)) if fails else None,
        failures_inf_pr_median=(float(np.median([r['inf_pr'] for r in fails if r['inf_pr'] is not None]))
                                if fails else None),
        failures_inf_du_median=(float(np.median([r['inf_du'] for r in fails if r['inf_du'] is not None]))
                                if fails else None),
        succeeded_active_fraction=(float(np.mean([r['active_upper'] + r['active_lower'] > 0
                                                  for r in succeeded]))
                                   if succeeded else None))


def run_variant(variant, cases=CASES):
    """한 변형을 이 프로세스에서 돌린다. 반환: 사례별 요약 dict."""
    from control.arena import load_config, build_scenarios
    from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
    import control.validation_suite as suite
    from models.team_light.control.baseline_v2 import baseline_params
    from models.team_light.control.dynamics import AxialDronePlant

    config = load_config()
    native = baseline_params()
    extra = {}
    if variant == 'V1':
        config = deepcopy(config)
        config['nmpc_common']['max_iter'] = 100
    if variant == 'V2':
        from models.team_light.control.trim import find_trim as plant_trim
        tr = plant_trim(native, 85.0)
        before = AxialDronePlant(native).evaluate_xdot(tr['state'], tr['state'][13:17])
        _, c2_splines = install_c2_curve()
        after = AxialDronePlant(native).evaluate_xdot(tr['state'], tr['state'][13:17])
        extra['plant_unchanged_by_patch'] = bool(np.array_equal(before, after))
        extra['curve_difference'] = curve_difference(native, c2_splines)
        if not extra['plant_unchanged_by_patch']:
            raise RuntimeError('C2 patch leaked into the plant — diagnosis would be confounded')
    factory = _factory_class(variant)(config, native)
    limits = Acceptance(**config['acceptance'])
    paper = PaperCriteria(**config['paper_criteria'])
    reference = json.loads((ROOT/'results'/'arena'/'smoke_reference.json').read_text(encoding='utf-8'))
    ref_rows = {(r['scenario_id'], r['controller']): r for r in reference['rows']}
    out = dict(variant=variant, max_iter=int(config['nmpc_common']['max_iter']), cases={}, **extra)
    for scenario in build_scenarios(config, factory.cp, native, only=list(cases)):
        print(f'== {variant} {scenario.id}', flush=True)
        row, result, log = suite.run_trial(factory, 'M17', scenario.profile, scenario.cases[0], limits)
        row.update(paper_evaluate(result, scenario.profile, paper, window=scenario.window,
                                  solve_log=log, n_max=native['n_max']))
        proxy = factory.last_proxy
        us = result['us']
        ref = ref_rows.get((scenario.id, 'M17'), {})
        summary = dict(
            stop_reason=row['stop_reason'], completed=row['stop_reason'] is None,
            simulated_seconds=row['simulated_seconds'], T_total=scenario.profile.T_total,
            paper_failed=row['paper_failed'], max_omega=row['max_omega'],
            window_rmse_velocity=row.get('window_rmse_velocity'), window_rmse_z=row.get('window_rmse_z'),
            trajectory_sha256=row['trajectory_sha256'],
            reproduces_reference=(row['trajectory_sha256'] == ref.get('trajectory_sha256')
                                  if variant == 'V0' else None),
            command_min_frac=float(us.min()/native['n_max']) if len(us) else None,
            command_max_frac=float(us.max()/native['n_max']) if len(us) else None,
            command_at_n_max_fraction=(float(np.mean(np.any(us >= native['n_max']*(1 - 1e-9), axis=1)))
                                       if len(us) else None),
            zero_thrust_fraction=_zero_thrust_fraction(native, result),
            solver=_summarize(proxy.records, out['max_iter']))
        out['cases'][scenario.id] = summary
        print(json.dumps({k: summary[k] for k in ('stop_reason', 'simulated_seconds', 'reproduces_reference')}
                         | {'failures': summary['solver']['failures']}, ensure_ascii=False), flush=True)
    return out


def verdicts(results):
    """미리 정한 판정 규칙을 적용한다."""
    v = {name: results.get(name) for name in VARIANTS}
    out = {}
    for case in CASES:
        base = v['V0']['cases'][case] if v.get('V0') else None
        if base is None or base['completed']:
            out[case] = dict(note='V0가 멈추지 않아 판정 대상이 아니다' if base else 'V0 결과 없음')
            continue
        fails = base['solver']['failures']
        h1 = (fails > 0 and base['solver']['failures_with_active_bound'] >= 0.5*fails
              and v.get('V1') is not None and not v['V1']['cases'][case]['completed'])
        curve = (v.get('V2') or {}).get('curve_difference') or {}
        confounded = any(d['max_relative'] > 0.02 for d in curve.values()) if curve else None
        h2 = bool(v.get('V2') and v['V2']['cases'][case]['completed'])
        h3 = bool(v.get('V3') and v['V3']['cases'][case]['completed'])
        iteration_limited = bool(v.get('V1') and v['V1']['cases'][case]['completed'])
        h4 = bool(v.get('V4') and v['V4']['cases'][case]['completed'])
        out[case] = dict(H1=bool(h1), H2=h2, H2_confounded=confounded, H3=h3,
                         iteration_limited=iteration_limited, H4=h4)
    return out


def write_markdown(path, data):
    m = data['meta']
    lines = ['# M17 솔버 실패 진단 — PILOT', '',
             '**PILOT** — 진단 전용(경기장 설정·코드 불변). 우위·열위 결론 없음.', '',
             f'재현: `{m["command"]}` · git `{m["git_revision"]}` dirty={m["git_dirty"]}', '',
             '가설: H1 회전수 명령 한계 활성 · H2 NLP 속 C1 곡선(PCHIP) · H3 입력 편차 기준이 호버 회전수.',
             '변형: V0 기준(+계측) · V1 max_iter 100 · V2 M17 NLP만 C2 곡선 · V3 입력 편차 기준 = 트림 회전수.', '',
             '판정 규칙(미리 정함): H1 = V0 실패 풀이의 절반 이상에서 U 한계 활성 AND V1도 멈춤 · '
             'H2 = V2가 끝까지(곡선 차이 ≤ 2%) · H3 = V3가 끝까지 · 반복 부족 = V1이 끝까지.',
             f'한계 활성 = n_max 대비 {BOUND_TOL:g} 이내(내점법 반복점은 한계에 정확히 닿지 않는다. 처음 1e-6에서 '
             'V0 측풍 한 건을 본 뒤 조정). 추력 0 계획 = 계획 명령이 실측 축방향 속도 기준 J > 양추력 한계.', '']
    if 'V2' in data['variants']:
        cd = data['variants']['V2'].get('curve_difference') or {}
        lines += [f'C2 곡선 대 PCHIP(운용 J {OPERATING_J[0]}~{OPERATING_J[1]}): ' + ', '.join(
            f'{k} 최대 상대차 {100*d["max_relative"]:.2f}%(J={d["J_at_max_relative"]:.3f})' for k, d in cd.items())
            + f' · 패치 전후 플랜트 xdot 비트 동일: {data["variants"]["V2"].get("plant_unchanged_by_patch")}', '']
    lines += ['| 사례 | 변형 | 끝까지 | 시간 s / 전체 | 정지사유 | 풀이 | 실패 | 연속 실패 최대 | 반복 중앙/최대 | '
              '실패 중 한계 활성 | 실패 중 추력0 계획 | 성공 중 추력0 계획 | 명령 범위(n_max 비) | 플랜트 추력0 % | '
              '\\|ω\\|max | 참조 재현 |',
              '|---|---|---|---|---|---:|---:|---:|---|---:|---:|---:|---|---:|---:|---|']
    for case in CASES:
        for name in VARIANTS:
            r = (data['variants'].get(name) or {}).get('cases', {}).get(case)
            if r is None:
                continue
            s = r['solver']
            rng = ('—' if r['command_min_frac'] is None
                   else f'{r["command_min_frac"]:.3f}~{r["command_max_frac"]:.3f}')
            lines.append(f'| {case} | {name} | {r["completed"]} | {r["simulated_seconds"]:.2f} / {r["T_total"]:.1f} | '
                         f'{r["stop_reason"] or ""} | {s["solves"]} | {s["failures"]} | {s["longest_failure_run"]} | '
                         f'{s["iter_median"]:.0f}/{s["iter_max"]} | {s["failures_with_active_bound"]}/{s["failures"]} | '
                         f'{s["failures_with_zero_thrust_plan"]}/{s["failures"]} | '
                         f'{"—" if s["succeeded_zero_thrust_fraction"] is None else format(100*s["succeeded_zero_thrust_fraction"], ".1f") + "%"} | '
                         f'{rng} | {100*r["zero_thrust_fraction"]:.1f} | {r["max_omega"]:.3g} | '
                         f'{"—" if r["reproduces_reference"] is None else r["reproduces_reference"]} |')
    lines += ['', '## 판정(미리 정한 규칙 적용)', '',
              '| 사례 | H1 | H2 | H2 교란 | H3 | 반복 부족 | H4(사후) | 비고 |', '|---|---|---|---|---|---|---|---|']
    for case, v in data['verdicts'].items():
        lines.append(f'| {case} | {v.get("H1", "")} | {v.get("H2", "")} | {v.get("H2_confounded", "")} | '
                     f'{v.get("H3", "")} | {v.get("iteration_limited", "")} | {v.get("H4", "")} | {v.get("note", "")} |')
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    from control.validation_suite import write_json, environment_fingerprint, git_state
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--variant', choices=VARIANTS, help='run one variant in this process')
    parser.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    parser.add_argument('--parallel', type=int, default=2, help='variant processes at once (memory!)')
    parser.add_argument('--variants', nargs='+', choices=VARIANTS, default=list(VARIANTS),
                        help='variants to (re)run before aggregating; others are read from their JSON')
    parser.add_argument('--aggregate-only', action='store_true', help='only rebuild M17_DIAGNOSIS.{json,md}')
    args = parser.parse_args(argv)
    OUT.mkdir(parents=True, exist_ok=True)
    if args.variant:
        result = run_variant(args.variant, args.cases)
        write_json(OUT/f'{args.variant}.json', result)
        return result
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', VECLIB_MAXIMUM_THREADS='1')
    pending = [] if args.aggregate_only else list(args.variants)
    running = []
    while pending or running:
        while pending and len(running) < args.parallel:
            name = pending.pop(0)
            log = open(OUT/f'{name}.log', 'w', encoding='utf-8')
            running.append((name, log, subprocess.Popen(
                [sys.executable, '-m', 'control.arena_m17_diagnosis', '--variant', name, '--cases', *args.cases],
                cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)))
        name, log, proc = running.pop(0)
        proc.wait()
        log.close()
        if proc.returncode != 0:
            raise SystemExit(f'variant {name} failed — see {OUT/(name + ".log")}')
    revision, dirty = git_state()
    variants = {name: json.loads((OUT/f'{name}.json').read_text(encoding='utf-8'))
                for name in VARIANTS if (OUT/f'{name}.json').exists()}
    data = dict(meta=dict(label=LABEL, command='python -m control.arena_m17_diagnosis',
                          created_utc=datetime.now(timezone.utc).isoformat(),
                          git_revision=revision, git_dirty=dirty, environment=environment_fingerprint()),
                variants=variants, verdicts=verdicts(variants))
    write_json(ROOT/'results'/'arena'/'M17_DIAGNOSIS.json', data)
    write_markdown(ROOT/'results'/'arena'/'M17_DIAGNOSIS.md', data)
    print((ROOT/'results'/'arena'/'M17_DIAGNOSIS.md').read_text(encoding='utf-8'))
    return data


if __name__ == '__main__':
    main()
