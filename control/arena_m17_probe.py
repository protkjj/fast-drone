"""M17 풀이 계측(읽기 전용) — 경기장 설정 그대로(양추력 하한 켬) 돌리며 실패 풀이를 분류한다. **PILOT**.

2026-09-26 오후: 양추력 하한(kj 결정 8.7-1)을 넣은 뒤에도 M17이 mission_VH 감속 구간(36.6 s)에서
IPOPT 반복 상한에 연속으로 걸려 멈췄다. 실패한 풀이의 계획을 두 가지로 가른다.
  (가) 예측 상태 기준 평탄 구간 — 계획한 로터 명령이 예측 노드의 축방향 속도로 보면 추력 0 전진비를
       넘는다. 하한을 현재(측정) 상태로 예측 구간 내내 고정한 한계(보고서 8.1·9.1)가 드러난 것이다.
  (나) 하한 활성 — 계획의 로터 칸 다수가 하한(추력 ≈ 0)에 붙어 있다.
같은 시각의 참조 가속도와, 명목 모델이 추력 ≥ 0·역유입 없이 낼 수 있는 최대 가·감속
(arena.acceleration_table, ρ 정의에 쓰는 a_avail)을 나란히 적는다.

계측은 솔버를 감싸 해를 읽기만 한다. 궤적 해시가 스모크 참조와 같은지로 확인한다.

실행: python -m control.arena_m17_probe mission_VH [다른 사례 …] [--out results/arena]
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import weakref

import numpy as np
from scipy.spatial.transform import Rotation

from control.arena import load_config, config_sha256, build_scenarios, acceleration_table, DEFAULT_CONFIG, ROOT
from control.arena_factory import ArenaFactory

ACCEPTED = ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
BOUND_TOL = 1e-4                 # n_max 대비 — 내점법 반복점은 한계에 정확히 닿지 않는다(M17 진단과 같은 값)
A_AVAIL_GRID = np.arange(0.0, 85.0 + 1e-9, 5.0)


def axial_speed(x):
    """동체 x축(추력축) 속도 — 음수면 0. M17 하한(control/nmpc.py)과 같은 식이다."""
    q = np.asarray(x[6:10], dtype=float)
    return max(float((Rotation.from_quat(q/np.linalg.norm(q)).as_matrix().T @ np.asarray(x[3:6]))[0]), 0.0)


class SolveProbe:
    """nmpc.solver를 감싸 풀이마다 계획을 분류한다(결과 무영향). NMPC는 약한 참조로 잡는다."""

    def __init__(self, solver, nmpc, j0):
        self._solver, self._nmpc, self.j0 = solver, weakref.ref(nmpc), float(j0)
        self.records = []

    def stats(self):
        return self._solver.stats()

    def __call__(self, **kw):
        sol = self._solver(**kw)
        nmpc = self._nmpc()
        stats = self._solver.stats()
        nx, nu, N = nmpc.nx, nmpc.nu, nmpc.N
        stride = nx + nu
        w = np.asarray(sol['x'], dtype=float).ravel()
        lbx = np.asarray(kw['lbx'], dtype=float).ravel()
        x_meas = np.asarray(kw['p'], dtype=float).ravel()[:nx]
        va_meas = axial_speed(x_meas)
        D, n_max = nmpc.p['D_prop'], float(nmpc.p['n_max'])
        plateau_pred = plateau_meas = active = 0
        va_pred = []
        for k in range(N):
            X, U = w[k*stride:k*stride + nx], w[k*stride + nx:k*stride + stride]
            va = axial_speed(X)
            va_pred.append(va)
            plateau_pred += int(np.sum(va/(U/(2*np.pi)*D + 1e-8) > self.j0))
            plateau_meas += int(np.sum(va_meas/(U/(2*np.pi)*D + 1e-8) > self.j0))
            active += int(np.sum(U - lbx[k*stride + nx:k*stride + stride] <= BOUND_TOL*n_max))
        self.records.append(dict(
            t=float(nmpc._t_now), status=stats.get('return_status'), iter_count=stats.get('iter_count'),
            plateau_slots_predicted=plateau_pred, plateau_slots_measured=plateau_meas,
            lower_active_slots=active, slots=N*nu, axial_speed_measured=va_meas,
            axial_speed_predicted_max=max(va_pred), floor_over_n_max=float(np.max(lbx[nx:stride]))/n_max))
        return sol


def reference_feasibility(profile, step, a_avail, dt=0.01):
    """사례 전체에서 참조 가속도가 명목 모델의 가용 가·감속을 넘는 시간 — 모든 제어기에 같은 시나리오 성질이다.

    참조 가속은 경기장 창과 같은 전진 차분(Δt_pred)이다. 반환: 넘는 시간 비율, 넘는 구간 목록, 최대 비율.
    """
    ts = np.arange(0.0, profile.T_total, dt)
    exceed, peak = [], 0.0
    for t in ts:
        v = profile.get_ref(t)[0][0]
        a = (profile.get_ref(t + step)[0][0] - v)/step
        if a == 0.0:
            exceed.append(False)
            continue
        avail = float(np.interp(v, A_AVAIL_GRID, a_avail['brake' if a < 0 else 'accel']))
        ratio = abs(a)/avail if avail > 0 else np.inf
        peak = max(peak, ratio)
        exceed.append(ratio > 1.0)
    exceed = np.array(exceed)
    edges = np.flatnonzero(np.diff(np.r_[False, exceed, False].astype(int)))
    intervals = [(float(ts[a]), float(ts[b - 1])) for a, b in zip(edges[::2], edges[1::2])]
    return dict(exceed_fraction=float(exceed.mean()), intervals=intervals, peak_ratio=float(peak))


def probe_case(config, factory, case_id, a_avail):
    import control.validation_suite as suite
    from control.nmpc import zero_thrust_advance_ratio
    from control.validation_metrics import Acceptance
    scenario = build_scenarios(config, factory.cp, factory.p, only=[case_id])[0]
    probes = []
    real = factory.make_for_profile

    def make(label, profile, case=None):
        ctrl = real(label, profile, case)
        probe = SolveProbe(ctrl.nmpc.solver, ctrl.nmpc, zero_thrust_advance_ratio(ctrl.nmpc.p))
        ctrl.nmpc.solver = probe
        probes.append(probe)
        return ctrl

    factory.make_for_profile = make
    try:
        row, result, _ = suite.run_trial(factory, 'M17', scenario.profile, scenario.cases[0],
                                         Acceptance(**config['acceptance']))
    finally:
        del factory.make_for_profile
    records = probes[0].records
    step = float(config['nmpc_common']['dt_pred_s'])
    for r in records:
        v = scenario.profile.get_ref(r['t'])[0][0]
        a = (scenario.profile.get_ref(r['t'] + step)[0][0] - v)/step
        table = a_avail['brake' if a < 0 else 'accel']
        r.update(v_ref=float(v), a_ref=float(a),
                 a_avail=float(np.interp(v, A_AVAIL_GRID, table)) if a != 0.0 else None)
    failed = [r for r in records if r['status'] not in ACCEPTED]
    ok = [r for r in records if r['status'] in ACCEPTED]
    share = lambda group, key: (None if not group else float(np.mean([r[key] > 0 for r in group])))
    return dict(case=case_id, stop_reason=row['stop_reason'], simulated_seconds=row['simulated_seconds'],
                max_omega=row['max_omega'], trajectory_sha256=row['trajectory_sha256'],
                reference_feasibility=reference_feasibility(scenario.profile, step, a_avail),
                solves=len(records), failed=len(failed),
                failed_share_plateau_predicted=share(failed, 'plateau_slots_predicted'),
                failed_share_lower_active=share(failed, 'lower_active_slots'),
                succeeded_share_plateau_predicted=share(ok, 'plateau_slots_predicted'),
                succeeded_share_lower_active=share(ok, 'lower_active_slots'),
                failed_solves=failed)


def write_markdown(path, data):
    m = data['meta']
    lines = ['# M17 풀이 계측 — 하한을 넣은 뒤 남은 실패(PILOT)', '',
             '**PILOT** — 진단 전용(읽기만 하는 계측). 우위·열위 결론 없음.', '',
             f'재현: `{m["command"]}` · 설정 sha256 `{m["config_sha256"][:12]}` · git `{m["git_revision"]}` '
             f'dirty={m["git_dirty"]}', '',
             '분류: (가) 예측 상태 기준 평탄 구간 = 계획한 로터 명령이 예측 노드의 축방향 속도로는 추력 0 전진비를 '
             '넘는다(하한을 현재 상태로 고정한 한계). (나) 하한 활성 = 로터 칸이 하한(추력 ≈ 0)에 붙어 있다. '
             '가용 가속 = 명목 모델이 추력 ≥ 0·역유입 없이 그 속도에서 낼 수 있는 최대 가·감속(ρ 정의의 a_avail).', '',
             '| 사례 | 정지·시간 s | \\|ω\\|max | 풀이 | 실패 | 실패 중 (가) | 실패 중 (나) | 성공 중 (가) | 성공 중 (나) | '
             '스모크 참조 해시 동일 |', '|---|---|---:|---:|---:|---:|---:|---:|---:|---|']
    pct = lambda x: '—' if x is None else f'{100*x:.0f}%'
    for c in data['cases']:
        lines.append(f"| {c['case']} | {c['stop_reason'] or '끝까지'} · {c['simulated_seconds']:.2f} | "
                     f"{c['max_omega']:.3g} | {c['solves']} | {c['failed']} | "
                     f"{pct(c['failed_share_plateau_predicted'])} | {pct(c['failed_share_lower_active'])} | "
                     f"{pct(c['succeeded_share_plateau_predicted'])} | {pct(c['succeeded_share_lower_active'])} | "
                     f"{c.get('reproduces_reference', '—')} |")
    lines += ['', '## 참조 가속이 가용 가·감속을 넘는 시간(시나리오 성질 — 모든 제어기에 같다)', '',
              '| 사례 | 넘는 시간 % | 넘는 구간 s | 최대 \\|참조\\|/가용 |', '|---|---:|---|---:|']
    for c in data['cases']:
        f = c['reference_feasibility']
        spans = ', '.join(f'{a:.1f}–{b:.1f}' for a, b in f['intervals']) or '없음'
        lines.append(f"| {c['case']} | {100*f['exceed_fraction']:.1f} | {spans} | {f['peak_ratio']:.2f} |")
    for c in data['cases']:
        if not c['failed_solves']:
            continue
        lines += ['', f"## {c['case']} — 실패 풀이", '',
                  '| t s | 상태 | 반복 | (가) 칸 | (나) 칸 / 전체 | 축방향 속도 측정 → 예측 최대 | 하한/n_max | '
                  '참조 속도 | 참조 가속 | 가용 가속(같은 방향) |',
                  '|---:|---|---:|---:|---|---|---:|---:|---:|---:|']
        for r in c['failed_solves']:
            avail = '—' if r['a_avail'] is None else f"{np.sign(r['a_ref'])*r['a_avail']:+.2f}"
            lines.append(f"| {r['t']:.2f} | {r['status']} | {r['iter_count']} | {r['plateau_slots_predicted']} | "
                         f"{r['lower_active_slots']} / {r['slots']} | {r['axial_speed_measured']:.2f} → "
                         f"{r['axial_speed_predicted_max']:.2f} | {r['floor_over_n_max']:.3f} | {r['v_ref']:.2f} | "
                         f"{r['a_ref']:+.2f} | {avail} |")
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    from control.validation_suite import write_json, environment_fingerprint, git_state
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('cases', nargs='+')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    factory = ArenaFactory(config)
    a_avail = {name: acceleration_table(factory.cp, A_AVAIL_GRID, sign)[1]
               for name, sign in (('accel', 1.0), ('brake', -1.0))}
    reference = json.loads((ROOT/'results'/'arena'/'smoke_reference.json').read_text(encoding='utf-8'))
    ref = {(r['scenario_id'], r['controller']): r for r in reference['rows']}
    cases = []
    for case_id in args.cases:
        c = probe_case(config, factory, case_id, a_avail)
        r = ref.get((case_id, 'M17'))
        c['reproduces_reference'] = None if r is None else r.get('trajectory_sha256') == c['trajectory_sha256']
        cases.append(c)
        print(case_id, c['stop_reason'], c['simulated_seconds'], 'failed', c['failed'], flush=True)
    revision, dirty = git_state()
    data = dict(meta=dict(label='PILOT', command='python -m control.arena_m17_probe ' + ' '.join(args.cases),
                          created_utc=datetime.now(timezone.utc).isoformat(), config_sha256=config_sha256(config),
                          git_revision=revision, git_dirty=dirty, environment=environment_fingerprint(),
                          reference_config_sha256=reference['config_sha256']),
                a_avail=dict(speeds=A_AVAIL_GRID.tolist(), **{k: v.tolist() for k, v in a_avail.items()}),
                cases=cases)
    args.out.mkdir(parents=True, exist_ok=True)
    write_json(args.out/'m17_probe.json', data)
    write_markdown(args.out/'M17_PROBE.md', data)
    print((args.out/'M17_PROBE.md').read_text(encoding='utf-8'))
    return data


if __name__ == '__main__':
    main()
