"""acados 감속 진단 MC — mission_sim.diagnose_deceleration의 acados판.

구성: EXT_EXACT(EXTERNAL cost + exact Hessian) + SQP max_iter=5 + Δu 상태확장.
근거: 2026-08-04 실험에서 GN 계열(RTI/SQP5-GN) 전 구성이 감속 발산,
      EXT+exact+SQP5만 생존 (results/acados_port_status.md).
조건: diagnose_deceleration과 동일 — seed=0, 30회, 돌풍 10m/s@35s 고정,
      초기조건 랜덤 (z±0.5, vx±0.5, vz±0.3), 감속(43~58s) max Dz 분포.
주의: 시행마다 솔버를 새로 빌드 — 발산 시행의 QP 승수 NaN 오염이
      다음 시행으로 새지 않도록 (solver.reset()으로 불충분함을 실측).

실행 (caffeinate로 감싸지 말 것 — DYLD 스트립):
  DYLD_LIBRARY_PATH=$HOME/acados/lib python3 acados_decel_diag.py > results/acados_decel_diag.txt 2>&1
"""
import time as timer

import numpy as np

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from hybrid_comparison import ProperHybrid
from vnmpc_acados import AcadosVirtualNMPC
from mission_sim import (MissionProfile, MissionController, run_mission,
                         compute_overall)
from gust_comparison import make_gust_fn


def diagnose_deceleration_acados(n_trials=30, seed=0, **vn_kwargs):
    plant = AxialDronePlant(P, dt=0.001)
    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    gust_fn = make_gust_fn('vertical', 10.0, 35.0, 1.0)
    rng = np.random.default_rng(seed)
    decel_start, decel_end = 43.0, 58.0

    kw = dict(cost_variant='EXT_EXACT', nlp_solver_type='SQP', nlp_max_iter=5,
              rate_aug=True, N=20, dt_nmpc=0.05, dt_ctrl=0.02,
              code_suffix='mcdiag')
    kw.update(vn_kwargs)

    print(f"acados 감속 진단 ({n_trials}회, seed={seed})")
    print(f"구성: {kw}")
    print("=" * 60, flush=True)

    decel_max_dz, statuses_ok = [], []
    t0_all = timer.time()
    for i in range(n_trials):
        x0 = AxialDronePlant.hover_state(P)
        x0[2] = 2.0 + rng.normal(0, 0.5)
        x0[3] = rng.normal(0, 0.5)
        x0[5] = rng.normal(0, 0.3)

        vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0, **kw)
        ctrl = MissionController(ProperHybrid(vn, P, dt=plant.dt), profile)
        res = run_mission(plant, ctrl, x0, profile, wind_fn=gust_fn)

        ts, xs, z_refs = res['ts'], res['xs'], res['z_refs']
        mask = (ts >= decel_start) & (ts < decel_end)
        if np.any(np.isnan(xs[mask])):
            dz = np.inf
        else:
            dz = float(np.max(np.abs(xs[mask, 2] - z_refs[mask])))
        decel_max_dz.append(dz)
        s = vn.get_stats()
        statuses_ok.append((s['n_ok'] + s.get('n_maxiter', 0), s['n_solves']))
        ov = compute_overall(res)
        print(f"  [{i+1}/{n_trials}] 감속 max_dz={dz:.2f}m  전체 z="
              f"{ov['rmse_z']:.3f}  솔브usable {statuses_ok[-1][0]}/{statuses_ok[-1][1]}",
              flush=True)

    print(f"\n완료 [{timer.time()-t0_all:.0f}s]")
    dz_arr = np.array(decel_max_dz)
    finite = dz_arr[np.isfinite(dz_arr)]
    bins = [('< 2m (양호)', dz_arr < 2), ('2~5m (주의)', (dz_arr >= 2) & (dz_arr < 5)),
            ('5~10m (위험)', (dz_arr >= 5) & (dz_arr < 10)),
            ('10~20m (심각)', (dz_arr >= 10) & (dz_arr < 20)),
            ('>= 20m (대참사)', dz_arr >= 20)]
    print("\n심각도 분포:")
    for label, m in bins:
        print(f"  {label:>16s}: {int(np.sum(m)):3d}회 ({100*np.mean(m):5.1f}%)")
    if len(finite):
        print(f"\n평균 {np.mean(finite):.2f} | 중앙 {np.median(finite):.2f}"
              f" | 최악 {np.max(finite):.2f}"
              f" | P90 {np.percentile(finite, 90):.2f}"
              f" | P95 {np.percentile(finite, 95):.2f} m")
    n5 = int(np.sum(dz_arr >= 5))
    print(f"\n판정: 감속 max Dz >= 5m 발생률 = {n5}/{n_trials}"
          f" ({100*n5/n_trials:.0f}%)")
    print("비교: IPOPT N=20 rerun 17%(P95 8.6m, 최악 44.3m) / N=10 13%(7.3m, 14.0m)")


if __name__ == '__main__':
    diagnose_deceleration_acados()
