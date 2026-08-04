"""지평선 이산화 비교 — VirtualNMPC N=20(dt 0.05) vs N=10(dt 0.1), 지평선 1s 동일.

동기 (bench_compute.py 후속):
  N=10/dt=0.1이 솔브 11.0ms/p95 11.5ms로 N=20+jit(7.7ms/p95 18.3ms)보다
  p95가 안정적 → 실시간 관점에서 매력적. 단, 이산화가 성글어지는 대가
  (예측 정확도, 특히 빠른 동역학)를 폐루프 RMSE로 같이 재야 판단 가능.

측정:
  통합 미션 65s (mission_sim과 동일 조건: 돌풍 10m/s@35s) 를 두 구성으로 실행,
  - 구간별/전체 RMSE (제어 성능)
  - 실제 벽시계 + 미션 중 실제 NMPC 솔브 시간 분포 (예산 계산 vs 실제 격차 규명)

실행:
  caffeinate -dims python3 horizon_comparison.py > results/horizon_comparison.txt 2>&1 &
"""
import time

import numpy as np

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from hybrid_comparison import VirtualNMPC, ProperHybrid
from mission_sim import (MissionProfile, MissionController, run_mission,
                         compute_phase_metrics, compute_overall)
from gust_comparison import make_gust_fn


def run_variant(label, N, dt_nmpc, plant, profile, gust_fn):
    print(f"\n════ {label} ════", flush=True)
    vn = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                     N=N, dt_nmpc=dt_nmpc, dt_ctrl=0.02)

    # 미션 중 실제 솔브 시간 계측 (벤치마크 vs 실전 격차 규명용)
    solve_times = []
    orig_solve = vn._solve

    def timed_solve(x13):
        t0 = time.perf_counter()
        r = orig_solve(x13)
        solve_times.append(time.perf_counter() - t0)
        return r

    vn._solve = timed_solve

    ctrl = MissionController(ProperHybrid(vn, P, dt=plant.dt), profile)
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 2.0

    t0 = time.perf_counter()
    res = run_mission(plant, ctrl, x0, profile, wind_fn=gust_fn)
    wall = time.perf_counter() - t0

    st = np.array(solve_times)
    print(f"  벽시계: {wall:.1f} s (실시간의 {wall/profile.T_total:.2f}배)")
    print(f"  미션 중 솔브 {len(st)}회: 중앙값 {np.median(st)*1e3:.1f} ms"
          f" | p95 {np.percentile(st, 95)*1e3:.1f} ms"
          f" | 최악 {st.max()*1e3:.1f} ms"
          f" | 합계 {st.sum():.1f} s ({100*st.sum()/wall:.0f}% of 벽시계)")

    ov = compute_overall(res)
    print(f"  전체 RMSE: z={ov['rmse_z']:.3f}  vx={ov['rmse_vx']:.3f}"
          f"  (max z err {ov['max_z_err']:.2f})")
    print(f"  {'구간':>6s}  {'RMSE z':>8s}  {'RMSE vx':>8s}  {'max z':>7s}")
    for m in compute_phase_metrics(res, profile):
        if m['diverged']:
            print(f"  {m['name']:>6s}  {'DIV':>8s}")
        else:
            print(f"  {m['name']:>6s}  {m['rmse_z']:>8.3f}  {m['rmse_vx']:>8.3f}"
                  f"  {m['max_z_err']:>7.2f}", flush=True)
    return res, ov, wall, st


if __name__ == '__main__':
    plant = AxialDronePlant(P, dt=0.001)
    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    gust_fn = make_gust_fn('vertical', 10.0, 35.0, 1.0)

    print("통합 미션 65s, 돌풍 10m/s@35s, Hybrid(VirtualNMPC+INDI), 참값 피드백")

    _, ov20, w20, st20 = run_variant("N=20, dt_nmpc=0.05 (기존)", 20, 0.05,
                                     plant, profile, gust_fn)
    _, ov10, w10, st10 = run_variant("N=10, dt_nmpc=0.1 (성긴 이산화)", 10, 0.1,
                                     plant, profile, gust_fn)

    print(f"\n════ 요약 ════")
    print(f"  {'':>14s}  {'RMSE z':>8s}  {'RMSE vx':>8s}  {'벽시계':>7s}  {'솔브중앙':>8s}  {'솔브p95':>8s}")
    print(f"  {'N=20':>14s}  {ov20['rmse_z']:>8.3f}  {ov20['rmse_vx']:>8.3f}"
          f"  {w20:>6.1f}s  {np.median(st20)*1e3:>6.1f}ms  {np.percentile(st20,95)*1e3:>6.1f}ms")
    print(f"  {'N=10,dt=0.1':>14s}  {ov10['rmse_z']:>8.3f}  {ov10['rmse_vx']:>8.3f}"
          f"  {w10:>6.1f}s  {np.median(st10)*1e3:>6.1f}ms  {np.percentile(st10,95)*1e3:>6.1f}ms")
    dz = 100 * (ov10['rmse_z'] - ov20['rmse_z']) / max(ov20['rmse_z'], 1e-9)
    dv = 100 * (ov10['rmse_vx'] - ov20['rmse_vx']) / max(ov20['rmse_vx'], 1e-9)
    print(f"  N=10의 성능 변화: RMSE z {dz:+.1f}%  RMSE vx {dv:+.1f}%")
