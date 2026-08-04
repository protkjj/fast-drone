"""Phase 3 — 확정 아키텍처(예방형 t=43 전환) 하의 그리드 비교 (G1/G2/G3).

맥락: 감속은 LQR 담당으로 확정 → 그리드는 acados가 나는 이륙~가속~순항
구간의 추종·솔브시간에만 영향. 이전 단독 비교(acados_grid_comparison.py)는
감속 발산으로 판정 불가였음 — 아키텍처 안에서는 전 그리드가 완주 가능.

주의: 최종 그리드 채택은 kj 결정 (표만 제공). 단일 결정론 런 —
폴백 MC에서 시행 간 편차가 ~0.05m 수준으로 관측되어 대표성 있음.

실행:
  DYLD_LIBRARY_PATH=$HOME/acados/lib python3 acados_grid_arch.py > results/acados_grid_arch.txt 2>&1
"""
import numpy as np

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from hybrid_comparison import ProperHybrid
from vnmpc_acados import AcadosVirtualNMPC
from controller import ScheduledLQR
from mission_sim import (MissionProfile, MissionController, run_mission,
                         compute_phase_metrics)
from gust_comparison import make_gust_fn
from acados_fallback_mc import LoggedFB, omega_metrics

G3_STEPS = np.array([0.02, 0.02, 0.03, 0.03, 0.05, 0.05,
                     0.08, 0.08, 0.12, 0.12, 0.20, 0.20])

GRIDS = [
    ('G1 균일 20x0.05', dict(N=20, dt_nmpc=0.05, code_suffix='ga1')),
    ('G2 균일 10x0.1', dict(N=10, dt_nmpc=0.1, code_suffix='ga2')),
    ('G3 비균일 12노드', dict(time_steps=G3_STEPS, code_suffix='ga3')),
]

if __name__ == '__main__':
    plant = AxialDronePlant(P, dt=0.001)
    profile = MissionProfile(70.0, 50.0)
    gust = make_gust_fn('vertical', 10.0, 35.0, 1.0)

    print("Phase 3 — 그리드 비교 (확정 아키텍처: GN-RTI+Δu, 예방형 t=43 전환)")
    print("감속·호버는 LQR 담당이라 전 그리드 공통 — acados 구간(이륙~순항)만 비교")
    print("=" * 78, flush=True)

    rows = []
    for label, gkw in GRIDS:
        vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0, dt_ctrl=0.02,
                               rate_aug=True, **gkw)
        hyb = ProperHybrid(vn, P, dt=plant.dt)
        lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=2.0)
        fb = LoggedFB(hyb, lqr, z_ref=2.0, z_err_limit=10.0, omega_limit=15.0,
                      cooldown_sec=1e6, dt=plant.dt, preempt_t=43.0)
        ctrl = MissionController(fb, profile)
        x0 = AxialDronePlant.hover_state(P)
        x0[2] = 2.0
        res = run_mission(plant, ctrl, x0, profile, wind_fn=gust)

        s = vn.get_stats()
        ph = {m['name']: (float('inf') if m['diverged'] else m['rmse_z'])
              for m in compute_phase_metrics(res, profile)}
        phv = {m['name']: (float('inf') if m['diverged'] else m['rmse_vx'])
               for m in compute_phase_metrics(res, profile)}
        omm = omega_metrics(res['xs'])
        rows.append((label, s, ph, phv, omm, fb.fallback_count))
        print(f"\n{label}: 솔브 {s['median_ms']:.2f}/{s['p95_ms']:.2f}"
              f"/{s['max_ms']:.2f} ms (중앙/p95/최악)"
              f" | status OK {s['n_ok']}/{s['n_solves']}"
              f" | |ω|max={omm['om_max']:.1f} 실기판정="
              f"{'FAIL' if omm['real_fail'] else 'ok'}")
        for name in ['이륙', '안정화', '가속', '순항']:
            print(f"  {name:>4s}: z={ph.get(name, float('nan')):.3f}"
                  f" vx={phv.get(name, float('nan')):.3f}", flush=True)

    print(f"\n{'═'*78}")
    print("[요약] acados 구간 성능 — 채택은 kj 결정")
    print(f"  {'그리드':<18s} {'솔브중앙':>8s} {'솔브p95':>8s}"
          f" {'이륙z':>7s} {'가속z':>7s} {'가속vx':>7s} {'순항z':>7s}")
    for label, s, ph, phv, omm, fc in rows:
        print(f"  {label:<18s} {s['median_ms']:>6.2f}ms {s['p95_ms']:>6.2f}ms"
              f" {ph.get('이륙', 0):>7.3f} {ph.get('가속', 0):>7.3f}"
              f" {phv.get('가속', 0):>7.3f} {ph.get('순항', 0):>7.3f}")
