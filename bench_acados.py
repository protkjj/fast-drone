"""Phase 2 — acados 확정 구성(GN-RTI 13D+Δu) 계산 성능 벤치마크.

측정 (이전 교훈 반영: 트림 근방만 재면 과도구간 꼬리를 과소평가하므로
**미션 프로파일 상태에서 샘플링**):
  1) 확정 아키텍처(예방형 t=43 전환) 미션에서 acados가 실제 담당하는
     이륙~순항 구간(t<43)의 in-mission 솔브 시간 분포
  2) rti_phase 분리: preparation vs feedback — 실기 제어 지연 기준은 feedback
     (preparation은 상태 도착 전에 미리 수행)
  3) qp_solver_cond_N 스윕 (partial condensing 지평선)

게이트: feedback p95 < 5ms, 최악 < 10ms (kj 지정)

실행 (caffeinate로 감싸지 말 것 — SIP가 DYLD_* 스트립):
  DYLD_LIBRARY_PATH=$HOME/acados/lib python3 bench_acados.py > results/bench_acados.txt 2>&1
"""
import time

import numpy as np

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from hybrid_comparison import ProperHybrid
from vnmpc_acados import AcadosVirtualNMPC
from fallback_controller import HybridWithFallback
from controller import ScheduledLQR
from mission_sim import MissionProfile, MissionController, run_mission
from gust_comparison import make_gust_fn
from acados_fallback_mc import LoggedFB

FINAL_KW = dict(N=20, dt_nmpc=0.05, dt_ctrl=0.02, rate_aug=True)


def collect_mission_states():
    """확정 아키텍처 미션 1회 — acados 담당 구간의 솔브 입력 상태 수집."""
    plant = AxialDronePlant(P, dt=0.001)
    profile = MissionProfile(70.0, 50.0)
    gust = make_gust_fn('vertical', 10.0, 35.0, 1.0)
    vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                           code_suffix='bench', **FINAL_KW)
    states = []
    orig = vn._solve

    def spy(x13):
        states.append(x13.copy())
        return orig(x13)

    vn._solve = spy
    hyb = ProperHybrid(vn, P, dt=plant.dt)
    lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=2.0)
    fb = LoggedFB(hyb, lqr, z_ref=2.0, z_err_limit=10.0, omega_limit=15.0,
                  cooldown_sec=1e6, dt=plant.dt, preempt_t=43.0)
    ctrl = MissionController(fb, profile)
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 2.0
    run_mission(plant, ctrl, x0, profile, wind_fn=gust)

    st = np.array(vn.solve_times)
    s = vn.get_stats()
    print(f"[1] in-mission (예방형 아키텍처, acados 담당 t<43 구간)")
    print(f"  솔브 {len(st)}회 | 중앙 {np.median(st)*1e3:.2f} ms"
          f" | p95 {np.percentile(st, 95)*1e3:.2f} ms"
          f" | 최악 {st.max()*1e3:.2f} ms"
          f" | status OK {s['n_ok']}/{s['n_solves']}", flush=True)
    return states


def bench_split(states):
    """[2] preparation/feedback 분리 — 미션 상태 시퀀스 재생."""
    vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                           code_suffix='benchsp', **FINAL_KW)
    preps, fbs = [], []
    for x13 in states:
        tp, tf, _ = vn.solve_split(x13)
        preps.append(tp)
        fbs.append(tf)
    preps, fbs = np.array(preps) * 1e3, np.array(fbs) * 1e3
    print(f"\n[2] rti_phase 분리 ({len(states)} 상태 재생)")
    print(f"  preparation: 중앙 {np.median(preps):.2f} | p95"
          f" {np.percentile(preps, 95):.2f} | 최악 {preps.max():.2f} ms")
    print(f"  feedback   : 중앙 {np.median(fbs):.2f} | p95"
          f" {np.percentile(fbs, 95):.2f} | 최악 {fbs.max():.2f} ms")
    ok = np.percentile(fbs, 95) < 5.0 and fbs.max() < 10.0
    print(f"  게이트 (fb p95<5ms, 최악<10ms): {'PASS' if ok else 'FAIL'}",
          flush=True)


def bench_cond_n(states):
    """[3] qp_solver_cond_N 스윕 — 동일 상태 시퀀스, 전체 솔브 시간."""
    print(f"\n[3] qp_solver_cond_N 스윕 (N=20)")
    sub = states[::4]                       # 537개면 충분
    for cn in [None, 2, 5, 10, 20]:
        vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                               qp_cond_N=cn,
                               code_suffix=f'benchc{cn or "d"}', **FINAL_KW)
        ts = []
        for x13 in sub:
            t0 = time.perf_counter()
            vn._solve(x13)
            ts.append(time.perf_counter() - t0)
        ts = np.array(ts) * 1e3
        print(f"  cond_N={str(cn):>4s}: 중앙 {np.median(ts):.2f}"
              f" | p95 {np.percentile(ts, 95):.2f}"
              f" | 최악 {ts.max():.2f} ms", flush=True)


if __name__ == '__main__':
    states = collect_mission_states()
    bench_split(states)
    bench_cond_n(states)
