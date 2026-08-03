"""Phase 4 — 확정 구성 통합 검증 + 가상명령·모터 플롯 (필수 산출물).

확정 구성 (2026-08-04):
  솔버: acados GN-RTI 13D + Δu (G1 균일 20×0.05, 0.55ms급)
  감속: ScheduledLQR 인수, 복귀 금지 — 예방형 t=43 (반응형 ω15는 비교 표기)
  안전: NaN 우선 > ω15 백스톱 > z오차/미수렴/분산

산출:
  - results/final_config_mission.txt : 구간별 지표 + ω 실기판정 + presolve 효과
  - results/final_config_vc_motors.png : 가상명령(T,ν) + 모터 시계열
    (Δu 페널티 하에서 채터링 없음을 눈으로 확인 — RMSE만으론 불충분)

실행:
  DYLD_LIBRARY_PATH=$HOME/acados/lib python3 final_config_mission.py > results/final_config_mission.txt 2>&1
"""
import time

import numpy as np

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from hybrid_comparison import ProperHybrid
from vnmpc_acados import AcadosVirtualNMPC
from controller import ScheduledLQR
from mission_sim import (MissionProfile, MissionController, run_mission,
                         compute_phase_metrics, compute_overall)
from gust_comparison import make_gust_fn
from acados_fallback_mc import LoggedFB, omega_metrics

FINAL_KW = dict(N=20, dt_nmpc=0.05, dt_ctrl=0.02, rate_aug=True)


def run_arch(label, preempt_t, suffix):
    plant = AxialDronePlant(P, dt=0.001)
    profile = MissionProfile(70.0, 50.0)
    gust = make_gust_fn('vertical', 10.0, 35.0, 1.0)
    vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                           code_suffix=suffix, **FINAL_KW)
    vc_log = []
    orig = vn._solve

    def spy(x13):
        u = orig(x13)
        vc_log.append(u.copy())
        return u

    vn._solve = spy
    hyb = ProperHybrid(vn, P, dt=plant.dt)
    lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=2.0)
    fb = LoggedFB(hyb, lqr, z_ref=2.0, z_err_limit=10.0, omega_limit=15.0,
                  cooldown_sec=1e6, dt=plant.dt, preempt_t=preempt_t)
    ctrl = MissionController(fb, profile)
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 2.0
    res = run_mission(plant, ctrl, x0, profile, wind_fn=gust)

    ov = compute_overall(res)
    omm = omega_metrics(res['xs'])
    s = vn.get_stats()
    print(f"\n════ {label} ════")
    print(f"  완주: {not ov.get('diverged', False)}"
          f" | 전환 {fb.fallback_count}회 ({fb.switch_events[:1]})"
          f" | 솔브 {s['median_ms']:.2f}/{s['p95_ms']:.2f} ms"
          f" OK {s['n_ok']}/{s['n_solves']}")
    print(f"  전체 z={ov['rmse_z']:.3f} vx={ov['rmse_vx']:.3f}"
          f" | |ω|max={omm['om_max']:.1f} run25={omm['run25']:.0f}ms"
          f" 실기판정={'FAIL' if omm['real_fail'] else 'PASS'}")
    for m in compute_phase_metrics(res, profile):
        tag = 'DIV' if m['diverged'] else f"z={m['rmse_z']:.3f} vx={m['rmse_vx']:.3f}"
        print(f"    {m['name']:>4s}: {tag}", flush=True)
    return res, np.array(vc_log), vn


def plot_final(res, vc, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    ts_vc = np.arange(len(vc)) * 0.02
    tsu, us = res['ts'][:-1], res['us']
    cols = ['#4477AA', '#EE6677', '#228833', '#CCBB44']
    fig, ax = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    ax[0].plot(ts_vc, vc[:, 0], lw=0.7, color=cols[0])
    ax[0].set_ylabel('T cmd [N]')
    for j, nm in enumerate(['nu_x', 'nu_y', 'nu_z']):
        ax[1].plot(ts_vc, vc[:, 1 + j], lw=0.7, color=cols[j], label=nm)
    ax[1].set_ylabel('nu [rad/s^2]')
    ax[1].legend(fontsize=8, ncol=3)
    for j in range(4):
        ax[2].plot(tsu[::5], us[::5, j], lw=0.5, color=cols[j],
                   label=f'n{j+1}')
    ax[2].set_ylabel('motor [rad/s]')
    ax[2].set_xlabel('time [s]')
    ax[2].legend(fontsize=8, ncol=4)
    for a in ax:
        a.axvline(43.0, color='gray', ls='--', lw=0.8)
        a.grid(alpha=0.2)
    ax[0].set_title('Final config: acados GN-RTI (t<43) + ScheduledLQR (t>=43)'
                    ' - virtual commands end at handover', fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    print(f"\n플롯 저장: {path}", flush=True)


if __name__ == '__main__':
    print("Phase 4 — 확정 구성 통합 검증")
    res, vc, _ = run_arch('예방형 t=43 (권고안)', 43.0, 'fin_pre')
    plot_final(res, vc, 'results/final_config_vc_motors.png')
    run_arch('반응형 ω15 (대안)', None, 'fin_rea')

    # presolve() cold 대책 효과
    vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                           code_suffix='fin_ps', **FINAL_KW)
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 2.0
    x13 = np.concatenate([x0[0:10], x0[10:13]])
    t0 = time.perf_counter()
    vn._solve(x13)
    cold = (time.perf_counter() - t0) * 1e3
    vn2 = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                            code_suffix='fin_ps2', **FINAL_KW)
    vn2.presolve(x0, n=3)
    t0 = time.perf_counter()
    vn2._solve(x13)
    warm = (time.perf_counter() - t0) * 1e3
    print(f"\n[presolve] cold 첫 솔브 {cold:.2f} ms → presolve(3) 후 {warm:.2f} ms"
          f" (arm 전 프리솔브 관행 유지 권장)")
