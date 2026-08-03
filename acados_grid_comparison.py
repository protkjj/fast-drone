"""
Phase 3 — 그리드 이산화 비교: acados G1/G2/G3 vs IPOPT 기준 (폐루프 65s 미션)
==============================================================================

목적:
  N=20(정밀·느림) vs N=10(빠름·과도구간 뭉개짐) 딜레마에 대한 비균일 그리드
  (근거리 촘촘·원거리 성글게)의 답을 폐루프로 검증.
  Δu 페널티가 없는 acados판의 **모터 채터링**을 Δu 있는 IPOPT판과 대조
  (RMSE만으론 판단 불가 — 과거 고도 정상인데 모터 0~1800 진동 사례).

구성:
  I20: IPOPT VirtualNMPC N=20 dt0.05 (Δu 페널티 있는 기준)
  G1 : acados 균일 20×0.05 (IPOPT 패리티)
  G2 : acados 균일 10×0.1
  G3 : acados 비균일 12노드 [0.02×2,0.03×2,0.05×2,0.08×2,0.12×2,0.2×2]=1.0s
       (첫 스텝 0.02 = dt_ctrl 일치는 의도된 설계)

산출:
  - results/acados_grid_comparison.txt : 표 (RMSE·솔브시간·모터 특성)
  - results/acados_grid_motors.png     : 모터 명령 시계열 (전체 + 감속 확대)

실행:
  DYLD_LIBRARY_PATH=$HOME/acados/lib python3 acados_grid_comparison.py \
      > results/acados_grid_comparison.txt 2>&1

⚠ caffeinate로 감싸지 말 것: /usr/bin/caffeinate는 SIP 보호 바이너리라
  DYLD_* 환경변수를 스트립함 → acados dylib 로드 실패.
  절전 방지가 필요하면 별도 셸에서 `caffeinate -dims &`를 띄워둘 것.

주의: 최종 그리드 채택은 이 표를 보고 kj가 결정 (자동 채택 금지).
     감속 구간 단일런 수치는 초기조건에 혼돈적으로 민감 (MC로 확정할 것).
"""
import time

import numpy as np

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from hybrid_comparison import VirtualNMPC, ProperHybrid
from vnmpc_acados import AcadosVirtualNMPC
from mission_sim import (MissionProfile, MissionController, run_mission,
                         compute_phase_metrics, compute_overall)
from gust_comparison import make_gust_fn

G3_STEPS = np.array([0.02, 0.02, 0.03, 0.03, 0.05, 0.05,
                     0.08, 0.08, 0.12, 0.12, 0.20, 0.20])

DECEL = (43.0, 58.0)


def build_variant(kind, **kw):
    if kind == 'ipopt':
        vn = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0, dt_ctrl=0.02, **kw)
        # IPOPT판은 자체 타이밍이 없으므로 계측 래핑
        vn.solve_times = []
        orig = vn._solve

        def timed(x13):
            t0 = time.perf_counter()
            r = orig(x13)
            vn.solve_times.append(time.perf_counter() - t0)
            return r

        vn._solve = timed
        return vn
    return AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0, dt_ctrl=0.02, **kw)


def motor_metrics(ts, us, t0, t1):
    """구간 [t0,t1)의 모터 명령 특성: 채터링 proxy = 스텝당 |Δn| 평균/최대."""
    mask = (ts[:-1] >= t0) & (ts[:-1] < t1)
    seg = us[mask]
    dn = np.abs(np.diff(seg, axis=0))
    sat = np.mean((seg <= 1e-6) | (seg >= 0.95 * P['n_max'])) * 100
    return {
        'mean_dn': float(np.mean(dn)),
        'p99_dn': float(np.percentile(dn, 99)),
        'max_dn': float(np.max(dn)),
        'sat_pct': float(sat),
    }


def run_variant(label, kind, plant, profile, gust_fn, **kw):
    print(f"\n════ {label} ════", flush=True)
    vn = build_variant(kind, **kw)
    ctrl = MissionController(ProperHybrid(vn, P, dt=plant.dt), profile)
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 2.0

    t0 = time.perf_counter()
    res = run_mission(plant, ctrl, x0, profile, wind_fn=gust_fn)
    wall = time.perf_counter() - t0

    st = np.array(vn.solve_times)
    ov = compute_overall(res)
    mm = motor_metrics(res['ts'], res['us'], *DECEL)

    print(f"  벽시계 {wall:.1f}s | 솔브 {len(st)}회"
          f" 중앙 {np.median(st)*1e3:.2f} / p95 {np.percentile(st,95)*1e3:.2f}"
          f" / 최악 {st.max()*1e3:.2f} ms  (⚠ MC 병행 중이면 타이밍 오염 — 참고용)")
    if kind == 'acados':
        s = vn.get_stats()
        print(f"  acados status OK {s['n_ok']}/{s['n_solves']}")
    print(f"  전체 RMSE: z={ov['rmse_z']:.3f} vx={ov['rmse_vx']:.3f}"
          f" (max z err {ov['max_z_err']:.2f})")
    print(f"  감속 모터: mean|Δn|={mm['mean_dn']:.2f} p99|Δn|={mm['p99_dn']:.1f}"
          f" max|Δn|={mm['max_dn']:.1f} rad/s/step, 포화 {mm['sat_pct']:.1f}%")
    print(f"  {'구간':>6s}  {'RMSE z':>8s}  {'RMSE vx':>8s}  {'max z':>7s}")
    phases = compute_phase_metrics(res, profile)
    for m in phases:
        if m['diverged']:
            print(f"  {m['name']:>6s}  {'DIV':>8s}")
        else:
            print(f"  {m['name']:>6s}  {m['rmse_z']:>8.3f}  {m['rmse_vx']:>8.3f}"
                  f"  {m['max_z_err']:>7.2f}", flush=True)
    return {'label': label, 'res': res, 'ov': ov, 'mm': mm,
            'st': st, 'wall': wall, 'phases': phases}


def plot_motors(results, path='results/acados_grid_motors.png'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = ['#4477AA', '#EE6677', '#228833', '#CCBB44']   # 모터 1~4 고정
    n = len(results)
    fig, axes = plt.subplots(n, 2, figsize=(14, 2.6 * n), sharex='col')

    for i, r in enumerate(results):
        ts, us = r['res']['ts'][:-1], r['res']['us']
        for j in range(4):
            axes[i, 0].plot(ts[::10], us[::10, j], lw=0.5,
                            color=colors[j], label=f'n{j+1}')
        axes[i, 0].set_ylabel(f"{r['label'].split(' ')[0]}\nmotor [rad/s]",
                              fontsize=8)
        axes[i, 0].set_ylim(0, P['n_max'] * 1.05)
        axes[i, 0].grid(alpha=0.2)

        zoom = (ts >= DECEL[0]) & (ts < DECEL[1])
        for j in range(4):
            axes[i, 1].plot(ts[zoom], us[zoom, j], lw=0.5, color=colors[j])
        axes[i, 1].set_ylim(0, P['n_max'] * 1.05)
        axes[i, 1].grid(alpha=0.2)
        axes[i, 1].text(0.02, 0.92, f"decel mean|dn|={r['mm']['mean_dn']:.2f}",
                        transform=axes[i, 1].transAxes, fontsize=8)

    axes[0, 0].legend(loc='upper right', fontsize=7, ncol=4)
    axes[0, 0].set_title('Full mission (65s)', fontsize=10)
    axes[0, 1].set_title(f'Deceleration zoom ({DECEL[0]:.0f}-{DECEL[1]:.0f}s)',
                         fontsize=10)
    axes[-1, 0].set_xlabel('time [s]')
    axes[-1, 1].set_xlabel('time [s]')
    fig.suptitle('Motor commands — chattering check (dU penalty absent in acados)',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"\n모터 플롯 저장: {path}", flush=True)


if __name__ == '__main__':
    plant = AxialDronePlant(P, dt=0.001)
    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    gust_fn = make_gust_fn('vertical', 10.0, 35.0, 1.0)

    print("Phase 3 — 그리드 비교 (65s 미션, 돌풍 10@35, 참값 피드백)")
    print("⚠ 감속진단 MC 병행 중이면 솔브 타이밍은 오염 — 확정 수치는 bench_acados로")

    results = [
        run_variant('I20 IPOPT N=20 (Δu 페널티 기준)', 'ipopt',
                    plant, profile, gust_fn, N=20, dt_nmpc=0.05),
        run_variant('G1 acados 균일 20x0.05', 'acados',
                    plant, profile, gust_fn, N=20, dt_nmpc=0.05,
                    code_suffix='g1'),
        run_variant('G2 acados 균일 10x0.1', 'acados',
                    plant, profile, gust_fn, N=10, dt_nmpc=0.1,
                    code_suffix='g2'),
        run_variant('G3 acados 비균일 12노드', 'acados',
                    plant, profile, gust_fn, time_steps=G3_STEPS,
                    code_suffix='g3'),
    ]

    # ── 요약 표 ──
    print(f"\n{'═'*100}")
    print("  [요약] 최종 채택은 보류 — kj 결정 사항")
    print(f"{'═'*100}")
    print(f"  {'구성':<28s} {'RMSE z':>7s} {'RMSE vx':>8s} {'감속 z':>7s}"
          f" {'솔브중앙':>8s} {'솔브p95':>8s} {'감속mean|Δn|':>12s} {'감속max|Δn|':>11s}")
    for r in results:
        decel_z = next((m['rmse_z'] for m in r['phases']
                        if m['name'] == '감속'), float('nan'))
        print(f"  {r['label']:<28s} {r['ov']['rmse_z']:>7.3f}"
              f" {r['ov']['rmse_vx']:>8.3f} {decel_z:>7.3f}"
              f" {np.median(r['st'])*1e3:>6.2f}ms"
              f" {np.percentile(r['st'],95)*1e3:>6.2f}ms"
              f" {r['mm']['mean_dn']:>12.2f} {r['mm']['max_dn']:>11.1f}")

    plot_motors(results)
