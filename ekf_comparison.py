"""
전 제어기 ESKF 비교 — 센서 노이즈 하에서 순위가 바뀌나?
========================================================

핵심 질문:
  "완벽 상태 가정에서 ProperHybrid가 전 조건 최강이었는데,
   센서 노이즈가 들어오면 순위가 유지되나?"

비교 대상 (5종):
  PID (스케줄), LQR (스케줄), NMPC, INDI, ProperHybrid

시나리오:
  70 m/s 순항 + 수직돌풍 (1-cosine, 10 m/s)
  — 이전 돌풍 비교와 동일 조건, ESKF만 추가

노이즈 레벨 스윕:
  GPS σ = 1.5m (표준) vs 0.02m (RTK)
  → 센서 품질이 결과를 얼마나 바꾸는지
"""

import numpy as np
import time as timer

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import (ScheduledPID, ScheduledLQR, INDIController)
from nmpc import NMPCController
from gust_comparison import make_gust_fn
from hybrid_comparison import VirtualNMPC, ProperHybrid

from ekf_sim import (simulate_with_ekf, simulate_perfect,
                      compute_metrics, compute_estimation_metrics)
from sensors import IMUSensor, GPSSensor, SensorSuite


def create_sensors(dt, noise_level=1.0, gps_noise_pos=1.5, seed=42):
    """센서 세트 생성 (GPS 노이즈 직접 지정 가능)."""
    imu = IMUSensor(
        noise_acc=0.02 * noise_level,
        noise_gyro=0.001 * noise_level,
        seed=seed,
    )
    gps = GPSSensor(
        dt_plant=dt,
        gps_rate=10.0,
        noise_pos=gps_noise_pos,
        noise_vel=gps_noise_pos / 3.0,  # 속도 노이즈 ≈ 위치/3
        seed=seed + 1,
    )
    return SensorSuite(imu, gps)


def make_controllers(V_cruise, z_ref, dt, x_trim, u_trim):
    """비교할 제어기 5종 생성."""
    ctrls = {}

    ctrls['PID스케줄'] = ScheduledPID(
        P, v_ref=[V_cruise, 0, 0], z_ref=z_ref, dt=dt)

    ctrls['LQR스케줄'] = ScheduledLQR(
        P, v_ref=[V_cruise, 0, 0], z_ref=z_ref)

    ctrls['NMPC'] = NMPCController(
        P, v_ref=[V_cruise, 0, 0], z_ref=z_ref,
        u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)

    ctrls['INDI'] = INDIController(
        P, v_ref=[V_cruise, 0, 0], z_ref=z_ref, dt=dt)

    vnmpc = VirtualNMPC(
        P, v_ref=[V_cruise, 0, 0], z_ref=z_ref,
        N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    ctrls['Hybrid'] = ProperHybrid(vnmpc, P, dt=dt)

    return ctrls


def _reset_controller(ctrl):
    """
    제어기 내부 상태 완전 리셋.

    버그 방지: NMPC/VirtualNMPC는 _last_t를 리셋하지 않으면
    두 번째 시뮬 실행 시 NLP를 재풀이하지 않는다.
    (t=0에서 시작하는데 _last_t=8.0이면 조건 불성립)
    """
    if hasattr(ctrl, 'reset'):
        ctrl.reset()

    # NMPC 계열: _last_t 리셋 (재풀이 강제)
    if hasattr(ctrl, '_last_t'):
        ctrl._last_t = -np.inf
    if hasattr(ctrl, '_u_current') and hasattr(ctrl, 'u_ref'):
        ctrl._u_current = ctrl.u_ref.copy()

    # ProperHybrid: 내부 VirtualNMPC도 리셋
    if hasattr(ctrl, 'nmpc'):
        if hasattr(ctrl.nmpc, '_last_t'):
            ctrl.nmpc._last_t = -np.inf
        if hasattr(ctrl.nmpc, '_u_current') and hasattr(ctrl.nmpc, 'u_ref'):
            ctrl.nmpc._u_current = ctrl.nmpc.u_ref.copy()


def run_single(plant, ctrl, x0, T_sim, wind_fn, sensors, label):
    """한 제어기의 참값/EKF 시뮬 실행."""
    # 참값 제어
    _reset_controller(ctrl)
    t0 = timer.time()
    res_p = simulate_perfect(plant, ctrl, x0.copy(), T_sim, wind_fn)
    t_perf = timer.time() - t0

    # EKF 제어 — 반드시 참값 후에 리셋!
    _reset_controller(ctrl)
    t0 = timer.time()
    res_e = simulate_with_ekf(
        plant, ctrl, x0.copy(), T_sim,
        sensors=sensors, seed=42, wind_fn=wind_fn)
    t_ekf = timer.time() - t0

    print(f"  {label:>10s}  참값 {t_perf:.1f}s / EKF {t_ekf:.1f}s", flush=True)

    return res_p, res_e


def print_comparison_table(results, z_ref, v_ref_x):
    """비교 결과 표 출력."""
    names = list(results.keys())

    print(f"\n  {'제어기':>10s}  {'조건':>6s}  "
          f"{'RMSE z':>8s}  {'RMSE vx':>8s}  {'최대Δz':>8s}  "
          f"{'저하율z':>8s}  {'추정err':>8s}")
    print(f"  {'─'*72}")

    for name in names:
        res_p, res_e = results[name]
        mp = compute_metrics(res_p, z_ref, v_ref_x)
        me = compute_metrics(res_e, z_ref, v_ref_x)

        # 추정 오차
        if 'est_errors' in res_e:
            em = compute_estimation_metrics(res_e)
            est_str = f"{em['pos_rmse']:.3f}m"
        else:
            est_str = "N/A"

        # 저하율
        deg_z = ((me['rmse_z'] - mp['rmse_z']) / max(mp['rmse_z'], 1e-6)) * 100

        # 발산 체크
        diverged = np.any(np.isnan(res_e['xs_true'])) or \
                   np.max(np.abs(res_e['xs_true'][:, 2] - z_ref)) > 50

        if diverged:
            print(f"  {name:>10s}  {'참값':>6s}  {mp['rmse_z']:>8.4f}  "
                  f"{mp['rmse_vx']:>8.4f}  {mp['max_z_err']:>8.3f}  "
                  f"{'':>8s}  {'':>8s}")
            print(f"  {name:>10s}  {'EKF':>6s}  {'발산!':>8s}  "
                  f"{'':>8s}  {'':>8s}  {'':>8s}  {est_str:>8s}")
        else:
            print(f"  {name:>10s}  {'참값':>6s}  {mp['rmse_z']:>8.4f}  "
                  f"{mp['rmse_vx']:>8.4f}  {mp['max_z_err']:>8.3f}  "
                  f"{'':>8s}  {'':>8s}")
            print(f"  {name:>10s}  {'EKF':>6s}  {me['rmse_z']:>8.4f}  "
                  f"{me['rmse_vx']:>8.4f}  {me['max_z_err']:>8.3f}  "
                  f"{deg_z:>+7.0f}%  {est_str:>8s}")


def main():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    V_cruise = 70.0
    z_ref = 50.0
    T_sim = 8.0

    # 돌풍 설정
    t_gust, T_gust, W_max = 2.0, 1.0, 10.0
    gust_fn = make_gust_fn('vertical', W_max, t_gust, T_gust)

    # 트림
    print("[준비] 트림 계산...")
    trim = find_trim(P, V_cruise)
    x_trim = trim['state'].copy()
    u_trim = trim['control'].copy()
    x0 = x_trim.copy()
    x0[2] = z_ref

    print(f"  {V_cruise} m/s 트림: θ={np.degrees(trim['theta']):.1f}°")

    # ════════════════════════════════════════════════
    # [1] 표준 GPS (σ=1.5m) — 순항+돌풍
    # ════════════════════════════════════════════════
    print("\n" + "=" * 75)
    print("  [1] 표준 GPS (σ=1.5m) — 70 m/s 순항 + 수직돌풍 10 m/s")
    print("=" * 75)

    sensors_std = create_sensors(dt, noise_level=1.0, gps_noise_pos=1.5)
    ctrls = make_controllers(V_cruise, z_ref, dt, x_trim, u_trim)

    results_std = {}
    for name, ctrl in ctrls.items():
        sensors_std.reset()
        res_p, res_e = run_single(
            plant, ctrl, x0, T_sim, gust_fn, sensors_std, name)
        results_std[name] = (res_p, res_e)

    print_comparison_table(results_std, z_ref, V_cruise)

    # ════════════════════════════════════════════════
    # [2] RTK GPS (σ=0.02m) — 순항+돌풍
    # ════════════════════════════════════════════════
    print("\n" + "=" * 75)
    print("  [2] RTK GPS (σ=0.02m) — 70 m/s 순항 + 수직돌풍 10 m/s")
    print("=" * 75)

    sensors_rtk = create_sensors(dt, noise_level=1.0, gps_noise_pos=0.02)
    ctrls_rtk = make_controllers(V_cruise, z_ref, dt, x_trim, u_trim)

    results_rtk = {}
    for name, ctrl in ctrls_rtk.items():
        sensors_rtk.reset()
        res_p, res_e = run_single(
            plant, ctrl, x0, T_sim, gust_fn, sensors_rtk, name)
        results_rtk[name] = (res_p, res_e)

    print_comparison_table(results_rtk, z_ref, V_cruise)

    # ════════════════════════════════════════════════
    # [3] 종합 요약
    # ════════════════════════════════════════════════
    print("\n" + "=" * 75)
    print("  [종합] 센서 노이즈 영향 요약")
    print("=" * 75)

    print(f"\n  {'제어기':>10s}  "
          f"{'참값 z':>8s}  {'GPS1.5 z':>8s}  {'RTK z':>8s}  "
          f"{'GPS저하':>8s}  {'RTK저하':>8s}")
    print(f"  {'─'*56}")

    names = list(results_std.keys())
    for name in names:
        rp_std, re_std = results_std[name]
        rp_rtk, re_rtk = results_rtk[name]

        mp = compute_metrics(rp_std, z_ref, V_cruise)
        me_std = compute_metrics(re_std, z_ref, V_cruise)
        me_rtk = compute_metrics(re_rtk, z_ref, V_cruise)

        # 발산 체크
        div_std = np.any(np.isnan(re_std['xs_true'])) or \
                  np.max(np.abs(re_std['xs_true'][:, 2] - z_ref)) > 50
        div_rtk = np.any(np.isnan(re_rtk['xs_true'])) or \
                  np.max(np.abs(re_rtk['xs_true'][:, 2] - z_ref)) > 50

        z_perf = mp['rmse_z']
        z_std = me_std['rmse_z'] if not div_std else float('nan')
        z_rtk = me_rtk['rmse_z'] if not div_rtk else float('nan')

        deg_std = ((z_std - z_perf) / max(z_perf, 1e-6) * 100) if not div_std else float('nan')
        deg_rtk = ((z_rtk - z_perf) / max(z_perf, 1e-6) * 100) if not div_rtk else float('nan')

        z_std_s = f"{z_std:.4f}" if not div_std else "발산!"
        z_rtk_s = f"{z_rtk:.4f}" if not div_rtk else "발산!"
        d_std_s = f"{deg_std:+.0f}%" if not np.isnan(deg_std) else "N/A"
        d_rtk_s = f"{deg_rtk:+.0f}%" if not np.isnan(deg_rtk) else "N/A"

        print(f"  {name:>10s}  "
              f"{z_perf:>8.4f}  {z_std_s:>8s}  {z_rtk_s:>8s}  "
              f"{d_std_s:>8s}  {d_rtk_s:>8s}")

    print()


if __name__ == '__main__':
    main()
