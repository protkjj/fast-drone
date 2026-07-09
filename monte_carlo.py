"""
몬테카를로 분석 — "Hybrid 1위가 우연이 아닌지" 통계 검증
==========================================================

랜덤화 항목 (매 trial):
  - 초기 위치 오차: ±1m
  - 초기 속도 오차: ±0.5 m/s
  - IMU 바이어스: 랜덤 생성 (가속도 σ=0.05, 자이로 σ=0.005)
  - GPS 노이즈 시드: 랜덤
  - 돌풍 강도: 8~12 m/s 균등분포
  - 돌풍 시작: 1.5~2.5 s 균등분포

시나리오: 70 m/s 순항 + 수직돌풍 (가장 까다로운 조건)
제어기: LQR스케줄, NMPC, Hybrid (RTK 확정)
N_trials: 100회 (macOS ~30분 예상)

출력: 각 제어기의 z RMSE 분포 (평균, 표준편차, 최악, 승률)
"""

import numpy as np
import time as timer
import sys

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import ScheduledLQR
from nmpc import NMPCController
from hybrid_comparison import VirtualNMPC, ProperHybrid
from gust_comparison import make_gust_fn
from sensors import IMUSensor, GPSSensor, SensorSuite
from ekf_sim import simulate_with_ekf, compute_metrics
from ekf_comparison import _reset_controller


def run_trial(plant, x0_base, trim, trial_seed, V_cruise, z_ref, dt):
    """
    한 trial 실행: 랜덤 조건 생성 → 3제어기 비교.

    Returns
    -------
    dict : {제어기명: rmse_z} 또는 None (발산 시)
    """
    rng = np.random.default_rng(trial_seed)

    # ── 랜덤 초기 조건 ──
    x0 = x0_base.copy()
    x0[0:3] += rng.normal(0, 1.0, 3)      # 위치 ±1m
    x0[3:6] += rng.normal(0, 0.5, 3)      # 속도 ±0.5 m/s

    # ── 랜덤 돌풍 ──
    W_max = rng.uniform(8.0, 12.0)         # 8~12 m/s
    t_gust = rng.uniform(1.5, 2.5)         # 시작 시각
    T_gust = 1.0
    gust_fn = make_gust_fn('vertical', W_max, t_gust, T_gust)

    # ── 랜덤 센서 (바이어스·시드) ──
    imu = IMUSensor(
        noise_acc=0.02, noise_gyro=0.001,
        seed=trial_seed * 10,
    )
    gps = GPSSensor(
        dt_plant=dt, gps_rate=10.0,
        noise_pos=0.02, noise_vel=0.007,   # RTK
        seed=trial_seed * 10 + 1,
    )
    sensors = SensorSuite(imu, gps)

    T_sim = 8.0
    u_trim = trim['control']
    results = {}

    # ── 3제어기 ──
    ctrls = {
        'LQR': ScheduledLQR(P, v_ref=[V_cruise, 0, 0], z_ref=z_ref),
        'NMPC': NMPCController(P, v_ref=[V_cruise, 0, 0], z_ref=z_ref,
                                u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02),
    }
    # Hybrid는 무거우니 매번 새로 생성
    vn = VirtualNMPC(P, v_ref=[V_cruise, 0, 0], z_ref=z_ref,
                      N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    ctrls['Hybrid'] = ProperHybrid(vn, P, dt=dt)

    for name, ctrl in ctrls.items():
        sensors.reset()
        _reset_controller(ctrl)
        try:
            res = simulate_with_ekf(
                plant, ctrl, x0.copy(), T_sim,
                sensors=sensors, seed=trial_seed, wind_fn=gust_fn)
            met = compute_metrics(res, z_ref, V_cruise)

            # 발산 체크
            if np.any(np.isnan(res['xs_true'])) or met['max_z_err'] > 50:
                results[name] = float('nan')
            else:
                results[name] = met['rmse_z']
        except Exception:
            results[name] = float('nan')

    return results


def main():
    N_TRIALS = 100

    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    V_cruise = 70.0
    z_ref = 50.0

    print("몬테카를로 준비 중...")
    trim = find_trim(P, V_cruise)
    x0_base = trim['state'].copy()
    x0_base[2] = z_ref

    names = ['LQR', 'NMPC', 'Hybrid']
    all_results = {n: [] for n in names}

    print(f"\n{'='*60}")
    print(f"  몬테카를로: {N_TRIALS}회 × 3제어기 (RTK, 70 m/s + 돌풍)")
    print(f"{'='*60}")
    print(f"  랜덤: 초기조건, IMU바이어스, 돌풍강도/시작시각")
    print()

    t_start = timer.time()

    for i in range(N_TRIALS):
        seed = 1000 + i
        trial = run_trial(plant, x0_base, trim, seed, V_cruise, z_ref, dt)

        for n in names:
            all_results[n].append(trial[n])

        # 진행률 (10회마다)
        if (i + 1) % 10 == 0:
            elapsed = timer.time() - t_start
            eta = elapsed / (i + 1) * (N_TRIALS - i - 1)
            # 현재까지 평균
            avgs = {n: np.nanmean(all_results[n]) for n in names}
            print(f"  [{i+1:3d}/{N_TRIALS}]  "
                  f"LQR={avgs['LQR']:.4f}  NMPC={avgs['NMPC']:.4f}  "
                  f"Hybrid={avgs['Hybrid']:.4f}  "
                  f"({elapsed:.0f}s, ETA {eta:.0f}s)",
                  flush=True)

    total_time = timer.time() - t_start

    # ── 통계 ──
    print(f"\n{'='*60}")
    print(f"  결과 ({N_TRIALS}회, {total_time:.0f}초)")
    print(f"{'='*60}")

    print(f"\n  {'제어기':>8s}  {'평균':>8s}  {'표준편차':>8s}  "
          f"{'최소':>8s}  {'최대':>8s}  {'발산':>5s}  {'1위횟수':>7s}")
    print(f"  {'─'*60}")

    # 발산 제거 후 통계
    clean = {}
    for n in names:
        arr = np.array(all_results[n])
        valid = arr[~np.isnan(arr)]
        clean[n] = valid

    # 1위 횟수 계산
    wins = {n: 0 for n in names}
    valid_trials = 0
    for i in range(N_TRIALS):
        vals = {n: all_results[n][i] for n in names}
        if any(np.isnan(v) for v in vals.values()):
            continue
        valid_trials += 1
        best = min(vals, key=vals.get)
        wins[best] += 1

    for n in names:
        v = clean[n]
        n_div = N_TRIALS - len(v)
        if len(v) > 0:
            print(f"  {n:>8s}  {np.mean(v):>8.4f}  {np.std(v):>8.4f}  "
                  f"{np.min(v):>8.4f}  {np.max(v):>8.4f}  "
                  f"{n_div:>4d}회  "
                  f"{wins[n]:>4d}/{valid_trials}")
        else:
            print(f"  {n:>8s}  {'전부 발산':>8s}")

    # 승률
    print(f"\n  승률 (유효 {valid_trials}회 중):")
    for n in names:
        pct = wins[n] / max(valid_trials, 1) * 100
        bar = '#' * int(pct / 2)
        print(f"  {n:>8s}  {pct:5.1f}%  {bar}")

    # Hybrid vs LQR 직접 비교
    hybrid_wins_vs_lqr = 0
    head_to_head = 0
    for i in range(N_TRIALS):
        h = all_results['Hybrid'][i]
        l = all_results['LQR'][i]
        if np.isnan(h) or np.isnan(l):
            continue
        head_to_head += 1
        if h < l:
            hybrid_wins_vs_lqr += 1

    print(f"\n  Hybrid vs LQR 직접 대결: "
          f"{hybrid_wins_vs_lqr}/{head_to_head} "
          f"({hybrid_wins_vs_lqr/max(head_to_head,1)*100:.1f}%)")

    print()


if __name__ == '__main__':
    main()
