"""
IMU 등급 스윕 — "좋은 IMU를 사면 얼마나 좋아지나?"
====================================================

RTK GPS 확정 상태에서, IMU 노이즈 레벨만 변경:
  - 표준 MEMS (1.0x): noise_acc=0.02, noise_gyro=0.001
  - 저노이즈 MEMS (0.5x): 절반
  - 고급 MEMS (0.1x): 1/10
  - 전술급 (0.02x): 1/50

시나리오: 70 m/s 순항 + 수직돌풍 (NMPC 병목이 IMU/자세였으니)
제어기: NMPC, Hybrid (IMU 민감도가 높은 둘)

목적: 실기 IMU 선정 시 정량 근거.
"""

import numpy as np
import time as timer

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import ScheduledLQR
from nmpc import NMPCController
from hybrid_comparison import VirtualNMPC, ProperHybrid
from gust_comparison import make_gust_fn
from sensors import IMUSensor, GPSSensor, SensorSuite
from ekf_sim import simulate_with_ekf, simulate_perfect, compute_metrics
from ekf_comparison import _reset_controller


def run_imu_level(plant, x0, trim, imu_scale, V_cruise, z_ref, dt, gust_fn):
    """한 IMU 레벨에서 3제어기 비교."""

    # RTK GPS 고정 + IMU 스케일링
    imu = IMUSensor(
        noise_acc=0.02 * imu_scale,
        noise_gyro=0.001 * imu_scale,
        seed=42,
    )
    gps = GPSSensor(
        dt_plant=dt, gps_rate=10.0,
        noise_pos=0.02, noise_vel=0.007,  # RTK 고정
        seed=43,
    )
    sensors = SensorSuite(imu, gps)

    u_trim = trim['control']
    T_sim = 8.0

    ctrls = {
        'LQR': ScheduledLQR(P, v_ref=[V_cruise, 0, 0], z_ref=z_ref),
        'NMPC': NMPCController(P, v_ref=[V_cruise, 0, 0], z_ref=z_ref,
                                u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02),
    }
    vn = VirtualNMPC(P, v_ref=[V_cruise, 0, 0], z_ref=z_ref,
                      N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    ctrls['Hybrid'] = ProperHybrid(vn, P, dt=dt)

    results = {}
    for name, ctrl in ctrls.items():
        # 참값
        _reset_controller(ctrl)
        rp = simulate_perfect(plant, ctrl, x0.copy(), T_sim, gust_fn)
        mp = compute_metrics(rp, z_ref, V_cruise)

        # EKF
        sensors.reset()
        _reset_controller(ctrl)
        re = simulate_with_ekf(plant, ctrl, x0.copy(), T_sim,
                                sensors=sensors, seed=42, wind_fn=gust_fn)
        me = compute_metrics(re, z_ref, V_cruise)

        div = np.any(np.isnan(re['xs_true'])) or me['max_z_err'] > 50
        results[name] = {
            'perfect': mp['rmse_z'],
            'ekf': me['rmse_z'] if not div else float('nan'),
            'div': div,
        }

    return results


def main():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    V_cruise = 70.0
    z_ref = 50.0
    trim = find_trim(P, V_cruise)
    x0 = trim['state'].copy()
    x0[2] = z_ref

    gust_fn = make_gust_fn('vertical', 10.0, 2.0, 1.0)

    # IMU 등급 정의
    imu_levels = [
        ('표준 MEMS',   1.0,   'σ_a=0.020, σ_g=0.0010'),
        ('저노이즈',    0.5,   'σ_a=0.010, σ_g=0.0005'),
        ('고급 MEMS',   0.1,   'σ_a=0.002, σ_g=0.0001'),
        ('전술급',      0.02,  'σ_a=0.0004, σ_g=0.00002'),
    ]

    print("=" * 65)
    print("  IMU 등급 스윕 — RTK 고정, 70 m/s + 돌풍")
    print("=" * 65)

    names = ['LQR', 'NMPC', 'Hybrid']

    print(f"\n  {'IMU 등급':>10s}  {'스케일':>6s}", end="")
    for n in names:
        print(f"  {n+'_z':>8s}", end="")
    print(f"  {'최강':>8s}")
    print(f"  {'─'*58}")

    all_results = []

    for label, scale, spec in imu_levels:
        print(f"  {label:>10s}  {scale:>5.2f}x", end="", flush=True)

        res = run_imu_level(plant, x0, trim, scale, V_cruise, z_ref, dt, gust_fn)
        all_results.append((label, scale, res))

        best_name = None
        best_z = np.inf
        for n in names:
            z = res[n]['ekf']
            if res[n]['div']:
                print(f"  {'발산!':>8s}", end="")
            else:
                print(f"  {z:>8.4f}", end="")
                if z < best_z:
                    best_z = z
                    best_name = n
        print(f"  {best_name:>8s}" if best_name else "")

    # 참값 기준 (노이즈 0)
    print(f"  {'참값(기준)':>10s}  {'0':>6s}", end="")
    # 참값은 첫 결과에서 가져옴
    for n in names:
        print(f"  {all_results[0][2][n]['perfect']:>8.4f}", end="")
    print()

    # 개선율 요약
    print(f"\n  {'IMU 등급':>10s}", end="")
    for n in names:
        print(f"  {n+'저하%':>9s}", end="")
    print()
    print(f"  {'─'*42}")

    for label, scale, res in all_results:
        print(f"  {label:>10s}", end="")
        for n in names:
            perf = res[n]['perfect']
            ekf = res[n]['ekf']
            if res[n]['div'] or np.isnan(ekf):
                print(f"  {'발산':>9s}", end="")
            else:
                deg = ((ekf - perf) / max(perf, 1e-6)) * 100
                print(f"  {deg:>+8.0f}%", end="")
        print()

    print()


if __name__ == '__main__':
    main()
