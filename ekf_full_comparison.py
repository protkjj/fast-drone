"""
전 시나리오 RTK+ESKF 종합 비교
================================

기존 5개 시나리오를 RTK GPS + ESKF 하에서 재검증:
  A. 순항 (70, 85 m/s)
  B. 호버→70 m/s 천이
  C. 수직돌풍 (70 m/s + 10 m/s gust)
  D. 호버 교란
  E. 순항 30 m/s (저속)

제어기 3종 (RTK 확정 후 유의미한 것만):
  - LQR스케줄, NMPC, Hybrid(ProperHybrid)
  (PID/INDI는 이전 결과에서 열등 확인, 시간 절약)

핵심 질문: "RTK면 Hybrid가 전 시나리오에서 최강인가?"
"""

import numpy as np
import time as timer
from scipy.spatial.transform import Rotation

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import ScheduledLQR
from nmpc import NMPCController
from hybrid_comparison import VirtualNMPC, ProperHybrid
from gust_comparison import make_gust_fn
from sweep_scheduled import reference_trajectory
from ekf_sim import simulate_with_ekf, simulate_perfect, compute_metrics
from ekf_comparison import create_sensors, _reset_controller


def make_ctrls(V, z_ref, dt, u_trim):
    """3종 제어기 생성."""
    c = {}
    c['LQR'] = ScheduledLQR(P, v_ref=[V, 0, 0], z_ref=z_ref)
    c['NMPC'] = NMPCController(P, v_ref=[V, 0, 0], z_ref=z_ref,
                                u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    vn = VirtualNMPC(P, v_ref=[V, 0, 0], z_ref=z_ref,
                      N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    c['Hybrid'] = ProperHybrid(vn, P, dt=dt)
    return c


def run_scenario(plant, ctrl, x0, T_sim, sensors, wind_fn=None):
    """참값+EKF 한 쌍 실행."""
    _reset_controller(ctrl)
    rp = simulate_perfect(plant, ctrl, x0.copy(), T_sim, wind_fn)

    sensors.reset()
    _reset_controller(ctrl)
    re = simulate_with_ekf(plant, ctrl, x0.copy(), T_sim,
                            sensors=sensors, seed=42, wind_fn=wind_fn)
    return rp, re


def print_row(name, z_ref, v_ref, rp, re):
    """한 줄 출력."""
    mp = compute_metrics(rp, z_ref, v_ref)
    me = compute_metrics(re, z_ref, v_ref)

    div = np.any(np.isnan(re['xs_true'])) or me['max_z_err'] > 50
    if div:
        print(f"  {name:>8s}  {mp['rmse_z']:>8.4f}  {'발산!':>8s}  {'N/A':>7s}")
        return mp['rmse_z'], float('nan')

    deg = ((me['rmse_z'] - mp['rmse_z']) / max(mp['rmse_z'], 1e-6)) * 100
    print(f"  {name:>8s}  {mp['rmse_z']:>8.4f}  {me['rmse_z']:>8.4f}  {deg:>+6.0f}%")
    return mp['rmse_z'], me['rmse_z']


def main():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt
    sensors = create_sensors(dt, noise_level=1.0, gps_noise_pos=0.02)  # RTK

    # 트림 캐시
    trims = {}
    for V in [0, 10, 20, 30, 40, 50, 60, 70, 80, 85]:
        tr = find_trim(P, float(V))
        if tr['residual'] < 1e-3:
            trims[V] = tr

    summary = []  # (시나리오, 제어기, 참값z, RTKz)

    # ════════════════════════════════════════════
    # A. 순항 70 m/s
    # ════════════════════════════════════════════
    print("\n" + "=" * 50)
    print("  [A] 순항 70 m/s (교란 회복)")
    print("=" * 50)

    V, z_ref = 70.0, 50.0
    trim = trims[70]
    x0 = trim['state'].copy(); x0[2] = z_ref; x0[3] += 1.0; x0[5] += 2.0
    ctrls = make_ctrls(V, z_ref, dt, trim['control'])

    print(f"  {'제어기':>8s}  {'참값z':>8s}  {'RTKz':>8s}  {'저하':>7s}")
    print(f"  {'─'*36}")
    for name, ctrl in ctrls.items():
        t0 = timer.time()
        rp, re = run_scenario(plant, ctrl, x0, 5.0, sensors)
        zp, ze = print_row(name, z_ref, V, rp, re)
        summary.append(('순항70', name, zp, ze))

    # ════════════════════════════════════════════
    # B. 순항 85 m/s (최대속도)
    # ════════════════════════════════════════════
    print("\n" + "=" * 50)
    print("  [B] 순항 85 m/s (최대속도)")
    print("=" * 50)

    V, z_ref = 85.0, 50.0
    trim = trims[85]
    x0 = trim['state'].copy(); x0[2] = z_ref; x0[3] += 1.0; x0[5] += 2.0
    ctrls = make_ctrls(V, z_ref, dt, trim['control'])

    print(f"  {'제어기':>8s}  {'참값z':>8s}  {'RTKz':>8s}  {'저하':>7s}")
    print(f"  {'─'*36}")
    for name, ctrl in ctrls.items():
        rp, re = run_scenario(plant, ctrl, x0, 5.0, sensors)
        zp, ze = print_row(name, z_ref, V, rp, re)
        summary.append(('순항85', name, zp, ze))

    # ════════════════════════════════════════════
    # C. 수직돌풍 (70 m/s + 10 m/s gust)
    # ════════════════════════════════════════════
    print("\n" + "=" * 50)
    print("  [C] 돌풍 — 70 m/s + 수직 10 m/s")
    print("=" * 50)

    V, z_ref = 70.0, 50.0
    trim = trims[70]
    x0 = trim['state'].copy(); x0[2] = z_ref
    gust_fn = make_gust_fn('vertical', 10.0, 2.0, 1.0)
    ctrls = make_ctrls(V, z_ref, dt, trim['control'])

    print(f"  {'제어기':>8s}  {'참값z':>8s}  {'RTKz':>8s}  {'저하':>7s}")
    print(f"  {'─'*36}")
    for name, ctrl in ctrls.items():
        rp, re = run_scenario(plant, ctrl, x0, 8.0, sensors, gust_fn)
        zp, ze = print_row(name, z_ref, V, rp, re)
        summary.append(('돌풍', name, zp, ze))

    # ════════════════════════════════════════════
    # D. 호버 교란
    # ════════════════════════════════════════════
    print("\n" + "=" * 50)
    print("  [D] 호버 교란 (0 m/s)")
    print("=" * 50)

    V, z_ref = 0.0, 10.0
    trim = trims[0]
    x0 = trim['state'].copy(); x0[2] = z_ref; x0[3] = 2.0; x0[5] = 1.0
    ctrls = make_ctrls(V, z_ref, dt, trim['control'])

    print(f"  {'제어기':>8s}  {'참값z':>8s}  {'RTKz':>8s}  {'저하':>7s}")
    print(f"  {'─'*36}")
    for name, ctrl in ctrls.items():
        rp, re = run_scenario(plant, ctrl, x0, 5.0, sensors)
        zp, ze = print_row(name, z_ref, V, rp, re)
        summary.append(('호버', name, zp, ze))

    # ════════════════════════════════════════════
    # E. 호버→70 천이 (가속)
    # ════════════════════════════════════════════
    print("\n" + "=" * 50)
    print("  [E] 호버→70 m/s 천이")
    print("=" * 50)

    z_ref = 50.0
    x0 = AxialDronePlant.hover_state(P); x0[2] = z_ref

    # 천이용 제어기: 시간변화 기준 추종
    class _TrackingWrapper:
        """reference_trajectory에 따라 v_ref를 동적으로 변경."""
        def __init__(self, inner_ctrl):
            self.inner = inner_ctrl
        def __call__(self, t, x):
            v_ref, z = reference_trajectory(t)
            # LQR/NMPC의 v_ref 동적 갱신
            if hasattr(self.inner, 'v_ref'):
                self.inner.v_ref = v_ref
            if hasattr(self.inner, 'z_ref'):
                self.inner.z_ref = z
            # Hybrid: 내부 NMPC의 v_ref도 갱신
            if hasattr(self.inner, 'nmpc') and hasattr(self.inner.nmpc, 'v_ref'):
                self.inner.nmpc.v_ref = v_ref
                self.inner.nmpc.z_ref = z
            return self.inner(t, x)
        def reset(self):
            _reset_controller(self.inner)

    trim0 = trims[0]
    ctrls_tr = {}
    ctrls_tr['LQR'] = _TrackingWrapper(
        ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=z_ref))
    ctrls_tr['NMPC'] = _TrackingWrapper(
        NMPCController(P, v_ref=[0, 0, 0], z_ref=z_ref, u_ref=trim0['control']))
    vn = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=z_ref)
    ctrls_tr['Hybrid'] = _TrackingWrapper(ProperHybrid(vn, P, dt=dt))

    T_tr = 20.0

    print(f"  {'제어기':>8s}  {'참값z':>8s}  {'RTKz':>8s}  {'저하':>7s}")
    print(f"  {'─'*36}")
    for name, ctrl in ctrls_tr.items():
        # 참값: reference_trajectory 기준 RMSE
        _reset_controller(ctrl)
        ts_p, xs_p, _ = plant.simulate(x0.copy(), ctrl, T_tr)
        vx_ref_p = np.array([reference_trajectory(t)[0][0] for t in ts_p])
        z_ref_p = np.array([reference_trajectory(t)[1] for t in ts_p])
        rmse_z_p = np.sqrt(np.mean((xs_p[:, 2] - z_ref_p)**2))

        # EKF
        sensors.reset()
        _reset_controller(ctrl)
        re = simulate_with_ekf(plant, ctrl, x0.copy(), T_tr,
                                sensors=sensors, seed=42)
        xs_e = re['xs_true']
        ts_e = re['ts']
        z_ref_e = np.array([reference_trajectory(t)[1] for t in ts_e])
        rmse_z_e = np.sqrt(np.mean((xs_e[:, 2] - z_ref_e)**2))

        div = np.any(np.isnan(xs_e)) or np.max(np.abs(xs_e[:, 2] - z_ref_e)) > 50
        if div:
            print(f"  {name:>8s}  {rmse_z_p:>8.4f}  {'발산!':>8s}  {'N/A':>7s}")
            summary.append(('천이', name, rmse_z_p, float('nan')))
        else:
            deg = ((rmse_z_e - rmse_z_p) / max(rmse_z_p, 1e-6)) * 100
            print(f"  {name:>8s}  {rmse_z_p:>8.4f}  {rmse_z_e:>8.4f}  {deg:>+6.0f}%")
            summary.append(('천이', name, rmse_z_p, rmse_z_e))

    # ════════════════════════════════════════════
    # 종합 요약
    # ════════════════════════════════════════════
    print("\n" + "=" * 60)
    print("  [종합] RTK + ESKF — 전 시나리오 × 3제어기")
    print("=" * 60)

    scenarios = ['순항70', '순항85', '돌풍', '호버', '천이']
    names = ['LQR', 'NMPC', 'Hybrid']

    print(f"\n  {'시나리오':>8s}", end="")
    for n in names:
        print(f"  {n+' 참':>8s} {n+' RTK':>8s}", end="")
    print()
    print(f"  {'─'*60}")

    for sc in scenarios:
        print(f"  {sc:>8s}", end="")
        for n in names:
            match = [(zp, ze) for s, nm, zp, ze in summary if s == sc and nm == n]
            if match:
                zp, ze = match[0]
                zp_s = f"{zp:.4f}" if not np.isnan(zp) else "N/A"
                ze_s = f"{ze:.4f}" if not np.isnan(ze) else "발산"
                print(f"  {zp_s:>8s} {ze_s:>8s}", end="")
            else:
                print(f"  {'?':>8s} {'?':>8s}", end="")
        print()

    # 최종 판정
    print(f"\n  {'시나리오':>8s}  {'RTK 1위':>12s}")
    print(f"  {'─'*24}")
    for sc in scenarios:
        best_name, best_z = None, np.inf
        for n in names:
            match = [(ze) for s, nm, zp, ze in summary if s == sc and nm == n]
            if match and not np.isnan(match[0]):
                if match[0] < best_z:
                    best_z = match[0]
                    best_name = n
        print(f"  {sc:>8s}  {best_name:>12s}  (z={best_z:.4f})")

    print()


if __name__ == '__main__':
    main()
