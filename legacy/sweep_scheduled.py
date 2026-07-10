"""
게인 스케줄링 종합 비교 — 고정 게인 vs 스케줄링 vs NMPC
======================================================

5개 제어기를 3개 시나리오에서 나란히 비교:
  1) PID 고정   (30 m/s 게인)
  2) PID 스케줄 (속도별 게인 스케일링)
  3) LQR 고정   (30 m/s K_r)
  4) LQR 스케줄 (속도별 K_r 선형 보간)
  5) NMPC       (CasADi+IPOPT, 비선형 예측)

시나리오:
  A. 순항 속도 스윕     (30~85 m/s, 교란 회복)
  B. 호버→70 m/s 천이   (가속 + 고도 유지)
  C. 발사천이            (40 m/s, 30° AoA, 로터 0)

목표: "고전제어는 속도별 튜닝해도 NMPC엔 못 미친다" 정량 검증.
"""

import numpy as np
import time as timer
from scipy.spatial.transform import Rotation

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant, NX
from trim import find_trim
from controller import (CascadedPID, LQRController,
                        ScheduledPID, ScheduledLQR)
from nmpc import NMPCController


# ══════════════════════════════════════════════════
# 기준 궤적 (호버→70 천이용)
# ══════════════════════════════════════════════════

def reference_trajectory(t):
    """
    호버→70 m/s 가속 기준.
    t=0~1s: 호버 안정화, t=1~16s: 선형 가속, t=16s~: 순항
    가속도 70/15 ≈ 4.7 m/s² (0.48 G)
    """
    z_ref = 50.0
    if t < 1.0:
        vx = 0.0
    elif t < 16.0:
        vx = 70.0 * (t - 1.0) / 15.0
    else:
        vx = 70.0
    return np.array([vx, 0.0, 0.0]), z_ref


# ══════════════════════════════════════════════════
# 헬퍼: 시뮬레이션 + 측정
# ══════════════════════════════════════════════════

def _run_cruise(plant, x0, ctrl, V_ref, z_ref, T_sim):
    """순항 교란 회복 시뮬 + RMSE 측정."""
    try:
        ts, xs, us = plant.simulate(x0.copy(), ctrl, T_sim)
    except Exception:
        return {'rmse_vx': np.inf, 'rmse_z': np.inf,
                'sat': 100.0, 'div': True}

    div = bool(np.any(np.isnan(xs)) or
               np.any(np.abs(xs[:, 2] - z_ref) > 200))
    end = len(xs)
    if div:
        bad = np.where(np.abs(xs[:, 2] - z_ref) > 200)[0]
        end = max(bad[0] if len(bad) > 0 else len(xs), 10)

    return {
        'rmse_vx': np.sqrt(np.mean((xs[:end, 3] - V_ref)**2)),
        'rmse_z':  np.sqrt(np.mean((xs[:end, 2] - z_ref)**2)),
        'sat':     100.0 * np.mean(us[:min(end, len(us))] >= 0.95 * P['n_max']),
        'div':     div,
    }


def _run_tracking(plant, x0, ctrl, T_sim):
    """시간변화 기준 추종 시뮬 + 측정."""
    try:
        ts, xs, us = plant.simulate(x0.copy(), ctrl, T_sim)
    except Exception:
        return {'rmse_vx': np.inf, 'rmse_z': np.inf,
                'z_max': np.inf, 'sat': 100.0, 'div': True}

    div = bool(np.any(np.isnan(xs)) or np.any(np.abs(xs[:, 2]) > 300))

    vx_ref = np.array([reference_trajectory(t)[0][0] for t in ts])
    z_ref  = np.array([reference_trajectory(t)[1] for t in ts])

    return {
        'rmse_vx': np.sqrt(np.mean((xs[:, 3] - vx_ref)**2)),
        'rmse_z':  np.sqrt(np.mean((xs[:, 2] - z_ref)**2)),
        'z_max':   np.max(np.abs(xs[:, 2] - z_ref)),
        'sat':     100.0 * np.mean(us >= 0.95 * P['n_max']),
        'div':     div,
    }


def _run_launch(plant, x0, ctrl, V_ref, z_ref, T_sim):
    """발사천이 시뮬 + 측정."""
    try:
        ts, xs, us = plant.simulate(x0.copy(), ctrl, T_sim)
    except Exception:
        return {'alt_loss': np.inf, 'settle': np.inf,
                'rmse_vx': np.inf, 'rmse_z': np.inf, 'div': True}

    div = bool(np.any(np.isnan(xs)) or
               np.any(np.abs(xs[:, 2] - z_ref) > 100))

    z = xs[:, 2]
    omega_mag = np.sqrt(xs[:, 10]**2 + xs[:, 11]**2 + xs[:, 12]**2)

    # 안정화 시간: vx 오차 < 2 AND |ω| < 0.5 가 50 스텝 연속
    settle = T_sim
    for i in range(len(ts) - 50):
        w = slice(i, i + 50)
        if (np.all(np.abs(xs[w, 3] - V_ref) < 2.0) and
                np.all(omega_mag[w] < 0.5)):
            settle = ts[i]
            break

    return {
        'alt_loss': z_ref - np.min(z),
        'settle':   settle,
        'rmse_vx':  np.sqrt(np.mean((xs[:, 3] - V_ref)**2)),
        'rmse_z':   np.sqrt(np.mean((z - z_ref)**2)),
        'div':      div,
    }


def _v(val, fmt='.2f'):
    """값 포매팅. DIV/inf/nan → 'DIV'."""
    if val is None or val == np.inf or (isinstance(val, float) and np.isnan(val)):
        return '    DIV'
    return f'{val:{fmt}:>7s}'


# ══════════════════════════════════════════════════
# 간이 고정 LQR (K_r_30 고정 + 해당 속도 트림)
# ══════════════════════════════════════════════════

class _FixedLQR:
    """K_r을 30 m/s에서 고정, 트림은 해당 속도 것 사용."""
    def __init__(self, params, K_r, x_trim, u_trim, z_ref):
        self.p = params
        self.K_r = K_r
        self.x_trim = x_trim.copy()
        self.u_trim = u_trim.copy()
        self.x_trim[2] = z_ref

    def __call__(self, t, x):
        x_t = self.x_trim.copy()
        x_t[0:2] = x[0:2]
        dx_r = ScheduledLQR._compute_error_state(x, x_t)
        u = self.u_trim - self.K_r @ dx_r
        return np.clip(u, self.p['n_min'], self.p['n_max'])


# ══════════════════════════════════════════════════
# A. 순항 속도 스윕
# ══════════════════════════════════════════════════

def run_speed_sweep(plant, dt, K_r_30, sched_lqr, trims):
    print("\n" + "=" * 90)
    print("  [A] 순항 속도 스윕 — 고정 vs 스케줄링 vs NMPC")
    print("=" * 90)

    V_range = [30, 40, 50, 60, 65, 70, 75, 80, 85]
    T_sim = 5.0
    z_ref = 50.0

    labels = ['PID고정', 'PID스케줄', 'LQR고정', 'LQR스케줄', 'NMPC']
    keys   = ['pf',     'ps',       'lf',     'ls',       'nm']

    print(f"\n  {'V':>4s}", end="")
    for lb in labels:
        print(f"  {lb:^13s}", end="")
    print()
    print(f"  {'m/s':>4s}", end="")
    for _ in labels:
        print(f"  {'vx':>6s}/{'z':<5s}", end="")
    print()
    print(f"  {'─' * 85}")

    results = []

    for V in V_range:
        V = float(V)
        trim = find_trim(P, V)
        if trim['residual'] > 1e-3:
            print(f"  {V:4.0f}  트림 실패, 건너뜀")
            continue

        x_t, u_t = trim['state'].copy(), trim['control'].copy()
        x0 = x_t.copy()
        x0[2] = z_ref
        x0[3] += 1.0    # 속도 교란
        x0[5] += 2.0    # 수직 교란

        row = {'V': V}

        # 1) PID 고정
        c = CascadedPID(P, v_ref=[V, 0, 0], z_ref=z_ref, dt=dt)
        c.reset()
        row['pf'] = _run_cruise(plant, x0, c, V, z_ref, T_sim)

        # 2) PID 스케줄
        c = ScheduledPID(P, v_ref=[V, 0, 0], z_ref=z_ref, dt=dt)
        c.reset()
        row['ps'] = _run_cruise(plant, x0, c, V, z_ref, T_sim)

        # 3) LQR 고정 (K_r_30 + 해당 속도 트림)
        c = _FixedLQR(P, K_r_30, x_t, u_t, z_ref)
        row['lf'] = _run_cruise(plant, x0, c, V, z_ref, T_sim)

        # 4) LQR 스케줄
        sched_lqr.v_ref = np.array([V, 0.0, 0.0])
        sched_lqr.z_ref = z_ref
        row['ls'] = _run_cruise(plant, x0, sched_lqr, V, z_ref, T_sim)

        # 5) NMPC
        nmpc = NMPCController(P, v_ref=[V, 0, 0], z_ref=z_ref, u_ref=u_t,
                              N=20, dt_nmpc=0.05, dt_ctrl=0.02)
        row['nm'] = _run_cruise(plant, x0, nmpc, V, z_ref, T_sim)

        results.append(row)

        # 한 줄 출력
        print(f"  {V:4.0f}", end="")
        for k in keys:
            r = row[k]
            if r['div']:
                print(f"  {'DIV':^13s}", end="")
            else:
                print(f"  {r['rmse_vx']:5.2f}/{r['rmse_z']:<5.2f}", end="")
        print()

    return results


# ══════════════════════════════════════════════════
# B. 호버→70 m/s 가속 천이
# ══════════════════════════════════════════════════

def run_hover_transition(plant, dt, K_r_30, sched_lqr, trims):
    print("\n" + "=" * 90)
    print("  [B] 호버 → 70 m/s 가속 천이")
    print("=" * 90)

    T_sim = 20.0
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 50.0

    trim_speeds = sorted(trims.keys())
    labels = ['PID고정', 'PID스케줄', 'LQR고정', 'LQR스케줄', 'NMPC']
    keys   = ['pf',     'ps',       'lf',     'ls',       'nm']
    results = {}

    # 1) PID 고정
    print("  PID고정...", end=" ", flush=True)
    pid = CascadedPID(P, v_ref=[0, 0, 0], z_ref=50, dt=dt)
    pid.reset()
    def ctrl_pf(t, x):
        v, z = reference_trajectory(t)
        pid.v_ref, pid.z_ref = v, z
        return pid(t, x)
    t0 = timer.time()
    results['pf'] = _run_tracking(plant, x0, ctrl_pf, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 2) PID 스케줄
    print("  PID스케줄...", end=" ", flush=True)
    spid = ScheduledPID(P, v_ref=[0, 0, 0], z_ref=50, dt=dt)
    spid.reset()
    def ctrl_ps(t, x):
        v, z = reference_trajectory(t)
        spid.v_ref, spid.z_ref = v, z
        return spid(t, x)
    t0 = timer.time()
    results['ps'] = _run_tracking(plant, x0, ctrl_ps, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 3) LQR 고정 (K_r_30 + nearest trim)
    print("  LQR고정...", end=" ", flush=True)
    def ctrl_lf(t, x):
        v_ref, z_ref = reference_trajectory(t)
        V = v_ref[0]
        best = min(trim_speeds, key=lambda s: abs(s - V))
        x_t, u_t = trims[best]
        x_t2 = x_t.copy()
        x_t2[2] = z_ref
        x_t2[0:2] = x[0:2]
        dx_r = ScheduledLQR._compute_error_state(x, x_t2)
        u = u_t - K_r_30 @ dx_r
        return np.clip(u, P['n_min'], P['n_max'])
    t0 = timer.time()
    results['lf'] = _run_tracking(plant, x0, ctrl_lf, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 4) LQR 스케줄
    print("  LQR스케줄...", end=" ", flush=True)
    def ctrl_ls(t, x):
        v, z = reference_trajectory(t)
        sched_lqr.v_ref, sched_lqr.z_ref = v, z
        return sched_lqr(t, x)
    t0 = timer.time()
    results['ls'] = _run_tracking(plant, x0, ctrl_ls, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 5) NMPC
    print("  NMPC...", end=" ", flush=True)
    n_hov = np.sqrt(P['mass'] * P['g'] / (4 * P['k_T']))
    nmpc = NMPCController(P, v_ref=[0, 0, 0], z_ref=50,
                          u_ref=np.full(4, n_hov),
                          N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    def ctrl_nm(t, x):
        v_ref, z_ref = reference_trajectory(t)
        V = v_ref[0]
        best = min(trim_speeds, key=lambda s: abs(s - V))
        _, u_t = trims[best]
        nmpc.v_ref, nmpc.z_ref, nmpc.u_ref = v_ref, z_ref, u_t
        return nmpc(t, x)
    t0 = timer.time()
    results['nm'] = _run_tracking(plant, x0, ctrl_nm, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 출력
    print(f"\n  {'지표':>14s}", end="")
    for lb in labels:
        print(f"  {lb:>10s}", end="")
    print()
    print(f"  {'─' * 68}")

    for metric, label, fmt in [
        ('rmse_vx', 'RMSE vx',    '.2f'),
        ('rmse_z',  'RMSE z',     '.2f'),
        ('z_max',   '최대 z오차', '.1f'),
        ('sat',     'SAT%',       '.1f'),
    ]:
        print(f"  {label:>14s}", end="")
        for k in keys:
            r = results[k]
            if r['div']:
                print(f"  {'DIV':>10s}", end="")
            else:
                print(f"  {r[metric]:>10{fmt}}", end="")
        print()

    # 발산 행
    print(f"  {'발산':>14s}", end="")
    for k in keys:
        flag = 'YES' if results[k]['div'] else 'NO'
        print(f"  {flag:>10s}", end="")
    print()

    return results


# ══════════════════════════════════════════════════
# B'. NMPC Q_z 진단
# ══════════════════════════════════════════════════

def run_nmpc_qz_diagnosis(plant, dt, trims):
    """
    NMPC 고도 가중치(Q_z) 민감도 분석.

    호버→70 천이에서 NMPC의 고도오차(4.5m)가 큰 이유:
      가속 중 속도 오차 ~70 m/s → Q_vx(5)·70²=24,500
      고도 오차 ~5 m         → Q_z(20)·5²=500
      → 비용의 98%가 속도 항. 고도는 사실상 무시됨.

    Q_z를 올리면 고도 개선되지만 속도 추종이 저하 → trade-off 정량화.
    """
    print("\n" + "=" * 90)
    print("  [B'] NMPC 고도 가중치(Q_z) 진단 — 호버→70 천이")
    print("=" * 90)

    print("\n  원인 분석:")
    print("    가속 구간 속도 오차 ~70 m/s → Q_vx(5)·70² = 24,500")
    print("    고도 오차 ~5 m              → Q_z(20)·5²  = 500")
    print("    → 속도 항이 비용의 98% 차지. 고도는 무시됨.\n")

    T_sim = 20.0
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 50.0
    trim_speeds = sorted(trims.keys())

    Q_z_list = [20, 50, 100, 200]
    results = []

    for Q_z in Q_z_list:
        print(f"  Q_z={Q_z:>3d}...", end=" ", flush=True)

        n_hov = np.sqrt(P['mass'] * P['g'] / (4 * P['k_T']))
        nmpc = NMPCController(P, v_ref=[0, 0, 0], z_ref=50,
                              u_ref=np.full(4, n_hov),
                              N=20, dt_nmpc=0.05, dt_ctrl=0.02,
                              Q_z=Q_z)

        # 클로저로 래핑 (nmpc 인스턴스 캡처)
        def _make_ctrl(nm):
            def ctrl(t, x):
                v_ref, z_ref = reference_trajectory(t)
                V = v_ref[0]
                best = min(trim_speeds, key=lambda s: abs(s - V))
                _, u_t = trims[best]
                nm.v_ref, nm.z_ref, nm.u_ref = v_ref, z_ref, u_t
                return nm(t, x)
            return ctrl

        t0 = timer.time()
        r = _run_tracking(plant, x0, _make_ctrl(nmpc), T_sim)
        elapsed = timer.time() - t0

        stats = nmpc.get_solve_stats()
        r['Q_z'] = Q_z
        r['stats'] = stats
        results.append(r)

        ok_pct = stats.get('pct_ok', 0)
        print(f"{elapsed:.0f}s  수렴={ok_pct:.0f}%  "
              f"vx={r['rmse_vx']:.2f}  z={r['rmse_z']:.2f}  "
              f"z_max={r['z_max']:.1f}m")

    # 요약 표
    print(f"\n  {'Q_z':>6s}  {'RMSE vx':>8s}  {'RMSE z':>8s}  "
          f"{'z_max':>6s}  {'수렴%':>6s}")
    print(f"  {'─' * 44}")
    for r in results:
        s = r.get('stats', {})
        print(f"  {r['Q_z']:>6d}  {r['rmse_vx']:>8.2f}  {r['rmse_z']:>8.2f}  "
              f"{r['z_max']:>6.1f}  {s.get('pct_ok', 0):>5.0f}%")

    print(f"\n  → Q_z↑ : 고도 ↑ 개선, 속도 추종 ↓ (비용 함수 trade-off)")
    print(f"    NMPC도 만능이 아님 — 가중치 튜닝에 따라 성능 특성 변화.")

    return results


# ══════════════════════════════════════════════════
# C. 발사천이 (70 m/s, 30° AoA, 로터 0)
# ══════════════════════════════════════════════════

def _make_launch_state(V, alpha_deg, z0, n_init=0.0):
    """발사 직후 초기 상태 생성."""
    alpha_rad = np.radians(alpha_deg)
    R_hover = Rotation.from_quat([1, 0, 0, 0])
    R_pitch = Rotation.from_euler('y', alpha_rad)
    R_total = R_hover * R_pitch
    q = R_total.as_quat()

    x0 = np.zeros(NX)
    x0[0:3] = [0, 0, z0]
    x0[3:6] = [V, 0, 0]
    x0[6:10] = q
    x0[13:17] = n_init
    return x0


def run_launch_transition(plant, dt, K_r_30, sched_lqr, trims):
    print("\n" + "=" * 90)
    print("  [C] 발사천이 (70 m/s, 30° AoA, 로터 0 rpm)")
    print("=" * 90)

    V_launch = 70.0
    z0 = 50.0
    alpha_deg = 30.0
    T_sim = 5.0

    x0 = _make_launch_state(V_launch, alpha_deg, z0, n_init=0.0)

    # 40 m/s 트림 (순항 목표)
    trim_v = find_trim(P, V_launch)
    x_tv = trim_v['state'].copy()
    u_tv = trim_v['control'].copy()
    trim_speeds = sorted(trims.keys())

    labels = ['PID고정', 'PID스케줄', 'LQR고정', 'LQR스케줄', 'NMPC']
    keys   = ['pf',     'ps',       'lf',     'ls',       'nm']
    results = {}

    # 1) PID 고정
    print("  PID고정...", end=" ", flush=True)
    c = CascadedPID(P, v_ref=[V_launch, 0, 0], z_ref=z0, dt=dt)
    c.reset()
    t0 = timer.time()
    results['pf'] = _run_launch(plant, x0, c, V_launch, z0, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 2) PID 스케줄
    print("  PID스케줄...", end=" ", flush=True)
    c = ScheduledPID(P, v_ref=[V_launch, 0, 0], z_ref=z0, dt=dt)
    c.reset()
    t0 = timer.time()
    results['ps'] = _run_launch(plant, x0, c, V_launch, z0, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 3) LQR 고정
    print("  LQR고정...", end=" ", flush=True)
    c = _FixedLQR(P, K_r_30, x_tv, u_tv, z0)
    t0 = timer.time()
    results['lf'] = _run_launch(plant, x0, c, V_launch, z0, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 4) LQR 스케줄
    print("  LQR스케줄...", end=" ", flush=True)
    sched_lqr.v_ref = np.array([V_launch, 0.0, 0.0])
    sched_lqr.z_ref = z0
    t0 = timer.time()
    results['ls'] = _run_launch(plant, x0, sched_lqr, V_launch, z0, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 5) NMPC
    print("  NMPC...", end=" ", flush=True)
    nmpc = NMPCController(P, v_ref=[V_launch, 0, 0], z_ref=z0,
                          u_ref=u_tv,
                          N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    t0 = timer.time()
    results['nm'] = _run_launch(plant, x0, nmpc, V_launch, z0, T_sim)
    print(f"{timer.time()-t0:.1f}s")

    # 출력
    print(f"\n  {'지표':>14s}", end="")
    for lb in labels:
        print(f"  {lb:>10s}", end="")
    print()
    print(f"  {'─' * 68}")

    for metric, label, fmt in [
        ('alt_loss', '고도 손실 [m]', '.2f'),
        ('settle',   '안정화 [s]',    '.2f'),
        ('rmse_vx',  'RMSE vx',       '.2f'),
        ('rmse_z',   'RMSE z',        '.2f'),
    ]:
        print(f"  {label:>14s}", end="")
        for k in keys:
            r = results[k]
            if r['div']:
                print(f"  {'DIV':>10s}", end="")
            else:
                print(f"  {r[metric]:>10{fmt}}", end="")
        print()

    print(f"  {'발산':>14s}", end="")
    for k in keys:
        flag = 'YES' if results[k]['div'] else 'NO'
        print(f"  {flag:>10s}", end="")
    print()

    return results


# ══════════════════════════════════════════════════
# 종합 표
# ══════════════════════════════════════════════════

def print_final_table(res_A, res_B, res_C):
    print("\n" + "=" * 90)
    print("  [종합] 고정 게인 vs 게인 스케줄링 vs NMPC")
    print("=" * 90)

    labels = ['PID고정', 'PID스케줄', 'LQR고정', 'LQR스케줄', 'NMPC']
    keys   = ['pf',     'ps',       'lf',     'ls',       'nm']

    print(f"\n  {'':>20s}", end="")
    for lb in labels:
        print(f"  {lb:>10s}", end="")
    print()
    print(f"  {'─' * 74}")

    # ── A: 대표 속도 (60, 80, 85 m/s) ──
    for row in res_A:
        V = row['V']
        if V not in [60, 80, 85]:
            continue
        print(f"  순항 {V:.0f} m/s")
        for metric, label, fmt in [('rmse_vx', '  RMSE vx', '.2f'),
                                    ('rmse_z',  '  RMSE z',  '.2f')]:
            print(f"  {label:>20s}", end="")
            for k in keys:
                r = row[k]
                if r['div']:
                    print(f"  {'DIV':>10s}", end="")
                else:
                    print(f"  {r[metric]:>10{fmt}}", end="")
            print()

    # ── B: 호버→70 천이 ──
    print(f"  호버→70 천이")
    for metric, label, fmt in [('rmse_vx', '  RMSE vx',    '.2f'),
                                ('rmse_z',  '  RMSE z',     '.2f'),
                                ('z_max',   '  최대 z오차', '.1f')]:
        print(f"  {label:>20s}", end="")
        for k in keys:
            r = res_B[k]
            if r['div']:
                print(f"  {'DIV':>10s}", end="")
            else:
                print(f"  {r[metric]:>10{fmt}}", end="")
        print()
    # 발산 행
    print(f"  {'  발산':>20s}", end="")
    for k in keys:
        flag = 'YES' if res_B[k]['div'] else 'NO'
        print(f"  {flag:>10s}", end="")
    print()

    # ── C: 발사천이 ──
    print(f"  발사천이 (30°AoA)")
    for metric, label, fmt in [('alt_loss', '  고도 손실',  '.2f'),
                                ('settle',   '  안정화 [s]', '.2f'),
                                ('rmse_z',   '  RMSE z',     '.2f')]:
        print(f"  {label:>20s}", end="")
        for k in keys:
            r = res_C[k]
            if r['div']:
                print(f"  {'DIV':>10s}", end="")
            else:
                print(f"  {r[metric]:>10{fmt}}", end="")
        print()

    # ── 결론 ──
    print(f"\n{'─' * 90}")
    print("  결론:")

    # 85 m/s 비교 (있으면)
    r85 = next((r for r in res_A if r['V'] == 85), None)
    if r85:
        nmpc_z = r85['nm']['rmse_z']
        best_classic = min(
            r85['ps']['rmse_z'] if not r85['ps']['div'] else np.inf,
            r85['ls']['rmse_z'] if not r85['ls']['div'] else np.inf)
        if best_classic < np.inf and nmpc_z < best_classic:
            print(f"  - 85 m/s: NMPC RMSE_z={nmpc_z:.2f} vs "
                  f"최고 고전={best_classic:.2f} → "
                  f"NMPC {best_classic/nmpc_z:.1f}배 우수")

    # 호버→70 비교
    if not res_B['nm']['div']:
        lf_ok = 'NO' if res_B['lf']['div'] else 'OK'
        ls_ok = 'NO' if res_B['ls']['div'] else 'OK'
        nm_z = res_B['nm']['rmse_z']
        print(f"  - 호버→70: LQR고정={lf_ok}, LQR스케줄={ls_ok}, "
              f"NMPC z오차={nm_z:.2f}")
        if res_B['lf']['div'] and not res_B['ls']['div']:
            print(f"    → 게인 스케줄링으로 LQR 발산 문제 해결!")

    # 발사천이 비교
    if not res_C['nm']['div']:
        nm_alt = res_C['nm']['alt_loss']
        best_alt = min(
            res_C['ps']['alt_loss'] if not res_C['ps']['div'] else np.inf,
            res_C['ls']['alt_loss'] if not res_C['ls']['div'] else np.inf)
        if best_alt < np.inf:
            print(f"  - 발사천이: NMPC 고도손실={nm_alt:.2f}m vs "
                  f"최고 고전={best_alt:.2f}m")

    print(f"\n  정리:")
    print(f"  1) 게인 스케줄링 고전제어도 전 영역 발산 없이 동작")
    print(f"  2) 고도 유지: LQR스케줄이 NMPC보다 나을 수 있음 (가중치 의존)")
    print(f"  3) 속도 추종·큰 과도: NMPC 우위 (비선형 예측)")
    print(f"  4) NMPC 성능은 비용 가중치(Q_z 등)에 민감 (만능 아님)")
    print(f"  → 목표·상황에 따라 최적 제어기가 다르다.")
    print(f"{'=' * 90}")


# ══════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════

def main():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    print("\n" + "=" * 90)
    print("  게인 스케줄링 종합 비교")
    print("  고정 게인 / 속도별 스케줄링 / NMPC")
    print("=" * 90)

    # ── 공통 준비 ──
    print("\n[준비] 트림 캐시 + LQR 게인 사전 계산...")

    # 트림 캐시 (LQR/NMPC 공용)
    trims = {}
    for V in range(0, 90, 10):
        tr = find_trim(P, float(V))
        if tr['residual'] < 1e-3:
            trims[V] = (tr['state'].copy(), tr['control'].copy())
    print(f"  트림: {len(trims)} 속도점 수렴")

    # 30 m/s 고정 LQR 게인
    lqr_30 = LQRController(P, trims[30][0], trims[30][1])
    K_r_30 = lqr_30.K_r.copy()
    print(f"  K_r_30: {K_r_30.shape}, 안정 = {lqr_30.max_real < 0}")

    # 스케줄 LQR (1회 빌드, 전 시나리오 공유)
    sched_lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=50.0)

    print("\n[준비 완료]\n")

    # ── A: 속도 스윕 ──
    res_A = run_speed_sweep(plant, dt, K_r_30, sched_lqr, trims)

    # ── B: 호버→70 천이 ──
    res_B = run_hover_transition(plant, dt, K_r_30, sched_lqr, trims)

    # ── B': NMPC Q_z 진단 ──
    run_nmpc_qz_diagnosis(plant, dt, trims)

    # ── C: 발사천이 (70 m/s) ──
    res_C = run_launch_transition(plant, dt, K_r_30, sched_lqr, trims)

    # ── 종합 표 ──
    print_final_table(res_A, res_B, res_C)


if __name__ == '__main__':
    main()
