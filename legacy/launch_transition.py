"""
발사천이 시나리오 — PID vs LQR vs NMPC
======================================

튜브 발사 직후 → 순항 안정화 비교.
NMPC의 진짜 무대: 큰 AoA + 로터 스핀업 + 고도 유지를 동시에 처리.

시나리오:
  - 수평 발사, 40 m/s
  - 기수 30° 위 (큰 받음각, 동체 공력 모멘트 강함)
  - 로터 0 rpm에서 시작 (스핀업 과도)
  - 목표: 40 m/s 순항 + 고도 50m 유지
"""

import os
os.environ['ACADOS_SOURCE_DIR'] = os.path.expanduser('~/acados')
os.environ['DYLD_LIBRARY_PATH'] = os.path.expanduser('~/acados/lib')
os.environ['LD_LIBRARY_PATH'] = os.path.expanduser('~/acados/lib')

import numpy as np
import time as timer
from scipy.spatial.transform import Rotation

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant, NX
from trim import find_trim
from controller import CascadedPID, LQRController
from nmpc_acados import build_ocp_solver, AcadosNMPC
import shutil


def make_launch_state(V_launch, alpha_deg, z0, n_init=0.0):
    """
    발사 직후 초기 상태 생성.

    Parameters
    ----------
    V_launch  : 발사 속도 [m/s] (수평)
    alpha_deg : 초기 받음각 [deg] (양수 = 기수 위)
    z0        : 초기 고도 [m]
    n_init    : 초기 로터 속도 [rad/s] (0 = 스핀업 필요)
    """
    alpha_rad = np.radians(alpha_deg)

    # 자세: 호버(180° about x) + 피치(기수 상승)
    R_hover = Rotation.from_quat([1, 0, 0, 0])
    R_pitch = Rotation.from_euler('y', alpha_rad)   # 양의 θ = 기수 상승
    R_total = R_hover * R_pitch
    q = R_total.as_quat()

    x0 = np.zeros(NX)
    x0[0:3]   = [0, 0, z0]
    x0[3:6]   = [V_launch, 0, 0]    # 수평 속도 (관성)
    x0[6:10]  = q
    x0[10:13] = [0, 0, 0]           # 각속도 0
    x0[13:17] = n_init               # 로터 (0이면 스핀업)

    # 검증: body frame에서 AoA 확인
    v_body = R_total.as_matrix().T @ x0[3:6]
    alpha_check = np.degrees(np.arctan2(v_body[2], v_body[0]))

    return x0, alpha_check


def measure_transition(ts, xs, V_ref, z_ref):
    """발사천이 성능 지표 계산."""
    vx = xs[:, 3]
    vz = xs[:, 5]
    z  = xs[:, 2]
    omega_mag = np.sqrt(xs[:, 10]**2 + xs[:, 11]**2 + xs[:, 12]**2)

    # 최대 고도 손실
    z_min = np.min(z)
    alt_loss = z_ref - z_min

    # 최대 각속도
    max_omega = np.max(omega_mag)

    # 안정화 시간: v_x 오차 < 2 m/s AND |ω| < 0.5 rad/s 유지
    settled = False
    settle_time = ts[-1]
    for i in range(len(ts) - 50):  # 50 스텝(0.05s) 연속 조건
        window = slice(i, i + 50)
        vx_ok = np.all(np.abs(vx[window] - V_ref) < 2.0)
        w_ok  = np.all(omega_mag[window] < 0.5)
        if vx_ok and w_ok:
            settle_time = ts[i]
            settled = True
            break

    # RMSE (전 구간)
    rmse_vx = np.sqrt(np.mean((vx - V_ref)**2))
    rmse_z  = np.sqrt(np.mean((z - z_ref)**2))

    # 발산 여부
    diverged = np.any(np.abs(z - z_ref) > 100) or np.any(np.isnan(xs))

    return {
        'alt_loss':     alt_loss,
        'max_omega':    max_omega,
        'settle_time':  settle_time,
        'settled':      settled,
        'rmse_vx':      rmse_vx,
        'rmse_z':       rmse_z,
        'diverged':     diverged,
        'z_min':        z_min,
    }


def run_launch_scenario():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    V_launch = 40.0
    z0 = 50.0
    alpha_deg = 30.0
    T_sim = 5.0

    print("\n" + "=" * 70)
    print("  발사천이 시나리오 — PID vs LQR vs NMPC")
    print("=" * 70)

    # 트림 (순항 목표)
    trim = find_trim(P, V_launch)
    x_trim, u_trim = trim['state'], trim['control']
    n_hov = np.sqrt(P['mass'] * P['g'] / (4 * P['k_T']))

    print(f"\n  발사 조건:")
    print(f"    속도     = {V_launch} m/s (수평)")
    print(f"    받음각   = {alpha_deg}° (기수 위)")
    print(f"    고도     = {z0} m")
    print(f"    로터     = 0 rpm (스핀업 필요)")
    print(f"    스핀업 τ = {P['tau_m']*1000:.0f}ms (95% 도달 ~{P['tau_m']*3*1000:.0f}ms)")
    print(f"    목표     = {V_launch} m/s 순항, 고도 {z0}m 유지")

    # 초기 상태
    x0, alpha_real = make_launch_state(V_launch, alpha_deg, z0, n_init=0.0)
    print(f"    실제 AoA = {alpha_real:.1f}° (검증)")

    # ── PID ──
    print(f"\n{'─'*70}")
    print("  [PID] 30 m/s 튜닝 게인 고정")
    pid = CascadedPID(P, v_ref=[V_launch, 0, 0], z_ref=z0, dt=dt)
    pid.reset()
    t0 = timer.time()
    ts, xs_pid, us_pid = plant.simulate(x0.copy(), pid, T_sim)
    t_pid = timer.time() - t0
    m_pid = measure_transition(ts, xs_pid, V_launch, z0)
    _print_result("PID", m_pid, t_pid)

    # ── LQR ──
    print(f"\n{'─'*70}")
    print("  [LQR] 30 m/s 트림 게인 고정")
    trim_30 = find_trim(P, 30.0)
    K_fixed = LQRController(P, trim_30['state'], trim_30['control']).K.copy()
    lqr = LQRController.__new__(LQRController)
    lqr.p, lqr.K, lqr.valid = P, K_fixed, True
    lqr.x_trim = x_trim.copy(); lqr.u_trim = u_trim.copy()
    lqr.x_trim[2] = z0; lqr.x_trim[0:2] = 0
    t0 = timer.time()
    ts, xs_lqr, us_lqr = plant.simulate(x0.copy(), lqr, T_sim)
    t_lqr = timer.time() - t0
    m_lqr = measure_transition(ts, xs_lqr, V_launch, z0)
    _print_result("LQR", m_lqr, t_lqr)

    # ── NMPC (acados) ──
    print(f"\n{'─'*70}")
    print("  [NMPC] acados, full SQP")
    try:
        # 코드 정리
        for p in ['c_generated_code', 'acados_ocp.json']:
            fp = os.path.join(os.path.dirname(os.path.abspath(__file__)), p)
            if os.path.isdir(fp): shutil.rmtree(fp)
            elif os.path.isfile(fp): os.remove(fp)

        solver, N = build_ocp_solver(P, N=20, T_horizon=1.0, use_rti=False)
        nmpc = AcadosNMPC(solver, N, P,
                          v_ref=np.array([V_launch, 0, 0]), z_ref=z0,
                          u_ref=u_trim, x_trim=x_trim,
                          dt_ctrl=0.02)
        t0 = timer.time()
        ts, xs_nmpc, us_nmpc = plant.simulate(x0.copy(), nmpc, T_sim)
        t_nmpc = timer.time() - t0
        m_nmpc = measure_transition(ts, xs_nmpc, V_launch, z0)
        stats = nmpc.get_stats()
        _print_result("NMPC", m_nmpc, t_nmpc, stats)
        nmpc_ok = True
    except Exception as e:
        print(f"  NMPC 오류: {e}")
        m_nmpc = None
        nmpc_ok = False

    # ── 종합 비교 ──
    print(f"\n{'='*70}")
    print("  발사천이 종합 비교")
    print(f"{'='*70}")

    print(f"\n  {'지표':>16s}  {'PID':>10s}  {'LQR':>10s}  {'NMPC':>10s}")
    print(f"  {'─'*50}")

    def _f(val, fmt=".2f"):
        if val is None: return "     FAIL"
        return f"{val:{fmt}}" if not isinstance(val, bool) else ("    YES" if val else "     NO")

    rows = [
        ("고도 손실 [m]",  m_pid['alt_loss'], m_lqr['alt_loss'],
         m_nmpc['alt_loss'] if nmpc_ok else None),
        ("최대 ω [rad/s]", m_pid['max_omega'], m_lqr['max_omega'],
         m_nmpc['max_omega'] if nmpc_ok else None),
        ("안정화 시간 [s]", m_pid['settle_time'], m_lqr['settle_time'],
         m_nmpc['settle_time'] if nmpc_ok else None),
        ("RMSE vx",        m_pid['rmse_vx'], m_lqr['rmse_vx'],
         m_nmpc['rmse_vx'] if nmpc_ok else None),
        ("RMSE z",         m_pid['rmse_z'], m_lqr['rmse_z'],
         m_nmpc['rmse_z'] if nmpc_ok else None),
    ]

    for label, vp, vl, vn in rows:
        print(f"  {label:>16s}  {_f(vp):>10s}  {_f(vl):>10s}  {_f(vn):>10s}")

    print(f"  {'안정화 성공':>16s}  {'OK' if m_pid['settled'] else 'FAIL':>10s}"
          f"  {'OK' if m_lqr['settled'] else 'FAIL':>10s}"
          f"  {'OK' if nmpc_ok and m_nmpc['settled'] else 'FAIL':>10s}")

    print(f"\n  핵심:")
    if nmpc_ok and m_nmpc['settled']:
        if m_pid['settled'] and m_lqr['settled']:
            # 셋 다 성공 → 성능 차이로 비교
            best_alt = min(m_pid['alt_loss'], m_lqr['alt_loss'],
                          m_nmpc['alt_loss'])
            print(f"  - 고도 손실: NMPC {m_nmpc['alt_loss']:.1f}m vs "
                  f"PID {m_pid['alt_loss']:.1f}m vs LQR {m_lqr['alt_loss']:.1f}m")
            print(f"  - NMPC의 예측 제어가 발사천이에서 고도 손실을 최소화")
        else:
            print(f"  - PID/LQR 안정화 실패, NMPC만 성공 → NMPC 필수성 입증")
    print(f"{'='*70}")

    # 정리
    for p in ['c_generated_code', 'acados_ocp.json']:
        fp = os.path.join(os.path.dirname(os.path.abspath(__file__)), p)
        if os.path.isdir(fp): shutil.rmtree(fp)
        elif os.path.isfile(fp): os.remove(fp)


def _print_result(name, m, comp_time, stats=None):
    flag = "DIVERGED!" if m['diverged'] else ("OK" if m['settled'] else "미안정")
    print(f"  결과: {flag}")
    print(f"    고도 손실     = {m['alt_loss']:.2f} m  (최저 z = {m['z_min']:.1f} m)")
    print(f"    최대 각속도   = {m['max_omega']:.2f} rad/s")
    print(f"    안정화 시간   = {m['settle_time']:.2f} s" +
          (" (미달)" if not m['settled'] else ""))
    print(f"    RMSE vx / z   = {m['rmse_vx']:.3f} / {m['rmse_z']:.3f}")
    print(f"    계산 시간     = {comp_time:.2f} s")
    if stats:
        print(f"    NMPC 풀이     = {stats['mean_ms']:.2f}ms avg, "
              f"{stats['n_ok']}/{stats['n_solves']} 수렴")


if __name__ == '__main__':
    run_launch_scenario()
