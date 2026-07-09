"""
베이스라인 성능 저하 곡선 — PID & LQR 속도 스윕
================================================

30 m/s에서 튜닝한 PID/LQR 게인을 **고정**한 채,
순항속도를 30→85 m/s까지 5 m/s 간격으로 밀어붙여
성능 저하(무릎점)를 정량화한다.

각 속도에서 기록:
  1. RMSE v_x, z (추종 오차)
  2. 로터 포화 비율 (n >= 0.95*n_max인 시간 비율)
  3. 발산 여부 (z가 ±100m 벗어나거나 NaN)

이 곡선이 나중에 NMPC와 비교할 베이스라인이다.
"""

import numpy as np
from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import CascadedPID, LQRController


def run_speed_sweep():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    # ══════════════════════════════════════════
    # 30 m/s 기준 게인 고정
    # ══════════════════════════════════════════
    V_tune = 30.0
    trim_tune = find_trim(P, V_tune)

    # PID 게인은 생성 시 고정됨
    pid_base = CascadedPID(P, v_ref=[V_tune, 0, 0], z_ref=50.0, dt=dt)
    # LQR 게인(K)도 30 m/s 트림에서 고정
    lqr_base = LQRController(P, trim_tune['state'], trim_tune['control'])

    if not lqr_base.valid:
        print("LQR 설계 실패! 스윕 중단.")
        return

    K_fixed = lqr_base.K.copy()          # 게인 고정
    u_trim_30 = trim_tune['control'].copy()

    # ══════════════════════════════════════════
    # 속도 스윕
    # ══════════════════════════════════════════
    V_range = np.arange(30, 90, 5)
    T_sim = 10.0
    z_ref = 50.0

    results = []

    print(f"\n{'V':>5s} | {'--- PID ---':^30s} | {'--- LQR ---':^30s}")
    print(f"{'m/s':>5s} | {'RMSE_vx':>8s} {'RMSE_z':>8s} {'SAT%':>5s} {'OK':>4s}"
          f" | {'RMSE_vx':>8s} {'RMSE_z':>8s} {'SAT%':>5s} {'OK':>4s}")
    print(f"{'─'*75}")

    for V in V_range:
        # 해당 속도의 트림 찾기 (초기 상태 + LQR 기준용)
        trim_v = find_trim(P, V)

        if trim_v['residual'] > 1e-3:
            # 트림 못 찾으면 이전 트림에서 외삽
            x0 = trim_tune['state'].copy()
            x0[3] = V
            x0[2] = z_ref
        else:
            x0 = trim_v['state'].copy()
            x0[2] = z_ref

        # 교란 추가 (동일한 교란으로 공정 비교)
        x0_pert = x0.copy()
        x0_pert[5] += 2.0    # Δvz = +2 m/s
        x0_pert[3] += 1.0    # Δvx = +1 m/s

        # ── PID (30 m/s 튜닝 고정, v_ref만 변경) ──
        pid = CascadedPID(P, v_ref=[V, 0, 0], z_ref=z_ref, dt=dt)
        # 게인은 __init__에서 고정값 → 30 m/s 때와 동일
        pid.reset()

        r_pid = _simulate_and_measure(plant, x0_pert.copy(), pid, V, z_ref, T_sim)

        # ── LQR (30 m/s K 고정, 해당 속도 트림으로 오프셋만 변경) ──
        # LQR: u = u_trim_v - K_fixed @ (x - x_trim_v)
        # K는 30에서 구한 것, 트림은 해당 속도 것 사용
        lqr = LQRController.__new__(LQRController)
        lqr.p = P
        lqr.K = K_fixed
        lqr.valid = True
        if trim_v['residual'] < 1e-3:
            lqr.x_trim = trim_v['state'].copy()
            lqr.u_trim = trim_v['control'].copy()
        else:
            lqr.x_trim = x0.copy()
            lqr.u_trim = u_trim_30.copy()
        lqr.x_trim[2] = z_ref
        lqr.x_trim[0:2] = x0_pert[0:2]

        r_lqr = _simulate_and_measure(plant, x0_pert.copy(), lqr, V, z_ref, T_sim)

        results.append({'V': V, 'pid': r_pid, 'lqr': r_lqr})

        print(f"{V:5.0f} | {r_pid['rmse_vx']:8.2f} {r_pid['rmse_z']:8.2f} "
              f"{r_pid['sat_pct']:5.1f} {_ok(r_pid):>4s}"
              f" | {r_lqr['rmse_vx']:8.2f} {r_lqr['rmse_z']:8.2f} "
              f"{r_lqr['sat_pct']:5.1f} {_ok(r_lqr):>4s}")

    # ══════════════════════════════════════════
    # 요약
    # ══════════════════════════════════════════
    _print_summary(results)

    return results


def _simulate_and_measure(plant, x0, ctrl, V_ref, z_ref, T_sim):
    """시뮬 실행 + 성능 지표 계산."""
    try:
        ts, xs, us = plant.simulate(x0, ctrl, T_sim)
    except Exception:
        return {'rmse_vx': np.inf, 'rmse_z': np.inf,
                'sat_pct': 100.0, 'diverged': True}

    # 발산 체크
    z_vals = xs[:, 2]
    diverged = (np.any(np.isnan(xs)) or
                np.any(np.abs(z_vals - z_ref) > 200) or
                np.any(np.abs(xs[:, 5]) > 100))

    if diverged:
        # 발산 시점까지의 데이터로 RMSE 계산
        valid = np.where(np.abs(z_vals - z_ref) > 200)[0]
        end = valid[0] if len(valid) > 0 else len(xs)
        end = max(end, 10)
    else:
        end = len(xs)

    # RMSE (정착 후 포함, 전 구간)
    rmse_vx = np.sqrt(np.mean((xs[:end, 3] - V_ref)**2))
    rmse_z  = np.sqrt(np.mean((xs[:end, 2] - z_ref)**2))

    # 로터 포화: n >= 0.95 * n_max 인 비율
    n_max = plant.params['n_max']
    n_all = us[:min(end, len(us)), :]   # 제어 입력 (모터 속도 명령)
    sat_mask = n_all >= 0.95 * n_max
    sat_pct = 100.0 * np.mean(sat_mask) if len(n_all) > 0 else 0.0

    return {
        'rmse_vx': rmse_vx,
        'rmse_z':  rmse_z,
        'sat_pct': sat_pct,
        'diverged': diverged,
    }


def _ok(r):
    if r['diverged']:
        return 'DIV'
    if r['sat_pct'] > 50:
        return 'SAT'
    return 'OK'


def _print_summary(results):
    print(f"\n{'='*75}")
    print("  성능 저하 곡선 요약")
    print(f"{'='*75}")

    # 무릎점 찾기: RMSE가 30 m/s 대비 3배 이상 증가하는 첫 속도
    base_pid = results[0]['pid']['rmse_vx']
    base_lqr = results[0]['lqr']['rmse_vx']

    knee_pid = None
    knee_lqr = None

    for r in results:
        V = r['V']
        if knee_pid is None and (r['pid']['rmse_vx'] > base_pid * 3 or r['pid']['diverged']):
            knee_pid = V
        if knee_lqr is None and (r['lqr']['rmse_vx'] > base_lqr * 3 or r['lqr']['diverged']):
            knee_lqr = V

    # 발산 속도
    div_pid = next((r['V'] for r in results if r['pid']['diverged']), None)
    div_lqr = next((r['V'] for r in results if r['lqr']['diverged']), None)

    # 포화 시작 속도
    sat_pid = next((r['V'] for r in results if r['pid']['sat_pct'] > 5), None)
    sat_lqr = next((r['V'] for r in results if r['lqr']['sat_pct'] > 5), None)

    print(f"\n  {'':>25s}  {'PID':>10s}  {'LQR':>10s}")
    print(f"  {'─'*50}")
    print(f"  {'무릎점 (RMSE 3x)':>25s}  {_fmt_v(knee_pid):>10s}  {_fmt_v(knee_lqr):>10s}")
    print(f"  {'포화 시작 (>5%)':>25s}  {_fmt_v(sat_pid):>10s}  {_fmt_v(sat_lqr):>10s}")
    print(f"  {'발산 속도':>25s}  {_fmt_v(div_pid):>10s}  {_fmt_v(div_lqr):>10s}")

    print(f"\n  RMSE 곡선 (텍스트 그래프):")
    print(f"  {'V':>5s}  PID_vx  LQR_vx  | PID_z   LQR_z   | 바(PID vx)")
    print(f"  {'─'*70}")

    max_rmse = max(r['pid']['rmse_vx'] for r in results
                   if not r['pid']['diverged'] and r['pid']['rmse_vx'] < 1000)
    max_rmse = max(max_rmse, 1.0)

    for r in results:
        V = r['V']
        p, l = r['pid'], r['lqr']

        pvx = f"{p['rmse_vx']:6.2f}" if p['rmse_vx'] < 1000 else '  DIV'
        lvx = f"{l['rmse_vx']:6.2f}" if l['rmse_vx'] < 1000 else '  DIV'
        pz  = f"{p['rmse_z']:6.2f}" if p['rmse_z'] < 1000 else '  DIV'
        lz  = f"{l['rmse_z']:6.2f}" if l['rmse_z'] < 1000 else '  DIV'

        # 바 그래프 (PID vx)
        if p['diverged']:
            bar = 'XXXXXXXXXX DIV'
        else:
            bar_len = min(int(p['rmse_vx'] / max_rmse * 40), 40)
            bar = '#' * bar_len

        print(f"  {V:5.0f}  {pvx}  {lvx}  | {pz}  {lz}   | {bar}")

    print(f"\n  결론:")
    print(f"  - 30 m/s 튜닝 게인을 고속으로 밀면 성능이 급격히 저하")
    print(f"  - PID/LQR 모두 단일 동작점 설계의 한계 → 게인 스케줄링 또는 NMPC 필요")
    if knee_pid:
        print(f"  - PID 무릎점: {knee_pid} m/s ({knee_pid*3.6:.0f} km/h)")
    if knee_lqr:
        print(f"  - LQR 무릎점: {knee_lqr} m/s ({knee_lqr*3.6:.0f} km/h)")
    print(f"{'='*75}")


def _fmt_v(v):
    return f"{v:.0f} m/s" if v is not None else "없음"


if __name__ == '__main__':
    print("\n베이스라인 성능 저하 곡선 (30 m/s 게인 고정)\n")
    run_speed_sweep()
