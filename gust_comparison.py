"""
돌풍 외란 비교 — 70 m/s 순항 + 1-cosine 돌풍
=============================================

시나리오:
  70 m/s 순항 중 t=2s에 돌풍 주입.
  (a) 측풍 (y방향) 10 m/s — 횡방향 교란, 요/롤 응답
  (b) 수직돌풍 (z방향) 10 m/s — 받음각 급변, 피치 응답

돌풍 형태: 1-cosine 펄스 (FAR 25 표준)
  w(t) = (W_max/2)(1 - cos(2π(t-t0)/T_gust))

핵심 관심:
  - NMPC가 예측 불가 외란에서도 우위인가?
  - 고전제어의 즉각 피드백이 대등/우수할 수 있는가?
  - → "왜 INDI(빠른 피드백)가 필요한가"로 이어지는 근거
"""

import numpy as np
import time as timer

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import (CascadedPID, LQRController,
                        ScheduledPID, ScheduledLQR, INDIController)
from nmpc import NMPCController


# ══════════════════════════════════════════════════
# 1-cosine 돌풍 모델
# ══════════════════════════════════════════════════

def make_gust_fn(direction, W_max, t_start, T_gust):
    """
    1-cosine 돌풍 (FAR 25 / MIL-F-8785C 표준).

    물리: 항공기가 공간적 돌풍 필드를 통과하는 것을 시간 영역으로 변환.
    1-cosine = 이산 돌풍의 표준 모델 (항공 인증 기준).

    Parameters
    ----------
    direction : 'lateral' (y) 또는 'vertical' (z, 상승돌풍)
    W_max     : 돌풍 최대 속도 [m/s]
    t_start   : 돌풍 시작 시각 [s]
    T_gust    : 돌풍 지속 시간 [s]
    """
    def gust_fn(t):
        w = np.zeros(3)
        if t_start <= t <= t_start + T_gust:
            pulse = (W_max / 2.0) * (1.0 - np.cos(2 * np.pi * (t - t_start) / T_gust))
            if direction == 'lateral':
                w[1] = pulse     # y: 측풍
            elif direction == 'vertical':
                w[2] = pulse     # z: 상승 돌풍 (z-up 관성)
        return w
    return gust_fn


# ══════════════════════════════════════════════════
# 측정
# ══════════════════════════════════════════════════

def measure_gust_response(ts, xs, us, V_ref, z_ref, t_gust_start, params):
    """돌풍 응답 지표 계산."""
    n_max = params['n_max']
    i0 = np.searchsorted(ts, t_gust_start)

    # 편차
    dy = xs[:, 1] - xs[i0, 1]        # 돌풍 직전 기준 횡편차
    dz = xs[:, 2] - z_ref            # 고도 기준
    dvx = xs[:, 3] - V_ref           # 속도 기준

    max_dy = np.max(np.abs(dy[i0:]))
    max_dz = np.max(np.abs(dz[i0:]))
    max_dvx = np.max(np.abs(dvx[i0:]))

    # 최대 각속도 (돌풍 이후)
    omega = np.sqrt(xs[i0:, 10]**2 + xs[i0:, 11]**2 + xs[i0:, 12]**2)
    max_omega = np.max(omega)

    # 회복 시간: |dvx|<0.5 AND |dz|<0.5 가 50스텝(0.05s) 연속
    settle = ts[-1] - t_gust_start   # default: 미회복
    for i in range(i0, len(ts) - 50):
        sl = slice(i, i + 50)
        if np.all(np.abs(dvx[sl]) < 0.5) and np.all(np.abs(dz[sl]) < 0.5):
            settle = ts[i] - t_gust_start
            break

    # RMSE (돌풍 이후)
    rmse_vx = np.sqrt(np.mean(dvx[i0:]**2))
    rmse_z = np.sqrt(np.mean(dz[i0:]**2))

    # 로터 포화
    sat = 100 * np.mean(us[i0:] >= 0.95 * n_max)

    # 발산
    div = bool(np.any(np.isnan(xs)) or np.any(np.abs(dz) > 50))

    return {
        'max_dy': max_dy, 'max_dz': max_dz, 'max_dvx': max_dvx,
        'max_omega': max_omega, 'settle': settle,
        'rmse_vx': rmse_vx, 'rmse_z': rmse_z,
        'sat': sat, 'div': div,
    }


# ══════════════════════════════════════════════════
# 간이 고정 LQR
# ══════════════════════════════════════════════════

class _FixedLQR:
    """K_r 고정(30 m/s) + 해당 속도 트림."""
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
# 메인
# ══════════════════════════════════════════════════

def main():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    V_cruise = 70.0
    z_ref = 50.0
    t_gust = 2.0       # 돌풍 시작 (2초 정상비행 후)
    T_gust = 1.0       # 돌풍 지속
    W_max = 10.0        # 돌풍 강도
    T_sim = 8.0         # 총 시뮬 (2s 정착 + 1s 돌풍 + 5s 회복)

    print("\n" + "=" * 90)
    print("  [3] 돌풍 외란 비교 — 70 m/s 순항 + 1-cosine 돌풍")
    print("=" * 90)
    print(f"  순항 속도: {V_cruise} m/s,  돌풍: {W_max} m/s 1-cosine,  "
          f"t={t_gust}s~{t_gust+T_gust}s")

    # ── 준비 ──
    print("\n[준비]")
    trim = find_trim(P, V_cruise)
    x_trim = trim['state'].copy()
    u_trim = trim['control'].copy()
    x0 = x_trim.copy()
    x0[2] = z_ref
    print(f"  70 m/s 트림: θ={np.degrees(trim['theta']):.1f}°, "
          f"α={np.degrees(trim['alpha']):.1f}°, "
          f"잔차={trim['residual']:.1e}")

    # 트림 캐시
    trims = {}
    for V in range(0, 90, 10):
        tr = find_trim(P, float(V))
        if tr['residual'] < 1e-3:
            trims[V] = (tr['state'].copy(), tr['control'].copy())

    # 30 m/s LQR 게인
    lqr_30 = LQRController(P, trims[30][0], trims[30][1])
    K_r_30 = lqr_30.K_r.copy()

    # 스케줄 LQR (1회 빌드)
    sched_lqr = ScheduledLQR(P, v_ref=[V_cruise, 0, 0], z_ref=z_ref)

    # 제어기 이름/키
    ctrl_names = ['PID고정', 'PID스케줄', 'LQR고정', 'LQR스케줄', 'NMPC', 'INDI']

    # 돌풍 시나리오
    gusts = [
        ('측풍 (y) 10 m/s',    make_gust_fn('lateral',  W_max, t_gust, T_gust)),
        ('수직돌풍 (z) 10 m/s', make_gust_fn('vertical', W_max, t_gust, T_gust)),
    ]

    # ── 돌풍 시나리오 각각 실행 ──
    all_results = {}

    for gust_name, gust_fn in gusts:
        print(f"\n{'─' * 90}")
        print(f"  {gust_name}")
        print(f"  1-cosine, t={t_gust}~{t_gust+T_gust}s, 최대={W_max} m/s")
        print(f"{'─' * 90}")

        # 매 시나리오마다 제어기 새로 생성 (적분기 초기화)
        controllers = [
            ('PID고정',   CascadedPID(P, v_ref=[V_cruise,0,0], z_ref=z_ref, dt=dt)),
            ('PID스케줄', ScheduledPID(P, v_ref=[V_cruise,0,0], z_ref=z_ref, dt=dt)),
            ('LQR고정',   _FixedLQR(P, K_r_30, x_trim, u_trim, z_ref)),
            ('LQR스케줄', sched_lqr),
            ('NMPC',      NMPCController(P, v_ref=[V_cruise,0,0], z_ref=z_ref,
                                         u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)),
            ('INDI',      INDIController(P, v_ref=[V_cruise,0,0], z_ref=z_ref, dt=dt)),
        ]
        # LQR 스케줄 기준 재설정
        sched_lqr.v_ref = np.array([V_cruise, 0.0, 0.0])
        sched_lqr.z_ref = z_ref

        gust_results = {}

        for name, ctrl in controllers:
            if hasattr(ctrl, 'reset'):
                ctrl.reset()

            print(f"  {name:>10s}...", end=" ", flush=True)
            t0 = timer.time()
            ts, xs, us = plant.simulate(x0.copy(), ctrl, T_sim, wind_fn=gust_fn)
            elapsed = timer.time() - t0

            r = measure_gust_response(ts, xs, us, V_cruise, z_ref, t_gust, P)
            gust_results[name] = r

            if r['div']:
                print(f"{elapsed:.1f}s  DIVERGED!")
            else:
                print(f"{elapsed:.1f}s  Δz={r['max_dz']:.2f}m  "
                      f"Δvx={r['max_dvx']:.2f}m/s  회복={r['settle']:.2f}s")

        all_results[gust_name] = gust_results

        # ── 시나리오별 상세 표 ──
        print(f"\n  {'지표':>14s}", end="")
        for n in ctrl_names:
            print(f"  {n:>10s}", end="")
        print()
        print(f"  {'─' * 68}")

        for metric, label, fmt in [
            ('max_dz',    '최대 Δz [m]',   '.2f'),
            ('max_dy',    '최대 Δy [m]',   '.3f'),
            ('max_dvx',   '최대 Δvx',      '.2f'),
            ('max_omega', '최대 ω [r/s]',  '.2f'),
            ('settle',    '회복 시간 [s]',  '.2f'),
            ('rmse_vx',   'RMSE vx',       '.3f'),
            ('rmse_z',    'RMSE z',        '.3f'),
            ('sat',       'SAT%',          '.1f'),
        ]:
            print(f"  {label:>14s}", end="")
            for n in ctrl_names:
                r = gust_results[n]
                if r['div']:
                    print(f"  {'DIV':>10s}", end="")
                else:
                    print(f"  {r[metric]:>10{fmt}}", end="")
            print()

    # ══════════════════════════════════════════════════
    # 종합
    # ══════════════════════════════════════════════════
    print(f"\n{'=' * 90}")
    print("  [종합] 돌풍 응답 비교")
    print(f"{'=' * 90}")

    print(f"\n  {'':>22s}", end="")
    for n in ctrl_names:
        print(f"  {n:>10s}", end="")
    print()
    print(f"  {'─' * 88}")

    for gust_name, gr in all_results.items():
        short = gust_name.split('(')[1].split(')')[0]  # 'y' or 'z'
        print(f"  {gust_name}")
        for metric, label, fmt in [
            ('max_dz',  '  최대 Δz [m]',  '.2f'),
            ('max_dvx', '  최대 Δvx',     '.2f'),
            ('settle',  '  회복 [s]',     '.2f'),
            ('rmse_z',  '  RMSE z',       '.3f'),
        ]:
            print(f"  {label:>22s}", end="")
            for n in ctrl_names:
                r = gr[n]
                if r['div']:
                    print(f"  {'DIV':>10s}", end="")
                else:
                    print(f"  {r[metric]:>10{fmt}}", end="")
            print()

    # ── 분석 ──
    print(f"\n{'─' * 90}")
    print("  분석:")

    for gust_name, gr in all_results.items():
        valid = {n: r for n, r in gr.items() if not r['div']}
        if not valid:
            continue

        # 최빠른 회복
        best_n = min(valid, key=lambda n: valid[n]['settle'])
        best_s = valid[best_n]['settle']
        nmpc_s = gr['NMPC']['settle'] if not gr['NMPC']['div'] else float('inf')

        # 최소 고도 편차
        best_dz_n = min(valid, key=lambda n: valid[n]['max_dz'])
        best_dz = valid[best_dz_n]['max_dz']
        nmpc_dz = gr['NMPC']['max_dz'] if not gr['NMPC']['div'] else float('inf')

        print(f"\n  {gust_name}:")
        print(f"    최빠른 회복: {best_n} ({best_s:.2f}s)"
              + (f" — NMPC({nmpc_s:.2f}s)보다 빠름" if best_n != 'NMPC' and nmpc_s > best_s else ""))
        print(f"    최소 Δz:     {best_dz_n} ({best_dz:.2f}m)"
              + (f" — NMPC({nmpc_dz:.2f}m)보다 작음" if best_dz_n != 'NMPC' and nmpc_dz > best_dz else ""))

    print(f"\n  핵심:")
    print(f"  - 예측 불가 외란: NMPC의 '예측' 장점이 무력화됨")
    print(f"    (NMPC 내부 모델에 바람이 없으므로 돌풍을 예측 못 함)")
    print(f"  - 고전제어의 즉각 피드백이 대등하거나 우수할 수 있음")
    print(f"  - 빠른 외란 억제가 목표면 → INDI(센서 기반 증분 제어) 필요")
    print(f"  - NMPC의 진짜 강점은 '예측 가능한' 상황 (자세천이, 가속, 제약)")
    print(f"{'=' * 90}")


if __name__ == '__main__':
    main()
