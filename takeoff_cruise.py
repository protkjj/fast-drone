"""
정지 → 이륙 → 고속순항 천이 시나리오
====================================

시나리오:
  t=0~2s:   이륙 (정지 → 고도 50m)
  t=2~15s:  가속 (0 → 70 m/s 순항)

초기: 호버 자세, 로터 호버 트림, v=0, z=0
목표: z=50m 유지하면서 70 m/s까지 가속

이게 어려운 이유:
  - 가속하려면 틸트 → 수직추력 감소 → 고도 유지 어려움
  - 속도 올라가면 동체 공력 비선형성 증가
  - PID/LQR은 30 m/s 튜닝 → 0~70 m/s 전 영역 커버 못 함
  - NMPC는 예측으로 틸트-추력 trade-off를 최적 관리
"""

import os
os.environ['ACADOS_SOURCE_DIR'] = os.path.expanduser('~/acados')
os.environ['DYLD_LIBRARY_PATH'] = os.path.expanduser('~/acados/lib')
os.environ['LD_LIBRARY_PATH'] = os.path.expanduser('~/acados/lib')

import numpy as np
import time as timer
import shutil

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant, NX
from trim import find_trim
from controller import CascadedPID, LQRController
from nmpc_acados import build_ocp_solver, AcadosNMPC


# ══════════════════════════════════════════════════
# 기준 궤적: 시간에 따른 속도/고도 명령
# ══════════════════════════════════════════════════

def reference_trajectory(t):
    """
    시간 → (v_ref[3], z_ref) 기준값.

    t=0~1s:   호버 안정화 (v=0, z=50)
    t=1~16s:  가속 (0→70 m/s, 선형), z=50 유지
    t=16s~:   순항 (v=70, z=50)

    가속도: 70/15 ≈ 4.7 m/s² (0.48 G, 현실적)
    """
    z_ref = 50.0

    if t < 1.0:
        vx_ref = 0.0
    elif t < 16.0:
        vx_ref = 70.0 * (t - 1.0) / 15.0
    else:
        vx_ref = 70.0

    return np.array([vx_ref, 0.0, 0.0]), z_ref


# ══════════════════════════════════════════════════
# 시간변화 기준을 추종하는 제어기 래퍼
# ══════════════════════════════════════════════════

class TrackingPID:
    """CascadedPID를 시간변화 기준으로 래핑."""
    def __init__(self, params, dt):
        self.pid = CascadedPID(params, v_ref=[0,0,0], z_ref=0, dt=dt)

    def reset(self):
        self.pid.reset()

    def __call__(self, t, x):
        v_ref, z_ref = reference_trajectory(t)
        self.pid.v_ref = v_ref
        self.pid.z_ref = z_ref
        return self.pid(t, x)


class TrackingLQR:
    """LQR을 시간변화 기준으로 래핑. 각 속도의 트림점을 보간."""
    def __init__(self, params, K_fixed, trims_cache):
        self.p = params
        self.K = K_fixed
        self.trims = trims_cache  # {V: (x_trim, u_trim)}

    def _get_trim(self, V):
        """가장 가까운 트림점 선택."""
        speeds = sorted(self.trims.keys())
        best = min(speeds, key=lambda s: abs(s - V))
        return self.trims[best]

    def __call__(self, t, x):
        v_ref, z_ref = reference_trajectory(t)
        V = v_ref[0]
        x_trim, u_trim = self._get_trim(V)

        x_t = x_trim.copy()
        x_t[2] = z_ref
        x_t[0:2] = x[0:2]

        dx = x - x_t
        dx[0:2] = 0.0
        u = u_trim - self.K @ dx
        return np.clip(u, self.p['n_min'], self.p['n_max'])


class TrackingNMPC:
    """acados NMPC를 시간변화 기준으로 래핑."""
    def __init__(self, solver, N, params, trims_cache, dt_ctrl=0.02):
        self.solver = solver
        self.N = N
        self.p = params
        self.trims = trims_cache
        self.dt_ctrl = dt_ctrl
        self.dt_nmpc = 1.0 / N  # T_horizon / N

        n_hov = np.sqrt(params['mass'] * params['g'] / (4 * params['k_T']))
        self._u_current = np.full(4, n_hov)
        self._last_t = -np.inf
        self.solve_times = []
        self.statuses = []

    def _get_trim(self, V):
        speeds = sorted(self.trims.keys())
        best = min(speeds, key=lambda s: abs(s - V))
        return self.trims[best]

    def __call__(self, t, x):
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            self._u_current = self._solve(t, x)
            self._last_t = t
        return self._u_current

    def _solve(self, t, x_current):
        t0_wall = timer.time()

        # 예측 지평선의 각 노드에 시간변화 기준 설정
        for k in range(self.N):
            t_k = t + k * self.dt_nmpc
            v_ref_k, z_ref_k = reference_trajectory(t_k)
            V_k = v_ref_k[0]
            _, u_trim_k = self._get_trim(V_k)

            yref_k = np.concatenate([v_ref_k, [z_ref_k], [0,0,0], u_trim_k])
            self.solver.cost_set(k, 'yref', yref_k)

            # 상태 초기화 (첫 풀이 시)
            if len(self.solve_times) == 0:
                x_trim_k, _ = self._get_trim(V_k)
                x_ws = x_trim_k.copy()
                x_ws[2] = z_ref_k
                self.solver.set(k, 'x', x_ws)
                self.solver.set(k, 'u', u_trim_k)

        # 종단
        v_ref_N, z_ref_N = reference_trajectory(t + self.N * self.dt_nmpc)
        yref_e = np.concatenate([v_ref_N, [z_ref_N], [0,0,0]])
        self.solver.cost_set(self.N, 'yref', yref_e)

        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        status = self.solver.solve()
        self.solve_times.append(timer.time() - t0_wall)
        self.statuses.append(status)

        u_opt = self.solver.get(0, 'u')
        if np.any(np.isnan(u_opt)):
            _, u_fb = self._get_trim(0)
            u_opt = u_fb

        # warm start shift
        for k in range(self.N - 1):
            self.solver.set(k, 'x', self.solver.get(k+1, 'x'))
            self.solver.set(k, 'u', self.solver.get(k+1, 'u'))

        return np.clip(u_opt, self.p['n_min'], self.p['n_max'])

    def get_stats(self):
        st = np.array(self.solve_times) if self.solve_times else np.array([0])
        ok = sum(1 for s in self.statuses if s == 0)
        return {'mean_ms': np.mean(st)*1000, 'max_ms': np.max(st)*1000,
                'n_ok': ok, 'n_total': len(self.statuses)}


# ══════════════════════════════════════════════════
# 성능 측정
# ══════════════════════════════════════════════════

def measure(ts, xs, us, params):
    n_max = params['n_max']
    results = {}

    # 시간별 기준값
    vx_ref = np.array([reference_trajectory(t)[0][0] for t in ts])
    z_ref  = np.array([reference_trajectory(t)[1] for t in ts])

    vx = xs[:, 3]
    z  = xs[:, 2]

    # 전체 RMSE
    results['rmse_vx'] = np.sqrt(np.mean((vx - vx_ref)**2))
    results['rmse_z']  = np.sqrt(np.mean((z - z_ref)**2))

    # 고도: 최대 편차
    results['z_max_err'] = np.max(np.abs(z - z_ref))

    # 속도 도달 시간 (v_x >= 65 m/s 처음 도달)
    idx_65 = np.where(vx >= 65.0)[0]
    results['t_reach_65'] = ts[idx_65[0]] if len(idx_65) > 0 else np.inf

    # 로터 포화: n >= 0.95*n_max 비율
    results['sat_pct'] = 100 * np.mean(us >= 0.95 * n_max)

    # 발산
    results['diverged'] = bool(np.any(np.isnan(xs)) or np.any(np.abs(z) > 200))

    # 구간별 RMSE
    # 호버 (0~1s), 가속 (1~16s), 순항 (16~20s)
    for label, t0, t1 in [('호버', 0, 1), ('가속', 1, 16), ('순항', 16, 20)]:
        mask = (ts >= t0) & (ts < t1)
        if np.sum(mask) > 0:
            results[f'rmse_vx_{label}'] = np.sqrt(np.mean((vx[mask] - vx_ref[mask])**2))
            results[f'rmse_z_{label}']  = np.sqrt(np.mean((z[mask] - z_ref[mask])**2))

    return results


# ══════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════

def run():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt
    T_sim = 20.0

    print("\n" + "=" * 70)
    print("  호버 → 70 m/s 순항 가속 천이")
    print("=" * 70)

    # 초기 상태: 호버, z=50m
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 50.0

    # 트림 캐시 (LQR/NMPC에서 사용)
    trims = {}
    for V in [0, 10, 20, 30, 40, 50, 60, 70]:
        tr = find_trim(P, float(V))
        if tr['residual'] < 1e-3:
            trims[V] = (tr['state'].copy(), tr['control'].copy())

    # 30 m/s LQR 게인
    K_30 = LQRController(P, trims[30][0], trims[30][1]).K.copy()

    # ── PID ──
    print("\n  [PID] 실행 중...", end=" ", flush=True)
    pid = TrackingPID(P, dt)
    pid.reset()
    t0 = timer.time()
    ts, xs_pid, us_pid = plant.simulate(x0.copy(), pid, T_sim)
    t_pid = timer.time() - t0
    m_pid = measure(ts, xs_pid, us_pid, P)
    print(f"{t_pid:.1f}s")

    # ── LQR ──
    print("  [LQR] 실행 중...", end=" ", flush=True)
    lqr = TrackingLQR(P, K_30, trims)
    t0 = timer.time()
    ts, xs_lqr, us_lqr = plant.simulate(x0.copy(), lqr, T_sim)
    t_lqr = timer.time() - t0
    m_lqr = measure(ts, xs_lqr, us_lqr, P)
    print(f"{t_lqr:.1f}s")

    # ── NMPC ──
    print("  [NMPC] 빌드 + 실행 중...", end=" ", flush=True)
    try:
        for p in ['c_generated_code', 'acados_ocp.json']:
            fp = os.path.join(os.path.dirname(os.path.abspath(__file__)), p)
            if os.path.isdir(fp): shutil.rmtree(fp)
            elif os.path.isfile(fp): os.remove(fp)

        solver, N = build_ocp_solver(P, N=20, T_horizon=1.0, use_rti=False)
        nmpc = TrackingNMPC(solver, N, P, trims, dt_ctrl=0.02)

        t0 = timer.time()
        ts, xs_nmpc, us_nmpc = plant.simulate(x0.copy(), nmpc, T_sim)
        t_nmpc = timer.time() - t0
        m_nmpc = measure(ts, xs_nmpc, us_nmpc, P)
        stats = nmpc.get_stats()
        nmpc_ok = True
        print(f"{t_nmpc:.1f}s")
    except Exception as e:
        print(f"오류: {e}")
        m_nmpc = None
        nmpc_ok = False

    # ── 결과 ──
    print(f"\n{'='*70}")
    print("  종합 비교")
    print(f"{'='*70}")

    def _v(m, key):
        if m is None: return "  FAIL"
        v = m.get(key, float('nan'))
        if isinstance(v, bool): return " YES" if v else "  NO"
        if v == np.inf: return "   N/A"
        return f"{v:7.2f}"

    print(f"\n  {'지표':>18s}  {'PID':>8s}  {'LQR':>8s}  {'NMPC':>8s}")
    print(f"  {'─'*46}")

    for label, key in [
        ("전체 RMSE vx",    'rmse_vx'),
        ("전체 RMSE z",     'rmse_z'),
        ("최대 고도 오차 [m]", 'z_max_err'),
        ("65 m/s 도달 [s]", 't_reach_65'),
        ("로터 포화 [%]",   'sat_pct'),
        ("호버 RMSE z",     'rmse_z_호버'),
        ("가속 RMSE vx",    'rmse_vx_가속'),
        ("가속 RMSE z",     'rmse_z_가속'),
        ("순항 RMSE vx",    'rmse_vx_순항'),
        ("순항 RMSE z",     'rmse_z_순항'),
    ]:
        vp = _v(m_pid, key)
        vl = _v(m_lqr, key)
        vn = _v(m_nmpc, key) if nmpc_ok else "  FAIL"
        print(f"  {label:>18s}  {vp:>8s}  {vl:>8s}  {vn:>8s}")

    print(f"  {'발산':>18s}  {'YES' if m_pid['diverged'] else 'NO':>8s}"
          f"  {'YES' if m_lqr['diverged'] else 'NO':>8s}"
          f"  {'YES' if nmpc_ok and m_nmpc['diverged'] else 'NO':>8s}")

    if nmpc_ok and stats:
        print(f"\n  NMPC 풀이: {stats['mean_ms']:.1f}ms avg, "
              f"{stats['n_ok']}/{stats['n_total']} 수렴")

    # 핵심 분석
    print(f"\n  분석:")
    if nmpc_ok and not m_nmpc['diverged']:
        if m_pid['diverged'] or m_lqr['diverged']:
            print("  → PID/LQR 발산, NMPC만 천이 성공 = NMPC 필수")
        else:
            # 가속 구간이 핵심
            pz = m_pid.get('rmse_z_가속', 99)
            lz = m_lqr.get('rmse_z_가속', 99)
            nz = m_nmpc.get('rmse_z_가속', 99)
            print(f"  → 가속 구간 고도 RMSE: PID {pz:.2f}, LQR {lz:.2f}, NMPC {nz:.2f}")
            if nz < pz * 0.5:
                print(f"  → NMPC가 가속 중 고도 유지에서 {pz/nz:.1f}배 우수")

    print(f"\n{'='*70}")

    # 정리
    for p in ['c_generated_code', 'acados_ocp.json']:
        fp = os.path.join(os.path.dirname(os.path.abspath(__file__)), p)
        if os.path.isdir(fp): shutil.rmtree(fp)
        elif os.path.isfile(fp): os.remove(fp)


if __name__ == '__main__':
    run()
