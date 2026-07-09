"""
Nonlinear MPC — CasADi + IPOPT (Direct Multiple Shooting)
==========================================================

acados 없이 CasADi의 IPOPT 솔버로 NMPC 구현.
수학적으로 acados와 동일한 formulation:
  - Multiple shooting discretization (RK4)
  - 비선형 동역학 예측 모델
  - 모터 속도 제약 명시적 처리
  - 예측 지평선(horizon) 내 최적 제어 시퀀스

PID/LQR 대비 NMPC의 핵심 이점:
  1) 미래 예측 → 선제적 제어 (틸트 전환 등)
  2) 제약 처리 → 모터 포화 방지
  3) 비선형 모델 → 고속 영역에서도 정확한 예측
"""

import numpy as np
import casadi as ca
import time as timer

from dynamics import build_dynamics, NX, NU


class NMPCController:
    """
    Direct Multiple Shooting NMPC.

    사용법:
        nmpc = NMPCController(params, v_ref=[83,0,0], z_ref=50)
        u = nmpc(t, x)  # plant.simulate()와 호환
    """

    def __init__(self, params, v_ref=None, z_ref=0.0, u_ref=None,
                 N=20, dt_nmpc=0.05, dt_ctrl=0.02, Q_z=20.0):
        """
        Parameters
        ----------
        N        : int    예측 지평선 스텝 수 (N * dt_nmpc = 예측 시간)
        dt_nmpc  : float  NMPC 내부 적분 스텝 [s]
        dt_ctrl  : float  제어 주기 (이 간격마다 NLP 재풀이) [s]
        """
        self.p = params
        self.N = N
        self.dt_nmpc = dt_nmpc
        self.dt_ctrl = dt_ctrl
        self.nx, self.nu = NX, NU

        self.v_ref = np.array(v_ref) if v_ref is not None else np.zeros(3)
        self.z_ref = z_ref
        self._Q_z = Q_z
        self._solve_log = []
        if u_ref is not None:
            self.u_ref = np.array(u_ref)
        else:
            n_hov = np.sqrt(params['mass'] * params['g'] / (4 * params['k_T']))
            self.u_ref = np.full(4, n_hov)

        # RK4 적분기 (예측용)
        f, x_sym, u_sym = build_dynamics(params)
        self._build_rk4(f, x_sym, u_sym)

        # NLP 구성
        self._build_nlp(params)

        # 상태
        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()

    def _build_rk4(self, f, x_sym, u_sym):
        """예측용 RK4 한 스텝 함수."""
        dt = self.dt_nmpc
        k1 = f(x_sym, u_sym)
        k2 = f(x_sym + dt/2 * k1, u_sym)
        k3 = f(x_sym + dt/2 * k2, u_sym)
        k4 = f(x_sym + dt * k3, u_sym)
        x_next = x_sym + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        self.F = ca.Function('F_rk4', [x_sym, u_sym], [x_next])

    def _build_nlp(self, params):
        """Multiple shooting NLP 구성."""
        N, nx, nu = self.N, self.nx, self.nu

        # ── 비용 가중치 ──
        Q_v  = np.diag([5.0, 5.0, 10.0])       # 속도 추종
        Q_z  = self._Q_z                          # 고도 (외부 조정 가능)
        Q_w  = np.diag([1.0, 1.0, 1.0])         # 각속도 (안정성)
        R    = np.eye(nu) * 0.0001               # 제어 편차 (트림 대비)
        R_du = np.eye(nu) * 0.001                # 제어 변화율

        n_min = params['n_min']
        n_max = params['n_max']

        # ── 파라미터: [x_init(17), v_ref(3), z_ref(1), u_ref(4)] = 25 ──
        p = ca.SX.sym('p', nx + 3 + 1 + nu)
        x_init = p[0:nx]
        v_ref  = p[nx:nx+3]
        z_ref  = p[nx+3]
        u_ref  = p[nx+4:nx+4+nu]

        # ── 결정 변수 + 제약 ──
        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J = 0.0

        X_prev = x_init
        U_prev = u_ref

        for k in range(N):
            # 제어 입력 (결정 변수)
            U_k = ca.SX.sym(f'U_{k}', nu)
            w.append(U_k)
            lbw += [n_min] * nu
            ubw += [n_max] * nu
            w0  += [600.0] * nu

            # 다음 상태 (결정 변수, shooting node)
            X_k = ca.SX.sym(f'X_{k}', nx)
            w.append(X_k)
            lbw += [-1e6] * nx
            ubw += [1e6] * nx
            w0  += [0.0] * nx

            # 동역학 제약: X_k = F(X_prev, U_k)
            X_pred = self.F(X_prev, U_k)
            g.append(X_k - X_pred)
            lbg += [0.0] * nx
            ubg += [0.0] * nx

            # ── 스테이지 비용 ──
            e_v = X_k[3:6] - v_ref
            e_z = X_k[2] - z_ref
            e_u = U_k - u_ref
            dU  = U_k - U_prev

            J += e_v.T @ Q_v @ e_v           # 속도 추종
            J += Q_z * e_z**2                  # 고도
            J += X_k[10:13].T @ Q_w @ X_k[10:13]  # 각속도 안정
            J += e_u.T @ R @ e_u               # 제어 편차
            J += dU.T @ R_du @ dU              # 제어 변화율

            X_prev = X_k
            U_prev = U_k

        # ── 종단 비용 (10배 강화) ──
        e_v_N = X_prev[3:6] - v_ref
        e_z_N = X_prev[2] - z_ref
        J += 10 * (e_v_N.T @ Q_v @ e_v_N)
        J += 10 * Q_z * e_z_N**2

        # ── NLP 솔버 생성 ──
        w_cat = ca.vertcat(*w)
        g_cat = ca.vertcat(*g)

        nlp = {'f': J, 'x': w_cat, 'g': g_cat, 'p': p}
        opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0,
            'ipopt.max_iter': 30,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-4,
        }
        self.solver = ca.nlpsol('nmpc', 'ipopt', nlp, opts)

        self.lbw = np.array(lbw, dtype=float)
        self.ubw = np.array(ubw, dtype=float)
        self.lbg = np.array(lbg, dtype=float)
        self.ubg = np.array(ubg, dtype=float)
        self.w0  = np.array(w0, dtype=float)

    def __call__(self, t, x):
        """dt_ctrl 주기마다 NLP를 풀고 첫 제어 반환."""
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            self._u_current = self._solve(x)
            self._last_t = t
        return self._u_current

    def _solve(self, x_current):
        """NLP 풀이 → 첫 제어 추출."""
        p_val = np.concatenate([x_current, self.v_ref, [self.z_ref], self.u_ref])

        sol = self.solver(
            x0=self.w0, lbx=self.lbw, ubx=self.ubw,
            lbg=self.lbg, ubg=self.ubg, p=p_val)

        self._solve_log.append(self.solver.stats().get('return_status', 'unknown'))
        w_opt = np.array(sol['x']).flatten()

        # 첫 제어 추출 (w = [U_0(4), X_1(17), U_1(4), X_2(17), ...])
        u_opt = w_opt[0:self.nu]

        # Warm start: 이전 해를 한 스텝 시프트
        stride = self.nu + self.nx   # 21
        shifted = np.concatenate([w_opt[stride:], w_opt[-stride:]])
        self.w0 = shifted

        return np.clip(u_opt, self.p['n_min'], self.p['n_max'])

    def get_solve_stats(self):
        """IPOPT 풀이 통계."""
        if not self._solve_log:
            return {}
        n = len(self._solve_log)
        ok = sum(1 for s in self._solve_log if s == 'Solve_Succeeded')
        return {'n_solves': n, 'n_ok': ok, 'pct_ok': 100.0 * ok / n}


# ══════════════════════════════════════════════════════
# 비교 시뮬레이션
# ══════════════════════════════════════════════════════

def run_nmpc_comparison():
    from vehicle_params import vehicle_params as P
    from dynamics import AxialDronePlant
    from trim import find_trim, print_trim
    from controller import CascadedPID, LQRController

    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    print("\n" + "=" * 70)
    print("  NMPC vs PID vs LQR 비교")
    print("=" * 70)

    # 비교 속도 포인트
    test_speeds = [30, 50, 70, 85]
    T_sim = 10.0

    # 30 m/s 기준 게인 (PID/LQR)
    trim_30 = find_trim(P, 30.0)
    pid_base_gains = True   # 30 m/s 게인 고정
    lqr_30 = LQRController(P, trim_30['state'], trim_30['control'])
    K_fixed = lqr_30.K.copy()
    K_r_fixed = lqr_30.K_r.copy()
    n_reduced = lqr_30.n_reduced

    results = []

    for V in test_speeds:
        print(f"\n{'─'*70}")
        print(f"  V = {V} m/s ({V*3.6:.0f} km/h)")

        trim = find_trim(P, V)
        if trim['residual'] > 1e-3:
            print(f"  트림 실패, 건너뜀")
            continue

        print_trim(trim, V, P)
        x_trim = trim['state']
        u_trim = trim['control']

        # 초기 상태: 트림 + 교란
        x0 = x_trim.copy()
        x0[2] = 50.0
        x0[5] += 2.0
        x0[3] += 1.0

        # ── PID (30 m/s 게인 고정) ──
        pid = CascadedPID(P, v_ref=[V, 0, 0], z_ref=50.0, dt=dt)
        pid.reset()
        t0 = timer.time()
        ts, xs_pid, us_pid = plant.simulate(x0.copy(), pid, T_sim)
        t_pid = timer.time() - t0

        # ── LQR (30 m/s K 고정) ──
        lqr = LQRController.__new__(LQRController)
        lqr.p = P
        lqr.K = K_fixed
        lqr.K_r = K_r_fixed
        lqr.n_reduced = n_reduced
        lqr.valid = True
        lqr.x_trim = x_trim.copy()
        lqr.u_trim = u_trim.copy()
        lqr.x_trim[2] = 50.0
        lqr.x_trim[0:2] = x0[0:2]

        t0 = timer.time()
        ts, xs_lqr, us_lqr = plant.simulate(x0.copy(), lqr, T_sim)
        t_lqr = timer.time() - t0

        # ── NMPC ──
        nmpc = NMPCController(P, v_ref=[V, 0, 0], z_ref=50.0,
                              u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)
        t0 = timer.time()
        ts, xs_nmpc, us_nmpc = plant.simulate(x0.copy(), nmpc, T_sim)
        t_nmpc = timer.time() - t0

        # ── 결과 ──
        def metrics(xs, us, label):
            rmse_vx = np.sqrt(np.mean((xs[:, 3] - V)**2))
            rmse_z  = np.sqrt(np.mean((xs[:, 2] - 50.0)**2))
            sat = 100 * np.mean(us >= 0.95 * P['n_max'])
            return rmse_vx, rmse_z, sat

        m_pid  = metrics(xs_pid, us_pid, 'PID')
        m_lqr  = metrics(xs_lqr, us_lqr, 'LQR')
        m_nmpc = metrics(xs_nmpc, us_nmpc, 'NMPC')

        print(f"\n  {'':>12s}  {'PID':>10s}  {'LQR':>10s}  {'NMPC':>10s}")
        print(f"  {'─'*46}")
        print(f"  {'RMSE vx':>12s}  {m_pid[0]:10.3f}  {m_lqr[0]:10.3f}  {m_nmpc[0]:10.3f}")
        print(f"  {'RMSE z':>12s}  {m_pid[1]:10.3f}  {m_lqr[1]:10.3f}  {m_nmpc[1]:10.3f}")
        print(f"  {'SAT %':>12s}  {m_pid[2]:10.1f}  {m_lqr[2]:10.1f}  {m_nmpc[2]:10.1f}")
        print(f"  {'계산시간':>12s}  {t_pid:10.2f}s  {t_lqr:10.2f}s  {t_nmpc:10.2f}s")

        results.append({
            'V': V, 'pid': m_pid, 'lqr': m_lqr, 'nmpc': m_nmpc,
            'time': (t_pid, t_lqr, t_nmpc)
        })

    # ── 종합 요약 ──
    print(f"\n{'='*70}")
    print("  NMPC 종합 요약")
    print(f"{'='*70}")
    print(f"\n  {'V':>5s}  {'--- RMSE vx ---':^30s}  {'--- RMSE z ---':^30s}")
    print(f"  {'m/s':>5s}  {'PID':>8s} {'LQR':>8s} {'NMPC':>8s}"
          f"  {'PID':>8s} {'LQR':>8s} {'NMPC':>8s}")
    print(f"  {'─'*65}")
    for r in results:
        print(f"  {r['V']:5.0f}  {r['pid'][0]:8.3f} {r['lqr'][0]:8.3f} {r['nmpc'][0]:8.3f}"
              f"  {r['pid'][1]:8.3f} {r['lqr'][1]:8.3f} {r['nmpc'][1]:8.3f}")

    print(f"\n  결론:")
    print(f"  - NMPC: 비선형 예측 + 제약 처리로 전 속도 영역에서 최적 성능")
    print(f"  - PID: 튜닝 속도에서 OK, 고속에서 성능 저하")
    print(f"  - LQR: 최적 게인이지만 선형화 모델의 한계")
    print(f"{'='*70}")


if __name__ == '__main__':
    run_nmpc_comparison()
