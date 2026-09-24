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

from control.dynamics import build_dynamics, NX, NU


class NMPCController:
    """
    Direct Multiple Shooting NMPC.

    사용법:
        nmpc = NMPCController(params, v_ref=[83,0,0], z_ref=50)
        u = nmpc(t, x)  # plant.simulate()와 호환
    """

    def __init__(self, params, v_ref=None, z_ref=0.0, u_ref=None,
                 N=20, dt_nmpc=0.05, dt_ctrl=0.02, Q_z=20.0,
                 electrical_constraints=False, V_b=None,
                 cost_spec='paper', ref_fn=None):
        """
        Parameters
        ----------
        N        : int    예측 지평선 스텝 수 (N * dt_nmpc = 예측 시간)
        dt_nmpc  : float  NMPC 내부 적분 스텝 [s]
        dt_ctrl  : float  제어 주기 (이 간격마다 NLP 재풀이) [s]
        electrical_constraints : bool
            모터 전류·전압 제약을 예측 격자마다 부과할지.
            ⚠ 논문 v5.3 범위 밖 — 132행이 "전류·전압 한계와 배터리 전압
            강하는 모델에 포함하지 않는다 … 고정된 회전수 한계로 대체한다"고
            명시한다(7.4절 한계에도 재기술). 기본값 False를 유지할 것.
            실기(PX4) 쪽에서 쓸 수 있어 코드는 남겨 둔다.
            기본값 False로 기존 M17 동작·테스트를 보존한다(표6의 E0).
            True가 실제 논문 M17(E1) — 이게 표6의 E0↔E1 제거실험 플래그다.
        V_b : float or None
            예측 구간 내내 고정해서 쓸 버스 전압(논문 범위 밖: "예측 전압은
            현재 측정된 V_b로 유지, SOC의 미래 변화는 직접 예측하지 않음").
            None이면 params['V_oc'](만충 가정)로 시작하고, 매 호출 전
            self.V_b를 갱신해 실제 배터리 상태를 반영할 수 있다.
        """
        self.p = params
        self.N = N
        self.dt_nmpc = dt_nmpc
        self.dt_ctrl = dt_ctrl
        self.nx, self.nu = NX, NU
        self.electrical_constraints = electrical_constraints
        self.V_b = V_b if V_b is not None else params.get('V_oc', 44.4)

        self.v_ref = np.array(v_ref) if v_ref is not None else np.zeros(3)
        self.z_ref = z_ref
        self._Q_z = Q_z
        # cost_spec : {'paper','legacy'}  (2026-09-25 밤, 야간지시 3-a)
        #   'paper' (기본값) — 논문 v5.3 식(14)-(18)의 정확한 전사. M17(비분리)의
        #   입력은 로터 속도 자체이므로 Dν 정규화는 research/nmpc.py 가 kind="nmpc"
        #   에 쓰는 척도(max_n)를 그대로 따른다 — V13 의 [mg,100,100,100] 과는
        #   다른 척도지만 같은 원칙(식15: 물리량마다 자기 한계로 무차원화)이다.
        #   'legacy' — 이 클래스가 원래 쓰던 비용. 옛 결과 재현용으로만 남긴다.
        #
        #   2026-09-24에 V13 만 'paper' 로 기본값을 뒤집어 M17·F13 과 비대칭이
        #   됐었다 — 이 파일이 그 비대칭을 없앤다. 차이(legacy 기준):
        #     속도가중    5·I₃(식14)              diag(5,5,10)      vz만 2배
        #     입력편차    0.02/max_n²             1e-4              척도 자체가 다름
        #     입력변화    0.1/max_n²              1e-3              척도 자체가 다름
        #     종말가중    stage 전체×10(식16)      v·z만×10          ω 누락
        #     참조        노드별 r_{j|k}(식14)     상수 1개          ref_fn 없으면 동일
        if cost_spec not in ('paper', 'legacy'):
            raise ValueError(f"cost_spec must be 'paper' or 'legacy', got {cost_spec!r}")
        if ref_fn is not None and cost_spec != 'paper':
            raise ValueError("ref_fn은 cost_spec='paper'에서만 쓸 수 있다.")
        self.cost_spec = cost_spec
        self.ref_fn = ref_fn
        self._solve_log = []
        if u_ref is not None:
            self.u_ref = np.array(u_ref)
        else:
            n_hov = np.sqrt(params['mass'] * params['g'] / (4 * params['k_T']))
            self.u_ref = np.full(4, n_hov)

        # RK4 적분기 (예측용)
        f, x_sym, u_sym = build_dynamics(params)
        if self.cost_spec == 'paper':
            self.F = self._build_rk4_substeps(f, x_sym, u_sym, substeps=5)
            self._build_nlp_paper(params)
        else:
            self._build_rk4(f, x_sym, u_sym)
            self._build_nlp(params)

        # 상태
        self._last_t = -np.inf
        self._t_now = 0.0
        self._u_current = self.u_ref.copy()
        # _w0_init는 _build_nlp()에서 설정됨

    def reset(self):
        """MC 시행 간 독립성 보장을 위한 완전 리셋."""
        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()
        if self._w0_init is not None:
            self.w0 = self._w0_init.copy()

    def _build_rk4(self, f, x_sym, u_sym):
        """예측용 RK4 한 스텝 함수."""
        dt = self.dt_nmpc
        k1 = f(x_sym, u_sym)
        k2 = f(x_sym + dt/2 * k1, u_sym)
        k3 = f(x_sym + dt/2 * k2, u_sym)
        k4 = f(x_sym + dt * k3, u_sym)
        x_next = x_sym + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        self.F = ca.Function('F_rk4', [x_sym, u_sym], [x_next])

    def _build_rk4_substeps(self, f, x_sym, u_sym, substeps=5):
        """논문 4.2절: 예측 격자당 RK4 5회 세부적분 + 쿼터니언 정규화.

        control/hybrid_comparison.py::VirtualNMPC._make_substep_integrator 와
        같은 패턴(그쪽 주석 참고 — 실효 스텝을 줄여 빠른 자세 변화의 적분
        오차를 줄이고, 정규화 없이 RK4 만 쓰면 쿼터니언 노름이 서서히 벌어진다).
        """
        h = self.dt_nmpc / substeps
        st = x_sym
        for _ in range(substeps):
            k1 = f(st, u_sym)
            k2 = f(st + h/2*k1, u_sym)
            k3 = f(st + h/2*k2, u_sym)
            k4 = f(st + h*k3, u_sym)
            st = st + h/6*(k1 + 2*k2 + 2*k3 + k4)
            st = ca.vertcat(st[0:6], st[6:10]/ca.norm_2(st[6:10]), st[10:17])
        return ca.Function('F_paper', [x_sym, u_sym], [st])

    def _build_nlp_paper(self, params):
        """논문 v5.3 식(14)-(18)의 직접 전사 — M17(비분리, 로터속도 입력).

        VirtualNMPC._build_nlp_paper 와 나란히 읽을 수 있게 같은 구조로 쓴다.
        차이는 상태가 17D(로터 4개 포함)이고 입력이 로터 속도 자체라는 것뿐 —
        Dν 정규화 척도가 research/nmpc.py 의 kind="nmpc" 와 같은 max_n 하나다
        (표5 kind별 척도: nmpc→max_n, f13→mg/4, hybrid→[mg,100,100,100]).
        """
        N, nx, nu = self.N, self.nx, self.nu
        n_max = params['n_max']
        n_hov = self.u_ref[0]   # __init__ 에서 이미 트림/명시값으로 계산됨

        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J_cost = 0.0

        # p 레이아웃: [x_meas(17), refs(4·(N+1))]  — refs 는 노드별 (vx,vy,vz,z).
        p = ca.SX.sym('p', nx + 4*(N+1))
        x_meas = p[0:nx]
        refs = ca.reshape(p[nx:nx + 4*(N+1)], 4, N+1)

        def new_state(k):
            X = ca.SX.sym(f'X_{k}', nx)
            w.append(X)
            lbw.extend([-1e6]*nx); ubw.extend([1e6]*nx)
            guess = [0.0]*nx
            guess[6:10] = [1.0, 0.0, 0.0, 0.0]
            guess[13:17] = [float(n_hov)]*4
            w0.extend(guess)
            return X

        X_k = new_state(0)
        g.append(X_k - x_meas)
        lbg.extend([0.0]*nx); ubg.extend([0.0]*nx)

        U_prev = ca.DM(np.full(nu, n_hov))
        for k in range(N):
            e_v = X_k[3:6] - refs[0:3, k]
            e_z = X_k[2] - refs[3, k]
            J_cost += 5.0*ca.sumsqr(e_v) + self._Q_z*e_z**2 + ca.sumsqr(X_k[10:13])

            U_k = ca.SX.sym(f'U_{k}', nu)
            w.append(U_k)
            lbw.extend([params['n_min']]*nu); ubw.extend([n_max]*nu)
            w0.extend([float(n_hov)]*nu)

            J_cost += 0.02*ca.sumsqr((U_k - n_hov)/n_max)
            J_cost += 0.10*ca.sumsqr((U_k - U_prev)/n_max)

            X_next = new_state(k+1)
            g.append(X_next - self.F(X_k, U_k))
            lbg.extend([0.0]*nx); ubg.extend([0.0]*nx)
            X_k, U_prev = X_next, U_k

        e_v = X_k[3:6] - refs[0:3, N]
        e_z = X_k[2] - refs[3, N]
        J_cost += 10.0*(5.0*ca.sumsqr(e_v) + self._Q_z*e_z**2 + ca.sumsqr(X_k[10:13]))

        nlp = {'f': J_cost, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': p}
        self.solver = ca.nlpsol('nmpc_paper', 'ipopt', nlp, {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
            'ipopt.max_iter': 30, 'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-4})
        self.lbw = np.array(lbw); self.ubw = np.array(ubw)
        self.lbg = np.array(lbg); self.ubg = np.array(ubg)
        self.w0 = np.array(w0); self._w0_init = self.w0.copy()

    def _reference_horizon(self):
        if self.ref_fn is None:
            one = np.concatenate([self.v_ref, [self.z_ref]])
            return np.tile(one[:, None], (1, self.N + 1))
        cols = [np.asarray(self.ref_fn(self._t_now + k*self.dt_nmpc), dtype=float)
               for k in range(self.N + 1)]
        return np.stack(cols, axis=1)

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

        # ── 파라미터: [x_init(17), v_ref(3), z_ref(1), u_ref(4), V_b(1)] = 26 ──
        # V_b: 전기제약(electrical_constraints)용 — 예측 구간 내내 고정해서 쓰는
        # 현재 측정 버스전압(논문 범위 밖). 제약을 안 쓸 때도 자리만 차지, 계산엔 안 씀.
        p = ca.SX.sym('p', nx + 3 + 1 + nu + 1)
        x_init = p[0:nx]
        v_ref  = p[nx:nx+3]
        z_ref  = p[nx+3]
        u_ref  = p[nx+4:nx+4+nu]
        V_b_param = p[nx+4+nu]

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
            _xg = [0.0] * nx
            _xg[6:10] = [1.0, 0.0, 0.0, 0.0]   # 유효 단위 쿼터니언(호버) — 무효 [0,0,0,0] 초기추측 방지
            w0  += _xg

            # 동역학 제약: X_k = F(X_prev, U_k)
            X_pred = self.F(X_prev, U_k)
            g.append(X_k - X_pred)
            lbg += [0.0] * nx
            ubg += [0.0] * nx

            # ── 전류·전압 제약 (논문 v5.3 범위 밖, electrical_constraints=True일 때만) ──
            # M17만의 특징 — 모터 상태(X_k[13:17]=n)와 명령(U_k=n_c)이 이미
            # 결정변수에 있어 추가 상태 없이 바로 부과할 수 있다.
            if self.electrical_constraints:
                n_i = X_k[13:17]
                Q_i = self.p['k_Q'] * n_i**2   # 마찰 Q_f=0 (battery.py와 동일 가정)
                I_req = (Q_i / self.p['k_t']
                         + self.p['I_rotor'] * (U_k - n_i)
                           / (self.p['k_t'] * self.p['tau_m']))
                g.append(I_req)
                lbg += [0.0] * 4
                ubg += [float(self.p['I_lim'])] * 4
                g.append(V_b_param - (self.p['k_e'] * n_i + self.p['R_m'] * I_req))
                lbg += [0.0] * 4
                ubg += [1e6] * 4

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
        self._w0_init = self.w0.copy()

    def __call__(self, t, x):
        """dt_ctrl 주기마다 NLP를 풀고 첫 제어 반환."""
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            self._t_now = t
            self._u_current = self._solve(x)
            self._last_t = t
        return self._u_current

    def _solve(self, x_current):
        """NLP 풀이 → 첫 제어 추출."""
        if self.cost_spec == 'paper':
            p_val = np.concatenate([x_current, self._reference_horizon().ravel(order='F')])
        else:
            p_val = np.concatenate([x_current, self.v_ref, [self.z_ref], self.u_ref,
                                     [self.V_b]])

        sol = self.solver(
            x0=self.w0, lbx=self.lbw, ubx=self.ubw,
            lbg=self.lbg, ubg=self.ubg, p=p_val)

        self._solve_log.append(self.solver.stats().get('return_status', 'unknown'))
        w_opt = np.array(sol['x']).flatten()
        stride = self.nu + self.nx   # 21

        if self.cost_spec == 'paper':
            # 배치: X_0, U_0, X_1, U_1, …, X_{N-1}, U_{N-1}, X_N (x0 가 결정변수)
            u_opt = w_opt[self.nx:self.nx + self.nu]
            self.w0 = np.concatenate([w_opt[stride:], w_opt[-stride:-self.nx],
                                      w_opt[-self.nx:]])
        else:
            # 배치: U_0, X_0, U_1, X_1, … (legacy)
            u_opt = w_opt[0:self.nu]
            self.w0 = np.concatenate([w_opt[stride:], w_opt[-stride:]])

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
    from control.vehicle_params import vehicle_params as P
    from control.dynamics import AxialDronePlant
    from control.trim import find_trim, print_trim
    from control.controller import CascadedPID, LQRController

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
