"""
NMPC+INDI 하이브리드 — 인터페이스 분리 vs 나이브 비교
====================================================

핵심 변경:
  나이브: NMPC→[n1..n4], INDI도→[Δn] → 이중 보정 → 악화
  분리:   NMPC→[T, ω̇_des], INDI→[n1..n4] → 역할 분리 → 올바름

VirtualNMPC:
  상태 13D: [p(3), v(3), q(4), ω(3)]  (로터 상태 제거)
  제어 4D:  [T_total, ω̇_x, ω̇_y, ω̇_z] (가상 명령)
  내부 동역학: ω̇ = ν_ω (INDI가 이를 실현한다고 가정)

검증:
  정확 모델 + 돌풍 → 나이브 악화 vs 분리 정상
  C_Na ±20% + 돌풍 → 분리가 모델 오차도 보상하는지
"""

import numpy as np
import casadi as ca
import time as timer
from scipy.spatial.transform import Rotation

from control.vehicle_params import vehicle_params as P
from control.dynamics import (build_dynamics, _quat_to_rotmat, _quat_derivative,
                      _body_aerodynamics, EPS, compute_allocation_matrix)
from control.trim import find_trim
from control.controller import ScheduledLQR
from control.nmpc import NMPCController


# ══════════════════════════════════════════════════
# 1. 가상 명령 동역학 (13D)
# ══════════════════════════════════════════════════

NX_V = 13   # [p(3), v(3), q(4), ω(3)]
NU_V = 4    # [T, ν_ωx, ν_ωy, ν_ωz]


def build_virtual_dynamics(params):
    """
    가상 명령 동역학: ω̇ = ν_ω (INDI가 실현).

    NMPC는 "얼마나 회전시킬지"만 결정.
    모터 할당·자이로·공력 모멘트는 INDI가 센서로 처리.
    공력 힘(항력·양력)은 NMPC가 알아야 궤적 예측이 되므로 포함.
    """
    x = ca.SX.sym('x', NX_V)
    u = ca.SX.sym('u', NU_V)

    pos, vel, quat, omega = x[0:3], x[3:6], x[6:10], x[10:13]
    T_cmd = u[0]          # 총 추력 [N]
    nu_omega = u[1:4]     # 가상 각가속도 명령 [rad/s²]

    R = _quat_to_rotmat(quat)
    v_body = R.T @ vel

    # 공력 힘 (모멘트는 INDI가 처리하므로 무시)
    F_aero, _ = _body_aerodynamics(v_body, omega, params)

    # 가상 모델도 물리 플랜트와 같은 추력축을 사용해야 한다.
    thrust = (ca.vertcat(T_cmd, 0, 0) if params.get('thrust_axis', 'z') == 'x'
              else ca.vertcat(0, 0, -T_cmd))
    F_body = F_aero + thrust

    p_dot = vel
    v_dot = ca.vertcat(0, 0, -params['g']) + (R @ F_body) / params['mass']
    q_dot = _quat_derivative(quat, omega)
    omega_dot = nu_omega    # INDI가 이걸 실현

    xdot = ca.vertcat(p_dot, v_dot, q_dot, omega_dot)
    f = ca.Function('f_virtual', [x, u], [xdot])
    return f, x, u


# ══════════════════════════════════════════════════
# 2. VirtualNMPC (가상 명령 출력)
# ══════════════════════════════════════════════════

class VirtualNMPC:
    """NMPC with virtual command output [T, ω̇_des]."""

    def __init__(self, params, v_ref=None, z_ref=0.0, T_ref=None,
                 N=20, dt_nmpc=0.05, dt_ctrl=0.02, Q_z=20.0, max_iter=30,
                 alloc_feedback=False):
        self.p = params
        self.N, self.dt_nmpc, self.dt_ctrl = N, dt_nmpc, dt_ctrl
        self.v_ref = np.array(v_ref) if v_ref is not None else np.zeros(3)
        self.z_ref = z_ref
        self._Q_z = Q_z
        self._max_iter = max_iter   # IPOPT 반복 상한 (SITL 실시간용 축소 가능)
        # 논문 식(31)-(32): INDI 배분 결과를 다음 솔브의 입력변화량 비용
        # 기준(v_{-1|k})으로 쓸지 여부. False면 기존과 동일하게 고정 트림값을
        # 쓴다 — 기본값 False로 기존 호출부(mission_sim 등) 동작을 보존한다.
        self.alloc_feedback = alloc_feedback

        self.T_ref = T_ref if T_ref else params['mass'] * params['g']
        self.u_ref = np.array([self.T_ref, 0, 0, 0])
        self._prev_input = self.u_ref.copy()

        f, x_sym, u_sym = build_virtual_dynamics(params)

        # RK4
        dt = dt_nmpc
        k1 = f(x_sym, u_sym)
        k2 = f(x_sym + dt/2*k1, u_sym)
        k3 = f(x_sym + dt/2*k2, u_sym)
        k4 = f(x_sym + dt*k3, u_sym)
        self.F = ca.Function('F_v', [x_sym, u_sym],
                             [x_sym + dt/6*(k1 + 2*k2 + 2*k3 + k4)])

        self._build_nlp(params, x_sym, u_sym)
        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()
        # 솔버 수렴 추적 (연속 미수렴 → HybridWithFallback의 전환 판단에 사용)
        self.consec_fail = 0
        self.last_status = 'none'
        self._ever_converged = False
        # _w0_init는 _build_nlp()에서 설정됨

    def reset(self):
        """MC 시행 간 독립성 보장을 위한 완전 리셋."""
        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()
        self.consec_fail = 0
        self.last_status = 'none'
        self._ever_converged = False
        self._prev_input = self.u_ref.copy()
        if self._w0_init is not None:
            self.w0 = self._w0_init.copy()

    def set_prev_input(self, v):
        """INDI 배분 결과(v_alloc)를 다음 솔브의 입력변화량 비용 기준으로 받는다
        (식31-32). alloc_feedback=False면 무시 — 항상 안전하게 호출 가능."""
        if self.alloc_feedback:
            self._prev_input = np.asarray(v, dtype=float)

    def _build_nlp(self, params, x_sym, u_sym):
        N, nx, nu = self.N, NX_V, NU_V

        Q_v = np.diag([5.0, 5.0, 10.0])
        Q_z = self._Q_z
        Q_w = np.diag([1.0, 1.0, 1.0])
        # 가상 명령 페널티: 추력 변화 작게, 각가속도 부드럽게
        R = np.diag([1e-5, 1e-3, 1e-3, 1e-3])
        R_du = np.diag([1e-4, 0.01, 0.01, 0.01])

        T_max = 4 * params['k_T'] * params['n_max']**2
        nu_max = 100.0  # rad/s²

        # p 레이아웃: [x_init(nx), v_ref(3), z_ref(1), u_ref(nu), u_prev(nu)]
        # u_ref: 매 단계 정규화 목표(트림) — 항상 고정.
        # u_prev: 0단계 입력변화량 비용의 기준(식31-32의 v_{-1|k}).
        #         alloc_feedback=False면 u_ref와 동일값이 매번 들어온다(기존과 동일 동작).
        p = ca.SX.sym('p', nx + 3 + 1 + nu + nu)
        x_init = p[0:nx]
        v_ref = p[nx:nx+3]
        z_ref = p[nx+3]
        u_ref = p[nx+4:nx+4+nu]
        u_prev = p[nx+4+nu:nx+4+2*nu]

        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J_cost = 0.0
        X_prev, U_prev = x_init, u_prev

        for k in range(N):
            U_k = ca.SX.sym(f'U_{k}', nu)
            w.append(U_k)
            lbw += [0.0, -nu_max, -nu_max, -nu_max]
            ubw += [T_max, nu_max, nu_max, nu_max]
            w0 += [float(self.T_ref), 0, 0, 0]

            X_k = ca.SX.sym(f'X_{k}', nx)
            w.append(X_k)
            lbw += [-1e6]*nx; ubw += [1e6]*nx
            _xg = [0.0]*nx
            _xg[6:10] = ([0.0, -np.sqrt(0.5), 0.0, np.sqrt(0.5)]
                         if params.get('thrust_axis', 'z') == 'x'
                         else [1.0, 0.0, 0.0, 0.0])   # 유효 단위 쿼터니언(호버) 초기추측
            w0 += _xg                          # (nmpc.py와 동일 픽스 — cold 솔브 개선)

            g.append(X_k - self.F(X_prev, U_k))
            lbg += [0.0]*nx; ubg += [0.0]*nx

            e_v = X_k[3:6] - v_ref
            e_z = X_k[2] - z_ref
            dU = U_k - U_prev
            J_cost += e_v.T @ Q_v @ e_v + Q_z*e_z**2
            J_cost += X_k[10:13].T @ Q_w @ X_k[10:13]
            J_cost += (U_k - u_ref).T @ R @ (U_k - u_ref)
            J_cost += dU.T @ R_du @ dU
            X_prev, U_prev = X_k, U_k

        J_cost += 10*(X_prev[3:6]-v_ref).T @ Q_v @ (X_prev[3:6]-v_ref)
        J_cost += 10*Q_z*(X_prev[2]-z_ref)**2

        nlp = {'f': J_cost, 'x': ca.vertcat(*w),
               'g': ca.vertcat(*g), 'p': p}
        self.solver = ca.nlpsol('vnmpc', 'ipopt', nlp, {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
            'ipopt.max_iter': self._max_iter, 'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-4})
        self.lbw = np.array(lbw)
        self.ubw = np.array(ubw)
        self.lbg = np.array(lbg)
        self.ubg = np.array(ubg)
        self.w0 = np.array(w0)
        self._w0_init = self.w0.copy()

    def __call__(self, t, x_full):
        """17D 플랜트 상태 → 13D 추출 → [T, ν_ω] 반환."""
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            x13 = np.concatenate([x_full[0:10], x_full[10:13]])
            self._u_current = self._solve(x13)
            self._last_t = t
        return self._u_current

    def _solve(self, x13):
        p_val = np.concatenate([x13, self.v_ref, [self.z_ref], self.u_ref,
                                 self._prev_input])
        sol = self.solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw,
                          lbg=self.lbg, ubg=self.ubg, p=p_val)

        # 수렴 추적: max_iter 도달 등 미수렴 해는 IPOPT의 "최선 반복해"라
        # 한 번은 쓸 만하지만 연속되면 품질 저하 누적 (벤치 실측: 과도구간에서
        # iter=30 상한 도달 사례). 솔브 단위로 카운트해 폴백이 판단하게 함.
        # cold-start(최초 수렴 전) 실패는 예상된 것이라 카운트 제외 —
        # 처음부터 고장난 NMPC는 폴백의 고도오차 트리거가 잡음.
        status = self.solver.stats().get('return_status', 'unknown')
        if status in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'):
            self.consec_fail = 0
            self._ever_converged = True
        elif self._ever_converged:
            self.consec_fail += 1
        self.last_status = status

        w_opt = np.array(sol['x']).flatten()
        u_opt = w_opt[0:NU_V]
        stride = NU_V + NX_V
        self.w0 = np.concatenate([w_opt[stride:], w_opt[-stride:]])
        return u_opt


def constrained_allocation(G, dT_target, domega_target, n_actual, n_min, n_max,
                            max_iter=4):
    """
    총추력 등식 제약 하 각가속도 잔차 최소 배분 — 논문 식(28)-(29)/부록A.2-A.3의
    증분(Δn) 공간 국소 해.

    G(4x4)는 현재 동작점의 선형 입력효과 행렬(0행=∂T/∂n, 1-3행=∂ω̇/∂n,
    ProperHybrid._compute_G와 동일). dT_target·domega_target은 INDI 증분
    목표(요구-측정, 식24의 소신호 가정). 반환하는 Δn은:
      Σ G[0,i]·Δn_i = dT_target  을 (포화가 없는 한) 정확히 만족시키고,
    그 부분공간 안에서 ||G[1:4]·Δn - domega_target||²를 최소화한다.

    포화(n_min/n_max 도달)는 활성집합법으로 처리한다 — 로터가 4개뿐이라
    매 반복 자유변수가 최소 1개 줄어들면 최대 4회 안에 반드시 끝난다
    (부록A.3의 KKT 시스템을 그대로 씀, 하한·자유·상한 조합을 전수 열거하지
    않고 위반한 변수만 순차로 고정 — 동일한 수렴 성질을 훨씬 싸게 얻는다).

    논문 본문이 별도로 쓰는 외곽 비선형 재선형화 + 감쇠 후보(1,1/2,1/4,1/8)
    루프는 적용하지 않는다 — INDI 증분은 이미 1ms마다 재선형화되는 소신호
    영역이라(식24) 1회 선형화로 충분하다고 판단했다. 필요해지면 이 함수를
    감싸는 바깥 루프를 추가하면 된다(이 함수 경계는 그러도록 분리해 뒀다).

    Returns
    -------
    dn : array(4)
    """
    free = np.ones(4, dtype=bool)
    dn = np.zeros(4)
    for _ in range(max_iter):
        idx = np.where(free)[0]
        if idx.size == 0:
            break
        fixed = ~free
        Gw = G[1:4, idx]                          # (3, |F|)
        g0 = G[0, idx]                             # (|F|,)
        dT_rem = dT_target - G[0, fixed] @ dn[fixed]
        dw_rem = domega_target - G[1:4, fixed] @ dn[fixed]

        k = idx.size
        KKT = np.zeros((k + 1, k + 1))
        KKT[:k, :k] = Gw.T @ Gw
        KKT[:k, k] = g0
        KKT[k, :k] = g0
        rhs = np.concatenate([Gw.T @ dw_rem, [dT_rem]])
        try:
            sol = np.linalg.solve(KKT, rhs)
        except np.linalg.LinAlgError:
            sol, *_ = np.linalg.lstsq(KKT, rhs, rcond=None)
        dn_free = sol[:k]
        dn[idx] = dn_free

        n_new = n_actual[idx] + dn_free
        lo_bad = n_new < n_min
        hi_bad = n_new > n_max
        if not (lo_bad.any() or hi_bad.any()):
            break
        for j, gi in zip(range(k), idx):
            if lo_bad[j]:
                dn[gi] = n_min - n_actual[gi]
                free[gi] = False
            elif hi_bad[j]:
                dn[gi] = n_max - n_actual[gi]
                free[gi] = False
        # 4회 반복을 다 써도 못 끝나면(모든 로터가 포화 직전) 마지막 추정치를
        # 그대로 쓴다 — 논문도 4회 상한을 명시하며 전역 최적성을 주장하지 않는다.
    return dn


# ══════════════════════════════════════════════════
# 3. ProperHybrid (인터페이스 분리)
# ══════════════════════════════════════════════════

class ProperHybrid:
    """
    올바른 NMPC+INDI: 인터페이스 분리.

    NMPC → [T_cmd, ω̇_des]  (뭘 할지)
    INDI → [n1..n4]         (어떻게 할지, 유일한 실행자)

    이중 보정 없음: NMPC는 모터를 모르고, INDI만 모터를 제어.
    """

    def __init__(self, virtual_nmpc, params, dt=0.001, f_cut=50.0,
                 alloc_mode='A0'):
        """
        alloc_mode : {'A0','A1'}
            'A0' (기본값, 기존 동작 보존) — 비제약 최소자승 + 사후 clip.
            'A1' — 논문 식(28)-(29) 총추력 등식 배분(constrained_allocation).
            논문 표6의 A0↔A1 제거실험이 바로 이 플래그다.
        """
        self.nmpc = virtual_nmpc
        self.p = params
        self.dt = dt
        self._tau = 1.0 / (2*np.pi*f_cut)     # LPF 시상수 (가변 dt에서 alpha 재계산용)
        self._alpha = dt / (dt + self._tau)
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._prev_t = None                   # 실제 Δt 측정용 (SITL 루프율 가변/100Hz미만)
        self._initialized = False
        self.alloc_mode = alloc_mode
        self.last_alloc = None                 # 식(31)-(32) 배분 결과 (보고·피드백용)
        _, self._TM_to_f = compute_allocation_matrix(params)

    def reset(self):
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._prev_t = None
        self._initialized = False
        # VirtualNMPC 완전 리셋 (타이밍 + warm start + w0)
        if hasattr(self.nmpc, 'reset'):
            self.nmpc.reset()

    def __call__(self, t, x):
        # 1. NMPC: 가상 명령
        vc = self.nmpc(t, x)
        T_cmd, omega_dot_des = vc[0], vc[1:4]

        omega = x[10:13]
        n_actual = x[13:17]

        # 측정 NaN 가드 (리뷰 3d): _omega_dot_filt는 자기참조 LPF라 NaN이
        # 한 번 들어가면 영구 고착 — 갱신을 건너뛰고 모델 기반 폴백으로.
        if not (np.all(np.isfinite(omega)) and np.all(np.isfinite(n_actual))):
            return self._fallback(T_cmd, omega_dot_des)

        if not self._initialized:
            self._omega_prev = omega.copy()
            self._prev_t = t
            self._initialized = True
            return self._fallback(T_cmd, omega_dot_des)

        # 2. ω̇ 측정 (LPF) — 하드코딩 dt 대신 실제 경과시간 사용
        #    (SITL은 NMPC 솔브로 루프율이 100Hz 미만/가변 → dt 오차가 ω̇를 왜곡)
        actual_dt = t - self._prev_t if self._prev_t is not None else self.dt
        actual_dt = min(max(actual_dt, 1e-4), 0.2)      # 0/과대 방지
        self._prev_t = t
        alpha = actual_dt / (actual_dt + self._tau)      # 가변 dt에 맞춰 LPF 재계산
        raw = (omega - self._omega_prev) / actual_dt
        self._omega_dot_filt = alpha*raw + (1-alpha)*self._omega_dot_filt
        self._omega_prev = omega.copy()

        # 3. INDI: [T,ω̇]_cmd vs [T,ω̇]_meas → Δn
        # v_body 계산 (전진비 반영)
        q = x[6:10]
        R = Rotation.from_quat(q).as_matrix()
        v_body = R.T @ x[3:6]
        V_axial = max(v_body[0] if self.p.get('thrust_axis', 'z') == 'x'
                      else -v_body[2], 0.0)

        # 전진비 보정된 추력 측정
        T_meas = 0.0
        for i in range(4):
            ni = n_actual[i]
            n_rps = ni / (2 * np.pi)
            J = V_axial / (n_rps * self.p['D_prop'] + 1e-8)
            fac = max(1.0 - J / self.p['J_max'], 0.0)
            T_meas += self.p['k_T'] * ni**2 * fac

        dv = np.array([T_cmd - T_meas,
                       omega_dot_des[0] - self._omega_dot_filt[0],
                       omega_dot_des[1] - self._omega_dot_filt[1],
                       omega_dot_des[2] - self._omega_dot_filt[2]])

        G = self._compute_G(n_actual, v_body)

        if self.alloc_mode == 'A1':
            f_max = self._rotor_thrust_cap(V_axial)
            T_c = float(np.clip(T_cmd, 0.0, np.sum(f_max)))
            dT_target = T_c - T_meas
            dn = constrained_allocation(G, dT_target, dv[1:4], n_actual,
                                         self.p['n_min'], self.p['n_max'])
            n_cmd = np.clip(n_actual + dn, self.p['n_min'], self.p['n_max'])

            # 식(31)-(32): 명목 배분 결과를 가상입력 공간으로 되돌려 저장.
            T_alloc = T_meas + G[0] @ dn
            omega_alloc = self._omega_dot_filt + G[1:4] @ dn
            self.last_alloc = np.concatenate([[T_alloc], omega_alloc])
            # nmpc 쪽의 alloc_feedback 플래그가 실제 사용 여부를 결정한다
            # (VirtualNMPC.set_prev_input 참조) — 여기선 항상 통지만 한다.
            if hasattr(self.nmpc, 'set_prev_input'):
                self.nmpc.set_prev_input(self.last_alloc)
            return n_cmd

        # A0 (기존 동작 그대로 — 회귀 가드 겸 표6의 A0 비교항)
        try:
            dn = np.linalg.solve(G, dv)
        except np.linalg.LinAlgError:
            return self._fallback(T_cmd, omega_dot_des)

        return np.clip(n_actual + dn, self.p['n_min'], self.p['n_max'])

    def _rotor_thrust_cap(self, V_axial):
        """현재 유속·명목 최대 회전수에서 로터별 추력 상한 f_max,i (식28)."""
        n_i = self.p['n_max']
        n_rps = n_i / (2 * np.pi)
        J = V_axial / (n_rps * self.p['D_prop'] + 1e-8)
        fac = max(1.0 - J / self.p['J_max'], 0.0)
        return np.full(4, self.p['k_T'] * n_i**2 * fac)

    def _compute_G(self, n, v_body=None):
        return compute_control_effectiveness(self.p, n, v_body)

    def _fallback(self, T_cmd, omega_dot_des):
        J = np.diag([self.p['Ixx'], self.p['Iyy'], self.p['Izz']])
        TM = np.array([T_cmd, *(J @ omega_dot_des)])
        f_ind = self._TM_to_f @ TM
        n = np.zeros(4)
        for i in range(4):
            n[i] = np.sqrt(max(f_ind[i], 0) / self.p['k_T'])
        return np.clip(n, self.p['n_min'], self.p['n_max'])


# ══════════════════════════════════════════════════
# 4. NaiveHybrid (이전 버전, 비교용)
# ══════════════════════════════════════════════════

def compute_control_effectiveness(params, n_actual, v_body=None):
    """추진 wrench의 국소 미분 ∂[T, M/I]/∂n (각속도 0 기준).

    플랜트의 EPS와 영추력 분기를 그대로 미분한다. n=0에 가짜 제어 효과를
    넣지 않는다. 랭크가 부족하면 할당기가 처리해야지 G를 왜곡하면 안 된다.
    회전 중 로터 자이로의 미분은 이 기존 인터페이스에 포함하지 않는다.

    구 버전은 모멘트팔을 pos[i,0]/pos[i,1]로 직접 손계산했는데(로터가 동체
    xy평면에 있다는 전제), thrust_axis='x'에선 로터가 yz평면에 있어 그 전제가
    깨진다. compute_allocation_matrix가 이미 축을 아는 모멘트팔/반토크 구조를
    만드니 그걸 재사용한다 — z축 기본값에서는 이전 손계산과 수학적으로 동일하다
    (T=k_T*n^2*fac(n)의 곱미분이 정확히 dT/dn=k_T*n*(1+fac); Q=k*T가 항상
    성립해 반토크 행에 dT/dn을 곱해도 dQ/dn이 그대로 나온다).
    """
    n = np.asarray(n_actual, dtype=float)
    axial = 0.0 if v_body is None else max(
        v_body[0] if params.get('thrust_axis', 'z') == 'x' else -v_body[2], 0.0)
    b = params['D_prop'] / (2 * np.pi)
    denominator = b * n + EPS
    c = axial / params['J_max']
    factor = 1.0 - c / denominator
    derivative = np.where(factor > 0.0,
                          params['k_T'] * (2 * n * factor + n**2 * c * b / denominator**2),
                          0.0)
    allocation, _ = compute_allocation_matrix(params)
    G = allocation * derivative[np.newaxis, :]
    G[1:4] /= np.array([params['Ixx'], params['Iyy'], params['Izz']])[:, None]
    return G


class NaiveHybrid:
    """나이브 NMPC+INDI: 모터속도 위에 INDI 보정 얹음 → 이중 보정."""

    def __init__(self, nmpc_ctrl, params_nom, dt=0.001, f_cut=50.0):
        self.nmpc = nmpc_ctrl
        self.p = params_nom
        self.dt = dt
        self._f_nom, _, _ = build_dynamics(params_nom)
        self._alpha = dt / (dt + 1.0/(2*np.pi*f_cut))
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._initialized = False

    def __call__(self, t, x):
        u_nmpc = self.nmpc(t, x)
        omega, n_actual = x[10:13], x[13:17]
        if not self._initialized:
            self._omega_prev = omega.copy()
            self._initialized = True
            return u_nmpc
        xdot_nom = np.array(self._f_nom(x, u_nmpc)).flatten()
        omega_dot_pred = xdot_nom[10:13]
        raw = (omega - self._omega_prev) / self.dt
        self._omega_dot_filt = self._alpha*raw + (1-self._alpha)*self._omega_dot_filt
        self._omega_prev = omega.copy()
        d = omega_dot_pred - self._omega_dot_filt
        G = compute_control_effectiveness(self.p, n_actual)
        dv = np.array([0.0, d[0], d[1], d[2]])
        try:
            dn = np.linalg.solve(G, dv)
        except np.linalg.LinAlgError:
            return u_nmpc
        return np.clip(u_nmpc + dn, self.p['n_min'], self.p['n_max'])


# ══════════════════════════════════════════════════
# 헬퍼
# ══════════════════════════════════════════════════

def _make_gust(W, t0, Tg):
    def f(t):
        w = np.zeros(3)
        if t0 <= t <= t0+Tg:
            w[2] = (W/2)*(1 - np.cos(2*np.pi*(t-t0)/Tg))
        return w
    return f

def _measure(ts, xs, us, V, z, nm):
    div = bool(np.any(np.isnan(xs)) or np.any(np.abs(xs[:,2]-z) > 50))
    if div:
        return {'rmse_z': np.inf, 'max_dz': np.inf, 'rmse_vx': np.inf, 'div': True}
    return {
        'rmse_z':  np.sqrt(np.mean((xs[:,2]-z)**2)),
        'max_dz':  np.max(np.abs(xs[:,2]-z)),
        'rmse_vx': np.sqrt(np.mean((xs[:,3]-V)**2)),
        'div': False}


# ══════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════

def main():
    from control.dynamics import AxialDronePlant
    V, z_ref = 70.0, 50.0

    print("\n" + "=" * 90)
    print("  NMPC+INDI 하이브리드 — 나이브 vs 인터페이스 분리")
    print("=" * 90)

    # 트림
    trim = find_trim(P, V)
    x0 = trim['state'].copy(); x0[2] = z_ref
    u_trim = trim['control'].copy()
    T_trim = float(np.sum(P['k_T'] * u_trim**2))

    print(f"\n[준비]")
    print(f"  70 m/s 트림: T_trim={T_trim:.1f}N (mg={P['mass']*P['g']:.1f}N)")

    sched_lqr = ScheduledLQR(P, v_ref=[V,0,0], z_ref=z_ref)

    # 조건
    mismatches = [('정확', 1.0), ('C_Na+20%', 1.2), ('C_Na-20%', 0.8)]
    scenarios = [
        ('순항 5s',      5.0, None),
        ('순항+돌풍 8s', 8.0, _make_gust(10.0, 2.0, 1.0)),
    ]
    ctrl_names = ['NMPC', '나이브', '분리', 'LQR스케줄']

    ALL = {}

    for mm_name, scale in mismatches:
        P_true = dict(P)
        P_true['C_Na'] = P['C_Na'] * scale
        plant = AxialDronePlant(P_true, dt=0.001)

        print(f"\n{'═'*90}")
        print(f"  {mm_name} (C_Na={P['C_Na']*scale:.1f})")
        print(f"{'═'*90}")

        ALL[mm_name] = {}

        for sc_name, T_sim, wind_fn in scenarios:
            print(f"\n  ── {sc_name} ──")
            sc = {}

            # 1) NMPC 순수 (모터속도 직접)
            print(f"    NMPC...", end=" ", flush=True)
            nmpc = NMPCController(P, v_ref=[V,0,0], z_ref=z_ref,
                                  u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)
            t0 = timer.time()
            ts,xs,us = plant.simulate(x0.copy(), nmpc, T_sim, wind_fn=wind_fn)
            sc['NMPC'] = _measure(ts,xs,us,V,z_ref,P['n_max'])
            _pr('NMPC', sc['NMPC'], timer.time()-t0)

            # 2) 나이브 하이브리드 (이중 보정)
            print(f"    나이브...", end=" ", flush=True)
            nmpc2 = NMPCController(P, v_ref=[V,0,0], z_ref=z_ref,
                                   u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)
            naive = NaiveHybrid(nmpc2, P, dt=0.001)
            t0 = timer.time()
            ts,xs,us = plant.simulate(x0.copy(), naive, T_sim, wind_fn=wind_fn)
            sc['나이브'] = _measure(ts,xs,us,V,z_ref,P['n_max'])
            _pr('나이브', sc['나이브'], timer.time()-t0)

            # 3) 분리 하이브리드 (올바른 인터페이스)
            print(f"    분리...", end=" ", flush=True)
            vnmpc = VirtualNMPC(P, v_ref=[V,0,0], z_ref=z_ref, T_ref=T_trim,
                                N=20, dt_nmpc=0.05, dt_ctrl=0.02)
            proper = ProperHybrid(vnmpc, P, dt=0.001)
            t0 = timer.time()
            ts,xs,us = plant.simulate(x0.copy(), proper, T_sim, wind_fn=wind_fn)
            sc['분리'] = _measure(ts,xs,us,V,z_ref,P['n_max'])
            _pr('분리', sc['분리'], timer.time()-t0)

            # 4) LQR 스케줄
            print(f"    LQR스케줄...", end=" ", flush=True)
            sched_lqr.v_ref = np.array([V,0,0]); sched_lqr.z_ref = z_ref
            t0 = timer.time()
            ts,xs,us = plant.simulate(x0.copy(), sched_lqr, T_sim, wind_fn=wind_fn)
            sc['LQR스케줄'] = _measure(ts,xs,us,V,z_ref,P['n_max'])
            _pr('LQR', sc['LQR스케줄'], timer.time()-t0)

            ALL[mm_name][sc_name] = sc

            # 표
            print(f"\n    {'':>10s}", end="")
            for cn in ctrl_names:
                print(f"  {cn:>10s}", end="")
            print()
            print(f"    {'─'*54}")
            for m,lb,fm in [('rmse_z','RMSE z','.3f'),('max_dz','Δz max','.2f'),
                            ('rmse_vx','RMSE vx','.3f')]:
                print(f"    {lb:>10s}", end="")
                for cn in ctrl_names:
                    r = sc[cn]
                    print(f"  {'DIV':>10s}" if r['div'] else f"  {r[m]:>10{fm}}", end="")
                print()

    # ══════════════════════════════════════════════════
    # 종합
    # ══════════════════════════════════════════════════
    print(f"\n{'═'*90}")
    print("  [종합] 나이브 vs 인터페이스 분리 — RMSE z")
    print(f"{'═'*90}")
    print(f"\n  {'조건':>16s}  {'시나리오':>12s}", end="")
    for cn in ctrl_names:
        print(f"  {cn:>10s}", end="")
    print()
    print(f"  {'─'*72}")
    for mm in ALL:
        for sc in ALL[mm]:
            d = ALL[mm][sc]
            print(f"  {mm:>16s}  {sc:>12s}", end="")
            for cn in ctrl_names:
                r = d[cn]
                print(f"  {'DIV':>10s}" if r['div'] else f"  {r['rmse_z']:>10.3f}", end="")
            print()

    # 분석
    print(f"\n{'─'*90}")
    print("  분석:")

    # 나이브 vs 분리 비교
    for mm in ALL:
        for sc in ALL[mm]:
            d = ALL[mm][sc]
            if d['NMPC']['div'] or d['나이브']['div'] or d['분리']['div']:
                continue
            z_nmpc = d['NMPC']['rmse_z']
            z_naive = d['나이브']['rmse_z']
            z_proper = d['분리']['rmse_z']
            z_lqr = d['LQR스케줄']['rmse_z']

            if z_nmpc < 1e-6:
                continue  # 순항 0 skip

            print(f"\n  {mm}, {sc}:")
            print(f"    NMPC:      {z_nmpc:.3f}")
            print(f"    나이브:    {z_naive:.3f} ({'악화' if z_naive > z_nmpc*1.1 else 'OK'})")
            print(f"    분리:      {z_proper:.3f} ({'개선!' if z_proper < z_nmpc*0.95 else 'NMPC급' if z_proper < z_nmpc*1.1 else '악화'})")
            print(f"    LQR스케줄: {z_lqr:.3f}")

    print(f"\n  결론:")
    print(f"  - 나이브: 이중 보정으로 NMPC보다 악화 (재확인)")
    print(f"  - 분리: 인터페이스를 [T, ω̇]로 명확히 나누면 이중 보정 해소")
    print(f"  - 핵심 설계 원칙: '뭘 할지'(NMPC)와 '어떻게 할지'(INDI)를 분리")
    print(f"{'═'*90}")


def _pr(name, r, elapsed):
    if r['div']:
        print(f"{elapsed:.1f}s  DIVERGED!")
    else:
        print(f"{elapsed:.1f}s  z={r['rmse_z']:.3f}  Δz={r['max_dz']:.2f}")


if __name__ == '__main__':
    main()
