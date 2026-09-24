"""
F13 — 로터추력 직접입력 NMPC + INDI (Sun et al. 2022 방식 재구성)
================================================================

V13(hybrid_comparison.VirtualNMPC + ProperHybrid, alloc_mode='A1')과 논문이
비교하는 가장 가까운 선행연구 비교군. 차이는 NMPC의 결정변수/출력뿐이다:

  V13: NMPC가 가상입력 [T, ω̇] 직접 출력 → INDI가 총추력 등식 제약(A1)으로 배분.
  F13: NMPC가 로터별 추력 f∈R⁴ 직접 출력 → 총추력·모멘트로 환산해 INDI에 전달
       (F13VirtualAdapter) → 이후 보정은 ProperHybrid를 그대로 재사용한다
       (alloc_mode='A0' — F13을 특징짓는 차이는 NMPC 출력 공간이지 INDI 배분
       방식이 아니므로, INDI 쪽은 V13 이전부터 있던 비제약 해를 그대로 쓴다).

13상태 예측모델은 V13처럼 회전 동역학을 이상화(ω̇=ν)하지 않고 실제 강체
모멘트 방정식(J·ω̇=M-ω×Jω)을 쓴다 — 이게 F13과 V13의 예측모델 차이다.
모터 회전수 상태·지연은 13상태 모델이라 둘 다 생략(그건 M17만의 특징).

참고(구조적 근거, vendoring 아님): Sun, Romero, Foehn, Kaufmann, Scaramuzza,
"A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control
for Quadrotor Agile Flight," IEEE T-RO 2022 (arXiv:2109.01365). 공개 코드
(uzh-rpg/agilicious)는 C++/ROS 전체 비행스택이라 이식하지 않고, "로터추력
직접출력 NMPC + INDI"라는 정식화만 우리 CasADi 컨벤션으로 재구현했다.
"""

import numpy as np
import casadi as ca

from control.dynamics import (_quat_to_rotmat, _quat_derivative,
                               _body_aerodynamics, compute_allocation_matrix)
from control.hybrid_comparison import NX_V

NU_F = 4  # 로터별 추력 입력 [N]


def build_f13_dynamics(params):
    """13상태 예측모델, 입력은 로터별 추력(N) 직접.

    V13(hybrid_comparison.build_virtual_dynamics)과 달리 ω̇=ν로 이상화하지
    않고 실제 강체 회전방정식을 쓴다. 로터 반작용 토크는 추력에 비례하는
    상수비로 근사한다(compute_allocation_matrix의 A행렬 — 논문 부록A.1의
    γ 근사와 동일한 이상화). 모터 회전수 상태·지연은 13상태라 생략한다.
    """
    x = ca.SX.sym('x', NX_V)
    u = ca.SX.sym('u', NU_F)

    pos, vel, quat, omega = x[0:3], x[3:6], x[6:10], x[10:13]
    R = _quat_to_rotmat(quat)
    v_body = R.T @ vel

    F_aero, M_aero = _body_aerodynamics(v_body, omega, params)

    A_alloc, _ = compute_allocation_matrix(params)  # numpy(4,4): [T,Mx,My,Mz]=A@f
    TM = ca.DM(A_alloc) @ u
    T_total, M_thrust = TM[0], TM[1:4]

    thrust = (ca.vertcat(T_total, 0, 0) if params.get('thrust_axis', 'z') == 'x'
              else ca.vertcat(0, 0, -T_total))
    F_body = F_aero + thrust
    M_body = M_aero + M_thrust

    p_dot = vel
    v_dot = ca.vertcat(0, 0, -params['g']) + (R @ F_body) / params['mass']
    q_dot = _quat_derivative(quat, omega)
    J_d = ca.vertcat(params['Ixx'], params['Iyy'], params['Izz'])
    omega_dot = (1.0 / J_d) * (M_body - ca.cross(omega, J_d * omega))

    xdot = ca.vertcat(p_dot, v_dot, q_dot, omega_dot)
    f = ca.Function('f_f13', [x, u], [xdot])
    return f, x, u


class RotorThrustNMPC13:
    """F13의 상위 예측기. 인터페이스는 VirtualNMPC와 동일한 패턴(__call__(t,x17)
    → 제어값, reset(), consec_fail/last_status)이되 출력이 로터별 추력 f∈R⁴."""

    def __init__(self, params, v_ref=None, z_ref=0.0, f_hover_total=None,
                 N=20, dt_nmpc=0.05, dt_ctrl=0.02, Q_z=20.0, max_iter=30):
        self.p = params
        self.N, self.dt_nmpc, self.dt_ctrl = N, dt_nmpc, dt_ctrl
        self.v_ref = np.array(v_ref) if v_ref is not None else np.zeros(3)
        self.z_ref = z_ref
        self._Q_z = Q_z
        self._max_iter = max_iter

        T_hover = f_hover_total if f_hover_total else params['mass'] * params['g']
        self.f_ref = np.full(NU_F, T_hover / 4.0)
        # 이상화: V13의 A1 배분과 달리 유속 의존 없는 상수 상한(Sun et al. 원래
        # 정식화와 동일한 단순화) — 이게 F13과 V13의 또 다른 의도적 차이.
        self.f_max = float(params['k_T'] * params['n_max']**2)

        f13, x_sym, u_sym = build_f13_dynamics(params)
        dt = dt_nmpc
        k1 = f13(x_sym, u_sym)
        k2 = f13(x_sym + dt/2*k1, u_sym)
        k3 = f13(x_sym + dt/2*k2, u_sym)
        k4 = f13(x_sym + dt*k3, u_sym)
        self.F = ca.Function('F_f13', [x_sym, u_sym],
                             [x_sym + dt/6*(k1 + 2*k2 + 2*k3 + k4)])

        self._build_nlp(params, x_sym, u_sym)
        self._last_t = -np.inf
        self._u_current = self.f_ref.copy()
        self.consec_fail = 0
        self.last_status = 'none'
        self._ever_converged = False

    def reset(self):
        self._last_t = -np.inf
        self._u_current = self.f_ref.copy()
        self.consec_fail = 0
        self.last_status = 'none'
        self._ever_converged = False
        if self._w0_init is not None:
            self.w0 = self._w0_init.copy()

    def _build_nlp(self, params, x_sym, u_sym):
        N, nx, nu = self.N, NX_V, NU_F

        Q_v = np.diag([5.0, 5.0, 10.0])
        Q_z = self._Q_z
        Q_w = np.diag([1.0, 1.0, 1.0])
        R = np.eye(nu) * 1e-4
        R_du = np.eye(nu) * 1e-2

        p = ca.SX.sym('p', nx + 3 + 1 + nu)
        x_init = p[0:nx]
        v_ref = p[nx:nx+3]
        z_ref = p[nx+3]
        u_ref = p[nx+4:nx+4+nu]

        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J_cost = 0.0
        X_prev, U_prev = x_init, u_ref

        for k in range(N):
            U_k = ca.SX.sym(f'Uf_{k}', nu)
            w.append(U_k)
            lbw += [0.0] * nu
            ubw += [self.f_max] * nu
            w0 += list(self.f_ref)

            X_k = ca.SX.sym(f'Xf_{k}', nx)
            w.append(X_k)
            lbw += [-1e6] * nx
            ubw += [1e6] * nx
            _xg = [0.0] * nx
            _xg[6:10] = ([0.0, -np.sqrt(0.5), 0.0, np.sqrt(0.5)]
                         if params.get('thrust_axis', 'z') == 'x'
                         else [1.0, 0.0, 0.0, 0.0])
            w0 += _xg

            g.append(X_k - self.F(X_prev, U_k))
            lbg += [0.0] * nx
            ubg += [0.0] * nx

            e_v = X_k[3:6] - v_ref
            e_z = X_k[2] - z_ref
            dU = U_k - U_prev
            J_cost += e_v.T @ Q_v @ e_v + Q_z * e_z**2
            J_cost += X_k[10:13].T @ Q_w @ X_k[10:13]
            J_cost += (U_k - u_ref).T @ R @ (U_k - u_ref)
            J_cost += dU.T @ R_du @ dU
            X_prev, U_prev = X_k, U_k

        J_cost += 10 * (X_prev[3:6] - v_ref).T @ Q_v @ (X_prev[3:6] - v_ref)
        J_cost += 10 * Q_z * (X_prev[2] - z_ref)**2

        nlp = {'f': J_cost, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': p}
        self.solver = ca.nlpsol('f13nmpc', 'ipopt', nlp, {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
            'ipopt.max_iter': self._max_iter, 'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-4})
        self.lbw, self.ubw = np.array(lbw), np.array(ubw)
        self.lbg, self.ubg = np.array(lbg), np.array(ubg)
        self.w0 = np.array(w0)
        self._w0_init = self.w0.copy()

    def __call__(self, t, x_full):
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            x13 = np.concatenate([x_full[0:10], x_full[10:13]])
            self._u_current = self._solve(x13)
            self._last_t = t
        return self._u_current

    def _solve(self, x13):
        p_val = np.concatenate([x13, self.v_ref, [self.z_ref], self.f_ref])
        sol = self.solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw,
                          lbg=self.lbg, ubg=self.ubg, p=p_val)
        status = self.solver.stats().get('return_status', 'unknown')
        if status in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'):
            self.consec_fail = 0
            self._ever_converged = True
        elif self._ever_converged:
            self.consec_fail += 1
        self.last_status = status

        w_opt = np.array(sol['x']).flatten()
        u_opt = w_opt[0:NU_F]
        stride = NU_F + NX_V
        self.w0 = np.concatenate([w_opt[stride:], w_opt[-stride:]])
        return u_opt


class F13VirtualAdapter:
    """RotorThrustNMPC13(로터추력 출력)을 VirtualNMPC와 같은 [T,ω̇] 호출
    인터페이스로 감싼다 — 기존 ProperHybrid(INDI)를 그대로 재사용하기 위함.
    논문이 말하는 "로터 추력을 총추력·각가속도로 환산해 INDI에 전달"이
    정확히 이 변환이다.
    """

    def __init__(self, params, **kwargs):
        self.p = params
        self.inner = RotorThrustNMPC13(params, **kwargs)
        self._A_alloc, _ = compute_allocation_matrix(params)
        self._J = np.array([params['Ixx'], params['Iyy'], params['Izz']])

    def reset(self):
        self.inner.reset()

    def __call__(self, t, x_full):
        f = self.inner(t, x_full)           # 로터별 추력(N)
        omega = x_full[10:13]
        TM = self._A_alloc @ f               # [T, Mx, My, Mz]
        T_total, M = TM[0], TM[1:4]
        omega_dot_des = (M - np.cross(omega, self._J * omega)) / self._J
        return np.concatenate([[T_total], omega_dot_des])

    def set_prev_input(self, v):
        """ProperHybrid가 alloc_mode='A1'일 때만 호출하는 훅. F13은 정의상
        배분결과 피드백(식31-32, V13 전용)을 쓰지 않으므로 항상 무시한다."""
        pass

    @property
    def consec_fail(self):
        return self.inner.consec_fail

    @property
    def last_status(self):
        return self.inner.last_status


def build_f13_controller(params, v_ref=None, z_ref=0.0, dt=0.001, **nmpc_kwargs):
    """F13 = F13VirtualAdapter(로터추력 NMPC) + 기존 ProperHybrid INDI(A0)."""
    from control.hybrid_comparison import ProperHybrid
    adapter = F13VirtualAdapter(params, v_ref=v_ref, z_ref=z_ref, **nmpc_kwargs)
    return ProperHybrid(adapter, params, dt=dt, alloc_mode='A0')
