"""
고정 경량 로켓형 연구 기체 — 6자유도 동역학 (CasADi)
================================================================

좌표계:
  관성: z-up, 중력 = [0, 0, -mg]
  동체: x 전방, y 우측, z 하방  (우수: x×y=z ✓)
  로터 추력: body +x 방향 (동체 길이축과 평행)
  y/z는 동체 고정축이며 수직 호버에서 세계 우측/하방을 뜻하지 않는다.

  양의 α: w_b > 0 (기수가 속도 위)
  양의 M_y: 기수 상승 (x→-z, 우수법칙 y축)

상태 x(17): [p(3), v(3), q(4), ω(3), n(4)]
제어 u(4):  [n1_cmd, n2_cmd, n3_cmd, n4_cmd]

호버: body +x가 세계 +z를 향하는 기수 상향 자세 (geometry.hover_quaternion).
"""

import casadi as ca
import numpy as np
from models.team_light.control.geometry import thrust_axis, hover_quaternion
from models.team_light.control.propeller_curve import uses_curve, symbolic_force_torque

EPS = 1e-8
NX  = 17
NU  = 4

# ════════════════════════════════════════════════════
# 헬퍼
# ════════════════════════════════════════════════════

def _quat_to_rotmat(q):
    """쿼터니언 [i,j,k,w] → R (동체→관성)."""
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        ca.horzcat(1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)),
        ca.horzcat(2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)),
        ca.horzcat(2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)))


def _quat_derivative(q, omega):
    """q_dot = 0.5·G^T·ω + Baumgarte 안정화."""
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    G = ca.vertcat(
        ca.horzcat( q3,  q2, -q1, -q0),
        ca.horzcat(-q2,  q3,  q0, -q1),
        ca.horzcat( q1, -q0,  q3, -q2))
    qd = 0.5 * G.T @ omega
    qd -= 1.0 * (ca.dot(q, q) - 1.0) * q    # Baumgarte
    return qd


# ════════════════════════════════════════════════════
# 공력
# ════════════════════════════════════════════════════

def _body_aerodynamics(v_body, omega, p):
    """
    축대칭 동체 공력 (z-down 동체).

    양의 α (w_b > 0): 공기가 아래에서 옴 → 수직력 F_z < 0 (위로 양력)
    정적 안정 (x_cp < 0):
      M_y = -x_cp · F_z = -(-)·(-) = 음수 → 기수 하강 → 복원 ✓
      (z-down에서 양의 M_y = 기수 상승, 음의 M_y = 기수 하강)
    """
    if p.get('aero_model') == 'distributed_light_v1':
        from models.team_light.control.light_aero import light_aerodynamics
        return light_aerodynamics(v_body, omega, p)
    u_b, v_b, w_b = v_body[0], v_body[1], v_body[2]
    rho, S, d = p['rho'], p['S_ref'], p['d_ref']

    V_sq  = u_b**2 + v_b**2 + w_b**2 + EPS
    V     = ca.sqrt(V_sq)
    V_cf  = ca.sqrt(v_b**2 + w_b**2 + EPS)
    q_bar = 0.5 * rho * V_sq

    # 수직력 (V_cf 분모 소거로 특이점 없음)
    # Nose-up hover/descending flight can have negative axial velocity.
    # Keep drag dissipative in reverse flow; coefficients remain unvalidated there.
    F_N_fac = 0.5 * rho * S * (p['C_Na'] * ca.sqrt(u_b**2 + EPS) + p['C_dc'] * V_cf)
    Fy = -F_N_fac * v_b
    Fz = -F_N_fac * w_b

    # 축력 (항력, 전방비행 시 -x 방향)
    C_A = p['C_A0'] + p['C_Aa2'] * (v_b**2 + w_b**2) / V_sq
    Fx = -q_bar * S * C_A * u_b / ca.sqrt(u_b**2 + EPS)

    F_aero = ca.vertcat(Fx, Fy, Fz)

    # 정적 모멘트: r_cp × F_N
    xcp = p['x_cp']
    M_static = ca.vertcat(0.0, -xcp * Fz, xcp * Fy)

    # 감쇠 모멘트: 0.25·ρ·V·S·d²·C_damp·ω
    df = 0.25 * rho * V * S * d**2
    M_damp = df * ca.vertcat(
        p['C_lp'] * omega[0],
        p['C_mq'] * omega[1],
        p['C_mq'] * omega[2])

    return F_aero, M_static + M_damp


# ════════════════════════════════════════════════════
# 로터 (body +x thrust, yz-plane layout)
# ════════════════════════════════════════════════════

def _rotor_forces_moments(v_body, n_vec, omega, p):
    """
    4로터 추력·모멘트 (RotorPy 방식 + 전진비).

    추력 방향: body +x. dirs는 동체 반작용 토크의 부호이다.
    기존 모델과 같이 로터 각가속도 반작용(-h_dot)은 포함하지 않는다.
    """
    # Positive air-relative velocity along body +x reduces rotor thrust.
    axis = ca.DM(thrust_axis(p))
    V_axial = ca.fmax(ca.dot(axis, v_body), 0.0)

    pos = p['rotor_positions']     # (4,3)
    dirs = p['rotor_directions']   # (4,)

    F_tot = ca.SX.zeros(3)
    M_tot = ca.SX.zeros(3)
    h_net = 0.0

    for i in range(p['num_rotors']):
        ni = n_vec[i]
        di = float(dirs[i])
        ri = pos[i]

        # 전진비
        n_rps = ni / (2.0 * ca.pi)
        J = V_axial / (n_rps * p['D_prop'] + EPS)
        fac = ca.fmax(1.0 - J / p['J_max'], 0.0)

        if uses_curve(p):
            Ti, Qi = symbolic_force_torque(p, ni, V_axial)
        else:
            Ti = p['k_T'] * ni**2 * fac
            Qi = p['k_Q'] * ni**2 * fac

        F_tot += axis * Ti
        M_tot += ca.cross(ca.DM(ri), axis * Ti) + axis * (di * Qi)
        # 로터 각운동량은 동체에 작용하는 반토크와 반대 방향.
        h_net -= p['I_rotor'] * ni * di

    M_tot -= ca.cross(omega, axis * h_net)

    return F_tot, M_tot


# ════════════════════════════════════════════════════
# 동역학 조립
# ════════════════════════════════════════════════════

def _compute_xdot(x, u, p, w=None):
    pos, vel, quat = x[0:3], x[3:6], x[6:10]
    omega, n_vec = x[10:13], x[13:17]
    n_cmd = u[0:4]

    R = _quat_to_rotmat(quat)
    # 대기속도 = 관성 속도 - 바람 (바람이 있으면 공력에 영향)
    v_body = R.T @ (vel - w) if w is not None else R.T @ vel

    F_a, M_a = _body_aerodynamics(v_body, omega, p)
    F_r, M_r = _rotor_forces_moments(v_body, n_vec, omega, p)
    F_body = F_a + F_r
    M_body = M_a + M_r

    p_dot = vel
    v_dot = ca.vertcat(0, 0, -p['g']) + (R @ F_body) / p['mass']
    q_dot = _quat_derivative(quat, omega)

    J_d = ca.vertcat(p['Ixx'], p['Iyy'], p['Izz'])
    w_dot = (1.0 / J_d) * (M_body - ca.cross(omega, J_d * omega))

    n_dot = (n_cmd - n_vec) / p['tau_m']

    return ca.vertcat(p_dot, v_dot, q_dot, w_dot, n_dot)


# ════════════════════════════════════════════════════
# 공개 API
# ════════════════════════════════════════════════════

def build_dynamics(params):
    """CasADi Function f(x[17], u[4]) → x_dot[17]."""
    x = ca.SX.sym('x', NX)
    u = ca.SX.sym('u', NU)
    xd = _compute_xdot(x, u, params)
    f = ca.Function('f_dynamics', [x, u], [xd], ['x', 'u'], ['x_dot'])
    return f, x, u


def compute_allocation_matrix(params, *, static_reference=False):
    """
    [T1..T4] ↔ [T_total, Mx, My, Mz] 변환 행렬.

    body +x 추력과 반작용 토크를 같은 규약으로 생성.
    Variable-Q/T profiles have NO exact constant matrix. Opt-in static seed only.
    """
    if uses_curve(params) and not static_reference:
        raise ValueError('Variable Q/T: use rotor_wrench/allocate_wrench, not a constant allocation matrix.')
    e_thrust = thrust_axis(params)
    k = params['k_Q'] / params['k_T']
    pos = params['rotor_positions']
    dirs = params['rotor_directions']
    nr = params['num_rotors']

    A = np.zeros((4, nr))
    for i in range(nr):
        A[0, i] = 1.0                                  # T_total
        m = np.cross(pos[i], e_thrust)                  # r × e_thrust
        A[1:4, i] = m + dirs[i] * k * e_thrust

    return A, np.linalg.inv(A)


class AxialDronePlant:
    """시뮬레이션 플랜트. CasADi RK4 적분."""

    def __init__(self, params, dt=0.001):
        self.params = params
        self.dt = dt
        self.nx, self.nu = NX, NU

        # 바람 없는 동역학 (트림·선형화·NMPC 모델용, 역호환)
        self.f, self.x_sym, self.u_sym = build_dynamics(params)

        # 바람 포함 동역학 (시뮬레이션용)
        # 적분기 파라미터: p = [u(4), w(3)] = 7D
        # 바람이 없으면 w=0 전달 → 기존과 동일
        self.w_sym = ca.SX.sym('w', 3)
        xd_w = _compute_xdot(self.x_sym, self.u_sym, params, self.w_sym)
        self.f_wind = ca.Function('f_wind',
                                   [self.x_sym, self.u_sym, self.w_sym], [xd_w])
        p_sim = ca.vertcat(self.u_sym, self.w_sym)
        dae = {'x': self.x_sym, 'p': p_sim, 'ode': xd_w}
        self.integrator = ca.integrator(
            'plant', 'rk', dae, 0.0, dt,
            {'number_of_finite_elements': 4})

        # Legacy attributes are static references only for variable-Q/T profiles.
        self.f_to_TM, self.TM_to_f = compute_allocation_matrix(params, static_reference=True)

    def step(self, x, u, w=None):
        if w is None:
            w = np.zeros(3)
        p = np.concatenate([u, w])
        xn = np.array(self.integrator(x0=x, p=p)['xf']).flatten()
        q = xn[6:10]
        qn = np.linalg.norm(q)
        if qn > 1e-10:
            xn[6:10] = q / qn
        xn[13:17] = np.clip(xn[13:17], self.params['n_min'], self.params['n_max'])
        return xn

    def simulate(self, x0, controller, T, wind_fn=None):
        """
        궤적 시뮬레이션.

        controller: callable(t, x) → u[4]
        wind_fn:    callable(t) → w[3] (관성 바람 속도) 또는 None
        """
        N = int(round(T / self.dt))
        ts = np.linspace(0, T, N + 1)
        xs = np.zeros((N + 1, self.nx))
        us = np.zeros((N, self.nu))
        xs[0] = x0
        for k in range(N):
            us[k] = controller(ts[k], xs[k])
            w = wind_fn(ts[k]) if wind_fn is not None else None
            xs[k + 1] = self.step(xs[k], us[k], w)
        return ts, xs, us

    def evaluate_xdot(self, x, u, w=None):
        if w is not None:
            return np.array(self.f_wind(x, u, w)).flatten()
        return np.array(self.f(x, u)).flatten()

    @staticmethod
    def hover_state(params):
        """body +x가 세계 +z를 향하는 수직 호버 초기 상태."""
        n_hov = np.sqrt(params['mass'] * params['g'] / (4 * params['k_T']))
        x0 = np.zeros(NX)
        x0[6:10] = hover_quaternion()
        x0[13:17] = n_hov                   # 호버 로터 속도
        return x0
