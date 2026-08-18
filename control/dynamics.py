"""
축대칭 미사일형 동체 + 쿼드콥터 추진 — 6자유도 동역학 (CasADi)
================================================================

좌표계:
  관성: z-up, 중력 = [0, 0, -mg]
  동체: x 전방, y 우측, z 하방  (우수: x×y=z ✓)
  로터 추력: body -z 방향 (위로 뜸)

  양의 α: w_b > 0 (기수가 속도 위)
  양의 M_y: 기수 상승 (x→-z, 우수법칙 y축)

상태 x(17): [p(3), v(3), q(4), ω(3), n(4)]
제어 u(4):  [n1_cmd, n2_cmd, n3_cmd, n4_cmd]

호버 쿼터니언: q = [1,0,0,0]  (180° about x → body z-down = inertial -z)
"""

import casadi as ca
import numpy as np

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
    u_b, v_b, w_b = v_body[0], v_body[1], v_body[2]
    rho, S, d = p['rho'], p['S_ref'], p['d_ref']

    V_sq  = u_b**2 + v_b**2 + w_b**2 + EPS
    V     = ca.sqrt(V_sq)
    V_cf  = ca.sqrt(v_b**2 + w_b**2 + EPS)
    q_bar = 0.5 * rho * V_sq

    # 수직력 (V_cf 분모 소거로 특이점 없음)
    F_N_fac = 0.5 * rho * S * (p['C_Na'] * u_b + p['C_dc'] * V_cf)
    Fy = -F_N_fac * v_b
    Fz = -F_N_fac * w_b

    # 축력 (항력, 전방비행 시 -x 방향)
    C_A = p['C_A0'] + p['C_Aa2'] * (v_b**2 + w_b**2) / V_sq
    Fx = -q_bar * S * C_A

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
# 로터 (z-thrust 쿼드콥터)
# ════════════════════════════════════════════════════

def _rotor_forces_moments(v_body, n_vec, omega, p):
    """
    4로터 추력·모멘트 (RotorPy 방식 + 전진비).

    추력 방향: body -z (위로 뜸).
    V_axial = -w_b (로터 추력축 방향 유입속도).
    """
    # V_axial: 로터 디스크 위에서 아래로 흐르는 공기 속도
    # (클라이밍 시 양수 → 추력 감소)
    V_axial = ca.fmax(-v_body[2], 0.0)

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

        Ti = p['k_T'] * ni**2 * fac
        Qi = p['k_Q'] * ni**2 * fac

        # 추력 [0, 0, -T] (body -z = 위)
        F_tot += ca.vertcat(0.0, 0.0, -Ti)

        # 추력 모멘트: r × [0, 0, -T]
        # M_x = r_y·(-T), M_y = -r_x·(-T) = r_x·T, M_z = 0
        # 아래는 교차곱 직접 전개
        M_tot += ca.vertcat(
            ri[1] * (-Ti),      # r_y · (-T)
           -ri[0] * (-Ti),      # r_x · T  (= -r_x·(-T))
            0.0)

        # 반토크: CW(dir=+1) → body에 CCW 반작용
        # z-down에서 CW(+z 방향) = 정방향, 반작용 = -z 방향 = -dir·Q
        # RotorPy와 동일 부호: M_z_reaction = dir · k_m · n²
        # (z-up에선 CW→CCW반작용=+z, z-down에선 CW→CCW반작용=-z지만
        #  'dir'의 부호 규약이 프레임 따라 정의되므로 일관되게 사용)
        M_tot += ca.vertcat(0.0, 0.0, di * Qi)

        h_net += p['I_rotor'] * ni * di

    # 자이로: τ = -ω × h,  h = [0, 0, h_net] (로터 스핀축 = body z)
    # ω × [0,0,h] = [ω_y·h, -ω_x·h, 0]
    # τ_gyro = -[ω_y·h, -ω_x·h, 0] = [-ω_y·h, ω_x·h, 0]
    M_tot += ca.vertcat(-omega[1]*h_net, omega[0]*h_net, 0.0)

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


def compute_allocation_matrix(params):
    """
    [T1..T4] ↔ [T_total, Mx, My, Mz] 변환 행렬.

    추력 방향 [0,0,-1] 기준 교차곱으로 자동 생성.
    """
    e_thrust = np.array([0, 0, -1])
    k = params['k_Q'] / params['k_T']
    pos = params['rotor_positions']
    dirs = params['rotor_directions']
    nr = params['num_rotors']

    A = np.zeros((4, nr))
    for i in range(nr):
        A[0, i] = 1.0                                  # T_total
        m = np.cross(pos[i], e_thrust)                  # r × e_thrust
        A[1, i] = m[0]                                  # Mx
        A[2, i] = m[1]                                  # My
        A[3, i] = dirs[i] * k                           # Mz (반토크)

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

        self.f_to_TM, self.TM_to_f = compute_allocation_matrix(params)

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
        """호버 초기 상태. q=[1,0,0,0] = 180° about x (body z-down)."""
        n_hov = np.sqrt(params['mass'] * params['g'] / (4 * params['k_T']))
        x0 = np.zeros(NX)
        x0[6] = 1.0                         # qx = 1 (180° about x)
        x0[13:17] = n_hov                   # 호버 로터 속도
        return x0
