"""
축대칭 미사일형 드론 6자유도 동역학 모델 (CasADi)
=================================================

RotorPy의 Multirotor 클래스를 참고하되, 다음을 변경:
  1) 동체 양력/수직력 추가 (RotorPy에 없던 핵심)
  2) 전진비(J) 의존 추력 모델
  3) numpy → CasADi 심볼릭 (acados NMPC 재사용 가능)
  4) 추력 방향 = body x (전방), 차동 추력으로 자세 제어

좌표계:
  관성(Inertial) 프레임:  z축 상방, 중력 = [0, 0, -mg]
  동체(Body) 프레임:      x_b 전방(노즈), y_b 우측, z_b 상방
  (우수 좌표계: x×y = z.  R=I일 때 동체=관성, 수평비행.)

  양의 받음각(AoA): 기수가 속도 위 → v_body의 w_b < 0
  양의 M_y: 우수법칙 y축 회전 → 기수 하강 (x→-z 방향)

상태 벡터 x (17차원):
  x[0:3]   = p       관성 위치          [m]
  x[3:6]   = v       관성 속도          [m/s]
  x[6:10]  = q       쿼터니언 [i,j,k,w] 동체→관성
  x[10:13] = omega   동체 각속도 [p,q,r] [rad/s]
  x[13:17] = n       로터 속도 4개      [rad/s]

제어 입력 u (4차원):
  u[0:4] = n_cmd   각 로터 속도 명령 [n1,n2,n3,n4] [rad/s]

로터 배치 (body y-z 평면, + 패턴):
       [r3 상]
         |
  [r2 좌]─●─[r1 우]  → body x (전방/추력)
         |
       [r4 하]

제어 할당:
  총 추력: T = Σ T_i
  피치:    M_y = d·(T3 - T4)       상/하 차동
  요:      M_z = d·(-T1 + T2)      우/좌 차동
  롤:      M_x = k·(-T1+T2-T3+T4)  반토크 차동
"""

import casadi as ca
import numpy as np

# ──────────────────────────────────────────────────────────────
# 상수
# ──────────────────────────────────────────────────────────────
EPS = 1e-8
NX = 17      # 상태 차원: p(3)+v(3)+q(4)+w(3)+n(4)
NU = 4       # 제어 차원: n_cmd(4)


# ══════════════════════════════════════════════════════════════
# 1. CasADi 심볼릭 헬퍼 함수
# ══════════════════════════════════════════════════════════════

def _quat_to_rotmat(q):
    """쿼터니언 [i,j,k,w] → 3x3 회전행렬 (동체→관성)."""
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    R = ca.vertcat(
        ca.horzcat(1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)),
        ca.horzcat(2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)),
        ca.horzcat(2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2))
    )
    return R


def _quat_derivative(q, omega):
    """
    쿼터니언 시간미분 + Baumgarte 안정화.

    q     = [qx, qy, qz, qw]  (scalar-last)
    omega = [p, q, r]          동체 각속도

    Baumgarte 항이 q를 단위구(|q|=1)로 끌어당겨
    acados NMPC 예측 지평선에서도 쿼터니언 drift 방지.
    """
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    G = ca.vertcat(
        ca.horzcat( q3,  q2, -q1, -q0),
        ca.horzcat(-q2,  q3,  q0, -q1),
        ca.horzcat( q1, -q0,  q3, -q2)
    )
    q_dot = 0.5 * G.T @ omega

    # Baumgarte 안정화
    K_quat = 1.0
    q_err = ca.dot(q, q) - 1.0
    q_dot = q_dot - K_quat * q_err * q

    return q_dot


# ══════════════════════════════════════════════════════════════
# 2. 공력 / 추력 서브모델
# ══════════════════════════════════════════════════════════════

def _body_aerodynamics(v_body, omega, p):
    """
    축대칭 동체의 공력: 수직력 + 축력 + 정적 모멘트 + 감쇠 모멘트.

    V=0 특이점 처리: atan2 불사용, 대수적 소거로 singularity-free.
    """
    u_b = v_body[0]
    v_b = v_body[1]
    w_b = v_body[2]

    rho   = p['rho']
    S_ref = p['S_ref']
    d_ref = p['d_ref']

    V_sq    = u_b**2 + v_b**2 + w_b**2 + EPS
    V       = ca.sqrt(V_sq)
    V_cf_sq = v_b**2 + w_b**2
    V_cf    = ca.sqrt(V_cf_sq + EPS)
    q_bar   = 0.5 * rho * V_sq

    # 수직력 (Normal Force) — V_cf 분모 소거로 특이점 없음
    F_N_factor = 0.5 * rho * S_ref * (p['C_Na'] * u_b + p['C_dc'] * V_cf)
    F_aero_y = -F_N_factor * v_b
    F_aero_z = -F_N_factor * w_b

    # 축력 (Axial Force = 항력)
    C_A = p['C_A0'] + p['C_Aa2'] * V_cf_sq / V_sq
    F_aero_x = -q_bar * S_ref * C_A

    F_aero = ca.vertcat(F_aero_x, F_aero_y, F_aero_z)

    # 정적 모멘트 (수직력 × CP 모멘트 팔)
    x_cp = p['x_cp']
    M_static = ca.vertcat(0.0, -x_cp * F_aero_z, x_cp * F_aero_y)

    # 감쇠 모멘트
    damp_factor = 0.25 * rho * V * S_ref * d_ref**2
    M_damp = damp_factor * ca.vertcat(
        p['C_lp'] * omega[0],
        p['C_mq'] * omega[1],
        p['C_mq'] * omega[2]
    )

    return F_aero, M_static + M_damp


def _rotor_forces_moments(v_body, n_vec, omega, p):
    """
    4개 로터의 추력, 반토크, 자이로 토크 합산.

    각 로터:
      - 추력 T_i = k_T · n_i² · max(1 - J/J_max, 0),  방향 = body +x
      - 위치 r_i에서 추력 → 교차곱으로 모멘트
      - 반토크 Q_i → 롤(M_x) 모멘트
      - 4개 로터 각운동량 합산 → 자이로 토크

    Parameters
    ----------
    v_body : ca.SX(3)   동체 좌표계 대기속도
    n_vec  : ca.SX(4)   각 로터 속도 [n1,n2,n3,n4]
    omega  : ca.SX(3)   동체 각속도 [p,q,r]
    p      : dict

    Returns
    -------
    F_total : ca.SX(3)  총 추력 (동체 좌표계)
    M_total : ca.SX(3)  총 모멘트 (동체 좌표계)
    """
    V_axial = v_body[0]

    F_total = ca.SX.zeros(3)
    M_total = ca.SX.zeros(3)
    h_rotor_net = 0.0   # 순(net) 로터 각운동량 (x축)

    rotor_pos = p['rotor_positions']    # (4, 3) numpy array
    rotor_dir = p['rotor_directions']   # (4,) numpy array

    for i in range(p['num_rotors']):
        n_i   = n_vec[i]
        dir_i = float(rotor_dir[i])
        r_i   = rotor_pos[i]            # [x, y, z] numpy

        # ── 전진비 ──
        n_rps = n_i / (2.0 * ca.pi)
        J = V_axial / (n_rps * p['D_prop'] + EPS)

        # ── 추력 ──
        thrust_factor = ca.fmax(1.0 - J / p['J_max'], 0.0)
        T_i = p['k_T'] * n_i**2 * thrust_factor

        # 추력 벡터 (body +x 방향)
        F_total += ca.vertcat(T_i, 0.0, 0.0)

        # ── 추력에 의한 모멘트 ──
        # M = r × [T, 0, 0] = [0, r_z·T, -r_y·T]
        M_total += ca.vertcat(
            0.0,
            r_i[2] * T_i,      # z offset → pitch moment
            -r_i[1] * T_i       # y offset → yaw moment
        )

        # ── 반토크 ──
        # 로터 CW(dir=+1) → 동체에 CCW 반토크 = -Q along x
        Q_i = p['k_Q'] * n_i**2 * thrust_factor
        M_total += ca.vertcat(-dir_i * Q_i, 0.0, 0.0)

        # ── 로터 각운동량 누적 ──
        h_rotor_net += p['I_rotor'] * n_i * dir_i

    # ── 자이로 토크 ──
    # h = [h_net, 0, 0],  τ_gyro = -ω × h = [0, -r·h, q·h]
    # 4로터 CW/CCW 교대 → h_net ≈ 0 (등속 시). 차동속도 시에만 발생.
    M_total += ca.vertcat(
        0.0,
        -omega[2] * h_rotor_net,
         omega[1] * h_rotor_net
    )

    return F_total, M_total


# ══════════════════════════════════════════════════════════════
# 3. 동역학 조립
# ══════════════════════════════════════════════════════════════

def _compute_xdot(x, u, p):
    """
    x_dot = f(x, u) — 6-DOF 강체 + 4모터 동역학.

    블록 1: ṗ = v
    블록 2: m·v̇ = 중력 + R·F_body
    블록 3: q̇ = quaternion kinematics + Baumgarte
    블록 4: J·ω̇ = τ - ω×(J·ω)
    블록 5: ṅ_i = (n_cmd_i - n_i) / τ_m  (i = 1..4)
    """
    # ── 상태 분해 ──
    pos   = x[0:3]
    vel   = x[3:6]
    quat  = x[6:10]
    omega = x[10:13]
    n_vec = x[13:17]     # 4개 로터 속도

    # ── 제어 입력 ──
    n_cmd = u[0:4]       # 4개 로터 속도 명령

    # ── 회전 행렬 ──
    R = _quat_to_rotmat(quat)

    # ── 동체 좌표계 대기속도 ──
    v_body = R.T @ vel

    # ── 힘·모멘트 합산 ──
    F_aero, M_aero = _body_aerodynamics(v_body, omega, p)
    F_rotor, M_rotor = _rotor_forces_moments(v_body, n_vec, omega, p)

    F_body = F_aero + F_rotor
    M_body = M_aero + M_rotor

    # ── 블록 1: 위치 운동학 ──
    p_dot = vel

    # ── 블록 2: 병진 동역학 ──
    g_vec = ca.vertcat(0.0, 0.0, -p['g'])
    v_dot = g_vec + (R @ F_body) / p['mass']

    # ── 블록 3: 회전 운동학 ──
    q_dot = _quat_derivative(quat, omega)

    # ── 블록 4: 회전 동역학 ──
    J_diag     = ca.vertcat(p['Ixx'], p['Iyy'], p['Izz'])
    J_inv_diag = ca.vertcat(1.0/p['Ixx'], 1.0/p['Iyy'], 1.0/p['Izz'])
    Jw = J_diag * omega
    omega_dot = J_inv_diag * (M_body - ca.cross(omega, Jw))

    # ── 블록 5: 모터 동역학 (4개 동시) ──
    n_dot = (n_cmd - n_vec) / p['tau_m']

    return ca.vertcat(p_dot, v_dot, q_dot, omega_dot, n_dot)


# ══════════════════════════════════════════════════════════════
# 4. 공개 API
# ══════════════════════════════════════════════════════════════

def build_dynamics(params):
    """
    CasADi 심볼릭 동역학 함수 생성.

    Returns
    -------
    f     : ca.Function  f(x[17], u[4]) → x_dot[17]
    x_sym : ca.SX(17)
    u_sym : ca.SX(4)
    """
    x_sym = ca.SX.sym('x', NX)
    u_sym = ca.SX.sym('u', NU)
    x_dot_expr = _compute_xdot(x_sym, u_sym, params)

    f = ca.Function('f_dynamics',
                     [x_sym, u_sym],
                     [x_dot_expr],
                     ['x', 'u'],
                     ['x_dot'])
    return f, x_sym, u_sym


def compute_allocation_matrix(params):
    """
    제어 할당 행렬 계산: [T1,T2,T3,T4] → [T_total, Mx, My, Mz]

    역행렬(TM_to_f)로 [T,Mx,My,Mz] → 개별 추력 변환 가능.
    PID/LQR 제어기에서 사용.

    Returns
    -------
    f_to_TM : np.ndarray(4,4)   개별추력 → [T, Mx, My, Mz]
    TM_to_f : np.ndarray(4,4)   [T, Mx, My, Mz] → 개별추력
    """
    k = params['k_Q'] / params['k_T']   # 토크/추력 비
    pos = params['rotor_positions']
    dirs = params['rotor_directions']

    f_to_TM = np.zeros((4, params['num_rotors']))
    for i in range(params['num_rotors']):
        f_to_TM[0, i] = 1.0                # T_total
        f_to_TM[1, i] = -dirs[i] * k       # M_x (반토크)
        f_to_TM[2, i] = pos[i, 2]          # M_y (z-offset × thrust)
        f_to_TM[3, i] = -pos[i, 1]         # M_z (-y-offset × thrust)

    TM_to_f = np.linalg.inv(f_to_TM)
    return f_to_TM, TM_to_f


class AxialDronePlant:
    """
    시뮬레이션용 플랜트 래퍼. CasADi RK4 적분기 사용.

    사용법:
        from vehicle_params import vehicle_params
        plant = AxialDronePlant(vehicle_params, dt=0.001)

        x = plant.default_initial_state()
        u = np.array([500, 500, 500, 500])  # 4개 로터 동일 속도

        x_next = plant.step(x, u)
    """

    def __init__(self, params, dt=0.001):
        self.params = params
        self.dt = dt
        self.nx = NX
        self.nu = NU

        self.f, self.x_sym, self.u_sym = build_dynamics(params)
        self._build_integrator(dt)

        # 제어 할당 행렬 (제어기에서 사용)
        self.f_to_TM, self.TM_to_f = compute_allocation_matrix(params)

    def _build_integrator(self, dt):
        x_dot_expr = self.f(self.x_sym, self.u_sym)
        dae = {'x': self.x_sym, 'p': self.u_sym, 'ode': x_dot_expr}
        opts = {'number_of_finite_elements': 4}
        self.integrator = ca.integrator('plant_integrator', 'rk', dae,
                                         0.0, dt, opts)
        self.dt = dt

    def step(self, x, u):
        """한 타임스텝 적분 → 다음 상태."""
        result = self.integrator(x0=x, p=u)
        x_next = np.array(result['xf']).flatten()

        # 쿼터니언 재정규화
        q = x_next[6:10]
        q_norm = np.linalg.norm(q)
        if q_norm > 1e-10:
            x_next[6:10] = q / q_norm

        # 로터 속도 클램핑
        x_next[13:17] = np.clip(x_next[13:17],
                                self.params['n_min'],
                                self.params['n_max'])
        return x_next

    def simulate(self, x0, controller, T, dt=None):
        """
        궤적 시뮬레이션.

        controller: callable(t, x) → u[4]
        """
        if dt is not None and abs(dt - self.dt) > 1e-12:
            self._build_integrator(dt)

        N = int(round(T / self.dt))
        ts = np.linspace(0.0, T, N + 1)
        xs = np.zeros((N + 1, self.nx))
        us = np.zeros((N, self.nu))
        xs[0] = x0

        for k in range(N):
            us[k] = controller(ts[k], xs[k])
            xs[k + 1] = self.step(xs[k], us[k])

        return ts, xs, us

    def evaluate_xdot(self, x, u):
        """적분 없이 x_dot만 계산 (디버깅용)."""
        return np.array(self.f(x, u)).flatten()

    @staticmethod
    def default_initial_state(speed=0.0, pitch_deg=0.0):
        """
        기본 초기 상태 (17D).

        speed     : 초기 전방 속도 [m/s]
        pitch_deg : 초기 피치각 [deg] (양수=기수 상방)
        """
        from scipy.spatial.transform import Rotation

        pitch_rad = np.radians(pitch_deg)
        rot = Rotation.from_euler('y', -pitch_rad)
        q = rot.as_quat()       # [qx, qy, qz, qw]
        R_mat = rot.as_matrix()
        v = R_mat @ np.array([speed, 0.0, 0.0])

        x0 = np.zeros(NX)
        x0[0:3]   = [0.0, 0.0, 0.0]
        x0[3:6]   = v
        x0[6:10]  = q
        x0[10:13] = [0.0, 0.0, 0.0]
        x0[13:17] = [0.0, 0.0, 0.0, 0.0]   # 로터 4개 정지

        return x0
