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

try:
    from models.team_light.control.propeller_curve import MODEL as PROP_CURVE_MODEL
except ImportError:
    PROP_CURVE_MODEL = 'apc_30k_pchip_v2'   # models/team_light 미병합 환경 대비 폴백

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
    # bulnabi 포팅: 음의 축방향 유입(u_b<0)에서 교차류 감쇠가 추력으로 뒤집히지
    # 않도록 정칙 유지(비행교정 범위 밖은 그대로 연속 이어감).
    F_N_fac = 0.5 * rho * S * (p['C_Na'] * ca.fabs(u_b) + p['C_dc'] * V_cf)
    Fy = -F_N_fac * v_b
    Fz = -F_N_fac * w_b

    # 축력 (항력, 전방비행 시 -x 방향)
    C_A = p['C_A0'] + p['C_Aa2'] * (v_b**2 + w_b**2) / V_sq
    Fx = -q_bar * S * C_A * ca.sign(u_b)

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
# 로터 (thrust_axis 가변 쿼드콥터)
# ════════════════════════════════════════════════════

def _rotor_forces_moments(v_body, n_vec, omega, p):
    """4로터 추력·모멘트 (RotorPy 방식 + 전진비).

    추력축은 p['thrust_axis'] 로 고른다 (bulnabi의 동일 함수와 동일 패턴,
    control/RESEARCH_PARITY 계열 정합성 작업에서 포팅).

      'z' (기본, 역호환) — 추력이 동체 -z. 어뢰형 동체를 수평으로 달고 일반
          쿼드처럼 숙여서 나는 배치. 로터가 동체 xy 평면에 있다.
          V_axial = -w_b.

      'x' — 추력이 동체 +x(기수). 로터가 **동체축에 수직인 yz 평면**에 놓여
          기수축을 둘러싼다. 호버에서 기수가 위를 보고, 순항은 눕혀서 난다.
          추진까지 축대칭인 로켓형 배치. V_axial = u_b.

    두 배치는 로터 기하와 추력 방향만 다르고 나머지 항(전진비·반토크·자이로)의
    구조는 같다. 부호를 손으로 옮기다 틀리기 쉬운 자리라 한 함수 안에 나란히
    두고 축만 갈랐다.
    """
    axis = p.get('thrust_axis', 'z')

    # 로터 추력축 방향 유입속도. 이 축이 바뀌면 전진비도 같이 바뀐다.
    V_axial = ca.fmax(v_body[0], 0.0) if axis == 'x' else ca.fmax(-v_body[2], 0.0)

    pos = p['rotor_positions']     # (4,3)
    dirs = p['rotor_directions']   # (4,)

    F_tot = ca.SX.zeros(3)
    M_tot = ca.SX.zeros(3)
    h_net = 0.0

    use_curve = p.get('propulsion_model') == PROP_CURVE_MODEL
    if use_curve:
        from models.team_light.control.propeller_curve import symbolic_force_torque

    for i in range(p['num_rotors']):
        ni = n_vec[i]
        di = float(dirs[i])
        ri = pos[i]

        if use_curve:
            # kj 지적(2026-09-25 밤): 선형 fac=1-J/J_max 모델이 APC
            # 5.5x6.5E 실제 CT(J)를 트림 회전수에서 0.43~0.69배로 크게
            # 과소평가한다(J_max=1.35인데 실제 곡선은 J≈0.8까지 거의
            # 평평함). J<0.6에서 값이 확 꺾여 트림·짧은시험 포화의 근본
            # 원인 후보. propulsion_model이 이 값이면(팀원 기체) 팀원
            # 자신의 매끄러운 PCHIP 곡선을 그대로 쓴다 — 재구현 안 함,
            # 우리 자신의 기체(이 값이 없는 경우)는 기존 선형모델 그대로.
            Ti, Qi = symbolic_force_torque(p, ni, V_axial)
        else:
            n_rps = ni / (2.0 * ca.pi)
            J = V_axial / (n_rps * p['D_prop'] + EPS)
            fac = ca.fmax(1.0 - J / p['J_max'], 0.0)
            Ti = p['k_T'] * ni**2 * fac
            Qi = p['k_Q'] * ni**2 * fac

        if axis == 'x':
            # 추력 [T, 0, 0].  모멘트 r × [T,0,0] = [0, r_z·T, -r_y·T]
            F_tot += ca.vertcat(Ti, 0.0, 0.0)
            M_tot += ca.vertcat(0.0, ri[2] * Ti, -ri[1] * Ti)
            M_tot += ca.vertcat(di * Qi, 0.0, 0.0)      # 반토크는 스핀축(=x) 둘레
        else:
            # 추력 [0, 0, -T] (body -z = 위).  모멘트 r × [0,0,-T]
            F_tot += ca.vertcat(0.0, 0.0, -Ti)
            M_tot += ca.vertcat(ri[1] * (-Ti), -ri[0] * (-Ti), 0.0)
            M_tot += ca.vertcat(0.0, 0.0, di * Qi)

        # ★ 로터 각운동량의 부호. 반작용 토크를 위에서 `+d_i*Q_i` 로 썼다는 것은
        #   d_i = -sigma_i (sigma_i = 로터가 실제로 도는 방향) 라는 뜻이다:
        #     공기가 로터를 -sigma*Q 로 막고, 모터가 +sigma*Q 를 공급하며,
        #     그 반작용으로 동체가 -sigma*Q 를 받는다.  -sigma*Q = +d*Q  ->  sigma = -d.
        #   따라서 h = I_r * sum(sigma_i * n_i) = **-** I_r * sum(d_i * n_i) 다.
        #   주의: k_Q=0 으로 끄는 각운동량 보존 시험은 이 부호를 판별하지 못한다
        #   (반토크와 h 가 같이 뒤집혀 똑같이 통과한다). 근거는 보존법칙이 아니라
        #   위의 반토크 규약이다.
        h_net -= p['I_rotor'] * ni * di

    # 자이로: tau = -omega x h,  h 는 로터 스핀축 방향 (위에서 부호를 이미 반영)
    if axis == 'x':
        # h = [h,0,0];  omega x h = [0, w_z·h, -w_y·h];  tau = -그것
        M_tot += ca.vertcat(0.0, -omega[2] * h_net, omega[1] * h_net)
    else:
        # h = [0,0,h];  omega x h = [w_y·h, -w_x·h, 0];  tau = -그것
        M_tot += ca.vertcat(-omega[1] * h_net, omega[0] * h_net, 0.0)

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


def compute_allocation_matrix(params, gamma=None):
    """
    [T1..T4] ↔ [T_total, Mx, My, Mz] 변환 행렬.

    추력 방향 e_thrust(축에 따라 [1,0,0] 또는 [0,0,-1]) 기준 교차곱으로 자동 생성.

    gamma : array(4) | None
        로터별 반토크/추력 비 Q_i/T_i. None(기본값)이면 기존처럼 상수
        k_Q/k_T 하나를 쓴다(비트 단위로 같은 행렬). 팀원 곡선 기체에서는
        이 비가 전진비 J에 따라 변한다 — J≈1.2에서 정지값의 약 2.5배라,
        상수로 두면 추력축 둘레(스핀축) 모멘트를 21~68% 과소평가한다
        (2026-09-25 경로 감사). 그때 reaction_torque_ratio()의 값을 넘긴다.
    """
    axis = params.get('thrust_axis', 'z')
    e_thrust = np.array([1, 0, 0]) if axis == 'x' else np.array([0, 0, -1])
    k = params['k_Q'] / params['k_T']
    pos = params['rotor_positions']
    dirs = params['rotor_directions']
    nr = params['num_rotors']
    ks = np.full(nr, k) if gamma is None else np.asarray(gamma, dtype=float)

    A = np.zeros((4, nr))
    for i in range(nr):
        A[0, i] = 1.0                                  # T_total
        m = np.cross(pos[i], e_thrust)                  # r × e_thrust
        k = ks[i]
        # 행 순서는 **축과 무관하게** [T_total, Mx, My, Mz] 로 고정한다.
        # 축마다 순서가 달라지면 쓰는 쪽이 조용히 어긋난다. 반토크는 스핀축
        # (추력축) 둘레에 걸린다.
        if axis == 'x':
            A[1, i] = dirs[i] * k                       # Mx (반토크, 스핀축 = x)
            A[2, i] = m[1]                              # My
            A[3, i] = m[2]                              # Mz
        else:
            A[1, i] = m[0]                              # Mx
            A[2, i] = m[1]                              # My
            A[3, i] = dirs[i] * k                       # Mz (반토크, 스핀축 = z)

    return A, np.linalg.inv(A)


# ════════════════════════════════════════════════════
# 로터 추진 — 수치 헬퍼 (곡선 기체와 선형 기체 공용)
# ════════════════════════════════════════════════════
# 팀원 기체(propulsion_model == PROP_CURVE_MODEL)는 추력·반토크가 APC Ct(J)·Cp(J)
# 곡선을 따른다. `_rotor_forces_moments`(심볼릭)와 같은 곡선을 수치 계산에서도
# 쓰도록 한 곳에 모았다 — 제어기마다 따로 적으면 한 군데만 선형식으로 남아도
# 알아채기 어렵다(2026-09-25 경로 감사에서 CPID·F13이 실제로 그랬다).

def uses_prop_curve(params):
    """팀원 APC 곡선 추진 모델을 쓰는 기체인가 — 곡선 분기의 단일 조건."""
    return params.get('propulsion_model') == PROP_CURVE_MODEL


def axial_airspeed(params, x):
    """상태 x(17)에서 로터 축방향 유입속도(추력축 방향 동체속도, 음수는 0).

    바람은 모른다 — 제어기는 관성속도만 안다(INDI의 T_meas와 같은 가정).
    """
    R = np.array(_quat_to_rotmat_np(x[6:10]))
    v_body = R.T @ np.asarray(x[3:6], dtype=float)
    if params.get('thrust_axis', 'z') == 'x':
        return max(float(v_body[0]), 0.0)
    return max(float(-v_body[2]), 0.0)


def _quat_to_rotmat_np(q):
    x, y, z, w = (float(c) for c in q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def rotor_thrust_torque(params, n, v_axial):
    """로터별 추력 T_i[N]과 반토크 크기 Q_i[N·m] (수치, 배열).

    곡선 기체는 팀원 곡선(`propeller_curve.values_and_derivatives`) 그대로,
    그 밖에는 기존 선형 fac 모델 — `_rotor_forces_moments`와 같은 분기다.
    """
    n = np.maximum(np.asarray(n, dtype=float), 0.0)
    va = max(float(v_axial), 0.0)
    if uses_prop_curve(params):
        from models.team_light.control.propeller_curve import values_and_derivatives
        T, Q, _, _ = values_and_derivatives(params, n, va)
        return np.asarray(T, dtype=float), np.asarray(Q, dtype=float)
    n_rps = n/(2.0*np.pi)
    J = va/(n_rps*params['D_prop'] + EPS)
    fac = np.maximum(1.0 - J/params['J_max'], 0.0)
    return params['k_T']*n**2*fac, params['k_Q']*n**2*fac


def reaction_torque_ratio(params, n, v_axial):
    """로터별 반토크/추력 비 γ_i = Q_i/T_i.

    선형 모델에선 fac가 약분돼 늘 k_Q/k_T(상수)다. 곡선 기체에선
    Q/T = (Cp/Ct)·D/(2π)가 전진비 J에 따라 변한다(J=0 → 0.0142, J≈1.24 →
    약 2.9배). 추력이 거의 0인 로터는 0/0을 피해 정지값을 쓴다.
    """
    T, Q = rotor_thrust_torque(params, n, v_axial)
    static = params['k_Q']/params['k_T']
    return np.where(T > 1e-9, Q/np.maximum(T, 1e-12), static)


_J_ZERO_CACHE = {}


def _positive_thrust_j_limit(params):
    """Ct 곡선의 영점 J(추력이 0이 되는 전진비). 팀원 함수는 부를 때마다
    brentq를 다시 푸므로 곡선 매듭(knots)별로 한 번만 계산해 둔다."""
    from models.team_light.control.propeller_curve import positive_thrust_j_limit
    key = tuple(tuple(row) for row in params['prop_curve']['knots'])
    if key not in _J_ZERO_CACHE:
        _J_ZERO_CACHE[key] = positive_thrust_j_limit(params)
    return _J_ZERO_CACHE[key]


def rotor_speed_for_thrust(params, thrust, v_axial):
    """추력 thrust[N]를 내는 로터 회전수 n[rad/s] (역산, 스칼라).

    곡선 기체: T(n) = ρ·Ct(J(n))·n_rps²·D⁴ 를 n에 대해 brentq로 푼다.
      Ct가 J에 대해 비증가라 T(n)은 n에 대해 단조증가다 — 브래킷 하나로 충분.
      브래킷 아래끝은 영추력 회전수(J = Ct의 영점): 그보다 느리면 추력이 0이다.
      요구 ≤ 0 이면 n_min, T(n_max)로도 모자라면 n_max(포화).
    그 밖의 기체: 기존 정지 역산 sqrt(T/k_T) 그대로(역호환, 비트 동일).

    왜 필요한가 — 정지 역산은 J≈0.4(20 m/s)까지는 0.4% 오차지만, 30 m/s에서
    추력을 53% 적게, 35 m/s 이상에선 명령 회전수가 영추력점 아래로 떨어져
    **추력 0**을 만든다(2026-09-25 경로 감사, kj 지적).
    """
    thrust = float(thrust)
    if not uses_prop_curve(params):
        return float(np.sqrt(max(thrust, 0.0)/params['k_T']))
    from scipy.optimize import brentq
    from models.team_light.control.propeller_curve import coefficients
    n_min, n_max = float(params['n_min']), float(params['n_max'])
    if thrust <= 0.0:
        return n_min
    va = max(float(v_axial), 0.0)
    D, rho = params['D_prop'], params['rho']

    def excess(n):
        # rotor_thrust_torque와 같은 식(팀원 곡선)이되 추력만 — 역산 안쪽이라
        # 토크·미분까지 매번 계산할 필요가 없다.
        n_rps = n/(2.0*np.pi)
        ct, _ = coefficients(params, va/(n_rps*D + 1e-8))
        return float(rho*n_rps**2*D**4*ct) - thrust

    if excess(n_max) <= 0.0:
        return n_max
    n_zero = 2.0*np.pi*va/(D*_positive_thrust_j_limit(params))
    lo = max(n_min, n_zero)
    if excess(lo) >= 0.0:
        return lo
    return float(brentq(excess, lo, n_max, xtol=1e-9, rtol=1e-12, maxiter=200))


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
        """추력축을 관성 +z로 향하게 한 정지 호버 상태 (scalar-last q)."""
        n_hov = np.sqrt(params['mass'] * params['g'] / (4 * params['k_T']))
        x0 = np.zeros(NX)
        if params.get('thrust_axis', 'z') == 'x':
            # Ry(-pi/2) maps body +x to inertial +z.
            x0[6:10] = [0.0, -np.sqrt(0.5), 0.0, np.sqrt(0.5)]
        else:
            x0[6] = 1.0                    # Rx(pi) maps body -z to inertial +z.
        x0[13:17] = n_hov                   # 호버 로터 속도
        return x0
