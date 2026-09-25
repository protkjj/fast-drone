"""고충실도(표 기반) 플랜트 — control/dynamics.py 는 손대지 않는다.

`control/dynamics.py`의 단순화 모델(추력 k_T·n²·(1-J/J_max), 항력 상수 C_A0)은
NMPC 예측·GSLQR 선형화·INDI 효과행렬·CPID 배분이 전부 공유한다. 이 파일은 그
공유 모델을 절대 건드리지 않는다 — 대신 같은 입출력 형태(17상태, 4입력)를
갖되 추력·항력을 실제 CT(J)/CP(J)/CD0(V) **표**로 계산하는 별도 플랜트를 추가한다.

왜 별도 경로인가 (2026-09-25, 검토자 지적 + kj 결정):
  1. 기존 z축 비트회귀가 안 깨진다 — control/dynamics.py 를 고치면 J>0.3~0.5
     인 모든 조건에서 값이 달라져 지금까지의 회귀 시험 전체가 무효가 된다.
  2. 형상과 무관하다 — 표 데이터(params['prop_table'], params['drag_table'])
     만 갈아 끼우면 새 형상 모델에도 그대로 쓴다. 형상팀을 기다릴 이유가 없다.
  3. **플랜트≠제어기 모델을 아키텍처 변경 없이 얻는다.** 제어기들은 여전히
     control/dynamics.py 의 단순화 모델을 쓰므로, 이 파일로 지은 플랜트에
     제어기를 그대로 물리면 명목 조건에서도 자연스러운 구조적 모델 오차가
     생긴다 — research 의 기존 원칙(명목=일치, 불일치는 별도 섭동 축)에서
     한 걸음 나아간 것이지만, 코드는 플랜트 클래스를 바꿔 끼우는 것뿐이다.
     `AxialDronePlant`(일치 모드)와 `AxialDronePlantHighFidelity`(고충실도
     모드) 중 어느 쪽을 결과로 채택할지는 kj 결정 — 둘 다 낼 수 있다.

`params`에 `prop_table`(J, CT, CP 배열)과 `drag_table`(speed_mps, CD0 배열)가
있어야 한다 — `control.vehicle_params.load_selected_params()`가 채워 둔다.
이 표가 없는 프로파일(예: 8kg 탐색용 `vehicle_params`/`rocket_params`)에는
쓸 수 없다 — `AxialDronePlantHighFidelity.__init__`이 명확한 에러를 낸다.
"""
from functools import lru_cache

import casadi as ca
import numpy as np

from control.dynamics import (EPS, NU, NX, _quat_derivative, _quat_to_rotmat,
                              compute_allocation_matrix)


@lru_cache(maxsize=32)
def _interpolator(grid, values):
    """research/model.py 의 `table()`과 같은 캐시된 선형보간 플러그인."""
    return ca.interpolant('lookup', 'linear', [list(grid)], list(values))


def _table(value, grid, values):
    lo, hi = grid[0], grid[-1]
    clipped = ca.fmin(ca.fmax(value, lo), hi)
    return _interpolator(tuple(grid), tuple(values))(clipped)


def _require_tables(params):
    for key, needed in (('prop_table', ('J', 'CT', 'CP')),
                        ('drag_table', ('speed_mps', 'CD0'))):
        if key not in params:
            raise ValueError(
                f"AxialDronePlantHighFidelity 에는 params['{key}']가 필요하다. "
                f"load_selected_params() 가 채워 둔다 — vehicle_params/"
                f"rocket_params(8kg 탐색용, 표 없음)에는 못 쓴다.")
        missing = [k for k in needed if k not in params[key]]
        if missing:
            raise ValueError(f"params['{key}']에 {missing}이 없다.")


def _body_aerodynamics_table(v_body, omega, p):
    """`control.dynamics._body_aerodynamics`와 같은 구조, 축력만 표 기반.

    법선력(C_Na, C_dc)·감쇠는 단순화 모델과 동일하게 둔다 — 검토자 지적은
    축력(C_A0)과 로터 추력·토크(J_max)에 한정됐다. 법선력까지 표로 바꾸는
    것은 이번 범위 밖이며, 소스 JSON에 법선력 표가 따로 없다(CN_alpha는
    상수 하나).
    """
    u_b, v_b, w_b = v_body[0], v_body[1], v_body[2]
    rho, S, d = p['rho'], p['S_ref'], p['d_ref']

    V_sq = u_b**2 + v_b**2 + w_b**2 + EPS
    V = ca.sqrt(V_sq)
    V_cf = ca.sqrt(v_b**2 + w_b**2 + EPS)
    q_bar = 0.5 * rho * V_sq

    F_N_fac = 0.5 * rho * S * (p['C_Na'] * ca.fabs(u_b) + p['C_dc'] * V_cf)
    Fy = -F_N_fac * v_b
    Fz = -F_N_fac * w_b

    # 축력만 표 기반 CD0(V) — C_A0 상수(0~20 m/s 평균) 대신 전 구간 실측값.
    dt = p['drag_table']
    C_A = _table(V, dt['speed_mps'], dt['CD0'])
    Fx = -q_bar * S * C_A * ca.sign(u_b)

    F_aero = ca.vertcat(Fx, Fy, Fz)

    xcp = p['x_cp']
    M_static = ca.vertcat(0.0, -xcp * Fz, xcp * Fy)
    df = 0.25 * rho * V * S * d**2
    M_damp = df * ca.vertcat(p['C_lp']*omega[0], p['C_mq']*omega[1], p['C_mq']*omega[2])
    return F_aero, M_static + M_damp


def _rotor_forces_moments_table(v_body, n_vec, omega, p):
    """`control.dynamics._rotor_forces_moments`와 같은 구조, 추력·토크만 표 기반.

    research/model.py의 `propellers()`와 같은 식(모터링 사분면만 재현, 그
    밖은 추력 0 으로 수동 연장): ct=max(CT(J),0), cp=max(CP(J), j·ct+.005).
    cp 하한은 (연구 원본과 동일) 순수 항력만 있어도 유도전력 이상은 필요하다는
    물리적 제약이다 — 표가 노이즈로 이보다 낮은 값을 줄 때의 안전장치.
    """
    axis = p.get('thrust_axis', 'z')
    V_axial = ca.fmax(v_body[0], 0.0) if axis == 'x' else ca.fmax(-v_body[2], 0.0)

    pos = p['rotor_positions']
    dirs = p['rotor_directions']
    pt = p['prop_table']
    D = p['D_prop']
    rho = p['rho']

    F_tot = ca.SX.zeros(3)
    M_tot = ca.SX.zeros(3)
    h_net = 0.0

    for i in range(p['num_rotors']):
        ni = n_vec[i]
        di = float(dirs[i])
        ri = pos[i]

        rev = ca.fmax(ni, 0.0) / (2.0*ca.pi)
        J = V_axial / (rev*D + EPS)
        ct_raw = _table(J, pt['J'], pt['CT'])
        cp_raw = _table(J, pt['J'], pt['CP'])
        ct = ca.fmax(ct_raw, 0.0)
        cp = ca.fmax(cp_raw, J*ct + .005)

        Ti = ct * rho * rev**2 * D**4
        Qi = cp * rho * rev**2 * D**5 / (2.0*ca.pi)

        if axis == 'x':
            F_tot += ca.vertcat(Ti, 0.0, 0.0)
            M_tot += ca.vertcat(0.0, ri[2]*Ti, -ri[1]*Ti)
            M_tot += ca.vertcat(di*Qi, 0.0, 0.0)
        else:
            F_tot += ca.vertcat(0.0, 0.0, -Ti)
            M_tot += ca.vertcat(ri[1]*(-Ti), -ri[0]*(-Ti), 0.0)
            M_tot += ca.vertcat(0.0, 0.0, di*Qi)

        # 부호 규약은 control.dynamics._rotor_forces_moments 와 동일(그쪽 주석 참고).
        h_net -= p['I_rotor'] * ni * di

    if axis == 'x':
        M_tot += ca.vertcat(0.0, -omega[2]*h_net, omega[1]*h_net)
    else:
        M_tot += ca.vertcat(-omega[1]*h_net, omega[0]*h_net, 0.0)

    return F_tot, M_tot


def _compute_xdot_table(x, u, p, w=None):
    pos, vel, quat = x[0:3], x[3:6], x[6:10]
    omega, n_vec = x[10:13], x[13:17]
    n_cmd = u[0:4]

    R = _quat_to_rotmat(quat)
    v_body = R.T @ (vel - w) if w is not None else R.T @ vel

    F_a, M_a = _body_aerodynamics_table(v_body, omega, p)
    F_r, M_r = _rotor_forces_moments_table(v_body, n_vec, omega, p)
    F_body, M_body = F_a + F_r, M_a + M_r

    p_dot = vel
    v_dot = ca.vertcat(0, 0, -p['g']) + (R @ F_body) / p['mass']
    q_dot = _quat_derivative(quat, omega)

    J_d = ca.vertcat(p['Ixx'], p['Iyy'], p['Izz'])
    w_dot = (1.0/J_d) * (M_body - ca.cross(omega, J_d*omega))
    n_dot = (n_cmd - n_vec) / p['tau_m']
    return ca.vertcat(p_dot, v_dot, q_dot, w_dot, n_dot)


def build_dynamics_table(params):
    """CasADi Function f(x[17], u[4]) → x_dot[17], 표 기반 추력·항력."""
    _require_tables(params)
    x = ca.SX.sym('x', NX)
    u = ca.SX.sym('u', NU)
    xd = _compute_xdot_table(x, u, params)
    f = ca.Function('f_dynamics_hifi', [x, u], [xd], ['x', 'u'], ['x_dot'])
    return f, x, u


class AxialDronePlantHighFidelity:
    """`control.dynamics.AxialDronePlant`와 동일 인터페이스, 표 기반 물리.

    제어기(트림·NMPC·GSLQR·INDI)는 여전히 control/dynamics.py 의 단순화
    모델을 쓴다 — 이 클래스는 시뮬레이션 **플랜트**로만 대신 꽂는다:

        plant = AxialDronePlantHighFidelity(selected_params, dt=0.001)
        controller = ProperHybrid(VirtualNMPC(selected_params, ...), selected_params, ...)
        ts, xs, us = plant.simulate(x0, controller, T)

    제어기 생성자에는 그대로 `selected_params`(단순화 모델)를 넘긴다 — 플랜트
    클래스만 바꾸는 것이 "제어기는 단순 모델만 알고, 실제로는 표 기반 고충실도
    기체를 난다"는 의도적 불일치를 만드는 전부다.
    """

    def __init__(self, params, dt=0.001):
        _require_tables(params)
        self.params = params
        self.dt = dt
        self.nx, self.nu = NX, NU

        self.f, self.x_sym, self.u_sym = build_dynamics_table(params)

        self.w_sym = ca.SX.sym('w', 3)
        xd_w = _compute_xdot_table(self.x_sym, self.u_sym, params, self.w_sym)
        self.f_wind = ca.Function('f_wind_hifi',
                                  [self.x_sym, self.u_sym, self.w_sym], [xd_w])
        p_sim = ca.vertcat(self.u_sym, self.w_sym)
        dae = {'x': self.x_sym, 'p': p_sim, 'ode': xd_w}
        self.integrator = ca.integrator(
            'plant_hifi', 'rk', dae, 0.0, dt, {'number_of_finite_elements': 4})

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
