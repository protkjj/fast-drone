"""Virtual dynamics and two existing NMPC/INDI inner-loop connections.

For the validated nonlinear-constraint optimizer use ComparisonNMPC(virtual=True).
The old constant-Q/T VirtualNMPC and its legacy demo are not distributed.
"""
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation
from models.team_light.control.dynamics import (build_dynamics, _quat_to_rotmat, _quat_derivative,
    _body_aerodynamics, EPS, compute_allocation_matrix)
from models.team_light.control.geometry import (thrust_axis, control_effectiveness, rotor_speeds_for_thrust,
    rotor_thrusts, allocate_wrench)
from models.team_light.control.propeller_curve import uses_curve
NX_V=13
NU_V=4


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

    # Body +x thrust, consistent with the full plant.
    F_body = F_aero + ca.DM(thrust_axis(params)) * T_cmd

    p_dot = vel
    v_dot = ca.vertcat(0, 0, -params['g']) + (R @ F_body) / params['mass']
    q_dot = _quat_derivative(quat, omega)
    omega_dot = nu_omega    # INDI가 이걸 실현

    xdot = ca.vertcat(p_dot, v_dot, q_dot, omega_dot)
    f = ca.Function('f_virtual', [x, u], [xdot])
    return f, x, u


class ProperHybrid:
    """
    NMPC+INDI 분리형 비교 구조. 성능 우위는 시험 결과로 판단한다.

    NMPC → [T_cmd, ω̇_des]  (뭘 할지)
    INDI → [n1..n4]         (어떻게 할지, 유일한 실행자)

    실제 모터 명령은 INDI가 생성한다. 새 NMPC의 보조 로터 변수는
    정적 명령 실현 가능성만 확인하며 실제 모터 상태를 예측하지 않는다.
    """

    def __init__(self, virtual_nmpc, params, dt=0.001, f_cut=50.0):
        self.nmpc = virtual_nmpc
        self.p = params
        self.dt = dt
        self._tau = 1.0 / (2*np.pi*f_cut)     # LPF 시상수 (가변 dt에서 alpha 재계산용)
        self._alpha = dt / (dt + self._tau)
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._prev_t = None                   # 실제 Δt 측정용 (SITL 루프율 가변/100Hz미만)
        self._initialized = False
        _, self._TM_to_f = compute_allocation_matrix(params, static_reference=True)

    def reset(self):
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._prev_t = None
        self._initialized = False
        # Outer optimizer reset (timing and warm start).
        if hasattr(self.nmpc, 'reset'):
            self.nmpc.reset()

    def __call__(self, t, x):
        # 1. NMPC: 가상 명령
        vc = self.nmpc(t, x)
        T_cmd, omega_dot_des = vc[0], vc[1:4]

        omega = x[10:13]
        n_actual = x[13:17]
        fallback_vb = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]

        # 측정 NaN 가드 (리뷰 3d): _omega_dot_filt는 자기참조 LPF라 NaN이
        # 한 번 들어가면 영구 고착 — 갱신을 건너뛰고 모델 기반 폴백으로.
        if not (np.all(np.isfinite(omega)) and np.all(np.isfinite(n_actual))):
            return self._fallback(T_cmd, omega_dot_des, fallback_vb)

        if not self._initialized:
            self._omega_prev = omega.copy()
            self._prev_t = t
            self._initialized = True
            if uses_curve(self.p):
                return np.clip(n_actual, self.p['n_min'], self.p['n_max'])
            return self._fallback(T_cmd, omega_dot_des, fallback_vb)

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
        # Same propeller map as the plant, including independent Ct and Cp.
        T_meas = float(np.sum(rotor_thrusts(self.p, n_actual, v_body)))

        dv = np.array([T_cmd - T_meas,
                       omega_dot_des[0] - self._omega_dot_filt[0],
                       omega_dot_des[1] - self._omega_dot_filt[1],
                       omega_dot_des[2] - self._omega_dot_filt[2]])

        G = self._compute_G(n_actual, v_body)
        try:
            dn = np.linalg.solve(G, dv)
        except np.linalg.LinAlgError:
            return self._fallback(T_cmd, omega_dot_des, v_body)

        return np.clip(n_actual + dn, self.p['n_min'], self.p['n_max'])

    def _compute_G(self, n, v_body=None):
        return compute_control_effectiveness(self.p, n, v_body)

    def _fallback(self, T_cmd, omega_dot_des, v_body=None):
        J = np.diag([self.p['Ixx'], self.p['Iyy'], self.p['Izz']])
        TM = np.array([T_cmd, *(J @ omega_dot_des)])
        if uses_curve(self.p):
            return allocate_wrench(self.p, TM, v_body)
        f_ind = self._TM_to_f @ TM
        return rotor_speeds_for_thrust(self.p, f_ind, v_body)


def compute_control_effectiveness(params, n_actual, v_body=None):
    """Shared rocket-layout derivative, including advance-ratio cutoff."""
    return control_effectiveness(params, n_actual, v_body)


class NaiveHybrid:
    """모터속도 명령 위에 증분 보정을 더하는 비교 구조; 간섭 여부는 검증 대상."""

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
        v_body = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]
        G = compute_control_effectiveness(self.p, n_actual, v_body)
        dv = np.array([0.0, d[0], d[1], d[2]])
        try:
            dn = np.linalg.solve(G, dv)
        except np.linalg.LinAlgError:
            return u_nmpc
        return np.clip(u_nmpc + dn, self.p['n_min'], self.p['n_max'])
