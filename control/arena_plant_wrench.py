"""플랜트 wrench 훅 — 팀원 벤더 파일(`models/team_light/control/dynamics.py`)은 안 건드리고,
그 심볼릭 xdot 위에 동체좌표 힘·모멘트 Δv̇ = R·F/m, Δω̇ = I⁻¹·M을 얹은 적분기를 새로 짓는다.

kj 결정(2026-09-27 밤, control-model-options-v2 병합 대신): "팀원 파일(models/team_light)은
수정하지 않는다. 대신 경기장 플랜트 래퍼에 '추가 힘·모멘트 훅'을 만든다. 훅은 적분기의 모든 단계
(RK 각 단계)에서 적용돼야 한다. m, I는 플랜트와 같은 파라미터에서 읽는다."

이 훅 하나로 표7(원고 v5.3) 세 항목을 담는다:
  - 로터 면내 항력       상태 의존(v_body·ω·n), 늘 켜져 있음 — params['rotor_inplane_drag_enabled']
  - 일정 외력(5 s)       고정, 시간창 — params['extra_force_body'/'extra_force_duration_s']
  - 외부 모멘트(5 s)     고정, 시간창 — params['extra_moment_body'/'extra_moment_duration_s']

무게중심 편차(표7 4행)는 이 훅으로 못 담는다 — 힘의 덧셈이 아니라 기존 힘들의 모멘트 팔(r_cp 등)이
바뀌는 것이라 구조가 다르다. 따로 구현해야 한다(결정 필요로 보고).

셋 다 기본은 꺼짐이다. 셋 다 꺼져 있으면 `build_plant`는 `AxialDronePlant`를 **그대로**
돌려준다(다르게 구현했는데 우연히 같은 게 아니라, 정말 같은 클래스·같은 코드다) — 기본 경로가
비트 단위로 바뀌지 않는다는 가장 강한 보장이다.

R(동체→관성)과 기저 xdot은 team_light의 `_quat_to_rotmat`·`_compute_xdot`을 그대로 불러와 쓴다
(재구현 아님) — 이 프로젝트에 쿼터니언 규약을 손으로 다시 맞추다 뒤집은 전력이 있어서다
(quaternion-scalar-last-w-zero 메모). team_light 쪽 규약이 바뀌면
`test_wrench_hook_matches_team_light_hover_convention`이 깨진다.
"""
import casadi as ca
import numpy as np

from models.team_light.control.dynamics import (
    AxialDronePlant, _compute_xdot, _quat_to_rotmat, NX, NU)

WRENCH_KEYS = ('rotor_inplane_drag_enabled', 'rotor_inplane_drag_coeff',
              'extra_force_body', 'extra_force_duration_s',
              'extra_moment_body', 'extra_moment_duration_s')


def _rotor_inplane_drag_delta(x_sym, params):
    """로터 면내 항력의 심볼릭 Δxdot(17). control-model-options-v2를 참고했으나 다시 짰다
    (팀원 파일을 안 건드리는 이 훅의 xdot 규약 위에서 새로 유도) — 그대로 옮기지 않았다.

    F_i = -C·|n_i|·V_plane,i,  V_plane,i = 로터 i 위치의 상대유속에서 추력축 성분을 뺀 것.
    합력·합모멘트를 Δv̇=R·F/m, Δω̇=I⁻¹·M으로 그대로 얹는다(무게중심 편차와 달리 모멘트 팔이
    안 바뀌므로 이 훅의 일반식과 같다).
    """
    from models.team_light.control.geometry import thrust_axis
    v_body, quat, omega, n_vec = x_sym[3:6], x_sym[6:10], x_sym[10:13], x_sym[13:17]
    axis = ca.DM(thrust_axis(params))
    coeff = float(params['rotor_inplane_drag_coeff'])
    F = ca.SX.zeros(3)
    M = ca.SX.zeros(3)
    for i in range(params['num_rotors']):
        ri = params['rotor_positions'][i]
        v_local = v_body + ca.cross(omega, ca.DM(ri))
        v_plane = v_local - axis*ca.dot(axis, v_local)
        f_i = -coeff*ca.fabs(n_vec[i])*v_plane
        F += f_i
        M += ca.cross(ca.DM(ri), f_i)
    return _wrench_to_xdot_delta(F, M, quat, params)


def _wrench_to_xdot_delta(F_body, M_body, quat, params):
    """동체좌표 힘·모멘트 하나를 Δxdot(17)로. Δv̇=R·F/m(3:6), Δω̇=I⁻¹·M(10:13), 나머지 0.
    Euler 방정식 Jω̇+ω×Jω=M은 M에 선형이라, 외부 모멘트를 더해도 ω×Jω 보정 항은 그대로다
    (M_total 자리에 ΔM만 더하면 된다 — kj가 준 Δω̇=I⁻¹M 그대로, 교차항 재유도 불필요)."""
    R = _quat_to_rotmat(quat)
    J_d = ca.vertcat(params['Ixx'], params['Iyy'], params['Izz'])
    dv = (R @ F_body)/params['mass']
    dw = M_body/J_d
    return ca.vertcat(ca.SX.zeros(3), dv, ca.SX.zeros(4), dw, ca.SX.zeros(4))


def reference_inplane_drag_coefficient(params, V_ref):
    """표7 5행 "동체 횡력의 1배" 계수 역산. V_ref 트림에서 로터 면내 항력 합력 크기가 동체
    공력의 면내(추력축 수직) 성분 크기와 같아지는 coeff. team_light의 find_trim·
    _body_aerodynamics를 그대로 불러와 쓴다(재구현 아님)."""
    from models.team_light.control.trim import find_trim
    from models.team_light.control.dynamics import _body_aerodynamics
    from models.team_light.control.geometry import thrust_axis
    trim = find_trim(params, V_ref)
    if not trim['converged']:
        raise ValueError(f'no converged trim at V={V_ref}')
    v_body, omega = trim['v_body'], np.zeros(3)
    axis = np.asarray(thrust_axis(params))
    # _body_aerodynamics는 심볼릭 함수라(light_aerodynamics가 ca.SX.zeros(3)로 시작한다) 숫자를
    # 그대로 넣어도 SX가 나온다 — AxialDronePlant.f처럼 ca.Function으로 감싸 수치로 뽑는다.
    v_sym, w_sym = ca.SX.sym('v', 3), ca.SX.sym('w', 3)
    f_aero = ca.Function('f_aero', [v_sym, w_sym], [_body_aerodynamics(v_sym, w_sym, params)[0]])
    F_aero = np.array(f_aero(v_body, omega)).flatten()
    lateral = np.linalg.norm(F_aero - axis*np.dot(axis, F_aero))

    n_vec = trim['control']
    scale = 0.0
    for i in range(params['num_rotors']):
        v_local = v_body + np.cross(omega, np.asarray(params['rotor_positions'][i]))
        v_plane = v_local - axis*np.dot(axis, v_local)
        scale += abs(n_vec[i])*np.linalg.norm(v_plane)
    if scale < 1e-9:
        raise ValueError(f'rotor in-plane relative velocity ~0 at V={V_ref} — pick another V_ref')
    return float(lateral/scale)


def hover_max_pitch_moment(params):
    """호버에서 낼 수 있는 최대 |M_y|(피치) — 표7 외부모멘트(25%·50%) 정의(kj 권고, 2026-09-27
    밤: "선행 기체 관성을 모르니 호버 최대 피치모멘트의 25%·50%로"). T_total=mass·g 유지,
    M_x=M_z=0(순수 피치)로 두고 로터별 추력 한계 안에서 최대화한다.

    `compute_allocation_matrix`는 곡선 추진모델에서 `static_reference=True`만 허용한다(선형
    k_T·n² 근사, "Variable-Q/T profiles have NO exact constant matrix" — team_light 자신의
    docstring). 이 함수는 그 근사를 그대로 받는다 — 정확한 한계가 아니라 표7 정의용 참고값이다.
    """
    from models.team_light.control.dynamics import compute_allocation_matrix
    _, TM_to_f = compute_allocation_matrix(params, static_reference=True)
    T_total = params['mass']*params['g']
    f_min, f_max = 0.0, params['k_T']*params['n_max']**2
    base = TM_to_f @ np.array([T_total, 0., 0., 0.])
    slope = TM_to_f @ np.array([0., 0., 1., 0.])       # d(로터별 추력)/d(My)
    lo_bounds, hi_bounds = [], []
    for b, s in zip(base, slope):
        if abs(s) < 1e-12:
            continue
        a, c = (f_min - b)/s, (f_max - b)/s
        lo_bounds.append(min(a, c))
        hi_bounds.append(max(a, c))
    my_lo, my_hi = max(lo_bounds), min(hi_bounds)
    if my_lo > my_hi:
        raise ValueError('no feasible pitch moment at this hover thrust')
    return float(min(-my_lo, my_hi))       # 대칭(보수적으로 더 작은 쪽)


def wrench_enabled(params):
    """params에 이 훅이 켤 것이 있는지 — 하나도 없으면 build_plant가 순정 AxialDronePlant를 준다."""
    return (bool(params.get('rotor_inplane_drag_enabled', False))
            or float(params.get('extra_force_duration_s', 0.0)) > 0.0
            or float(params.get('extra_moment_duration_s', 0.0)) > 0.0)


class _ExtraWrenchPlant:
    """`AxialDronePlant`와 같은 `step`/`evaluate_xdot`/`dt`/`params` 인터페이스. `build_plant`가
    무엇이든 켜져 있을 때만 만든다 — 직접 생성하지 말 것(꺼짐 판단을 안 거친다)."""

    def __init__(self, params, dt=0.001):
        self.params = params
        self.dt = float(dt)
        self.nx, self.nu = NX, NU

        drag_on = bool(params.get('rotor_inplane_drag_enabled', False))
        x_sym = ca.SX.sym('x', NX)
        u_sym = ca.SX.sym('u', NU)
        w_sym = ca.SX.sym('w', 3)
        F_ext_sym = ca.SX.sym('F_ext', 3)     # 늘 있는 파라미터. 안 쓰면 값이 정확히 0 벡터라
        M_ext_sym = ca.SX.sym('M_ext', 3)     # R@0=0, xdot+0=xdot — 그래프에 남아도 비트에 영향 없음
        drag_delta = _rotor_inplane_drag_delta(x_sym, params) if drag_on else None

        xdot_wind = _compute_xdot(x_sym, u_sym, params, w_sym)
        if drag_delta is not None:
            xdot_wind = xdot_wind + drag_delta
        xdot_wind = xdot_wind + _wrench_to_xdot_delta(F_ext_sym, M_ext_sym, x_sym[6:10], params)
        p_sim = ca.vertcat(u_sym, w_sym, F_ext_sym, M_ext_sym)
        self.integrator = ca.integrator('plant_wrench', 'rk', {'x': x_sym, 'p': p_sim, 'ode': xdot_wind},
                                        0.0, dt, {'number_of_finite_elements': 4})

        xdot_still = _compute_xdot(x_sym, u_sym, params)     # find_trim용 — 바람·펄스 없음(§평가 참고)
        if drag_delta is not None:
            xdot_still = xdot_still + drag_delta
        self.f = ca.Function('f_wrench', [x_sym, u_sym], [xdot_still])
        self._elapsed = 0.0
        self._force = np.asarray(params.get('extra_force_body', (0., 0., 0.)), dtype=float)
        self._force_duration = float(params.get('extra_force_duration_s', 0.0))
        self._moment = np.asarray(params.get('extra_moment_body', (0., 0., 0.)), dtype=float)
        self._moment_duration = float(params.get('extra_moment_duration_s', 0.0))

    def _pulse(self):
        """지금 스텝(elapsed → elapsed+dt) 동안 켜져 있는 외력·외부모멘트. RK 4단계 전체가 이
        dt 안에서 일어나므로 한 번 고른 값이 이번 step() 호출의 모든 하위단계에 그대로 쓰인다
        (CasADi 적분기 파라미터는 한 번의 step 호출 동안 상수라서 — u·w와 같은 방식)."""
        F = self._force if self._elapsed < self._force_duration - 1e-12 else np.zeros(3)
        M = self._moment if self._elapsed < self._moment_duration - 1e-12 else np.zeros(3)
        return F, M

    def step(self, x, u, w=None):
        if w is None:
            w = np.zeros(3)
        F, M = self._pulse()
        p = np.concatenate([u, w, F, M])
        xn = np.array(self.integrator(x0=x, p=p)['xf']).flatten()
        q = xn[6:10]
        qn = np.linalg.norm(q)
        if qn > 1e-10:
            xn[6:10] = q/qn
        xn[13:17] = np.clip(xn[13:17], self.params['n_min'], self.params['n_max'])
        self._elapsed += self.dt
        return xn

    def evaluate_xdot(self, x, u, w=None):
        # 트림 탐색(find_trim) 등에서 쓴다. 펄스가 아직 안 걸린 t=0 기준 xdot이다 — 로터 면내
        # 항력만 담고(상태 의존이라 트림 자체를 바꿀 수 있다), 펄스형 외력·모멘트는 안 담는다
        # (그건 트림에서가 아니라 시뮬레이션 중 켜지는 시간창 교란이다).
        return np.array(self.f(x, u)).flatten()

    @staticmethod
    def hover_state(params):
        return AxialDronePlant.hover_state(params)


def build_plant(params, dt=0.001):
    """경기장이 실제로 쓰는 생성자. params에 켤 것이 없으면 team_light `AxialDronePlant`를
    그대로 돌려준다 — 다른 구현이 우연히 같은 게 아니라 정말 같은 코드다."""
    if not wrench_enabled(params):
        return AxialDronePlant(params, dt=dt)
    return _ExtraWrenchPlant(params, dt=dt)
