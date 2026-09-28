"""플랜트 wrench 훅(`control/arena_plant_wrench.py`) — kj 결정(2026-09-27 밤)의 시험 항목 그대로.

  1) 꺼짐 = team_light AxialDronePlant를 정말 그대로 씀(타입 자체가 같다, 비트 동일).
  2) 켜짐(힘) = 정지 상태에서 Δv̇ = R·F/m 해석값과 맞음.
  3) 켜짐(모멘트) = Δω̇ = I⁻¹·M 해석값과 맞음.
  4) 팀원 규약 확인 — 호버에서 body +x 힘이 world +z 가속을 낸다(hover_quaternion 문서 그대로).
     팀원 쪽 규약이 바뀌면 이 시험이 깨진다.
  5) 펄스 시간창 — duration_s가 지나면 꺼진다.
  6) 로터 면내 항력 — 호버(v=ω=0)에서 정확히 0, 면내 속도가 있으면 반대 방향으로 작용.
  7) 항력 계수 보정 — 그 V_ref 트림에서 실제로 "동체 횡력 1배"가 나오는지.
"""
import numpy as np
import pytest

from control.arena_plant_wrench import (build_plant, wrench_enabled, _ExtraWrenchPlant,
                                        reference_inplane_drag_coefficient, hover_max_pitch_moment)
from models.team_light.control.dynamics import AxialDronePlant, _body_aerodynamics
from models.team_light.control.vehicle_params import vehicle_params
from models.team_light.control.geometry import hover_quaternion, thrust_axis

DT = 0.002


@pytest.fixture
def params():
    return dict(vehicle_params)


@pytest.fixture
def hover_state(params):
    return AxialDronePlant.hover_state(params)


def test_disabled_wrench_is_the_real_team_light_plant(params):
    plant = build_plant(params, dt=DT)
    assert type(plant) is AxialDronePlant
    assert not wrench_enabled(params)


@pytest.mark.parametrize('extra', [
    dict(rotor_inplane_drag_enabled=True, rotor_inplane_drag_coeff=0.01),
    dict(extra_force_body=(1.0, 0.0, 0.0), extra_force_duration_s=1.0),
    dict(extra_moment_body=(0.0, 0.5, 0.0), extra_moment_duration_s=1.0),
])
def test_enabled_wrench_builds_the_wrapper_not_the_real_plant(params, extra):
    plant = build_plant(dict(params, **extra), dt=DT)
    assert isinstance(plant, _ExtraWrenchPlant)
    assert wrench_enabled(dict(params, **extra))


def test_constant_force_gives_the_analytic_acceleration_at_a_static_state(params, hover_state):
    """정지 상태(v=ω=0)에서 힘만 켜고 한 스텝 — 켜짐과 꺼짐의 차분을 해석값과 댄다.
    다른 항(공력·로터·중력)은 두 실행에 똑같이 들어가 차분에서 지워진다."""
    F_body = np.array([2.0, -1.0, 0.5])
    on = build_plant(dict(params, extra_force_body=tuple(F_body), extra_force_duration_s=10.0), dt=DT)
    off = AxialDronePlant(params, dt=DT)
    u_trim = hover_state[13:17]           # 트림 로터 속도를 명령으로(정상상태 유지)
    x_on = on.step(hover_state.copy(), u_trim.copy())
    x_off = off.step(hover_state.copy(), u_trim.copy())
    from scipy.spatial.transform import Rotation
    R = Rotation.from_quat(hover_state[6:10]).as_matrix()
    dv_analytic = (R @ F_body)/params['mass']
    dv_numeric = (x_on[3:6] - x_off[3:6])/DT
    np.testing.assert_allclose(dv_numeric, dv_analytic, rtol=1e-4, atol=1e-8)
    # 각속도 채널까지 정확히 0은 아니다 — RK4 한 스텝 안에서 F가 바꾼 v가 중간단계의 공력
    # 모멘트(_body_aerodynamics가 v_body에 의존)에 되먹임되는 2차 결합이 실제로 있다(실측
    # ~1e-3 rad/s² 수준, dt=0.002s). 이 훅의 Δω̇=I⁻¹M 자체는 M에만 의존해 정확히 분리돼 있다
    # (그 주장은 아래 모멘트 시험이 한다) — 여기서는 그 결합의 크기가 요청한 F에 비해
    # 작은지만 본다(수 배 안).
    coupling = np.abs((x_on[10:13] - x_off[10:13])/DT)
    assert np.all(coupling < 0.01)


def test_constant_moment_gives_the_analytic_angular_acceleration_at_a_static_state(params, hover_state):
    M_body = np.array([0.3, -0.2, 0.1])
    on = build_plant(dict(params, extra_moment_body=tuple(M_body), extra_moment_duration_s=10.0), dt=DT)
    off = AxialDronePlant(params, dt=DT)
    u_trim = hover_state[13:17]
    x_on = on.step(hover_state.copy(), u_trim.copy())
    x_off = off.step(hover_state.copy(), u_trim.copy())
    J_d = np.array([params['Ixx'], params['Iyy'], params['Izz']])
    dw_analytic = M_body/J_d
    dw_numeric = (x_on[10:13] - x_off[10:13])/DT
    np.testing.assert_allclose(dw_numeric, dw_analytic, rtol=1e-4, atol=1e-8)
    # 위 힘 시험과 대칭인 2차 결합(자세가 M 때문에 회전 → 그 자세가 v̇의 R 곱셈에 되먹임).
    coupling = np.abs((x_on[3:6] - x_off[3:6])/DT)
    assert np.all(coupling < 0.01)


def test_body_x_force_accelerates_toward_world_z_at_hover(params, hover_state):
    """팀원 규약(hover_quaternion 문서: body +x → world +z) 확인. 이게 깨지면 이 훅의
    R 곱셈 방향이나 team_light의 규약 자체가 바뀐 것이다."""
    np.testing.assert_allclose(hover_state[6:10], hover_quaternion())
    on = build_plant(dict(params, extra_force_body=(10.0, 0.0, 0.0), extra_force_duration_s=10.0), dt=DT)
    off = AxialDronePlant(params, dt=DT)
    u_trim = hover_state[13:17]
    dv = (on.step(hover_state.copy(), u_trim.copy())[3:6]
          - off.step(hover_state.copy(), u_trim.copy())[3:6])/DT
    assert dv[2] > 0.5 and abs(dv[0]) < 1e-6 and abs(dv[1]) < 1e-6


def test_pulse_turns_off_after_its_duration(params, hover_state):
    duration_s = 3*DT
    plant = build_plant(dict(params, extra_force_body=(5.0, 0.0, 0.0), extra_force_duration_s=duration_s),
                        dt=DT)
    x = hover_state.copy()
    u_trim = hover_state[13:17]
    for k in range(6):
        assert (plant._elapsed < duration_s) == np.any(plant._pulse()[0] != 0.0)
        x = plant.step(x, u_trim.copy())


def test_rotor_inplane_drag_is_exactly_zero_at_hover(params, hover_state):
    """v_body=ω=0인 호버에서는 모든 로터의 면내 상대유속이 정확히 0이라, 항력도 정확히 0이다
    (트림을 안 건드린다는 근거)."""
    on = build_plant(dict(params, rotor_inplane_drag_enabled=True, rotor_inplane_drag_coeff=0.05), dt=DT)
    off = AxialDronePlant(params, dt=DT)
    u_trim = hover_state[13:17]
    x_on = on.evaluate_xdot(hover_state, u_trim)
    x_off = off.evaluate_xdot(hover_state, u_trim)
    np.testing.assert_allclose(x_on, x_off, atol=1e-12)


def test_rotor_inplane_drag_opposes_the_inplane_velocity(params, hover_state):
    """면내(body y-z, thrust_axis=x라서) 속도를 주면 항력이 그 반대 방향으로 작용해야 한다.

    x[3:6]은 관성(월드) 속도다(`_compute_xdot`가 `v_body = R.T @ vel`로 body 속도를 만든다) —
    호버 자세(R이 단위행렬이 아님)에서 그대로 [0,2,0]을 넣으면 body 속도가 아니게 된다(처음에
    이 실수로 시험이 거짓 실패했다). 단위 자세로 바꿔 월드=body가 되게 한다."""
    x = hover_state.copy()
    x[6:10] = np.array([0.0, 0.0, 0.0, 1.0])  # 단위 쿼터니언(scalar-last) — R = I
    x[3:6] = [0.0, 2.0, 0.0]                  # 단위 자세라 body y 속도와 같다(면내)
    on = build_plant(dict(params, rotor_inplane_drag_enabled=True, rotor_inplane_drag_coeff=0.05), dt=DT)
    off = AxialDronePlant(params, dt=DT)
    u_trim = hover_state[13:17]
    xd_on = on.evaluate_xdot(x, u_trim)
    xd_off = off.evaluate_xdot(x, u_trim)
    assert xd_on[4] < xd_off[4]                # v̇_y가 항력 때문에 더 작아진다(감속 방향)


def test_wrench_keys_documented_in_module_are_the_ones_read():
    """WRENCH_KEYS 목록과 실제로 읽는 키가 어긋나면(리팩터 실수 등) 여기서 잡는다."""
    import inspect
    from control import arena_plant_wrench as m
    source = inspect.getsource(m)
    for key in m.WRENCH_KEYS:
        assert f"'{key}'" in source, key


@pytest.mark.parametrize('V_ref', [20.0, 40.0, 60.0, 85.0])
def test_calibrated_drag_coefficient_gives_exactly_one_body_x_lateral_force(params, V_ref):
    """정의(표7 5행) 그대로: 그 계수로 켠 항력이 그 트림에서 동체 공력의 면내 성분과 크기가
    같아야 한다."""
    from models.team_light.control.trim import find_trim
    import casadi as ca

    coeff = reference_inplane_drag_coefficient(params, V_ref)
    assert coeff > 0
    trim = find_trim(params, V_ref)
    v_body, omega, n_vec = trim['v_body'], np.zeros(3), trim['control']
    axis = np.asarray(thrust_axis(params))

    v_sym, w_sym = ca.SX.sym('v', 3), ca.SX.sym('w', 3)
    f_aero = ca.Function('f_aero', [v_sym, w_sym], [_body_aerodynamics(v_sym, w_sym, params)[0]])
    F_aero = np.array(f_aero(v_body, omega)).flatten()
    lateral = np.linalg.norm(F_aero - axis*np.dot(axis, F_aero))

    drag_force = np.zeros(3)
    for i in range(params['num_rotors']):
        v_local = v_body + np.cross(omega, np.asarray(params['rotor_positions'][i]))
        v_plane = v_local - axis*np.dot(axis, v_local)
        drag_force += -coeff*abs(n_vec[i])*v_plane
    np.testing.assert_allclose(np.linalg.norm(drag_force), lateral, rtol=1e-9)


def test_hover_max_pitch_moment_saturates_a_rotor_and_stays_in_bounds(params):
    """그 My에서 로터별 추력을 다시 계산하면 전부 [0, f_max] 안이고 적어도 하나는 경계에
    닿아야 한다(이게 아니면 '최대'가 아니다)."""
    from models.team_light.control.dynamics import compute_allocation_matrix
    my = hover_max_pitch_moment(params)
    _, TM_to_f = compute_allocation_matrix(params, static_reference=True)
    T_total = params['mass']*params['g']
    f_max = params['k_T']*params['n_max']**2
    for sign in (+1.0, -1.0):
        f = TM_to_f @ np.array([T_total, 0.0, sign*my, 0.0])
        assert np.all(f >= -1e-9) and np.all(f <= f_max + 1e-9)
        assert np.any(np.abs(f) < 1e-6) or np.any(np.abs(f - f_max) < 1e-6)
