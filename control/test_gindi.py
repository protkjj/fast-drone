"""GINDI(표5 확장 비교군) 구조 검증.

여기서 고정하는 것은 **구조**다 — 인터페이스, 좌표·부호 규약, 식(18) 입력
상자, 적분기 리셋. 추종 성능은 고정하지 않는다. 게인이 아직 control에서
§5.3 프로토콜로 튜닝되지 않았기 때문이다(results/GINDI_STATUS.md 참조).

성능 수치를 테스트에 박으면 나중에 정식 튜닝이 그 숫자를 바꿀 때 "테스트가
깨졌다"가 되어, 튜닝 결과를 테스트에 맞추려는 압력이 생긴다. §5.3이 금지하는
바로 그 방향이다.
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from control.gindi import ALPHA_LIMIT, GeometricGuidance, make_gindi
from control.trim import find_trim
from control.vehicle_params import load_selected_params, vehicle_params

SELECTED = load_selected_params()


def _trim_state(params, V, z=50.0):
    trim = find_trim(params, V, quiet=True)
    assert trim['converged'], f"트림 실패 V={V}"
    x = np.zeros(17)
    x[2] = z
    x[3:6] = [V, 0.0, 0.0]
    x[6:10] = trim['state'][6:10]
    x[13:17] = trim['state'][13:17]
    return x


# ── 인터페이스 ────────────────────────────────────────────────────────

def test_guidance_matches_the_virtual_nmpc_interface():
    """ProperHybrid가 상위 제어기에 요구하는 것을 모두 갖췄는지.

    이게 GINDI가 '공통 INDI'를 쓰는 방식이다 — 하위 루프를 새로 쓰지 않고
    V13과 같은 ProperHybrid에 가상입력을 넣는다.
    """
    g = GeometricGuidance(vehicle_params, v_ref=[0, 0, 0], z_ref=50.)
    assert callable(g)
    assert hasattr(g, 'set_prev_input') and hasattr(g, 'reset')
    u = g(0.0, _trim_state(vehicle_params, 0.0))
    assert u.shape == (4,)
    g.set_prev_input(np.zeros(4))        # 받기만 하고 무시해도 예외 없이


def test_make_gindi_wraps_the_shared_indi():
    from control.hybrid_comparison import ProperHybrid
    ctrl = make_gindi(SELECTED, v_ref=[8, 0, 0], z_ref=50.)
    assert isinstance(ctrl, ProperHybrid)
    assert isinstance(ctrl.nmpc, GeometricGuidance)
    # 표6 플래그가 그대로 전달되는지 — 비교군 간 하위 루프를 맞추려면 필요
    tuned = make_gindi(SELECTED, v_ref=[8, 0, 0], z_ref=50.,
                       alloc_mode='A1', time_align='S1')
    assert tuned.alloc_mode == 'A1' and tuned.time_align == 'S1'


# ── 좌표·부호 규약 ────────────────────────────────────────────────────

@pytest.mark.parametrize('params,V,label', [
    (vehicle_params, 0.0, 'vehicle(z) 호버'),
    (vehicle_params, 12.0, 'vehicle(z) 전진'),
    (SELECTED, 0.0, 'selected(x) 호버'),
    (SELECTED, 8.0, 'selected(x) 전진'),
])
def test_trim_is_a_fixed_point_of_the_outer_loop(params, V, label):
    """트림 상태를 넣으면 T=mg 수준, α≈0 이 나와야 한다.

    추력축 분기·헤딩 구성·공력 앞먹임의 부호가 하나라도 틀리면 여기서 깨진다.
    z축과 x축을 모두 시험하는 이유가 그것이다 — 이 프로젝트에서 부호는
    z-up/z-down 시점 차이로 뒤집힌 전력이 있다.
    """
    x = _trim_state(params, V)
    g = GeometricGuidance(params, v_ref=[V, 0, 0], z_ref=50.)
    u = g(0.0, x)

    # 트림 총추력은 무게를 지탱하는 수준(공력 성분 때문에 정확히 mg는 아니다)
    weight = params['mass']*params['g']
    assert 0.8*weight < u[0] < 1.3*weight, f"{label}: T={u[0]:.3f} vs mg={weight:.3f}"
    # 각가속도는 0 — 트림은 정지점이다
    np.testing.assert_allclose(u[1:4], 0.0, atol=1e-6,
                               err_msg=f"{label}: α={u[1:4]}")


@pytest.mark.parametrize('params,V', [(vehicle_params, 12.0), (SELECTED, 8.0)])
def test_desired_attitude_equals_current_attitude_at_trim(params, V):
    """목표 자세 R_d 가 트림 자세와 같아야 한다 — 회전오차의 직접 검증."""
    x = _trim_state(params, V)
    g = GeometricGuidance(params, v_ref=[V, 0, 0], z_ref=50.)
    g(0.0, x)
    R_cur = Rotation.from_quat(x[6:10]).as_matrix()
    e_R = Rotation.from_matrix(R_cur.T @ g._last_axes).as_rotvec()
    assert np.linalg.norm(e_R) < 1e-6, f"e_R={e_R}, 각 {np.degrees(np.linalg.norm(e_R)):.4f}°"
    # 목표 자세는 정상 회전행렬이어야 한다(직교·오른손)
    np.testing.assert_allclose(g._last_axes.T @ g._last_axes, np.eye(3), atol=1e-9)
    assert np.linalg.det(g._last_axes) == pytest.approx(1.0, abs=1e-9)


# ── 식(18) 입력 상자 ──────────────────────────────────────────────────

def test_alpha_is_limited_to_eq18():
    """|α_i| ≤ 100 rad/s². NMPC 계열이 제약으로 갖는 것을 여기선 직접 건다.

    같은 입력 상자를 쓰지 않으면 표5 비교가 '다른 문제를 푼 결과'가 된다.
    """
    x = _trim_state(SELECTED, 8.0)
    x[10:13] = [30.0, -25.0, 20.0]        # 큰 각속도 오차를 강제로 만든다
    # 게인을 키워 한계를 확실히 넘기게 한다
    g = GeometricGuidance(SELECTED, v_ref=[8, 0, 0], z_ref=50.,
                          gains={'kpW': 500.0})
    u = g(0.0, x)
    assert np.abs(u[1:4]).max() <= ALPHA_LIMIT + 1e-9
    assert np.abs(u[1:4]).max() == pytest.approx(ALPHA_LIMIT)   # 실제로 물렸는지


def test_thrust_is_limited_to_the_physical_capacity():
    """0 ≤ T ≤ Σf_max,i. 낼 수 없는 추력을 명령하면 INDI가 잘못 배분한다."""
    from control.hybrid_comparison import rotor_thrust_cap
    x = _trim_state(SELECTED, 8.0)
    x[2] = 0.0                              # 고도 오차 50 m → 거대한 상승 요구
    g = GeometricGuidance(SELECTED, v_ref=[8, 0, 0], z_ref=50.)
    u = g(0.0, x)
    v_axial = float((Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6])[0])
    cap = rotor_thrust_cap(SELECTED, max(v_axial, 0.0)).sum()
    assert 0.0 <= u[0] <= cap + 1e-9
    assert u[0] == pytest.approx(cap)       # 포화가 실제로 걸렸는지


# ── 상태 관리 ─────────────────────────────────────────────────────────

def test_outer_loop_holds_its_command_between_updates():
    """상위 50 Hz / 하위 1 kHz 구조 — 갱신 사이엔 직전 명령을 유지한다."""
    x = _trim_state(SELECTED, 8.0)
    g = GeometricGuidance(SELECTED, v_ref=[8, 0, 0], z_ref=50., dt_ctrl=0.02)
    first = g(0.0, x).copy()
    x2 = x.copy()
    x2[2] += 5.0                            # 상태를 크게 흔들어도
    np.testing.assert_array_equal(g(0.005, x2), first)   # 주기 전엔 그대로
    assert not np.allclose(g(0.020, x2), first)          # 주기가 되면 갱신


def test_reset_clears_integrators_and_filters():
    x = _trim_state(SELECTED, 8.0)
    # 작은 고도 오차. 크게 주면 추력이 포화해 조건부 적분이 적분기를 0으로
    # 잡아두므로(바로 아래 테스트가 그걸 확인한다) 이 테스트가 무의미해진다.
    x[2] -= 0.2
    g = GeometricGuidance(SELECTED, v_ref=[8, 0, 0], z_ref=50.)
    for k in range(50):
        g(k*0.02, x)
    assert np.linalg.norm(g._int_v) > 1e-6
    g.reset()
    np.testing.assert_array_equal(g._int_v, np.zeros(3))
    np.testing.assert_array_equal(g._int_w, np.zeros(3))
    np.testing.assert_array_equal(g._deriv_v, np.zeros(3))
    assert g._last_ev is None and g._last_axes is None


def test_conditional_integration_holds_while_thrust_saturated():
    """포화 중 적분기 정지(안티와인드업). 낼 수 없는 추력을 계속 적분하면
    포화가 풀린 뒤 크게 튄다."""
    x = _trim_state(SELECTED, 8.0)
    x[2] = 0.0                              # 요구 추력이 상한을 넘는 상태
    g = GeometricGuidance(SELECTED, v_ref=[8, 0, 0], z_ref=50.)
    for k in range(20):
        g(k*0.02, x)
    np.testing.assert_array_equal(g._int_v, np.zeros(3))
    np.testing.assert_array_equal(g._int_w, np.zeros(3))


def test_rate_gains_accept_per_axis_values():
    """축마다 관성이 다르면 스칼라 하나로는 실효 게인이 갈린다.

    선정 프로파일은 Iyy/Ixx = 4.6 이라 kpW=3.75 에서 롤 490 대 피치 107 이
    되고 롤 루프가 불안정해진다(results/GINDI_STATUS.md). 튜너가 축별로
    줄 수 있어야 한다.
    """
    x = _trim_state(SELECTED, 8.0)
    x[10:13] = [0.3, -0.2, 0.1]
    inertia = np.array([SELECTED['Ixx'], SELECTED['Iyy'], SELECTED['Izz']])
    from control.gindi import GINDI_GAINS
    # 피치축의 실효 게인 kpW/Iyy 에 모든 축을 맞춘다 — 반올림하면 피치축이
    # 스칼라 설정과 정확히 같아지지 않으므로 나눗셈 그대로 쓴다.
    equalised = GINDI_GAINS['kpW']/inertia[1]*inertia
    scalar = GeometricGuidance(SELECTED, v_ref=[8, 0, 0], z_ref=50.)(0.0, x)
    per_axis = GeometricGuidance(SELECTED, v_ref=[8, 0, 0], z_ref=50.,
                                 gains={'kpW': tuple(equalised)})(0.0, x)
    # 스칼라 3.75 는 롤에서 kpW/Ixx=490, 통일하면 107 → 롤만 크게 다르고
    # 피치·요는 원래 107 이었으므로 그대로여야 한다
    assert abs(per_axis[1]) < abs(scalar[1])
    assert per_axis[2] == pytest.approx(scalar[2], rel=1e-6)
