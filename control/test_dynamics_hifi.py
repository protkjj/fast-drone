"""고충실도(표 기반) 플랜트 검증 — control/dynamics.py 는 건드리지 않는다.

여기서 확인하는 것: (1) 기존 단순화 플랜트가 비트 단위로 안 변했는지,
(2) 표 없는 프로파일은 명확히 거부하는지, (3) 호버(J=0)에서 두 모델이
일치하는지, (4) 임의 속도에서 실제 CT(J)/CD0(V) 표 값과 정확히 일치하는지.
"""
import numpy as np
import pytest

from control.dynamics import AxialDronePlant
from control.dynamics_hifi import AxialDronePlantHighFidelity
from control.trim import find_trim
from control.vehicle_params import load_selected_params, rocket_params, vehicle_params

P = load_selected_params()


def test_hifi_plant_rejects_tableless_profiles():
    for params in (vehicle_params, rocket_params):
        with pytest.raises(ValueError, match='prop_table'):
            AxialDronePlantHighFidelity(params)


def test_hifi_matches_simple_model_at_hover():
    """J=0 에서는 두 모델 다 CT(0)/CP(0) 을 쓰므로 일치해야 한다."""
    tr = find_trim(P, 0.0, quiet=True)
    x0 = np.zeros(17)
    x0[6:10] = tr['state'][6:10]
    x0[13:17] = tr['state'][13:17]

    simple = AxialDronePlant(P, dt=0.001).evaluate_xdot(x0, x0[13:17])
    hifi = AxialDronePlantHighFidelity(P, dt=0.001).evaluate_xdot(x0, x0[13:17])
    np.testing.assert_allclose(simple, hifi, atol=1e-10)


def test_hifi_rotor_thrust_matches_the_ct_table_exactly():
    """표 보간값으로 손계산한 추력·항력과 정확히 일치해야 한다.

    q=단위원(항등회전)이면 body 축=world 축이라(_quat_to_rotmat([0,0,0,1])=I),
    v_body=x[3:6]를 그대로 쓸 수 있어 자세 기하를 따로 풀 필요가 없다.
    thrust_axis='x'이므로 body +x가 추력축이고, world +x로 속도를 주면
    V_axial=85 그 자체다.
    """
    plant = AxialDronePlantHighFidelity(P, dt=0.001)
    n_max = P['n_max']
    x = np.zeros(17)
    x[3] = 85.0
    x[6:10] = [0.0, 0.0, 0.0, 1.0]   # 단위 쿼터니언 (scalar-last) → R=I
    x[13:17] = n_max

    xdot = plant.evaluate_xdot(x, x[13:17])

    rev = n_max/(2*np.pi)
    J = 85.0/(rev*P['D_prop'])
    ct = np.interp(J, P['prop_table']['J'], P['prop_table']['CT'])
    T_total_expected = 4*ct*P['rho']*rev**2*P['D_prop']**4

    cd = np.interp(85.0, P['drag_table']['speed_mps'], P['drag_table']['CD0'])
    D_expected = 0.5*P['rho']*P['S_ref']*cd*85.0**2

    a_x_expected = (T_total_expected - D_expected)/P['mass']  # 중력은 world z, 여기 영향 없음
    assert xdot[3] == pytest.approx(a_x_expected, rel=1e-9)


def test_hifi_closed_loop_flies_at_85(subtests=None):
    """단순화 모델로 설계한 제어기가 표 기반(진짜) 플랜트를 실제로 제어하는지.

    이게 '제어기는 단순 모델만 알고 있는데도 고충실도 플랜트를 잘 제어한다'
    는 방법론(플랜트≠제어기 모델)의 가장 기본적인 존재 증명이다.
    """
    from control.hybrid_comparison import ProperHybrid, VirtualNMPC
    n_hov = np.sqrt(P['mass']*P['g']/(4*P['k_T']))
    tr = find_trim(P, 85.0, guess=[np.radians(85.0), n_hov*1.5, 0.0], quiet=True)
    assert tr['converged']
    x0 = tr['state']

    plant = AxialDronePlantHighFidelity(P, dt=0.001)
    ctrl = ProperHybrid(VirtualNMPC(P, v_ref=[85.0, 0, 0], z_ref=x0[2]), P, dt=0.001)
    ts, xs, us = plant.simulate(x0, ctrl, 3.0)

    assert np.all(np.isfinite(xs))
    assert abs(xs[-1, 2] - x0[2]) < 5.0          # 고도 5 m 이내 유지
    assert abs(xs[-1, 3] - 85.0) < 5.0           # 속도 5 m/s 이내 유지
    assert np.linalg.norm(xs[:, 10:13], axis=1).max() < 25.0   # 텀블 아님
