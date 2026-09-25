"""
thrust_axis 일반화 검증 (dynamics.py / hybrid_comparison.py / nmpc_f13.py).

bulnabi 브랜치가 이미 검증해둔 패턴(control/test_research_parity.py)과 같은
방식으로: (1) z축 기본값이 기존과 여전히 동일한지, (2) x축(로켓형 배치)도
같은 물리 규칙을 따르는지를 CasADi 자동미분/직접 적분으로 대조한다.
thrust_axis='z'는 control/vehicle_params.py의 기존 vehicle_params 그대로
쓰고, thrust_axis='x'는 이 테스트 안에서만 쓰는 합성 파라미터로 만든다
(vehicle_params.py 자체엔 이번 작업 범위상 손대지 않았다).
"""

import numpy as np
import casadi as ca

from control.vehicle_params import vehicle_params as P_Z
from control.dynamics import (AxialDronePlant, build_dynamics, _quat_to_rotmat,
                               _rotor_forces_moments, compute_allocation_matrix)
from control.hybrid_comparison import build_virtual_dynamics, compute_control_effectiveness
from control.nmpc_f13 import build_f13_dynamics


def _x_axis_params():
    """뿔나비 rocket_params와 같은 패턴의 합성 x축 파라미터 (기수축 둘레 로터)."""
    arm = P_Z['arm_length']
    s = arm / np.sqrt(2)
    p = dict(P_Z)
    p.update({
        'thrust_axis': 'x',
        'rotor_positions': np.array([
            [0.0,  s,  s],
            [0.0, -s,  s],
            [0.0, -s, -s],
            [0.0,  s, -s],
        ]),
        'rotor_directions': np.array([1, -1, 1, -1]),
    })
    return p


PARAM_SETS = [('z', P_Z), ('x', _x_axis_params())]


def test_hover_state_is_true_equilibrium():
    """AxialDronePlant.hover_state가 두 축 모두에서 실제 xdot≈0 평형인지."""
    for axis, p in PARAM_SETS:
        x = AxialDronePlant.hover_state(p)
        f, _, _ = build_dynamics(p)
        xdot = np.asarray(f(x, x[13:17])).ravel()
        print(f"  [{axis}축] |xdot|_max = {np.max(np.abs(xdot)):.2e}")
        assert np.allclose(xdot, 0, atol=1e-8), f"{axis}축 호버가 평형이 아님"
    print("  PASS")


def test_allocation_matrix_matches_casadi_jacobian():
    """compute_allocation_matrix(다음 compute_control_effectiveness의 기반)가
    _rotor_forces_moments를 직접 자동미분한 것과 축 모두에서 일치하는지."""
    for axis, p in PARAM_SETS:
        n_sym = ca.SX.sym('n', 4)
        omega0 = ca.DM.zeros(3)
        v0 = ca.DM.zeros(3)   # 호버(무유입) 기준 — 전진비 fac=1
        F, M = _rotor_forces_moments(v0, n_sym, omega0, p)
        total = F[0] if axis == 'x' else -F[2]
        wrench = ca.vertcat(total, M[0], M[1], M[2])
        jac_fn = ca.Function('wrench_jac', [n_sym], [ca.jacobian(wrench, n_sym)])

        n_hov = np.sqrt(p['mass'] * p['g'] / (4 * p['k_T']))
        n_actual = np.full(4, n_hov)
        expected = np.asarray(jac_fn(n_actual))   # [T,Mx,My,Mz] wrench, 관성 미포함

        A_alloc, _ = compute_allocation_matrix(p)
        # 호버(fac=1)에서 dT_i/dn_i = 2*k_T*n_i — 이 스칼라만큼 배율.
        dT_dn = 2 * p['k_T'] * n_actual
        actual = A_alloc * dT_dn[np.newaxis, :]

        err = np.max(np.abs(actual - expected))
        print(f"  [{axis}축] |A·dT/dn - CasADi jacobian|_max = {err:.2e}")
        assert err < 1e-8, f"{axis}축 배분행렬이 자동미분과 불일치"
    print("  PASS")


def test_compute_control_effectiveness_matches_casadi_jacobian():
    """compute_control_effectiveness(관성으로 나눈 최종 G)가 직접 자동미분한
    ∂[T,ω̇]/∂n과 축 모두에서 일치하는지 (전진비 반영 상태, v_body≠0)."""
    for axis, p in PARAM_SETS:
        v_body = np.array([25.0, 0, 0]) if axis == 'x' else np.array([0, 0, -25.0])
        n_actual = np.array([600.0, 650.0, 580.0, 700.0])

        n_sym = ca.SX.sym('n', 4)
        F, M = _rotor_forces_moments(ca.DM(v_body), n_sym, ca.DM.zeros(3), p)
        total = F[0] if axis == 'x' else -F[2]
        J = ca.DM([p['Ixx'], p['Iyy'], p['Izz']])
        wrench = ca.vertcat(total, M[0]/J[0], M[1]/J[1], M[2]/J[2])
        jac_fn = ca.Function('wrench_jac', [n_sym], [ca.jacobian(wrench, n_sym)])
        expected = np.asarray(jac_fn(n_actual))

        actual = compute_control_effectiveness(p, n_actual, v_body)
        err = np.max(np.abs(actual - expected))
        print(f"  [{axis}축] G(전진비 반영) 오차 = {err:.2e}")
        assert err < 1e-8, f"{axis}축 compute_control_effectiveness가 자동미분과 불일치"
    print("  PASS")


def test_virtual_dynamics_matches_physical_plant():
    """hybrid_comparison.build_virtual_dynamics(VirtualNMPC 내부 모델)가 같은
    총추력에서 실제 17상태 플랜트와 병진·자세 미분이 같은지 (V13/F13의 전제)."""
    for axis, p in PARAM_SETS:
        x17 = AxialDronePlant.hover_state(p)
        x17[3:6] = [20, 3, 5] if axis == 'x' else [20, 3, -5]
        x17[10:13] = [0.05, -0.1, 0.2]
        n_vec = np.array([550.0, 600.0, 520.0, 610.0])
        x17[13:17] = n_vec

        # 실제 플랜트가 보는 것과 같은 v_body(전진비 반영)로 T_total을 뽑아야
        # 한다 — v_body=0으로 뽑으면 fac=1로 과대평가돼 축마다 다른 유입속도
        # 때문에 z축은 우연히 맞고 x축만 틀리는 식의 테스트 버그가 생긴다.
        R = np.asarray(_quat_to_rotmat(ca.DM(x17[6:10])))
        v_body_real = R.T @ x17[3:6]
        F_r, _ = _rotor_forces_moments(
            ca.DM(v_body_real), ca.DM(n_vec), ca.DM(x17[10:13]), p)
        T_total = float(F_r[0]) if axis == 'x' else -float(F_r[2])

        f_full, _, _ = build_dynamics(p)
        xdot_full = np.asarray(f_full(x17, n_vec)).ravel()

        f_virt, _, _ = build_virtual_dynamics(p)
        x13 = np.concatenate([x17[0:10], x17[10:13]])
        u_virt = np.array([T_total, 0, 0, 0])   # ω̇는 비교 대상 아님(INDI가 대체)
        xdot_virt = np.asarray(f_virt(x13, u_virt)).ravel()

        # p_dot(0:3), v_dot(3:6), q_dot(6:10) 만 비교 — 회전은 가상모델이 ν로 대체.
        err = np.max(np.abs(xdot_virt[0:10] - xdot_full[0:10]))
        print(f"  [{axis}축] |가상모델 - 실제플랜트|_max(병진+자세) = {err:.2e}")
        assert err < 1e-6, f"{axis}축 가상모델이 실제 플랜트와 불일치"
    print("  PASS")


def test_f13_dynamics_hover_equilibrium():
    """nmpc_f13.build_f13_dynamics도 두 축 모두에서 호버가 평형인지."""
    for axis, p in PARAM_SETS:
        x13 = AxialDronePlant.hover_state(p)[0:13]
        n_hov = np.sqrt(p['mass'] * p['g'] / (4 * p['k_T']))
        f_hov = p['k_T'] * n_hov**2
        u = np.full(4, f_hov)   # 로터별 추력(N), 호버 등분배

        f13, _, _ = build_f13_dynamics(p)
        xdot = np.asarray(f13(x13, u)).ravel()
        print(f"  [{axis}축] F13 |xdot|_max = {np.max(np.abs(xdot)):.2e}")
        assert np.allclose(xdot, 0, atol=1e-6), f"{axis}축 F13 호버가 평형이 아님"
    print("  PASS")


if __name__ == '__main__':
    tests = [
        test_hover_state_is_true_equilibrium,
        test_allocation_matrix_matches_casadi_jacobian,
        test_compute_control_effectiveness_matches_casadi_jacobian,
        test_virtual_dynamics_matches_physical_plant,
        test_f13_dynamics_hover_equilibrium,
    ]
    for t in tests:
        print(f"\n{'='*55}\n{t.__name__}")
        t()
    print(f"\n{'='*55}\nALL THRUST_AXIS TESTS PASSED\n{'='*55}")
