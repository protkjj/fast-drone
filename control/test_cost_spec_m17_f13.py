"""M17·F13 cost_spec='paper' 검증 — 야간지시 3-a (2026-09-25).

V13(hybrid_comparison.py)에 이미 있는 검증 패턴을 그대로 따른다: 문제
규모가 논문 식(31)과 맞는지, 비용함수가 손계산과 기계정밀도로 일치하는지,
호버 트림이 고정점인지. 세 제어기가 다른 입력 공간(로터속도/로터추력/
가상입력)을 쓰므로 Dν 척도도 다르지만(연구 kind별: nmpc→max_n, f13→mg/4,
hybrid→[mg,100,100,100]), 식(14)-(18) 자체는 동일해야 한다.
"""
import numpy as np
import pytest

from control.nmpc import NMPCController
from control.nmpc_f13 import NU_F, RotorThrustNMPC13
from control.trim import find_trim
from control.vehicle_params import load_selected_params, vehicle_params

SELECTED = load_selected_params()


def test_m17_defaults_to_paper_and_matches_eq31_nvar():
    m = NMPCController(vehicle_params, v_ref=[10, 0, 0], z_ref=50.)
    assert m.cost_spec == 'paper'
    N, nx, nu = m.N, 17, 4
    assert m.w0.size == (N+1)*nx + N*nu == 437     # 논문 n_var,17


def test_f13_defaults_to_paper_and_matches_eq31_nvar():
    f = RotorThrustNMPC13(vehicle_params, v_ref=[10, 0, 0], z_ref=50.)
    assert f.cost_spec == 'paper'
    N, nx, nu = f.N, 13, 4
    assert f.w0.size == (N+1)*nx + N*nu == 353     # 논문 n_var,13


def test_m17_paper_cost_matches_hand_calc():
    N, nx, nu = 20, 17, 4
    m = NMPCController(vehicle_params, v_ref=[10, 0, 0], z_ref=50.)
    J_fn = m.solver.get_function('nlp_f')
    rng = np.random.default_rng(3)
    w = m.w0 + rng.standard_normal(m.w0.size)
    refs = rng.standard_normal((4, N+1))*3 + np.array([[10.], [0.], [0.], [50.]])
    x0 = np.zeros(nx); x0[6:10] = [1, 0, 0, 0]
    p_val = np.concatenate([x0, refs.ravel(order='F')])

    n_max = vehicle_params['n_max']
    n_hov = m.u_ref[0]
    Xs, Us, i = [], [], 0
    for _ in range(N):
        Xs.append(w[i:i+nx]); i += nx
        Us.append(w[i:i+nu]); i += nu
    Xs.append(w[i:i+nx]); i += nx
    assert i == w.size

    def stage(X, k):
        return (5.0*np.sum((X[3:6]-refs[0:3, k])**2) + 20.0*(X[2]-refs[3, k])**2
                + np.sum(X[10:13]**2))

    expected, u_prev = 0.0, np.full(4, n_hov)
    for k in range(N):
        expected += stage(Xs[k], k)
        expected += 0.02*np.sum(((Us[k]-n_hov)/n_max)**2)
        expected += 0.10*np.sum(((Us[k]-u_prev)/n_max)**2)
        u_prev = Us[k]
    expected += 10.0*stage(Xs[N], N)

    assert float(J_fn(w, p_val)[0]) == pytest.approx(expected, rel=1e-12)


def test_f13_paper_cost_matches_hand_calc():
    N, nx, nu = 20, 13, 4
    f = RotorThrustNMPC13(vehicle_params, v_ref=[10, 0, 0], z_ref=50.)
    J_fn = f.solver.get_function('nlp_f')
    rng = np.random.default_rng(4)
    w = f.w0 + rng.standard_normal(f.w0.size)
    refs = rng.standard_normal((4, N+1))*3 + np.array([[10.], [0.], [0.], [50.]])
    x0 = np.zeros(nx); x0[6:10] = [1, 0, 0, 0]
    p_val = np.concatenate([x0, refs.ravel(order='F')])

    D_nu = vehicle_params['mass']*vehicle_params['g']/4.0
    Xs, Us, i = [], [], 0
    for _ in range(N):
        Xs.append(w[i:i+nx]); i += nx
        Us.append(w[i:i+nu]); i += nu
    Xs.append(w[i:i+nx]); i += nx
    assert i == w.size

    def stage(X, k):
        return (5.0*np.sum((X[3:6]-refs[0:3, k])**2) + 20.0*(X[2]-refs[3, k])**2
                + np.sum(X[10:13]**2))

    expected, u_prev = 0.0, np.full(4, D_nu)
    for k in range(N):
        expected += stage(Xs[k], k)
        expected += 0.02*np.sum(((Us[k]-D_nu)/D_nu)**2)
        expected += 0.10*np.sum(((Us[k]-u_prev)/D_nu)**2)
        u_prev = Us[k]
    expected += 10.0*stage(Xs[N], N)

    assert float(J_fn(w, p_val)[0]) == pytest.approx(expected, rel=1e-12)


@pytest.mark.parametrize('params,label', [(vehicle_params, 'z축'), (SELECTED, 'x축')])
def test_m17_hovers_at_trim_rotor_speed(params, label):
    tr = find_trim(params, 0.0, quiet=True)
    assert tr['converged']
    x = np.zeros(17)
    x[6:10] = tr['state'][6:10]
    x[13:17] = tr['state'][13:17]
    m = NMPCController(params, v_ref=[0, 0, 0], z_ref=0.)
    for k in range(5):
        u = m(k*0.02, x)
    np.testing.assert_allclose(u, x[13:17], rtol=1e-3, err_msg=label)


@pytest.mark.parametrize('params,label', [(vehicle_params, 'z축'), (SELECTED, 'x축')])
def test_f13_hovers_at_mg_over_4(params, label):
    tr = find_trim(params, 0.0, quiet=True)
    assert tr['converged']
    x13 = np.zeros(13)
    x13[6:10] = tr['state'][6:10]
    f = RotorThrustNMPC13(params, v_ref=[0, 0, 0], z_ref=0.)
    for k in range(5):
        u = f(k*0.02, np.concatenate([x13, tr['state'][13:17]]))
    expected = params['mass']*params['g']/4.0
    np.testing.assert_allclose(u, expected, rtol=1e-3, err_msg=label)


def test_m17_legacy_mode_still_available_and_unchanged_nvar():
    m = NMPCController(vehicle_params, v_ref=[10, 0, 0], z_ref=50., cost_spec='legacy')
    assert m.w0.size == 20*(4+17)   # 기존 배치, x0가 파라미터


def test_f13_legacy_mode_still_available_and_unchanged_nvar():
    f = RotorThrustNMPC13(vehicle_params, v_ref=[10, 0, 0], z_ref=50., cost_spec='legacy')
    assert f.w0.size == 20*(NU_F+13)
