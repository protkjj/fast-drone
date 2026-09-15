import casadi as ca
import numpy as np
import pytest
from research.model import build, initial_state, profile, rotation, rpm_limit, motors


@pytest.fixture(params=["simple", "selected"])
def model(request):
    p = profile(request.param)
    return p, build(p)


def diagnostic(f, x, u):
    return {k: np.asarray(v).ravel() for k, v in f["diag"](x=x, u=u, env=np.zeros(6), scales=np.ones(5)).items()}


def test_hover_and_state_dimensions(model):
    p, f = model
    x = initial_state(p)
    dx = np.asarray(f["rhs"](x, x[13:17], np.zeros(6), np.ones(5))).ravel()
    np.testing.assert_allclose(dx[:17], 0, atol=1e-8)
    assert dx[17] < 0  # A hovering vehicle consumes battery charge.
    assert f["full"].size1_in(0) == 17
    assert f["virtual"].size1_in(0) == 13


def test_selected_hover_reproduces_upstream():
    p = profile()
    f = build(p)
    anchor = p["anchors"]["hover"]
    x = initial_state(p)
    assert np.isclose(x[13]*60/(2*np.pi), anchor["rpm"], rtol=1e-5)
    d = diagnostic(f, x, x[13:17])
    assert np.isclose(sum(d["thrust"]), anchor["T"], rtol=1e-5)
    assert np.isclose(d["shaft"][0], 4*anchor["P_shaft"], rtol=1e-5)
    assert np.isclose(d["power"][0], anchor["P"], rtol=1e-5)


def test_selected_cruise_propeller_anchor():
    p = profile()
    f = build(p)
    a = p["anchors"]["cruise"]
    x = initial_state(p)
    x[3] = a["speed_mps"]
    # Upstream prop.solve_point uses J=V/(f D), not V*cos(theta)/(f D).
    # Match its AXIAL propeller operating point, not claim full 6DOF trim parity.
    x[6:10] = [0, 0, 0, 1]
    x[13:17] = a["rpm"]*2*np.pi/60
    d = diagnostic(f, x, x[13:17])
    assert np.isclose(sum(d["thrust"]), a["T"], rtol=1e-5)
    assert np.isclose(d["shaft"][0], 4*a["P_shaft"], rtol=1e-5)
    assert not np.any(d["outside_map"])


def test_motor_battery_energy_balance_and_limits(model):
    p, f = model
    rng = np.random.default_rng(42)
    for _ in range(30):
        x = initial_state(p)
        x[13:17] *= rng.uniform(.2, 1.8, 4)
        x[17] = rng.uniform(.2, 1)
        u = rng.uniform(0, rpm_limit(p), 4)
        d = diagnostic(f, x, u)
        np.testing.assert_allclose(d["motor_energy_residual"], 0, atol=1e-8)
        np.testing.assert_allclose(d["bus_residual"], 0, atol=1e-8)
        assert np.all(d["current"] >= 0)
        assert np.all(d["current"] <= p["motor"]["current_limit_A"]+1e-9)
        assert d["ibus"][0] <= p["battery"]["max_C"]*p["battery"]["capacity_Ah"]+1e-9
        assert d["power"][0] >= 0


def test_aerodynamic_force_is_passive_even_in_reverse(model):
    p, f = model
    rng = np.random.default_rng(7)
    for _ in range(30):
        x = initial_state(p)
        x[3:6] = rng.normal(0, 40, 3)
        force = np.asarray(f["aero"](x[:13], [0,0,0])).ravel()
        vb = np.asarray(rotation(ca.DM(x[6:10]))).T @ x[3:6]
        assert force @ vb <= 1e-8


def test_indi_effectiveness_is_actual_map_derivative(model):
    p, f = model
    n = initial_state(p)[13:17]*[.8, 1.1, 1.2, .9]
    _, g = f["effect"](n, 12)
    columns = []
    for i in range(4):
        dn = np.eye(4)[i]*.01
        plus = np.asarray(f["effect"](n+dn, 12)[0]).ravel()
        minus = np.asarray(f["effect"](n-dn, 12)[0]).ravel()
        columns.append((plus-minus)/.02)
    np.testing.assert_allclose(np.asarray(g), np.array(columns).T, rtol=1e-6, atol=1e-8)


def test_virtual_translation_matches_actual_at_equal_thrust(model):
    p, f = model
    x = initial_state(p)
    x[3:6] = [10, 2, 3]
    x[10:13] = [.1, -.2, .3]
    d = diagnostic(f, x, x[13:17])
    actual = np.asarray(f["rhs"](x, x[13:17], [0]*6, [1]*5)).ravel()
    virtual = np.asarray(f["virtual"](x[:13], [sum(d["thrust"]), 0,0,0], [0]*3)).ravel()
    np.testing.assert_allclose(actual[:10], virtual[:10], atol=1e-9)


def test_step_does_not_snap_velocity_to_reference(model):
    p, f = model
    x = initial_state(p)
    u = x[13:17]*1.1
    nxt = np.asarray(f["step"](x, u, [0]*6, [1]*5)).ravel()
    assert 0 < nxt[5] < .1
    assert np.isclose(np.linalg.norm(nxt[6:10]), 1)
    # No target speed is even an argument to the plant.
    assert f["step"].n_in() == 4
