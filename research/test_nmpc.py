"""Numerical conditioning must not change the physical optimization problem."""
import casadi as ca
import numpy as np
import pytest
from research.model import profile, build, initial_state
from research.nmpc import build_problem, N


@pytest.mark.parametrize("kind", ["hybrid", "nmpc", "f13"])
def test_dimensionless_coordinates_preserve_cost_defects_and_bounds(kind):
    p=profile("selected")
    functions=build(p)
    raw, a=build_problem(p,functions,kind,normalize=False)
    scaled, b=build_problem(p,functions,kind,normalize=True)
    nx=a["nx"]
    x0=initial_state(p)[:nx]
    xs=np.tile(x0,N+1)
    # Deliberately non-feasible states/commands exercise both defect and cost
    # transformations, not just the zero residual of an exact hover solution.
    xs[nx+3]=.2
    us=np.tile(a["hover"],N)*1.01
    physical=np.concatenate([xs,us])
    factors=np.concatenate([np.tile(b["state_scaling"],N+1),np.tile(b["command_scaling"],N)])
    refs=np.tile([3,0,0,20],N+1)
    parameters=np.concatenate([x0,refs,a["hover"],[0,0,0,24]])
    evaluate=lambda problem,values: ca.Function("evaluate",[problem["x"],problem["p"]],
                                               [problem["f"],problem["g"]])(values,parameters)
    raw_f,raw_g=evaluate(raw,physical)
    scaled_f,scaled_g=evaluate(scaled,physical/factors)
    assert float(scaled_f)==pytest.approx(float(raw_f),rel=1e-11)
    np.testing.assert_allclose(np.asarray(scaled_g).ravel()*np.tile(b["state_scaling"],N+1),
                               np.asarray(raw_g).ravel(),atol=1e-9,rtol=1e-9)
    np.testing.assert_allclose(np.asarray(b["lbx"])*factors,a["lbx"])
    np.testing.assert_allclose(np.asarray(b["ubx"])*factors,a["ubx"])


def test_constraint_form_motor_prediction_matches_the_saturated_plant_inside_limits():
    p=profile("selected");functions=build(p);x=initial_state(p)[:17]
    limit=min(p["motor"]["current_limit_A"],p["battery"]["capacity_Ah"]*p["battery"]["max_C"]*p["battery"]["esc_efficiency"]/4)
    rng=np.random.default_rng(42)
    for _ in range(20):
        command=x[13:17]+rng.uniform(-50,50,4)
        free,current,voltage=functions["constrained"](x,command,[0,0,0],24)
        assert np.all(np.asarray(current)>=0) and np.all(np.asarray(current)<=limit)
        assert np.all(np.asarray(voltage)<=24)
        np.testing.assert_allclose(free,functions["full"](x,command,[0,0,0],24),atol=1e-10,rtol=1e-10)
    # A command outside that domain is NOT equivalent: the explicit inequality
    # must reject it, while the actual plant coasts with current clipped to zero.
    free,current,_=functions["constrained"](x,[0]*4,[0,0,0],24)
    assert np.all(np.asarray(current)<0)
    assert not np.allclose(free,functions["full"](x,[0]*4,[0,0,0],24))


def test_hybrid_envelope_adds_only_four_first_move_constraints_not_motor_states():
    p=profile();functions=build(p)
    old,a=build_problem(p,functions,"hybrid")
    new,b=build_problem(p,functions,"hybrid",hybrid_envelope=True)
    assert a["nx"]==b["nx"]==13
    assert old["x"].numel()==new["x"].numel()
    assert new["g"].numel()==old["g"].numel()+4
    assert new["p"].numel()==old["p"].numel()+20
    assert b["hybrid_envelope"]["horizon_s"]==.02
    # A known coupled map: infeasible combinations must not pass just because
    # each virtual axis separately fits in an axis-aligned min/max interval.
    matrix=np.array([[1,1,1,1],[1,-1,1,-1],[1,1,-1,-1],[-1,1,1,-1]],dtype=float)
    inverse=np.linalg.inv(matrix);offset=np.array([0,.1,.2,.3])
    force=np.array([1,2,3,4]);u=matrix@force+offset
    xs=np.tile(initial_state(p)[:13],N+1)
    us=np.tile(u/np.array(b["command_scaling"]),N)
    base=np.r_[initial_state(p)[:13],np.tile([0,0,0,20],N+1),a["hover"],0,0,0,24]
    params=np.r_[base,inverse.ravel(),offset]
    evaluate=ca.Function("envelope_test",[new["x"],new["p"]],[new["g"],new["f"]])
    g,cost=evaluate(np.r_[xs,us],params)
    np.testing.assert_allclose(np.asarray(g).ravel()[-4:]*p["mass_kg"]*p["g"],force,atol=1e-10)
    # Later commands do not get today's local envelope frozen over a 1 s horizon.
    us[4:8]+=1
    after,_=evaluate(np.r_[xs,us],params)
    np.testing.assert_allclose(np.asarray(after)[-4:],np.asarray(g)[-4:])
    old_cost=ca.Function("old_cost",[old["x"],old["p"]],[old["f"]])
    assert float(cost)==pytest.approx(float(old_cost(np.r_[xs,np.tile(u/np.array(b["command_scaling"]),N)],base)))


@pytest.mark.parametrize("name",["simple","selected"])
def test_motor_endpoint_forecast_matches_independent_frozen_airframe_integration(name):
    from scipy.integrate import solve_ivp
    p=profile(name);functions=build(p);x=initial_state(p)[:17]
    # Same held bus voltage and inflow. Compare RK4 against an independently
    # integrated full-predictor motor slice; airframe motion is intentionally held.
    for command,voltage in [(np.zeros(4),24),(x[13:17]*1.4,24),(x[13:17]*1.4,15)]:
        def rhs(t,n):
            at=x.copy();at[13:17]=n
            return np.asarray(functions["full"](at,command,[0,0,0],voltage)).ravel()[13:17]
        exact=solve_ivp(rhs,(0,.02),x[13:17],rtol=1e-10,atol=1e-10).y[:,-1]
        actual=x[13:17].copy()
        for _ in range(20):actual=np.asarray(functions["motor_step"](actual,command,0,voltage)).ravel()
        np.testing.assert_allclose(actual,exact,atol=.01,rtol=1e-5)


def test_larger_rotor_inertia_narrows_the_twenty_ms_response_interval():
    import copy
    from research.model import rpm_limit
    p=profile();slower=copy.deepcopy(p)
    slower["motor"]["rotor_inertia_kg_m2"]*=2
    intervals=[]
    for parameters in (p,slower):
        functions=build(parameters);endpoints=[]
        for command in (np.zeros(4),np.full(4,rpm_limit(parameters))):
            n=initial_state(parameters)[13:17]
            for _ in range(20):n=np.asarray(functions["motor_step"](n,command,0,24)).ravel()
            endpoints.append(n)
        intervals.append(endpoints)
    assert np.all(intervals[1][0]>intervals[0][0])  # Slower passive deceleration.
    assert np.all(intervals[1][1]<intervals[0][1])  # Slower powered acceleration.
