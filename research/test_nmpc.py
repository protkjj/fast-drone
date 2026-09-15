"""Numerical conditioning must not change the physical optimization problem."""
import casadi as ca
import numpy as np
import pytest
from research.model import profile, build, initial_state
from research.nmpc import build_problem, N


@pytest.mark.parametrize("kind", ["hybrid", "nmpc"])
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
