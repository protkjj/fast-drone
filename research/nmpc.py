"""Shared 50 Hz IPOPT problems, serialized unchanged to the browser.

Both controllers use N=20, prediction dt=50 ms and the same tracking cost.
Hybrid optimizes [T, alpha]; standalone optimizes four rotor-speed commands.
No controller is allowed to write a simulated position or velocity.
"""
import casadi as ca
import numpy as np
from research.model import initial_state, rpm_limit

N = 20
PRED_DT = .05
CONTROL_DT = .02


def build_solver(p, functions, kind):
    if kind not in ("hybrid", "nmpc"):
        raise ValueError(kind)
    nx = 13 if kind == "hybrid" else 17
    x = ca.MX.sym("X", nx, N+1)
    u = ca.MX.sym("U", 4, N)
    x0 = ca.MX.sym("measured_state", nx)
    refs = ca.MX.sym("references", 4, N+1)  # vx,vy,vz,z at each prediction node
    prev = ca.MX.sym("last_applied_command", 4)
    wind = ca.MX.sym("estimated_wind", 3)
    bus = ca.MX.sym("measured_bus_voltage")
    max_n = rpm_limit(p)
    max_t = float(functions["effect"]([max_n]*4, 0)[0][0])
    if kind == "hybrid":
        lower, upper = [0, -100, -100, -100], [max_t, 100, 100, 100]
        hover = [p["mass_kg"]*p["g"], 0, 0, 0]
        scaling = ca.DM([p["mass_kg"]*p["g"], 100, 100, 100])
        fun = lambda state, command: functions["virtual"](state, command, wind)
    else:
        lower, upper = [0]*4, [max_n]*4
        hover = initial_state(p)[13:17].tolist()
        scaling = ca.DM([max_n]*4)
        fun = lambda state, command: functions["full"](state, command, wind, bus)

    # 5 integration substeps avoid RK4 instability from a 20 ms motor time scale.
    def integrate(state, command):
        for _ in range(5):
            h = PRED_DT/5
            k1 = fun(state, command)
            k2 = fun(state+h/2*k1, command)
            k3 = fun(state+h/2*k2, command)
            k4 = fun(state+h*k3, command)
            state = state+h/6*(k1+2*k2+2*k3+k4)
            state = ca.vertcat(state[:6], state[6:10]/ca.norm_2(state[6:10]), state[10:])
        return state

    # Keep one transition graph. Reusing it at all nodes avoids serializing 20
    # copies of the integrator and its second derivatives into the WASM bundle.
    one_x, one_u = ca.MX.sym("one_x", nx), ca.MX.sym("one_u", 4)
    transition = ca.Function("prediction_step", [one_x, one_u, wind, bus], [integrate(one_x, one_u)])
    eq = [x[:, 0]-x0]
    cost = 0
    previous = prev
    for k in range(N+1):
        error = x[3:6, k]-refs[:3, k]
        stage = 5*ca.sumsqr(error) + 20*(x[2,k]-refs[3,k])**2 + ca.sumsqr(x[10:13,k])
        cost += (10 if k == N else 1)*stage
        if k < N:
            eq.append(x[:, k+1]-transition(x[:,k], u[:,k], wind, bus))
            # Dimensionless penalties, fixed for every profile and test case.
            cost += .02*ca.sumsqr((u[:,k]-ca.DM(hover))/scaling)
            cost += .1*ca.sumsqr((u[:,k]-previous)/scaling)
            previous = u[:,k]
    variables = ca.vertcat(ca.vec(x), ca.vec(u))
    parameters = ca.vertcat(x0, ca.vec(refs), prev, wind, bus)
    opts = {"ipopt.print_level": 0, "print_time": False, "ipopt.sb": "yes",
            "ipopt.max_iter": 30, "ipopt.tol": 1e-5, "ipopt.acceptable_tol": 1e-4,
            "ipopt.mu_strategy": "adaptive", "ipopt.warm_start_init_point": "yes",
            "ipopt.warm_start_bound_push": 1e-6, "ipopt.warm_start_mult_bound_push": 1e-6,
            "error_on_fail": False}
    solver = ca.nlpsol(kind, "ipopt", {"x": variables, "p": parameters, "f": cost, "g": ca.vertcat(*eq)}, opts)
    # JSON has no Infinity. Generous finite state bounds are numerical guards,
    # not a target-speed limiter; there is no velocity clipping in the plant.
    lo_x, hi_x = [-1e8]*nx, [1e8]*nx
    if nx == 17:
        lo_x[13:17], hi_x[13:17] = [0]*4, [max_n*1.2]*4
    metadata = {"kind": kind, "nx": nx, "N": N, "prediction_dt_s": PRED_DT,
                "control_dt_s": CONTROL_DT, "max_iterations": 30,
                "lbx": lo_x*(N+1)+lower*N, "ubx": hi_x*(N+1)+upper*N,
                "lbg": [0]*(nx*(N+1)), "ubg": [0]*(nx*(N+1)),
                "hover": hover, "lower": lower, "upper": upper,
                "state_count": nx*(N+1), "max_rotor_rad_s": max_n}
    return solver, metadata
