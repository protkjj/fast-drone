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


def build_problem(p, functions, kind, normalize=True, actuator_constraints=False, hybrid_envelope=False):
    """Return the physical NLP in optional dimensionless decision coordinates.

    ``normalize=False`` exists for equivalence tests, not controller tuning.
    Tracking/input costs and actuator bounds describe the same physical problem.
    """
    if kind not in ("hybrid", "nmpc", "f13"):
        raise ValueError(kind)
    nx = 13 if kind in ("hybrid", "f13") else 17
    decision_x = ca.MX.sym("X_scaled", nx, N+1)
    decision_u = ca.MX.sym("U_scaled", 4, N)
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
    elif kind == "f13":
        # 표5 F13: 결정변수가 로터별 추력(N) 직접, [0,max single-rotor thrust]^4.
        max_f = float(functions["rotors"]([max_n]*4, 0)[0][0])
        hover_f = p["mass_kg"]*p["g"]/4
        lower, upper = [0]*4, [max_f]*4
        hover = [hover_f]*4
        scaling = ca.DM([hover_f]*4)
        fun = lambda state, command: functions["f13"](state, command, wind)
    else:
        lower, upper = [0]*4, [max_n]*4
        hover = initial_state(p)[13:17].tolist()
        scaling = ca.DM([max_n]*4)
        fun = lambda state, command: functions["full"](state, command, wind, bus)
        if actuator_constraints:
            fun = lambda state, command: functions["constrained"](state, command, wind, bus)[0]

    state_scale = np.ones(nx)
    if normalize and nx == 17:
        state_scale[13:17] = max_n
    command_scale = np.asarray(scaling).ravel() if normalize else np.ones(4)
    x = ca.diag(ca.DM(state_scale)) @ decision_x
    u = ca.diag(ca.DM(command_scale)) @ decision_u

    # 5 integration substeps avoid RK4 instability from a 20 ms motor time scale.
    current_limit = min(p["motor"]["current_limit_A"],
                        p["battery"]["capacity_Ah"]*p["battery"]["max_C"]*p["battery"]["esc_efficiency"]/4)
    voltage_scale = p["battery"]["series"]*4.2
    def integrate(state, command):
        actuator = []
        def rhs(at):
            if kind == "nmpc" and actuator_constraints:
                derivative,current,voltage = functions["constrained"](at,command,wind,bus)
                # Check EVERY RK4 stage, not just prediction node endpoints.
                actuator.extend([current/current_limit,(bus-voltage)/voltage_scale])
                return derivative
            return fun(at,command)
        for _ in range(5):
            h = PRED_DT/5
            k1 = rhs(state)
            k2 = rhs(state+h/2*k1)
            k3 = rhs(state+h/2*k2)
            k4 = rhs(state+h*k3)
            state = state+h/6*(k1+2*k2+2*k3+k4)
            state = ca.vertcat(state[:6], state[6:10]/ca.norm_2(state[6:10]), state[10:])
        return state, ca.vertcat(*actuator)

    # Keep one transition graph. Reusing it at all nodes avoids serializing 20
    # copies of the integrator and its second derivatives into the WASM bundle.
    one_x, one_u = ca.MX.sym("one_x", nx), ca.MX.sym("one_u", 4)
    transition = ca.Function("prediction_step", [one_x, one_u, wind, bus], list(integrate(one_x, one_u)))
    actuator = []
    eq = [(x[:, 0]-x0)/ca.DM(state_scale)]
    cost = 0
    previous = prev
    for k in range(N+1):
        error = x[3:6, k]-refs[:3, k]
        stage = 5*ca.sumsqr(error) + 20*(x[2,k]-refs[3,k])**2 + ca.sumsqr(x[10:13,k])
        cost += (10 if k == N else 1)*stage
        if k < N:
            nxt,limits=transition(x[:,k],u[:,k],wind,bus)
            eq.append((x[:, k+1]-nxt)/ca.DM(state_scale));actuator.append(limits)
            # Dimensionless penalties, fixed for every profile and test case.
            cost += .02*ca.sumsqr((u[:,k]-ca.DM(hover))/scaling)
            cost += .1*ca.sumsqr((u[:,k]-previous)/scaling)
            previous = u[:,k]
    variables = ca.vertcat(ca.vec(decision_x), ca.vec(decision_u))
    parameters = ca.vertcat(x0, ca.vec(refs), prev, wind, bus)
    # JSON has no Infinity. Generous finite state bounds are numerical guards,
    # not a target-speed limiter; there is no velocity clipping in the plant.
    lo_x, hi_x = [-1e8]*nx, [1e8]*nx
    if nx == 17:
        lo_x[13:17], hi_x[13:17] = [0]*4, [max_n*1.2]*4
    lo_x, hi_x = (np.asarray(lo_x)/state_scale).tolist(), (np.asarray(hi_x)/state_scale).tolist()
    lo_u, hi_u = (np.asarray(lower)/command_scale).tolist(), (np.asarray(upper)/command_scale).tolist()
    metadata = {"kind": kind, "nx": nx, "N": N, "prediction_dt_s": PRED_DT,
                "control_dt_s": CONTROL_DT, "max_iterations": 30,
                "lbx": lo_x*(N+1)+lo_u*N, "ubx": hi_x*(N+1)+hi_u*N,
                "lbg": [0]*(nx*(N+1)), "ubg": [0]*(nx*(N+1)),
                "hover": hover, "lower": lower, "upper": upper,
                "state_scaling": state_scale.tolist(), "command_scaling": command_scale.tolist(),
                "state_count": nx*(N+1), "max_rotor_rad_s": max_n}
    constraints = ca.vertcat(*eq)
    metadata["constraint_scaling"] = state_scale.tolist()*(N+1)
    if kind == "nmpc" and actuator_constraints:
        constraints = ca.vertcat(constraints,*actuator)
        metadata["lbg"] += [0]*160*N
        # At each of 5*4 RK4 stages: four currents <= limit, four voltage
        # headrooms >= 0. Upper headroom is only a generous numerical guard.
        metadata["ubg"] += ([1]*4+[1e8]*4)*20*N
        metadata["constraint_scaling"] += ([current_limit]*4+[voltage_scale]*4)*20*N
    metadata["actuator_constraints"] = bool(kind == "nmpc" and actuator_constraints)
    if kind == "hybrid" and hybrid_envelope:
        # INDI supplies a local coupled virtual-input envelope, not motor commands
        # for NMPC to optimize. Four affine inequalities constrain ONLY u[:,0].
        # The other 19 inputs retain the original virtual prediction model.
        # Parameters: row-major inverse effectiveness (16), virtual offset (4).
        interface = ca.MX.sym("actuator_interface", 20)
        force = ca.vertcat(*[ca.dot(interface[4*i:4*i+4], u[:,0]-interface[16:20])
                            for i in range(4)])
        weight = p["mass_kg"]*p["g"]
        metadata["hybrid_envelope"] = {"parameter_size": 20,
            "constraint_start": int(constraints.numel()), "force_scale_N": weight,
            "horizon_s": CONTROL_DT, "kind": "local-first-move-force-envelope-v1"}
        parameters = ca.vertcat(parameters, interface)
        constraints = ca.vertcat(constraints, force/weight)
        # Runtime replaces these four bounds with the measured-state envelope.
        # Zero interface / loose bounds explicitly disables it for ablation.
        metadata["lbg"] += [-1e8]*4
        metadata["ubg"] += [1e8]*4
        metadata["constraint_scaling"] += [weight]*4
    return {"x": variables, "p": parameters, "f": cost, "g": constraints}, metadata


def build_solver(p, functions, kind):
    problem, metadata = build_problem(p, functions, kind, actuator_constraints=True, hybrid_envelope=True)
    opts = {"ipopt.print_level": 0, "print_time": False, "ipopt.sb": "yes",
            "ipopt.max_iter": 30, "ipopt.tol": 1e-5, "ipopt.acceptable_tol": 1e-4,
            # Normalized constraints are checked in physical units by runtime.js.
            "ipopt.constr_viol_tol": 1e-7, "ipopt.acceptable_constr_viol_tol": 1e-7,
            "ipopt.mu_strategy": "adaptive", "ipopt.warm_start_init_point": "yes",
            "ipopt.warm_start_bound_push": 1e-6, "ipopt.warm_start_mult_bound_push": 1e-6,
            "error_on_fail": False}
    solver = ca.nlpsol(kind, "ipopt", problem, opts)
    return solver, metadata
