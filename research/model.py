"""One CasADi equation source for Python and WebAssembly research runs.

Mechanical state: [p, v, q_xyzw, body_rate, rotor_rad_s] (17).
The plant has one additional *battery* state, SOC (18 total, not 18 airframe
states). Controllers see 13 or 17 mechanical states, never hidden plant values.
"""
import copy
from functools import lru_cache
import json
from pathlib import Path

import casadi as ca
import numpy as np

from control.dynamics import _quat_to_rotmat as rotation, _quat_derivative as qdot

DT = .001
DIRS = ca.DM([1, -1, 1, -1])


def profile(name="selected"):
    p = json.loads((Path(__file__).parent / "profiles/selected.json").read_text())
    if name == "selected":
        return p
    if name != "simple":
        raise ValueError(name)
    p = copy.deepcopy(p)
    p.update(id="simple-passive", label="단순 검증 기체 (가정)", mass_kg=2,
             inertia_kg_m2=[.02, .04, .04], arm_m=.2, cp_from_cg_m=0)
    p["prop"] = {"diameter_m": .25, "tip_mach_limit": .7, "simple": True}
    p["aero"].update(CN_alpha=0, CN_cross=0, CD0=[.2] * len(p["aero"]["CD0"]))
    p["motor"].update(kv_rpm_V=900, resistance_ohm=.10, i0_A=.5,
                      rotor_inertia_kg_m2=3e-5, current_limit_A=30)
    p["provenance"] = {"classification": "assumed-unit-test-model"}
    p["assumptions"] = ["Entire simple model is assumed for unit/architecture tests, not the selected aircraft.",
                         "Body +x thrust, world +z up, scalar-last quaternion; 2 kg mass.",
                         "Rotor inertia 3e-5 kg m2, motor 900 rpm/V, 30 A cap, 20 ms speed loop.",
                         "Passive analytic propeller and drag; no thermal model or regenerative braking.",
                         "Battery and sensor parameters are explicit test assumptions."]
    return p


def clip(value, lower, upper):
    return ca.fmin(ca.fmax(value, lower), upper)


@lru_cache(maxsize=32)
def _interpolator(grid, values):
    return ca.interpolant("lookup", "linear", [list(grid)], list(values))


def table(value, grid, values):
    """The same CasADi linear-interpolation plugin exists on Python and WASM."""
    return _interpolator(tuple(grid), tuple(values))(clip(value, grid[0], grid[-1]))


def aero(vb, rate, p):
    speed2 = ca.sumsqr(vb)
    speed = ca.sqrt(speed2 + 1e-12)
    cross_speed = ca.sqrt(ca.sumsqr(vb[1:3]) + 1e-12)
    a = p["aero"]
    cd = table(speed, a["speed_mps"], a["CD0"])
    # Smooth odd axial drag; normal extension dissipates energy in every quadrant.
    fx = -.5*p["rho"]*p["area_m2"]*speed2*cd*vb[0]/ca.sqrt(vb[0]**2 + 1e-6)
    normal = .5*p["rho"]*p["area_m2"]*(a["CN_alpha"]*ca.sqrt(vb[0]**2 + 1e-12)
                                             + a["CN_cross"]*cross_speed)
    force = ca.vertcat(fx, -normal*vb[1], -normal*vb[2])
    moment = ca.cross(ca.DM([p["cp_from_cg_m"], 0, 0]), force)
    moment += .25*p["rho"]*speed*p["area_m2"]*p["body_diameter_m"]**2*ca.DM(a["damping"])*rate
    return force, moment


def propellers(n, axial, p, scale=1):
    d = p["prop"]["diameter_m"]
    thrust, torque, advance, outside = [], [], [], []
    for i in range(4):
        rev = ca.fmax(n[i], 0)/(2*np.pi)
        j = ca.fmax(axial, 0)/(rev*d + 1e-8)
        if p["prop"].get("simple"):
            ct = .10 / (1 + j*j)
            # P >= T*V plus positive induced/profile power by construction.
            cp = j*ct + .05/(1 + j*j)
            invalid = axial < -.1
        else:
            ct_raw = table(j, p["prop"]["J"], p["prop"]["CT"])
            cp_raw = table(j, p["prop"]["J"], p["prop"]["CP"])
            # Only the motoring quadrant is claimed as reproduced. Outside it,
            # use a passive zero-thrust continuation and REPORT the sample.
            ct = ca.fmax(ct_raw, 0)
            cp = ca.fmax(cp_raw, j*ct + .005)
            invalid = ca.logic_or(axial < -.1, ca.logic_or(j > p["prop"]["J"][-1],
                                                          ca.logic_or(ct_raw < 0, cp_raw < j*ct)))
        thrust.append(scale*ct*p["rho"]*rev**2*d**4)
        torque.append(scale*cp*p["rho"]*rev**2*d**5/(2*np.pi))
        advance.append(j)
        outside.append(ca.if_else(ca.logic_and(rev < 1, ca.fabs(axial) < .1), 0, invalid))
    return tuple(ca.vertcat(*v) for v in (thrust, torque, advance, outside))


def motors(n, cmd, torque, voltage, p):
    m, b = p["motor"], p["battery"]
    kt = 60/(2*np.pi*m["kv_rpm_V"])
    friction = kt*m["i0_A"]*n/ca.sqrt(n*n + 1)
    desired_torque = torque + friction + m["rotor_inertia_kg_m2"]*(cmd-n)/m["speed_loop_tau_s"]
    # No regeneration: negative requested torque means passive coasting.
    limit = min(m["current_limit_A"], b["capacity_Ah"]*b["max_C"]*b["esc_efficiency"]/4)
    request = clip(desired_torque/kt, 0, limit)
    emf = kt*n
    voltage_cap = ca.fmax((voltage-emf)/m["resistance_ohm"], 0)
    current = ca.fmin(request, voltage_cap)
    winding_voltage = emf + m["resistance_ohm"]*current
    electrical = ca.dot(winding_voltage, current)/b["esc_efficiency"]
    dn = (kt*current - torque-friction)/m["rotor_inertia_kg_m2"]
    return dn, current, electrical, friction


def battery_voltage(n, cmd, torque, soc, p):
    b = p["battery"]
    voc = b["series"]*table(soc, [0, .5, 1], [3.4, 3.7, 4.2])
    voltage = voc
    # Newton on the high-voltage branch: V^2 - Voc V + R P(V) = 0.
    # Motor current is voltage-limited inside P(V); we don't draw impossible power.
    z = ca.SX.sym("voltage")
    power_expr = motors(n, cmd, torque, z, p)[2]
    dp = ca.jacobian(power_expr, z)
    f = ca.Function("bus_power", [z, n, cmd, torque], [power_expr, dp])
    for _ in range(8):
        power, derivative = f(voltage, n, cmd, torque)
        residual = voltage**2 - voc*voltage + b["resistance_ohm"]*power
        gradient = 2*voltage-voc+b["resistance_ohm"]*derivative
        voltage = clip(voltage-residual/ca.fmax(gradient, 1e-6), .5*voc, voc)
    return voltage, voc


def motor_limits(n, cmd, torque, voltage, p):
    """Diagnostic limits; these never change the physical saturation equations."""
    m, b = p["motor"], p["battery"]
    kt = 60/(2*np.pi*m["kv_rpm_V"])
    friction = kt*m["i0_A"]*n/ca.sqrt(n*n+1)
    requested = (torque+friction+m["rotor_inertia_kg_m2"]*(cmd-n)/m["speed_loop_tau_s"])/kt
    limit = min(m["current_limit_A"], b["capacity_Ah"]*b["max_C"]*b["esc_efficiency"]/4)
    cap = ca.fmax((voltage-kt*n)/m["resistance_ohm"], 0)
    bounded = clip(requested, 0, limit)
    actual = ca.fmin(bounded, cap)
    return [requested, ca.DM(limit), cap, requested>limit+1e-8,
            bounded>cap+1e-8, requested < -1e-8, ca.fabs(actual-requested)>1e-8]


def mechanical(x, dn, thrust, torque, wind, moment_disturbance, p, scales):
    r = rotation(x[6:10])
    rate, n = x[10:13], x[13:17]
    vb = r.T @ (x[3:6]-wind)
    force, moment = aero(vb, rate, p)
    a = p["arm_m"]/np.sqrt(2)
    y, z = ca.DM([a, -a, -a, a]), ca.DM([a, a, -a, -a])
    moment += ca.vertcat(ca.dot(DIRS, torque), ca.dot(z, thrust), -ca.dot(y, thrust))
    # Rotor angular momentum is opposite the airframe reaction torque.
    h = ca.vertcat(-p["motor"]["rotor_inertia_kg_m2"]*ca.dot(DIRS, n), 0, 0)
    hdot = ca.vertcat(-p["motor"]["rotor_inertia_kg_m2"]*ca.dot(DIRS, dn), 0, 0)
    inertia = ca.DM(p["inertia_kg_m2"])*scales[1:4]
    alpha = (moment + moment_disturbance - ca.cross(rate, inertia*rate+h) - hdot)/inertia
    acceleration = r @ (force + ca.vertcat(ca.sum1(thrust), 0, 0))/(p["mass_kg"]*scales[0])
    acceleration += ca.DM([0, 0, -p["g"]])
    return ca.vertcat(x[3:6], acceleration, qdot(x[6:10], rate), alpha, dn), force, moment


def build(p):
    x = ca.SX.sym("state", 18)
    cmd = ca.SX.sym("rotor_command", 4)
    environment = ca.SX.sym("environment", 6)  # wind world xyz, applied moment body xyz
    scales = ca.SX.sym("plant_scales", 5)  # m, Ixx, Iyy, Izz, CT/CP; nominal controller uses 1
    n, soc = x[13:17], x[17]
    vb = rotation(x[6:10]).T @ (x[3:6]-environment[:3])
    thrust, torque, advance, outside = propellers(n, vb[0], p, scales[4])
    # Helpers require primitive symbols; substitute the physical torque afterwards.
    nn, uu, qq, ss = ca.SX.sym("n", 4), ca.SX.sym("u", 4), ca.SX.sym("Q", 4), ca.SX.sym("soc")
    vv, ocv = battery_voltage(nn, uu, qq, ss, p)
    bus = ca.Function("bus", [nn, uu, qq, ss], [vv, ocv])
    voltage, voc = bus(n, cmd, torque, soc)
    dn, current, power, friction = motors(n, cmd, torque, voltage, p)
    dx17, force, moment = mechanical(x[:17], dn, thrust, torque, environment[:3], environment[3:], p, scales)
    ibus = power/voltage
    dsoc = -ibus/(3600*p["battery"]["capacity_Ah"])
    rhs = ca.Function("plant_rhs", [x, cmd, environment, scales], [ca.vertcat(dx17, dsoc)])
    kin = p["motor"]["rotor_inertia_kg_m2"]*ca.dot(n, dn)
    shaft = ca.dot(torque, n)
    copper = p["motor"]["resistance_ohm"]*ca.sumsqr(current)
    friction_power = ca.dot(friction, n)
    outputs = [thrust, torque, current, dn, voltage, ibus, power, shaft, copper,
               friction_power, kin, power*p["battery"]["esc_efficiency"]-shaft-copper-friction_power-kin,
               voc-voltage-p["battery"]["resistance_ohm"]*ibus, outside, advance, force, moment]
    names = ["thrust", "torque", "current", "dn", "voltage", "ibus", "power", "shaft", "copper",
             "friction", "rotor_kinetic_rate", "motor_energy_residual", "bus_residual", "outside_map", "J", "aero_force", "moment"]
    outputs += motor_limits(n, cmd, torque, voltage, p)
    names += ["requested_current", "current_limit", "voltage_current_cap", "current_limited",
              "voltage_limited", "coasting", "tracking_limited"]
    diag = ca.Function("diagnostics", [x, cmd, environment, scales], outputs,
                       ["x", "u", "env", "scales"], names)
    k1 = rhs(x, cmd, environment, scales)
    k2 = rhs(x+DT/2*k1, cmd, environment, scales)
    k3 = rhs(x+DT/2*k2, cmd, environment, scales)
    k4 = rhs(x+DT*k3, cmd, environment, scales)
    nxt = x+DT/6*(k1+2*k2+2*k3+k4)
    nxt[6:10] /= ca.norm_2(nxt[6:10])
    step = ca.Function("plant_step", [x, cmd, environment, scales], [nxt])

    # Standalone NMPC predicts motors, including electrical/current limits. Bus
    # voltage is measured and held over the 1 s horizon, not secretly read ahead.
    xm, bus_v = ca.SX.sym("mechanical", 17), ca.SX.sym("bus_voltage")
    vbm = rotation(xm[6:10]).T @ (xm[3:6]-environment[:3])
    tm, qm, _, _ = propellers(xm[13:17], vbm[0], p)
    dnm = motors(xm[13:17], cmd, qm, bus_v, p)[0]
    full_rhs = mechanical(xm, dnm, tm, qm, environment[:3], ca.DM.zeros(3), p, ca.DM.ones(5))[0]
    full = ca.Function("full_prediction", [xm, cmd, environment[:3], bus_v], [full_rhs])

    # A constraint-form motor predictor avoids differentiating a nested current
    # clamp inside IPOPT. It is identical to the plant when requested current
    # and winding voltage satisfy the actuator constraints checked by the NLP.
    # The actual plant above ALWAYS retains its physical saturation model.
    motor = p["motor"]
    kt = 60/(2*np.pi*motor["kv_rpm_V"])
    friction_m = kt*motor["i0_A"]*xm[13:17]/ca.sqrt(xm[13:17]**2+1)
    free_dn = (cmd-xm[13:17])/motor["speed_loop_tau_s"]
    requested_current = (qm+friction_m+motor["rotor_inertia_kg_m2"]*free_dn)/kt
    required_voltage = kt*xm[13:17]+motor["resistance_ohm"]*requested_current
    free_rhs = mechanical(xm,free_dn,tm,qm,environment[:3],ca.DM.zeros(3),p,ca.DM.ones(5))[0]
    constrained = ca.Function("constraint_form_prediction", [xm,cmd,environment[:3],bus_v],
                              [free_rhs,requested_current,required_voltage])

    xv, virtual = ca.SX.sym("virtual_state", 13), ca.SX.sym("virtual_command", 4)
    rv = rotation(xv[6:10])
    fv, _ = aero(rv.T @ (xv[3:6]-environment[:3]), xv[10:13], p)
    dxv = ca.vertcat(xv[3:6], rv @ (fv + ca.vertcat(virtual[0], 0, 0))/p["mass_kg"]
                    + ca.DM([0, 0, -p["g"]]), qdot(xv[6:10], xv[10:13]), virtual[1:])
    vf = ca.Function("virtual_prediction", [xv, virtual, environment[:3]], [dxv])
    # INDI allocation is differentiated from this SAME nominal propulsion map.
    pn, pa = ca.SX.sym("rpm", 4), ca.SX.sym("axial")
    ti, qi, _, _ = propellers(pn, pa, p)
    a = p["arm_m"]/np.sqrt(2)
    mi = ca.vertcat(ca.dot(DIRS, qi), ca.dot(ca.DM([a,a,-a,-a]), ti),
                    -ca.dot(ca.DM([a,-a,-a,a]), ti))
    wrench = ca.vertcat(ca.sum1(ti), mi/ca.DM(p["inertia_kg_m2"]))
    effect = ca.Function("effectiveness", [pn, pa], [wrench, ca.jacobian(wrench, pn)])
    rotors = ca.Function("rotor_channels", [pn, pa],
                        [ti, qi, ca.diag(ca.jacobian(ti,pn)), ca.diag(ca.jacobian(qi,pn))])
    # Invert the SAME propulsion graph. The allocator does not differentiate
    # through bisection decisions: it uses dQ/dn / dT/dn for its force Jacobian.
    desired, axial, maximum = ca.MX.sym("rotor_thrust",4), ca.MX.sym("inflow"), ca.MX.sym("maximum_rad_s")
    low, high = ca.MX.zeros(4), ca.repmat(maximum,4,1)
    for _ in range(28):
        middle = (low+high)/2
        below = rotors(middle,axial)[0] < desired
        low = ca.if_else(below,middle,low)
        high = ca.if_else(below,high,middle)
    cap = rotors(ca.repmat(maximum,4,1),axial)[0]
    inverse = ca.Function("inverse_rotor_thrust", [desired,axial,maximum],
                          [ca.if_else(desired<=1e-12,0,
                           ca.if_else(desired>=cap,maximum,(low+high)/2))])
    # A nominal, frozen-inflow / measured-voltage actuator forecast. Reuse the
    # physical motor equations (including passive coasting), not a fitted slew
    # constant. This is an endpoint capability estimate, NOT a full-airframe or
    # future battery-voltage guarantee. No SOC or hidden plant scales are inputs.
    motor_rhs = ca.Function("actuator_rhs", [pn, cmd, pa, bus_v],
                            [motors(pn, cmd, qi, bus_v, p)[0]])
    k1 = motor_rhs(pn, cmd, pa, bus_v)
    k2 = motor_rhs(pn+DT/2*k1, cmd, pa, bus_v)
    k3 = motor_rhs(pn+DT/2*k2, cmd, pa, bus_v)
    k4 = motor_rhs(pn+DT*k3, cmd, pa, bus_v)
    motor_step = ca.Function("actuator_step", [pn, cmd, pa, bus_v],
                            [pn+DT/6*(k1+2*k2+2*k3+k4)])
    af = ca.Function("aerodynamics", [xv, environment[:3]], [fv])
    return {"rhs": rhs, "step": step, "diag": diag, "full": full, "constrained": constrained,
            "virtual": vf, "effect": effect, "aero": af, "rotors": rotors,
            "inverse_thrust": inverse, "motor_step": motor_step}


def initial_state(p, altitude=20):
    ct = .1 if p["prop"].get("simple") else p["prop"]["CT"][0]
    n = 2*np.pi*np.sqrt(p["mass_kg"]*p["g"]/(4*ct*p["rho"]*p["prop"]["diameter_m"]**4))
    return np.array([0,0,altitude,0,0,0,0,-np.sqrt(.5),0,np.sqrt(.5),0,0,0,n,n,n,n,1.0])


def rpm_limit(p):
    # Command limit; true rotor motion is integrated, never clipped to a desired speed.
    tip = 2*p["prop"]["tip_mach_limit"]*p["sound_speed_mps"]/p["prop"]["diameter_m"]
    no_load = p["motor"]["kv_rpm_V"]*p["battery"]["series"]*4.2*2*np.pi/60
    return min(tip, no_load)
