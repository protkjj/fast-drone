"""Why level trim fails, and which design levers reopen it.

This extends `trim_envelope.py` (which established *that* level trim fails between
about 19.6 and 81.1 m/s) by asking *why*, and *what would have to change*. It calls
the same unchanged plant through `LevelTrimAudit`; no separate physics is defined
here. Every function is a diagnostic: none of this is a controller, a trajectory,
a certification, or a proposal to edit the stored profile.

Four questions, in the order a reader should ask them:

1. `normal_balance_roots` — is the trim attitude unique? `trim_envelope` takes a
   single `brentq` root on [0, pi/2]. If a second, high-alpha branch existed, the
   audit would merely have missed it and the vehicle would be fine. It does not.

2. `moment_requirement` — a closed form for the failure. At level trim the normal
   force equals `W*cos(theta)`, so the pitch moment the rotors must supply is
   `|cp| * W * cos(theta)`: set by WEIGHT, nearly independent of speed. Capacity is
   `a * T_total`. Hence trim needs `T_total >= |cp| * W * cos(theta) / a`, a floor
   of about 74% of weight. Mid-speed level flight needs far less thrust than that,
   which is precisely why it is the band that fails.

3. `constant_altitude_window` — level flight is over-determined (attitude is fixed
   by the normal balance alone, leaving no free variable). Releasing the *constant
   speed* requirement while holding altitude restores one degree of freedom: a
   shallower attitude needs more thrust to hold altitude, and the surplus becomes
   forward acceleration. This is the transition-path check left open as step 3 of
   the audit's "next decisions".

4. `lever_threshold` — how far a single design parameter must move to reopen a set
   of speeds. Thresholds are reported, not applied; the stored profile is never
   modified, only deep copies.

Limits, restated because they matter more than the numbers:
  · Still air, zero roll rate, frozen SOC, rigid fixed-pitch rotors, no control
    surfaces, no gusts, no stability or margin claim.
  · `constant_altitude_window` holds altitude exactly. Climbing or diving paths are
    NOT searched, so a closed window is not proof that no transition exists.
  · The 40 A per-motor limit and the equal-share battery budget are assumptions
    (see DESIGN_SOURCE_AUDIT.md §4), not product ratings.
  · The aerodynamic coefficients have no measured angle-of-attack validity range.
"""
import argparse
import copy
import json

import numpy as np
from scipy.optimize import brentq

from research.trim_envelope import LevelTrimAudit


def normal_balance_roots(audit, speed, samples=3001):
    """Sign changes of Fz(theta) - W*cos(theta) on (0, pi/2], on a fixed grid.

    A single root means the level-trim attitude is unique and no high-alpha branch
    was missed. A fixed grid can in principle skip a pair of roots inside one cell;
    this is a dense check, not a proof of uniqueness.
    """
    thetas = np.linspace(1e-6, np.pi/2, samples)
    residual = np.array([float(
        np.asarray(audit.f["aero"](audit.state(speed, t, 1)[:13], np.zeros(3))).ravel()[2]
        - audit.weight*np.cos(t)) for t in thetas])
    crossings = np.where(np.sign(residual[:-1])*np.sign(residual[1:]) < 0)[0]
    return [float(brentq(lambda t: float(
        np.asarray(audit.f["aero"](audit.state(speed, t, 1)[:13], np.zeros(3))).ravel()[2]
        - audit.weight*np.cos(t)), thetas[i], thetas[i+1], xtol=1e-14))
        for i in crossings]


def moment_requirement(audit, speed):
    """Closed form of the pitch-balance constraint at the level-trim point.

    `required = |cp| * W * cos(theta)` is checked against the plant's own moment so
    the identity is verified, not assumed.
    """
    fb = audit.force_balance(speed)
    theta = np.deg2rad(fb["theta_deg"])
    cp = abs(audit.p["cp_from_cg_m"])
    closed_form = cp*audit.weight*np.cos(theta)
    return {
        "speed_mps": float(speed), "theta_deg": fb["theta_deg"],
        "total_thrust_N": fb["total_thrust_N"],
        "required_moment_Nm": abs(fb["required_rotor_pitch_moment_Nm"]),
        "closed_form_moment_Nm": float(closed_form),
        "closed_form_error_Nm": float(abs(closed_form - abs(fb["required_rotor_pitch_moment_Nm"]))),
        "capacity_Nm": fb["positive_thrust_pitch_capacity_Nm"],
        "authority_ratio": fb["pitch_authority_ratio"],
        # Loosest conceivable bound over ALL roll orientations: every newton of
        # thrust placed on the rotor with the longest possible pitch arm, ignoring
        # the roll and yaw balance that would really constrain it. The modelled X
        # layout reaches arm/sqrt(2)*T (model.py:160-162), and that is the best
        # REALISABLE value, because its z-pairs each hold one rotor of each spin
        # direction so yaw and roll cancel at any thrust split. Where even this
        # bound falls short, no roll orientation can rescue the point.
        "orientation_free_upper_bound_Nm": float(audit.p["arm_m"]*fb["total_thrust_N"]),
        "orientation_free_ratio": float(closed_form/(audit.p["arm_m"]*fb["total_thrust_N"])),
        "thrust_floor_N": float(closed_form/audit.pitch_arm),
        "thrust_floor_as_weight_fraction": float(closed_form/audit.pitch_arm/audit.weight),
        # Pitch balance ONLY. Current, voltage, propeller map and tip Mach are not
        # checked here, so 83.3 m/s reports True while `LevelTrimAudit.evaluate`
        # rejects it on current. Never quote this alone as "trim is feasible".
        "pitch_feasible": bool(fb["rotor_thrust_nonnegative"]),
    }


def _constant_altitude_point(audit, speed, theta):
    """Thrust that holds altitude at this attitude, and the acceleration it leaves.

    Vertical:   T*sin(th) + Fx*sin(th) + Fz*cos(th) = W
    Horizontal: m*ax     = T*cos(th) + Fx*cos(th) - Fz*sin(th)
    """
    fa = np.asarray(audit.f["aero"](audit.state(speed, theta, 1)[:13], np.zeros(3))).ravel()
    fx, fz = float(fa[0]), float(fa[2])
    s, c = np.sin(theta), np.cos(theta)
    if s < 1e-9:
        return None
    thrust = (audit.weight - fx*s - fz*c)/s
    if thrust <= 0:
        return None
    swing = abs(audit.p["cp_from_cg_m"])*fz/audit.pitch_arm
    low = (thrust-swing)/4
    if low < -1e-12:
        return None
    return {"theta_rad": float(theta), "total_thrust_N": float(thrust),
            "acceleration_mps2": float((thrust*c + fx*c - fz*s)/audit.p["mass_kg"]),
            "low_pair_thrust_N": float(max(low, 0.0)),
            "high_pair_thrust_N": float((thrust+swing)/4)}


def _rotor_limits_ok(audit, speed, theta, low, high):
    """Rotor speed, current, voltage and propeller-map checks for one thrust pair.

    Mirrors the checks `LevelTrimAudit.evaluate` applies, but at a caller-chosen
    attitude instead of the level-trim one, so accelerating points can be tested.
    """
    axial = speed*np.cos(theta)

    def thrust_of(n):
        return float(audit.f["rotors"]([n]*4, axial)[0][0])

    if max(low, high) > thrust_of(audit.max_n):
        return False
    speeds = np.array([0.0 if t <= 1e-10 else brentq(lambda n: thrust_of(n)-t, 0, audit.max_n)
                       for t in (low, low, high, high)])
    state = audit.state(speed, theta, 1)
    state[13:17] = speeds
    diag = {k: np.asarray(v).ravel() for k, v in audit.f["diag"](
        x=state, u=speeds, env=np.zeros(6), scales=np.ones(6)).items()}
    kt = 60/(2*np.pi*audit.p["motor"]["kv_rpm_V"])
    needed_voltage = kt*speeds + audit.p["motor"]["resistance_ohm"]*diag["requested_current"]
    return not (np.any(diag["requested_current"] > float(diag["current_limit"][0]) + 1e-9)
                or np.any(needed_voltage > float(diag["voltage"][0]) + 1e-8)
                or np.any(diag["outside_map"]))


def constant_altitude_window(audit, speed, samples=900):
    """Attitudes at this speed that hold altitude within every modelled limit.

    Returns the surviving acceleration interval. An empty window means no constant
    altitude flight at this speed is possible in the model at ANY attitude, whether
    steady or accelerating; it does not rule out climbing or diving paths.
    """
    ceiling = np.deg2rad(audit.force_balance(speed)["theta_deg"])
    found = []
    for theta in np.linspace(np.deg2rad(0.02), ceiling, samples):
        point = _constant_altitude_point(audit, speed, theta)
        if point and _rotor_limits_ok(audit, speed, theta, point["low_pair_thrust_N"],
                                      point["high_pair_thrust_N"]):
            found.append(point)
    if not found:
        return {"speed_mps": float(speed), "feasible": False, "samples_feasible": 0}
    accelerations = [p["acceleration_mps2"] for p in found]
    return {"speed_mps": float(speed), "feasible": True, "samples_feasible": len(found),
            "samples": samples,
            "min_acceleration_mps2": float(min(accelerations)),
            "max_acceleration_mps2": float(max(accelerations)),
            "min_total_thrust_N": float(min(p["total_thrust_N"] for p in found)),
            "max_total_thrust_N": float(max(p["total_thrust_N"] for p in found))}


def combined_authority(audit, speed, crosswind=0.0):
    """Share of pitch authority consumed by trim plus a steady crosswind.

    Four non-negative rotor thrusts summing to T can produce `My = a*P` and
    `Mz = -a*Q` only while `T >= max(|P+Q|, |P-Q|) = |P| + |Q|`: pitch and yaw
    demands ADD, so a crosswind stacks on top of what trim already spends.

    The attitude is held at the still-air trim value, so this is the moment needed
    to KEEP THAT HEADING, not a survival criterion. A statically stable body will
    otherwise weathervane, which drives sideslip back to zero at a crab angle.
    Wind enters through the plant's own aerodynamics function, not a copy of it.
    """
    theta = np.deg2rad(audit.force_balance(speed)["theta_deg"])
    wind = np.array([0.0, float(crosswind), 0.0])
    force = np.asarray(audit.f["aero"](audit.state(speed, theta, 1)[:13], wind)).ravel()
    offset = audit.p["cp_from_cg_m"]
    pitch_moment, yaw_moment = -offset*float(force[2]), offset*float(force[1])
    total_thrust = audit.weight*np.sin(theta) - float(force[0])
    needed = (abs(pitch_moment) + abs(yaw_moment))/audit.pitch_arm
    return {"speed_mps": float(speed), "crosswind_mps": float(crosswind),
            "sideslip_deg": float(np.degrees(np.arctan2(crosswind, speed))),
            "total_thrust_N": float(total_thrust),
            "thrust_needed_for_moments_N": float(needed),
            "authority_used": float(needed/total_thrust) if total_thrust > 0 else float("inf"),
            "holds_heading": bool(needed <= total_thrust)}


def crosswind_limit(audit, speed, upper=60.0):
    """Crosswind at which heading can no longer be held, by bisection."""
    if not combined_authority(audit, speed, 0.0)["holds_heading"]:
        return 0.0
    if combined_authority(audit, speed, upper)["holds_heading"]:
        return None
    return float(brentq(lambda w: combined_authority(audit, speed, w)["authority_used"] - 1,
                        1e-3, upper, xtol=1e-6))


def max_constant_altitude_speed(audit, low=5.0, high=100.0, tolerance=0.01, samples=900):
    """Bisect the highest speed with a non-empty constant-altitude window.

    `low` must be feasible and `high` infeasible; the bracket is checked, and the
    result is only as fine as `samples` makes each window test.
    """
    if not constant_altitude_window(audit, low, samples)["feasible"]:
        raise ValueError("Lower bracket speed has no feasible window.")
    if constant_altitude_window(audit, high, samples)["feasible"]:
        raise ValueError("Upper bracket speed is feasible; widen the bracket.")
    while high - low > tolerance:
        middle = (low + high)/2
        if constant_altitude_window(audit, middle, samples)["feasible"]:
            low = middle
        else:
            high = middle
    return {"max_speed_mps": float(low), "max_speed_kmh": float(low*3.6),
            "bracket_upper_mps": float(high), "tolerance_mps": float(tolerance)}


def lever_threshold(edit, failing, passing, speeds, tolerance, profile=None):
    """Smallest change to one parameter that makes every `speeds` entry feasible.

    `edit(value)` returns a function mutating a profile copy. `failing` is a value
    known to fail and `passing` one known to pass; direction is inferred, so a lever
    that improves downward (offset) and one that improves upward (arm) both work.
    Returns None when even `passing` fails, i.e. the lever cannot do it alone.
    """
    base = LevelTrimAudit().p if profile is None else profile

    def feasible(value):
        candidate = copy.deepcopy(base)
        edit(value)(candidate)
        trial = LevelTrimAudit(candidate)
        return all(trial.evaluate(v, 1)["model_feasible"] for v in speeds)

    if not feasible(passing):
        return None
    if feasible(failing):
        return float(failing)
    while abs(passing - failing) > tolerance:
        middle = (failing + passing)/2
        if feasible(middle):
            passing = middle
        else:
            failing = middle
    return float(passing)


def offset_edit(value):
    """Move the pressure centre toward the CG (a smaller static margin)."""
    return lambda p: p.update(cp_from_cg_m=-abs(value))


def arm_edit(value):
    """Lengthen the rotor arm. In the sizing geometry this also lengthens the fins,
    which moves the pressure centre aft; that coupling is NOT modelled here."""
    return lambda p: p.update(arm_m=value)


def current_edit(value):
    """Raise the per-motor current rating, with the pack budget opened up so the
    motor rating alone binds."""
    def edit(p):
        p["motor"]["current_limit_A"] = value
        p["battery"]["max_C"] = 400
    return edit


FULL_BAND = [0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 83.3]


def make_report(speeds=None):
    speeds = FULL_BAND if speeds is None else list(speeds)
    audit = LevelTrimAudit()
    cruise = [83.3]
    return {
        "schema": "trim-lever-audit-v1", "profile": audit.p["id"],
        "scope": {"still_air": True, "roll_deg": 0, "frozen_soc": 1.0,
                  "constant_altitude_only": True, "control_surfaces": False},
        "limitations": [
            "Diagnostic of one model; not flight validation, stability, or a design proposal.",
            "Climbing and diving transition paths are not searched.",
            "Motor 40 A and the equal-share pack budget are assumptions, not ratings.",
            "Arm and fin span are coupled in the sizing geometry; the arm lever ignores that.",
        ],
        "root_counts": {str(v): len(normal_balance_roots(audit, v))
                        for v in [10, 20, 40, 60, 83.3]},
        "moment": [moment_requirement(audit, v) for v in speeds if v > 0],
        "constant_altitude": [constant_altitude_window(audit, v)
                              for v in [20, 30, 40, 50, 60, 70, 80, 83.3]],
        "max_constant_altitude_speed": max_constant_altitude_speed(audit),
        "thresholds": {
            "cruise_only": {
                "offset_m": lever_threshold(offset_edit, 0.08871, 0.0, cruise, 1e-4),
                "arm_m": lever_threshold(arm_edit, 0.16884, 0.6, cruise, 1e-3),
                "motor_current_A": lever_threshold(current_edit, 40.0, 200.0, cruise, 0.05),
            },
            "full_band": {
                "offset_m": lever_threshold(offset_edit, 0.08871, 0.0, speeds, 1e-4),
                "arm_m": lever_threshold(arm_edit, 0.16884, 1.2, speeds, 1e-3),
                "motor_current_A": lever_threshold(current_edit, 40.0, 400.0, speeds, 0.5),
            },
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=argparse.FileType("x"),
                        help="Optional NEW JSON path; never overwrites.")
    arguments = parser.parse_args()
    report = make_report()
    rendered = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if arguments.output:
        arguments.output.write(rendered)
        print(f"Saved: {arguments.output.name}")
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
