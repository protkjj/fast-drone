"""Offline level-flight audit using the *unchanged* common research plant.

This is not a controller, trajectory planner, or flight-envelope certification.
We test still air, positive forward speed, zero roll/body rate, horizontal flight,
and frozen-SOC operating points. SOC still decreases in the checked plant RHS.

Body +x is thrust. With nose-up angle theta, body air velocity is
[V*cos(theta), 0, -V*sin(theta)]. Required body force is
[W*sin(theta), 0, W*cos(theta)]. Consequently the normal-force equation alone
determines theta; axial balance then determines total thrust. Pitch balance
determines the two rotor pairs. Negative required thrust is reported, not clipped.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

from research.model import build, initial_state, profile, rpm_limit


class LevelTrimAudit:
    def __init__(self, p=None):
        self.p = copy.deepcopy(profile() if p is None else p)
        self.f = build(self.p)
        self.weight = self.p["mass_kg"]*self.p["g"]
        self.pitch_arm = self.p["arm_m"]/np.sqrt(2)
        self.max_n = rpm_limit(self.p)

    def validate(self, speed, soc):
        if not np.isfinite(speed) or not 0 <= speed <= self.p["aero"]["speed_mps"][-1]:
            raise ValueError("Speed must be finite and inside the aerodynamic speed table.")
        if not np.isfinite(soc) or not 0 <= soc <= 1:
            raise ValueError("SOC must be finite and between 0 and 1.")

    def state(self, speed, theta, soc):
        x = initial_state(self.p)
        x[3] = speed
        x[6:10] = [0, -np.sin(theta/2), 0, np.cos(theta/2)]
        x[17] = soc
        return x

    def force_balance(self, speed):
        """Solve force balance, then derive the necessary signed rotor forces."""
        self.validate(speed, 1)

        def force(theta):
            x = self.state(speed, theta, 1)
            return np.asarray(self.f["aero"](x[:13], np.zeros(3))).ravel()

        def normal_residual(theta):
            return force(theta)[2] - self.weight*np.cos(theta)

        # At hover alpha is undefined. A vertical attitude is still well-defined.
        # The second case also handles the simple model with zero normal force.
        if speed == 0 or abs(normal_residual(np.pi/2)) < 1e-12:
            theta = np.pi/2
        else:
            theta = brentq(normal_residual, 0, np.pi/2, xtol=1e-14)
        fa = force(theta)
        total = self.weight*np.sin(theta) - fa[0]
        aero_pitch = -self.p["cp_from_cg_m"]*fa[2]
        # z=[+a,+a,-a,-a]; rotor pitch moment = a*(T1+T2-T3-T4).
        # The paired solution cancels both roll reaction torque and yaw moment.
        pair_positive_z = (total-aero_pitch/self.pitch_arm)/4
        pair_negative_z = (total+aero_pitch/self.pitch_arm)/4
        required = [pair_positive_z]*2 + [pair_negative_z]*2
        return {
            "speed_mps": float(speed), "speed_kmh": float(speed*3.6),
            "theta_deg": float(np.degrees(theta)),
            "alpha_deg": float(np.degrees(theta)) if speed > 0 else None,
            "aero_force_body_N": fa.tolist(), "total_thrust_N": float(total),
            "required_rotor_pitch_moment_Nm": float(-aero_pitch),
            "positive_thrust_pitch_capacity_Nm": float(self.pitch_arm*total),
            "pitch_authority_ratio": float(abs(aero_pitch)/(self.pitch_arm*total)),
            "required_rotor_thrust_N": [float(t) for t in required],
            "rotor_thrust_nonnegative": bool(min(required) >= -1e-10),
            "max_abs_cp_offset_for_positive_thrust_m":
                float(self.pitch_arm*total/abs(fa[2])) if abs(fa[2]) > 1e-10 else None,
        }

    def evaluate(self, speed, soc=1):
        self.validate(speed, soc)
        result = self.force_balance(speed)
        reasons = []
        result.update(soc=float(soc), model_feasible=False, rotor_rpm=None,
                      rejection_reasons=reasons)
        if soc < self.p["battery"]["minimum_soc"]:
            reasons.append("below_minimum_soc")
        if not result["rotor_thrust_nonnegative"]:
            reasons.append("negative_rotor_thrust_required")
            return result

        theta = np.deg2rad(result["theta_deg"])
        axial = speed*np.cos(theta)

        def thrust(n):
            return float(self.f["rotors"]([n]*4, axial)[0][0])

        cap = thrust(self.max_n)
        result["rotor_thrust_at_speed_limit_N"] = cap
        if max(result["required_rotor_thrust_N"]) > cap+1e-10:
            reasons.append("rotor_speed_limit")
            return result
        # A boundary value inside roundoff tolerance can be treated as zero;
        # physically negative solutions were already rejected above.
        n = np.array([0 if t <= 1e-10 else brentq(lambda n: thrust(n)-t, 0, self.max_n)
                      for t in result["required_rotor_thrust_N"]])
        x = self.state(speed, theta, soc)
        x[13:17] = n
        d = {k: np.asarray(v).ravel() for k, v in self.f["diag"](
            x=x, u=n, env=np.zeros(6), scales=np.ones(6)).items()}
        dx = np.asarray(self.f["rhs"](x, n, np.zeros(6), np.ones(6))).ravel()
        motor = self.p["motor"]
        kt = 60/(2*np.pi*motor["kv_rpm_V"])
        required_voltage = kt*n + motor["resistance_ohm"]*d["requested_current"]
        # Diagnostic only: the runtime currently limits rotational tip speed.
        # Match the sizing project's total-V helical approximation, not a full
        # azimuth-resolved crossflow/induced-velocity blade model.
        tip = np.hypot(speed, n*self.p["prop"]["diameter_m"]/2)/self.p["sound_speed_mps"]
        result.update(
            rotor_rpm=(n*60/(2*np.pi)).tolist(),
            required_motor_current_A=d["requested_current"].tolist(),
            actual_motor_current_A=d["current"].tolist(),
            effective_motor_current_limit_A=float(d["current_limit"][0]),
            required_motor_voltage_V=required_voltage.tolist(),
            bus_voltage_V=float(d["voltage"][0]), bus_current_A=float(d["ibus"][0]),
            electrical_power_W=float(d["power"][0]),
            propeller_advance_ratio=d["J"].tolist(), outside_propeller_map=d["outside_map"].astype(bool).tolist(),
            helical_tip_mach=tip.tolist(),
            max_acceleration_residual_mps2=float(max(abs(dx[3:6]))),
            max_angular_acceleration_residual_radps2=float(max(abs(dx[10:13]))),
            max_rotor_acceleration_residual_radps2=float(max(abs(dx[13:17]))),
            soc_rate_per_s=float(dx[17]),
        )
        if np.any(d["current_limited"]):
            reasons.append("current_limit")
        if np.any(required_voltage > d["voltage"][0]+1e-8):
            reasons.append("voltage_limit")
        if np.any(d["outside_map"]):
            reasons.append("outside_propeller_map")
        if max(tip) > self.p["prop"]["tip_mach_limit"]+1e-9:
            reasons.append("helical_tip_mach_limit")
        if (max(abs(dx[3:6])) > 1e-7 or max(abs(dx[10:13])) > 1e-6
                or max(abs(dx[13:17])) > 1e-5):
            reasons.append("nonzero_steady_state_residual")
        result["model_feasible"] = not reasons
        return result

    def pitch_boundaries(self, maximum_speed):
        """Roots bracketing sign changes on a 0.5 m/s grid, not global certification."""
        self.validate(maximum_speed, 1)

        def margin(speed):
            return min(self.force_balance(speed)["required_rotor_thrust_N"])

        grid = np.linspace(0, maximum_speed, max(2, int(np.ceil(maximum_speed/.5))+1))
        values = [margin(v) for v in grid]
        roots = []
        for i in range(len(grid)-1):
            if values[i] == 0:
                roots.append(float(grid[i]))
            elif values[i]*values[i+1] < 0:
                roots.append(float(brentq(margin, grid[i], grid[i+1], xtol=1e-10)))
        if values[-1] == 0:
            roots.append(float(grid[-1]))
        return roots


def make_report(speeds=None, socs=None):
    speeds = [0, 5, 10, 15, 20, 30, 40, 60, 80, 83.3, 90, 100] if speeds is None else list(speeds)
    socs = [1, .5, .2] if socs is None else list(socs)
    if not speeds or not socs:
        raise ValueError("At least one speed and SOC are required.")
    audit = LevelTrimAudit()
    root = Path(__file__).parent
    return {
        "schema": "level-trim-audit-v1", "profile": audit.p["id"],
        "source_hashes": {name: hashlib.sha256((root/name).read_bytes()).hexdigest()
                          for name in ["model.py", "profiles/selected.json", "trim_envelope.py"]},
        "scope": {"wind_mps": [0, 0, 0], "roll_deg": 0, "body_rate_radps": [0, 0, 0],
                  "flight_path_deg": 0, "acceleration_mps2": [0, 0, 0],
                  "attitude_range_deg": [0, 90], "frozen_soc_operating_points": socs},
        "aerodynamics_validated": False,
        "limitations": [
            "Model-feasible means algebraic balance and the listed model limits only, not real-flight validation.",
            "No empirical alpha-validity interval, propeller crossflow/thermal validation, or stability proof.",
            "No transition trajectory, arbitrary-roll search, gusts, or actuator reserve margin is certified.",
            "A failed level trim is not proof that an accelerating/climbing transition is impossible.",
            "Power/current at a rejected point include rotor deceleration; they are not steady cruise consumption.",
            "Helical tip Mach is the upstream total-speed approximation, not a full crossflow blade calculation.",
        ],
        "pitch_boundary_scan_max_mps": float(max(speeds)),
        "pitch_balance_boundaries_mps": audit.pitch_boundaries(max(speeds)),
        "cases": [audit.evaluate(v, soc) for soc in socs for v in speeds],
        "sensitivity": sensitivity_cases(audit.p),
    }


def sensitivity_cases(nominal):
    """One-at-a-time illustrative assumptions, NOT measured uncertainty bounds."""
    variants = [("cp_toward_cg_10mm", .01, 1), ("cp_away_from_cg_10mm", -.01, 1),
                ("normal_force_minus20pct", 0, .8), ("normal_force_plus20pct", 0, 1.2)]
    cases = []
    for name, cp_delta, normal_scale in variants:
        p = copy.deepcopy(nominal)
        p["cp_from_cg_m"] += cp_delta
        p["aero"]["CN_alpha"] *= normal_scale
        p["aero"]["CN_cross"] *= normal_scale
        audit = LevelTrimAudit(p)
        cases.append({"variant": name, "illustrative_not_measured": True,
                      "cp_delta_m": cp_delta, "normal_coefficient_scale": normal_scale,
                      "cases": [audit.evaluate(v, 1) for v in [15, 40, 83.3]]})
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Optional NEW JSON output path; never overwrite an existing report.")
    parser.add_argument("--speeds", nargs="+", type=float)
    parser.add_argument("--soc", nargs="+", type=float)
    args = parser.parse_args()
    report = make_report(args.speeds, args.soc)
    rendered = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if args.output:
        with args.output.open("x") as stream:
            stream.write(rendered)
        print(f"Saved {len(report['cases'])} operating points: {args.output}")
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
