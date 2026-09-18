"""Reproduce a selected CSV against a pinned sizing checkout, then export SI data.

This is an offline, explicit import. The user's CSV and upstream checkout are never
modified. Flight runs use the checked-in JSON, not a developer's Downloads folder.
"""
import argparse
import csv
import dataclasses
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

UPSTREAM_SHA = "db793a7039c76c9937597a240efc94f68c5a8af0"
CSV_SHA = "326342d0e471bbb424eb558e4034f4e2458a7a2b669b19cfc2af59b96e8528ff"


def export(csv_path, upstream):
    sha = subprocess.check_output(["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True).strip()
    if sha != UPSTREAM_SHA:
        raise ValueError(f"Upstream commit mismatch: {sha}")
    data = csv_path.read_bytes()
    if hashlib.sha256(data).hexdigest() != CSV_SHA:
        raise ValueError("This importer is pinned to the reviewed selected_design.csv")
    rows = list(csv.DictReader(data.decode("utf-8-sig").splitlines()))
    if len(rows) != 1:
        raise ValueError("Expected one selected design")
    row = rows[0]
    sys.path.insert(0, str(upstream))
    import main
    import constants as k
    from interfaces import DesignVars
    from modules import geom, prop, thrm

    values = {f.name: float(row["dv_" + f.name]) for f in dataclasses.fields(DesignVars)}
    values["n_ser"] = int(values["n_ser"])
    dv = DesignVars(**values)
    result = main.evaluate(dv, split=False)
    flat = {}
    for prefix, group in [("dv", values), ("diag", result.diag), ("ec", result.ec), ("g", result.g)]:
        flat.update({prefix + "_" + key: val for key, val in group.items()})
    checked = 0
    for key, val in row.items():
        if not key.startswith(("dv_", "diag_", "ec_", "g_")) or val == "":
            continue
        actual = flat[key]
        equal = (math.isclose(float(val), actual, rel_tol=1e-8, abs_tol=1e-9)
                 if isinstance(actual, (float, int)) else actual == val)
        if not equal:
            raise ValueError(f"Sizing reproduction failed: {key}: {val} != {actual}")
        checked += 1
    p = result.wght.payload
    air, aero = result.pre.atm, result.pre.aero
    d_motor, l_motor, _ = thrm.motor_geometry(p.m_mot)
    d_pod, l_pod, _, _ = geom.pod(p.m_mot, d_motor, l_motor, result.pre.hull, dv)
    pod = (d_pod, l_pod)
    speeds = list(range(0, 151, 5))
    cd = [aero.F_drag(max(v, 1), 0, pod) / (.5 * air.rho * max(v, 1)**2 * aero.S_ref) for v in speeds]
    # At alpha=pi/2, the potential term vanishes; CL is -CA, not CN.
    cn_cross = aero.F_drag(1, math.pi / 2, pod) / (.5 * air.rho * aero.S_ref)
    js = [k.J_map_max * i / (k.n_J_grid - 1) for i in range(k.n_J_grid)]
    resistance, i0 = prop.motor_regression(p.m_mot, p.smot.kv)
    capacity = p.E_batt / (3.7 * dv.n_ser)
    anchors = {}
    for name, speed, hover in [("hover", 0, True), ("cruise", k.V_cr, False)]:
        point = prop.solve_point(speed, result.wght.MTOW, p.m_mot, result.pre.pmap,
                                 aero, air, p.U_eval, hover=hover, kv=p.smot.kv, pod=pod)
        anchors[name] = {**dataclasses.asdict(point), "speed_mps": speed, "bus_voltage_V": p.U_eval}
    return {
        "id": "selected-6931", "label": "선정 기체 6931 (개념설계 모델)",
        "provenance": {"repository": "https://github.com/gocksk/Rocket-Drone-Project",
                       "commit": sha, "csv": csv_path.name, "sha256": CSV_SHA,
                       "source_id": row["source_id"], "reproduced_fields": checked,
                       "classification": "conceptual-sizing-not-measured"},
        "mass_kg": result.wght.MTOW,
        "inertia_kg_m2": [result.mass.J_xx, result.mass.J_yy, result.mass.J_zz],
        "rho": air.rho, "g": k.g, "sound_speed_mps": air.a_snd,
        "body_diameter_m": dv.d_body, "body_length_m": dv.d_body * dv.lambda_body,
        "area_m2": aero.S_ref, "arm_m": result.diag["arm_rotor"],
        "cp_from_cg_m": result.diag["x_cg"] - result.diag["x_cp"],
        "cg_from_nose_m": result.diag["x_cg"],
        "prop": {"diameter_m": dv.d_prop, "J": js,
                 "CT": [result.pre.pmap.CT(j) for j in js],
                 "CP": [result.pre.pmap.CP(j) for j in js], "tip_mach_limit": k.M_tip_max},
        "aero": {"speed_mps": speeds, "CD0": cd, "CN_alpha": aero.CN_alpha,
                 "CN_cross": cn_cross, "damping": [-5, -10, -10]},
        "motor": {"kv_rpm_V": p.smot.kv, "resistance_ohm": resistance, "i0_A": i0,
                  "rotor_inertia_kg_m2": 1e-5, "speed_loop_tau_s": .02, "current_limit_A": 40},
        "battery": {"series": dv.n_ser, "nominal_energy_Wh": p.E_batt,
                    "capacity_Ah": capacity, "resistance_ohm": .010 * dv.n_ser / capacity,
                    "max_C": 40, "esc_efficiency": .95, "minimum_soc": .2},
        "anchors": anchors,
        "assumptions": [
            "Body +x thrust; scalar-last body-to-world quaternion; world +z up.",
            "CSV/source sizing outputs are predictions, not flight or bench measurements.",
            "Rotor inertia 1e-5 kg m2, ESC speed-loop 20 ms and 40 A per motor are explicit test assumptions.",
            "Motor electrical inductance and thermal transients are not modeled; no regenerative ESC braking.",
            "Battery Wh is nominal 3.7 V/cell times Ah. R uses the same Ah, correcting upstream 4.2/3.7 inconsistency.",
            "CN is extended symmetrically to lateral directions; reverse-flow continuation is dissipative but unvalidated.",
            "Upstream uses total V for propeller J; the 6DOF plant uses axial inflow. Static sizing is not a 6DOF trim solution.",
            "Aerodynamic damping [-5,-10,-10] and off-map propeller behavior require sensitivity tests.",
            "Sensor rates/noise, bias random walk, GPS latency and RPM telemetry are assumed, not measured."
        ]
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, required=True)
    ap.add_argument("--upstream", type=Path, required=True)
    ap.add_argument("--output", type=Path, default=Path(__file__).parent / "profiles/selected.json")
    args = ap.parse_args()
    profile = export(args.csv, args.upstream)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(profile, indent=2, ensure_ascii=False) + "\n")
    print(f"Exported {profile['id']}: {profile['provenance']['reproduced_fields']} fields reproduced")
