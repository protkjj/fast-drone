"""Read-only trace of selected sizing inputs, CAD geometry, and simulator assumptions.

No CSV, STL, upstream source, selected profile, or controller is modified. The
optional report is created exclusively. Counterfactuals are diagnostic copies,
not replacement aircraft profiles or validated aerodynamic reconstructions.
"""
import argparse
import copy
import csv
import dataclasses
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from research.export_selected import export, UPSTREAM_SHA
from research.model import profile
from research.trim_envelope import LevelTrimAudit

ROOT = Path(__file__).parent
ASSET = ROOT/"assets/drone_v2.stl"
STL_SHA = "276045699ee8e9ee03fe5d75d4915d88a0688fbcb12a9e8d1e8f5c0702302e35"


def cad_geometry(path, body_radius):
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != STL_SHA:
        raise ValueError("STL SHA mismatch: fixed face ranges apply only to the reviewed CAD.")
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    vertices = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].astype(float)*.001
    fins = []
    # The component audit identifies four separate 12-triangle trapezoidal fins.
    for first in [8300, 8312, 8324, 8336]:
        v = vertices[first:first+12].reshape(-1, 3)
        sy, sz = np.sign(v.mean(axis=0)[1:])
        radial = (sy*v[:, 1]+sz*v[:, 2])/np.sqrt(2)
        normal = (sy*v[:, 1]-sz*v[:, 2])/np.sqrt(2)
        inner, outer = min(radial), max(radial)
        root = v[np.isclose(radial, inner, atol=1e-8, rtol=0), 0]
        tip = v[np.isclose(radial, outer, atol=1e-8, rtol=0), 0]
        # Thin-fin center-plane convention: do not count material inside the
        # body cylinder as exposed planform. This is not a CFD surface integral.
        if not inner < body_radius < outer:
            raise ValueError("Expected a fin root buried inside the body cylinder.")
        root_le = np.interp(body_radius, [inner, outer], [min(root), min(tip)])
        root_te = np.interp(body_radius, [inner, outer], [max(root), max(tip)])
        cr, ct, span = root_te-root_le, max(tip)-min(tip), outer-body_radius
        fins.append({"root_le_m": float(root_le), "root_te_m": float(root_te),
                     "tip_le_m": float(min(tip)), "tip_te_m": float(max(tip)),
                     "root_chord_m": float(cr), "tip_chord_m": float(ct),
                     "span_m": float(span), "leading_edge_sweep_m": float(min(tip)-root_le),
                     "thickness_m": float(np.ptp(normal)),
                     "exposed_area_m2": float(span*(cr+ct)/2),
                     "whole_area_m2": float((outer-inner)*(np.ptp(root)+ct)/2),
                     "buried_radial_depth_m": float(body_radius-inner)})
    centers = []
    # Opposing blade-pair bounding boxes give an independent center estimate.
    # Do not read the visualization's hardcoded center as geometric evidence.
    for first, end in [(18308,54812),(54812,96056),(96056,132560),(132560,173804)]:
        v = vertices[first:end].reshape(-1, 3)
        centers.append((v.min(axis=0)+v.max(axis=0))/2)
    body = vertices[:8300].reshape(-1, 3)
    return {"sha256": STL_SHA, "units_interpreted_as": "mm converted to m",
            "body_length_m": float(np.ptp(body[:, 0])),
            "body_diameter_y_m": float(np.ptp(body[:, 1])), "fins": fins,
            "blade_pair_centers_from_nose_m": [c.tolist() for c in centers],
            "rotor_radius_from_body_axis_m": [float(np.hypot(c[1], c[2])) for c in centers]}


def barrowman(fin, diameter, nose_length, nose_cp_fraction, cn_nose):
    """Independent reproduction of upstream aero.py's small-angle CP formula.

    Using it on STL geometry is still a conceptual estimate, not a measurement.
    A common compressibility multiplier cancels out of the weighted CP.
    """
    cr, ct, span = fin["root_chord_m"], fin["tip_chord_m"], fin["span_m"]
    sweep = fin["leading_edge_sweep_m"]
    mid_length = np.hypot(span, sweep+(ct-cr)/2)
    interference = 1+diameter/2/(span+diameter/2)
    cn_fin = interference*4*4*(span/diameter)**2/(1+np.sqrt(1+(2*mid_length/(cr+ct))**2))
    x_fin = (fin["root_le_m"]+sweep*(cr+2*ct)/(3*(cr+ct))
             +(cr+ct-cr*ct/(cr+ct))/6)
    x_nose = nose_cp_fraction*nose_length
    return {"fin_cp_from_nose_m": float(x_fin), "nose_cp_from_nose_m": float(x_nose),
            "CN_fin_incompressible": float(cn_fin),
            "CN_total_incompressible": float(cn_nose+cn_fin),
            "cp_from_nose_m": float((cn_nose*x_nose+cn_fin*x_fin)/(cn_nose+cn_fin))}


def lumped_mass_properties(parts, body_length):
    """Independent trace of upstream's lumped approximation, not solid-CAD inertia."""
    mass = sum(p["m"] for p in parts)
    cg = sum(p["m"]*p["x"] for p in parts)/mass
    ixx = sum(p["m"]*p["r"]**2 for p in parts)
    shell = next(p for p in parts if p["name"] == "shell")
    iyy = sum(p["m"]*((p["x"]-cg)**2+.5*p["r"]**2) for p in parts)+shell["m"]*body_length**2/12
    return {"mass_kg": mass, "cg_from_nose_m": cg, "inertia_kg_m2": [ixx, iyy, iyy]}


def fixed_total_pitch_authority(total, per_rotor_cap, radial_arm):
    if (not np.all(np.isfinite([total, per_rotor_cap, radial_arm])) or total < 0
            or per_rotor_cap < 0 or radial_arm <= 0 or total > 4*per_rotor_cap):
        raise ValueError("Expected nonnegative thrust, positive arm, and total <= four rotor caps.")
    increase = per_rotor_cap-total/4
    decrease = total/4  # Current propulsion cannot request negative thrust.
    factor = 4*radial_arm/np.sqrt(2)
    return {"increase_room_per_rotor_N": increase, "decrease_room_per_rotor_N": decrease,
            "increase_only_pitch_estimate_Nm": factor*increase,
            "fixed_total_pitch_limit_Nm": factor*min(increase, decrease)}


def audit_sources(csv_path, upstream):
    # HEAD alone would not detect a local source edit. Require clean tracked
    # source too, then reuse the existing 70-field reproduction guard.
    if subprocess.check_output(["git", "-C", str(upstream), "status", "--porcelain", "--untracked-files=no"], text=True).strip():
        raise ValueError("Upstream tracked source is modified; refusing to label it pinned.")
    reexported = export(csv_path, upstream)
    # export() verified the commit/CSV and loaded modules from the pinned checkout.
    import main
    import constants as k
    from interfaces import DesignVars
    from modules import geom, thrm

    with csv_path.open(encoding="utf-8-sig") as stream:
        row = next(csv.DictReader(stream))
    inputs = {field.name: float(row["dv_"+field.name]) for field in dataclasses.fields(DesignVars)}
    inputs["n_ser"] = int(inputs["n_ser"])
    dv = DesignVars(**inputs)
    result = main.evaluate(dv, split=False)
    p, h = profile(), result.pre.hull
    source_fin = {"root_le_m": h.x_fin, "root_te_m": h.x_fin+h.c_root,
                  "tip_le_m": h.x_fin+h.x_t, "tip_te_m": h.x_fin+h.x_t+h.c_tip,
                  "root_chord_m": h.c_root, "tip_chord_m": h.c_tip, "span_m": h.b_fin,
                  "leading_edge_sweep_m": h.x_t, "thickness_m": h.t_fin,
                  "exposed_area_m2": inputs["S_fin"]/4}
    cad = cad_geometry(ASSET, h.r_body)
    cp_args = (inputs["d_body"], h.l_nose, k.k_cp_nose, k.CN_nose)
    source_cp = barrowman(source_fin, *cp_args)
    cad_cp = barrowman(cad["fins"][0], *cp_args)
    np.testing.assert_allclose(source_cp["cp_from_nose_m"], result.pre.aero.x_cp, atol=1e-12)
    parts = [dataclasses.asdict(item) for item in result.mass.breakdown]
    mass = lumped_mass_properties(parts, h.l_body)
    np.testing.assert_allclose(mass["cg_from_nose_m"], result.mass.x_cg, atol=1e-12)
    np.testing.assert_allclose(mass["inertia_kg_m2"], p["inertia_kg_m2"], atol=1e-12)

    payload = result.wght.payload
    pod = geom.pod(payload.m_mot, *thrm.motor_geometry(payload.m_mot)[:2], h, dv)
    shifted = copy.deepcopy(parts)
    next(item for item in shifted if item["name"] == "props")["x"] = pod[3]
    prop_position_only = lumped_mass_properties(shifted, h.l_body)
    cg_delta = prop_position_only["cg_from_nose_m"]-mass["cg_from_nose_m"]
    nominal_trim = LevelTrimAudit(p)
    cp_copy = copy.deepcopy(p)
    cp_copy["cp_from_cg_m"] = p["cg_from_nose_m"]-cad_cp["cp_from_nose_m"]
    cp_only_trim = LevelTrimAudit(cp_copy)
    transition_speed = result.diag["V_trans"]
    authority = fixed_total_pitch_authority(result.diag["T_req_trans"],
                                            result.diag["T_avail_trans"]/4, result.layout.arm_rotor)
    np.testing.assert_allclose(authority["increase_only_pitch_estimate_Nm"], result.stab.M_ctrl, atol=1e-12)
    b = p["battery"]
    source_files = ["main.py", "constants.py", "modules/geom.py", "modules/aero.py", "modules/wght.py",
                    "modules/prop.py", "modules/stab.py", "modules/thrm.py", "modules/miss.py", "data/motor_specs.csv"]
    return {
        "schema": "selected-design-source-audit-v1", "upstream_commit": UPSTREAM_SHA,
        "source_csv_sha256": reexported["provenance"]["sha256"],
        "reproduced_csv_fields": reexported["provenance"]["reproduced_fields"],
        "reexport_matches_selected_profile": reexported == p,
        "upstream_file_sha256": {name: hashlib.sha256((upstream/name).read_bytes()).hexdigest() for name in source_files},
        "local_file_sha256": {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in
                              ["design_source_audit.py", "model.py", "profiles/selected.json", "trim_envelope.py"]},
        "source_design_inputs": inputs, "source_mass_breakdown": parts,
        "source_mass_properties": mass, "source_fin_geometry": source_fin,
        "source_cp_independent_reproduction": source_cp,
        "source_motor_center_from_nose_m": pod[2], "source_prop_location_from_nose_m": pod[3],
        "cad_geometry": cad, "cad_small_angle_cp_estimate": cad_cp,
        "prop_mass_position_counterfactual": {
            "description": "Move only the existing prop mass from motor center to source geom.pod x_prop; do not change profile.",
            "result": prop_position_only, "cg_shift_aft_m": cg_delta},
        "source_stability_gate": {"speed_mps": transition_speed, **dataclasses.asdict(result.stab), **authority,
                                  "research_level_trim": nominal_trim.evaluate(transition_speed)},
        "electrical_provenance": {
            "selected_motor_product": None, "confirmed_motor_current_rating_A": None,
            "confirmed_esc_current_rating_A": None, "sized_motor_mass_kg_each": payload.m_mot,
            "sized_kv_rpm_V": payload.smot.kv, "source_battery_continuous_C_assumption": k.c_rate_max,
            "csv_predicted_peak_PACK_current_A": float(row["diag_I_max"]),
            "simulator_motor_current_assumption_A": p["motor"]["current_limit_A"],
            "simulator_uniform_pack_budget_A_per_motor": b["capacity_Ah"]*b["max_C"]*b["esc_efficiency"]/4,
            "simulator_effective_motor_limit_A": min(p["motor"]["current_limit_A"], b["capacity_Ah"]*b["max_C"]*b["esc_efficiency"]/4),
            "motor_fit_mass_range_kg": [k.m_mot_fit_lo, k.m_mot_fit_hi],
            "motor_fit_kv_range_rpm_V": [k.kv_fit_lo, k.kv_fit_hi]},
        "cad_cp_only_counterfactual": {
            "changed_fields": {"cp_from_cg_m": cp_copy["cp_from_cg_m"]},
            "not_a_complete_cad_aircraft_model": True,
            "description": "Keep selected mass, CG, inertia, aero forces and propulsion; replace CP only to isolate its influence.",
            "pitch_boundaries_mps": cp_only_trim.pitch_boundaries(100),
            "cases": [cp_only_trim.evaluate(v) for v in [15, 40, 83.3]]},
        "limitations": [
            "CSV contains conceptual sizing outputs, not measured properties or an identified motor/ESC product.",
            "STL is geometry only: it cannot supply material density, component mass, true CG, or measured CP.",
            "CAD fin exposure uses a thin center-plane cylinder intersection and the upstream small-angle Barrowman approximation.",
            "The prop-position and CAD-CP counterfactuals are isolated diagnostics, not approved model corrections.",
            "Fixed-total moment bound omits electrical constraints and is an upper bound, not additional control authority.",
            "The simulator's uniform motor-current budget is sufficient/conservative for pack current, not an equivalent DC bus limit.",
        ],
    }


def main_cli():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit_sources(args.csv, args.upstream)
    text = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+"\n"
    if args.output:
        with args.output.open("x") as stream:
            stream.write(text)
        print(f"Saved source/CAD audit: {args.output}")
    else:
        print(text, end="")


if __name__ == "__main__":
    main_cli()
