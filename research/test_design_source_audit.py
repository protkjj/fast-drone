"""Geometry/provenance checks must not silently replace the selected model."""
import numpy as np
import pytest

from research.design_source_audit import (
    ASSET, barrowman, cad_geometry, fixed_total_pitch_authority, lumped_mass_properties,
)
from research.model import profile


@pytest.fixture(scope="module")
def cad():
    return cad_geometry(ASSET, profile()["body_diameter_m"]/2)


def test_mesh_matches_body_and_rotor_radius_but_not_selected_fin_placement(cad):
    p = profile()
    assert np.isclose(cad["body_length_m"], p["body_length_m"], atol=1e-7)
    np.testing.assert_allclose(cad["rotor_radius_from_body_axis_m"], p["arm_m"], atol=5e-6)
    assert len(cad["fins"]) == 4
    for fin in cad["fins"]:
        assert .488 < fin["root_le_m"] < .489  # Source selected x_fin is 0.549235 m.
        assert .078 < fin["root_chord_m"] < .079
        assert .125 < fin["span_m"] < .127
        assert .0039 < fin["thickness_m"] < .0041
        assert fin["tip_te_m"] > fin["root_te_m"]+.028


def test_planform_discards_only_the_part_buried_inside_the_body(cad):
    for fin in cad["fins"]:
        assert fin["buried_radial_depth_m"] > .0025
        assert fin["exposed_area_m2"] < fin["whole_area_m2"]
        assert np.isclose(fin["exposed_area_m2"],
                          fin["span_m"]*(fin["root_chord_m"]+fin["tip_chord_m"])/2)
    assert np.isclose(sum(f["exposed_area_m2"] for f in cad["fins"]), .0288875911193)


def test_same_barrowman_formula_gives_a_different_cp_for_cad(cad):
    p = profile()
    result = barrowman(cad["fins"][0], p["body_diameter_m"], 3*p["body_diameter_m"], .466, 2)
    assert np.isclose(result["cp_from_nose_m"], .4749123837618052)
    selected_cp = p["cg_from_nose_m"]-p["cp_from_cg_m"]
    assert .04 < selected_cp-result["cp_from_nose_m"] < .043


def test_bad_mesh_hash_is_rejected_before_using_fixed_face_ranges(tmp_path):
    wrong = tmp_path/"unreviewed.stl"
    wrong.write_bytes(b"not the reviewed CAD")
    with pytest.raises(ValueError, match="SHA"):
        cad_geometry(wrong, .05)


def test_fixed_total_allocation_checks_downward_as_well_as_upward_headroom():
    result = fixed_total_pitch_authority(6, 12.5, .17)
    assert result["increase_room_per_rotor_N"] == 11
    assert result["decrease_room_per_rotor_N"] == 1.5
    assert np.isclose(result["fixed_total_pitch_limit_Nm"], .17/np.sqrt(2)*6)
    assert result["increase_only_pitch_estimate_Nm"] > 7*result["fixed_total_pitch_limit_Nm"]


def test_pitch_authority_vanishes_at_maximum_collective_thrust():
    result = fixed_total_pitch_authority(40, 10, .17)
    assert result["fixed_total_pitch_limit_Nm"] == 0


@pytest.mark.parametrize("total,cap,arm", [(-1,10,.1),(41,10,.1),(1,-1,.1),(1,10,0)])
def test_invalid_allocation_inputs_are_rejected(total, cap, arm):
    with pytest.raises(ValueError):
        fixed_total_pitch_authority(total, cap, arm)


def test_mass_reconstruction_and_prop_shift_have_traceable_effects():
    parts = [{"name":"shell","m":1.,"x":.3,"r":.05},
             {"name":"props","m":.02,"x":.5,"r":.15}]
    before = lumped_mass_properties(parts, .6)
    after = lumped_mass_properties([parts[0],dict(parts[1],x=.56)], .6)
    assert np.isclose(after["cg_from_nose_m"]-before["cg_from_nose_m"], .02*.06/1.02)
    assert after["inertia_kg_m2"][0] == before["inertia_kg_m2"][0]
    assert after["inertia_kg_m2"][1] > before["inertia_kg_m2"][1]
    assert parts[1]["x"] == .5
