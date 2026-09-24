"""Offline trim checks must not turn force-only sizing into a flight claim."""
import copy
import json

import numpy as np
import pytest

from research.trim_envelope import LevelTrimAudit, make_report


@pytest.fixture(scope="module")
def audit():
    return LevelTrimAudit()


def test_hover_matches_anchor_but_has_no_defined_angle_of_attack(audit):
    result = audit.evaluate(0)
    assert result["model_feasible"]
    assert result["alpha_deg"] is None
    assert result["theta_deg"] == 90
    np.testing.assert_allclose(result["rotor_rpm"], audit.p["anchors"]["hover"]["rpm"], rtol=1e-5)
    assert result["max_acceleration_residual_mps2"] < 1e-8
    assert result["max_angular_acceleration_residual_radps2"] < 1e-8
    assert result["max_rotor_acceleration_residual_radps2"] < 1e-8
    assert result["soc_rate_per_s"] < 0  # Frozen-SOC operating point, not perpetual flight.


def test_source_cruise_force_solution_is_reproduced_not_mistaken_for_trim(audit):
    result = audit.evaluate(83.3)
    anchor = audit.p["anchors"]["cruise"]
    assert np.isclose(result["theta_deg"], np.degrees(anchor["theta"]), rtol=1e-3)
    assert np.isclose(result["total_thrust_N"], anchor["T"], rtol=1e-3)
    assert result["rotor_thrust_nonnegative"]
    assert not result["model_feasible"]
    assert "current_limit" in result["rejection_reasons"]
    assert max(result["required_motor_current_A"]) > 45
    assert result["effective_motor_current_limit_A"] < 40.01
    assert result["max_acceleration_residual_mps2"] < 1e-8
    assert result["max_angular_acceleration_residual_radps2"] < 1e-8
    assert result["max_rotor_acceleration_residual_radps2"] > 2000


def test_intermediate_speed_has_a_force_moment_conflict_not_a_solver_timeout(audit):
    result = audit.evaluate(40)
    assert result["rejection_reasons"] == ["negative_rotor_thrust_required"]
    assert min(result["required_rotor_thrust_N"]) < -1.5
    assert result["pitch_authority_ratio"] > 2
    assert result["rotor_rpm"] is None  # Never clip a negative requirement and call it solved.
    # Even an arbitrary roll cannot exceed radial arm * total positive thrust.
    assert abs(result["required_rotor_pitch_moment_Nm"]) > audit.p["arm_m"]*result["total_thrust_N"]


def test_geometry_sign_and_full_rhs_are_independently_consistent(audit):
    result = audit.evaluate(15)
    assert result["model_feasible"]
    theta = np.deg2rad(result["theta_deg"])
    axial, _, normal = result["aero_force_body_N"]
    thrust = result["total_thrust_N"]
    weight = audit.p["mass_kg"]*audit.p["g"]
    np.testing.assert_allclose([(thrust+axial)*np.cos(theta)-normal*np.sin(theta),
                               (thrust+axial)*np.sin(theta)+normal*np.cos(theta)], [0, weight], atol=1e-8)
    arm = audit.p["arm_m"]/np.sqrt(2)
    forces = np.array(result["required_rotor_thrust_N"])
    rotor_pitch = np.dot([arm, arm, -arm, -arm], forces)
    assert np.isclose(rotor_pitch, audit.p["cp_from_cg_m"]*normal)
    assert result["max_angular_acceleration_residual_radps2"] < 1e-8


def test_equal_rotors_do_not_cancel_cruise_aerodynamic_pitch_moment(audit):
    result = audit.evaluate(83.3)
    x = audit.state(83.3, np.deg2rad(result["theta_deg"]), 1)
    x[13:17] = audit.p["anchors"]["cruise"]["rpm"]*2*np.pi/60
    rhs = np.asarray(audit.f["rhs"](x, x[13:17], np.zeros(6), np.ones(6))).ravel()
    assert rhs[11] > 40  # rad/s², despite approximately balanced translational forces.


def test_reversing_cp_swaps_required_pairs_without_changing_force_trim(audit):
    p = copy.deepcopy(audit.p)
    p["cp_from_cg_m"] *= -1
    other = LevelTrimAudit(p).evaluate(15)
    original = audit.evaluate(15)
    assert other["model_feasible"]
    assert other["theta_deg"] == original["theta_deg"]
    np.testing.assert_allclose(other["required_rotor_thrust_N"], original["required_rotor_thrust_N"][::-1])


def test_cp_zero_is_an_explicit_counterfactual_not_a_silent_plant_change(audit):
    p = copy.deepcopy(audit.p)
    p["cp_from_cg_m"] = 0
    counterfactual = LevelTrimAudit(p).evaluate(83.3)
    assert counterfactual["model_feasible"]
    assert not audit.evaluate(83.3)["model_feasible"]
    assert audit.p["cp_from_cg_m"] < -.08


def test_soc_changes_bus_not_required_force_balance(audit):
    full, low = audit.evaluate(15, 1), audit.evaluate(15, .2)
    np.testing.assert_allclose(full["required_rotor_thrust_N"], low["required_rotor_thrust_N"])
    assert low["bus_voltage_V"] < full["bus_voltage_V"]
    assert low["model_feasible"]


@pytest.mark.parametrize("speed,holds", [(15, True), (83.3, False)])
def test_actual_1khz_integration_accepts_only_the_feasible_operating_point(audit, speed, holds):
    result = audit.evaluate(speed)
    x = audit.state(speed, np.deg2rad(result["theta_deg"]), 1)
    x[13:17] = np.array(result["rotor_rpm"])*2*np.pi/60
    start = x.copy()
    command = x[13:17].copy()
    for _ in range(100):  # 100 ms; no controller or target-speed correction.
        x = np.asarray(audit.f["step"](x, command, np.zeros(6), np.ones(6))).ravel()
    if holds:
        np.testing.assert_allclose(x[3:17], start[3:17], atol=1e-8)
        assert np.isclose(x[0]-start[0], speed*.1)
    else:
        assert max(abs(x[13:17]-start[13:17])) > 70
        assert abs(x[11]) > .1
    assert x[17] < start[17]


def test_speed_command_limit_and_helical_tip_are_reported_separately(audit):
    assert "rotor_speed_limit" in audit.evaluate(100)["rejection_reasons"]
    result = audit.evaluate(90)
    assert "helical_tip_mach_limit" in result["rejection_reasons"]
    assert max(result["helical_tip_mach"]) > audit.p["prop"]["tip_mach_limit"]


def test_sampled_pitch_boundaries_are_reproducible(audit):
    boundaries = audit.pitch_boundaries(100)
    assert len(boundaries) == 2
    assert 19 < boundaries[0] < 20
    assert 81 < boundaries[1] < 83
    for speed in boundaries:
        assert abs(min(audit.force_balance(speed)["required_rotor_thrust_N"])) < 1e-8


@pytest.mark.parametrize("speed,soc", [(-1,1),(np.nan,1),(151,1),(5,-.1),(5,1.1),(5,np.nan)])
def test_invalid_operating_points_are_rejected(audit, speed, soc):
    with pytest.raises(ValueError):
        audit.evaluate(speed, soc)


def test_report_never_equates_model_balance_with_measured_aero_validity():
    report = make_report(speeds=[0, 40, 83.3], socs=[1])
    assert not report["aerodynamics_validated"]
    assert report["scope"]["roll_deg"] == 0
    assert len(report["cases"]) == 3
    assert len(report["sensitivity"]) == 4
    for variant in report["sensitivity"]:
        assert variant["illustrative_not_measured"]
        assert [r["model_feasible"] for r in variant["cases"]] == [True, False, False]
    json.dumps(report, allow_nan=False)
