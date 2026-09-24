"""A cause for the trim gap must stay a diagnosis, not become a design claim."""
import numpy as np
import pytest

from research.trim_envelope import LevelTrimAudit
from research.trim_levers import (FULL_BAND, arm_edit, combined_authority,
                                  constant_altitude_window, crosswind_limit,
                                  current_edit, lever_threshold,
                                  max_constant_altitude_speed, moment_requirement,
                                  normal_balance_roots, offset_edit)


@pytest.fixture(scope="module")
def audit():
    return LevelTrimAudit()


@pytest.mark.parametrize("speed", [5, 10, 20, 40, 60, 83.3])
def test_level_trim_attitude_is_unique_so_no_high_alpha_branch_was_missed(audit, speed):
    """The single `brentq` root is not a solver artefact hiding a second branch."""
    roots = normal_balance_roots(audit, speed)
    assert len(roots) == 1
    assert np.isclose(np.degrees(roots[0]), audit.force_balance(speed)["theta_deg"], atol=1e-6)


def test_required_moment_is_set_by_weight_not_by_airspeed(audit):
    """|cp|*W*cos(theta) reproduces the plant moment to machine precision, and the
    value barely moves across the band because cos(theta) -> 1."""
    for speed in [10, 20, 40, 60, 83.3]:
        result = moment_requirement(audit, speed)
        assert result["closed_form_error_Nm"] < 1e-12
    high_speed = [moment_requirement(audit, v)["required_moment_Nm"] for v in [40, 60, 83.3]]
    assert max(high_speed) - min(high_speed) < 0.03


def test_thrust_floor_is_about_three_quarters_of_weight(audit):
    """Trim needs T >= |cp|*W*cos(theta)/a, which no mid-speed level point supplies."""
    floor = moment_requirement(audit, 83.3)["thrust_floor_as_weight_fraction"]
    assert 0.70 < floor < 0.78
    worst = moment_requirement(audit, 40)
    assert worst["total_thrust_N"] < worst["thrust_floor_N"]
    assert not worst["pitch_feasible"]


def test_no_roll_orientation_can_rescue_the_middle_of_the_band(audit):
    """Even the orientation-free upper bound arm*T, which no realisable layout
    attains under roll and yaw balance, still falls short around 40 m/s."""
    assert moment_requirement(audit, 40)["orientation_free_ratio"] > 1
    assert moment_requirement(audit, 50)["orientation_free_ratio"] > 1


def test_mid_band_needs_acceleration_and_is_not_a_steady_solution(audit):
    """A window at 40 m/s exists only while accelerating; zero acceleration is out."""
    window = constant_altitude_window(audit, 40, samples=300)
    assert window["feasible"]
    assert window["min_acceleration_mps2"] > 1.0


def test_target_cruise_speed_has_no_constant_altitude_solution_at_any_attitude(audit):
    """83.3 m/s fails steady AND accelerating, so it is not merely hard to reach."""
    assert not constant_altitude_window(audit, 83.3, samples=300)["feasible"]
    assert not audit.evaluate(83.3, 1)["model_feasible"]


def test_reachable_speed_falls_far_short_of_the_three_hundred_kmh_target(audit):
    result = max_constant_altitude_speed(audit, samples=300)
    assert 55 < result["max_speed_mps"] < 70
    assert result["max_speed_mps"] < 83.3


def test_current_alone_never_reopens_the_middle_band(audit):
    """The mid-band rejection is negative thrust, which no electrical headroom fixes.
    This asymmetry is why 'use a bigger motor' is not an answer."""
    assert lever_threshold(current_edit, 40.0, 400.0, FULL_BAND, 0.5) is None
    assert lever_threshold(current_edit, 40.0, 200.0, [83.3], 0.05) is not None


def test_geometry_levers_reopen_the_band_only_after_large_moves(audit):
    offset = lever_threshold(offset_edit, 0.08871, 0.0, FULL_BAND, 1e-3)
    arm = lever_threshold(arm_edit, 0.16884, 1.2, FULL_BAND, 1e-2)
    assert offset < 0.5*abs(audit.p["cp_from_cg_m"])
    assert arm > 2*audit.p["arm_m"]


def test_trim_consumes_most_authority_at_the_sweep_upper_speed(audit):
    """V_H = 18 m/s is inside the trimmable band but spends ~85% of pitch authority
    holding attitude, so very little is left for disturbances."""
    still_air = combined_authority(audit, 18, 0)
    assert still_air["holds_heading"]
    assert 0.8 < still_air["authority_used"] < 0.9


def test_crosswind_demand_stacks_on_trim_and_breaks_v_high_early(audit):
    """Pitch and yaw demands add, so the sweep's crosswind cases at V_H exceed
    authority while the same crosswind at V_L is comfortable."""
    assert combined_authority(audit, 18, 2)["authority_used"] > 1
    assert combined_authority(audit, 8, 5)["authority_used"] < 0.3
    assert crosswind_limit(audit, 18) < 2.5
    assert crosswind_limit(audit, 8) > 15


def test_crosswind_limit_collapses_as_speed_approaches_the_trim_boundary(audit):
    limits = [crosswind_limit(audit, v) for v in [8, 12, 15, 18]]
    assert limits == sorted(limits, reverse=True)


def test_diagnostics_never_mutate_the_stored_profile(audit):
    """Levers must work on copies; a silently edited profile would corrupt every
    later result in the session."""
    before = (audit.p["cp_from_cg_m"], audit.p["arm_m"], audit.p["motor"]["current_limit_A"])
    lever_threshold(offset_edit, 0.08871, 0.0, [83.3], 1e-2)
    fresh = LevelTrimAudit()
    assert (fresh.p["cp_from_cg_m"], fresh.p["arm_m"],
            fresh.p["motor"]["current_limit_A"]) == before
