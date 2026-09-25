"""Tracking, safety and gust-recovery criteria shared by validation stages."""
from dataclasses import dataclass, asdict

import numpy as np


@dataclass(frozen=True)
class Acceptance:
    # Research acceptance settings, not a certification claim.
    max_z_error: float = 10.0
    max_velocity_error: float = 15.0
    max_omega: float = 35.0
    sustained_omega: float = 25.0
    sustained_seconds: float = 0.2
    recovery_z: float = 0.5
    recovery_velocity: float = 0.5
    recovery_omega: float = 1.0
    recovery_hold: float = 0.5

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('acceptance thresholds must be finite and positive')


def longest_duration(ts, mask):
    """Longest contiguous true interval, measured using actual timestamps."""
    longest = 0.0
    start = None
    for i, flag in enumerate(mask):
        if flag and start is None:
            start = float(ts[i])
        if start is not None:
            end = float(ts[i])
            longest = max(longest, end-start)
            if not flag:
                start = None
    return longest


def evaluate(result, profile, limits=Acceptance(), n_max=None):
    ts, xs = result['ts'], result['xs']
    vr, zr = result['v_refs'], result['z_refs']
    finite = bool(np.all(np.isfinite(xs)) and np.all(np.isfinite(result['us'])))
    ez = np.abs(xs[:, 2]-zr)
    ev = np.linalg.norm(xs[:, 3:6]-vr, axis=1)
    omega = np.linalg.norm(xs[:, 10:13], axis=1)
    reasons = []
    if not finite:
        reasons.append('nonfinite')
    if ts[-1] < profile.T_total-1e-8:
        reasons.append('incomplete')
    if result.get('error'):
        reasons.append('simulation_error')
    if np.max(ez) > limits.max_z_error:
        reasons.append('altitude_error')
    if np.max(ev) > limits.max_velocity_error:
        reasons.append('velocity_error')
    if np.max(omega) > limits.max_omega:
        reasons.append('omega_peak')
    sustained = longest_duration(ts, omega > limits.sustained_omega)
    if sustained >= limits.sustained_seconds-1e-10:
        reasons.append('omega_sustained')
    phases = []
    for name, start, end in profile.get_phase_boundaries():
        mask = (ts >= start) & (ts < end)
        if end == profile.T_total:
            mask |= ts == end
        phases.append(dict(name=name, samples=int(mask.sum()),
                           rmse_z=float(np.sqrt(np.mean(ez[mask]**2))) if mask.any() else None,
                           rmse_velocity=float(np.sqrt(np.mean(ev[mask]**2))) if mask.any() else None))
    metrics = dict(
        rmse_z=float(np.sqrt(np.mean(ez**2))),
        rmse_vx=float(np.sqrt(np.mean((xs[:, 3]-vr[:, 0])**2))),
        rmse_velocity=float(np.sqrt(np.mean(ev**2))),
        max_z_error=float(np.max(ez)), max_velocity_error=float(np.max(ev)),
        max_omega=float(np.max(omega)), omega_sustained_seconds=sustained,
        simulated_seconds=float(ts[-1]), phases=phases,
    )
    if n_max is not None and len(result['us']):
        metrics['motor_saturation_fraction'] = float(np.mean(result['us'] >= .95*n_max))
    if profile.gust_interval is None:
        tail = ts >= profile.T_total-limits.recovery_hold-1e-9
        final_settled = bool(tail.any() and ts[-1] >= profile.T_total-1e-8 and finite
                             and np.all(ez[tail] <= limits.recovery_z)
                             and np.all(ev[tail] <= limits.recovery_velocity)
                             and np.all(omega[tail] <= limits.recovery_omega))
        metrics['final_settled'] = final_settled
        if not final_settled:
            reasons.append('final_hover_not_settled')
    if profile.gust_interval is not None:
        start, end = profile.gust_interval
        stable = (ez <= limits.recovery_z) & (ev <= limits.recovery_velocity) & (
            omega <= limits.recovery_omega) & np.all(np.isfinite(xs), axis=1)
        before = (ts >= start-limits.recovery_hold-1e-9) & (ts < start)
        ready = bool(before.any() and ts[before][0] <= start-limits.recovery_hold+1e-8
                     and np.all(stable[before]) and ts[-1] >= start)
        # Recovery starts AFTER the pulse ends, includes lateral velocity, and
        # must persist through the observed tail for at least recovery_hold.
        stable_tail = np.logical_and.accumulate(stable[::-1])[::-1]
        candidates = np.flatnonzero((ts >= end-1e-9) & stable_tail & (
            ts[-1]-ts >= limits.recovery_hold-1e-9))
        recovered = bool(len(candidates) and ts[-1] >= profile.T_total-1e-8 and finite)
        recovery_time = float(max(0, ts[candidates[0]]-end)) if recovered else None
        after = ts >= start
        metrics.update(pre_gust_ready=ready, recovered=recovered,
                       recovery_seconds=recovery_time,
                       gust_max_z_error=float(np.max(ez[after])) if after.any() else None,
                       gust_max_velocity_error=float(np.max(ev[after])) if after.any() else None)
        if not ready:
            reasons.append('pre_gust_not_settled')
        if not recovered:
            reasons.append('gust_not_recovered')
    metrics.update(passed=not reasons, failure_reasons=reasons)
    return metrics


@dataclass(frozen=True)
class PaperCriteria:
    """Paper v5.3 section 5.10 criteria, reported ALONGSIDE Acceptance.

    They are stricter than Acceptance (e.g. body rate 20 rad/s for 0.1 s)
    and never change `passed`; both views are kept so neither is hidden.
    """
    z_error: float = 5.0
    z_error_seconds: float = 0.5
    omega: float = 20.0
    omega_seconds: float = 0.1
    cruise_velocity_floor: float = 5.0
    cruise_velocity_fraction: float = 0.25
    cruise_velocity_seconds: float = 2.0
    consecutive_solver_failures: int = 3
    recovery_z: float = 0.5
    recovery_velocity_floor: float = 0.5
    recovery_velocity_fraction: float = 0.05
    recovery_hold: float = 1.0


def _longest_consecutive(flags):
    longest = run = 0
    for flag in flags:
        run = run+1 if flag else 0
        longest = max(longest, run)
    return longest


def paper_evaluate(result, profile, criteria=PaperCriteria(), window=None,
                   solve_log=None, n_max=None):
    """Section 5.10 failure flags, the 5.8 evaluation-window RMSE and command TV.

    `window` = (start, end) seconds for eq.(35); None uses the whole record.
    Cruise tracking applies where the reference speed is constant and nonzero
    (mission cruise, the whole gust test, reference-profile holds at speed).
    The solver rule interprets "no valid upper update" as non-accepted status.
    """
    ts, xs = result['ts'], result['xs']
    vr, zr = result['v_refs'], result['z_refs']
    finite_rows = np.all(np.isfinite(xs), axis=1)
    ez = np.abs(xs[:, 2]-zr)
    ev = np.linalg.norm(xs[:, 3:6]-vr, axis=1)
    omega = np.linalg.norm(xs[:, 10:13], axis=1)
    reasons = []
    if not np.all(finite_rows):
        reasons.append('paper_nonfinite')
    if np.any(xs[finite_rows, 2] < 0.0):
        reasons.append('paper_ground_contact')
    z_run = longest_duration(ts, ez > criteria.z_error)
    w_run = longest_duration(ts, omega > criteria.omega)
    if z_run >= criteria.z_error_seconds-1e-10 or w_run >= criteria.omega_seconds-1e-10:
        reasons.append('paper_state_limit')
    cruise = np.zeros(len(ts), dtype=bool)
    for name, start, duration, v0, v1, *_ in profile.phases:
        if v0 == v1 and v0 > 0:
            cruise |= (ts >= start) & (ts <= start+duration)
    limit = np.maximum(criteria.cruise_velocity_floor,
                       criteria.cruise_velocity_fraction*np.linalg.norm(vr, axis=1))
    track_run = longest_duration(ts, cruise & (ev > limit))
    if track_run >= criteria.cruise_velocity_seconds-1e-10:
        reasons.append('paper_tracking_failure')
    log = solve_log or []
    solver_run = _longest_consecutive(not entry.get('accepted', True) for entry in log)
    if solver_run >= criteria.consecutive_solver_failures:
        reasons.append('paper_upper_update_failure')
    lo, hi = window if window is not None else (ts[0], ts[-1])
    in_window = (ts >= lo-1e-9) & (ts <= hi+1e-9)
    metrics = dict(
        paper_failed=bool(reasons), paper_reasons=reasons,
        paper_z_error_run_seconds=z_run, paper_omega_run_seconds=w_run,
        paper_cruise_velocity_run_seconds=track_run,
        paper_max_consecutive_solver_failures=solver_run,
        window=[float(lo), float(hi)],
        window_complete=bool(ts[-1] >= hi-1e-8),
        window_rmse_velocity=float(np.sqrt(np.mean(ev[in_window]**2))) if in_window.any() else None,
        window_rmse_z=float(np.sqrt(np.mean(ez[in_window]**2))) if in_window.any() else None,
        window_max_velocity_error=float(np.max(ev[in_window])) if in_window.any() else None,
        window_max_z_error=float(np.max(ez[in_window])) if in_window.any() else None,
        window_p95_velocity_error=float(np.percentile(ev[in_window], 95)) if in_window.any() else None,
        window_p95_z_error=float(np.percentile(ez[in_window], 95)) if in_window.any() else None,
        window_max_omega=float(np.max(omega[in_window])) if in_window.any() else None)
    us = np.asarray(result['us'])
    if n_max is not None and len(us) > 1:
        # Eq.(36) at the common 2 ms command log: sum of |delta n_c| / n_max.
        metrics['command_total_variation'] = float(np.sum(np.abs(np.diff(us, axis=0)))/n_max)
    if profile.gust_interval is not None:
        _, end = profile.gust_interval
        v_limit = np.maximum(criteria.recovery_velocity_floor,
                             criteria.recovery_velocity_fraction*np.linalg.norm(vr, axis=1))
        good = (ez <= criteria.recovery_z) & (ev <= v_limit) & finite_rows
        recovered_at = None
        for i in np.flatnonzero(ts >= end-1e-9):
            held = (ts >= ts[i]-1e-12) & (ts <= ts[i]+criteria.recovery_hold+1e-9)
            if ts[-1] < ts[i]+criteria.recovery_hold-1e-9:
                break
            if np.all(good[held]):
                recovered_at = float(ts[i]-end)
                break
        metrics['paper_recovery_seconds'] = recovered_at
    return metrics
