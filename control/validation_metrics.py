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
