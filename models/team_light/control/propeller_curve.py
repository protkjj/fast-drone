"""Small source-anchored propeller surrogate, shared by NumPy and CasADi.

Cp is POWER coefficient, so Q = Cp*rho*n_rps**2*D**5/(2*pi).
The 30k-rpm shape is reused at all RPM: explicit modeling assumption, not a
measured propulsion map. Clipped numerical continuation is not valid data.
"""
from functools import lru_cache

import casadi as ca
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import brentq


MODEL = 'apc_30k_pchip_v2'


def uses_curve(p):
    return p.get('propulsion_model') == MODEL


def _key(p):
    return tuple(tuple(row) for row in p['prop_curve']['knots'])


@lru_cache(maxsize=8)
def _splines(knots):
    rows = np.asarray(knots)
    return tuple(PchipInterpolator(rows[:, 0], rows[:, k], extrapolate=False)
                 for k in (1, 2))


def coefficients(p, advance_ratio):
    ct, cp = _splines(_key(p))
    j = np.clip(np.asarray(advance_ratio), ct.x[0], ct.x[-1])
    return np.maximum(ct(j), 0.), np.maximum(cp(j), 0.)


def _symbolic_spline(spline, j):
    clipped = ca.fmin(ca.fmax(j, float(spline.x[0])), float(spline.x[-1]))
    result = 0.
    for i in range(len(spline.x)-2, -1, -1):
        d = clipped-float(spline.x[i])
        a, b, c, e = map(float, spline.c[:, i])
        segment = ((a*d+b)*d+c)*d+e
        result = segment if i == len(spline.x)-2 else ca.if_else(
            clipped < float(spline.x[i+1]), segment, result)
    return ca.fmax(result, 0.)


def symbolic_force_torque(p, rate, axial_velocity):
    n_rps = ca.fmax(rate, 0.)/(2*ca.pi)
    j = ca.fmax(axial_velocity, 0.)/(n_rps*p['D_prop']+1e-8)
    ct, cp = _splines(_key(p))
    return (p['rho']*n_rps**2*p['D_prop']**4*_symbolic_spline(ct, j),
            p['rho']*n_rps**2*p['D_prop']**5/(2*ca.pi)*_symbolic_spline(cp, j))


def values_and_derivatives(p, rates, axial_velocity=0.):
    """Return T, Q, dT/domega, dQ/domega at fixed axial flow."""
    n = np.maximum(np.asarray(rates, dtype=float), 0.)
    va = max(float(axial_velocity), 0.)
    c = p['D_prop']/(2*np.pi)
    denom = c*n+1e-8
    j = va/denom
    ct, cp = _splines(_key(p))
    jclip = np.clip(j, ct.x[0], ct.x[-1])
    djdn = -va*c/denom**2
    in_range = (j >= ct.x[0]) & (j <= ct.x[-1])
    outputs, derivatives = [], []
    for spline, scale in ((ct, p['rho']*p['D_prop']**4/(2*np.pi)**2),
                          (cp, p['rho']*p['D_prop']**5/(2*np.pi)**3)):
        raw = spline(jclip)
        coefficient = np.maximum(raw, 0.)
        slope = np.where(in_range & (raw > 0.), spline.derivative()(jclip), 0.)
        outputs.append(scale*n*n*coefficient)
        derivatives.append(scale*(2*n*coefficient+n*n*slope*djdn))
    return *outputs, *derivatives


def positive_thrust_j_limit(p):
    ct, _ = _splines(_key(p))
    return float(brentq(ct, ct.x[-2], ct.x[-1]))


def domain_status(p, rates, v_body):
    rates = np.asarray(rates)
    rpm = rates*60/(2*np.pi)
    va = float(np.asarray(v_body)[0])
    j = max(va, 0.)/(rates*p['D_prop']/(2*np.pi)+1e-8)
    low, high = p['prop_curve']['assumed_rpm_working_range']
    valid = ((rpm >= low-1e-6) & (rpm <= high+1e-6)
             & (j <= positive_thrust_j_limit(p)+1e-8) & (va >= -1e-8))
    return dict(inside_assumed_domain=bool(np.all(valid)), advance_ratio=j,
                rpm=rpm, positive_thrust_j_limit=positive_thrust_j_limit(p),
                meaning='Interpolation in J; frozen 30k-rpm shape at other RPM is an assumption')


def supported_rate_bounds(p, v_body):
    """Conservative subset for authority, NOT new hardware motor bounds."""
    va = max(float(np.asarray(v_body)[0]), 0.)
    source_min, source_max = np.asarray(p['prop_curve']['assumed_rpm_working_range'])*2*np.pi/60
    lower = max(p['n_min'], source_min,
                2*np.pi*va/(p['D_prop']*positive_thrust_j_limit(p)))
    return lower, min(p['n_max'], source_max)
