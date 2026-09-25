"""Static nonlinear authority: independent T(J), Q(J), no constant-Q/T LP.

Multi-start local extrema within the positive-thrust interpolation domain.
Not a global optimality proof or a dynamic motor-reachability guarantee.
"""
import numpy as np
from scipy.linalg import null_space
from scipy.optimize import minimize

from models.team_light.control.geometry import rotor_wrench, rotor_thrusts, control_effectiveness
from models.team_light.control.propeller_curve import supported_rate_bounds, domain_status


def nonlinear_trim_authority(p, trim):
    vb, n0 = trim['v_body'], trim['control']
    if not domain_status(p, n0, vb)['inside_assumed_domain']:
        return dict(valid=False, reason='Trim outside assumed propeller domain')
    nmin, nmax = supported_rate_bounds(p, vb)
    lo, hi, s0 = nmin/p['n_max'], nmax/p['n_max'], n0/p['n_max']
    inertia = np.array([p['Ixx'], p['Iyy'], p['Izz']])
    w0 = rotor_wrench(p, n0, vb)
    scales = np.r_[max(p['mass']*p['g'], abs(w0[0])), [.5]*3]
    def wrench(s):
        return rotor_wrench(p, s*p['n_max'], vb)
    def jac(s):
        return control_effectiveness(p, s*p['n_max'], vb)*np.r_[1., inertia][:, None]*p['n_max']
    margins, endpoints = [], []
    for axis in (1, 2, 3):
        other = [i for i in range(4) if i != axis]
        constraints = dict(type='eq', fun=lambda s: (wrench(s)[other]-w0[other])/scales[other],
                           jac=lambda s: jac(s)[other]/scales[other, None])
        direction = null_space(jac(s0)[other])[:, 0]
        seeds = [s0, np.clip(s0+.04*direction, lo, hi), np.clip(s0-.04*direction, lo, hi)]
        axis_margins, axis_endpoints = [], []
        for sign in (1., -1.):  # minimum, maximum
            feasible = []
            for seed in seeds:
                sol = minimize(lambda s: sign*wrench(s)[axis]/scales[axis], seed,
                               jac=lambda s: sign*jac(s)[axis]/scales[axis],
                               bounds=[(lo, hi)]*4, constraints=constraints, method='SLSQP',
                               options={'ftol': 1e-11, 'maxiter': 200})
                error = float(np.max(abs(wrench(sol.x)[other]-w0[other])))
                if sol.success and error < 1e-7:
                    feasible.append((sol, error))
            if not feasible:
                return dict(valid=False, reason=f'Nonlinear authority search failed on axis {axis}, sign {sign}')
            best, error = min(feasible, key=lambda pair: pair[0].fun)
            axis_margins.append(float(max(0., sign*(w0[axis]-wrench(best.x)[axis])/inertia[axis-1])))
            axis_endpoints.append(dict(rotor_rad_s=best.x*p['n_max'],
                                       wrench= wrench(best.x), fixed_components_max_error=error))
        margins.append(axis_margins)
        endpoints.append(axis_endpoints)
    headroom = float(1-np.max(n0)/p['n_max'])
    fmax = rotor_thrusts(p, np.full(4, nmax), vb)
    f = rotor_thrusts(p, n0, vb)
    return dict(valid=True, method='multistart SLSQP, local static extrema, independent Ct/Cp',
                rpm_headroom_fraction=headroom,
                thrust_upper_headroom_fraction=float(np.min((fmax-f)/fmax)),
                angular_accel_margin_rad_s2=margins, endpoints=endpoints,
                authority_rate_bounds_rad_s=[nmin, nmax],
                passes_predeclared_margin=bool(headroom >= .05 and np.min(margins) >= 3.))
