"""Bounded planar trims for the body +x rocket layout.

theta is nose elevation above world horizontal (90 degrees at hover).
dn is half the difference between the +body-z and -body-z rotor pairs.
An unsuccessful search is NOT proof of a global physical speed limit.
"""
import numpy as np
from scipy.optimize import least_squares, OptimizeResult
from scipy.spatial.transform import Rotation

from models.team_light.control.vehicle_params import vehicle_params
from models.team_light.control.dynamics import AxialDronePlant, NX
from models.team_light.control.geometry import pitch_quaternion, rotor_thrusts


def find_trim(params, V_cruise, *, strict=True):
    """Find [theta, n_+z, n_-z] without clipping optimizer variables.

    strict=False is for diagnostic sweeps only: inspect valid/residual before
    using the returned state. Non-inverted forward flight is assumed.
    """
    if not np.isfinite(V_cruise) or V_cruise < 0:
        raise ValueError('V_cruise must be a finite nonnegative speed.')
    plant = AxialDronePlant(params)
    n_hov = np.sqrt(params['mass'] * params['g'] / (4 * params['k_T']))
    positive_z = np.asarray(params['rotor_positions'])[:, 2] > 0

    def state(opt):
        theta, n_plus, n_minus = opt
        x = np.zeros(NX)
        x[3] = V_cruise
        x[6:10] = pitch_quaternion(theta)
        x[13:17] = np.where(positive_z, n_plus, n_minus)
        return x

    def residual(opt):
        x = state(opt)
        xd = plant.evaluate_xdot(x, x[13:17])
        return xd[[3, 5, 11]]

    bounds = ([-np.pi/2, params['n_min'], params['n_min']],
              [np.pi/2, params['n_max'], params['n_max']])
    best = None
    if V_cruise == 0.0 and params['n_min'] <= n_hov <= params['n_max']:
        opt = np.array([np.pi/2, n_hov, n_hov])
        best = OptimizeResult(x=opt, fun=residual(opt), success=True,
                              message='Analytical nose-up hover')
    else:
        # At large advance ratio a low-speed initial guess may have zero thrust
        # AND zero derivative. Try higher initial rotor rates as well.
        for rate in (n_hov, .6*params['n_max'], .9*params['n_max']):
            for angle in (np.pi/2 - 1e-6, np.pi/4, 0.15, 0.0, -0.15):
                n0 = np.clip(rate, params['n_min'], params['n_max'])
                sol = least_squares(residual, [angle, n0, n0], bounds=bounds,
                                    x_scale=[1., max(n_hov, 1.), max(n_hov, 1.)],
                                    ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=600)
                if best is None or np.linalg.norm(sol.fun) < np.linalg.norm(best.fun):
                    best = sol
                if best.success and np.linalg.norm(best.fun) < 1e-8:
                    break
            if best.success and np.linalg.norm(best.fun) < 1e-8:
                break
    theta, n_plus, n_minus = best.x
    x_trim = state(best.x)
    u_trim = x_trim[13:17].copy()
    xd = plant.evaluate_xdot(x_trim, u_trim)
    res_norm = float(np.linalg.norm(np.r_[xd[3:6], xd[10:13]]))
    valid = bool(best.success and np.isfinite(res_norm) and res_norm < 1e-6
                 and np.all(u_trim >= params['n_min'])
                 and np.all(u_trim <= params['n_max']))
    vb = Rotation.from_quat(x_trim[6:10]).as_matrix().T @ x_trim[3:6]
    result = dict(state=x_trim, control=u_trim, theta=float(theta),
                  n_eq=float((n_plus+n_minus)/2), dn=float((n_plus-n_minus)/2),
                  alpha=float(np.arctan2(vb[2], vb[0])), v_body=vb,
                  residual=res_norm, xdot=xd, valid=valid, converged=valid,
                  solver_success=bool(best.success), message=str(best.message),
                  T_total=float(np.sum(rotor_thrusts(params, u_trim, vb))))
    if strict and not valid:
        raise ValueError(f'No valid bounded rocket trim found at {V_cruise:g} m/s '
                         f'(residual={res_norm:.3g}). Do not reuse the old '
                         'quad-layout trim or compare from this state.')
    return result


def print_trim(trim, V_cruise, params):
    print(f"  speed={V_cruise:g} m/s, nose elevation={np.degrees(trim['theta']):.3f} deg")
    print(f"  alpha={np.degrees(trim['alpha']):.3f} deg, n_mean={trim['n_eq']:.3f} rad/s")
    print(f"  +z/-z pair differential={trim['dn']:.3f} rad/s")
    print(f"  rotor rates={trim['control']}")
    print(f"  thrust={trim['T_total']:.3f} N, weight={params['mass']*params['g']:.3f} N")
    print(f"  residual={trim['residual']:.3e}, valid={trim['valid']}")


def trim_speed_sweep(params, V_range=None, verbose=True):
    """All search results and highest valid sampled speed, not physical Vmax."""
    if V_range is None:
        V_range = np.arange(0, 90, 5)
    results = []
    for V in V_range:
        trim = find_trim(params, float(V), strict=False)
        n_max = float(np.max(trim['control']))
        saturated = bool(n_max > params['n_max'] * .95)
        results.append(dict(V=V, trim=trim, converged=trim['valid'],
                            n_max_actual=n_max, saturated=saturated))
        if verbose:
            flag = 'VALID' if trim['valid'] else 'INVALID'
            print(f"  {V:5.1f} m/s | theta={np.degrees(trim['theta']):+7.2f} deg "
                  f"| n_max={n_max:7.1f} | residual={trim['residual']:.2e} | {flag}"
                  + (' (less than 5% upper speed margin)' if saturated else ''))
    valid = [float(r['V']) for r in results if r['converged']]
    return results, max(valid) if valid else float('nan')


if __name__ == '__main__':
    print('Fixed pack_forward v2: bounded planar trim sweep')
    results, highest = trim_speed_sweep(vehicle_params)
    print(f'Highest valid SAMPLED speed: {highest:g} m/s')
    print('This is not a global maximum-speed or closed-loop stability proof.')
    print('Invalid trims cannot be used as equilibria; no automatic resizing is performed.')
