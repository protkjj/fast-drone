"""Rocket-layout frame helpers (world z-up, quaternion xyzw, body-to-world).

Body +x is the nose/thrust axis. Rotors lie in the body yz plane.
rotor_directions denotes the sign of the reaction torque on the BODY
along +x; rotor angular momentum has the opposite sign.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from models.team_light.control.propeller_curve import uses_curve, values_and_derivatives


def thrust_axis(params):
    axis = np.asarray(params['thrust_axis'], dtype=float)
    if axis.shape != (3,) or not np.allclose(axis, [1., 0., 0.]):
        raise ValueError('This branch supports rocket layout: body +x thrust only.')
    return axis


def pitch_quaternion(theta):
    """Nose elevation above the world horizontal; pi/2 is nose-up hover."""
    return (Rotation.from_euler('x', np.pi)
            * Rotation.from_euler('y', theta)).as_quat()


def hover_quaternion():
    return pitch_quaternion(np.pi / 2)


def axial_speed(v_body, params):
    return max(float(np.dot(thrust_axis(params), v_body)), 0.0)


def rotor_thrusts(params, n, v_body=None):
    """Thrust from the selected propeller model along body +x."""
    n = np.asarray(n, dtype=float)
    va = axial_speed(v_body, params) if v_body is not None else 0.0
    if uses_curve(params):
        return values_and_derivatives(params, n, va)[0]
    denom = n * params['D_prop'] / (2 * np.pi) + 1e-8
    fac = np.maximum(1 - va / (params['J_max'] * denom), 0.0)
    return params['k_T'] * n**2 * fac


def rotor_torques(params, n, v_body=None):
    """Unsigned reaction torque magnitude; body signs are applied separately."""
    if uses_curve(params):
        va = axial_speed(v_body, params) if v_body is not None else 0.
        return values_and_derivatives(params, n, va)[1]
    return rotor_thrusts(params, n, v_body)*params['k_Q']/params['k_T']


def rotor_wrench(params, n, v_body=None):
    """[total axial thrust, body Mx, My, Mz], excluding gyro at nonzero omega."""
    thrust = rotor_thrusts(params, n, v_body)
    torque = rotor_torques(params, n, v_body)
    axis = thrust_axis(params)
    moments = np.sum(np.cross(params['rotor_positions'], thrust[:, None]*axis), axis=0)
    moments += axis*np.dot(params['rotor_directions'], torque)
    return np.r_[np.sum(thrust), moments]


def control_effectiveness(params, n, v_body=None):
    """d[T, angular acceleration]/dn, excluding rotor gyro coupling.

Uses the derivative of the actual clipped propeller map. A stopped or
zero-thrust rotor may have zero effectiveness; callers must handle rank loss.
"""
    n = np.maximum(np.asarray(n, dtype=float), 0.0)
    va = axial_speed(v_body, params) if v_body is not None else 0.0
    if uses_curve(params):
        _, _, dT, dQ = values_and_derivatives(params, n, va)
        axis = thrust_axis(params)
        arms = np.cross(np.asarray(params['rotor_positions']), axis)
        dm = arms*dT[:, None]+(params['rotor_directions']*dQ)[:, None]*axis
        inertia = np.array([params['Ixx'], params['Iyy'], params['Izz']])
        return np.vstack([dT, (dm/inertia).T])
    c = params['D_prop'] / (2 * np.pi)
    denom = c * n + 1e-8
    raw = 1 - va / (params['J_max'] * denom)
    dfac = va * c / (params['J_max'] * denom**2)
    dT = np.where(raw > 0, params['k_T'] * (2*n*raw + n**2*dfac), 0.)
    axis = thrust_axis(params)
    arms = np.cross(np.asarray(params['rotor_positions']), axis)
    reaction = (np.asarray(params['rotor_directions'])[:, None]
                * (params['k_Q'] / params['k_T']) * axis)
    inertia = np.array([params['Ixx'], params['Iyy'], params['Izz']])
    return np.vstack([dT, ((arms + reaction) / inertia).T * dT])


def rotor_speeds_for_thrust(params, target, v_body=None):
    """Bounded inverse of the SAME clipped thrust map (new profiles only).

    The 8 kg profile retains its historical static allocation unless explicitly
    opted in. At zero target thrust, choose the lower motor bound, not a
    spinning zero-thrust branch. Saturation is explicit in the returned rates.
    """
    target = np.maximum(np.asarray(target, dtype=float), 0.)
    if not params.get('consistent_allocation', False):
        return np.clip(np.sqrt(target/params['k_T']), params['n_min'], params['n_max'])
    lo = np.full(target.shape, params['n_min'], dtype=float)
    hi = np.full(target.shape, params['n_max'], dtype=float)
    clipped = np.clip(target, rotor_thrusts(params, lo, v_body),
                      rotor_thrusts(params, hi, v_body))
    for _ in range(42):
        mid = (lo+hi)/2
        lower = rotor_thrusts(params, mid, v_body) < clipped
        lo = np.where(lower, mid, lo)
        hi = np.where(lower, hi, mid)
    return np.where(target <= 0., params['n_min'], (lo+hi)/2)


def allocate_wrench(params, desired, v_body=None):
    """Bounded allocation using actual T(J), Q(J) for curve profiles.

    This is numerical allocation, not a new control architecture. Infeasible
    requests return the best bounded approximation, as legacy clipping did.
    """
    from scipy.optimize import least_squares
    from models.team_light.control.dynamics import compute_allocation_matrix
    A, inverse = compute_allocation_matrix(params, static_reference=True)
    desired = np.asarray(desired, dtype=float)
    initial = rotor_speeds_for_thrust(params, inverse@desired, v_body)
    if not uses_curve(params):
        return initial
    if np.allclose(desired, 0., atol=1e-14):
        return np.full(4, params['n_min'])
    scale = np.maximum(np.sum(abs(A), axis=1)*params['k_T']*params['n_max']**2, 1e-3)
    inertia_scale = np.r_[1., params['Ixx'], params['Iyy'], params['Izz']]
    upper = params['n_max']
    def residual(s):
        return (rotor_wrench(params, s*upper, v_body)-desired)/scale
    def jacobian(s):
        return control_effectiveness(params, s*upper, v_body)*inertia_scale[:, None]*upper/scale[:, None]
    best = None
    for seed in (initial/upper, np.full(4, .9)):
        sol = least_squares(residual, seed, jac=jacobian,
                            bounds=(params['n_min']/upper, 1.),
                            xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=120)
        if best is None or np.linalg.norm(sol.fun) < np.linalg.norm(best.fun):
            best = sol
        if np.linalg.norm(best.fun) < 1e-10:
            break
    return best.x*upper


def force_to_attitude(force, heading=0.0, max_tilt=np.pi):
    """Align body +x with requested world force; heading fixes the free roll.

max_tilt is the thrust-axis tilt FROM WORLD UP, not the body's pitch angle.
The default reference side axis is world -y, matching pitch_quaternion().
"""
    magnitude = float(np.linalg.norm(force))
    b1 = np.asarray(force, dtype=float) / magnitude if magnitude > 1e-8 else np.array([0., 0., 1.])
    cos_limit = np.cos(max_tilt)
    if b1[2] < cos_limit:
        horizontal = b1[:2]
        norm = np.linalg.norm(horizontal)
        horizontal = horizontal / norm if norm > 1e-8 else np.array([1., 0.])
        b1 = np.r_[horizontal * np.sin(max_tilt), cos_limit]
    side = np.array([np.sin(heading), -np.cos(heading), 0.])
    b2 = side - b1 * np.dot(b1, side)
    if np.linalg.norm(b2) < 1e-6:
        side = np.eye(3)[np.argmin(np.abs(b1))]
        b2 = side - b1 * np.dot(b1, side)
    b2 /= np.linalg.norm(b2)
    return magnitude, np.column_stack([b1, b2, np.cross(b1, b2)])
