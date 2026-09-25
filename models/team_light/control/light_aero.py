"""Low-order component aerodynamics, NOT a CFD/wind-tunnel calibrated model.

Small-angle cone force + distributed viscous crossflow + component drag.
High-alpha/reverse-flow continuation is an explicitly unvalidated assumption.
Force points are all expressed about the mass-derived CG.
"""
import casadi as ca


def light_aerodynamics(v_body, omega, p):
    eps = 1e-8
    rho = p['rho']
    force = ca.SX.zeros(3)
    moment = ca.SX.zeros(3)

    def add_at(position, kind, coefficients):
        nonlocal force, moment
        r = ca.DM(position)
        local = v_body+ca.cross(omega, r)
        if kind == 'nose':
            transverse = ca.vertcat(0., local[1], local[2])
            f = -.5*rho*coefficients*ca.sqrt(local[0]**2+eps)*transverse
        elif kind == 'crossflow':
            transverse = ca.vertcat(0., local[1], local[2])
            f = -.5*rho*coefficients*ca.sqrt(ca.dot(transverse, transverse)+eps)*transverse
        else:
            f = -.5*rho*ca.sqrt(ca.dot(local, local)+eps)*ca.DM(coefficients)*local
        force += f
        moment += ca.cross(r, f)

    # Skin-friction area scales with body length; pressure area with diameter.
    axial_cda = p['C_pressure']*p['S_ref']+p['C_f']*p['wetted_area']
    u = v_body[0]
    force += ca.vertcat(-.5*rho*axial_cda*u*ca.sqrt(u*u+eps), 0., 0.)
    add_at(p['nose_cp'], 'nose', p['C_Na']*p['S_ref'])
    for strip in p['body_strips']:
        add_at(strip['position'], 'crossflow', p['C_dc']*strip['projected_area'])
    for element in p['drag_elements']:
        add_at(element['position'], 'drag', element['cda'])
    return p['aero_scale']*force, p['aero_scale']*moment
