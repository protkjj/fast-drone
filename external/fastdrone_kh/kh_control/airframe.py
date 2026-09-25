"""Fixed component mass/geometry construction, not an airframe search API."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
DATA=Path(__file__).resolve().parents[1]/"data"


def box_inertia(mass, dimensions):
    a, b, c = dimensions
    return np.diag([b*b+c*c, a*a+c*c, a*a+b*b]) * mass / 12


def mass_properties(elements):
    mass = sum(e['mass'] for e in elements)
    cg = sum(e['mass']*np.asarray(e['position']) for e in elements) / mass
    inertia = np.zeros((3, 3))
    for element in elements:
        r = np.asarray(element['position']) - cg
        inertia += (np.asarray(element['inertia_centroid'])
                    + element['mass']*(np.dot(r, r)*np.eye(3)-np.outer(r, r)))
    return float(mass), cg, inertia


def build_fixed_airframe():
    """Mass/geometry construction for pack_forward only; propulsion finalized by factory."""
    reference = json.loads((DATA / 'apc_reference.json').read_text(encoding='utf-8'))
    L, D = .32, .075
    nose_length = .30*L
    # Rotor radius + body radius + >=5 mm radial clearance, rounded upward.
    arm = .115
    prop_d = reference['diameter_m']
    prop_mass = reference['prop_mass_kg']
    if np.sqrt(2)*arm <= prop_d or arm-prop_d/2 <= D/2:
        raise ValueError('Rotor disks overlap one another or the body envelope.')
    s = arm/np.sqrt(2)
    rotor_datum = np.array([[-.015, s, s], [-.015, -s, s],
                            [-.015, -s, -s], [-.015, s, -s]])
    elements = []

    def add_box(name, mass, position, dimensions):
        elements.append(dict(name=name, mass=mass, position=list(position),
                             inertia_centroid=box_inertia(mass, dimensions).tolist(),
                             dimensions_m=list(dimensions), basis='design assumption'))

    pack_x = .020
    add_box('battery_mass_only', .360, [pack_x, 0, 0], [.095, .046, .042])
    add_box('camera_payload', .150, [.070, 0, 0], [.030, .035, .035])
    add_box('electronics', .100, [-.080, 0, 0], [.035, .044, .040])
    add_box('wiring_and_mounts', .110, [-.040, 0, 0], [.090, .050, .050])
    # Equivalent thin cylindrical shell, including canopy; geometric surrogate.
    shell_mass = .190
    shell_I = np.diag([D*D/4, L*L/12+D*D/8, L*L/12+D*D/8])*shell_mass
    elements.append(dict(name='shell_equivalent', mass=shell_mass, position=[0., 0., 0.],
                         inertia_centroid=shell_I.tolist(), basis='design assumption'))
    arm_length = arm-D/2
    arm_centers = []
    for i, rotor in enumerate(rotor_datum):
        direction = np.r_[0., rotor[1:]/arm]
        center = np.array([rotor[0], 0, 0])+direction*(D/2+arm)/2
        arm_centers.append(center)
        arm_mass = .22*arm_length  # assumed assembly mass per metre
        arm_I = arm_mass*arm_length**2/12*(np.eye(3)-np.outer(direction, direction))
        elements.append(dict(name=f'arm_{i+1}', mass=arm_mass, position=center.tolist(),
                             inertia_centroid=arm_I.tolist(), basis='design assumption'))
        add_box(f'motor_{i+1}', .045, rotor, [.025, .028, .028])
        # Time-averaged two-blade prop inertia, not a CAD-derived tensor.
        prop_Ixx = prop_mass*prop_d**2/12
        elements.append(dict(name=f'prop_{i+1}', mass=prop_mass,
                             position=(rotor+np.array([.015, 0, 0])).tolist(),
                             inertia_centroid=np.diag([prop_Ixx, prop_Ixx/2,
                                                      prop_Ixx/2]).tolist(),
                             basis='APC listed mass; uniform blade approximation'))
    mass, cg, inertia = mass_properties(elements)
    if not np.allclose(inertia, np.diag(np.diag(inertia)), atol=1e-12):
        raise ValueError('Current dynamics require principal body axes (diagonal inertia).')
    # These coefficients come from a fixed source row, not from trim fitting.
    rho = 1.225
    ct, cp = reference['static_rows_rpm_ct_cp'][1][1:]
    kt = ct*rho*prop_d**4/(2*np.pi)**2
    kq = cp*rho*prop_d**5/(2*np.pi)**3
    nmax = 33000.*2*np.pi/60
    # Geometric crossflow integration stations over cylinder + conical nose.
    edges = np.linspace(-L/2, L/2, 13)
    centers = (edges[:-1]+edges[1:])/2
    diameters = D*np.minimum(1., (L/2-centers)/nose_length)
    body_strips = [dict(position=(np.array([x, 0., 0.])-cg).tolist(),
                        projected_area=float(d*np.diff(edges)[i]))
                   for i, (x, d) in enumerate(zip(centers, diameters))]
    frontal = np.pi*D**2/4
    wetted = np.pi*D*(L-nose_length)+np.pi*(D/2)*np.hypot(nose_length, D/2)
    # C_Nalpha=2 and cone CP at 2/3 nose length from tip: small-angle anchor only.
    nose_cp = np.array([L/2-2*nose_length/3, 0., 0.])-cg
    drag_elements = []
    for center, rotor in zip(arm_centers, rotor_datum):
        # Elliptic support fairing assumptions; no wing/lifting surface added.
        drag_elements.append(dict(position=(center-cg).tolist(),
                                  cda=[.25*.004*arm_length,
                                       1.1*.015*arm_length, 1.1*.015*arm_length]))
        drag_elements.append(dict(position=(rotor-cg).tolist(),
                                  cda=[.4*np.pi*.028**2/4,
                                       1.1*.028*.025, 1.1*.028*.025]))
    return dict(
        profile_id='light_rocket_v2_pack_forward',
        layout='rocket_x', thrust_axis=np.array([1., 0., 0.]),
        mass=mass, Ixx=inertia[0, 0], Iyy=inertia[1, 1], Izz=inertia[2, 2],
        cg_datum=cg, inertia_tensor=inertia, mass_elements=deepcopy(elements),
        body_length=L, body_diameter=D, nose_length=nose_length,
        S_ref=frontal, d_ref=D, wetted_area=wetted, rho=rho, g=9.81,
        aero_model='distributed_light_v1', body_strips=body_strips,
        nose_cp=nose_cp, C_Na=2., C_dc=1.1, C_pressure=.20, C_f=.005,
        drag_elements=drag_elements, aero_scale=1.,
        # No independent fitted x_cp or empirical pitch damping coefficient.
        num_rotors=4, arm_length=arm, rotor_positions=rotor_datum-cg,
        rotor_directions=np.array([1., -1., 1., -1.]),
        D_prop=prop_d, k_T=kt, k_Q=kq, J_max=1.3485,
        I_rotor=prop_mass*prop_d**2/12+.008*.014**2/2,
        tau_m=.020, n_min=0., n_max=nmax,
        consistent_allocation=True, virtual_actuator_constraints=True,
        reference=reference, assumptions_only=True)
