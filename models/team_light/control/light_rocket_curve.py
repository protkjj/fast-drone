"""Only the selected pack_forward v2 aircraft is published in this repository."""
import json
from pathlib import Path
import numpy as np
from models.team_light.control.airframe import build_fixed_airframe
from models.team_light.control.propeller_curve import MODEL, positive_thrust_j_limit


def make_curve_rocket(candidate='pack_forward'):
    """Compatibility API: new dictionary each call; no alternate candidate."""
    if candidate != 'pack_forward':
        raise ValueError('This repository distributes only the frozen pack_forward v2 aircraft.')
    p = build_fixed_airframe()
    path = Path(__file__).resolve().parents[1]/'data'/'apc_curve.json'
    curve = json.loads(path.read_text(encoding='utf-8'))
    p.update(profile_id='light_rocket_v2_pack_forward', propulsion_model=MODEL, prop_curve=curve)
    _, ct0, cp0 = curve['knots'][0]
    p['k_T'] = ct0*p['rho']*p['D_prop']**4/(2*np.pi)**2
    p['k_Q'] = cp0*p['rho']*p['D_prop']**5/(2*np.pi)**3
    p['J_max'] = positive_thrust_j_limit(p)
    return p
