"""User-selected aircraft lock. Controller tuning must never change this hash."""
import hashlib
import json

from models.team_light.control.light_rocket_curve import make_curve_rocket
from models.team_light.control.serialization import jsonable

PARAMETER_SHA256 = '13a1dcc6332987eba93ea4653f4d6087741d1ac085701982ed94c22e6658c807'


def parameter_hash(p):
    return hashlib.sha256(json.dumps(jsonable(p), sort_keys=True,
                                    separators=(',', ':')).encode()).hexdigest()


def baseline_params():
    p = make_curve_rocket('pack_forward')
    if parameter_hash(p) != PARAMETER_SHA256:
        raise ValueError('Frozen aircraft changed; do not silently retune the baseline.')
    return p
