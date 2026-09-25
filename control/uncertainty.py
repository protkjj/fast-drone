"""Plant-only model mismatch; controllers always receive nominal parameters."""
from copy import deepcopy
from itertools import product

import numpy as np


# Multipliers: grouped aero factors scale related coefficients together.
# Each inertia axis can also be varied independently.
FACTORS = {
    'mass': ('mass',),
    'Ixx': ('Ixx',), 'Iyy': ('Iyy',), 'Izz': ('Izz',),
    'aero': ('aero_scale',),
    'drag': ('C_pressure', 'C_f'),
    'normal_force': ('C_Na', 'C_dc'),
    'thrust': ('k_T',), 'torque': ('k_Q',),
    'motor_tau': ('tau_m',),
}

# Exploratory ranges; replace with measured tolerances for research claims.
DEFAULT_RANGES = {key: [0.8, 1.2] for key in FACTORS if key != 'aero'}


def validate_ranges(ranges):
    if not isinstance(ranges, dict) or not ranges:
        raise ValueError('uncertainty ranges must be a nonempty object')
    for key, bounds in ranges.items():
        if key not in FACTORS:
            raise ValueError(f'unknown uncertainty factor: {key}')
        if len(bounds) != 2 or not np.all(np.isfinite(bounds)) or not 0 < bounds[0] <= bounds[1]:
            raise ValueError(f'{key}: expected two positive ordered bounds')
    return ranges


def perturb_params(nominal, factors):
    params = deepcopy(nominal)
    for factor, value in factors.items():
        if factor not in FACTORS or not np.isfinite(value) or value <= 0:
            raise ValueError(f'invalid factor {factor}={value}')
        for key in FACTORS[factor]:
            params[key] = nominal[key]*value
        if factor == 'drag':
            for element in params['drag_elements']:
                element['cda'] = [v*value for v in element['cda']]
        if factor in ('thrust', 'torque'):
            column = 1 if factor == 'thrust' else 2
            for row in params['prop_curve']['knots']:
                row[column] *= value
    # Bookkeeping follows the dynamical principal moments. This is epistemic
    # parameter error, not a redesign of mass elements or CG geometry.
    if any(k in factors for k in ('Ixx', 'Iyy', 'Izz')):
        for i, key in enumerate(('Ixx', 'Iyy', 'Izz')):
            params['inertia_tensor'][i, i] = params[key]
    return params


def sweep_cases(ranges, points=3, mode='oat'):
    """OAT isolates causes; grid is the Cartesian product of selected factors."""
    validate_ranges(ranges)
    if points < 2 or mode not in ('oat', 'grid'):
        raise ValueError('points >= 2 and mode oat/grid are required')
    cases = [dict(case_id='nominal', factors={})]
    keys = sorted(ranges)
    levels = [np.linspace(*ranges[key], points).tolist() for key in keys]
    if mode == 'grid':
        values = (dict(zip(keys, row)) for row in product(*levels))
    else:
        values = ({key: v} for key, vals in zip(keys, levels) for v in vals)
    seen = {()}
    for factors in values:
        canonical = tuple(sorted((k, v) for k, v in factors.items() if v != 1.0))
        if canonical in seen:
            continue
        seen.add(canonical)
        cases.append(dict(case_id=f'sweep_{len(cases):05d}', factors=factors))
    return cases


def monte_carlo_cases(ranges, trials=100, seed=42):
    """Independent uniform draws of ALL selected factors for each trial."""
    validate_ranges(ranges)
    if trials < 1:
        raise ValueError('trials must be positive')
    rng = np.random.default_rng(seed)
    return [dict(case_id=f'mc_{i:05d}', factors={
        key: float(rng.uniform(*ranges[key])) for key in sorted(ranges)
    }) for i in range(trials)]
