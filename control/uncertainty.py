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


def cg_offset_params(nominal, offset_body):
    """무게중심(CG) 편차 — 표7(원고 v5.3) 18행(kj 결정 2026-09-28로 17→18행). `perturb_params`의
    곱셈 배율 체계(FACTORS)와는 다르다 — CG 편차는 배율이 아니라 **덧셈식 기하 이동**이라 별도
    함수로 뒀다.

    팀원 공력모델(distributed_light_v1, `models/team_light/control/light_aero.py` docstring:
    "Force points are all expressed about the mass-derived CG.")은 단일 x_cp가 아니라 코
    (`nose_cp`)·`body_strips`·`drag_elements`·`rotor_positions` 전부를 CG 기준 위치벡터로 둔다.
    진짜 CG가 가정한 원점에서 동체좌표 `offset_body`만큼 벗어나 있다면, 그 원점 기준으로 잰 모든
    위치벡터는 새(진짜) CG 기준으로 `position - offset_body`가 된다 — 원점이 그만큼 CG 쪽으로
    옮겨졌으니, 다른 모든 점의 상대위치는 반대 방향으로 그만큼 줄어든다.

    관성 텐서는 안 바꾼다(평행축 정리 보정을 생략한다) — kj 결정: 편차가 작아 그 보정은 2차로
    작다고 가정한다. 이 가정은 여기 기록으로만 남기고 검증하지 않는다.

    offset_body=[0,0,0]이면 반환값이 nominal과 완전히 같아야 한다(호출부의 비트 동일 시험 참고).
    """
    offset = np.asarray(offset_body, dtype=float)
    if offset.shape != (3,):
        raise ValueError('offset_body must be a 3-vector')
    params = deepcopy(nominal)
    params['nose_cp'] = (np.asarray(nominal['nose_cp'], dtype=float) - offset).tolist()
    params['rotor_positions'] = np.asarray(nominal['rotor_positions'], dtype=float) - offset
    params['body_strips'] = [dict(strip, position=(np.asarray(strip['position'], dtype=float)
                                                    - offset).tolist())
                             for strip in nominal['body_strips']]
    params['drag_elements'] = [dict(element, position=(np.asarray(element['position'], dtype=float)
                                                        - offset).tolist())
                               for element in nominal['drag_elements']]
    return params


CG_OFFSET_AXES = {'y': 1, 'z': 2}     # 3벡터에서 어느 칸을 채우는지(0=x는 늘 0)


def cg_offset_arm_fraction(nominal, axis, fraction=0.1):
    """표7 18행 "팔 길이의 10%"를 axis('y' 또는 'z') 방향 오프셋으로. 팔 길이는 로터 위치의
    y·z 성분 노름이다(4개 로터가 대칭이라 어느 로터로 재도 같다). kj 결정(2026-09-28): 부호는
    +로 고정한다(제어기 결과를 보지 않고 미리 정한 값 — 8.3절 오류의 반복을 피하려는 것과
    같은 원칙, tuning-set-must-match-test-severity 메모)."""
    if axis not in CG_OFFSET_AXES:
        raise ValueError(f"axis must be one of {sorted(CG_OFFSET_AXES)}")
    arm = float(np.linalg.norm(np.asarray(nominal['rotor_positions'])[0, 1:3]))
    offset = np.zeros(3)
    offset[CG_OFFSET_AXES[axis]] = fraction*arm
    return cg_offset_params(nominal, offset)


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
