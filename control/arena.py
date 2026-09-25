"""경기장(arena) 설정 — `configs/arena.json` 하나로 모든 시나리오를 정의한다.

kj 작업지시서(2026-09-25) 작업 B·D·F의 공통 기반이다. 이 모듈이 하는 일:

  1) 설정 파일 읽기·구조 검사              load_config, validate_config
  2) 1절 확정 사항과 대조(불변식 I-7)       check_confirmed_facts
  3) 시나리오 → 민석 스위트 입력 변환       build_scenarios
     (`validation_suite.run_trial`이 먹는 profile·case 형태)
  4) 참조 프로필의 가속도 비율 ρ 맞추기     available_acceleration,
     (논문 v5.3 §5.5: ρ = a_ref / a_avail)  ramp_duration_for_rho

제어기는 여기서 만들지 않는다 — `control/arena_factory.py`의 몫이다.
시나리오 정의와 제어기 생성을 한 파일에 섞으면, 나중에 경기장이 기울었을 때
"시나리오가 달랐는지, 제어기가 다른 정보를 받았는지"를 가르기 어렵다.
"""
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from control.mission_profiles import MissionProfile, GustProfile, SmoothstepProfile
from control.uncertainty import FACTORS, perturb_params

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'configs' / 'arena.json'
SCHEMA = 'arena/1'
CONTROLLERS = ('V13', 'M17', 'F13', 'GSLQR', 'CPID')
SCENARIO_TYPES = ('mission', 'gust', 'reference')
# 팀 기체의 명목 트림 확인 범위(docs/VALIDATION.md). 이 밖의 속도는 팀
# 문서가 검증하지 않았으므로 설정 단계에서 막는다.
SPEED_RANGE = (0.0, 85.0)


# ══════════════════════════════════════════════════════════════════
# 1) 설정 읽기·검사
# ══════════════════════════════════════════════════════════════════

def load_config(path=DEFAULT_CONFIG):
    """설정 파일을 읽고 구조를 검사해 돌려준다."""
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    validate_config(config)
    return config


def config_sha256(config):
    """설정 내용의 해시. 키 순서·공백과 무관하게 같은 내용이면 같은 값이다."""
    text = json.dumps(config, sort_keys=True, separators=(',', ':'), ensure_ascii=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def resolve_speed(value, config):
    """'V_L'/'V_H' 같은 기호 속도를 숫자로. 숫자는 그대로 둔다."""
    if isinstance(value, str):
        return float(config['speeds_m_s'][value])
    return float(value)


def _is_multiple(value, step):
    return abs(value/step - round(value/step)) < 1e-7


def validate_config(config):
    """구조가 틀린 설정은 실행 전에 ValueError로 막는다.

    여기서 보는 것은 '돌릴 수 있는가'다. '확정 사항과 맞는가'는
    check_confirmed_facts가 따로 본다 — 둘을 섞으면 확정 사항이 바뀌었을 때
    어디를 고쳐야 할지 헷갈린다.
    """
    if config.get('schema') != SCHEMA:
        raise ValueError(f"schema must be {SCHEMA!r}, got {config.get('schema')!r}")
    for key in ('confirmed', 'vehicle', 'plant', 'controller_model', 'speeds_m_s',
                'altitude_m', 'preview_horizon_s', 'nmpc_common', 'controllers',
                'seeds', 'acceptance', 'paper_criteria', 'scenarios'):
        if key not in config:
            raise ValueError(f'missing config key: {key}')
    dt = float(config['plant']['dt_s'])
    if not dt > 0:
        raise ValueError('plant.dt_s must be positive')
    for label, spec in config['controllers'].items():
        if label not in CONTROLLERS or spec.get('kind') != label:
            raise ValueError(f'unknown controller entry {label!r}: {spec}')
    nm = config['nmpc_common']
    if not _is_multiple(nm['dt_ctrl_s'], dt):
        raise ValueError('nmpc_common.dt_ctrl_s must be a multiple of the plant step')
    horizon = nm['N']*nm['dt_pred_s']
    if abs(horizon - config['preview_horizon_s']) > 1e-9:
        raise ValueError(f'preview_horizon_s must equal N*dt_pred ({horizon:g} s) so every '
                         'controller gets exactly what the NMPC family looks ahead')
    lo_main, hi_main = config['seeds']['main']
    lo_tune, hi_tune = config['seeds']['tuning']
    if not (hi_main < lo_tune or hi_tune < lo_main):
        raise ValueError('main-test and tuning seed ranges must not overlap')
    ids = [s['id'] for s in config['scenarios']]
    if len(ids) != len(set(ids)):
        raise ValueError('scenario ids must be unique')
    for s in config['scenarios']:
        if s['type'] not in SCENARIO_TYPES:
            raise ValueError(f"{s['id']}: unknown scenario type {s['type']!r}")
        speeds = ([s['speed']] if 'speed' in s else [s['from'], s['to']])
        for v in speeds:
            v = resolve_speed(v, config)
            if not SPEED_RANGE[0] <= v <= SPEED_RANGE[1]:
                raise ValueError(f"{s['id']}: speed {v} outside checked range {SPEED_RANGE}")
        durations = (s.get('durations_s') or s.get('times_s') or
                     [s.get('lead_s'), s.get('tail_s')])
        for d in durations:
            if d is None or d <= 0 or not _is_multiple(d, dt):
                raise ValueError(f"{s['id']}: durations must be positive multiples of {dt}")
        for factor in s.get('perturbation', {}):
            if factor not in FACTORS:
                raise ValueError(f"{s['id']}: unknown perturbation factor {factor!r}")
        if s['type'] == 'gust' and s['direction'] not in ('lateral', 'vertical'):
            raise ValueError(f"{s['id']}: gust direction must be lateral or vertical")
    return config


# ══════════════════════════════════════════════════════════════════
# 2) 확정 사항 대조 — 불변식 I-7
# ══════════════════════════════════════════════════════════════════

def check_confirmed_facts(config, native_params=None):
    """설정이 작업지시서 1절 확정 사항과 어긋나는 곳을 문자열 목록으로 돌려준다.

    빈 목록이면 통과. 예외 대신 목록을 돌려주는 이유는, 검증 스크립트가
    어긋난 항목을 한꺼번에 표로 보여줄 수 있게 하기 위해서다.
    """
    from models.team_light.control.baseline_v2 import PARAMETER_SHA256, parameter_hash

    bad = []
    c = config['confirmed']
    veh = config['vehicle']
    if veh['profile_id'] != c['vehicle_profile_id']:
        bad.append(f"vehicle.profile_id {veh['profile_id']} != confirmed {c['vehicle_profile_id']}")
    if abs(veh['mass_kg'] - c['vehicle_mass_kg']) > 1e-12:
        bad.append(f"vehicle.mass_kg {veh['mass_kg']} != confirmed {c['vehicle_mass_kg']}")
    if veh['parameter_sha256'] != PARAMETER_SHA256:
        bad.append('vehicle.parameter_sha256 differs from the frozen team aircraft hash')
    if native_params is not None:
        if native_params.get('profile_id') != c['vehicle_profile_id']:
            bad.append(f"plant profile_id {native_params.get('profile_id')} != confirmed")
        if abs(float(native_params['mass']) - c['vehicle_mass_kg']) > 1e-12:
            bad.append(f"plant mass {native_params['mass']} != confirmed {c['vehicle_mass_kg']}")
        if parameter_hash(native_params) != veh['parameter_sha256']:
            bad.append('plant parameter hash differs from vehicle.parameter_sha256')
    if config['plant']['class'] != 'models.team_light.control.dynamics.AxialDronePlant':
        bad.append('plant must be the team distributed-aero AxialDronePlant')
    if config['controller_model']['builder'] != 'control.kh_adapter.build_controller_params':
        bad.append('controller model must be the lumped fit from control.kh_adapter')
    for name, key in (('V_L', 'V_L_m_s'), ('V_H', 'V_H_m_s')):
        if abs(config['speeds_m_s'][name] - c[key]) > 1e-12:
            bad.append(f'{name} {config["speeds_m_s"][name]} != confirmed {c[key]}')
    if set(config['controllers']) != set(CONTROLLERS):
        bad.append(f"controllers {sorted(config['controllers'])} != required {sorted(CONTROLLERS)}")
    missions = [s for s in config['scenarios'] if s['type'] == 'mission']
    if not missions:
        bad.append('no integrated mission scenario')
    for s in missions:
        if [float(d) for d in s['durations_s']] != [float(d) for d in c['mission_phases_s']]:
            bad.append(f"{s['id']}: phases {s['durations_s']} != confirmed {c['mission_phases_s']}")
        if c['mission_takeoff'] is False and s.get('takeoff', False):
            bad.append(f"{s['id']}: takeoff is excluded by the confirmed mission definition")
        if c['mission_disturbance'] is False and ('gust' in s or 'wind' in s):
            bad.append(f"{s['id']}: the integrated mission carries no disturbance")
    for s in (s for s in config['scenarios'] if s['type'] == 'gust'):
        if c['gust_wind_reference'] == 'absolute' and 'peak_m_s' not in s:
            bad.append(f"{s['id']}: gust must be given as an absolute wind speed (peak_m_s)")
        if c['gust_start'] == 'common_trim_cruise' and 'speed' not in s:
            bad.append(f"{s['id']}: gust must start from a common cruise trim speed")
    return bad


# ══════════════════════════════════════════════════════════════════
# 4) 가속도 비율 ρ (논문 §5.5)
# ══════════════════════════════════════════════════════════════════

def smoothstep_rate(u):
    """식(32) S(u)의 도함수 S'(u) = 630·u⁴·(1-u)⁴.

    S(u)=126u⁵-420u⁶+540u⁷-315u⁸+70u⁹ 를 미분해 인수분해한 것이다.
    최댓값은 u=0.5에서 630/256 ≈ 2.4609 — 램프 평균 기울기의 2.46배가 순간 최대
    가속도라는 뜻이다(반코사인 램프는 π/2≈1.57배).
    """
    u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
    return 630.0*u**4*(1.0-u)**4


def _our_state(cp, V, theta, n_pos, n_neg):
    """control/trim.py의 build_trim_state와 같은 구성(우리 쪽 자세 규약).

    트림 솔버 내부 함수라 재사용할 수 없어서 같은 식을 옮겨 적었다 —
    test_arena_fairness가 트림점에서 두 구성이 비트 단위로 같은지 확인한다.
    """
    from control.dynamics import NX
    if cp.get('thrust_axis', 'z') == 'x':
        R_hover = Rotation.from_quat([0.0, -np.sqrt(0.5), 0.0, np.sqrt(0.5)])
    else:
        R_hover = Rotation.from_quat([1, 0, 0, 0])
    q = (R_hover * Rotation.from_euler('y', theta)).as_quat()
    x = np.zeros(NX)
    x[3] = V
    x[6:10] = q
    x[13:17] = [np.clip(n_pos, cp['n_min'], cp['n_max'])]*2 + \
               [np.clip(n_neg, cp['n_min'], cp['n_max'])]*2
    return x


def available_acceleration(cp, V, direction, guess):
    """명목 제어기 모델로 본, 속도 V에서 ±x 방향으로 낼 수 있는 최대 가속도.

    논문 §5.5: "a_avail은 해당 속도와 방향에서 최대 회전수로 낼 수 있는
    가속도의 명목 모델 값". 준정상 문제로 푼다 —
      변수  θ(피치), n₊·n₋(로터 쌍, trim.py와 같은 짝짓기)
      제약  수직가속 0(고도 유지), 피치 각가속 0(모멘트 평형), 0≤n≤n_max
      목적  direction·v̇_x 최대화
    guess는 그 속도의 트림 [θ, n_eq, Δn]. 트림에서 출발해야 엉뚱한 가지
    (기수가 뒤를 보는 해)로 빠지지 않는다 — trim.py가 경고하는 그 함정이다.

    **역유입 금지 제약**(동체 축방향 유속 u_b ≥ 0)을 건다. 처음엔 없었는데,
    30 m/s 이하 제동에서 최적화기가 기수를 뒤로 81° 젖혀 최대 회전수로
    역추력을 내는 해(69 m/s²)를 찾았다. 그 자세에선 프로펠러에 역유입이
    생기는데 우리 모델은 V_axial=max(u_b,0)이라 역유입을 정지 추력으로
    취급한다 — 모델이 틀리는 영역의 값이었다. 팀 플랜트의 도메인 판정
    (propeller_curve.domain_status)도 역유입을 무효로 본다. 그래서 a_avail은
    '명목 모델이 유효한 영역'에서만 잰다.
    """
    from control.dynamics import AxialDronePlant
    plant = AxialDronePlant(cp, dt=0.001)
    direction = 1.0 if direction > 0 else -1.0

    def xdot(z):
        x = _our_state(cp, V, *z)
        return plant.evaluate_xdot(x, x[13:17])

    def axial_inflow(z):
        x = _our_state(cp, V, *z)
        R = Rotation.from_quat(x[6:10]).as_matrix()
        axis = 0 if cp.get('thrust_axis', 'z') == 'x' else 2
        sign = 1.0 if axis == 0 else -1.0
        return sign*float((R.T @ x[3:6])[axis])

    theta0, n_eq, dn = guess
    bounds = [(-np.pi, np.pi), (cp['n_min'], cp['n_max']), (cp['n_min'], cp['n_max'])]
    cons = [{'type': 'eq', 'fun': lambda z: xdot(z)[5]},
            {'type': 'eq', 'fun': lambda z: xdot(z)[11]},
            {'type': 'ineq', 'fun': axial_inflow}]
    best = None
    for dtheta in (0.0, 0.3, -0.3):
        z0 = [theta0 + dtheta, n_eq + dn, n_eq - dn]
        res = minimize(lambda z: -direction*xdot(z)[3], z0, method='SLSQP',
                       bounds=bounds, constraints=cons,
                       options={'ftol': 1e-12, 'maxiter': 500})
        xd = xdot(res.x)
        if abs(xd[5]) > 1e-6 or abs(xd[11]) > 1e-6 or axial_inflow(res.x) < -1e-9:
            continue                       # 제약 미충족 해는 버린다
        a = direction*xd[3]
        if best is None or a > best:
            best = a
    if best is None:
        raise ValueError(f'available acceleration not found at V={V} (direction {direction:+.0f})')
    return max(float(best), 0.0)


def acceleration_table(cp, speeds, direction):
    """속도 격자에서 a_avail(V). 트림은 1 m/s 연속법으로 따라간다.

    냉시동 트림은 이 기체의 제어기 모델에서 25 m/s 위로 못 간다(실측). 연속법으로
    0→85 m/s가 전부 풀리는 것을 확인했다 — 그래서 여기서도 연속법을 쓴다.
    """
    from control.trim import find_trim
    speeds = np.asarray(sorted(set(float(v) for v in speeds)))
    table, guess, v_prev = {}, None, 0.0
    for V in np.arange(0.0, speeds[-1] + 0.5, 1.0):
        tr = find_trim(cp, float(V), guess=guess, quiet=True)
        if not tr['converged']:
            raise ValueError(f'controller-model trim continuation broke at {V} m/s: {tr["why"]}')
        guess, v_prev = tr['guess'], V
        if np.any(np.isclose(speeds, V)):
            table[float(V)] = available_acceleration(cp, V, direction, tr['guess'])
    return np.array(sorted(table)), np.array([table[v] for v in sorted(table)])


def ramp_duration_for_rho(v0, v1, rho, a_avail, n_grid=4001):
    """ρ = max_t a_ref(t)/a_avail(v_ref(t)) 가 목표값이 되는 램프 시간 T_r.

    논문은 a_ref를 "식(32)로 연결한 속도 변화의 최대 가속도"로만 적었는데,
    속도가 0→85처럼 크게 변하면 a_avail이 램프 중에 계속 변해 '어느 속도의
    a_avail로 나누느냐'가 정해지지 않는다. 여기서는 **램프 경로 전체에서
    가장 빡빡한 지점의 비**로 정의했다(결정 필요 항목으로 보고서에 기록).
    a_ref(u) = |Δv|·S'(u)/T_r 이므로 T_r = max_u |Δv|·S'(u)/a_avail(v(u)) / ρ —
    반복 없이 닫힌 식으로 나온다.
    """
    u = np.linspace(0.0, 1.0, n_grid)
    dv = v1 - v0
    v = v0 + dv*SmoothstepProfile.shape(u)
    ratio = abs(dv)*smoothstep_rate(u)/np.maximum(a_avail(v), 1e-9)
    k = int(np.argmax(ratio))
    return float(ratio[k]/rho), dict(u_star=float(u[k]), v_star=float(v[k]),
                                     a_avail_at_star=float(a_avail(v[k:k+1])[0]))


# ══════════════════════════════════════════════════════════════════
# 3) 시나리오 → 스위트 입력
# ══════════════════════════════════════════════════════════════════

@dataclass
class Scenario:
    """스위트가 한 번 돌리는 단위. cases는 항상 1개(대표 사례)다."""
    id: str
    type: str
    profile: object
    cases: list
    window: tuple
    meta: dict = field(default_factory=dict)


def _round_up(value, step):
    return float(np.ceil(value/step - 1e-9)*step)


def build_scenarios(config, cp=None, native_params=None, only=None):
    """설정의 시나리오를 (profile, case) 목록으로 만든다.

    cp는 명목 제어기 모델 — 참조 프로필의 a_avail 계산에만 쓴다(명목 모델
    값이라는 논문 정의 그대로). native_params는 섭동 사례에서 플랜트 트림이
    존재하는지 미리 표시하는 데만 쓴다 — 이 정보는 제어기에 전달되지 않는다.
    """
    from models.team_light.control.trim import find_trim as plant_trim

    alt = float(config['altitude_m'])
    dt = float(config['plant']['dt_s'])
    out, accel_cache = [], {}
    for s in config['scenarios']:
        if only and s['id'] not in only:
            continue
        meta = dict(type=s['type'])
        factors = dict(s.get('perturbation', {}))
        case = dict(case_id=s['id'], factors=factors)
        if s['type'] == 'mission':
            V = resolve_speed(s['speed'], config)
            profile = MissionProfile(V, alt, tuple(float(d) for d in s['durations_s']))
            window = (float(s.get('evaluation_start_s', 0.0)), profile.T_total)
            meta.update(cruise_speed=V, phases=profile.get_phase_boundaries())
            if factors and native_params is not None:
                perturbed = perturb_params(native_params, factors)
                meta['plant_trim_at_cruise'] = bool(
                    plant_trim(perturbed, V, strict=False).get('valid', False))
        elif s['type'] == 'gust':
            V = resolve_speed(s['speed'], config)
            settle, pulse, recovery = (float(t) for t in s['times_s'])
            profile = GustProfile(V, alt, settle, pulse, recovery)
            peak = float(s['peak_m_s'])
            case.update(gust_direction=s['direction'], gust_peak=peak,
                        wind_reference='absolute',
                        flow_angle_deg=float(np.degrees(np.arctan2(peak, V))))
            window = (0.0, profile.T_total)
            meta.update(cruise_speed=V, peak_m_s=peak, direction=s['direction'],
                        flow_angle_deg=case['flow_angle_deg'])
        else:
            if cp is None:
                raise ValueError('reference profiles need the nominal controller model (cp)')
            v0, v1 = resolve_speed(s['from'], config), resolve_speed(s['to'], config)
            direction = 1.0 if v1 > v0 else -1.0
            lo, hi = min(v0, v1), max(v0, v1)
            key = (direction, lo, hi)
            if key not in accel_cache:
                # 기동이 지나가는 속도 구간만 잰다 — 구간 밖 값이 보간에 섞이지 않게.
                grid = np.unique(np.r_[np.arange(lo, hi, 5.0), hi])
                accel_cache[key] = acceleration_table(cp, grid, direction)
            speeds, accels = accel_cache[key]

            def a_avail(v, speeds=speeds, accels=accels):
                return np.interp(v, speeds, accels)

            T_raw, where = ramp_duration_for_rho(v0, v1, float(s['rho']), a_avail)
            # 스위트는 구간 길이가 플랜트 스텝의 배수여야 한다. 0.1 s 단위로
            # 올림해 ρ가 목표보다 커지지 않게 한다(실제 ρ는 meta에 남긴다).
            T_r = _round_up(T_raw, max(0.1, dt))
            profile = SmoothstepProfile(v0, v1, alt, float(s['lead_s']), T_r, float(s['tail_s']))
            window = (profile.ramp_start - 1.0, profile.ramp_end + 5.0)
            meta.update(v0=v0, v1=v1, rho_target=float(s['rho']),
                        rho_actual=float(s['rho'])*T_raw/T_r, ramp_s=T_r, **where,
                        a_avail_table=dict(speeds=speeds.tolist(), accel=accels.tolist(),
                                           direction=direction))
        out.append(Scenario(s['id'], s['type'], profile, [case], window, meta))
    return out
