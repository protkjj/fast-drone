"""경기장 공정성 불변식 — kj 작업지시서(2026-09-25) 작업 D.

성능이 아니라 **경기장이 기울지 않았는가**만 본다. 기울어짐은 양방향 모두
실격이다(하이브리드에 유리한 설정도, 기준선을 약하게 만드는 설정도).

  I-1  모든 제어기가 같은 플랜트 팩토리에서 같은 해시의 플랜트를 받는다
  I-2  모든 제어기가 같은 관측 정보(같은 키, 같은 참조 미리보기 길이)를 받는다
  I-3  NMPC 계열의 예측 단계·간격·반복 상한·허용 오차·웜스타트·소프트 제약·
       비용함수 사양이 모두 같다
  I-4  튜닝 기록의 평가 횟수가 같고, 튜닝 난수·본시험 난수가 겹치지 않는다
  I-5  NMPC 3종을 호버·V_L·V_H의 정확한 트림에서 풀면 트림 입력 근처의 해를
       내고, 수렴하며, NaN이 없다
  I-6  같은 설정으로 같은 사례를 두 번 돌리면 결과가 비트 단위로 같다
  I-7  시나리오 설정이 1절 확정 사항과 일치한다
  I-8  (kj 추가) 운용 범위에서 모든 제어기 경로의 추진 모델이 플랜트와 ±5%
  I-9  (추가) 모든 제어기가 플랜트의 정확한 트림에서 출발하면 뒤집히거나
       발산하지 않는다 — 자세 규약 불일치(추력축 180°) 검출용
  I-10 (kj 추가) 제어기 내부 모델의 가속도가 같은 상태·입력에서 플랜트와 일치한다
       (병진 ≤ 0.05 g, 트림 각가속도 ≤ 2 rad/s², 명목 트림 = 플랜트 트림)
  I-11 (kj 결정 2026-09-26) 적분기 기준. CPID 두 적분기는 가속도 권한 ±1 g·포화 시
       조건부이고, 출발 때 명목 트림값으로 채운다(NMPC 웜스타트와 같은 정보).
       GSLQR·CPID는 한계 도달·적분 정지 스텝을 같은 형식으로 기록한다

무거운 폐루프 사례는 ARENA_QUICK=1이면 건너뛴다(scripts/verify_arena.py --quick).
"""
from copy import deepcopy
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from control.arena import (load_config, check_confirmed_facts, build_scenarios, validate_config,
                           DEFAULT_CONFIG, _our_state)
from control.arena_factory import (ArenaFactory, ARENA_LABELS, NMPC_LABELS, PreviewViolation,
                                   ReferenceWindow, cpid_heading_for_plant)
from control.mission_profiles import GustProfile
from control.uncertainty import perturb_params
from control.validation_metrics import Acceptance
import control.validation_suite as suite
from models.team_light.control.baseline_v2 import baseline_params, parameter_hash
from models.team_light.control.trim import find_trim as plant_trim

QUICK = os.environ.get('ARENA_QUICK') == '1'
slow = pytest.mark.skipif(QUICK, reason='ARENA_QUICK=1: heavy closed-loop check skipped')

# 모든 제어기를 한 번씩 돌려 보기 위한 아주 짧은 사례(40 스텝, NMPC 솔브 4회).
SHORT = GustProfile(85.0, 20.0, 0.04, 0.02, 0.02)


def _case(name, **extra):
    return dict(case_id=name, factors={}, gust_direction='lateral', gust_peak=0.0, **extra)


@pytest.fixture(scope='module')
def config():
    return load_config(DEFAULT_CONFIG)


@pytest.fixture(scope='module')
def native():
    return baseline_params()


@pytest.fixture(scope='module')
def factory(config, native):
    return ArenaFactory(config, native)


@pytest.fixture(scope='module')
def captured_short_runs(factory):
    """5종을 SHORT로 한 번씩 돌리고 (행, 제어기 객체, 결과)를 모은다 — I-1·I-2·I-3 공용."""
    made = {}
    real_make = factory.make_for_profile

    def capture(label, profile, case=None):
        made[label] = real_make(label, profile, case)
        return made[label]

    factory.make_for_profile = capture
    try:
        runs = {}
        for label in ARENA_LABELS:
            row, result, _ = suite.run_trial(factory, label, SHORT, _case('short'), Acceptance())
            runs[label] = (row, made[label], result)
    finally:
        del factory.make_for_profile           # 인스턴스 속성을 지워 원래 메서드로
    return runs


# ── I-1 ─────────────────────────────────────────────────────────────

def test_i1_same_plant_factory_hash_and_initial_state(factory, native, monkeypatch):
    seen = []
    real = suite.AxialDronePlant

    class Spy(real):
        def __init__(self, params, dt=0.001):
            seen.append((parameter_hash(params), float(dt), real.__module__))
            super().__init__(params, dt=dt)

    monkeypatch.setattr(suite, 'AxialDronePlant', Spy)
    for factors in ({}, {'mass': 1.3}):
        seen.clear()
        case = dict(_case('i1'), factors=factors)
        rows, starts = {}, {}
        for label in ARENA_LABELS:
            row, result, _ = suite.run_trial(factory, label, SHORT, case, Acceptance())
            rows[label], starts[label] = row, result['xs'][0]
        expected = parameter_hash(perturb_params(native, factors))
        assert len(seen) == len(ARENA_LABELS)
        assert set(seen) == {(expected, factory.dt, 'models.team_light.control.dynamics')}
        assert {r['truth_parameter_sha256'] for r in rows.values()} == {expected}
        # 제어기는 플랜트 섭동을 모른다: 명목 제어기 모델 해시가 사례와 무관하게 같다.
        assert {r['controller_model_sha256'] for r in rows.values()} == {factory.controller_model_sha256}
        first = starts['V13']
        for label, x0 in starts.items():
            np.testing.assert_array_equal(x0, first, err_msg=f'{label} starts elsewhere')


def test_i1_controllers_never_hold_the_plant_parameter_dict(factory):
    assert factory.cp is not factory.p
    assert parameter_hash(factory.cp) == factory.controller_model_sha256 != parameter_hash(factory.p)


# ── I-2 ─────────────────────────────────────────────────────────────

def test_i2_same_observation_keys_and_preview_window(captured_short_runs, config):
    specs = {label: ctrl.observation_spec() for label, (_, ctrl, _) in captured_short_runs.items()}
    assert all(spec == specs['V13'] for spec in specs.values()), specs
    H = config['preview_horizon_s']
    for label, (_, ctrl, _) in captured_short_runs.items():
        assert type(ctrl.window) is ReferenceWindow
        assert ctrl.window.horizon_s == H
        assert ctrl.window.max_lookahead <= H + 1e-9, label
    # 같은 창을 받았고, 쓰는 폭은 구조가 정한다: NMPC는 예측 구간 전체, 나머지는 현재만.
    for label in NMPC_LABELS:
        assert captured_short_runs[label][1].window.max_lookahead == pytest.approx(H, abs=1e-9)
    for label in ('GSLQR', 'CPID'):
        assert captured_short_runs[label][1].window.max_lookahead == pytest.approx(0.0, abs=1e-12)


def test_i2_preview_beyond_horizon_is_refused():
    window = ReferenceWindow(SHORT, 1.0)
    window.set_time(0.02)
    window(1.02)                               # 정확히 H는 허용
    with pytest.raises(PreviewViolation):
        window(1.03)


def test_i2_preview_horizon_must_equal_nmpc_prediction_horizon(config):
    bad = deepcopy(config)
    bad['preview_horizon_s'] = 2.0
    with pytest.raises(ValueError, match='preview_horizon_s'):
        validate_config(bad)


# ── I-3 ─────────────────────────────────────────────────────────────

I3_KEYS = ('N', 'dt_pred_s', 'dt_ctrl_s', 'ipopt_options', 'cost_spec', 'cost_weights', 'Q_z',
           'preview', 'soft_constraints', 'warm_start')


def test_i3_nmpc_family_solver_settings_are_identical(captured_short_runs, config):
    settings = {label: captured_short_runs[label][1].settings for label in NMPC_LABELS}
    for key in I3_KEYS:
        values = {label: s[key] for label, s in settings.items()}
        assert len({repr(v) for v in values.values()}) == 1, f'{key} differs: {values}'
    nm = config['nmpc_common']
    opts = settings['V13']['ipopt_options']
    assert opts['ipopt.max_iter'] == nm['max_iter'] and opts['ipopt.tol'] == nm['tol']
    # 벽시계 제한이 있으면 같은 설정도 컴퓨터 부하에 따라 결과가 달라진다(kj).
    assert not any('time' in key for key in opts if key.startswith('ipopt.max'))


def test_i3_nmpc_thrust_bounds_are_symmetric(captured_short_runs):
    """V13·F13 NLP의 추력 상한은 둘 다 정지값(논문 식18의 T_max,0)이다.

    F13에 넣은 측정 γ(J)는 스핀축 반토크 환산만 바꾸고 상한은 건드리지 않는다
    (kj 확인 요청, 2026-09-25). 한쪽만 유속 의존 상한으로 바뀌면 대칭이 깨지므로
    여기서 묶어 둔다. V13 가상입력 T의 상한 = 4 × F13 로터별 상한.
    """
    v13 = captured_short_runs['V13'][1].nmpc
    f13 = captured_short_runs['F13'][1].nmpc
    stride = 13 + 4                            # 배치 [X_0, U_0, X_1, U_1, …]
    T_max_v13 = float(v13.ubw[13])             # 첫 입력 블록 U_0의 T 상한
    f_max_f13 = float(f13.ubw[13])             # 첫 입력 블록 U_0의 로터 1 상한
    assert T_max_v13 == pytest.approx(4*f13.f_max) and f_max_f13 == f13.f_max
    assert all(float(v13.ubw[k*stride + 13]) == T_max_v13 for k in range(v13.N))


@pytest.mark.parametrize('speed', [0.0, 20.0, 85.0])
@pytest.mark.parametrize('label', NMPC_LABELS)
def test_i5_nmpc_converges_near_trim_input_at_exact_trim(factory, config, label, speed):
    """I-5: 명목 제어기 모델의 정확한 트림(플랜트 규약)에서 연속 3회 솔브.

    허용오차 근거(2026-09-25 실측): 첫 스텝 입력의 트림 대비 편차는 최대 2.3%
    (F13, 20 m/s), |α| 최대 4.7e-3 rad/s², 반복 5회. 편차는 논문 식(17)의 입력
    편차 항이 호버 기준값 쪽으로 당기는 것이라 0이 아니다. 여유를 두어 ±5%,
    |α| ≤ 0.1, 반복 ≤ 15로 둔다. 곡선(PCHIP, C1) 모델이 IPOPT 수렴을 망치면
    여기서 걸린다(kj 확인 요청).
    """
    alt = config['altitude_m']
    profile = GustProfile(speed, alt, 1.0, 1.0, 1.0)
    ctrl = factory.make_for_profile(label, profile)
    tr = factory.model.trim(speed)
    x = tr['state'].copy()
    x[2] = alt
    u_trim = factory._trim_input(label, np.array([speed, 0.0, 0.0]))
    nmpc = ctrl.nmpc
    first = None
    for k in range(3):
        t = 0.02*k
        factory.update_at(ctrl, t, np.array([speed, 0.0, 0.0]), alt)
        ctrl(t, x)
        if first is None:
            first = np.array(nmpc._u_current, dtype=float).copy()
    log = ctrl.monitor.solve_log
    assert len(log) == 3
    assert all(e['accepted'] and e['finite'] and e['iter_count'] <= 15 for e in log), log
    if label == 'V13':
        assert first[0] == pytest.approx(u_trim[0], rel=0.05)
        assert np.abs(first[1:]).max() <= 0.1
    else:
        np.testing.assert_allclose(first, u_trim, rtol=0.05)


def test_i3_same_warm_start_rule(factory):
    """세 NMPC의 첫 웜스타트: X 블록 = 실측 초기상태, U 블록 = 명목 모델 트림 입력."""
    for label in NMPC_LABELS:
        ctrl = factory.make_for_profile(label, SHORT)
        tr = plant_trim(factory.p, 85.0)
        x = tr['state'].copy()
        x[2] = 20.0
        ctrl._warm_start(x)
        nmpc = ctrl.nmpc
        nx = 17 if label == 'M17' else 13
        stride = nx + 4
        u_trim = factory._trim_input(label, np.array([85.0, 0.0, 0.0]))
        for k in range(nmpc.N + 1):
            np.testing.assert_array_equal(nmpc.w0[k*stride:k*stride + nx], x[:nx])
        for k in range(nmpc.N):
            np.testing.assert_array_equal(nmpc.w0[k*stride + nx:k*stride + stride], u_trim)
        assert nmpc._cold_start is False


# ── I-6 ─────────────────────────────────────────────────────────────

@slow
@pytest.mark.parametrize('label', ARENA_LABELS)
def test_i6_same_case_twice_is_bit_identical(factory, label):
    profile = GustProfile(85.0, 20.0, 0.3, 0.2, 0.3)
    case = dict(_case('i6'), gust_direction='vertical', gust_peak=-5.0)
    first = suite.run_trial(factory, label, profile, case, Acceptance())[0]
    second = suite.run_trial(factory, label, profile, case, Acceptance())[0]
    assert first['trajectory_sha256'] == second['trajectory_sha256']


# ── I-7 ─────────────────────────────────────────────────────────────

def test_i7_config_matches_confirmed_facts(config, native):
    assert check_confirmed_facts(config, native) == []


@pytest.mark.parametrize('mutate, needle', [
    (lambda c: c['scenarios'][0].update(durations_s=[1.0, 8.0, 3.0, 25.0, 3.0]), 'phases'),
    (lambda c: c['scenarios'][0].update(takeoff=True), 'takeoff'),
    (lambda c: c['scenarios'][0].update(gust={'peak': 2}), 'disturbance'),
    (lambda c: c['controllers'].pop('CPID'), 'controllers'),
    (lambda c: c['speeds_m_s'].update(V_H=40.0), 'V_H'),
    (lambda c: c['scenarios'][1].pop('peak_m_s'), 'absolute'),
    (lambda c: c['vehicle'].update(mass_kg=1.2), 'mass'),
])
def test_i7_violations_are_detected(config, native, mutate, needle):
    bad = deepcopy(config)
    mutate(bad)
    violations = check_confirmed_facts(bad, native)
    assert any(needle in v for v in violations), violations


def test_i7_mission_starts_in_hover_at_altitude_without_takeoff(config):
    missions = build_scenarios(config, only=[s['id'] for s in config['scenarios']
                                             if s['type'] == 'mission'])
    for scenario in missions:
        phases = scenario.profile.phases
        assert [round(p[2], 9) for p in phases] == config['confirmed']['mission_phases_s']
        name, start, duration, v0, v1, z0, z1 = phases[0]
        assert (start, v0, v1) == (0.0, 0, 0) and z0 == z1 == config['altitude_m']
        assert scenario.window[0] == 3.0          # 첫 호버 3초는 평가 제외(논문 §5.9)


# ── I-8 ─────────────────────────────────────────────────────────────

def _plant_trim_points(config, native, step=5.0):
    V_L, V_H = config['speeds_m_s']['V_L'], config['speeds_m_s']['V_H']
    for V in np.arange(V_L, V_H + 1e-9, step):
        tr = plant_trim(native, float(V))
        v_body = Rotation.from_quat(tr['state'][6:10]).as_matrix().T @ tr['state'][3:6]
        yield float(V), tr, v_body


def test_i8_symbolic_model_matches_plant_wrench_over_operating_range(config, native, factory):
    """M17 예측·트림·GSLQR 선형화가 쓰는 심볼릭 로터 모델 — 추력·모멘트(반토크 포함)."""
    import casadi as ca
    from control.dynamics import _rotor_forces_moments
    from models.team_light.control.geometry import rotor_wrench
    vb, n, w = ca.SX.sym('vb', 3), ca.SX.sym('n', 4), ca.SX.sym('w', 3)
    F, M = _rotor_forces_moments(vb, n, w, factory.cp)[:2]
    fm = ca.Function('fm', [vb, n, w], [F, M])
    for V, tr, v_body in _plant_trim_points(config, native):
        Fs, Ms = (np.array(v).ravel() for v in fm(v_body, tr['control'], np.zeros(3)))
        W = rotor_wrench(native, tr['control'], v_body)
        assert Fs[0] == pytest.approx(W[0], rel=0.05), V
        assert np.abs(Ms - W[1:4]).max() <= 0.05*np.abs(W[1:4]).max() + 1e-9, V


def test_i8_indi_measurement_and_effectiveness_match_plant(config, native, factory):
    from control.hybrid_comparison import _curve_thrust, compute_control_effectiveness
    from models.team_light.control.geometry import rotor_thrusts, control_effectiveness
    for V, tr, v_body in _plant_trim_points(config, native):
        n = tr['control']
        T_plant = rotor_thrusts(native, n, v_body)
        np.testing.assert_allclose(_curve_thrust(factory.cp, n, max(v_body[0], 0.0)), T_plant, rtol=0.05)
        G = compute_control_effectiveness(factory.cp, n, v_body)
        G_plant = control_effectiveness(native, n, v_body)
        assert np.abs(G - G_plant).max() <= 0.05*np.abs(G_plant).max(), V


@pytest.mark.parametrize('speed', [0.0, 5.0, 10.0, 15.0, 20.0, 30.0, 50.0, 85.0])
def test_i8_cpid_requested_thrust_matches_plant_at_trim(native, factory, speed):
    """kj 지시: 트림 회전수에서 CPID가 요청한 추력과 플랜트 추력이 ±5% 안.

    CPID 할당에 플랜트 트림의 [T, M]을 넣으면, 곡선 역산으로 나온 명령 회전수에서
    플랜트가 내는 로터별 추력이 트림 추력과 같아야 한다(0~20 m/s가 CPID 설계 영역,
    30·50·85 m/s는 역산 자체의 정확도 확인).
    """
    from control.controller import CascadedPID
    from models.team_light.control.geometry import rotor_wrench, rotor_thrusts
    heading = cpid_heading_for_plant(factory.cp, factory.model.plant_hover_quat)
    cpid = CascadedPID(factory.cp, v_ref=[speed, 0, 0], z_ref=20.0, heading=heading)
    tr = plant_trim(native, speed)
    x = tr['state'].copy()
    v_body = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]
    W = rotor_wrench(native, tr['control'], v_body)
    n_cmd = cpid._allocate(W[0], W[1:4], x)
    np.testing.assert_allclose(rotor_thrusts(native, n_cmd, v_body),
                               rotor_thrusts(native, tr['control'], v_body), rtol=0.05)


def test_i8_static_inverse_would_break_the_cpid_check(native, factory):
    """판별력: 예전 정지 역산 n=sqrt(f/k_T)였다면 30 m/s에서 위 검사가 실패한다."""
    from models.team_light.control.geometry import rotor_thrusts
    tr = plant_trim(native, 30.0)
    v_body = Rotation.from_quat(tr['state'][6:10]).as_matrix().T @ tr['state'][3:6]
    T_trim = rotor_thrusts(native, tr['control'], v_body)
    n_static = np.sqrt(T_trim/factory.cp['k_T'])
    assert np.max(np.abs(rotor_thrusts(native, n_static, v_body)/T_trim - 1)) > 0.4


def test_i8_f13_reaction_torque_follows_curve(config, native, factory):
    """F13 예측모델의 반토크/추력 비가 상수 k_Q/k_T가 아니라 측정 운용점의 곡선 값."""
    from control.dynamics import reaction_torque_ratio, axial_airspeed
    from control.nmpc_f13 import RotorThrustNMPC13
    from models.team_light.control.geometry import rotor_thrusts, rotor_torques
    f13 = RotorThrustNMPC13(factory.cp, v_ref=[85.0, 0, 0], z_ref=20.0)
    assert f13._gamma_param
    for V, tr, v_body in _plant_trim_points(config, native, step=10.0):
        x = tr['state'].copy()
        x[2] = 20.0
        gamma = reaction_torque_ratio(factory.cp, x[13:17], axial_airspeed(factory.cp, x))
        plant_ratio = rotor_torques(native, x[13:17], v_body)/rotor_thrusts(native, x[13:17], v_body)
        np.testing.assert_allclose(gamma, plant_ratio, rtol=0.05)
    f13._last_t = -np.inf
    f13(0.0, x)                                  # __call__이 측정값으로 γ를 채운다
    np.testing.assert_allclose(f13._gamma, gamma, rtol=0, atol=0)


# ── I-9 ─────────────────────────────────────────────────────────────
# 플랜트의 정확한 트림에서 출발해 상수 참조로 2초. 목적은 성능 비교가 아니라
# '뒤집힘·발산 없음' 확인이다 — 자세 규약이 추력축 둘레로 180° 어긋나면 2초 안에
# 자세가 140~180° 돈다(음성 대조군으로 확인). 허용치(2026-09-25 실측 근거):
#   정상 제어기 최대 — 자세 변화 27°(CPID, 20 m/s: 공력 피드포워드 없음),
#   |ω| 5.7, |Δz| 0.69, |Δv| 3.6. 그래서 45°·10 rad/s·2 m·5 m/s로 둔다.
#   F13 어댑터 결함(공력 모멘트 누락)은 이 검사가 85 m/s에서 |ω| 21.8로 잡았다.
#   2026-09-26: 그 27°는 CPID 적분기 냉시동 과도였다. 사전 채움(I-11) 뒤 GSLQR·CPID
#   최대는 자세 0.37°, |ω| 0.042, |Δz| 0.034, |Δv| 0.025. 허용치는 판별에 충분해서
#   (음성 대조군 > 90°) 그대로 둔다.
I9_LIMITS = dict(attitude_deg=45.0, omega=10.0, dz=2.0, dv=5.0)


def _trim_hold(factory, label, speed, seconds=2.0):
    from control.mission_profiles import MissionProfile
    if speed > 0:
        profile = GustProfile(speed, 20.0, seconds/2, seconds/4, seconds/4)
    else:
        profile = MissionProfile(0.0, 20.0, (seconds - 0.8, 0.2, 0.2, 0.2, 0.2))
    row, result, _ = suite.run_trial(factory, label, profile, _case('i9'), Acceptance())
    xs = result['xs']
    R0 = Rotation.from_quat(xs[0, 6:10])
    attitude = max(np.degrees(np.linalg.norm((R0.inv()*Rotation.from_quat(q)).as_rotvec()))
                   for q in xs[::25, 6:10])
    return row, attitude


def _assert_holds(row, attitude, where):
    assert row['stop_reason'] is None, where
    assert attitude < I9_LIMITS['attitude_deg'], f'{where}: attitude moved {attitude:.1f} deg'
    assert row['max_omega'] < I9_LIMITS['omega'], where
    assert row['max_z_error'] < I9_LIMITS['dz'] and row['max_velocity_error'] < I9_LIMITS['dv'], where


CPID_OUT_OF_REGION = pytest.mark.xfail(
    strict=True,
    reason='CPID 설계 영역 밖(85 m/s): 기울기 한계 35°(트림 추력축은 수직에서 ~83°)라 '
           '적분기를 트림값으로 채워도(±1 g에서 잘림) 트림 자세를 못 만든다. 작업지시서가 '
           'CPID는 호버·저속만 요구한다. strict — 예상과 달리 통과하면 알린다.')


@pytest.mark.parametrize('label, speed', [('GSLQR', 0.0), ('GSLQR', 20.0), ('GSLQR', 85.0),
                                          ('CPID', 0.0), ('CPID', 20.0),
                                          pytest.param('CPID', 85.0, marks=CPID_OUT_OF_REGION)])
def test_i9_baselines_hold_the_plant_trim(factory, label, speed):
    """전 제어기 × 0·20·85 m/s(kj). CPID 85 m/s는 설계 영역 밖이라 실패가 예상이다."""
    _assert_holds(*_trim_hold(factory, label, speed), f'{label}@{speed}')


@slow
@pytest.mark.parametrize('speed', [0.0, 20.0, 85.0])
@pytest.mark.parametrize('label', NMPC_LABELS)
def test_i9_nmpc_family_holds_the_plant_trim(factory, label, speed):
    _assert_holds(*_trim_hold(factory, label, speed), f'{label}@{speed}')


def _default_convention_trims(cp, speeds):
    """'hover_quat'을 뺀(우리 기본 규약) 제어기 모델의 트림 — 1 m/s 연속법."""
    from control.trim import find_trim
    cp_default = {k: v for k, v in cp.items() if k != 'hover_quat'}
    out, guess = {}, None
    for V in np.arange(0.0, max(speeds) + 0.5, 1.0):
        tr = find_trim(cp_default, float(V), guess=guess, quiet=True)
        assert tr['converged'], V
        guess = tr['guess']
        out[float(V)] = tr
    return cp_default, [out[float(v)] for v in speeds]


def test_i9_negative_control_unconverted_trims_flip_gslqr(factory, config):
    """판별력: 플랜트 규약이 아닌(우리 기본 규약) 트림을 쓰면 GSLQR이 2초 안에 크게 돈다."""
    from control.arena_factory import ArenaController
    from control.controller import ScheduledLQR
    g = factory.gains['GSLQR']
    V_table = [float(v) for v in config['controllers']['GSLQR']['V_table_m_s']]
    _, raw = _default_convention_trims(factory.cp, V_table)
    bad = ScheduledLQR(factory.cp, v_ref=[0, 0, 0], z_ref=0.0, V_table=V_table,
                       Q=np.diag(g['Q_diag']), R=np.eye(4)*g['R_scale'],
                       integral_states=tuple(g['integral_states']), Q_integral=g['Q_integral'],
                       dt=factory.dt, trims=raw)

    def build_bad(window, v0, z0):
        ctrl = deepcopy(bad)
        ctrl.reset()
        ctrl.v_ref, ctrl.z_ref = v0.copy(), z0
        return ArenaController('GSLQR', ctrl, window)

    factory._build_gslqr = build_bad
    try:
        _, attitude = _trim_hold(factory, 'GSLQR', 0.0)
    finally:
        del factory._build_gslqr
    assert attitude > 90.0


def test_i9_cpid_attitude_target_matches_plant_hover(factory):
    """CPID 규약은 트림 유지로 안 드러난다(180°에서 자세오차 벡터가 0인 특이점).
    그래서 목표자세를 직접 대조한다: 호버 추력에서 R_des = 플랜트 호버 자세."""
    from control.controller import CascadedPID
    heading = cpid_heading_for_plant(factory.cp)
    cpid = CascadedPID(factory.cp, v_ref=[0, 0, 0], z_ref=20.0, heading=heading)
    _, R_des = cpid._force_to_attitude(np.array([0.0, 0.0, factory.cp['mass']*factory.cp['g']]))
    np.testing.assert_allclose(R_des, Rotation.from_quat(factory.model.plant_hover_quat).as_matrix(),
                               atol=1e-12)


def test_i9_hover_quat_trims_are_equilibria_in_the_plant_convention(factory, native):
    """hover_quat으로 직접 푼 트림이 (1) 우리 모델의 정확한 평형이고, (2) 기본 규약 트림을
    독립적으로 옮긴 것(추력축 180° + 로터 치환, convention_map)과 같고, (3) 플랜트 트림
    자세와 가깝다 — 규약 정렬을 두 경로로 교차검증한다."""
    from control.arena_factory import convention_map
    from control.dynamics import AxialDronePlant
    plant = AxialDronePlant(factory.cp)
    speeds = (0.0, 20.0, 85.0)
    cp_default, defaults = _default_convention_trims(factory.cp, speeds)
    R_rel, perm = convention_map(cp_default, factory.cp['hover_quat'])
    for V, other in zip(speeds, defaults):
        tr = factory.model.trim(V)
        xdot = plant.evaluate_xdot(tr['state'], tr['control'])
        assert np.linalg.norm(xdot[3:13]) < 1e-6, V
        moved = (Rotation.from_quat(other['state'][6:10])*R_rel)
        gap = np.degrees(np.linalg.norm((moved.inv()*Rotation.from_quat(tr['state'][6:10])).as_rotvec()))
        assert gap < 1e-4, f'V={V}: direct vs converted trim attitude differ by {gap:.2e} deg'
        np.testing.assert_allclose(tr['control'], other['control'][perm], rtol=1e-6)
        ref = plant_trim(native, V)['state']
        angle = np.degrees(np.linalg.norm(
            (Rotation.from_quat(ref[6:10]).inv()*Rotation.from_quat(tr['state'][6:10])).as_rotvec()))
        assert angle < 5.0, f'V={V}: model trim attitude {angle:.2f} deg away from plant trim'


# ── I-10 ────────────────────────────────────────────────────────────
# 제어기 내부 모델 대 플랜트 가속도(kj 추가, 2026-09-25). 같은 상태·같은 로터 입력에서
# 각 제어기의 예측/선형화 모델이 내는 가속도를 플랜트와 비교한다.
#   M17·GSLQR  17상태 모델(control.dynamics) — M17 예측, GSLQR 트림·선형화
#   F13        로터추력 입력 13상태 모델 — 입력은 플랜트의 실제 로터 추력, γ는 측정값
#   V13        가상입력 모델 — T = 플랜트 총추력(각가속도는 입력 ν라 비교 대상 아님)
# 관문(kj 결정): 병진 ≤ 0.05 g(트림+섭동), 트림 각가속도 ≤ 2 rad/s², 트림 일치.
# 트림 밖(자세 ±10°) 각가속도 불일치는 관문이 아니라 보고 대상이다(집중정수 한계).
I10_TRANS = 0.05*9.81
I10_ANG_TRIM = 2.0


def _model_accelerations(factory, x):
    import casadi as ca
    from control.dynamics import AxialDronePlant, reaction_torque_ratio, axial_airspeed
    from control.hybrid_comparison import build_virtual_dynamics
    from control.nmpc_f13 import build_f13_dynamics
    from models.team_light.control.dynamics import AxialDronePlant as TeamPlant
    from models.team_light.control.geometry import rotor_thrusts
    cache = factory.__dict__.setdefault('_i10_cache', {})
    if not cache:
        cache['ours'] = AxialDronePlant(factory.cp)
        cache['plant'] = TeamPlant(factory.p)
        cache['v13'] = build_virtual_dynamics(factory.cp)[0]
        cache['f13'] = build_f13_dynamics(factory.cp, torque_ratio=ca.SX.sym('g', 4))[0]
    u = x[13:17]
    plant = cache['plant'].evaluate_xdot(x, u)
    ours = cache['ours'].evaluate_xdot(x, u)
    v_body = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]
    T = rotor_thrusts(factory.p, u, v_body)
    x13 = np.concatenate([x[0:10], x[10:13]])
    gamma = reaction_torque_ratio(factory.cp, u, axial_airspeed(factory.cp, x))
    f13 = np.array(cache['f13'](x13, T, gamma)).ravel()
    v13 = np.array(cache['v13'](x13, np.r_[T.sum(), plant[10:13]])).ravel()
    return plant, {'M17/GSLQR': (ours[3:6], ours[10:13]), 'F13': (f13[3:6], f13[10:13]),
                   'V13': (v13[3:6], None)}


def _perturbed_states(native, speed):
    """트림 + 결정적 섭동 6개(난수 없음): 자세 ±5°, 각속도 0.5 rad/s, 로터 ±5%, 속도 ±5 m/s."""
    x0 = plant_trim(native, speed)['state'].copy()
    x0[2] = 20.0
    yield x0
    for axis, sign in (('y', 1), ('y', -1), ('z', 1)):
        x = x0.copy()
        x[6:10] = (Rotation.from_quat(x0[6:10])*Rotation.from_euler(axis, sign*5, degrees=True)).as_quat()
        yield x
    x = x0.copy(); x[10:13] = [0.5, -0.5, 0.5]; yield x
    x = x0.copy(); x[13:17] *= [1.05, 0.95, 1.05, 0.95]; yield x
    if speed > 0:
        x = x0.copy(); x[3:6] += [5.0, 0.0, -5.0]; yield x


@pytest.mark.parametrize('speed', [0.0, 20.0, 40.0, 60.0, 85.0])
def test_i10_translational_acceleration_matches_plant(factory, native, speed):
    for k, x in enumerate(_perturbed_states(native, speed)):
        plant, models = _model_accelerations(factory, x)
        for name, (a, _) in models.items():
            err = np.abs(a - plant[3:6]).max()
            assert err <= I10_TRANS, f'{name} @ {speed} m/s state {k}: |Δa| {err:.3f} m/s²'


@pytest.mark.parametrize('speed', list(np.arange(0.0, 85.1, 5.0)))
def test_i10_angular_acceleration_matches_plant_at_trim(factory, native, speed):
    x = next(_perturbed_states(native, float(speed)))
    plant, models = _model_accelerations(factory, x)
    for name, (_, w) in models.items():
        if w is not None:
            err = np.abs(w - plant[10:13]).max()
            assert err <= I10_ANG_TRIM, f'{name} @ {speed} m/s: |Δω̇| {err:.3f} rad/s²'


@pytest.mark.parametrize('speed', [0.0, 20.0, 40.0, 60.0, 85.0])
def test_i10_controller_model_trim_matches_plant_trim(factory, native, speed):
    """GSLQR이 스케줄하는 명목 트림이 플랜트 트림과 같은 자리에 있어야 한다.
    허용치 근거(실측): 로터 회전수 최대 0.5%, 자세 최대 0.64°(20 m/s) → 2%·2°."""
    tr = factory.model.trim(speed)
    ref = plant_trim(native, speed)
    np.testing.assert_allclose(tr['control'], ref['control'], rtol=0.02)
    gap = np.degrees(np.linalg.norm(
        (Rotation.from_quat(ref['state'][6:10]).inv()*Rotation.from_quat(tr['state'][6:10])).as_rotvec()))
    assert gap < 2.0, f'{speed} m/s: model trim attitude {gap:.2f} deg from the plant trim'


def test_i10_constant_pressure_centre_would_fail_the_trim_gate(factory, native):
    """판별력: 압력중심을 상수(논문 식9 그대로)로 두면 85 m/s 트림에서 관문을 크게 넘는다."""
    from control.dynamics import AxialDronePlant
    from models.team_light.control.dynamics import AxialDronePlant as TeamPlant
    constant = {k: v for k, v in factory.cp.items() if k != 'x_cp_poly'}
    x = next(_perturbed_states(native, 85.0))
    d = AxialDronePlant(constant).evaluate_xdot(x, x[13:17]) - \
        TeamPlant(factory.p).evaluate_xdot(x, x[13:17])
    assert np.abs(d[10:13]).max() > 5*I10_ANG_TRIM


# ── I-11 ────────────────────────────────────────────────────────────
# 적분기 기준(kj 결정 2026-09-26). 근거 실측(설계점검, PILOT):
#   - CPID 속도 적분을 상태 크기 5 m로 자르면 Ki 0.15에서 0.75 m/s²뿐이라 10·20 m/s
#     시행의 59~86% 동안 한계에 붙었다. 가속도 권한으로 바꾸면 한계에 한 번도 안 닿는다.
#   - CPID 고도 적분(기존 방식, 2.5 m/s²)은 20 m/s 트림 유지에 필요한 2.85 m/s²를 막아
#     60초 유지에서 고도 오차 +0.175 m를 남겼다.
#   - CPID만 트림 정보 없이(적분기 0) 출발해서 20 m/s에서 채워지는 데 ~30초가 걸렸다.
#     명목 트림값으로 채우자 설계점검 3/6 → 6/6 정착.
#   - GSLQR LQI 한계(상태 크기 ±5)의 가속도 환산 권한은 x 10.6~20.9, z 13.6~70.6 m/s²이고
#     설계점검 10/10에서 한 번도 안 닿았다(한계 ∞와 비트 동일). 그대로 두고 계측만 한다.

def test_i11_arena_cpid_uses_acceleration_authority_and_preload(factory, config):
    ctrl = factory.make_for_profile('CPID', GustProfile(20.0, 20.0, 1.0, 1.0, 1.0))
    pid = ctrl.inner
    assert pid.a_int_max == pytest.approx(9.81) and pid.a_int_z_max == pytest.approx(9.81)
    assert config['controllers']['CPID']['integrator_preload'] == 'controller_model_trim'
    assert ctrl.settings['integrator_preload'] == 'controller_model_trim'
    # 권한은 Ki와 무관하다: Ki를 바꿔도 계측이 보고하는 한계(m/s²)는 그대로다
    limits = {name: lim for name, (_, lim) in pid.integrator_status()['channels'].items()}
    pid.Ki_vel, pid.Ki_z = 4*pid.Ki_vel, 0.25*pid.Ki_z
    assert limits == {name: lim for name, (_, lim) in pid.integrator_status()['channels'].items()}


@pytest.mark.parametrize('speed', [0.0, 10.0, 20.0])
def test_i11_cpid_preload_reproduces_the_trim_thrust_vector(factory, speed):
    """출발값을 채운 CPID는 명목 트림 상태에서 트림과 같은 추력 벡터를 요구하고,
    자세 목표도 트림 자세와 같다(적분 채널이 트림의 공력 몫을 이미 내고 있다)."""
    ctrl = factory.make_for_profile('CPID', GustProfile(speed, 20.0, 1.0, 1.0, 1.0))
    tr = factory.model.trim(speed)
    x = tr['state'].copy()
    x[2] = 20.0
    ctrl._warm_start(x)
    pid = ctrl.inner
    a_des = np.concatenate([-pid.Ki_vel*pid._int_ev, [-pid.Ki_z*pid._int_ez]])
    np.testing.assert_allclose(a_des, factory.model.trim_acceleration(speed), atol=1e-12)
    T, R_des = pid._force_to_attitude(pid.m*(a_des + np.array([0.0, 0.0, pid.g])))
    assert T == pytest.approx(float(np.sum(factory.model.rotor_thrusts(tr['control'], x))), rel=1e-9)
    gap = np.degrees(np.linalg.norm(
        (Rotation.from_matrix(R_des).inv()*Rotation.from_quat(x[6:10])).as_rotvec()))
    assert gap < 0.1, f'{speed} m/s: CPID target attitude {gap:.3f} deg from the trim attitude'


def test_i11_cpid_integrators_freeze_while_saturated(factory):
    """포화(여기서는 기울기 한계) 스텝에서 경기장 방식은 두 적분기를 모두 멈춘다.
    기존 방식(a_int_z_max=None)의 고도 적분은 포화와 상관없이 쌓인다(비트 동일 기본값)."""
    from control.controller import CascadedPID
    x = factory.model.trim(0.0)['state'].copy()
    x[2] = 19.0                                       # 고도 오차 −1 m
    heading = cpid_heading_for_plant(factory.cp)
    for a_int_z_max, z_frozen in ((9.81, True), (None, False)):
        pid = CascadedPID(factory.cp, v_ref=[30.0, 0, 0], z_ref=20.0, heading=heading, dt=0.002)
        pid.Ki_vel, pid.a_int_z_max = 0.15, a_int_z_max
        before_v, before_z = pid._int_ev.copy(), pid._int_ez
        pid(0.0, x)
        assert pid._saturated                         # 수평 30 m/s² 요구 → 기울기 35° 한계
        np.testing.assert_array_equal(pid._int_ev, before_v)
        assert (pid._int_ez == before_z) == z_frozen
        assert pid.integrator_status()['frozen']


def test_i11_integrator_log_has_the_same_format_for_gslqr_and_cpid(captured_short_runs):
    reports = {label: run[0].get('integrators') for label, run in captured_short_runs.items()}
    for label in NMPC_LABELS:
        assert reports[label] is None                 # NMPC 계열은 적분기가 없다
    keys = {'steps', 'at_limit_steps', 'at_limit_fraction', 'frozen_steps', 'frozen_fraction',
            'channels'}
    assert set(reports['GSLQR']) == set(reports['CPID']) == keys
    assert set(reports['GSLQR']['channels']) == {'z', 'vx'}
    assert set(reports['CPID']['channels']) == {'vx', 'vy', 'z'}
    for label in ('GSLQR', 'CPID'):
        assert reports[label]['steps'] == len(captured_short_runs[label][2]['us'])


def test_i11_one_percent_rule_flags_only_long_limit_contact(config):
    from control.arena import integrator_limit_flags

    def entry(fraction):
        return dict(controller='CPID', scenario_id='s', integrators=dict(
            steps=1000, at_limit_steps=int(1000*fraction), at_limit_fraction=fraction,
            frozen_steps=0, frozen_fraction=0.0,
            channels={'z': dict(limit=9.81, at_limit_steps=int(1000*fraction), peak_fraction=1.0)}))
    threshold, flags = integrator_limit_flags(
        [entry(0.0), entry(0.01), entry(0.011), dict(controller='V13', integrators=None)], config)
    assert threshold == 0.01
    assert [f['at_limit_fraction'] for f in flags] == [0.011]


# ── I-4 ─────────────────────────────────────────────────────────────

def test_i4_compass_search_spends_exactly_the_budget():
    from control.arena_tune import compass_search
    for budget in (1, 7, 24, 60):
        calls = []

        def objective(e):
            calls.append(e)
            return float(np.sum((np.asarray(e) - 0.3)**2))

        _, _, info = compass_search(3, budget, objective)
        assert info['spent'] == budget == len(calls)


def test_i4_compass_search_is_deterministic():
    from control.arena_tune import compass_search

    def objective(e):
        return float(np.sum((np.asarray(e) - np.array([0.5, -0.25]))**2))

    assert compass_search(2, 30, objective)[:2] == compass_search(2, 30, objective)[:2]


def _record(controller, **extra):
    base = dict(controller=controller, budget=24, spent=24, status='complete',
                scenario_ids=['tune_a', 'tune_b'], objective={'metric': 'x'}, search={'m': 1},
                seeds=dict(tuning_range=[2000, 2999], main_range=[1000, 1999], used=[]))
    base.update(extra)
    return base


def test_i4_checker_accepts_equal_records(config):
    from control.arena_tune import check_tuning_records
    assert check_tuning_records([_record(label) for label in ARENA_LABELS], config) == []


@pytest.mark.parametrize('broken, needle', [
    (dict(budget=120, spent=120), 'budgets differ'),
    (dict(spent=20), 'spent'),
    (dict(scenario_ids=['mission_VH']), 'main-test'),
    (dict(seeds=dict(tuning_range=[1500, 2500], main_range=[1000, 1999], used=[])), 'overlaps'),
    (dict(seeds=dict(tuning_range=[2000, 2999], main_range=[1000, 1999], used=[1001])), 'outside'),
    (dict(objective={'metric': 'y'}), 'objective'),
])
def test_i4_checker_flags_unfair_records(config, broken, needle):
    from control.arena_tune import check_tuning_records
    records = [_record(label) for label in ARENA_LABELS]
    records[0].update(broken)
    violations = check_tuning_records(records, config)
    assert any(needle in v for v in violations), violations


def test_i4_committed_pilot_tuning_records_are_fair(config):
    """저장소에 커밋된 튜닝 기록(PILOT)이 있으면 그것도 같은 검사를 통과해야 한다."""
    import json
    from control.arena_tune import check_tuning_records
    from control.arena import ROOT
    runs = sorted((ROOT/'results'/'arena'/'tuning').glob('*/'))
    if not runs:
        pytest.skip('no committed tuning run yet')
    for run in runs:
        records = [json.loads(p.read_text(encoding='utf-8')) for p in sorted(run.glob('*.record.json'))]
        if all(r.get('status') == 'complete' for r in records) and records:
            assert check_tuning_records(records, config) == [], run


def test_i8_our_state_matches_trim_solver_state(factory):
    """a_avail 계산(control/arena.py)이 트림 솔버와 같은 상태 구성을 쓰는지."""
    from control.trim import find_trim
    tr = find_trim(factory.cp, 0.0, quiet=True)
    theta, n_eq, dn = tr['guess']
    np.testing.assert_array_equal(_our_state(factory.cp, 0.0, theta, n_eq + dn, n_eq - dn),
                                  tr['state'])
