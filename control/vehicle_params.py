"""
축대칭 미사일형 ISR 드론 + 쿼드콥터 추진 — 기체 물리 파라미터
==============================================================

[플레이스홀더] 형상팀 확정 전까지 임시 값.
300 km/h 트림 가능하도록 물리적으로 정당한 범위 내에서 조정됨.

형상: 미사일형 유선형 동체 + 4로터 (표준 쿼드콥터 배치)

조정 이력 (v2, 물리적 근거 포함):
  C_A0:  0.50 → 0.12  유선형 오자이브 동체는 C_D0 ≈ 0.08~0.15 (참고: MK82 폭탄 0.12)
                        기존 0.50은 "둔탁한 원통"에 가까웠음. 유선형이 우리 컨셉.
  C_Aa2: 2.0  → 1.0   유선형 동체는 유도항력 증가가 완만. 기존값은 고받음각에서 과도.
  x_cp:  -0.15→-0.10   적당한 정적 안정성 유지하면서 로터 차동 부담 완화.
                        CP가 CG에서 멀수록 모멘트↑ → 차동 포화↑.
  n_max: 1500 → 1800   팁 마하수 제한 확인:
                        팁속도 = n_max × (D/2) = 1800 × 0.15 = 270 m/s
                        마하 0.79 < 0.8 한계 (340 m/s × 0.8 = 272 m/s) ✓
  k_T:   5e-5 → 6e-5   고피치 프로펠러로 소폭 효율 증가.
                        정적 T/W = 4·6e-5·1800²/78.5 = 9.9 (높지만 소형 레이싱급과 유사)
"""
import json
from pathlib import Path

import numpy as np

_arm = 0.25
_s = _arm / np.sqrt(2)

vehicle_params = {

    # ── 질량 특성 (축대칭: Iyy = Izz) ──
    'mass': 8.0,
    'Ixx':  0.02,
    'Iyy':  0.70,
    'Izz':  0.70,

    # ── 기체 형상 ──
    'body_length':   1.0,
    'body_diameter': 0.15,
    'S_ref': np.pi * 0.15**2 / 4,
    'd_ref': 0.15,

    # ── 대기 ──
    'rho': 1.225,
    'g':   9.81,

    # ── 동체 공력 ──
    # 핵심 변경: C_A0 대폭 감소 (유선형 미사일 컨셉에 부합)
    # 진단 결과: 기존 C_A0=0.5 → 60 m/s에서 26° 틸트 필요 → 동체 반양력 102N
    #           → 로터가 무게(78N) + 반양력(102N) = 181N 감당 → 포화!
    # C_A0=0.12면: 83 m/s에서 ~5° 틸트, 반양력 ~15N, 로터 부담 정상화
    'C_Na':  6.0,          # [/rad] 수직력 기울기 (유지: 동체+팔 합산)
    'C_dc':  1.2,          # [-]    교차류 항력 (유지: 원통 표준)
    'C_A0':  0.12,         # [-]    영받음각 축력 ← 유선형 오자이브 동체
    'C_Aa2': 1.0,          # [-]    유도 축력 ← 유선형은 완만
    'x_cp': -0.10,         # [m]    압력 중심 ← 로터 차동 부담 완화

    # ── 감쇠 ──
    'C_mq': -10.0,
    'C_lp':  -5.0,

    # ── 로터 배치 (4개, × 패턴) ──
    'num_rotors': 4,
    'arm_length': _arm,
    'rotor_positions': np.array([
        [ _s,  _s, 0],     # r1: 전방우측 (CW)
        [ _s, -_s, 0],     # r2: 전방좌측 (CCW)
        [-_s, -_s, 0],     # r3: 후방좌측 (CW)
        [-_s,  _s, 0],     # r4: 후방우측 (CCW)
    ]),
    'rotor_directions': np.array([1, -1, 1, -1]),

    # ── 로터/프로펠러 ──
    # n_hov = √(mg/(4·k_T)) = √(78.5/2.4e-4) ≈ 572 rad/s
    # 정적 T/W = 4·6e-5·1800²/78.5 = 9.9
    'D_prop':    0.30,
    'k_T':       6.0e-5,       # [N/(rad/s)²] ← 고피치 프로펠러
    'k_Q':       5.0e-6,       # [N·m/(rad/s)²]
    'J_max':     2.0,
    'I_rotor':   0.0005,

    # ── 모터 ──
    'tau_m': 0.02,
    'n_min': 0.0,
    'n_max': 1800.0,           # [rad/s] ← 팁 마하 0.79 (한계 0.8 이내)

    # ── 배터리·전기 (논문 식11-14, M17 전류·전압 제약용) ──
    # [플레이스홀더] 실측 전 임시값 — 12S(44.4V급) LiPo, Kv≈400rpm/V 가정.
    # control/battery.py의 BatteryModel과 NMPCController(electrical_constraints=True)
    # 둘 다 이 키들을 참조한다. rocket_params는 이 dict를 복사해 쓰므로 자동 상속.
    'V_oc':     44.4,      # [V]   개방전압(공칭, SOC 의존성은 단순화해 상수로 둠)
    'R_b':      0.01,      # [Ω]   배터리 내부저항
    'C_b':      16.0,      # [Ah]  배터리 용량
    'k_e':      0.0239,    # [V/(rad/s)]  역기전력 상수 (Kv 400rpm/V 환산)
    'k_t':      0.0239,    # [N·m/A]      토크 상수 (이상적 모터: k_t=k_e, SI)
    'R_m':      0.05,      # [Ω]   모터 권선 저항
    'I_lim':    40.0,      # [A]   모터 1개당 전류 한계
    'eta_esc':  0.95,      # [-]   ESC 변환 효율
}


# ══════════════════════════════════════════════════════════════════════
# 로켓형 배치 (추진까지 축대칭)
# ══════════════════════════════════════════════════════════════════════
#
# 위 vehicle_params 는 어뢰형 동체를 **수평**으로 달고 일반 쿼드처럼 숙여서 나는
# 배치다 (thrust_axis 'z'). 아래는 우리가 만들려는 기체다:
#
#   · 로터 넷이 **동체축에 수직인 평면**에 놓여 기수축을 둘러싼다
#   · 추력이 기수 방향 -> 호버에서 기수가 위를 본다
#   · 기수축을 따라 내려다보면 동체 단면이 원으로 보이고 그 둘레에 로터가 있다
#
# 동체 공력·질량·로터 계수는 그대로다. 바뀌는 것은 추력축과 로터 기하뿐이다.
# ⚠️ **이것은 실제 확정 기체가 아니다.** control 브랜치 안에서 제어 알고리즘을
#    비교하기 위한 **탐색용 프로파일**이다 (질량 8 kg, 2026-09-15 결정).
#    확정 설계는 아래 `selected_params` (선정안 6931, 1.7117 kg) 다.
#    두 프로파일은 목적이 다르다 — 이걸 헷갈려서 문서에서 8 kg 과 1.71 kg 이
#    섞여 돌아다닌 전력이 있다.
rocket_params = dict(vehicle_params)
rocket_params.update({
    'thrust_axis': 'x',

    # 기수축(x) 둘레 반지름 arm_length 에 45도 간격. x=0 이라 무게중심 평면에 놓인다.
    'rotor_positions': np.array([
        [0.0,  _s,  _s],     # r1 (CW)
        [0.0, -_s,  _s],     # r2 (CCW)
        [0.0, -_s, -_s],     # r3 (CW)
        [0.0,  _s, -_s],     # r4 (CCW)
    ]),
    'rotor_directions': np.array([1, -1, 1, -1]),

    # ★ 동체 지름 150 -> 90 mm (kj 결정, 2026-09-15).
    #
    #   150 mm 로는 **52.0 m/s(187 km/h) 위에 정상비행 트림이 아예 없다.**
    #   막히는 곳이 회전수 천장이 아니라 후방 로터가 음추력을 요구하는
    #   지점이다. 목표 83.3 m/s 가 그 공백 한가운데에 있었다.
    #
    #   상한은 V* ∝ sqrt(m / (rho · S_ref · C_Na)) 이라 기준면적으로 연다.
    #   연속법으로 재본 실측 (2 m/s 간격, 3 식 트림):
    #     150 mm  S_ref 0.017671  ->  52.0 m/s  187 km/h
    #     120     0.011310        ->  64.0      230
    #     100     0.007854        ->  76.0      274
    #      90     0.006362        ->  84.0      302   <- 목표 통과
    #      80     0.005027        ->  90.0      324
    #
    #   ⚠ 공력 기준면적만 바꿨다. 질량 8 kg·관성·로터는 그대로 두었다.
    #     실제로는 가는 동체가 더 가벼우므로 이 조합은 **보수적**이다.
    #     탑재 부피는 형상팀 몫이다.
    'body_diameter': 0.090,
    'S_ref': np.pi * 0.090 ** 2 / 4,
    'd_ref': 0.090,
})


# ══════════════════════════════════════════════════════════════════════
# 확정 설계 — 선정안 6931 (research/profiles/selected.json 파생)
# ══════════════════════════════════════════════════════════════════════
#
# ⚠️ **이 프로파일은 20 m/s 위에서 트림이 존재하지 않는 것이 정상 동작이다.**
#    find_trim 이 고속에서 실패하면 버그가 아니라 물리적 한계다
#    (research/TRIM_CAUSE_AND_LEVERS.md, 2026-09-22 규명:
#     요구 피치모멘트 |cp|·W·cos θ 가 가용 a·T 를 넘어 한쪽 로터쌍이 음추력을
#     요구한다). **계수를 끼워맞춰 고속 트림을 만들어내지 말 것.**
#
# ⚠️ **research 와 정량적으로 일치시키는 것이 목적이 아니다.** 이 프로파일의
#    용도는 "저속에는 트림이 있고 고속에는 없다"는 **정성적 사실**을 이
#    코드베이스(control/)에서 확인하는 것이다. 아래 C_A0 근사 때문에 트림
#    경계가 research 의 19.63 m/s 와 정확히 같지 않을 수 있다 — 18 이 나오든
#    21 이 나오든 그 자체는 버그가 아니다.
#
# ⚠️ **ScheduledLQR 과 쓸 때 기본 V_table(0~80, step 10)은 부적합하다.**
#    실제 트림 구간(0~19.63 m/s)에 맞는 조밀한 격자를 명시적으로 줘야 한다
#    (예: V_table=[0, 4, 8, 12, 16, 19]). 기본 격자를 그대로 쓰면 20 이상이
#    전부 제외돼 격자가 0·10 두 점만 남고, 그 위는 **외삽**이라 V=15 에서
#    직접 설계 대비 85.6% 오차가 난다(실측). 격자를 맞추면 7.7% 로 떨어지는데
#    그건 오염이 아니라 격자 간격에서 오는 정상적인 보간 오차다.
#
# 복사가 아니라 **파생**이다. 숫자를 손으로 옮겨 적으면 원본이 바뀔 때 조용히
# 어긋난다 (rocket_params 가 8 kg 으로 굳은 것이 바로 그 사례다). research/ 가
# 없으면 None 을 돌려주고 control/ 의 나머지는 그대로 동작한다.

_SELECTED_JSON = Path(__file__).resolve().parent.parent / 'research' / 'profiles' / 'selected.json'


def _require(d, keys, ctx):
    """빠진 키는 조용히 옛 값으로 넘어가지 말고 여기서 바로 터뜨린다.

    2026-09-25 검토에서 드러난 문제: 예전 로더는 `params = dict(vehicle_params)`
    로 옛 8kg/30cm-프로펠러 기체를 베이스로 깔고 일부 키만 덮어썼다. J_max 가
    그 덮어쓰기 목록에서 빠져 있었고, 아무 에러도 없이 옛 기체의 값(2.0)이
    그대로 쓰였다 — 85 m/s 에서 추력 여유를 37% 과소평가하는 결과로 이어졌다.
    이 함수는 그 재발을 막는다: 소스 JSON에 필요한 필드가 없으면 여기서 바로
    ValueError 를 낸다. 프로파일이 바뀌면 로더도 같이 바뀌어야 한다는 뜻이다.
    """
    missing = [k for k in keys if k not in d]
    if missing:
        raise ValueError(
            f"선정 프로파일의 '{ctx}'에 필요한 키가 없다: {missing}. "
            f"프로파일 스키마가 바뀐 것으로 보인다 — 로더를 함께 갱신할 것.")


def _fit_prop_decay_rate(J_grid, CT_grid, j_operating_max=1.2):
    """control 의 선형 감쇠 fac(J)=max(1-J/J_max,0) 을 최소제곱으로 맞춘다.

    실제 CT(J)/CT(0) 곡선은 한동안 평평하다가 급히 떨어지는 모양이다(선정
    프로펠러: J≈1.0 까지 0.7 이상 유지되다 J≈1.54 에서 0을 지나 음수로).
    직선 하나로는 어차피 못 맞추는데, **어디를 맞출지**가 문제다.

    CT(J)=0 이 되는 지점(옛 방식이 하려던 것)을 J_max 로 쓰면, 곡선이 평평한
    저-중 전진비 구간(실제 기체가 대부분의 시간을 보내는 곳)에서 크게
    어긋난다 — 검토자 실측: 85 m/s(J≈1.12)에서 추력을 37% 과소평가했다.

    대신 **실제 운용 전진비 범위**([0, j_operating_max])에서 최소제곱으로
    맞춘다. fac_model(J)=1-b·J 는 b 에 대해 선형이므로 닫힌 형태로 풀린다:
        b* = Σ Jᵢ·(1-factᵢ) / Σ Jᵢ²,   J_max = 1/b*
    이 범위 밖(예: 극단적 저속 전진 + 초고회전)에서는 이 근사가 더 나빠질 수
    있다 — 표 기반 플랜트(dynamics_hifi.py)가 그 구간까지 정확히 다룬다.
    """
    J = np.asarray(J_grid, dtype=float)
    CT = np.asarray(CT_grid, dtype=float)
    mask = J <= j_operating_max
    J_fit = J[mask]
    fac_real = CT[mask] / CT[0]
    b = float(np.sum(J_fit * (1.0 - fac_real)) / np.sum(J_fit**2))
    if b <= 0:
        raise ValueError(f"J_max 최소제곱 적합 실패 (b={b}) — CT(J) 표가 "
                         f"[0,{j_operating_max}] 구간에서 증가 형태다. 표를 확인할 것.")
    return 1.0 / b


def load_selected_params(path=_SELECTED_JSON):
    """research 선정 프로파일 -> control 파라미터 dict. 없으면 None.

    2026-09-25 재작성: 예전엔 `dict(vehicle_params)` 로 옛 8kg 기체를 베이스로
    깔고 일부 키만 덮어썼다 — 빠뜨린 키(J_max 등)가 조용히 옛 값으로 새는
    구조였다. 지금은 **모든 물리 파라미터를 이 함수 안에서 소스로부터 직접
    계산**한다. 유일한 예외는 물리량이 아니라 **설계 선택**인 두 개
    (`num_rotors`=4, CLAUDE.md 확정사항; `n_min`=0.0, 모든 로터의 보편적 하한)
    뿐이고, 그 둘도 상속이 아니라 이 함수 안에 명시돼 있다.
    """
    try:
        source = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None

    _require(source, ('mass_kg', 'g', 'body_length_m', 'body_diameter_m',
                      'area_m2', 'rho', 'sound_speed_mps', 'cp_from_cg_m',
                      'arm_m', 'inertia_kg_m2', 'prop', 'aero', 'motor',
                      'battery', 'id'), '최상위')
    prop, aero, motor, battery = (source['prop'], source['aero'],
                                  source['motor'], source['battery'])
    _require(prop, ('diameter_m', 'J', 'CT', 'CP', 'tip_mach_limit'), 'prop')
    _require(aero, ('speed_mps', 'CD0', 'CN_alpha', 'CN_cross', 'damping'), 'aero')
    _require(motor, ('kv_rpm_V', 'resistance_ohm', 'rotor_inertia_kg_m2',
                     'speed_loop_tau_s', 'current_limit_A'), 'motor')
    _require(battery, ('series', 'resistance_ohm', 'capacity_Ah',
                       'esc_efficiency'), 'battery')

    rho, diameter = source['rho'], prop['diameter_m']
    J_grid = np.asarray(prop['J'], dtype=float)
    CT_grid = np.asarray(prop['CT'], dtype=float)
    CP_grid = np.asarray(prop['CP'], dtype=float)

    # 추진 계수 환산. research 는 전진비 의존 맵(CT(J), CP(J))을 쓰고 control 은
    # 상수 k 하나에 전진비 선형 감쇠(J_max)를 쓴다. 정지(J=0) 값으로 환산한다:
    #   research  T = CT·rho·rev²·D⁴,  rev = n/(2π)   ->  k_T = CT·rho·D⁴/(4π²)
    #             Q = CP·rho·rev²·D⁵/(2π)             ->  k_Q = CP·rho·D⁵/(8π³)
    # 검산: 이렇게 얻은 k_T 로 계산한 호버 회전수가 원본의 호버 앵커 rpm 을
    #       1.5e-5 rpm 오차로 재현한다(단위 규약이 일관된다는 뜻). 이 정적
    #       (J=0) 값은 그대로 둔다 — 아래 J_max 재적합과는 독립이다.
    k_T = CT_grid[0]*rho*diameter**4/(4*np.pi**2)
    k_Q = CP_grid[0]*rho*diameter**5/(8*np.pi**3)
    J_max = _fit_prop_decay_rate(J_grid, CT_grid)

    # 축력 계수. research 의 CD0 는 **속도 테이블**이고 control 은 상수 하나다.
    # 이 프로파일이 실제로 검증받는 구간은 0~19.6 m/s 뿐인데 하필 거기서 CD0 가
    # 가장 빠르게 변한다(0.887 -> 0.568). 전 구간 평균(0.523)을 쓰면 정작 쓰는
    # 구간에서 틀린 값이 되므로 **0~20 m/s 구간 평균만** 쓴다. ⚠ 85 m/s 실제값
    # (≈0.484)의 1.39배라 고속 시뮬레이션에는 이 상수 대신 표 기반 플랜트
    # (dynamics_hifi.py, drag_table 사용)를 쓸 것 — J_max 와 달리 이 근사는
    # 그대로 둔다(검토자 지적은 J_max 재적합만 요청했다).
    speeds = np.asarray(aero['speed_mps'], float)
    drag_table = np.asarray(aero['CD0'], float)
    low_speed = speeds <= 20.0
    C_A0 = float(drag_table[low_speed].mean())

    # 회전수 상한 — research/model.py rpm_limit 과 같은 식(팁마하 vs 무부하 중 작은 값).
    tip = 2*prop['tip_mach_limit']*source['sound_speed_mps']/diameter
    no_load = motor['kv_rpm_V']*battery['series']*4.2*2*np.pi/60

    arm = source['arm_m']
    half = arm/np.sqrt(2)
    inertia = source['inertia_kg_m2']
    damping = aero['damping']          # [x, y, z] = [롤, 피치, 요]

    # 모터/배터리 상수 — 예전엔 이 여섯 개(k_e,k_t,R_m,I_lim,eta_esc,V_oc,R_b,C_b)
    # 전부 옛 8kg 기체(12S, Kv≈400rpm/V 가정) 값을 그대로 상속했다. 논문
    # v5.3 은 전류·전압·배터리를 모델에서 제외하므로(electrical_constraints
    # 기본 False) 헤드라인 결과엔 영향이 없었지만, 소스에 데이터가 있으니
    # 바르게 채운다 — 실기(PX4) 쪽 자산이나 나중에 이 경로를 켤 때를 위해.
    kt_motor = 60.0/(2*np.pi*motor['kv_rpm_V'])   # 이상적 모터: k_t = k_e (SI)

    params = {
        'thrust_axis': 'x',
        'mass': source['mass_kg'],
        'Ixx': inertia[0], 'Iyy': inertia[1], 'Izz': inertia[2],
        'body_length': source['body_length_m'],
        'body_diameter': source['body_diameter_m'],
        'S_ref': source['area_m2'],
        'd_ref': source['body_diameter_m'],
        'rho': rho,
        'g': source['g'],
        'C_Na': aero['CN_alpha'],
        'C_dc': aero['CN_cross'],
        'C_A0': C_A0,
        'C_Aa2': 0.0,          # research 에 대응항이 없다(gz_aero 검증: C_Aa2 ≡ 0)
        'x_cp': source['cp_from_cg_m'],
        'C_lp': damping[0],
        'C_mq': damping[1],
        'num_rotors': 4,       # 설계 선택(CLAUDE.md 확정사항), 측정값 아님
        'arm_length': arm,
        # 기수축(x) 둘레. rocket_params 와 같은 배치이고 인덱스 순서도 같다
        # (z>0 = r1,r2 / z<0 = r3,r4 -> 피치 쌍, 쌍 안에서 요·반토크 상쇄).
        'rotor_positions': np.array([
            [0.0,  half,  half],
            [0.0, -half,  half],
            [0.0, -half, -half],
            [0.0,  half, -half],
        ]),
        'rotor_directions': np.array([1, -1, 1, -1]),
        'D_prop': diameter,
        'k_T': k_T,
        'k_Q': k_Q,
        'J_max': J_max,
        'I_rotor': motor['rotor_inertia_kg_m2'],
        'tau_m': motor['speed_loop_tau_s'],
        'n_min': 0.0,          # 모든 로터의 보편적 하한, 측정값 아님
        'n_max': float(min(tip, no_load)),

        'k_e': kt_motor, 'k_t': kt_motor,
        'R_m': motor['resistance_ohm'],
        'I_lim': motor['current_limit_A'],
        'eta_esc': battery['esc_efficiency'],
        'V_oc': battery['series']*4.2,
        'R_b': battery['resistance_ohm'],
        'C_b': battery['capacity_Ah'],

        # 표 기반(고충실도) 플랜트 전용 원본 테이블 — control/dynamics.py 의
        # 기존 함수(모든 제어기가 공유)는 안 건드리고, 별도 경로
        # (control/dynamics_hifi.py)만 이걸 읽는다. 형상이 바뀌어도 이 표만
        # 갈아 끼우면 된다.
        'prop_table': {'J': J_grid.tolist(), 'CT': CT_grid.tolist(),
                       'CP': CP_grid.tolist()},
        'drag_table': {'speed_mps': speeds.tolist(), 'CD0': drag_table.tolist()},

        'source': str(path),
        'source_id': source.get('id'),
    }
    return params


selected_params = load_selected_params()
