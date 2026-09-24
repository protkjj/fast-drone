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


def load_selected_params(path=_SELECTED_JSON):
    """research 선정 프로파일 -> control 파라미터 dict. 없으면 None."""
    try:
        source = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None

    prop, aero, motor = source['prop'], source['aero'], source['motor']
    rho, diameter = source['rho'], prop['diameter_m']

    # 추진 계수 환산. research 는 전진비 의존 맵(CT(J), CP(J))을 쓰고 control 은
    # 상수 k 하나에 전진비 선형 감쇠(J_max)를 쓴다. 정지(J=0) 값으로 환산한다:
    #   research  T = CT·rho·rev²·D⁴,  rev = n/(2π)   ->  k_T = CT·rho·D⁴/(4π²)
    #             Q = CP·rho·rev²·D⁵/(2π)             ->  k_Q = CP·rho·D⁵/(8π³)
    # 검산: 이렇게 얻은 k_T 로 계산한 호버 회전수가 원본의 호버 앵커 rpm 을
    #       1.5e-5 rpm 오차로 재현한다(단위 규약이 일관된다는 뜻).
    k_T = prop['CT'][0]*rho*diameter**4/(4*np.pi**2)
    k_Q = prop['CP'][0]*rho*diameter**5/(8*np.pi**3)

    # 축력 계수. research 의 CD0 는 **속도 테이블**이고 control 은 상수 하나다.
    # 이 프로파일이 실제로 검증받는 구간은 0~19.6 m/s 뿐인데 하필 거기서 CD0 가
    # 가장 빠르게 변한다(0.887 -> 0.568). 전 구간 평균(0.523)을 쓰면 정작 쓰는
    # 구간에서 틀린 값이 되므로 **0~20 m/s 구간 평균만** 쓴다.
    speeds = np.asarray(aero['speed_mps'], float)
    drag_table = np.asarray(aero['CD0'], float)
    low_speed = speeds <= 20.0
    C_A0 = float(drag_table[low_speed].mean())

    # 회전수 상한 — research/model.py rpm_limit 과 같은 식(팁마하 vs 무부하 중 작은 값).
    tip = 2*prop['tip_mach_limit']*source['sound_speed_mps']/diameter
    no_load = motor['kv_rpm_V']*source['battery']['series']*4.2*2*np.pi/60

    arm = source['arm_m']
    half = arm/np.sqrt(2)
    inertia = source['inertia_kg_m2']
    damping = aero['damping']          # [x, y, z] = [롤, 피치, 요]

    params = dict(vehicle_params)
    params.update({
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
        'I_rotor': motor['rotor_inertia_kg_m2'],
        'tau_m': motor['speed_loop_tau_s'],
        'n_min': 0.0,
        'n_max': float(min(tip, no_load)),
        'source': str(path),
        'source_id': source.get('id'),
    })
    return params


selected_params = load_selected_params()
