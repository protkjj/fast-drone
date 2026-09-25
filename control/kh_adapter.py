"""새 기체(light_rocket_v2_pack_forward, 팀원 저장소) 어댑터.

kj 지시(권고순서 1, 2026-09-25 저녁 갱신) 그대로:
  1) 팀원 플랜트는 재구현하지 않는다 — 이제 `models/team_light/control`에
     git 병합으로 정식 편입되어 있다(kms301111/fast-drone-control, 커밋
     3bf4e60). 예전에는 이름 충돌 때문에 `external/fastdrone_kh`를
     `kh_control`로 이름 바꿔 벤더링했었지만, 병합 이후로는 그게 중복이라
     여기서도 `models.team_light.control`을 그대로 쓴다(어댑터를 새로 만들지
     않는다 — kj: "이 플랜트를 그대로 쓴다").
  2) 우리 제어기(V13 등)의 예측모델이 쓰는 **집중정수** 공력 계수는 팀원의
     **분산** 공력 모델에서 α·V를 샘플링해 최소제곱으로 적합한다 — 로터
     기하(위치·방향)는 근사하지 않고 팀원 값을 그대로 쓴다(둘 다 동체좌표계
     안에서만 쓰이는 값이라 `control/test_kh_convention_match.py`가 확인한
     것처럼 섞어도 안전하다).
  3) `models/team_light/control/`(ProperHybrid 등)는 우리 control/의 옛
     스냅샷(a00c75e, 2026-09-09, cost_spec/time_align/alloc_feedback 등
     이번 세션 기능이 전혀 없는 187줄짜리 구버전)이다. 팀원의 "분리형
     부진" 결과는 이 구버전 결과이므로, 재현은 반드시 이 병합 이후의
     **현재** `control/hybrid_comparison.py`(1006줄)로 한다 — 플랜트만
     팀원 것을 쓰고 제어기는 우리 최신 코드를 쓰는 이유가 이것이다.

결과 구조: **플랜트는 팀원 것(고충실도, 분산 공력+Ct/Cp 곡선), 제어기
예측모델은 우리 것(단순화, 이 파일이 적합한 집중정수)** — 2026-09-24 밤에
만든 "플랜트≠제어기 모델" 원칙(`control/dynamics_hifi.py`)을 외부 기체에도
그대로 적용한 것이다.
"""
import casadi as ca
import numpy as np

from models.team_light.control.baseline_v2 import baseline_params as _kh_baseline_params  # noqa: E402
from models.team_light.control.light_aero import light_aerodynamics as _kh_light_aero  # noqa: E402
from models.team_light.control.trim import find_trim as _kh_find_trim  # noqa: E402


def kh_native_params():
    """팀원 플랜트에 그대로 넣을 원본 파라미터. 절대 수정하지 말 것 —
    이 dict가 `models.team_light.control.dynamics.AxialDronePlant`/`find_trim`의 입력이다.
    """
    return _kh_baseline_params()


def _build_aero_function(p):
    v = ca.SX.sym('v', 3)
    w = ca.SX.sym('w', 3)
    f, m = _kh_light_aero(v, w, p)
    return ca.Function('kh_light_aero', [v, w], [f, m])


def _trim_v_body(native_params, speeds):
    """팀원 자체 find_trim으로 얻은 실제 트림점의 동체좌표 속도(u,w).

    독립적인 (V,α) 전 구간 격자를 처음 시도했을 때 적합 품질이 심각하게
    나빴다(Fx 최대상대잔차 158%, My 146배) — V=85·α=85° 같은, 트림된 기체가
    절대 도달하지 않는 조합까지 넣어 전체 최소제곱을 오염시킨 것이었다.
    실제로 제어기가 마주칠 영역(트림 근방)에서만 좋은 근사면 되므로, 이
    함수로 트림 곡선 자체를 앵커로 삼는다.
    """
    anchors = []
    for V in speeds:
        try:
            tr = _kh_find_trim(native_params, float(V))
        except Exception:
            continue
        q, v_world = tr['state'][6:10], tr['state'][3:6]
        from scipy.spatial.transform import Rotation
        v_body = Rotation.from_quat(q).as_matrix().T @ v_world
        anchors.append((float(V), float(v_body[0]), float(v_body[2])))
    return anchors


def _sample_near_trim(anchors, alpha_window_deg=20.0, v_scale=(0.85, 1.15),
                      n_alpha=9, n_v=3):
    """트림 곡선 각 앵커점 주변의 국소 범위만 샘플링(전 구간 격자 대신).

    실제 비행이 벗어나는 범위(모델오차·초기오차·돌풍)를 감당할 정도로만
    폭을 준다 — ±20°, ±15% 속도. 그 이상은 표7 섭동 시험 몫이지 명목
    예측모델 적합의 몫이 아니다.
    """
    rows = []
    for V, u0, w0 in anchors:
        V_local = np.hypot(u0, w0)
        alpha0 = np.arctan2(w0, u0)
        for da in np.linspace(-np.radians(alpha_window_deg),
                              np.radians(alpha_window_deg), n_alpha):
            for vs in np.linspace(v_scale[0], v_scale[1], n_v):
                a = alpha0 + da
                Vp = max(V_local*vs, 0.5)
                u, w_ax = Vp*np.cos(a), Vp*np.sin(a)
                rows.append((V, np.degrees(a), u, w_ax))
    return rows


def fit_lumped_aero(native_params, speeds=None, alpha_window_deg=20.0,
                    omega_probe=0.05, verbose=True):
    """분산 공력 모델(`models.team_light.control.light_aero`)을 우리 집중정수 계수로 적합.

    샘플링은 **팀원 자체 트림 곡선 근방**(±각도창, ±속도스케일)에서만 한다
    (`_sample_near_trim` 참고 — 전 구간 격자를 썼을 때의 실패 기록도 함께
    적어 뒀다). 반환: (coeffs, report) — coeffs는
    `C_Na,C_dc,C_A0,C_Aa2,x_cp,C_lp,C_mq`를 담은 dict, report는 각 적합의
    최대 상대잔차(적합 품질, 숨기지 않는다).
    """
    if speeds is None:
        speeds = np.arange(2.0, 90.001, 2.0)          # 0 근방 제외(sign 특이점)

    f_aero = _build_aero_function(native_params)
    rho, S = native_params['rho'], native_params['S_ref']

    anchors = _trim_v_body(native_params, speeds)
    rows = _sample_near_trim(anchors, alpha_window_deg=alpha_window_deg)
    U = np.array([r[2] for r in rows])
    W = np.array([r[3] for r in rows])
    F = np.zeros((len(rows), 3))
    for i, (V, a_deg, u, w_ax) in enumerate(rows):
        force, _ = f_aero([u, 0.0, w_ax], [0.0, 0.0, 0.0])
        F[i] = np.array(force).ravel()
    Fx, Fz = F[:, 0], F[:, 2]

    # ── 법선력 계수 (C_Na, C_dc): Fz = -0.5*rho*S*(C_Na*|u| + C_dc*|w|)*w ──
    X1 = -0.5*rho*S*np.abs(U)*W
    X2 = -0.5*rho*S*np.abs(W)*W
    A_fz = np.column_stack([X1, X2])
    (C_Na, C_dc), *_ = np.linalg.lstsq(A_fz, Fz, rcond=None)
    fz_pred = A_fz @ [C_Na, C_dc]
    fz_resid = fz_pred - Fz

    # ── 축력 계수 (C_A0, C_Aa2): Fx*sign(u) = -0.5*rho*S*(C_A0*V² + C_Aa2*w²) ──
    Vsq = U**2 + W**2
    Y = Fx*np.sign(U)
    Z1 = -0.5*rho*S*Vsq
    Z2 = -0.5*rho*S*(W**2)
    A_fx = np.column_stack([Z1, Z2])
    (C_A0, C_Aa2), *_ = np.linalg.lstsq(A_fx, Y, rcond=None)
    fx_pred = A_fx @ [C_A0, C_Aa2]
    fx_resid = fx_pred - Y

    # ── 압력중심 x_cp: My = -x_cp * Fz  (같은 격자, 이미 계산한 Fz 재사용) ──
    My = np.zeros(len(rows))
    for i, (V, a_deg, u, w_ax) in enumerate(rows):
        _, moment = f_aero([u, 0.0, w_ax], [0.0, 0.0, 0.0])
        My[i] = float(np.array(moment).ravel()[1])
    A_my = (-Fz).reshape(-1, 1)
    (x_cp,), *_ = np.linalg.lstsq(A_my, My, rcond=None)
    my_pred = A_my @ [x_cp]
    my_resid = my_pred - My

    # ── 감쇠 계수 (C_lp, C_mq): 명목 순항점 근방에서 ω 섭동 유한차분 ──
    V0 = 40.0
    d_ref = native_params['d_ref']
    lp_vals, mq_vals = [], []
    for a_deg in (0.0, 15.0, 30.0, 45.0):
        a = np.radians(a_deg)
        u0, w0 = V0*np.cos(a), V0*np.sin(a)
        V_speed = np.hypot(u0, w0)
        df = 0.25*rho*V_speed*S*d_ref**2
        _, m_base = f_aero([u0, 0.0, w0], [0.0, 0.0, 0.0])
        m_base = np.array(m_base).ravel()
        _, m_p = f_aero([u0, 0.0, w0], [omega_probe, 0.0, 0.0])
        lp_vals.append((float(m_p[0]) - m_base[0])/(df*omega_probe))
        _, m_q = f_aero([u0, 0.0, w0], [0.0, omega_probe, 0.0])
        mq_vals.append((float(m_q[1]) - m_base[1])/(df*omega_probe))
    C_lp, C_mq = float(np.mean(lp_vals)), float(np.mean(mq_vals))

    def _r2(y, resid):
        ss_res = float(np.sum(resid**2))
        ss_tot = float(np.sum((y - y.mean())**2))
        return 1.0 - ss_res/ss_tot if ss_tot > 1e-12 else float('nan')

    # 상대잔차를 개별 샘플로 나누면 신호가 0에 가까운 점(예: 거의 순수
    # 옆미끄럼이라 축력이 0에 가까운 저속 큰-α 점)에서 비율이 터진다 —
    # 처음 이 지표를 그렇게 짰다가 "Fx 최대상대잔차 66배"라는 가짜 경보를
    # 봤다(실제 절대오차는 0.02N, 중량 11.5N의 0.2%). R²(전체 분산 대비
    # 잔차)와 절대 RMS/최대오차로 바꿨다 — 무게 대비 %도 함께 준다.
    weight = native_params['mass']*native_params['g']
    coeffs = {'C_Na': float(C_Na), 'C_dc': float(C_dc), 'C_A0': float(C_A0),
             'C_Aa2': float(C_Aa2), 'x_cp': float(x_cp),
             'C_lp': C_lp, 'C_mq': C_mq}
    report = {
        'n_samples': len(rows), 'weight_N': float(weight),
        'Fz_r2': _r2(Fz, fz_resid),
        'Fz_rms_N': float(np.sqrt(np.mean(fz_resid**2))),
        'Fz_max_abs_N': float(np.max(np.abs(fz_resid))),
        'Fx_r2': _r2(Y, fx_resid),
        'Fx_rms_N': float(np.sqrt(np.mean(fx_resid**2))),
        'Fx_max_abs_N': float(np.max(np.abs(fx_resid))),
        'My_r2': _r2(My, my_resid),
        'My_rms_Nm': float(np.sqrt(np.mean(my_resid**2))),
        'My_max_abs_Nm': float(np.max(np.abs(my_resid))),
        'C_lp_samples': lp_vals, 'C_mq_samples': mq_vals,
    }
    if verbose:
        print(f"공력 적합 품질 (n={report['n_samples']} 샘플, 중량 {weight:.3f} N 기준):")
        print(f"  Fz  R²={report['Fz_r2']:.4f}  RMS {report['Fz_rms_N']:.4f} N "
              f"({100*report['Fz_rms_N']/weight:.2f}%W)  최대오차 {report['Fz_max_abs_N']:.4f} N")
        print(f"  Fx  R²={report['Fx_r2']:.4f}  RMS {report['Fx_rms_N']:.4f} N "
              f"({100*report['Fx_rms_N']/weight:.2f}%W)  최대오차 {report['Fx_max_abs_N']:.4f} N")
        print(f"  My  R²={report['My_r2']:.4f}  RMS {report['My_rms_Nm']:.4f} N·m  "
              f"최대오차 {report['My_max_abs_Nm']:.4f} N·m")
        print(f"  C_lp 샘플 {np.round(lp_vals,4)}  C_mq 샘플 {np.round(mq_vals,4)}"
              f"  (분산 {'큼 — 단일상수 근사가 약함' if np.ptp(mq_vals)>0.5*abs(np.mean(mq_vals)) else '작음'})")
    return coeffs, report


def build_controller_params(native_params=None, aero_coeffs=None):
    """우리 제어기(V13/M17/F13/GSLQR/CPID)에 넣을 params dict.

    기하·질량·추진(k_T/k_Q/J_max 정적기준)은 팀원 값 **그대로**, 공력만
    `fit_lumped_aero`가 적합한 집중정수로 바꾼다. `thrust_axis`는 팀원
    스키마(3벡터)를 우리 스키마(문자열)로 정규화한다(스키마 차이, 물리 차이
    아님 — `test_kh_convention_match.py` 참고).
    """
    native = native_params if native_params is not None else kh_native_params()
    coeffs = aero_coeffs if aero_coeffs is not None else fit_lumped_aero(native, verbose=False)[0]

    p = dict(native)
    p['thrust_axis'] = 'x'
    p.update(coeffs)
    # 우리 쪽 body_length/body_diameter 등은 native 에 이미 같은 이름으로
    # 있다(mass,Ixx,Iyy,Izz,body_length,body_diameter,S_ref,d_ref,rho,g,
    # num_rotors,arm_length,rotor_positions,rotor_directions,D_prop,k_T,k_Q,
    # J_max,I_rotor,tau_m,n_min,n_max) — 그대로 상속한다.
    return p
