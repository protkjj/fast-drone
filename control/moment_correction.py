"""제어기 모델 공력 모멘트 보정항(피치·요) — kj 결정(2026-09-28 밤, 보고서 23절).

왜: 제어기 집중정수 모델의 정적 모멘트는 M = r_cp × F_N 한 가지(`control/dynamics.py::_body_aerodynamics`)이고
x_cp(V)는 **트림 곡선에서만** 맞췄다. 트림을 벗어나면 플랜트(팀원 분산 공력)와 크게 달랐다(22절: 85 m/s
+30°에서 12배, 40 m/s +5°는 부호 반대). V13(INDI 측정)·CPID는 이 모델 모멘트를 안 쓰고 M17·F13·GSLQR은
직접 쓰므로 기울어진 경로였다.

보정항(트림에서 정의상 0):
    ΔM_y = q̄·S·d · Σ_i V̂^i · (Σ_{j=1..3} c_ij Δα^j + e_i β²)    Δα = α − α_ref(V)
    ΔM_z = q̄·S·d · Σ_i V̂^i · Σ_{j∈{1,3}} k_ij β^j               β  = atan2(v_b, u_b)
  - 피치의 β² 항: 트림 받음각이 0이 아니면 옆미끄럼이 피치 모멘트를 β에 대해 짝함수로 바꾼다(85 m/s
    ±20°에서 플랜트 ω̇_y −20.9, 보정 전 모델과 부호 반대). 피치 흔들기에선 v_b = 0이라 요·롤 결합은 없다.
  - V̂ = V/V_SCALE. α = atan2(w_b, u_b). α_ref(V)는 **제어기 모델 명목 트림**의 α(1 m/s 격자) 선형 보간.
  - j ≥ 1이라 Δα = β = 0이면 정확히 0 — 명목 트림(기존 경로로 계산)이 그대로 평형점으로 남는다.
  - 각도는 ±ANGLE_CLIP로 자른다(적합 창 ±20° 밖은 다항식 외삽, 역유입에서 atan2가 뒤집히는 것 방지).
  - 적합 대상: 같은 동체 속도·ω=0에서 (플랜트 모멘트 − 기존 모델 모멘트). 모멘트 그대로 최소제곱한다
    (각가속도 관문이 모멘트/관성 기준이라 — 계수로 적합하면 저속의 작은 모멘트가 과대 가중된다).

    python3 -m control.moment_correction      # configs/controller_moment_model.json + 적합 보고
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from control.arena import ROOT

DEFAULT_FILE = ROOT/'configs'/'controller_moment_model.json'
SCHEMA = 'controller_moment_model/1'
V_SCALE = 85.0
V_DEG = 3
PITCH_TERMS = (('alpha', 1), ('alpha', 2), ('alpha', 3), ('beta', 2))
YAW_TERMS = (('beta', 1), ('beta', 3))
FIT_WINDOW_DEG = 20.0
ANGLE_CLIP_DEG = 30.0
FIT_SPEEDS = tuple(float(v) for v in np.arange(2.0, 85.001, 1.0))
FIT_ANGLES_DEG = tuple(float(a) for a in np.arange(-FIT_WINDOW_DEG, FIT_WINDOW_DEG + 1e-9, 1.0))


def _funcs(native, cp):
    import casadi as ca
    from control.kh_adapter import _build_aero_function
    from control.dynamics import _body_aerodynamics
    v, w = ca.SX.sym('v', 3), ca.SX.sym('w', 3)
    base = {k: val for k, val in cp.items() if k != 'moment_correction'}
    model = ca.Function('model_aero', [v, w], list(_body_aerodynamics(v, w, base)))
    return _build_aero_function(native), model


def alpha_reference_table(model_trim, speeds=None):
    """제어기 모델 명목 트림의 α(동체 속도 기준) — 1 m/s 격자. model_trim은 ArenaFactory.model.trim."""
    speeds = np.arange(0.0, V_SCALE + 0.5, 1.0) if speeds is None else np.asarray(speeds, dtype=float)
    alphas = []
    for V in speeds:
        tr = model_trim(float(V))
        vb = Rotation.from_quat(tr['state'][6:10]).as_matrix().T @ tr['state'][3:6]
        alphas.append(float(np.arctan2(vb[2], vb[0])) if V > 0 else 0.0)
    return speeds.tolist(), alphas


def angles(vb, V_grid, a_ref):
    """모델(control/dynamics.py::_moment_correction)과 같은 식 — Δα, β(자르기 전)."""
    V = float(np.linalg.norm(vb))
    alpha_ref = float(np.interp(min(max(V, V_grid[0]), V_grid[-1]), V_grid, a_ref))
    return float(np.arctan2(vb[2], vb[0])) - alpha_ref, float(np.arctan2(vb[1], vb[0]))


def _samples(native, cp, V_grid, a_ref):
    """행 = (V, Δα, β, ΔM_y, ΔM_z, q̄Sd). 트림 동체 속도를 x-z면 안에서(피치) 또는 동체 z축으로(요)
    ±20° 돌린다. 두 흔들기 모두의 피치·요 잔차를 쓴다(교차 결합 β² 적합용)."""
    plant, model = _funcs(native, cp)
    rho, S, d = cp['rho'], cp['S_ref'], cp['d_ref']
    rows = []
    for V in FIT_SPEEDS:
        alpha0 = float(np.interp(V, V_grid, a_ref))
        vb0 = np.array([V*np.cos(alpha0), 0.0, V*np.sin(alpha0)])
        qsd = 0.5*rho*V**2*S*d
        for deg in FIT_ANGLES_DEG:
            for axis in ('y', 'z'):
                if axis == 'y':
                    a = alpha0 + np.radians(deg)
                    vb = np.array([V*np.cos(a), 0.0, V*np.sin(a)])
                else:
                    vb = Rotation.from_euler('z', np.radians(deg)).as_matrix() @ vb0
                dM = (np.array(plant(vb, [0, 0, 0])[1]).ravel() - np.array(model(vb, [0, 0, 0])[1]).ravel())
                da, beta = angles(vb, V_grid, a_ref)
                rows.append((V, da, beta, float(dM[1]), float(dM[2]), qsd))
    return rows


def term_value(name, power, d_alpha, beta):
    return (d_alpha if name == 'alpha' else beta)**power


def basis(Vhat, d_alpha, beta, terms):
    """열 순서: 항마다 V̂^0..V̂^V_DEG — 모델 쪽 _moment_correction과 같은 순서."""
    return np.array([Vhat**i*term_value(name, power, d_alpha, beta)
                     for name, power in terms for i in range(V_DEG + 1)])


def fit(native, cp, model_trim):
    V_grid, a_ref = alpha_reference_table(model_trim)
    rows = _samples(native, cp, V_grid, a_ref)
    blocks = {}
    for key, terms, col in (('pitch', PITCH_TERMS, 3), ('yaw', YAW_TERMS, 4)):
        A = np.array([r[5]*basis(r[0]/V_SCALE, r[1], r[2], terms) for r in rows])
        b = np.array([r[col] for r in rows])
        c, *_ = np.linalg.lstsq(A, b, rcond=None)
        resid = A @ c - b
        blocks[key] = dict(terms=[list(t) for t in terms], coeffs=c.tolist(),
                           rms_Nm=float(np.sqrt(np.mean(resid**2))), max_abs_Nm=float(np.max(np.abs(resid))),
                           target_rms_Nm=float(np.sqrt(np.mean(b**2))))
    return dict(schema=SCHEMA, V_scale=V_SCALE, V_degree=V_DEG, fit_window_deg=FIT_WINDOW_DEG,
                angle_clip_deg=ANGLE_CLIP_DEG, alpha_ref=dict(V=V_grid, alpha=a_ref), **blocks)


def model_sha256(path=DEFAULT_FILE):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path=DEFAULT_FILE):
    doc = json.loads(Path(path).read_text(encoding='utf-8'))
    if doc.get('schema') != SCHEMA:
        raise ValueError(f'{path}: not a controller moment model')
    return doc


def main(argv=None):
    from control.arena import load_config
    from control.arena_factory import ArenaFactory
    from control.validation_suite import baseline_params
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', type=Path, default=DEFAULT_FILE)
    args = parser.parse_args(argv)
    native = baseline_params()
    factory = ArenaFactory(load_config(), native)
    doc = fit(native, factory.cp, factory.model.trim)
    args.out.write_text(json.dumps(doc, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    for axis in ('pitch', 'yaw'):
        r = doc[axis]
        print(f"{axis}: residual rms {r['rms_Nm']:.2e} N·m (target rms {r['target_rms_Nm']:.2e}), "
              f"max {r['max_abs_Nm']:.2e}")
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
