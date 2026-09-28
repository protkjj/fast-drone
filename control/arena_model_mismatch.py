"""I-10 확장 측정(정보용) — 제어기 내부 모델 대 플랜트 가속도 불일치를 트림 밖 큰 각도까지 잰다.

kj 지시(2026-09-28): 공력 담당 확인 결과 플랜트 공력 모델은 큰 받음각에서도 신뢰할 수 있다. 그런데
제어기 내부 공력 모델(집중정수)은 ±20° 창에서 적합했다(`control/kh_adapter.py::fit_lumped_aero`).
그래서 큰 받음각에서는 플랜트와 제어기 모델이 크게 달라질 수 있다. 불일치를 ±30°까지 수치로 기록한다.
측정 결과(보고서 22절) 이 모멘트 모델은 M17·F13·GSLQR만 직접 쓰고 V13(INDI 측정)·CPID는 안 써서
기울어진 경로였다 → 보정항을 넣었다(control/moment_correction.py, configs/arena_v2.json, 보고서 23절).
±20° 관문은 control/test_arena_fairness.py의 I-10 확장 시험, ±30°는 이 도구의 정보용 기록.

측정: 속도마다 플랜트 트림 상태에서 자세만 동체 y축(받음각 방향)·z축(옆미끄럼 방향)으로 돌리고,
로터 입력은 트림값 그대로 둔다. 같은 상태·입력에서 각 모델의 가속도를 플랜트와 비교한다.

    python3 -m control.arena_model_mismatch            # results/arena/model_mismatch/ 에 JSON·MD
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from control.arena import ROOT

ANGLES_DEG = tuple(range(-30, 31, 5))
SPEEDS = (0.0, 20.0, 40.0, 60.0, 85.0)
AXES = {'y': 'pitch (angle of attack)', 'z': 'yaw (sideslip)'}


def model_accelerations(factory, x, cache=None):
    """같은 상태·로터 입력에서 플랜트와 각 제어기 모델의 가속도(I-10 정의 그대로).
      M17·GSLQR  17상태 모델(control.dynamics) — M17 예측, GSLQR 트림·선형화
      F13        로터추력 입력 13상태 모델 — 입력은 플랜트의 실제 로터 추력, γ는 측정값
      V13        가상입력 모델 — T = 플랜트 총추력(각가속도는 입력 ν라 비교 대상 아님)
    반환: (플랜트 xdot, {이름: (병진 가속도, 각가속도 또는 None)})."""
    import casadi as ca
    from control.dynamics import AxialDronePlant, reaction_torque_ratio, axial_airspeed
    from control.hybrid_comparison import build_virtual_dynamics
    from control.nmpc_f13 import build_f13_dynamics
    from models.team_light.control.dynamics import AxialDronePlant as TeamPlant
    from models.team_light.control.geometry import rotor_thrusts
    if cache is None:
        cache = factory.__dict__.setdefault('_i10_cache', {})
    if not cache:
        # 제어기마다 실제로 예측·선형화에 쓰는 모델(arena_v2의 모멘트 보정은 M17·F13·GSLQR만).
        # 보정이 없는 설정이면 cp_for가 같은 명목 cp를 돌려줘 기존 I-10과 같다.
        cp_for = getattr(factory, '_cp_for', lambda label: factory.cp)
        cache['ours'] = AxialDronePlant(cp_for('M17'))
        cache['plant'] = TeamPlant(factory.p)
        cache['v13'] = build_virtual_dynamics(factory.cp)[0]
        cache['f13'] = build_f13_dynamics(cp_for('F13'), torque_ratio=ca.SX.sym('g', 4))[0]
    u = x[13:17]
    plant = cache['plant'].evaluate_xdot(x, u)
    ours = cache['ours'].evaluate_xdot(x, u)
    v_body = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]
    T = rotor_thrusts(factory.p, u, v_body)
    x13 = np.concatenate([x[0:10], x[10:13]])
    gamma = reaction_torque_ratio(factory.cp, u, axial_airspeed(factory.cp, x))   # 추진 모델 — 보정과 무관
    f13 = np.array(cache['f13'](x13, T, gamma)).ravel()
    v13 = np.array(cache['v13'](x13, np.r_[T.sum(), plant[10:13]])).ravel()
    return plant, {'M17/GSLQR': (ours[3:6], ours[10:13]), 'F13': (f13[3:6], f13[10:13]),
                   'V13': (v13[3:6], None)}


def measure(factory, native, speeds=SPEEDS, angles=ANGLES_DEG):
    from models.team_light.control.trim import find_trim
    rows = []
    for V in speeds:
        x0 = find_trim(native, float(V))['state'].copy()
        x0[2] = 20.0
        for axis in AXES:
            for deg in angles:
                x = x0.copy()
                x[6:10] = (Rotation.from_quat(x0[6:10])*Rotation.from_euler(axis, deg, degrees=True)).as_quat()
                plant, models = model_accelerations(factory, x)
                vb = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]
                row = dict(speed=float(V), axis=axis, angle_deg=deg,
                           alpha_deg=float(np.degrees(np.arctan2(vb[2], vb[0]))) if V > 0 else None)
                for name, (a, w) in models.items():
                    row[f'{name}_dacc'] = float(np.abs(a - plant[3:6]).max())
                    row[f'{name}_domega'] = None if w is None else float(np.abs(w - plant[10:13]).max())
                rows.append(row)
    return rows


def write(rows, out, config_name='configs/arena.json'):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out/'model_mismatch.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding='utf-8')
    lines = ['# I-10 확장 측정(정보용): 제어기 모델 대 플랜트, 트림 자세 ±30°', '',
             '관문이 아니다(I-10 관문: 병진 ≤ 0.49 m/s², 트림 각가속도 ≤ 2 rad/s²). 로터 입력은 트림값.',
             '제어기 내부 공력은 ±20° 창에서 적합했다(kh_adapter.fit_lumped_aero).',
             f'설정: {config_name}', '']
    for axis, title in AXES.items():
        lines += [f'## 동체 {axis}축 회전 — {title}', '',
                  '| V (m/s) | 각도 (°) | 받음각 (°) | M17/GSLQR Δa | M17/GSLQR Δω̇ | F13 Δa | F13 Δω̇ | V13 Δa |',
                  '|---:|---:|---:|---:|---:|---:|---:|---:|']
        for r in rows:
            if r['axis'] != axis:
                continue
            alpha = '—' if r['alpha_deg'] is None else f"{r['alpha_deg']:.1f}"
            lines.append(f"| {r['speed']:g} | {r['angle_deg']:+d} | {alpha} | {r['M17/GSLQR_dacc']:.3f} | "
                         f"{r['M17/GSLQR_domega']:.2f} | {r['F13_dacc']:.3f} | {r['F13_domega']:.2f} | "
                         f"{r['V13_dacc']:.3f} |")
        lines.append('')
    lines.append('단위: Δa = max 성분 |a_model − a_plant| [m/s²], Δω̇ = max 성분 [rad/s²].')
    (out/'MODEL_MISMATCH.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')


def main(argv=None):
    from control.arena import load_config
    from control.arena_factory import ArenaFactory
    from control.validation_suite import baseline_params
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=ROOT/'configs'/'arena.json')
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena'/'model_mismatch')
    args = parser.parse_args(argv)
    native = baseline_params()
    factory = ArenaFactory(load_config(args.config), native)
    rows = measure(factory, native)
    write(rows, args.out, str(args.config.relative_to(ROOT)) if args.config.is_absolute() else str(args.config))
    print(f'wrote {len(rows)} rows to {args.out}')


if __name__ == '__main__':
    main()
