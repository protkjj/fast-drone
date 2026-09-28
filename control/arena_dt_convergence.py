"""플랜트 적분 수렴성 점검(원고 표 4·6.1절) — 같은 입력 열을 dt 2·1·0.5 ms로 적분해 상태 차이를 본다.

kj 결정(2026-09-28 밤): 대표 시나리오 몇 개(고속 포함)로 확인하고, 2 ms가 부족해 보이면 멈추고 보고한다.

방법
  1) 2 ms 경기장 폐루프 시행을 돌려 입력 열 u_k와 상태 x_k를 얻는다(`validation_suite.run_trial`).
  2) 기체는 개루프로 불안정할 수 있어 입력만 길게 재생하면 dt와 무관하게 차이가 지수적으로 커진다.
     그래서 **구간(기본 1 s)마다** 2 ms 궤적의 상태에서 다시 출발해, 같은 입력(2 ms 영차 유지)과 같은
     바람으로 dt' = 1 ms, 0.5 ms 플랜트를 적분하고 구간 끝 상태를 비교한다 — dt에서 오는 오차만 남는다.
  3) 0.5 ms를 기준으로 2 ms·1 ms의 차이를 보고, 비율(2 ms 오차 / 1 ms 오차)로 수렴 차수를 본다.
  4) **주 지표 = 한 스텝 국소 오차**: 궤적의 모든 스텝에서 같은 상태·입력으로 2 ms 한 스텝과 0.5 ms 네 스텝을
     비교한다. 1 s 구간 지표는 피드백 없이 입력만 재생하므로, 개루프로 불안정한 궤적(V13 고속)에서는
     1e-9 수준 차이가 지수적으로 커져 dt 적정성 판단에 쓸 수 없다(2026-09-28 실측: 구간 끝 2.2 m/s,
     같은 궤적의 한 스텝 오차 9.5e-9 m/s). 구간 지표는 참고로 남긴다.

주의: 팀원 플랜트는 한 스텝 안에서 RK4를 4번 나눠 적분한다(`number_of_finite_elements: 4`) — dt 2 ms의 실제
적분 간격은 0.5 ms다. 제어 갱신·입력 유지 주기(500 Hz)는 이 점검과 별개다(원고와 코드의 차이 목록).

    python3 -m control.arena_dt_convergence            # results/arena/dt_convergence/
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from control.arena import ROOT, load_config, build_scenarios

SCENARIOS = ('tune3_step_altitude_p0.7_V0', 'tune2_gust_lateral_p13_V82',
             'tune2_ramp_accel_V17_70_rho0.8', 'tune2_ramp_brake_V80_25_rho1.2')
FINE_DT = (1e-3, 5e-4)
SEGMENT_S = 1.0


def _attitude_deg(qa, qb):
    return float(np.degrees(np.linalg.norm((Rotation.from_quat(qa).inv()*Rotation.from_quat(qb)).as_rotvec())))


def _diff(xa, xb):
    return dict(pos_m=float(np.linalg.norm(xa[0:3] - xb[0:3])), vel_m_s=float(np.linalg.norm(xa[3:6] - xb[3:6])),
                att_deg=_attitude_deg(xa[6:10], xb[6:10]), omega_rad_s=float(np.linalg.norm(xa[10:13] - xb[10:13])),
                rotor_rad_s=float(np.max(np.abs(xa[13:17] - xb[13:17]))))


def replay_segments(truth, result, dt, segment_s=SEGMENT_S, fine=FINE_DT):
    """구간마다 2 ms 상태에서 출발해 dt'로 재적분. 반환: 구간별 {dt: 끝 상태}와 2 ms 끝 상태."""
    from control.arena_plant_wrench import build_plant
    xs, us, winds = result['xs'], result['us'], result['wind']
    n = len(us)
    per = int(round(segment_s/dt))
    plants = {h: build_plant(truth, dt=h) for h in fine}
    rows = []
    for k0 in range(0, n - per + 1, per):
        ends = {}
        for h, plant in plants.items():
            sub = int(round(dt/h))
            x = xs[k0].copy()
            for k in range(k0, k0 + per):
                for _ in range(sub):
                    x = plant.step(x, us[k].copy(), winds[k].copy())
            ends[h] = x
        rows.append(dict(t0=float(k0*dt), end_2ms=xs[k0 + per], ends=ends))
    return rows


def one_step_errors(truth, result, dt):
    """모든 스텝: 같은 상태·입력·바람에서 dt 한 스텝 대 dt/4 네 스텝(0.5 ms 기준). 묶음별 최댓값."""
    from control.arena_plant_wrench import build_plant
    coarse, fine = build_plant(truth, dt=dt), build_plant(truth, dt=dt/4)
    xs, us, winds = result['xs'], result['us'], result['wind']
    worst = None
    for k in range(len(us)):
        a = coarse.step(xs[k].copy(), us[k].copy(), winds[k].copy())
        b = xs[k].copy()
        for _ in range(4):
            b = fine.step(b, us[k].copy(), winds[k].copy())
        d = _diff(a, b)
        worst = d if worst is None else {key: max(worst[key], d[key]) for key in d}
    return worst


def run(labels=('GSLQR', 'V13'), scenarios=SCENARIOS, config_path=None):
    from control.arena_factory import ArenaFactory
    from control.validation_metrics import Acceptance
    from control.validation_suite import baseline_params, run_trial, plant_truth
    config = load_config(config_path) if config_path else load_config()
    native = baseline_params()
    factory = ArenaFactory(config, native)
    built = {s.id: s for s in build_scenarios(config, factory.cp, native, scenarios=config['tuning']['scenarios'])}
    limits = Acceptance(**config['acceptance'])
    report = []
    for sid in scenarios:
        s = built[sid]
        truth = plant_truth(native, s.cases[0])
        for label in labels:
            metrics, result, _ = run_trial(factory, label, s.profile, s.cases[0], limits)
            if metrics['stop_reason']:
                report.append(dict(scenario=sid, controller=label, skipped=f"trial stopped: {metrics['stop_reason']}"))
                continue
            rows = replay_segments(truth, result, factory.dt)
            worst = {}
            for r in rows:
                ref = r['ends'][min(FINE_DT)]
                pairs = {'2ms_vs_0.5ms': _diff(r['end_2ms'], ref), '1ms_vs_0.5ms': _diff(r['ends'][1e-3], ref)}
                for name, d in pairs.items():
                    w = worst.setdefault(name, {k: 0.0 for k in d})
                    for k, v in d.items():
                        w[k] = max(w[k], v)
            ratio = {k: (worst['2ms_vs_0.5ms'][k]/worst['1ms_vs_0.5ms'][k] if worst['1ms_vs_0.5ms'][k] > 0 else None)
                     for k in worst['2ms_vs_0.5ms']}
            local = one_step_errors(truth, result, factory.dt)
            report.append(dict(scenario=sid, controller=label, one_step_2ms_vs_0p5ms=local,
                               segments=len(rows), segment_s=SEGMENT_S,
                               worst=worst, ratio_2ms_over_1ms=ratio,
                               window_rmse_velocity=metrics.get('rmse_velocity'), max_omega=metrics.get('max_omega')))
            print(sid, label, 'one-step', json.dumps(local), flush=True)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena'/'dt_convergence')
    parser.add_argument('--controllers', nargs='*', default=['GSLQR', 'V13'])
    args = parser.parse_args(argv)
    report = run(tuple(args.controllers))
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out/'dt_convergence.json').write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
