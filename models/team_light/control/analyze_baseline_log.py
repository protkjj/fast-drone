"""Postprocess saved baseline traces; never rerun or retune a controller."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from models.team_light.control.baseline_v2 import baseline_params, PARAMETER_SHA256
from models.team_light.control.mission_sim import MissionProfile
from models.team_light.control.propeller_curve import positive_thrust_j_limit
from models.team_light.control.trim import find_trim


def analyze(folder):
    report = json.loads((folder / 'report.json').read_text(encoding='utf-8'))
    if report['parameter_sha256'] != PARAMETER_SHA256:
        raise ValueError('Not the frozen pack_forward v2 baseline.')
    p = baseline_params()
    rows = []
    phase_rows = []
    for result in report['results']:
        tag = f"{result['controller']}_{result['speed_m_s']:g}_{result['case']}"
        if result['mission']:
            tag += '_mission'
        with np.load(folder / (tag + '.npz')) as data:
            t, x, u = data['t'], data['x'], data['u']
            wind = data['wind']
            # Same state/flow alignment used by the online domain monitor.
            vb = Rotation.from_quat(x[1:, 6:10]).inv().apply(x[1:, 3:6] - wind)
            rpm = x[1:, 13:17] * 60 / (2 * np.pi)
            j = np.maximum(vb[:, 0:1], 0.) / (x[1:, 13:17] * p['D_prop'] / (2*np.pi) + 1e-8)
            lo, hi = p['prop_curve']['assumed_rpm_working_range']
            failure_log = [a for a in result['solve_log'] if not a['accepted']]
            row = dict(tag=tag, completed=result['completed'], elapsed_s=float(t[-1]),
                       first_solver_failure_s=failure_log[0]['t'] if failure_log else None,
                       solver_failure_statuses=dict(Counter(a['status'] for a in failure_log)),
                       maximum_rate_norm_rad_s=float(np.max(np.linalg.norm(x[:, 10:13], axis=1))),
                       actual_rotor_min_rpm=float(np.min(rpm)), actual_rotor_max_rpm=float(np.max(rpm)),
                       outside_low_rpm_fraction=float(np.mean(np.any(rpm < lo-1e-6, axis=1))),
                       outside_high_rpm_fraction=float(np.mean(np.any(rpm > hi+1e-6, axis=1))),
                       outside_positive_thrust_J_fraction=float(np.mean(np.any(j > positive_thrust_j_limit(p)+1e-8, axis=1))),
                       reverse_axial_flow_fraction=float(np.mean(vb[:, 0] < -1e-8)))
            rows.append(row)
            if result['mission']:
                for name, begin, end in MissionProfile(70., 50.).get_phase_boundaries():
                    mask = (t >= begin) & (t < end)
                    if not np.any(mask):
                        continue
                    ez = x[mask, 2] - data['z_ref'][mask]
                    evx = x[mask, 3] - data['v_ref'][mask, 0]
                    phase_rows.append(dict(controller=result['controller'], phase=name,
                                           interval_s=[begin, end], phase_completed=bool(t[-1] >= end-1e-8),
                                           z_rmse_m=float(np.sqrt(np.mean(ez**2))),
                                           z_max_m=float(np.max(abs(ez))),
                                           vx_rmse_m_s=float(np.sqrt(np.mean(evx**2)))))
    trims = []
    for speed in (0., 70., 80., 85.):
        trim = find_trim(p, speed)
        trims.append(dict(speed_m_s=speed, theta_deg=float(np.degrees(trim['theta'])),
                          residual=float(trim['residual']),
                          max_rpm=float(np.max(trim['control'])*60/(2*np.pi)),
                          rpm_margin_percent=float(100*(1-np.max(trim['control'])/p['n_max']))))
    diagnostic = dict(parameter_sha256=PARAMETER_SHA256, trims=trims, traces=rows,
                      mission_phases=phase_rows)
    (folder / 'DIAGNOSTICS.json').write_text(json.dumps(diagnostic, ensure_ascii=False, indent=2), encoding='utf-8')
    lines = ['# 저장 로그 추가 점검', '', '기체·제어기 재튜닝 없이 저장된 시계열을 분석했다.', '',
             '## 트림 재확인', '', '| 속도 m/s | 피치 deg | 잔차 | 최대 rpm | 회전수 여유 % |',
             '|---:|---:|---:|---:|---:|']
    for r in trims:
        lines.append(f"| {r['speed_m_s']:g} | {r['theta_deg']:.3f} | {r['residual']:.2e} | {r['max_rpm']:.1f} | {r['rpm_margin_percent']:.2f} |")
    lines += ['', '## 통합 미션 구간별 결과', '',
              '기존 미션은 상승 중에도 수직속도 기준을 0으로 반환한다. 따라서 전체 속도 norm RMSE에는 상승 운동도 포함된다. 아래에는 전진속도와 고도 지표를 따로 기재했다.',
              '부분 구간은 그 구간을 끝내기 전에 중단했다는 뜻이며, 완주 결과와 RMSE 순위를 비교하면 안 된다.', '',
              '| 제어기 | 구간 | 구간 완료 | 고도 RMSE m | 최대 고도편차 m | 전진속도 RMSE m/s |',
              '|---|---|---|---:|---:|---:|']
    for r in phase_rows:
        lines.append(f"| {r['controller']} | {r['phase']} | {'완료' if r['phase_completed'] else '부분'} | {r['z_rmse_m']:.3f} | {r['z_max_m']:.3f} | {r['vx_rmse_m_s']:.3f} |")
    lines += ['', '## 최적화 실패와 추진 가정 범위', '',
              '추진 가정 범위 이탈 원인은 서로 중복될 수 있다. 역방향 유입, 낮은 회전수, 양의 추력 전진비 범위 초과를 구분한다. 범위 밖에서는 수치 연장식이 사용되므로 물리적으로 검증된 운동으로 볼 수 없다.', '',
              '| 시험 | 최초 솔버 실패 s | 낮은 rpm % | 높은 rpm % | 전진비 초과 % | 역방향 유입 % |',
              '|---|---:|---:|---:|---:|---:|']
    for r in rows:
        if (r['first_solver_failure_s'] is not None or any(r[k] > 0. for k in
                ('outside_low_rpm_fraction', 'outside_high_rpm_fraction', 'outside_positive_thrust_J_fraction', 'reverse_axial_flow_fraction'))):
            first = '-' if r['first_solver_failure_s'] is None else f"{r['first_solver_failure_s']:.3f}"
            lines.append(f"| {r['tag']} | {first} | {100*r['outside_low_rpm_fraction']:.2f} | {100*r['outside_high_rpm_fraction']:.2f} | {100*r['outside_positive_thrust_J_fraction']:.2f} | {100*r['reverse_axial_flow_fraction']:.2f} |")
    lines += ['', '솔버 실패 원인 코드와 각속도·회전수 극값은 DIAGNOSTICS.json에 있다. 시간적 선후관계만으로 근본 원인이 증명되는 것은 아니다.']
    (folder / 'DIAGNOSTICS.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    missions = [r for r in report['results'] if r['mission']]
    if missions:
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for r in missions:
            tag = f"{r['controller']}_{r['speed_m_s']:g}_{r['case']}_mission"
            with np.load(folder / (tag+'.npz')) as data:
                label = r['controller'] + ('' if r['completed'] else ' [stopped]')
                axes[0].plot(data['t'], data['x'][:, 2], label=label)
                axes[1].plot(data['t'], data['x'][:, 3], label=label)
        times = np.linspace(0., 65., 651)
        vr, zr = MissionProfile(70., 50.).compute_refs(times)
        axes[0].plot(times, zr, 'k--', label='Reference')
        axes[1].plot(times, vr[:, 0], 'k--', label='Reference')
        for axis, label in zip(axes, ('Altitude [m]', 'Forward speed [m/s]')):
            axis.set_ylabel(label)
            axis.grid(alpha=.25)
            axis.legend(ncol=2, fontsize=8)
            for boundary in (10, 13, 28, 43, 58):
                axis.axvline(boundary, color='k', alpha=.15, linewidth=.8)
        axes[1].set_xlabel('Time [s]')
        fig.suptitle('Frozen pack_forward v2 | legacy 70 m/s mission | stopped traces are partial')
        fig.tight_layout()
        fig.savefig(folder/'mission_trajectories.png', dpi=150)
        plt.close(fig)
    print(folder / 'DIAGNOSTICS.md')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=Path)
    analyze(parser.parse_args().folder)
