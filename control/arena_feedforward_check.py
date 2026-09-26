"""참조 가속도 피드포워드 전후 대조 — kj 결정(2026-09-26 오후). 결과는 **PILOT**(우위 결론 없음).

kj 결정 2: 참조 가속도 피드포워드를 두 기준선에 넣었다.
  - CPID는 PX4 PositionControl의 `_acc_sp`와 같은 구조다.
  - GSLQR은 표준 추종 LQR 피드포워드이고, 명목 모델로 정한 선형 유효 범위에서 포화한다
    (control/controller.py::feedforward_validity).
두 제어기 모두 NMPC와 같은 참조 창에서, NMPC 예측 격자의 첫 간격(0.05 s) 앞까지만 읽는다.
"적용 전후 가속 사례 RMSE를 기록한다" — 이 도구가 그 표를 만든다.

  (1) 가속 사례, 피드포워드 끔/켬
      GSLQR  본시험 mission_VH·mission_VH_mass1.3·mission_VH_tau2·ref_accel_0_VH_rho1·
             ref_brake_VH_VL_rho1, 튜닝 tune_ramp_40_70_rho0.8·tune_ramp_0_15_rho0.1
      CPID   튜닝 tune_ramp_0_15_rho0.1
             (설계 영역 안의 가속 사례는 이것뿐이다. 본시험의 CPID 사례는 V_L 돌풍이다)
  (2) 참조가 일정한 사례: 끔/켬 궤적이 비트 동일해야 한다(피드포워드가 0이라 건너뛴다)
  (3) 설계점검 끔/켬 — 매끄러운 계단(지금 근거)과 원계단(옛 기록) 둘 다
      계단 참조를 차분하면 계단 직전 0.05 s 동안 큰 가속도 펄스가 된다. 본시험 참조는 매끄럽다.
      고도 계단은 속도 참조가 일정해서 동일해야 한다.
  (4) GSLQR 유효 범위 표, 그리고 사례마다 참조 가속도가 그 범위를 넘은 시간 비율
      (선형 가정의 한계가 걸린 몫)

'개선' 문턱은 두지 않고 모든 사례를 적는다. 끔 = configs/arena.json에서 reference_feedforward만 뺀 설정이다.

실행: python -m control.arena_feedforward_check [--out results/arena]
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from control.arena import load_config, config_sha256, build_scenarios, DEFAULT_CONFIG, ROOT
from control.arena_factory import ArenaFactory

LABELS = ('GSLQR', 'CPID')
ACCEL_CASES = {'GSLQR': ('mission_VH', 'mission_VH_mass1.3', 'mission_VH_tau2', 'ref_accel_0_VH_rho1',
                         'ref_brake_VH_VL_rho1', 'tune_ramp_40_70_rho0.8', 'tune_ramp_0_15_rho0.1'),
               'CPID': ('tune_ramp_0_15_rho0.1',)}
CONSTANT_CASES = {'GSLQR': ('gust_lateral_p10_VH',), 'CPID': ('gust_lateral_p10_VL', 'gust_vertical_m5_VL')}


def without_feedforward(config):
    """경기장 설정에서 참조 가속도 피드포워드만 뺀 사본(= 이번 결정 전의 기준선)."""
    off = deepcopy(config)
    for label in LABELS:
        off['controllers'][label].pop('reference_feedforward', None)
    return off


def scenario(config, factory, case_id):
    """본시험·튜닝 시나리오를 id로 찾아 만든다."""
    specs = [s for s in list(config['scenarios']) + list(config['tuning']['scenarios']) if s['id'] == case_id]
    if len(specs) != 1:
        raise ValueError(f'scenario {case_id!r} not found once in the arena config')
    return build_scenarios(config, factory.cp, factory.p, scenarios=specs)[0]


def run_case(config, factory, label, case_id, keep=False):
    """사례 하나(튜닝 목적함수·스모크와 같은 판정). keep이면 (지표, 궤적, 시나리오)를 돌려준다."""
    import control.validation_suite as suite
    from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
    s = scenario(config, factory, case_id)
    row, result, log = suite.run_trial(factory, label, s.profile, s.cases[0], Acceptance(**config['acceptance']))
    paper = paper_evaluate(result, s.profile, PaperCriteria(**config['paper_criteria']), window=s.window,
                           solve_log=log, n_max=factory.p['n_max'])
    integ = row.get('integrators') or {}
    metrics = dict(case=case_id, stop_reason=row['stop_reason'], simulated_seconds=row['simulated_seconds'],
                   T_total=float(s.profile.T_total), paper_failed=paper['paper_failed'],
                   paper_reasons=paper['paper_reasons'], window_rmse_velocity=paper['window_rmse_velocity'],
                   window_rmse_z=paper['window_rmse_z'], max_omega=row['max_omega'],
                   at_limit_fraction=integ.get('at_limit_fraction'), trajectory_sha256=row['trajectory_sha256'])
    return (metrics, result, s) if keep else metrics


def common_window(s, off, on):
    """두 실행이 **모두 살아 있던** 평가 구간의 RMSE(v는 3축 노름, z)와 궤적이 처음 갈라진 시각.

    정지 시각이 다르면 창 RMSE는 평가 구간 길이가 달라 직접 비교할 수 없다 — 늦게 멈춘 쪽이 발산
    중인 구간을 더 포함한다(2026-09-26 GSLQR 적분 한계 대조에서 본 착시와 같다).
    """
    n = min(len(off['ts']), len(on['ts']))
    ts = off['ts'][:n]
    differ = np.any(off['xs'][:n] != on['xs'][:n], axis=1)
    mask = (ts >= s.window[0]) & (ts <= s.window[1])
    out = dict(end_s=float(ts[mask][-1]) if mask.any() else None,
               first_difference_t=float(ts[int(np.argmax(differ))]) if differ.any() else None)
    if not mask.any():
        out['off'] = out['on'] = dict(rmse_v=None, rmse_z=None)
        return out
    vr, zr = s.profile.compute_refs(ts[mask])
    for name, r in (('off', off), ('on', on)):
        xs = r['xs'][:n][mask]
        out[name] = dict(rmse_v=float(np.sqrt(np.mean(np.sum((xs[:, 3:6] - vr)**2, axis=1)))),
                         rmse_z=float(np.sqrt(np.mean((xs[:, 2] - zr)**2))))
    # 구간별(공통 구간 안): 피드포워드가 작동하는 가속 구간의 변화와, 발산 꼬리(감속 등)의 변화를 가른다
    out['phases'] = []
    for phase, t0, t1 in s.profile.get_phase_boundaries():
        sel = mask & (ts >= t0) & (ts < t1)
        if not sel.any():
            continue
        vr, zr = s.profile.compute_refs(ts[sel])
        entry = dict(phase=phase, start_s=float(ts[sel][0]), end_s=float(ts[sel][-1]))
        for name, r in (('off', off), ('on', on)):
            xs = r['xs'][:n][sel]
            entry[name] = dict(rmse_v=float(np.sqrt(np.mean(np.sum((xs[:, 3:6] - vr)**2, axis=1)))),
                               rmse_z=float(np.sqrt(np.mean((xs[:, 2] - zr)**2))),
                               max_z_error=float(np.max(np.abs(xs[:, 2] - zr))))
        out['phases'].append(entry)
    return out


def feedforward_saturation(config, factory, case_id, t_end):
    """참조 가속도(창과 같은 전진 차분)가 GSLQR 유효 범위를 넘은 시간 비율 — 실행 구간 [0, t_end)."""
    step = float(config['nmpc_common']['dt_pred_s'])
    table = np.array(factory._gslqr_prototype().feedforward_limits())
    V_tab, lower, upper = table[:, 0], table[:, 1], table[:, 2]
    profile = scenario(config, factory, case_id).profile
    ts = np.arange(0.0, t_end, factory.dt)
    active = saturated = 0
    peak = 0.0
    for t in ts:
        v = profile.get_ref(t)[0][0]
        a = (profile.get_ref(t + step)[0][0] - v)/step
        if a == 0.0:
            continue
        active += 1
        peak = max(peak, abs(a))
        V = np.clip(v, V_tab[0], V_tab[-1])
        saturated += a > np.interp(V, V_tab, upper) or a < -np.interp(V, V_tab, lower)
    n = max(len(ts), 1)
    return dict(active_fraction=active/n, saturated_fraction=saturated/n, peak_a_ref=peak)


def design_points(config, factory, shape):
    import control.arena_design_check as dc
    rows = []
    for label in LABELS:
        for V in dc.POINTS[label]:
            for step_name in dc.STEPS:
                m = dc.run(config, factory, label, V, step_name, shape=shape)
                rows.append({k: m[k] for k in ('controller', 'speed', 'step', 'settling_s', 'overshoot_pct',
                                               'steady_error', 'cross_coupling_max', 'max_omega', 'stop_reason',
                                               'trajectory_sha256')})
    return rows


def compare(config, sections=('accel', 'constant', 'design')):
    """끔/켬 두 팩토리로 같은 사례를 돌린다. 제어기 모델은 하나를 같이 쓴다(같은 명목 모델)."""
    off_config = without_feedforward(config)
    on = ArenaFactory(config)
    off = ArenaFactory(off_config, on.p, model=on.model)
    out = dict(validity=[dict(V=V, brake=lo, accel=hi) for V, lo, hi in on._gslqr_prototype().feedforward_limits()])
    if 'accel' in sections:
        out['accel'] = []
        for label in LABELS:
            for case_id in ACCEL_CASES[label]:
                before, traj_off, s = run_case(off_config, off, label, case_id, keep=True)
                after, traj_on, _ = run_case(config, on, label, case_id, keep=True)
                row = dict(controller=label, case=case_id, off=before, on=after,
                           common=common_window(s, traj_off, traj_on))
                del traj_off, traj_on
                if label == 'GSLQR':
                    row['saturation'] = feedforward_saturation(config, on, case_id, after['simulated_seconds'])
                out['accel'].append(row)
                print(_line(row), flush=True)
    if 'constant' in sections:
        out['constant'] = []
        for label in LABELS:
            for case_id in CONSTANT_CASES[label]:
                before = run_case(off_config, off, label, case_id)
                after = run_case(config, on, label, case_id)
                out['constant'].append(dict(controller=label, case=case_id, off=before, on=after,
                                            identical=before['trajectory_sha256'] == after['trajectory_sha256']))
                print(label, case_id, 'identical', out['constant'][-1]['identical'], flush=True)
    if 'design' in sections:
        # 매끄러운 계단(지금 근거, kj 결정 2026-09-26 오후)과 원계단(옛 기록) 둘 다 끔/켬을 대조한다
        for key, shape in (('design', 'smooth'), ('design_step', 'step')):
            before, after = design_points(off_config, off, shape), design_points(config, on, shape)
            out[key] = [dict(off=b, on=a, identical=b['trajectory_sha256'] == a['trajectory_sha256'])
                        for b, a in zip(before, after)]
    return out


def _line(row):
    b, a = row['off'], row['on']
    return (f"{row['controller']:5s} {row['case']:24s} off v {b['window_rmse_velocity']:.4g} z {b['window_rmse_z']:.4g} "
            f"t {b['simulated_seconds']:.2f} |w| {b['max_omega']:.3g} -> on v {a['window_rmse_velocity']:.4g} "
            f"z {a['window_rmse_z']:.4g} t {a['simulated_seconds']:.2f} |w| {a['max_omega']:.3g}")


def _fmt(x, spec='.4g'):
    return '—' if x is None else format(x, spec)


def _change(b, a):
    if b is None or a is None or b == 0:
        return '—'
    return f'{100*(a - b)/abs(b):+.0f}%'


def write_markdown(path, data):
    m = data['meta']
    lines = ['# 참조 가속도 피드포워드 전후 대조(PILOT)', '',
             '**PILOT** — kj 결정 2(2026-09-26 오후): CPID(PX4식)·GSLQR(표준 추종 LQR, 선형 유효 범위 포화)에 '
             '참조 가속도 피드포워드. **결과를 본 뒤·기준선 유리** 변경이다. 우위 결론 없음.', '',
             f'재현: `{m["command"]}` · 설정 sha256 `{m["config_sha256"][:12]}` · git `{m["git_revision"]}` '
             f'dirty={m["git_dirty"]}', '',
             '끔 = `configs/arena.json`에서 `reference_feedforward`만 뺀 설정. 창 RMSE는 논문 §5.8 평가 창이다. '
             '정지 시각이 다르면 평가 구간 길이가 달라 RMSE를 직접 비교할 수 없다(시간 열을 같이 볼 것).', '']
    if 'accel' in data:
        lines += ['## 1. 가속 사례', '',
                  '| 제어기 | 사례 | 끔: 정지·시간 s | 켬: 정지·시간 s | 창 RMSE v 끔 → 켬 | 창 RMSE z 끔 → 켬 | '
                  '공통 구간 RMSE v 끔 → 켬 | 공통 구간 RMSE z 끔 → 켬 | 처음 갈라진 시각 s | '
                  '\\|ω\\|max 끔 → 켬 | 논문 실패 끔 → 켬 | 참조 가속 최대 m/s² | 유효 범위 넘은 시간 % |',
                  '|---|---|---|---|---|---|---|---|---:|---|---|---:|---:|']
        for r in data['accel']:
            b, a, c = r['off'], r['on'], r['common']
            sat = r.get('saturation')
            lines.append(
                f"| {r['controller']} | {r['case']} | {b['stop_reason'] or '끝까지'} · {b['simulated_seconds']:.2f} | "
                f"{a['stop_reason'] or '끝까지'} · {a['simulated_seconds']:.2f} | "
                f"{_fmt(b['window_rmse_velocity'])} → {_fmt(a['window_rmse_velocity'])} "
                f"({_change(b['window_rmse_velocity'], a['window_rmse_velocity'])}) | "
                f"{_fmt(b['window_rmse_z'])} → {_fmt(a['window_rmse_z'])} "
                f"({_change(b['window_rmse_z'], a['window_rmse_z'])}) | "
                f"{_fmt(c['off']['rmse_v'])} → {_fmt(c['on']['rmse_v'])} "
                f"({_change(c['off']['rmse_v'], c['on']['rmse_v'])}, ~{_fmt(c['end_s'], '.2f')} s) | "
                f"{_fmt(c['off']['rmse_z'])} → {_fmt(c['on']['rmse_z'])} "
                f"({_change(c['off']['rmse_z'], c['on']['rmse_z'])}) | {_fmt(c['first_difference_t'], '.3f')} | "
                f"{b['max_omega']:.3g} → {a['max_omega']:.3g} | {b['paper_failed']} → {a['paper_failed']} | "
                f"{'—' if sat is None else format(sat['peak_a_ref'], '.3g')} | "
                f"{'—' if sat is None else format(100*sat['saturated_fraction'], '.1f')} |")
        lines += ['', '공통 구간 = 평가 창 안에서 두 실행이 모두 살아 있던 구간(끝 시각을 함께 적음). 정지 시각이 '
                  '다르면 창 RMSE는 구간 길이가 달라 직접 비교할 수 없어 이 열로 본다.', '',
                  '### 1-1. 구간별(공통 구간 안) — 변화가 어느 비행 구간에서 났는가', '',
                  '| 제어기 | 사례 | 구간 | 시각 s | RMSE v 끔 → 켬 | RMSE z 끔 → 켬 | 최대 고도오차 끔 → 켬 |',
                  '|---|---|---|---|---|---|---|']
        for r in data['accel']:
            for p in r['common'].get('phases', []):
                b, a = p['off'], p['on']
                lines.append(f"| {r['controller']} | {r['case']} | {p['phase']} | {p['start_s']:.1f}–{p['end_s']:.1f} | "
                             f"{_fmt(b['rmse_v'])} → {_fmt(a['rmse_v'])} | {_fmt(b['rmse_z'])} → {_fmt(a['rmse_z'])} | "
                             f"{_fmt(b['max_z_error'])} → {_fmt(a['max_z_error'])} |")
        lines += ['',
                  '유효 범위 넘은 시간 % = 참조 가속도가 GSLQR 선형 피드포워드의 유효 범위(4절)를 넘어 포화된 '
                  '스텝의 비율(실행 구간 기준). 이 몫의 차이는 선형 가정의 한계다(참조 정보는 다 받았다).', '']
    if 'constant' in data:
        lines += ['## 2. 참조가 일정한 사례 — 끔/켬 비트 동일이어야 한다', '',
                  '| 제어기 | 사례 | 비트 동일 | 정지·시간 s | \\|ω\\|max |', '|---|---|---|---|---:|']
        for r in data['constant']:
            a = r['on']
            lines.append(f"| {r['controller']} | {r['case']} | {r['identical']} | "
                         f"{a['stop_reason'] or '끝까지'} · {a['simulated_seconds']:.2f} | {a['max_omega']:.3g} |")
        lines.append('')
    notes = {'design': ('3. 설계점검 — 매끄러운 계단(지금 근거) 끔 → 켬',
                        '모든 제어기에 같은 매끄러운 계단(논문 식(32) smoothstep, 1 s 전이 — kj 결정 2026-09-26 오후). '
                        'PX4가 설정값을 궤적 생성기로 매끄럽게 만든 뒤 가속도를 피드포워드하는 구조와 같다. '
                        '정착시간은 계단 시작부터 재서 전이 1 s가 들어 있다.'),
             'design_step': ('3-1. 설계점검 — 원계단(옛 기록) 끔 → 켬',
                             '원계단을 차분하면 계단 직전 0.05 s 동안 1 m/s ÷ 0.05 s = 20 m/s²의 참조 가속 펄스가 된다'
                             '(GSLQR은 유효 범위에서 포화). 기준선의 루프가 아니라 그 펄스를 재게 되어 설계점검을 '
                             '매끄러운 계단으로 바꿨다. 기록으로만 남긴다.')}
    for key in ('design', 'design_step'):
        if key not in data:
            continue
        title, note = notes[key]
        lines += [f'## {title}', '', note, '',
                  '| 제어기 | 속도 | 계단 | 동일 | 정착 s 끔 → 켬 | 오버슈트 % 끔 → 켬 | 정상오차 끔 → 켬 | '
                  '\\|ω\\|max 끔 → 켬 | 정지(켬) |', '|---|---:|---|---|---|---|---|---|---|']
        for r in data[key]:
            b, a = r['off'], r['on']
            lines.append(f"| {a['controller']} | {a['speed']:g} | {a['step']} | {r['identical']} | "
                         f"{_fmt(b['settling_s'], '.2f')} → {_fmt(a['settling_s'], '.2f')} | "
                         f"{_fmt(b['overshoot_pct'], '.1f')} → {_fmt(a['overshoot_pct'], '.1f')} | "
                         f"{_fmt(b['steady_error'], '.3g')} → {_fmt(a['steady_error'], '.3g')} | "
                         f"{b['max_omega']:.3g} → {a['max_omega']:.3g} | {a['stop_reason'] or '—'} |")
        settled = sum(r['on']['settling_s'] is not None for r in data[key])
        lines += ['', f'켬에서 정착한 점: {settled}/{len(data[key])}', '']
    lines += ['## 4. GSLQR 선형 피드포워드 유효 범위(명목 모델만, 규칙은 실행 전에 적음)', '',
              '선형 목표점(트림 자세 ⊗ δφ, ω = 0, n = u = clip(u_trim + δn))에서 명목 모델의 가속도가 '
              '|v̇_x − a| ≤ 0.1|a|, |v̇_z| ≤ 0.1|a|, |ω̇| ≤ 2 rad/s²를 0부터 이어서 만족하는 가장 큰 |a|.', '',
              '| V m/s | 감속 한계 m/s² | 가속 한계 m/s² |', '|---:|---:|---:|']
    lines += [f"| {r['V']:g} | {r['brake']:.1f} | {r['accel']:.1f} |" for r in data['validity']]
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    from control.validation_suite import write_json, environment_fingerprint, git_state
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    parser.add_argument('--sections', nargs='+', choices=('accel', 'constant', 'design'),
                        default=['accel', 'constant', 'design'])
    args = parser.parse_args(argv)
    config = load_config(args.config)
    data = compare(config, tuple(args.sections))
    revision, dirty = git_state()
    command = 'python -m control.arena_feedforward_check' + (
        '' if len(args.sections) == 3 else ' --sections ' + ' '.join(args.sections))
    data['meta'] = dict(label='PILOT', command=command, created_utc=datetime.now(timezone.utc).isoformat(),
                        config_sha256=config_sha256(config), git_revision=revision, git_dirty=dirty,
                        environment=environment_fingerprint())
    args.out.mkdir(parents=True, exist_ok=True)
    write_json(args.out/'feedforward.json', data)
    write_markdown(args.out/'FEEDFORWARD.md', data)
    print((args.out/'FEEDFORWARD.md').read_text(encoding='utf-8'))
    return data


if __name__ == '__main__':
    main()
