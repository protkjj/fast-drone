"""적분기 한계 점검 — kj 요청(2026-09-26). 결과는 **PILOT**(진단, 비교 결론 없음).

kj 질문 두 가지에 답하는 수치를 한 곳에서 재현한다.
  "CPID 적분 한계 수정이 구조적 버그 수정인지 성능 보정인지"
  "GSLQR의 LQI 적분기에도 같은 기준의 한계가 걸려 있는지"

  (1) GSLQR 권한   운용점마다 LQI 증강 폐루프의 정상상태를 풀어 '단위 외란 가속도(1 m/s²)를
                   지우는 데 필요한 적분상태 ξ'를 구한다. 한계(±integral_limit)가 허용하는
                   외란 가속도 = 한계 / max|ξ|. 한계는 상태 단위(ξ_z m·s, ξ_vx m)다.
  (2) CPID 필요량   명목 모델 트림을 유지하려면 적분기가 떠맡아야 하는 가속도
                   (ControllerModel.trim_acceleration: 수평 = 항력 몫, 수직 = 양력 몫) 대 권한.
  (3) --binding    설계점검을 한계 ∞로 다시 돌려 궤적 해시를 비교한다. 같으면 한계값이 결과를
                   만들지 않았다는 뜻이다(성능 손잡이가 아님).
  (4) --history    CPID 수정 경과를 같은 설계점검으로 재현한다.
                   P만 → 속도 적분 + 상태 한계 5 m(= 0.75 m/s²) → 가속도 권한 5 m/s²
                   → 현재(±1 g 두 적분기, 조건부, 출발값 사전 채움)
  (5) --hold       CPID 20 m/s 60초 유지. 고도 적분 기존 방식(상태 한계 → 2.5 m/s², 무조건
                   적분) 대 현재(±1 g, 조건부).

실행: python -m control.arena_integrators [--binding] [--history] [--hold]
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from control.arena import load_config, config_sha256, DEFAULT_CONFIG, ROOT
from control.arena_factory import ArenaFactory
import control.arena_design_check as dc

CPID_NEED_SPEEDS = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0)
UNLIMITED = 1e9            # '한계 없음' 대용 — 도달할 수 없는 크기


def gslqr_authority(factory):
    """운용점별 LQI 정상상태: 단위 외란 가속도당 ξ, 한계가 허용하는 외란 가속도."""
    from control.controller import linearize_error_state
    lqr = factory._gslqr_prototype()
    if tuple(lqr.integral_states) != (0, 1):
        raise ValueError('this check assumes integral_states (0, 1) = (dz, dvx)')
    rows = []
    for k, V in enumerate(lqr.V_table):
        A_r, B_r, _, _ = linearize_error_state(factory.cp, lqr._x_trim_arr[k], lqr._u_trim_arr[k])
        K_r = lqr._K_r_flat[k].reshape(4, 14)
        K_i = lqr._K_i_flat[k].reshape(4, 2)
        C = np.zeros((2, 14))
        C[0, 0] = C[1, 1] = 1.0
        # 증강 폐루프 [δx_r; ξ]' = A_aug [δx_r; ξ] + E d. 정상상태에서 0 = A_aug s + E d.
        A_aug = np.block([[A_r - B_r @ K_r, -B_r @ K_i], [C, np.zeros((2, 2))]])
        row = dict(speed=float(V), max_real_eig=float(np.max(np.real(np.linalg.eigvals(A_aug)))))
        for axis, idx in (('x', 1), ('z', 3)):          # 오차상태 δvx·δvz 행에 외란 가속도
            E = np.zeros(16)
            E[idx] = 1.0
            xi = np.linalg.solve(A_aug, -E)[14:16]
            row[f'xi_per_unit_d{axis}'] = xi.tolist()
            row[f'authority_{axis}'] = float(lqr.integral_limit/np.max(np.abs(xi)))
        rows.append(row)
    return rows


def cpid_trim_need(factory, speeds=CPID_NEED_SPEEDS):
    g = factory.gains['CPID']
    az = g.get('a_int_z_max')
    authority = dict(horizontal=float(g['a_int_max']),
                     vertical=float(az) if az is not None else float(g['Ki_z'])*float(g['int_z_max']))
    rows = []
    for V in speeds:
        a = factory.model.trim_acceleration(V)
        rows.append(dict(speed=float(V), a_x=float(a[0]), a_z=float(a[2]),
                         tilt_deg=float(np.degrees(np.arctan2(np.hypot(a[0], a[1]), a[2] + factory.cp['g']))),
                         within=bool(abs(a[0]) <= authority['horizontal']
                                     and abs(a[2]) <= authority['vertical'])))
    return authority, rows


def _design_rows(config, factory, label):
    return [dc.run(config, factory, label, V, step) for V in dc.POINTS[label] for step in dc.STEPS]


def binding(config, model):
    """한계를 ∞로 바꿔도 설계점검 궤적이 비트 단위로 같은가."""
    out = {}
    for label, over in (('GSLQR', {'integral_limit': UNLIMITED}),
                        ('CPID', {'a_int_max': UNLIMITED, 'a_int_z_max': UNLIMITED})):
        base = _design_rows(config, ArenaFactory(config, model=model), label)
        free = _design_rows(config, ArenaFactory(config, overrides={label: over}, model=model), label)
        out[label] = [dict(speed=a['speed'], step=a['step'], identical=a['trajectory_sha256'] == b['trajectory_sha256'],
                           at_limit_fraction=(a.get('integrators') or {}).get('at_limit_fraction'))
                      for a, b in zip(base, free)]
    return out


CPID_STAGES = (
    ('P only', {'Ki_vel': 0.0, 'a_int_z_max': None}, False),
    ('vel integral, state limit 5 m (=0.75 m/s^2)', {'a_int_max': 0.75, 'a_int_z_max': None}, False),
    ('vel integral, authority 5 m/s^2', {'a_int_max': 5.0, 'a_int_z_max': None}, False),
    ('current: +-1 g both, conditional, preload', {}, True),
)


def _config_with_preload(config, preload):
    cfg = deepcopy(config)
    if not preload:
        cfg['controllers']['CPID'].pop('integrator_preload', None)
    return cfg


def history(config, model):
    out = []
    for name, over, preload in CPID_STAGES:
        cfg = _config_with_preload(config, preload)
        fac = ArenaFactory(cfg, overrides={'CPID': over} if over else None, model=model)
        for r in _design_rows(cfg, fac, 'CPID'):
            out.append(dict(stage=name, speed=r['speed'], step=r['step'], settling_s=r['settling_s'],
                            overshoot_pct=r['overshoot_pct'], steady_error=r['steady_error'],
                            cross_coupling_max=r['cross_coupling_max'], max_omega=r['max_omega'],
                            at_limit_fraction=(r.get('integrators') or {}).get('at_limit_fraction')))
    return out


def hold(config, model, speed=20.0, seconds=60.0):
    """CPID 60초 트림 유지 — 고도 적분 기존 방식 대 현재. 끝 1초 평균 고도오차."""
    import control.validation_suite as suite
    from control.validation_metrics import Acceptance
    out = []
    for name, over in (('altitude integral legacy (state limit, unconditional)', {'a_int_z_max': None}),
                       ('current (+-1 g, conditional)', {})):
        fac = ArenaFactory(config, overrides={'CPID': over} if over else None, model=model)
        profile = dc.StepProfile(speed, float(config['altitude_m']), 0.0, 0.0, duration=seconds)
        row, result, _ = suite.run_trial(fac, 'CPID', profile, dict(case_id='hold', factors={}), Acceptance())
        ts, xs = result['ts'], result['xs']
        tail = ts >= ts[-1] - 1.0
        out.append(dict(variant=name, speed=speed, seconds=seconds,
                        final_z_error=float(np.mean(xs[tail, 2] - float(config['altitude_m']))),
                        final_speed_error=float(np.mean(xs[tail, 3] - speed)),
                        max_omega=row['max_omega'], integrators=row.get('integrators'),
                        trajectory_sha256=row['trajectory_sha256']))
    return out


def write_markdown(path, data):
    m = data['meta']
    lines = ['# 적분기 한계 점검 — PILOT', '',
             '**PILOT** — 진단. 비교 결론 없음. 재현: `' + m['command'] + '` · 설정 sha256 `'
             + m['config_sha256'][:12] + '` · git `' + str(m['git_revision']) + '` dirty=' + str(m['git_dirty']), '',
             '## GSLQR LQI 한계의 가속도 환산 권한', '',
             f'한계는 적분 **상태** ±{data["gslqr_limit"]:g}(ξ_z m·s, ξ_vx m). 권한 = 한계 / 단위 외란당 |ξ|.', '',
             '| 속도 m/s | 권한 x m/s² | 권한 z m/s² | 증강 폐루프 max Re |', '|---:|---:|---:|---:|']
    lines += [f'| {r["speed"]:g} | {r["authority_x"]:.3g} | {r["authority_z"]:.3g} | {r["max_real_eig"]:.3f} |'
              for r in data['gslqr_authority']]
    auth = data['cpid_authority']
    lines += ['', '## CPID 트림 유지에 적분기가 떠맡는 가속도(명목 모델 트림)', '',
              f'권한: 수평 {auth["horizontal"]:g}, 수직 {auth["vertical"]:g} m/s²', '',
              '| 속도 m/s | 수평 m/s² | 수직 m/s² | 추력 기울기 ° | 권한 안 |', '|---:|---:|---:|---:|---|']
    lines += [f'| {r["speed"]:g} | {r["a_x"]:+.3f} | {r["a_z"]:+.3f} | {r["tilt_deg"]:.2f} | {r["within"]} |'
              for r in data['cpid_trim_need']]
    if 'binding' in data:
        lines += ['', '## 한계를 ∞로 바꿔도 같은가(설계점검 궤적 해시)', '']
        for label, rows in data['binding'].items():
            same = sum(r['identical'] for r in rows)
            lines.append(f'- {label}: {same}/{len(rows)} 비트 동일, 한계 도달 비율 최대 '
                         f'{max((r["at_limit_fraction"] or 0.0) for r in rows):.4f}')
    if 'history' in data:
        lines += ['', '## CPID 수정 경과(설계점검)', '',
                  '| 단계 | 속도 | 계단 | 정착 s | 오버슈트 % | 정상오차 | 교차결합 | \\|ω\\|max | 한계 도달 |',
                  '|---|---:|---|---:|---:|---:|---:|---:|---:|']
        for r in data['history']:
            settle = '미정착' if r['settling_s'] is None else f'{r["settling_s"]:.2f}'
            lim = '—' if r['at_limit_fraction'] is None else f'{r["at_limit_fraction"]:.3f}'
            lines.append(f'| {r["stage"]} | {r["speed"]:g} | {r["step"]} | {settle} | {r["overshoot_pct"]:.1f} | '
                         f'{r["steady_error"]:.3g} | {r["cross_coupling_max"]:.3g} | {r["max_omega"]:.3g} | {lim} |')
    if 'hold' in data:
        lines += ['', '## CPID 20 m/s 60초 유지(고도 적분 방식 비교)', '',
                  '| 방식 | 끝 1초 고도오차 m | 끝 1초 속도오차 m/s | \\|ω\\|max | 한계 도달 비율 |',
                  '|---|---:|---:|---:|---:|']
        for r in data['hold']:
            lim = (r['integrators'] or {}).get('at_limit_fraction')
            lines.append(f'| {r["variant"]} | {r["final_z_error"]:+.4f} | {r["final_speed_error"]:+.4f} | '
                         f'{r["max_omega"]:.3g} | {"—" if lim is None else f"{lim:.3f}"} |')
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    from control.validation_suite import write_json, environment_fingerprint, git_state
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    parser.add_argument('--binding', action='store_true', help='rerun the design check with limits removed')
    parser.add_argument('--history', action='store_true', help='CPID fix stages on the design check')
    parser.add_argument('--hold', action='store_true', help='CPID 60 s hold at 20 m/s, altitude integral variants')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    factory = ArenaFactory(config)
    revision, dirty = git_state()
    flags = [f for f in ('binding', 'history', 'hold') if getattr(args, f)]
    data = dict(meta=dict(label='PILOT', command=' '.join(['python -m control.arena_integrators']
                                                          + [f'--{f}' for f in flags]),
                          created_utc=datetime.now(timezone.utc).isoformat(),
                          config_sha256=config_sha256(config), git_revision=revision, git_dirty=dirty,
                          controller_model_sha256=factory.controller_model_sha256,
                          environment=environment_fingerprint()),
                gslqr_limit=float(factory.gains['GSLQR']['integral_limit']),
                gslqr_authority=gslqr_authority(factory))
    data['cpid_authority'], data['cpid_trim_need'] = cpid_trim_need(factory)
    if args.binding:
        data['binding'] = binding(config, factory.model)
    if args.history:
        data['history'] = history(config, factory.model)
    if args.hold:
        data['hold'] = hold(config, factory.model)
    args.out.mkdir(parents=True, exist_ok=True)
    write_json(args.out/'integrators.json', data)
    write_markdown(args.out/'INTEGRATORS.md', data)
    print((args.out/'INTEGRATORS.md').read_text(encoding='utf-8'))
    return data


if __name__ == '__main__':
    main()
