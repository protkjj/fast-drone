"""제어기 모델 수정 전후 PILOT 비교 — kj 결정(2026-09-25): "바뀌는 PILOT 수치는 이전 값과 함께 기록한다".

**PILOT** — 수정이 예비 수치를 얼마나 바꿨는지 보는 기록이다. 우위·열위 결론은 내리지 않는다.

이전 값은 results/SOLVER_FAILURE_REPORT_2026-09-25.md(작업 A) 표를 그대로 옮겨 적었다.
그때 제어기 모델은 선형 프로펠러(1−J/J_max)와 상수 압력중심이었다. 이후 값은 현재 코드로
같은 시나리오를 다시 돌려 얻는다. 둘 사이에 바뀐 것은 다음과 같다.
  - 프로펠러: 팀원 APC PCHIP 곡선(cb08eee, 병합) — 선형식은 85 m/s 트림 추력을 57% 과소평가했다
  - 압력중심: x_cp(V) 3차 스케줄(kh_adapter.fit_cp_schedule, I-10)
  - 호버 규약 hover_quat(V13 예측에는 영향 없음, 트림 계산에만 쓰임)
판별 실험(discriminate)은 둘을 가르려고 '프로펠러만'(x_cp 상수) 변형도 함께 돌린다.

작업 A의 판별("H-시나리오 지지")은 선형 프로펠러 모델로 얻었다. 그때 V13의 91% 포화가 모델
결함 탓이었다면 전제(짧은 시험 실패)부터 달라지므로, kj의 판정 규칙을 새 수치에 다시 적용한다:
  기본·트림 웜스타트 둘 다 실패 → H-시나리오 지지 유지
  기본은 실패, 트림 웜스타트는 통과 → H-웜스타트이력 지지
  둘 다 통과 → 판별의 전제(짧은 시험 실패)가 사라짐 — 이전 실패는 모델 결함이었을 가능성

실행(오래 걸린다 — timing 부분만 45분 이상, 백그라운드 권장):
  caffeinate -dims python3 -m control.pilot_before_after > results/arena/pilot_before_after.log 2>&1 &
  python3 -m control.pilot_before_after --parts discriminate      # 일부만
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from control.arena import ROOT

LABEL = 'PILOT'
SOURCE = 'results/SOLVER_FAILURE_REPORT_2026-09-25.md'
PARTS = ('discriminate', 'mission', 'timing')


def _row(completed, z, v, sat, fails, calls, stop=None, omega=None, seconds=None):
    return dict(completed=completed, z_rmse_m=z, vel_rmse_m_s=v, saturation_fraction=sat,
                optimizer_failures=fails, optimizer_calls=calls, stop_reason=stop,
                max_omega=omega, elapsed_sim_s=seconds)


# 이전 값(작업 A 보고서의 표 그대로). max_omega가 None인 곳은 원표에 |ω|가 없었다.
BEFORE = {
    # §7 판별 실험(쿼터니언 eps 정칙화 적용 뒤, 선형 프로펠러), 짧은 시험 nominal 4초
    'discriminate': {
        'default@80': _row(False, 0.6974, 13.120, 0.914, 2, 50, 'velocity error limit'),
        'trim@80': _row(False, 0.6975, 13.119, 0.912, 2, 50, 'velocity error limit'),
        'default@85': _row(False, 0.6945, 14.345, 0.912, 3, 51, 'velocity error limit'),
        'trim@85': _row(False, 0.6982, 14.355, 0.915, 2, 51, 'velocity error limit'),
    },
    # §3.3 65초 통합 미션(35 s 수직 돌풍 10 m/s), 순항 70/80/85
    'mission': {
        'V13@70': _row(True, 0.524, 2.869, 0.065, 2, 3250, omega=11.15, seconds=65.0),
        'V13@80': _row(True, 1.307, 3.538, 0.075, 2, 3250, omega=16.21, seconds=65.0),
        'V13@85': _row(True, 0.833, 3.267, 0.081, 2, 3250, omega=12.58, seconds=65.0),
    },
    # §3.4 가감속 원래(15/15/15) vs 완화(8/3/25), 돌풍 없음
    'timing': {
        'original@70': _row(True, 0.360, 1.852, 0.077, 2, 2750, omega=10.04, seconds=55.0),
        'relaxed@70': _row(True, 0.136, 1.275, 0.086, 2, 2000, omega=12.07, seconds=40.0),
        'original@80': _row(True, 0.217, 1.771, 0.083, 4, 2750, omega=15.97, seconds=55.0),
        'relaxed@80': _row(True, 0.414, 1.841, 0.127, 5, 2000, omega=15.42, seconds=40.0),
        'original@85': _row(True, 2.374, 4.370, 0.146, 15, 2750, omega=14.62, seconds=55.0),
        'relaxed@85': _row(True, 0.642, 3.198, 0.138, 3, 2000, omega=14.70, seconds=40.0),
    },
}

KEEP = ('completed', 'stop_reason', 'elapsed_sim_s', 'z_rmse_m', 'vel_rmse_m_s', 'max_omega',
        'saturation_fraction', 'optimizer_failures', 'optimizer_calls',
        'prop_domain_outside_fraction')


def _keep(result):
    return {k: result.get(k) for k in KEEP}


def run_discriminate(native):
    """§7 판별 실험 재실행 — 현재 모델과 '프로펠러만'(x_cp 상수) 두 제어기 모델로."""
    from control.kh_adapter import build_controller_params
    from control.kh_repro import run_case, make_our_split, make_our_split_trim_warmstart
    out = {}
    for model_name, cp in (('current', build_controller_params(native)),
                           ('propeller_only', build_controller_params(native, cp_schedule=False))):
        for V in (80.0, 85.0):
            out[f'{model_name}/default@{V:g}'] = _keep(run_case(
                native, lambda s, z, cp=cp: make_our_split(cp, s, z), f'V13 {model_name} 기본', V,
                'nominal'))
            out[f'{model_name}/trim@{V:g}'] = _keep(run_case(
                native, lambda s, z, cp=cp: make_our_split_trim_warmstart(cp, native, s, z),
                f'V13 {model_name} 트림', V, 'nominal'))
    return out


def run_mission(native):
    """§3.3 65초 통합 미션(팀원 프로파일·35 s 돌풍 그대로), 우리 V13, 순항 70/80/85."""
    from control.kh_adapter import build_controller_params
    from control.kh_mission_repro import run_mission as mission, make_our_v13_mission
    from models.team_light.control.mission_sim import MissionProfile
    cp = build_controller_params(native)
    return {f'V13@{V:g}': _keep(mission(native, lambda profile: make_our_v13_mission(cp, profile),
                                        f'V13 {V:g}m/s', profile=MissionProfile(V, 50.0)))
            for V in (70.0, 80.0, 85.0)}


def run_timing(native):
    """§3.4 가감속 원래 vs 완화, 돌풍 없음."""
    from control.kh_adapter import build_controller_params
    from control.kh_mission_repro import run_mission as mission, make_our_v13_mission
    from control.kh_timing_severity_compare import original_profile, relaxed_profile
    cp = build_controller_params(native)
    out = {}
    for V in (70.0, 80.0, 85.0):
        for name, builder in (('original', original_profile), ('relaxed', relaxed_profile)):
            out[f'{name}@{V:g}'] = _keep(mission(
                native, lambda profile: make_our_v13_mission(cp, profile), f'V13 {V:g} {name}',
                profile=builder(V), gust_fn=lambda t: np.zeros(3)))
    return out


def rejudge(after):
    """kj 판별 규칙을 새 수치(현재 모델)에 다시 적용한다."""
    verdicts = {}
    for V in ('80', '85'):
        default = after.get(f'current/default@{V}')
        trim = after.get(f'current/trim@{V}')
        if default is None or trim is None:
            continue
        if not default['completed'] and not trim['completed']:
            verdicts[V] = 'H-시나리오 지지 유지(기본·트림 웜스타트 둘 다 실패)'
        elif not default['completed'] and trim['completed']:
            verdicts[V] = 'H-웜스타트이력 지지(트림 웜스타트만 통과)'
        elif default['completed'] and trim['completed']:
            verdicts[V] = '판별 전제 소멸(둘 다 완주) — 이전 실패는 모델 결함이었을 가능성'
        else:
            verdicts[V] = '예상 밖(기본은 완주, 트림 웜스타트는 실패) — 따로 조사 필요'
    return verdicts


def _fmt(v, spec):
    return '—' if v is None else format(v, spec)


def write_markdown(path, data):
    lines = [f'# PILOT 전후 비교 — 제어기 모델 수정(프로펠러 곡선·x_cp(V)) 전후', '',
             '**PILOT** — 예비 수치의 변화 기록. 우위·열위 결론 없음.', '',
             f'이전 값 출처: `{SOURCE}`(선형 프로펠러·상수 x_cp 시절). '
             f'재현: `{data["meta"]["command"]}` · git `{data["meta"]["git_revision"]}` '
             f'dirty={data["meta"]["git_dirty"]}', '']
    header = ('| 사례 | 시점 | 완주 | 시간 s | z RMSE | v RMSE | |ω|max | 포화 % | 솔버실패 | 정지사유 |',
              '|---|---|---|---:|---:|---:|---:|---:|---:|---|')
    for part in data['parts']:
        lines += [f'## {part}', '', *header]
        after = data['after'][part]
        keys = sorted(set(BEFORE[part]) | {k.split('/', 1)[-1] for k in after})
        for key in keys:
            rows = [('이전', BEFORE[part].get(key))]
            rows += [(name, after.get(f'{name}/{key}') if part == 'discriminate' else after.get(key))
                     for name in (('current', 'propeller_only') if part == 'discriminate' else ('이후',))]
            for when, r in rows:
                if r is None:
                    continue
                fails = (f'{r["optimizer_failures"]}/{r["optimizer_calls"]}'
                         if r.get('optimizer_calls') is not None else '—')
                sat = r.get('saturation_fraction')
                lines.append(f'| {key} | {when} | {r["completed"]} | {_fmt(r.get("elapsed_sim_s"), ".2f")} | '
                             f'{_fmt(r.get("z_rmse_m"), ".4g")} | {_fmt(r.get("vel_rmse_m_s"), ".4g")} | '
                             f'{_fmt(r.get("max_omega"), ".3g")} | '
                             f'{_fmt(None if sat is None else 100*sat, ".1f")} | {fails} | '
                             f'{r.get("stop_reason") or ""} |')
        lines.append('')
    if 'verdicts' in data:
        lines += ['## 작업 A 판별 규칙 재적용(현재 모델)', '']
        lines += [f'- {V} m/s: {text}' for V, text in data['verdicts'].items()] or ['- (판별 부분 미실행)']
        lines.append('')
    Path(path).write_text('\n'.join(lines), encoding='utf-8')


def main(argv=None):
    from control.kh_adapter import kh_native_params
    from control.validation_suite import write_json, environment_fingerprint, git_state
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--parts', nargs='+', choices=PARTS, default=list(PARTS))
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    args = parser.parse_args(argv)
    native = kh_native_params()
    runners = dict(discriminate=run_discriminate, mission=run_mission, timing=run_timing)
    revision, dirty = git_state()
    data = dict(meta=dict(label=LABEL, source=SOURCE,
                          command='python -m control.pilot_before_after --parts ' + ' '.join(args.parts),
                          created_utc=datetime.now(timezone.utc).isoformat(),
                          git_revision=revision, git_dirty=dirty,
                          environment=environment_fingerprint()),
                parts=list(args.parts), before={p: BEFORE[p] for p in args.parts}, after={})
    args.out.mkdir(parents=True, exist_ok=True)
    for part in args.parts:
        print(f'== {LABEL} {part} ==', flush=True)
        data['after'][part] = runners[part](native)
        if part == 'discriminate':
            data['verdicts'] = rejudge(data['after'][part])
        write_json(args.out/'pilot_before_after.json', data)        # 부분마다 저장(중단 대비)
        write_markdown(args.out/'PILOT_BEFORE_AFTER.md', data)
    print(json.dumps(data.get('verdicts', {}), ensure_ascii=False, indent=1))
    return data


if __name__ == '__main__':
    main()
