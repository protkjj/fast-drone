"""본 실험 캠페인 — `configs/main_experiment.json`을 묶음별 경기장 설정으로 전개하고, 계획을 보이고,
해시 가드를 통과할 때만 실행한다(kj 결정 2026-09-28, 보고서 21절).

    python3 -m control.main_experiment --plan [--speed-ratio 1.3]

실행(조각 배분·이어 돌리기·합치기·교차 확인)은 `control/main_distributed.py`가 한다(DISTRIBUTED_RUN.md B절).

설계
  - `configs/arena.json`(기준 설정)은 바꾸지 않는다. 묶음마다 기준 설정을 깊은 복사해 'scenarios'만
    바꾼 **파생 설정**을 만들고 `validate_config`를 통과시킨다. 사다리 변형은 `controllers.V13`의
    할당·C·S 세 항목만 바꾼다(팩토리 무수정).
  - 표 7 값은 설정에 **규칙**으로 적고(0.7 g × 질량 등) 여기서 명목 파라미터로 숫자를 만든다 —
    형상 값이 바뀌어도 숫자가 낡지 않는다. 만든 숫자는 manifest에 남긴다.
  - 가드: 기준 설정의 정규화 해시와 튜닝 기록(<제어기>.record.json) sha256을 대조한다. 비어 있거나
    다르면 실행 거부. 튜닝값은 기준 설정으로 한 번 계산해(`tuned_overrides`가 기록의 설정 해시를
    기준 설정과 대조한다) 모든 파생 설정의 팩토리에 똑같이 준다.
  - 기체 한계: 시나리오의 시작·끝 속도에서 섭동된 플랜트(`plant_truth`, run_trial과 같은 함수)의
    트림이 없으면 `vehicle_limit`으로 따로 적고 돌리지 않는다. CG 행은 6자유도 트림, 나머지는 벤더.
"""
import argparse
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from control.arena import (ROOT, load_config, config_sha256, validate_config, build_scenarios,
                           excluded_from, resolve_speed)

DEFAULT_SPEC = ROOT/'configs'/'main_experiment.json'
SCHEMA = 'main_experiment/1'
NEAR_SATURATION = 0.93        # --plan에 따로 적는 트림 최대 회전수 비율(결과 해석용)
BATCHES = ('reference', 'table7_base', 'table7', 'gust', 'mission', 'ladder')


@dataclass
class Batch:
    name: str               # 'reference', 'table7', 'ladder:V13-0' …
    config: dict            # 파생 경기장 설정(시나리오 교체, 사다리면 V13 항목 교체)
    controllers: list       # 이 묶음에서 돌릴 제어기(경기장 라벨 — 사다리 변형도 'V13')
    variant: str = None     # 사다리 변형 이름(기록용)


# ══════════════════════════════════════════════════════════════════
# 읽기·가드
# ══════════════════════════════════════════════════════════════════

def load_spec(path=DEFAULT_SPEC):
    spec = json.loads(Path(path).read_text(encoding='utf-8'))
    if spec.get('schema') != SCHEMA:
        raise ValueError(f"schema must be {SCHEMA!r}, got {spec.get('schema')!r}")
    for key in ('base_config', 'tuned', 'controllers', 'common', 'table7', 'gust', 'mission', 'ladder'):
        if key not in spec:
            raise ValueError(f'missing main-experiment key: {key}')
    return spec


def _sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def guard_problems(spec, root=ROOT):
    """실행을 막는 사유 목록(비어 있으면 통과). --plan은 이것을 보여만 주고, --run은 하나라도 있으면 거부."""
    problems = []
    base = load_config(root/spec['base_config']['path'])
    actual = config_sha256(base)
    if actual != spec['base_config']['config_sha256']:
        problems.append(f"base config hash {actual[:12]} != recorded {spec['base_config']['config_sha256'][:12]}")
    tuned = spec['tuned']
    if not tuned.get('run_dir'):
        problems.append('tuned.run_dir is empty (tuning not finalized)')
    for label in spec['controllers']:
        entry = tuned['records'].get(label)
        if entry is None or not entry.get('sha256'):
            problems.append(f'{label}: tuning record sha256 is empty')
            continue
        if not tuned.get('run_dir'):
            continue
        path = root/tuned['run_dir']/f'{label}.record.json'
        if not path.exists():
            problems.append(f'{label}: {path} missing')
        elif _sha256_file(path) != entry['sha256']:
            problems.append(f'{label}: record sha256 {_sha256_file(path)[:12]} != recorded {entry["sha256"][:12]}')
    return problems


# ══════════════════════════════════════════════════════════════════
# 전개
# ══════════════════════════════════════════════════════════════════

def row_values(row, spec, native, base):
    """표7 행 하나 → (perturbation, extra_params). 규칙을 명목 파라미터로 숫자화한다."""
    from control.arena_plant_wrench import reference_inplane_drag_coefficient, hover_max_pitch_moment
    t7 = spec['table7']
    start, duration = float(t7['disturbance_start_s']), float(t7['disturbance_duration_s'])
    perturbation = dict(row.get('perturbation', {}))
    extra = dict(row.get('extra_params', {}))
    if 'cg_offset' in row:
        extra.update(cg_offset_axis=row['cg_offset']['axis'],
                     cg_offset_arm_fraction=float(row['cg_offset']['arm_fraction']))
    if 'rotor_inplane_drag' in row:
        rule = row['rotor_inplane_drag']
        V_ref = resolve_speed(rule['calibrate_at'], base)
        extra.update(rotor_inplane_drag_enabled=True,
                     rotor_inplane_drag_coeff=float(rule['multiple'])*reference_inplane_drag_coefficient(native, V_ref))
    if 'force_world' in row:
        rule = row['force_world']
        F = float(rule['g_multiple'])*native['mass']*native['g']*np.asarray(rule['direction'], dtype=float)
        extra.update(extra_force_world=F.tolist(), extra_force_start_s=start, extra_force_duration_s=duration)
    if 'moment_body' in row:
        rule = row['moment_body']
        M = float(rule['hover_max_pitch_fraction'])*hover_max_pitch_moment(native)*np.asarray(rule['axis'], dtype=float)
        extra.update(extra_moment_body=M.tolist(), extra_moment_start_s=start, extra_moment_duration_s=duration)
    return perturbation, extra


def _maneuver(spec, name):
    for m in spec['common']['maneuvers']:
        if m['name'] == name:
            return m
    raise ValueError(f'unknown maneuver {name!r}')


def _reference(spec, sid, maneuver, rho, **more):
    common = spec['common']['reference']
    m = _maneuver(spec, maneuver)
    return dict(id=sid, type='reference', **{'from': m['from']}, to=m['to'], rho=float(rho),
                lead_s=common['lead_s'], tail_s=common['tail_s'], ramp_round_s=common['ramp_round_s'], **more)


def _table7_base(spec, base_entry, sid):
    if base_entry.get('type') == 'cruise':
        return dict(id=sid, type='cruise', speed=base_entry['speed'], times_s=list(base_entry['times_s']),
                    evaluation_start_s=base_entry['evaluation_start_s'])
    return _reference(spec, sid, base_entry['maneuver'], base_entry['rho'],
                      min_total_s=spec['table7']['reference_min_total_s'])


def _with_case(scenario, perturbation=None, extra=None):
    s = dict(scenario)
    if perturbation:
        s['perturbation'] = dict(perturbation)
    if extra:
        s['extra_params'] = dict(extra)
    return s


def _derived(base, scenarios, v13_spec=None):
    config = deepcopy(base)
    config['scenarios'] = scenarios
    if v13_spec is not None:
        config['controllers']['V13'].update(v13_spec)
    validate_config(config)
    return config


def build_batches(spec, base, native):
    """모든 묶음을 전개한다. 시나리오 id는 전 묶음에서 겹치지 않는다(접두어 'main_')."""
    rhos = spec['common']['rho_levels']
    maneuvers = [m['name'] for m in spec['common']['maneuvers']]
    reference = [_reference(spec, f'main_ref_{m}_rho{r:g}', m, r) for m in maneuvers for r in rhos]

    t7 = spec['table7']
    bases = [_table7_base(spec, b, f'main_t7_{b["name"]}') for b in t7['bases']]
    table7 = []
    for row in t7['rows']:
        perturbation, extra = row_values(row, spec, native, base)
        for b in bases:
            table7.append(_with_case(dict(b, id=f'{b["id"]}__{row["name"]}'), perturbation, extra))

    g = spec['gust']
    gust = [dict(id=f'main_gust_{c["direction"]}_{c["peak_m_s"]:+g}_{v}', type='gust', speed=v,
                 direction=c['direction'], peak_m_s=float(c['peak_m_s']), times_s=list(g['times_s']))
            for v in g['speeds'] for c in g['cases']]
    mi = spec['mission']
    mission = [dict(id=f'main_mission_{v}', type='mission', speed=v, durations_s=list(mi['durations_s']),
                    evaluation_start_s=mi['evaluation_start_s']) for v in mi['speeds']]

    labels = list(spec['controllers'])
    batches = [Batch('reference', _derived(base, reference), labels),
               Batch('table7_base', _derived(base, bases), labels),
               Batch('table7', _derived(base, table7), labels),
               Batch('gust', _derived(base, gust), labels),
               Batch('mission', _derived(base, mission), labels)]

    lad = spec['ladder']
    for variant, flags in lad['variants'].items():
        scenarios = []
        for cond in lad['conditions']:
            if variant == 'V13' and cond['name'] == 'nominal':
                continue            # 참조 묶음의 V13과 같은 시행 — 다시 돌리지 않는다
            for s in reference:
                scenarios.append(_with_case(dict(s, id=f'main_lad_{variant}_{cond["name"]}__{s["id"][9:]}'),
                                            cond.get('perturbation'), cond.get('extra_params')))
        batches.append(Batch(f'ladder:{variant}', _derived(base, scenarios, flags), ['V13'], variant))
    return batches


# ══════════════════════════════════════════════════════════════════
# 기체 한계(트림 사전 확인)
# ══════════════════════════════════════════════════════════════════

def scenario_speeds(scenario):
    """사전 확인할 속도: 시작·끝. 임무는 호버에서 출발해 순항 속도를 지나므로 둘 다."""
    profile = scenario.profile
    v_start = float(profile.get_ref(0.0)[0][0])
    v_end = float(profile.get_ref(profile.T_total)[0][0])
    speeds = {v_start, v_end}
    if scenario.type == 'mission':
        speeds.add(float(profile.cruise_speed))
    return sorted(speeds)


def trim_status(native, scenario, cache=None):
    """(분류, 속도별 결과). 분류는 None(돌림) 또는 'vehicle_limit'."""
    from control.validation_suite import plant_truth, parameter_hash
    from control.arena_trim import find_trim_6dof
    from models.team_light.control.trim import find_trim
    case = scenario.cases[0]
    truth = plant_truth(native, case)
    use_6dof = 'cg_offset_axis' in case.get('extra_params', {})
    key = parameter_hash(truth)
    detail = {}
    for V in scenario_speeds(scenario):
        tag = (key, V, use_6dof)
        if cache is not None and tag in cache:
            detail[V] = cache[tag]
            continue
        if use_6dof:
            r = find_trim_6dof(truth, V, strict=False)
            result = dict(valid=bool(r['valid']), method='6dof', condition=r['condition'],
                          residual=r['residual'])
        else:
            r = find_trim(truth, V, strict=False)
            result = dict(valid=bool(r['valid']), method='planar', residual=r['residual'])
        if r['valid']:
            result['max_n_fraction'] = float(np.max(r['control'])/truth['n_max'])
        detail[V] = result
        if cache is not None:
            cache[tag] = result
    limited = any(not d['valid'] for d in detail.values())
    return ('vehicle_limit' if limited else None), detail


def authority_boundary(detail, spec, base):
    """해석 표시(kj 2026-09-28): 기준 속도(V_H) 트림의 최대 회전수가 문턱(0.93 n_max) 이상이면 True.
    결과를 버리지 않고 '권한 경계' 사례로 표시만 한다."""
    rule = spec['interpretation']['authority_boundary']
    V = resolve_speed(rule['speed'], base)
    d = detail.get(V)
    return bool(d is not None and d.get('max_n_fraction', 0.0) >= float(rule['max_n_fraction']))


# ══════════════════════════════════════════════════════════════════
# 계획
# ══════════════════════════════════════════════════════════════════

def plan(spec, speed_ratio=1.0, check_trims=True, root=ROOT):
    """묶음별 시행 수·시뮬레이션 시간·예상 시간, ρ 목표 대 실제, 기체 한계 목록."""
    from control.arena_factory import ArenaFactory
    from control.validation_suite import baseline_params
    base = load_config(root/spec['base_config']['path'])
    native = baseline_params()
    cp = ArenaFactory(base, native).cp
    rates = spec['timing_mac_wall_per_sim_s']
    rows, rho_rows, limits, near, cache = [], [], [], [], {}
    for batch in build_batches(spec, base, native):
        scenarios = build_scenarios(batch.config, cp, native)
        count, skipped_region, skipped_limit, sim_s, wall = 0, 0, 0, 0.0, 0.0
        for s in scenarios:
            if s.type == 'reference':
                rho_rows.append(dict(batch=batch.name, id=s.id, target=s.meta['rho_target'],
                                     actual=s.meta['rho_actual'], v_star=s.meta['v_star'],
                                     ramp_s=s.meta['ramp_s'], T_total=s.profile.T_total))
            status = None
            if check_trims:
                status, detail = trim_status(native, s, cache)
                if status:
                    limits.append(dict(batch=batch.name, id=s.id, detail=detail))
                worst = max((d.get('max_n_fraction', 0.0) for d in detail.values()), default=0.0)
                if worst >= NEAR_SATURATION and not batch.name.startswith('ladder'):
                    near.append(dict(batch=batch.name, id=s.id, max_n_fraction=worst))
            for label in batch.controllers:
                if excluded_from(batch.config, label, s):
                    skipped_region += 1
                    continue
                if status:
                    skipped_limit += 1
                    continue
                count += 1
                sim_s += s.profile.T_total
                wall += s.profile.T_total*float(rates[label])*speed_ratio
        rows.append(dict(batch=batch.name, scenarios=len(scenarios), trials=count,
                         excluded_design_region=skipped_region, vehicle_limit=skipped_limit,
                         sim_seconds=sim_s, est_hours_sequential=wall/3600.0))
    return dict(batches=rows, rho=rho_rows, vehicle_limits=limits, near_saturation=near,
                guard_problems=guard_problems(spec, root), speed_ratio=speed_ratio)


def print_plan(result):
    print('\n## 묶음별 시행 수와 예상 시간(순차, 맥 실측 × speed_ratio '
          f'{result["speed_ratio"]:g} — Windows 미측정)\n')
    print('| 묶음 | 시나리오 | 시행 | 설계영역 제외 | 기체 한계 | 시뮬레이션 s | 예상 시간(h) |')
    print('|---|---:|---:|---:|---:|---:|---:|')
    total = dict(trials=0, sim=0.0, hours=0.0)
    for r in result['batches']:
        print(f"| {r['batch']} | {r['scenarios']} | {r['trials']} | {r['excluded_design_region']} | "
              f"{r['vehicle_limit']} | {r['sim_seconds']:.0f} | {r['est_hours_sequential']:.1f} |")
        total['trials'] += r['trials']
        total['sim'] += r['sim_seconds']
        total['hours'] += r['est_hours_sequential']
    print(f"| **합계** | | **{total['trials']}** | | | {total['sim']:.0f} | **{total['hours']:.1f}** |")
    print('\n## ρ 목표 대 실제(참조형 시나리오, 사다리는 참조 묶음과 같은 프로필이라 생략)\n')
    print('| 묶음 | 시나리오 | 목표 | 실제 | 차이 | v* (m/s) | 전이 s | 전체 s |')
    print('|---|---|---:|---:|---:|---:|---:|---:|')
    seen = set()
    for r in result['rho']:
        if r['batch'].startswith('ladder') or (r['batch'] == 'table7' and r['id'].split('__')[0] in seen):
            continue
        seen.add(r['id'].split('__')[0])
        print(f"| {r['batch']} | {r['id'].split('__')[0]} | {r['target']:g} | {r['actual']:.4f} | "
              f"{100*(r['actual']/r['target']-1):+.2f}% | {r['v_star']:.2f} | {r['ramp_s']:.3f} | {r['T_total']:.3f} |")
    print('\n## 기체 한계(트림 없음 → 돌리지 않음, 제어기 실패로 세지 않음)\n')
    if not result['vehicle_limits']:
        print('없음')
    for r in result['vehicle_limits']:
        bad = {f'{v:g}': d for v, d in r['detail'].items() if not d['valid']}
        print(f"- {r['batch']} / {r['id']}: {json.dumps(bad, ensure_ascii=False)}")
    print(f'\n## 포화 경계(시작·끝 트림의 최대 회전수 ≥ {NEAR_SATURATION:.0%} n_max, 사다리 제외)\n')
    for r in sorted(result.get('near_saturation', []), key=lambda r: -r['max_n_fraction']):
        print(f"- {r['batch']} / {r['id']}: {r['max_n_fraction']:.3f}")
    print('\n## 실행 가드\n')
    if result['guard_problems']:
        print('**실행 불가**:')
        for p in result['guard_problems']:
            print(f'- {p}')
    else:
        print('통과')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--spec', type=Path, default=DEFAULT_SPEC)
    parser.add_argument('--plan', action='store_true', required=True)
    parser.add_argument('--speed-ratio', type=float, default=1.0,
                        help='(학교 Windows 벽시계)/(맥 벽시계). env_check 재현 시간으로 잰다')
    parser.add_argument('--no-trim-check', action='store_true', help='트림 사전 확인 생략(빠름)')
    args = parser.parse_args(argv)
    print_plan(plan(load_spec(args.spec), speed_ratio=args.speed_ratio, check_trims=not args.no_trim_check))
    return 0


if __name__ == '__main__':
    sys.exit(main())
