"""튜닝 기록 분석 도구 — `control/arena_tune.py`가 남긴 기록을 **읽기만** 한다.

이 모듈은 결론을 내리지 않는다. 숫자·표·그림·"실행 계획"만 만든다.
해석 주의사항은 results/TUNING_ANALYSIS_README.md에 있다.

읽는 기록(형식은 arena_tune.py 그대로, 바꾸지 않는다):
  <run-dir>/<제어기>.record.json   요약: budget, spent, status, parameter_names, prior,
                                   prior_objective, best_values, best_exponents, best_objective …
  <run-dir>/<제어기>.jsonl         평가 1회 = 1줄: index, exponents, values, objective,
                                   cache_hit, scenarios(캐시 적중이면 None)

제어기별로 계산하는 것:
  - 첫 평가(사전값 = 기본 가중치)의 목적함수 값과 실패 여부
  - 학습 곡선: 평가 k회까지의 누적 최선 값(목적함수는 작을수록 좋다)
  - "최선 값의 90%" 도달 시점 — 정의가 여럿이라 전부 계산한다(README 참고)
      A. 개선폭 90%   f ≤ f₀ − 0.9·(f₀ − f*)        f₀ = 첫 평가
      B. 비율 기준    f ≤ f*/0.9                    (최선보다 약 11% 나쁜 수준 이내)
      C. 개선폭 90%, f₀를 첫 비실패 평가로 다시 잡음  (실패 탈출과 실제 개선을 분리)
    그리고 첫 비실패 평가 횟수
  - 튜닝 파라미터 개수, GSLQR 운용점 개수(configs/arena.json에서 읽음)
  - 최종 가중치 ×0.5·×2 민감도 실험의 실행 계획 파일(실행하지 않는다)

공정성 게이트(I-4와 같은 원칙): 제어기 간 평가 예산이 다르면 분석을 거부한다.

실행 예:
  python -m control.tuning_analysis --run-dir results/arena/tuning/pilot24
  python -m control.tuning_analysis --run-dir results/arena/tuning/pilot24 --truncate-to-common
"""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from control.arena import load_config, config_sha256, DEFAULT_CONFIG
from control.arena_factory import ARENA_LABELS, NMPC_LABELS
from control.arena_tune import check_tuning_records

PARTIAL = 'PARTIAL'
SENSITIVITY_FACTORS = (0.5, 2.0)
# 목적함수 비교용 상대 허용오차. 기록은 JSON에 float로 왕복하므로 정확히 같아야 하지만,
# 임계값을 f₀ − 0.9·(f₀ − f*)처럼 계산하면 반올림으로 f*보다 아주 조금 작아질 수 있다.
_RTOL = 1e-12


class AnalysisRefused(RuntimeError):
    """공정성·일관성 검사를 통과하지 못해 분석을 거부한다."""


# ══════════════════════════════════════════════════════════════════
# 1) 기록 읽기
# ══════════════════════════════════════════════════════════════════

def load_run(run_dir):
    """run-dir의 기록을 {제어기: dict(record=…, evaluations=[…])}로 읽는다.

    JSONL은 평가 순서대로 한 줄씩이다. 인덱스가 0부터 빈틈없이 이어지지 않으면
    (중간 줄 유실·중복 기록) 곡선이 틀어지므로 거부한다.
    """
    run_dir = Path(run_dir)
    record_paths = sorted(run_dir.glob('*.record.json'))
    if not record_paths:
        raise AnalysisRefused(f'no *.record.json in {run_dir}')
    runs = {}
    for path in record_paths:
        record = json.loads(path.read_text(encoding='utf-8'))
        label = record['controller']
        log_path = run_dir/f'{label}.jsonl'
        if not log_path.exists():
            raise AnalysisRefused(f'{label}: record exists but {log_path.name} is missing')
        evaluations = [json.loads(line) for line in log_path.read_text(encoding='utf-8').splitlines()
                       if line.strip()]
        indices = [e['index'] for e in evaluations]
        if indices != list(range(len(evaluations))):
            raise AnalysisRefused(f'{label}: evaluation indices are not 0..{len(evaluations) - 1} '
                                  'in order (lost or duplicated lines)')
        runs[label] = dict(record=record, evaluations=evaluations)
    return runs


# ══════════════════════════════════════════════════════════════════
# 2) 공정성 게이트 — 예산이 다르면 거부(I-4와 같은 원칙)
# ══════════════════════════════════════════════════════════════════

def check_runs(runs, config, truncate_to_common=False):
    """분석해도 되는지 검사한다. 통과하면 (분석할 평가 수, 라벨)을 돌려준다.

    항상 거부하는 경우:
      - 제어기 간 budget이 다르다 (잘라서 맞추는 것도 허용하지 않는다 —
        예산이 다르면 탐색 보폭 재시작 시점부터 달라져 같은 조건이 아니다)
      - record의 spent와 JSONL 줄 수가 다르다 (기록 자체가 어긋남)
    기본 모드(엄격): arena_tune.check_tuning_records(I-4)가 위반 0건이어야 한다
      → 모든 제어기가 예산을 정확히 다 썼고(status complete) 시나리오·난수·목적함수가 같다.
    --truncate-to-common: 미완료를 허용하되 모든 제어기를 가장 짧은 평가 수로 자르고
      라벨을 PARTIAL로 바꾼다. I-4의 나머지 항목(시나리오·난수·목적함수)은 그대로 검사한다.
    """
    budgets = {label: run['record']['budget'] for label, run in runs.items()}
    if len(set(budgets.values())) != 1:
        raise AnalysisRefused(f'budgets differ between controllers: {budgets}')
    for label, run in runs.items():
        spent, logged = run['record']['spent'], len(run['evaluations'])
        if spent != logged:
            raise AnalysisRefused(f'{label}: record spent={spent} but JSONL has {logged} evaluations')
        if logged == 0:
            raise AnalysisRefused(f'{label}: no evaluations logged yet')
    records = [run['record'] for run in runs.values()]
    if not truncate_to_common:
        violations = check_tuning_records(records, config)
        if violations:
            raise AnalysisRefused('I-4 violations (use --truncate-to-common only for in-progress '
                                  'runs): ' + '; '.join(violations))
        return next(iter(budgets.values())), records[0].get('label', 'PILOT')
    # 미완료 기록을 "완료로 쳤을 때" I-4의 나머지 항목이 통과하는지 본다.
    # 원본 record는 건드리지 않고 복사본의 spent·status만 바꾼다 — 예산 소진 여부는
    # 여기서 공통 최소치로 잘라 맞추므로, 그 두 항목만 검사에서 빼기 위한 것이다.
    as_if_complete = [dict(r, spent=r['budget'], status='complete') for r in records]
    violations = check_tuning_records(as_if_complete, config)
    if violations:
        raise AnalysisRefused('I-4 violations other than budget completion: ' + '; '.join(violations))
    return min(len(run['evaluations']) for run in runs.values()), PARTIAL


def check_record_consistency(label, run, n_used):
    """record 요약과 JSONL 원자료가 서로 맞는지(완료 기록일 때만 의미가 있다)."""
    record, objectives = run['record'], [e['objective'] for e in run['evaluations'][:n_used]]
    problems = []
    if record.get('prior_objective') is not None and not _close(objectives[0], record['prior_objective']):
        problems.append(f"{label}: JSONL first objective {objectives[0]} != prior_objective "
                        f"{record['prior_objective']}")
    if record.get('best_objective') is not None and n_used == len(run['evaluations']) \
            and not _close(min(objectives), record['best_objective']):
        problems.append(f"{label}: JSONL min objective {min(objectives)} != best_objective "
                        f"{record['best_objective']}")
    if any(v != 0.0 for v in run['evaluations'][0]['exponents']):
        problems.append(f'{label}: first evaluation is not the prior (exponents not all zero)')
    return problems


def _close(a, b):
    return abs(a - b) <= _RTOL*max(1.0, abs(a), abs(b))


# ══════════════════════════════════════════════════════════════════
# 3) 곡선·지표 계산 — 순수 함수(테스트 대상)
# ══════════════════════════════════════════════════════════════════

def learning_curve(objectives):
    """평가 k회까지의 누적 최선 값(작을수록 좋음). 길이는 입력과 같다."""
    return np.minimum.accumulate(np.asarray(objectives, dtype=float)).tolist()


def first_reach(curve, threshold, start=0):
    """curve[k] ≤ threshold인 첫 k(start부터)를 **1부터 센 평가 횟수**로. 없으면 None."""
    tol = _RTOL*max(1.0, abs(threshold))
    for k in range(start, len(curve)):
        if curve[k] <= threshold + tol:
            return k + 1
    return None


def evaluation_failures(evaluations):
    """평가마다 '한 시나리오라도 실패했는가'. 캐시 적중 줄은 scenarios가 None이므로
    같은 좌표(exponents)를 처음 실제로 돌린 줄의 결과를 따른다."""
    first_seen, flags = {}, []
    for e in evaluations:
        key = tuple(e['exponents'])
        if e.get('scenarios') is not None:
            failed = any(s['failed'] for s in e['scenarios'])
            first_seen.setdefault(key, failed)
        elif key in first_seen:
            failed = first_seen[key]
        else:
            raise AnalysisRefused(f"evaluation {e['index']}: cache hit without an earlier "
                                  'evaluation of the same point')
        flags.append(failed)
    return flags


def improvement_threshold(f_start, f_best, fraction=0.9):
    """f_start → f_best 개선폭의 fraction만큼 온 목적함수 값."""
    return f_start - fraction*(f_start - f_best)


def controller_metrics(label, evaluations, config, record=None):
    """한 제어기의 지표 dict. evaluations는 이미 공통 길이로 잘린 목록이다."""
    objectives = [float(e['objective']) for e in evaluations]
    failed = evaluation_failures(evaluations)
    curve = learning_curve(objectives)
    f0, f_best = objectives[0], curve[-1]
    first_scenarios = evaluations[0]['scenarios'] or []
    first_nonfailed = next((k for k, f in enumerate(failed) if not f), None)
    if first_nonfailed is None:
        to_90_nonfailed = None
    else:
        f_nf = objectives[first_nonfailed]
        to_90_nonfailed = first_reach(curve, improvement_threshold(f_nf, f_best), start=first_nonfailed)
    group = 'NMPC' if label in NMPC_LABELS else label
    names = config['tuning']['parameters'][group]
    if record is not None and list(record['parameter_names']) != list(names):
        raise AnalysisRefused(f"{label}: record parameter_names {record['parameter_names']} differ "
                              f'from configs/arena.json {names} — config changed after tuning')
    return dict(
        controller=label,
        evaluations=len(objectives),
        first_objective=f0,
        first_failed=failed[0],
        first_failed_scenarios=[s['id'] for s in first_scenarios if s['failed']],
        best_objective=f_best,
        best_at_evaluation=int(np.argmin(objectives)) + 1,
        failed_evaluations=int(sum(failed)),
        first_nonfailed_evaluation=None if first_nonfailed is None else first_nonfailed + 1,
        evals_to_90pct_improvement=first_reach(curve, improvement_threshold(f0, f_best)),
        evals_to_within_best_over_0p9=first_reach(curve, f_best/0.9),
        evals_to_90pct_improvement_from_first_nonfailed=to_90_nonfailed,
        n_parameters=len(names),
        gslqr_operating_points=(len(config['controllers']['GSLQR']['V_table_m_s'])
                                if label == 'GSLQR' else None),
        curve=curve,
        objectives=objectives,
        failed=failed,
    )


# ══════════════════════════════════════════════════════════════════
# 4) 민감도 실험 "실행 계획" — 계획만 쓰고 실행하지 않는다
# ══════════════════════════════════════════════════════════════════

def sensitivity_plan(runs, config):
    """최종 가중치에서 파라미터를 하나씩 ×0.5·×2 한 실행 목록.

    ×0.5·×2는 arena_tune의 log₂ 배수 좌표에서 정확히 −1·+1이다. 그래서 튜닝 중에 이미
    같은 좌표를 평가했다면 그 목적함수 값을 참고로 붙인다(새로 돌린 값이 아니다).
    시나리오는 튜닝 시나리오(record의 scenario_ids) — 본시험 시나리오는 쓰지 않는다.
    """
    runs_out = []
    for label in _ordered(runs):
        record, evaluations = runs[label]['record'], runs[label]['evaluations']
        logged = {}
        for e in evaluations:
            logged.setdefault(tuple(e['exponents']), e['objective'])
        best_e = [float(x) for x in record['best_exponents']]
        for i, name in enumerate(record['parameter_names']):
            for factor in SENSITIVITY_FACTORS:
                exponents = list(best_e)
                exponents[i] += float(np.log2(factor))
                values = dict(record['best_values'])
                values[name] = record['best_values'][name]*factor
                runs_out.append(dict(
                    id=f'{label}:{name}:x{factor:g}', controller=label, parameter=name,
                    factor=factor, exponents=exponents, values=values,
                    tuning_log_objective=logged.get(tuple(exponents))))
    first = runs[_ordered(runs)[0]]['record']
    return dict(
        status='PLAN_ONLY — not executed',
        generated_utc=datetime.now(timezone.utc).isoformat(),
        design='one-at-a-time: each tuned parameter ×0.5 and ×2 around the final (best) values',
        scenario_set='tuning',
        scenario_ids=first['scenario_ids'],
        objective=first['objective'],
        config_sha256=config_sha256(config),
        how_to_run=('values → overrides via control.arena_tune.parameter_space(...)[2], then '
                    'control.arena_tune.Evaluator(...)(overrides); not done by this tool'),
        baseline={label: dict(best_values=runs[label]['record']['best_values'],
                              best_objective=runs[label]['record']['best_objective'])
                  for label in _ordered(runs)},
        runs=runs_out,
    )


# ══════════════════════════════════════════════════════════════════
# 5) 분석 묶음 + 저장(표·그림)
# ══════════════════════════════════════════════════════════════════

def analyze(run_dir, config, truncate_to_common=False):
    """검사 → 지표. 거부 사유가 있으면 AnalysisRefused를 던진다."""
    runs = load_run(run_dir)
    n_used, label = check_runs(runs, config, truncate_to_common)
    inconsistencies, warnings = [], []
    for name, run in runs.items():
        inconsistencies += check_record_consistency(name, run, n_used)
        recorded = run['record'].get('config_sha256')
        if recorded and recorded != config_sha256(config):
            # 설명 문구만 바뀌어도 해시는 달라진다 → 거부하지 않고 표에 경고로 남긴다.
            # 분석에 실제로 쓰는 항목(파라미터 이름)은 controller_metrics에서 따로 대조한다.
            warnings.append(f'{name}: record config_sha256 differs from the config used here')
    if inconsistencies:
        # 요약(record)과 원자료(JSONL)가 어긋나면 어느 쪽을 믿을지 알 수 없다.
        raise AnalysisRefused('record/log inconsistency: ' + '; '.join(inconsistencies))
    metrics = [controller_metrics(name, runs[name]['evaluations'][:n_used], config, runs[name]['record'])
               for name in _ordered(runs)]
    return dict(label=label, run_dir=str(run_dir), evaluations_per_controller=n_used,
                budget=next(iter(runs.values()))['record']['budget'],
                failure_penalty=float(config['tuning']['objective']['failure_penalty']),
                warnings=warnings, controllers=metrics), runs


def _ordered(runs):
    """경기장 표준 순서(V13, M17, F13, GSLQR, CPID) — 색·행 순서가 제어기를 따라가게."""
    known = [label for label in ARENA_LABELS if label in runs]
    return known + sorted(set(runs) - set(known))


TABLE_COLUMNS = [
    ('controller', 'controller'),
    ('evaluations', 'evals'),
    ('n_parameters', 'n_params'),
    ('gslqr_operating_points', 'gslqr_op_points'),
    ('first_objective', 'first_obj'),
    ('first_failed', 'first_failed'),
    ('first_failed_scenarios', 'first_failed_scenarios'),
    ('best_objective', 'best_obj'),
    ('best_at_evaluation', 'best_at'),
    ('failed_evaluations', 'n_failed_evals'),
    ('first_nonfailed_evaluation', 'first_nonfailed'),
    ('evals_to_90pct_improvement', 'to90_A_improve'),
    ('evals_to_within_best_over_0p9', 'to90_B_ratio'),
    ('evals_to_90pct_improvement_from_first_nonfailed', 'to90_C_improve_nonfailed'),
]


def _cell(value):
    if value is None:
        return '—'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if isinstance(value, float):
        return f'{value:.6g}'
    if isinstance(value, list):
        return ', '.join(value) if value else '—'
    return str(value)


def write_outputs(analysis, runs, config, out_dir):
    """summary.csv / summary.md / analysis.json / learning_curves.png (+ 완료 기록이면
    sensitivity_plan.json). 쓴 파일 경로 목록을 돌려준다."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = analysis['controllers']
    written = []

    with (out_dir/'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([header for _, header in TABLE_COLUMNS])
        for row in rows:
            writer.writerow([_cell(row[key]) for key, _ in TABLE_COLUMNS])
    written.append(out_dir/'summary.csv')

    lines = [f"# Tuning analysis — {analysis['label']} (no conclusions)", '',
             f"run-dir: `{analysis['run_dir']}` · budget {analysis['budget']} · "
             f"evaluations analysed per controller {analysis['evaluations_per_controller']} · "
             f"failure penalty {analysis['failure_penalty']:g}", '',
             'Objective: lower is better. to90_A/B/C definitions: see results/TUNING_ANALYSIS_README.md.',
             '', '| ' + ' | '.join(h for _, h in TABLE_COLUMNS) + ' |',
             '|' + '---|'*len(TABLE_COLUMNS)]
    lines += ['| ' + ' | '.join(_cell(row[key]) for key, _ in TABLE_COLUMNS) + ' |' for row in rows]
    if analysis['warnings']:
        lines += ['', '## Warnings', ''] + [f'- {w}' for w in analysis['warnings']]
    (out_dir/'summary.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    written.append(out_dir/'summary.md')

    (out_dir/'analysis.json').write_text(json.dumps(analysis, ensure_ascii=False, indent=2) + '\n',
                                         encoding='utf-8')
    written.append(out_dir/'analysis.json')

    written.append(plot_learning_curves(analysis, out_dir/'learning_curves.png'))

    if analysis['label'] != PARTIAL:
        plan = sensitivity_plan(runs, config)
        (out_dir/'sensitivity_plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2) + '\n',
                                                     encoding='utf-8')
        written.append(out_dir/'sensitivity_plan.json')
    return written


# 제어기별 고정 색(순위가 아니라 제어기를 따라간다) — dataviz 기본 범주 팔레트 1~5번,
# 밝은 배경 검사기 통과. 대비 3:1 미만인 색이 있지만 패널 제목이 제어기를 밝히므로
# 식별이 색에만 의존하지 않는다(표 summary.csv도 함께 나간다).
SERIES_COLOR = {'V13': '#2a78d6', 'M17': '#eb6834', 'F13': '#1baf7a',
                'GSLQR': '#eda100', 'CPID': '#e87ba4'}
INK, MUTED = '#333333', '#8a8a8a'
# 90% 도달 지점 A·B·C는 제어기 색이 아니라 모양으로 구분한다(모든 패널 공통).
REACH_MARKERS = [('evals_to_90pct_improvement', 'o', 'A: 90% of improvement from 1st eval'),
                 ('evals_to_within_best_over_0p9', 's', 'B: within best/0.9'),
                 ('evals_to_90pct_improvement_from_first_nonfailed', '^',
                  'C: 90% of improvement from 1st non-failed')]


def plot_learning_curves(analysis, path):
    """제어기별 작은 그림(small multiples). 패널마다:
    누적 최선 계단선 + 평가 원값(실패는 ×) + A·B·C 도달 지점 + 실패 벌점 수준(점선).

    한 축에 다섯 곡선을 겹치면 선·라벨이 겹치고, 정의 A 하나만 찍으면 '실패 탈출'이
    빠른 수렴처럼 보인다(README 주의사항). 그래서 패널을 나누고 A·B·C를 함께 찍는다.
    y축은 모든 패널이 공유한다 — 패널마다 축이 다르면 크기 비교가 왜곡된다.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    rows = analysis['controllers']
    ncols = min(3, len(rows))
    nrows = int(np.ceil(len(rows)/ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(3.6*ncols, 2.9*nrows + 0.8), dpi=150,
                             sharex=True, sharey=True, squeeze=False)
    positive = all(v > 0 for row in rows for v in row['objectives'])
    for ax, row in zip(axes.flat, rows):
        color = SERIES_COLOR.get(row['controller'], MUTED)
        x = np.arange(1, len(row['curve']) + 1)
        failed = np.asarray(row['failed'])
        objectives = np.asarray(row['objectives'])
        ax.axhline(analysis['failure_penalty'], color=MUTED, linewidth=1, linestyle=':')
        ax.plot(x[~failed], objectives[~failed], linestyle='none', marker='.', markersize=5,
                color=color, alpha=0.45)
        ax.plot(x[failed], objectives[failed], linestyle='none', marker='x', markersize=5,
                color=INK, alpha=0.7)
        ax.step(x, row['curve'], where='post', color=color, linewidth=2)
        for key, marker, _ in REACH_MARKERS:
            k = row[key]
            if k is not None:
                ax.plot(k, row['curve'][k - 1], marker=marker, markersize=8, color=INK,
                        markerfacecolor='white', markeredgewidth=1.5, linestyle='none')
        ax.set_title(row['controller'], fontsize=10, color=INK, loc='left')
        if positive:
            ax.set_yscale('log')
        ax.grid(True, which='major', color='#e5e5e5', linewidth=0.8)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
    for ax in list(axes.flat)[len(rows):]:
        ax.set_visible(False)
    # 각 열의 '보이는 맨 아래' 패널에 x 눈금·라벨을 단다(빈 칸 위 패널도 포함).
    for col in range(ncols):
        visible = [axes[r, col] for r in range(nrows) if r*ncols + col < len(rows)]
        if visible:
            visible[-1].xaxis.set_tick_params(labelbottom=True)
            visible[-1].set_xlabel('Evaluations (cache hits counted)', fontsize=9)
    for ax in axes[:, 0]:
        ax.set_ylabel('Objective (lower is better)', fontsize=9)
    handles = [Line2D([], [], color=MUTED, linewidth=2, label='best so far'),
               Line2D([], [], color=MUTED, marker='.', linestyle='none', label='evaluation'),
               Line2D([], [], color=INK, marker='x', linestyle='none', label='failed evaluation'),
               Line2D([], [], color=MUTED, linestyle=':', label='failure penalty')]
    handles += [Line2D([], [], color=INK, marker=m, markerfacecolor='white', linestyle='none', label=t)
                for _, m, t in REACH_MARKERS]
    fig.legend(handles=handles, loc='lower center', ncol=4, frameon=False, fontsize=8)
    fig.suptitle(f"Tuning learning curves — {analysis['label']} (no conclusions)", fontsize=11,
                 x=0.01, ha='left')
    fig.tight_layout(rect=(0, 0.1, 1, 0.97))
    fig.savefig(path)
    plt.close(fig)
    return Path(path)


# ══════════════════════════════════════════════════════════════════
# 6) 명령행
# ══════════════════════════════════════════════════════════════════

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out-dir', type=Path, help='default: <run-dir>/analysis')
    parser.add_argument('--truncate-to-common', action='store_true',
                        help='allow in-progress runs: cut every controller to the shortest '
                             'evaluation count and label the output PARTIAL (no sensitivity plan)')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    try:
        analysis, runs = analyze(args.run_dir, config, args.truncate_to_common)
    except AnalysisRefused as exc:
        print(f'REFUSED: {exc}')
        return 2
    written = write_outputs(analysis, runs, config, args.out_dir or args.run_dir/'analysis')
    print((Path(written[0]).parent/'summary.md').read_text(encoding='utf-8'))
    if analysis['label'] == PARTIAL:
        print('PARTIAL: sensitivity plan not written (final weights do not exist yet).')
    for path in written:
        print(f'wrote {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
