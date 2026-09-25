#!/usr/bin/env python3
"""경기장 검증 — 외부 검토자가 명령 하나로 돌린다 (작업지시서 작업 F).

    python scripts/verify_arena.py            # 전체 (목표 30분 이내)
    python scripts/verify_arena.py --quick    # 무거운 사례·M17 NLP 생략 (CPU 1개·3 GB에서 20분 이내)

차례로 하는 일(각 단계 통과/실패를 표로 출력, 하나라도 실패하면 종료코드 1):
  1) 실행 환경 요약 — OS·CPU·파이썬·패키지·스레드 설정
  2) 공정성 불변식 테스트 control/test_arena_fairness.py (I-1 ~ I-11) — 묶음별 새 프로세스
  3) 설정 파일과 1절 확정 사항 대조(check_confirmed_facts)
  4) 스모크 사례 일부를 **새 프로세스에서** 다시 돌려 커밋된 참조
     (results/arena/smoke_reference.json)와 비교
       - 실행 환경 지문이 참조와 같으면: 궤적 해시가 비트 단위로 같아야 한다
       - 다르면: 수치 지표 상대오차 ≤ --rtol (기본 1e-3, 잠정치),
         그리고 판정(passed·paper_failed)·정지사유가 같아야 한다

스레드 수는 1로 고정해 하위 프로세스를 띄운다(BLAS 병렬 합산 순서가 결과를 바꾸지
않게). 결과는 시뮬레이션 설정의 함수이지 컴퓨터 속도의 함수가 아니다.

자원 예산(마지막 줄에 실측을 출력한다): --quick은 CPU 1개·메모리 3 GB에서 20분 이내
(kj 2026-09-26), 전체는 30분 이내(작업지시서). 단계는 하나씩 차례로 돌고 하위 프로세스는
단일 스레드라 CPU 1개로 충분하다. 예산을 넘겨도 검증 판정(종료코드)은 바꾸지 않는다 —
느린 컴퓨터를 실패로 만들면 안 되기 때문이다. 대신 표에 WARN으로 드러낸다.
"""
import argparse
import json
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
import tempfile
import time

# 빠른 모드 예산은 kj 지시(2026-09-26), 전체 모드 30분은 작업지시서. 전체 모드는 메모리 조건이
# 없어 실측만 적는다(M17 NLP를 지어서 ~2.8 GiB까지 간다). 빠른 모드는 M17 NLP를 짓지 않는다.
BUDGET = {True: dict(minutes=20.0, memory_gb=3.0), False: dict(minutes=30.0, memory_gb=None)}


def peak_memory_gb():
    """지금까지 끝난 하위 프로세스와 이 프로세스 중 최대 상주 메모리(GiB)."""
    scale = 1.0 if platform.system() == 'Darwin' else 1024.0     # macOS는 바이트, 리눅스는 KB
    peak = max(resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
               resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    return peak*scale/1024**3

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT/'results'/'arena'/'smoke_reference.json'
THREADS = {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
           'VECLIB_MAXIMUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}
# 재실행할 사례(짧은 것 위주). --quick은 앞의 일부만.
RERUN_FULL = [('gust_lateral_p10_VH', c) for c in ('GSLQR', 'CPID', 'V13', 'F13', 'M17')] + \
             [('mission_VH', c) for c in ('GSLQR', 'CPID')]
RERUN_QUICK = [('gust_lateral_p10_VH', 'GSLQR'), ('gust_lateral_p10_VH', 'V13')]
# 교차 머신 비교에 쓰는 수치 지표와 반드시 같아야 하는 판정 지표
NUMERIC = ('rmse_z', 'rmse_velocity', 'max_omega', 'window_rmse_velocity', 'window_rmse_z',
           'simulated_seconds')
EXACT = ('passed', 'tracking_pass', 'paper_failed', 'stop_reason')
FINGERPRINT_KEYS = ('system', 'machine', 'cpu', 'python', 'packages')


def env():
    e = dict(os.environ)
    e.update(THREADS)
    e['PYTHONPATH'] = str(ROOT) + os.pathsep + e.get('PYTHONPATH', '')
    return e


def run(cmd, timeout):
    started = time.perf_counter()
    proc = subprocess.run(cmd, cwd=ROOT, env=env(), capture_output=True, text=True,
                          timeout=timeout)
    return proc, time.perf_counter() - started


def step_environment():
    sys.path.insert(0, str(ROOT))
    from control.validation_suite import environment_fingerprint
    fp = environment_fingerprint()
    ok = sys.version_info >= (3, 10)
    detail = (f"{fp['system']} {fp['machine']} | {fp['cpu']} | Python {fp['python']} | "
              + ', '.join(f'{k} {v}' for k, v in fp['packages'].items()))
    return ok, detail, fp


# 공정성 테스트를 불변식 묶음별로 **따로 된 프로세스**에서 돌린다. 한 프로세스에서 다 돌리면
# NMPC NLP(M17 하나에 ~2 GB)를 여러 번 지으며 메모리 단편화가 쌓여 최대 2.6~4.4 GiB까지 갔다
# (2026-09-26 실측). 묶음마다 새 프로세스면 최대치가 'NLP 하나 짓기' 수준으로 묶인다.
# 마지막 묶음은 앞 묶음들의 부정이라, 이름이 바뀌어도 빠지는 테스트가 없다.
TEST_GROUPS = (('I-1', 'i1_'), ('I-2 I-3 I-11', 'i2_ or i3_ or i11_'), ('I-5', 'i5_'),
               ('I-6', 'i6_'), ('I-9', 'i9_'))
TEST_GROUPS += (('rest', 'not (' + ' or '.join(k for _, k in TEST_GROUPS) + ')'),)


def step_tests(quick):
    e = env()
    if quick:
        e['ARENA_QUICK'] = '1'
    started = time.perf_counter()
    ok, counts, notes = True, {}, []
    for name, expr in TEST_GROUPS:
        cmd = [sys.executable, '-m', 'pytest', 'control/test_arena_fairness.py', '-q', '-p',
               'no:cacheprovider', '-k', expr]
        proc = subprocess.run(cmd, cwd=ROOT, env=e, capture_output=True, text=True, timeout=3600)
        last = ([line for line in proc.stdout.strip().splitlines() if line.strip()] or ['no output'])[-1]
        for part in last.split(' in ')[0].split(','):
            fields = part.strip().split()
            if len(fields) == 2 and fields[0].isdigit():
                counts[fields[1]] = counts.get(fields[1], 0) + int(fields[0])
        if proc.returncode != 0:
            ok = False
            notes.append(f'{name}: {last}')
    summary = ', '.join(f'{v} {k}' for k, v in counts.items())
    detail = f'{summary} in {len(TEST_GROUPS)} processes ({time.perf_counter()-started:.0f}s)'
    return ok, detail + ('; FAILED ' + ' | '.join(notes) if notes else '')


def step_facts():
    from control.arena import load_config, check_confirmed_facts
    from models.team_light.control.baseline_v2 import baseline_params
    violations = check_confirmed_facts(load_config(), baseline_params())
    return not violations, 'no violations' if not violations else '; '.join(violations)


def same_environment(a, b):
    return all(a.get(k) == b.get(k) for k in FINGERPRINT_KEYS)


def compare_rows(ref, new, bitwise, rtol):
    problems = []
    if bitwise:
        if ref.get('trajectory_sha256') != new.get('trajectory_sha256'):
            problems.append('trajectory hash differs')
        return problems
    for key in EXACT:
        if ref.get(key) != new.get(key):
            problems.append(f'{key}: {ref.get(key)!r} -> {new.get(key)!r}')
    for key in NUMERIC:
        a, b = ref.get(key), new.get(key)
        if a is None or b is None:
            if a != b:
                problems.append(f'{key}: {a} -> {b}')
            continue
        if abs(a - b) > rtol*max(abs(a), abs(b)) + 1e-9:
            problems.append(f'{key}: {a:.6g} -> {b:.6g}')
    return problems


def step_reruns(quick, rtol, fingerprint):
    if not REFERENCE.exists():
        return False, f'reference missing: {REFERENCE.relative_to(ROOT)}', []
    reference = json.loads(REFERENCE.read_text(encoding='utf-8'))
    bitwise = same_environment(reference['environment'], fingerprint)
    rows = {(r['scenario_id'], r['controller']): r for r in reference['rows']}
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for scenario, controller in (RERUN_QUICK if quick else RERUN_FULL):
            if (scenario, controller) not in rows:
                results.append((scenario, controller, False, 'not in reference'))
                continue
            out = Path(tmp)/f'{scenario}_{controller}.json'
            proc, seconds = run([sys.executable, '-m', 'control.validation_suite',
                                 '--config', 'configs/arena.json', '--smoke',
                                 '--only-cases', scenario, '--only-controllers', controller,
                                 '--output', tmp, '--reference-out', str(out)], timeout=3600)
            if proc.returncode != 0 or not out.exists():
                results.append((scenario, controller, False,
                                f'rerun failed: {proc.stderr.strip().splitlines()[-1:]}'))
                continue
            new = json.loads(out.read_text(encoding='utf-8'))
            if new['config_sha256'] != reference['config_sha256']:
                results.append((scenario, controller, False, 'config changed since the reference'))
                continue
            problems = compare_rows(rows[(scenario, controller)], new['rows'][0], bitwise, rtol)
            results.append((scenario, controller, not problems,
                            ('bit-identical' if bitwise else f'within rtol {rtol:g}')
                            if not problems else '; '.join(problems)) + (seconds,))
    ok = all(r[2] for r in results)
    mode = 'same environment: bit-identical required' if bitwise else \
        f'different environment: rtol {rtol:g} + identical verdicts'
    return ok, mode, results


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--quick', action='store_true', help='skip heavy closed-loop checks')
    parser.add_argument('--rtol', type=float, default=1e-3,
                        help='cross-machine relative tolerance (provisional, see VERIFY.md)')
    args = parser.parse_args()
    started = time.perf_counter()
    table = []
    ok_env, detail, fingerprint = step_environment()
    table.append(('1 environment', ok_env, detail))
    ok_tests, detail = step_tests(args.quick)
    table.append(('2 fairness tests' + (' (quick)' if args.quick else ''), ok_tests,
                  f'{detail}, peak memory so far {peak_memory_gb():.2f} GiB'))
    ok_facts, detail = step_facts()
    table.append(('3 config vs confirmed facts', ok_facts, detail))
    ok_reruns, mode, results = step_reruns(args.quick, args.rtol, fingerprint)
    table.append(('4 smoke reruns', ok_reruns, mode))
    for row in results:
        scenario, controller, ok, detail = row[:4]
        seconds = f' ({row[4]:.0f}s)' if len(row) > 4 else ''
        table.append((f'   {scenario} / {controller}', ok, detail + seconds))
    minutes, memory = (time.perf_counter() - started)/60.0, peak_memory_gb()
    budget = BUDGET[args.quick]
    within = minutes <= budget['minutes'] and (budget['memory_gb'] is None
                                               or memory <= budget['memory_gb'])
    target = f'target {budget["minutes"]:g} min' + (f', {budget["memory_gb"]:g} GB' if budget['memory_gb']
                                                     else ', memory not budgeted') + ', 1 CPU'
    note = '; quick mode skips M17 NLP builds (~2 GB each), the full mode covers them' if args.quick else ''
    table.append(('5 resource budget', None if not within else True,
                  f'{minutes:.1f} min, peak memory {memory:.2f} GiB ({target}){note}'))
    width = max(len(name) for name, *_ in table)
    print('\n' + '='*100)
    for name, ok, detail in table:
        print(f'{name:<{width}}  {"PASS" if ok else ("WARN" if ok is None else "FAIL")}  {detail}')
    print('='*100)
    passed = ok_env and ok_tests and ok_facts and ok_reruns
    print('ALL PASS' if passed else 'SOME CHECKS FAILED')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
