#!/usr/bin/env python3
"""경기장 검증 — 외부 검토자가 명령 하나로 돌린다 (작업지시서 작업 F).

    python scripts/verify_arena.py            # 전체 (목표 30분 이내)
    python scripts/verify_arena.py --quick    # 무거운 사례 생략 (목표 10분 이내)

차례로 하는 일(각 단계 통과/실패를 표로 출력, 하나라도 실패하면 종료코드 1):
  1) 실행 환경 요약 — OS·CPU·파이썬·패키지·스레드 설정
  2) 공정성 불변식 테스트 control/test_arena_fairness.py (I-1 ~ I-11)
  3) 설정 파일과 1절 확정 사항 대조(check_confirmed_facts)
  4) 스모크 사례 일부를 **새 프로세스에서** 다시 돌려 커밋된 참조
     (results/arena/smoke_reference.json)와 비교
       - 실행 환경 지문이 참조와 같으면: 궤적 해시가 비트 단위로 같아야 한다
       - 다르면: 수치 지표 상대오차 ≤ --rtol (기본 1e-3, 잠정치),
         그리고 판정(passed·paper_failed)·정지사유가 같아야 한다

스레드 수는 1로 고정해 하위 프로세스를 띄운다(BLAS 병렬 합산 순서가 결과를 바꾸지
않게). 결과는 시뮬레이션 설정의 함수이지 컴퓨터 속도의 함수가 아니다.
"""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import time

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


def step_tests(quick):
    cmd = [sys.executable, '-m', 'pytest', 'control/test_arena_fairness.py', '-q', '-p',
           'no:cacheprovider']
    e = env()
    if quick:
        e['ARENA_QUICK'] = '1'
    started = time.perf_counter()
    proc = subprocess.run(cmd, cwd=ROOT, env=e, capture_output=True, text=True, timeout=3600)
    last = [line for line in proc.stdout.strip().splitlines() if line.strip()][-1:]
    return proc.returncode == 0, f'{(last or ["no output"])[0]} ({time.perf_counter()-started:.0f}s)'


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
    table = []
    ok_env, detail, fingerprint = step_environment()
    table.append(('1 environment', ok_env, detail))
    ok_tests, detail = step_tests(args.quick)
    table.append(('2 fairness tests' + (' (quick)' if args.quick else ''), ok_tests, detail))
    ok_facts, detail = step_facts()
    table.append(('3 config vs confirmed facts', ok_facts, detail))
    ok_reruns, mode, results = step_reruns(args.quick, args.rtol, fingerprint)
    table.append(('4 smoke reruns', ok_reruns, mode))
    for row in results:
        scenario, controller, ok, detail = row[:4]
        seconds = f' ({row[4]:.0f}s)' if len(row) > 4 else ''
        table.append((f'   {scenario} / {controller}', ok, detail + seconds))
    width = max(len(name) for name, *_ in table)
    print('\n' + '='*100)
    for name, ok, detail in table:
        print(f'{name:<{width}}  {"PASS" if ok else "FAIL"}  {detail}')
    print('='*100)
    passed = ok_env and ok_tests and ok_facts and ok_reruns
    print('ALL PASS' if passed else 'SOME CHECKS FAILED')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
