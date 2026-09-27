#!/usr/bin/env python3
"""분산 실행(경기장 튜닝·본 실험) 전 환경 점검 — kj 작업지시서 5단계.

여러 컴퓨터가 서로 다른 숫자를 내는 흔한 원인(패키지 버전 차이, 스레드 환경변수 미설정, 코드가
합의된 커밋이 아님, 설정·게인이 합의된 것이 아님)을 실행 전에 잡는다. **하나라도 어긋나면
거부한다**(kj 규칙: "불일치면 실행을 거부한다") — 단, 패키지 버전은 플랫폼이 다르면 정확히 같은
빌드를 못 구할 수 있어 WARN으로만 남긴다(최종 판정은 `scripts/cross_check.py`의 허용오차 비교가
한다, `scripts/verify_arena.py`의 기존 방침과 같다).

이 스크립트는 부모 셸의 환경변수를 바꿀 수 없다(별도 프로세스라서). 그래서 스레드 변수는
**이미 export돼 있는지 확인**만 한다 — `DISTRIBUTED_RUN.md`가 export 명령을 준다.

검사 항목
  1) 파이썬·패키지 버전 — requirements-lock.txt와 대조(다르면 WARN, 같은 OS·아키텍처면 FAIL)
  2) 스레드 환경변수 4개가 전부 '1'로 설정돼 있는지 (FAIL)
  3) git 커밋이 --commit(또는 결과 폴더 manifest의 git_revision)과 같고 dirty가 아닌지 (FAIL)
  4) --config가 있으면: 설정 sha256이 --expect-config-sha256과 같은지,
     제어기 모델(게인) sha256이 --expect-controller-model-sha256과 같은지 (FAIL)

실행: python3 scripts/setup_env.py --commit <sha> [--config configs/arena.json
        --expect-config-sha256 <sha> --expect-controller-model-sha256 <sha>]
      종료코드 0 = 실행해도 좋다. 1 = 거부(이유를 표에 적는다).
"""
import argparse
import platform
import subprocess
import sys
from importlib.metadata import version, PackageNotFoundError
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOCK = ROOT / 'requirements-lock.txt'
THREAD_VARIABLES = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')
# requirements-lock.txt를 실제로 만든 기준 컴퓨터(2026-09-27). 이 값이 바뀌면 락 파일도 새로 만들 것.
REFERENCE_PLATFORM = ('Darwin', 'arm64')


def read_lock(path=LOCK):
    """requirements-lock.txt를 {패키지: 버전} 으로. '#' 뒤는 주석, python 자체도 한 줄이다."""
    pins = {}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        line = line.split('#', 1)[0].strip()
        if not line or '==' not in line:
            continue
        name, ver = line.split('==', 1)
        pins[name.strip().lower()] = ver.strip()
    return pins


def check_versions(pins):
    """(통과여부, 항목별 결과). 파이썬 자체는 pins['python']과 platform.python_version()로.
    나머지는 importlib.metadata.version. 지금 컴퓨터가 락 파일을 만든 기준 플랫폼과 다르면
    못 맞아도 WARN(치명적이지 않음)이다 — 같은 플랫폼에서 다르면 FAIL이다."""
    same_platform = (platform.system(), platform.machine()) == REFERENCE_PLATFORM
    rows, ok = [], True
    installed = dict(python=platform.python_version())
    for name in pins:
        if name == 'python':
            continue
        try:
            installed[name] = version(name)
        except PackageNotFoundError:
            installed[name] = None
    for name, expected in pins.items():
        got = installed.get(name)
        match = (got == expected)
        severity = 'FAIL' if (not match and same_platform) else ('WARN' if not match else 'OK')
        rows.append((name, expected, got, severity))
        if severity == 'FAIL':
            ok = False
    return ok, rows


def check_threads():
    import os
    rows, ok = [], True
    for name in THREAD_VARIABLES:
        got = os.environ.get(name)
        good = got == '1'
        rows.append((name, '1', got, 'OK' if good else 'FAIL'))
        ok = ok and good
    return ok, rows


def check_git(expect_commit):
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'],
                                         cwd=ROOT, text=True).strip())
    rows = [('git dirty', 'False', str(dirty), 'OK' if not dirty else 'FAIL')]
    if expect_commit:
        match = revision == expect_commit
        rows.append(('git revision', expect_commit, revision, 'OK' if match else 'FAIL'))
    else:
        rows.append(('git revision', '(no --commit given, not checked)', revision, 'OK'))
    return (not dirty) and (not expect_commit or revision == expect_commit), rows


def check_config(config_path, expect_config_sha256, expect_controller_model_sha256):
    sys.path.insert(0, str(ROOT))
    from control.arena import load_config, config_sha256
    from control.arena_factory import ArenaFactory
    config = load_config(config_path)
    got_config = config_sha256(config)
    rows = [('config sha256', expect_config_sha256 or '(not given, not checked)', got_config,
             'OK' if (not expect_config_sha256 or got_config == expect_config_sha256) else 'FAIL')]
    ok = not expect_config_sha256 or got_config == expect_config_sha256
    if expect_controller_model_sha256:
        got_model = ArenaFactory(config).controller_model_sha256
        match = got_model == expect_controller_model_sha256
        rows.append(('controller_model sha256', expect_controller_model_sha256, got_model,
                     'OK' if match else 'FAIL'))
        ok = ok and match
    return ok, rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--commit', help='expected git revision (40-char sha); omit to skip this check')
    parser.add_argument('--config', help='arena config to check (e.g. configs/arena.json)')
    parser.add_argument('--expect-config-sha256')
    parser.add_argument('--expect-controller-model-sha256')
    args = parser.parse_args(argv)

    pins = read_lock()
    ok_v, rows_v = check_versions(pins)
    ok_t, rows_t = check_threads()
    ok_g, rows_g = check_git(args.commit)
    sections = [('1) 패키지 버전', rows_v, ok_v), ('2) 스레드 환경변수', rows_t, ok_t),
               ('3) git 상태', rows_g, ok_g)]
    overall = ok_t and ok_g and ok_v
    if args.config:
        ok_c, rows_c = check_config(args.config, args.expect_config_sha256, args.expect_controller_model_sha256)
        sections.append(('4) 설정·게인 해시', rows_c, ok_c))
        overall = overall and ok_c

    print(f'환경: {platform.system()} {platform.machine()} · Python {platform.python_version()}', flush=True)
    for title, rows, ok in sections:
        print(f'\n{title}: {"PASS" if ok else "FAIL/WARN 있음"}')
        for name, expected, got, severity in rows:
            print(f'  [{severity:4s}] {name}: 기대={expected} 실측={got}')
    print(f'\n{"=" * 60}')
    print('실행해도 좋음(PASS)' if overall else '거부 — 위 FAIL을 고친 뒤 다시 확인할 것')
    return 0 if overall else 1


if __name__ == '__main__':
    raise SystemExit(main())
