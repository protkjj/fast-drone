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
  4) --config가 있으면: 설정 sha256이 --expect-config-sha256과 같은지 (FAIL),
     그 컴퓨터에서 적합한 제어기 모델 집중정수 계수(C_Na·C_dc·C_A0·C_Aa2·x_cp·C_lp·C_mq, x_cp(V) 다항식)가
     맥 기준값(configs/controller_model_reference.json)과 rtol 1e-8 안인지 (FAIL).
     제어기 모델 sha256(--expect-controller-model-sha256)은 **기록만** 한다(INFO) — 2026-09-28 리눅스 실측:
     계수 적합의 부동소수 차이로 sha가 달라졌다(2582197d… → 10b40c8c…). 실행 경로는 여전히 그 컴퓨터에서
     적합한 값을 쓴다 — 기준 파일은 비교 전용이다.

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
# 파이썬 자체는 pip 설치 대상이 아니라 requirements-lock.txt에 안 넣고 여기서 따로 관리한다.
REFERENCE_PYTHON = '3.13.7'
MODEL_REFERENCE = ROOT/'configs'/'controller_model_reference.json'
MODEL_KEYS = ('C_Na', 'C_dc', 'C_A0', 'C_Aa2', 'x_cp', 'C_lp', 'C_mq')      # fit_lumped_aero가 적합하는 값
MODEL_RTOL = 1e-8


def model_coefficients(cp):
    """그 컴퓨터에서 적합한 제어기 모델 계수(비교 대상). x_cp_poly는 fit_cp_schedule 결과."""
    out = {k: float(cp[k]) for k in MODEL_KEYS}
    out['x_cp_poly'] = [float(c) for c in cp['x_cp_poly']]
    return out


def write_model_reference(path=MODEL_REFERENCE):
    """기준 컴퓨터(REFERENCE_PLATFORM)에서만 만든다. 실행 경로는 이 파일을 읽지 않는다."""
    import json
    sys.path.insert(0, str(ROOT))
    from control.arena_factory import ControllerModel
    from control.validation_suite import baseline_params
    model = ControllerModel(baseline_params())
    doc = dict(schema='controller_model_reference/1', purpose=(
        '비교 전용 — scripts/setup_env.py가 다른 컴퓨터의 적합 계수를 rtol 1e-8로 대조한다. 실행 경로는 이 파일을 '
        '읽지 않고 그 컴퓨터에서 적합한 값을 쓴다.'),
        made_on=list(REFERENCE_PLATFORM), controller_model_sha256=model.sha256,
        coefficients=model_coefficients(model.cp))
    Path(path).write_text(json.dumps(doc, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    return doc


def compare_model(cp, reference):
    """(합격, 행 목록). 계수마다 상대오차 = |a−b| / max(|a|,|b|) (0 대 0은 0)."""
    import numpy as np
    got, ref = model_coefficients(cp), reference['coefficients']
    rows, ok = [], True
    for key in MODEL_KEYS + ('x_cp_poly',):
        a, b = np.atleast_1d(np.asarray(got[key], float)), np.atleast_1d(np.asarray(ref[key], float))
        if a.shape != b.shape:
            rows.append((f'model {key}', f'{b.size}개', f'{a.size}개', 'FAIL'))
            ok = False
            continue
        scale = np.maximum(np.abs(a), np.abs(b))
        rel = float(np.max(np.where(scale > 0, np.abs(a - b)/np.where(scale > 0, scale, 1.0), 0.0)))
        good = rel <= MODEL_RTOL
        ok = ok and good
        rows.append((f'model {key}', f'맥 기준 ±rtol {MODEL_RTOL:g}', f'최대 상대오차 {rel:.2e}', 'OK' if good else 'FAIL'))
    return ok, rows


def read_lock(path=LOCK):
    """requirements-lock.txt를 {패키지: 버전} 으로. '#' 뒤는 주석. 파이썬 자체는 이 파일에 없다
    (pip 설치 대상이 아니라서) — REFERENCE_PYTHON으로 따로 확인한다."""
    pins = {}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        line = line.split('#', 1)[0].strip()
        if not line or '==' not in line:
            continue
        name, ver = line.split('==', 1)
        pins[name.strip().lower()] = ver.strip()
    return pins


def check_versions(pins):
    """(통과여부, 항목별 결과). importlib.metadata.version으로 패키지를 본다.

    파이썬 자체는 따로 본다: **주(main.minor)** 버전이 다르면(3.13 vs 3.12 등) 플랫폼과 무관하게
    FAIL이다(언어 자체 동작이 달라질 수 있어서). 패치 버전(3.13.7 vs 3.13.2)만 다르면 지금
    컴퓨터가 기준 플랫폼과 같을 때만 FAIL, 다르면 WARN이다 — 나머지 패키지도 같은 규칙이다.
    """
    same_platform = (platform.system(), platform.machine()) == REFERENCE_PLATFORM
    rows, ok = [], True

    py_got = platform.python_version()
    if py_got == REFERENCE_PYTHON:
        py_severity = 'OK'
    elif tuple(py_got.split('.')[:2]) != tuple(REFERENCE_PYTHON.split('.')[:2]):
        py_severity = 'FAIL'                        # 주(main.minor) 버전이 다르면 플랫폼 무관 FAIL
    else:
        py_severity = 'FAIL' if same_platform else 'WARN'   # 패치만 다름: 기준 플랫폼에서만 FAIL
    rows.append(('python', REFERENCE_PYTHON, py_got, py_severity))
    if py_severity == 'FAIL':
        ok = False

    installed = {}
    for name in pins:
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
    import json
    factory = ArenaFactory(config)          # 모멘트 보정 설정이면 모델 파일 sha도 여기서 확인된다
    reference = json.loads(MODEL_REFERENCE.read_text(encoding='utf-8'))
    ok_model, rows_model = compare_model(factory.cp, reference)
    rows += rows_model
    ok = ok and ok_model
    got_model = factory.controller_model_sha256
    expect = expect_controller_model_sha256 or reference['controller_model_sha256']
    rows.append(('controller_model sha256(기록만)', expect, got_model,
                 'INFO' if got_model != expect else 'OK'))
    return ok, rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--commit', help='expected git revision (40-char sha); omit to skip this check')
    parser.add_argument('--config', help='arena config to check (e.g. configs/arena.json)')
    parser.add_argument('--expect-config-sha256')
    parser.add_argument('--expect-controller-model-sha256', help='기록만 한다(INFO) — 판정은 계수 비교')
    parser.add_argument('--write-model-reference', action='store_true',
                        help='기준 컴퓨터에서만: configs/controller_model_reference.json을 새로 만든다')
    args = parser.parse_args(argv)
    if args.write_model_reference:
        if (platform.system(), platform.machine()) != REFERENCE_PLATFORM:
            parser.error(f'the model reference is made only on {REFERENCE_PLATFORM}')
        doc = write_model_reference()
        print(f"wrote {MODEL_REFERENCE} (controller_model_sha256 {doc['controller_model_sha256'][:12]}…)")
        return 0

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
