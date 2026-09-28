# 남은 작업 — 우선순위 (브랜치 `protkjj/arena-completion`)

세션이 끊겨도 이 파일만 보고 이어갈 수 있게 적는다. **단계마다 갱신한다.**
상세 근거·수치는 `results/ARENA_STATUS_2026-09-26.md`(최신 17절).

최종 갱신: 2026-09-28 (b 완료, 태그는 로컬만)

## 지금 돌고 있는 것 — 건드리지 않는다

- **V13 재튜닝 retune_v3**: PID 59326, 2026-09-28 10:08 시작, 14:36 기준 23/120.
  - 평가 1회 약 13분(튜닝 시나리오 18개) → 약 26시간 예상, 9/29 12시 전후 종료. kj 확인: 정상 속도.
  - 시작 커밋 `8911ee1`(git_dirty=False), config_sha256 `1376310b…`(정규화 JSON 해시. 원시 파일 sha와 다르니 주의).
  - 진행 확인: `tail -3 results/arena/tuning/retune_v3_V13.log`, `ps aux | grep arena_tune | grep -v grep`
  - 멈췄을 때(재개 가드가 설정 해시·시나리오 id를 확인함). **실행 전 kj 확인 필요**:
    `caffeinate -dims env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 python3 -m control.arena_tune --controllers V13 --budget 120 --run-dir results/arena/tuning/retune_v3 > results/arena/tuning/retune_v3_V13.log 2>&1 &`
    (로그를 덮어쓰지 않으려면 `>>`로)
  - CPID·GSLQR retune_v3는 끝남(11:53, 11:17).
  - **커밋 보류**: `retune_v3/` 폴더와 `*.log`는 V13이 끝난 뒤 한꺼번에(kj 결정). 로그는 기존 관례대로 커밋 안 함.

## 우선순위 목록

### a. 튜닝 평가 안의 시나리오 병렬화 `--scenario-workers N` — [x] 구현·시험 완료 (V13 속도 실측은 b에서)
- 결과(2026-09-28): `control/arena_tune.py`의 `Evaluator(scenario_workers=N)`와 CLI `--scenario-workers`. spawn 작업자 풀은 튜닝 동안 유지하고, 작업자마다 제어기 모델 sha256을 부모와 대조한다.
- 시험은 `control/test_arena_tune_parallel.py` 7개로, 모두 통과했다(11분 40초).
  - CPID·GSLQR의 시나리오 18개 전부 × (사전값, 다른 후보)에서 목적함수와 시나리오별 항목(궤적 sha256 포함)이 비트 동일했다.
  - `tune_controller`가 쓰는 `.jsonl`도 바이트 동일했다.
- 속도(GSLQR, 작업자 3개, V13 튜닝과 동시 실행): 순차 33.5~41.9 s, 병렬 첫 호출 47.7 s(작업자 기동 포함), 둘째 호출 20.8 s. 약 1.6~2배.
- 주의: spawn 방식이라 표준입력(`python3 - <<EOF`)으로 넣은 스크립트에서는 작업자가 뜨지 못한다. `-m` 모듈 실행이나 파일 스크립트로 돌릴 것.
- 평가 1회 = 시나리오 18개를 **순차로** 돈다. 이를 N개 프로세스로 나눠 돌리는 옵션이다.
- **기본값은 순차(N=1)**다. 기본 경로는 지금과 비트 동일해야 한다.
- 시험: N=1 결과와 N>1 결과가 **비트 동일**해야 한다(목적함수, 시나리오별 지표, 기록 순서).
- 주의:
  - 메모리 예산(kj 기준 3 GB): M17 NLP 1회가 1.9 GiB다.
  - 성능 코어는 4개다. 무거운 단일 스레드가 5개 이상이면 약 2배 느려진다.
  - 병렬로 돌면 튜닝 프로세스 수 × N이 코어를 두고 경쟁한다. 기본값을 순차로 두는 이유다.

### b. 태그 — [x] 로컬 태그 완료, **push는 kj 확인 대기**
- 결과(2026-09-28, 보고서 18절): `control/arena_tune_repro.py`로 retune_v3 평가 0을 HEAD와 병렬 경로에서 재현했다. V13(N=3)·GSLQR·CPID 모두 비트 동일했다. 반대 방향 시험(기록 조작)은 MISMATCH였다.
- V13 병렬(N=3)은 393 s로 순차의 약 2배 빠르다. 작업자 1개 약 550 MiB다.
- 로컬 태그: `tune-final` → `8911ee1`, `tune-final-2` → 이 결과를 기록한 커밋.
- push 명령(확인 후): `git push origin tune-final tune-final-2`
- `tune-final` → `8911ee1`(retune_v3 시작 커밋). a와 무관하게 지금 달 수 있다.
- `tune-final-2` → 병렬 옵션을 추가한 커밋. **결과 무영향 확인 뒤**에 단다.
  - 확인할 것: 같은 설정 해시(`1376310b…`)에서 N=1과 N>1이 비트 동일한지.
  - 확인할 것: tune-final과 비교해 튜닝 평가값이 같은지(8911ee1 이후 `run_trial`에 build_plant와 관측 지연이 들어갔다. 기본 꺼짐이면 비트 동일하다는 주장을 **튜닝 경로에서 실측**할 것).
- 태그 push는 kj 확인 뒤에 한다.

### c. F13·M17 재튜닝, 학교 컴퓨터에서 tune-final-2로 — [ ] 다음 작업
- `DISTRIBUTED_RUN.md` A절(튜닝 분산)이 이미 있다. 이를 tune-final-2 태그 checkout 기준으로 고친다.
  - 설정 해시 확인과 `--scenario-workers` 권장값을 추가한다.
  - run-dir 이름은 retune_v3와 합칠 수 있게 정한다.
- 실행은 kj가 학교 컴퓨터에서 한다.

### d. 표7 나머지 배선 — [ ]
- 일정 외력과 외부 모멘트를 5초간 건다. 추가 힘·모멘트 훅(`control/arena_plant_wrench.py`)을 쓴다.
  - 25%·50% 모멘트 기준은 `hover_max_pitch_moment` 0.9366 N·m다(17.2절).
- 자이로 진동: 센서 옵션, 기본 꺼짐. 관측 지연처럼 측정값 공급 지점에서 모든 제어기에 똑같이 준다.
- 둘 다 꺼짐이 비트 동일한지 시험한다. 회귀 전체를 실행한다.

### e. `configs/main_experiment.json` 작성 — [ ] (실행하지 않는다)
- 참조 프로필 20개
- 표7 18종
- 돌풍: V_L·V_H × (측풍 5, 측풍 10, 수직풍 ±5)
- 통합 임무: ρ 분리 보고(`control/arena_mission_rho.py` 재사용)
- 사다리: V13-0 / V13-1 / V13-2 / V13
- `validate_config` 통과를 시험한다. 실행은 하지 않는다.

### f. V13·F13·M17 재튜닝이 끝난 뒤 — [ ]
- `arena_tune --summarize`: I-4 확인, 적분기 1%
- 설계점검: `arena_design_check --tuned …`
- 튜닝값 스모크
- 튜닝 기록 분석: `control/tuning_analysis.py`. A·B·C·first_nonfailed를 함께 인용한다(개선폭 비율만 쓰면 실패한 첫 평가에 유리하게 보인다).
- retune_v3 폴더 커밋(stdout 로그 *.log 는 관례대로 제외)

### g. 관문 통과 후 — [ ]
- control 브랜치로 병합, 태그 `main-exp-v1`. 병합과 태그 push 전에 kj 확인을 받는다.

## 완료 기록
- 2026-09-28 14:38: 관측 지연 버퍼(`add0ff5`), extra_params 배선(`f898a80`), 보고서 17절(`454349d`) push. 전체 회귀 213 passed, 1 xfailed.
