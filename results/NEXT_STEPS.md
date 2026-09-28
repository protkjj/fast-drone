# 남은 작업 — 우선순위 (브랜치 `protkjj/arena-completion`)

세션이 끊겨도 이 파일만 보고 이어갈 수 있게 적는다. **단계마다 갱신한다.**
상세 근거·수치는 `results/ARENA_STATUS_2026-09-26.md`(최신 17절).

최종 갱신: 2026-09-28 밤 (a·b 완료, c 준비 완료 — tune-final-3, d 완료, d3 보류, 다음은 e)

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

## 어디서 돌리나 (kj 2026-09-28)
- **학교 Windows**: F13·M17 튜닝(`tune-final-3`), **본 실험 전체**. DISTRIBUTED_RUN.md B절도 PowerShell 기준으로 써야 한다.
- **맥**: 코드 작업(d·e), 튜닝 합치기와 마무리(f), 병합(g), 분석.
- d·e는 튜닝 결과를 기다리지 않는다. 지금 시작할 수 있다.
  - 학교 튜닝은 태그 코드로 돌므로, 맥에서 코드를 바꿔도 영향이 없다.
  - 단 d 뒤에는 기본값에서 결과가 그대로인지 `arena_tune_repro`로 확인한다.
- **학교 튜닝이 도는 동안 `configs/arena.json`을 바꾸지 않는다.** 설정 해시가 바뀌면 재개 가드와 I-4가 깨진다. 본 실험 설정은 별도 파일(e)로 만든다.

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

### b. 태그 — [x] 완료(tune-final·tune-final-2 push, kj 승인)
- 결과(2026-09-28, 보고서 18절): `control/arena_tune_repro.py`로 retune_v3 평가 0을 HEAD와 병렬 경로에서 재현했다. V13(N=3)·GSLQR·CPID 모두 비트 동일했다. 반대 방향 시험(기록 조작)은 MISMATCH였다.
- V13 병렬(N=3)은 393 s로 순차의 약 2배 빠르다. 작업자 1개 약 550 MiB다.
- 로컬 태그: `tune-final` → `8911ee1`, `tune-final-2` → 이 결과를 기록한 커밋.
- push 명령(확인 후): `git push origin tune-final tune-final-2`
- `tune-final` → `8911ee1`(retune_v3 시작 커밋). a와 무관하게 지금 달 수 있다.
- `tune-final-2` → 병렬 옵션을 추가한 커밋. **결과 무영향 확인 뒤**에 단다.
  - 확인할 것: 같은 설정 해시(`1376310b…`)에서 N=1과 N>1이 비트 동일한지.
  - 확인할 것: tune-final과 비교해 튜닝 평가값이 같은지(8911ee1 이후 `run_trial`에 build_plant와 관측 지연이 들어갔다. 기본 꺼짐이면 비트 동일하다는 주장을 **튜닝 경로에서 실측**할 것).
- 태그 push는 kj 확인 뒤에 한다.

### c. F13·M17 재튜닝, 학교 컴퓨터(**Windows**)에서 **tune-final-3**으로 — [~] 준비 완료, 실행은 kj
- kj 결정(2026-09-28): 새 태그 `tune-final-3`을 쓴다. 기존 태그는 옮기지 않는다.
  - 제어기별 태그: V13·GSLQR·CPID는 `tune-final`, F13·M17은 `tune-final-3`(보고서 19.1절).
- 준비한 것(보고서 19절):
  - 재현 도구의 rtol 1e-3 + 판정 일치 합격 판정
  - 긴 시나리오 먼저 배정
  - 고아 작업자 방지(반대 방향 실험으로 확인)
  - Windows 절전 방지
  - `arena_suggest_workers`
  - `env_check/` 기준 기록(V13 평가 0)
- 작업자당 메모리: V13 0.6 GiB, F13 1.2 GiB(실측 0.84 GiB), M17 2.0 GiB.
- 절차는 `DISTRIBUTED_RUN.md` A절(Windows PowerShell)에 있다. **Windows에서는 아직 한 번도 돌려 보지 않았다.** A.4(재현)와 A.5(고아 점검)가 처음 시험이다.
- 끝나면 A.10(맥에서 합치기 + I-4)을 한다.
- `DISTRIBUTED_RUN.md` A절(튜닝 분산)이 이미 있다. 이를 tune-final-2 태그 checkout 기준으로 고친다.
  - 설정 해시 확인과 `--scenario-workers` 권장값을 추가한다.
  - run-dir 이름은 retune_v3와 합칠 수 있게 정한다.
- 실행은 kj가 학교 컴퓨터에서 한다.

### d. 표7 나머지 배선 — [x] 완료(d1·d2 + 6자유도 트림, 보고서 20절). 자이로 진동(d3)은 **보류**(kj)
- d1 외력·모멘트: `extra_force_world`, `extra_force_start_s`, `extra_moment_start_s` 추가. 정수 스텝 시간창 [start, start+duration).
  - kj 확정: t=3 s 시작, 5 s. 외력 세계 +y(0.7g·1.4g = 8.062·16.125 N), 모멘트 동체 +M_y(0.234·0.468 N·m). 부호는 결과 보기 전에 정함.
  - +M_y는 V_L·V_H 모두 **기수를 든다**(기하 계산·시뮬레이션 일치).
- d2 무게중심 편차: `extra_params`의 `cg_offset_axis`, `cg_offset_arm_fraction`를 `run_trial`에 연결(플랜트만).
- 발견: 벤더 `find_trim`은 평면(좌우 대칭) 탐색기라 CG y(전 속도)와 호버 CG z 트림을 표현하지 못한다. 트림은 물리적으로 존재한다.
  - kj 결정 → `control/arena_trim.py::find_trim_6dof`(롤 0 → 옆미끄럼 0, 둘 다 없으면 'vehicle limit'). CG 사례에만 쓰고 `plant_trim_condition`을 기록한다.
  - 명목 순항은 벤더와 비트 동일, 호버는 5e-12.
- d3 자이로 진동: 보류. 샘플링이 1 kHz가 아니라 500 Hz라 85 m/s에서 1·2차가 모두 INDI 대역으로 에일리어싱된다(20.5절).
- 확인: 전체 회귀 240 passed, 1 xfailed. `arena_tune_repro` GSLQR·CPID·V13 평가 0 모두 PASS·**비트 동일**(20.6절).

### e. `configs/main_experiment.json` 작성 — [ ] kj 답 받음(2026-09-28), 다음 작업(실행하지 않는다)
- kj 답 요지:
  - 표7 18행(한 번에 하나씩 플랜트에만): 질량 ±30%, 추력 계수 ±30%, 동체 공력 +50%·+100%, CG 동체 y·z +10%(팔 길이),
    로터 면내 항력 1배(V_H 보정), 일정 외력 0.7g·1.4g(5 s), 외부 모멘트 25%·50%(5 s), 상태 지연 10·30 ms,
    모터 시간 상수 2배, 회전수 지연 5 ms, 자이로 진동 1배(**d3 보류 중** — 이 행은 d3 뒤에).
  - 섭동마다 V_L·V_H 트림 존재를 먼저 확인하고, 없으면 '기체 한계'로 따로 분류한다.
    - **구현 주의(20.4절)**: 주 실행기는 `run_trial`의 ValueError를 `setup_error`(제어기 실패)로 센다. 실행 **전** 트림 확인 →
      `excluded_from`로 건너뛰고 `skip_reason='vehicle_limit'`. CG 행은 `find_trim_6dof`, 나머지는 벤더 `find_trim(strict=False)`.
  - 외력·모멘트 평가 창: 인가 구간 전체 + 해제 뒤 7 s를 포함(t=3 시작 → 창 끝 ≥ 15 s).
  - 참조 프로필 20개: 기동 2종(가속·제동) × {V_L=20, V_H=85} × ρ {0.5, 0.8, 1.0, 1.2, 1.5}. 고도 20 m, 식(32) smoothstep, 전이 시간은 arena.py ρ 규칙.
    - 가속@V_L 호버→20, 가속@V_H 호버→85, 제동@V_L 20→호버, 제동@V_H 85→20.
  - 사다리: V13-0=A0C0S0, V13-1=A1C0S0, V13-2=A1C1S0, V13=A1C1S1. 모두 V13 튜닝값(표준 제거 실험, 변형별 미튜닝을 한계로 기록).
    조건: 참조 20개 × {기준, 모터 τ 2배, 회전수 지연 5 ms, 상태 지연 30 ms}.
  - 완성 시스템(V13+폴백) 제외. C2 제외.
  - 원고 갱신 목록: CG 2방향(18행), 외력 방향, 외부 모멘트 정의(호버 최대 피치 모멘트 기준), 자이로 진동 가정값, 기동 2종, C2 제외, 폴백 제외, 외란 시작 시각.
    **추가**: 6자유도 트림(CG 행), +M_y = 기수 듦, 샘플링 500 Hz.
- 미결(e 착수 때 확인): 표7 외란·모델 행을 어느 프로필에 거는지(V_L·V_H 순항으로 추정 — 답 4의 "V_L·V_H 트림" 문구). 돌풍·통합 임무는 기존 설계 그대로.
- 설계(유지):
  - `main_experiment.json`에 기준 설정(`arena.json`) 해시, 튜닝값 파일 해시, 태그를 적는다. 불일치면 실행 거부, 튜닝 전에는 빈 해시라 실행 거부.
  - `arena.json`의 해시가 안 바뀌는지 시험으로 확인한다. `validate_config` 통과를 시험하고 실행은 하지 않는다.
- 본 실험은 전부 Windows에서 한다. 교차 확인도 다른 Windows 컴퓨터로. DISTRIBUTED_RUN.md B절을 PowerShell로 쓴다.
- 돌풍: V_L·V_H × (측풍 5, 측풍 10, 수직풍 ±5). 통합 임무: ρ 분리 보고(`control/arena_mission_rho.py` 재사용).

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
