# 남은 작업 — 우선순위 (브랜치 `protkjj/arena-completion`)

세션이 끊겨도 이 파일만 보고 이어갈 수 있게 적는다. **단계마다 갱신한다.**
상세 근거·수치는 `results/ARENA_STATUS_2026-09-26.md`(최신 17절).

최종 갱신: 2026-09-28 밤 (a·b 완료, c 준비 완료 — tune-final-3, d·e 완료, d3 보류, 다음은 e2(분산 도구)·f(튜닝 끝난 뒤))

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

## ⚠️ 2026-09-28 밤: 제어기 모델 모멘트 보정 — 튜닝 계획이 바뀌었다(보고서 23절)
- 제어기 집중정수 모델의 트림 밖 피치·요 모멘트가 플랜트와 크게 달랐고(85 m/s ±20°에서 181 rad/s², 40 m/s 부호 반대), M17·F13·GSLQR만 이 모델을 써서 V13 쪽으로 기울어져 있었다 → 보정항 추가(`configs/arena_v2.json`).
- **학교 F13·M17 `tune-final-3`은 옛 모델(arena.json)이라 본 실험에 쓸 수 없다.** 진행 상황은 kj가 학교에서 A.7로 확인해 보고, **중단은 kj가 확인한 뒤에** 한다(맥에서는 볼 수 없음).
- 재튜닝: **M17·F13·GSLQR → `tune-final-5`, 설정 `configs/arena_v2.json`**(태그는 검증·커밋 뒤, kj 확인 후 단다).
- 승계: CPID 완료(`configs/tuning_carryover.json`). **V13은 retune_v3이 끝나면** `python3 -m control.tuning_carryover --controller V13 --run-dir results/arena/tuning/retune_v3`.
- 맥 V13 retune_v3(arena.json)은 그대로 둔다 — V13은 보정 대상이 아니라 승계로 쓴다.
- **학교 태그는 `tune-final-6`**(실행 코드 = `tune-final-5`, A.3이 제어기 모델을 계수 rtol 1e-8로 비교 — 리눅스 sha 불일치 대응, 보고서 23.8절). 결과 폴더 `tune5`. 맥 GSLQR `tune5`는 `tune-final-5`로 진행 중.
- ⚠️ **`tune-final-4`(push됨)는 무효**: 보정항이 호버에서 미분 NaN → GSLQR 설계 실패(맥 `tune4` GSLQR 120회 전부 벌점, 쓰지 않음). 고친 코드는 `tune-final-5`(kj 확인 후 태그), 결과 폴더는 `tune5`. `tune4` 폴더는 격리(`tune4_invalid_nan`) 제안 — kj 확인 후.

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

### e. `configs/main_experiment.json` + `control/main_experiment.py` — [x] 작성·시험 완료, 실행 안 함(보고서 21절)
- kj 결정(2026-09-28, 21.1절):
  - 참조 15개: 가속@V_L·가속@V_H·제동@V_H × ρ 5단계. **제동@V_L 제외.**
  - 표 7: 17행 × 기준 5개(순항 V_L·V_H + ρ = 1.0 가속@V_L·가속@V_H·제동@V_H).
  - 통합 임무 V_H만, ρ 올림 dt, 사다리 4변형 × 4조건, 폴백·C2 제외, 기체 한계는 사전 확인 뒤 따로 분류.
- 가용 가속도: 넓히지 않음. ρ는 '트림 자세 근처(±0.3 rad)의 가용 가속도' 기준. 공력 모델은 큰 받음각에서도 신뢰 가능(공력 담당 확인) → 큰 받음각 제동이면 34~50 m/s 실제 가용 감속이 2~9배, 제동 ρ는 보수적(21.2절, 근거 정정).
- `python3 -m control.main_experiment --plan`: 726회, 맥 순차 추정 19.1 h(F13·M17 속도는 추정, Windows 미측정). 기체 한계 0, 포화 경계 V_H 4종.
- 가드: 튜닝 해시가 비어 **지금은 실행 거부**가 정상이다. f가 끝나면 `tuned.run_dir`과 제어기별 `sha256`을 채운다.
- 원고 갱신 목록(추가): 제동@V_L 제외 한계 문장, 가용 가속도 정의('트림 자세 근처 기준, 제동 ρ는 보수적 — 34~50 m/s 실제 가용 감속 2~9배', 넓히지 않는 이유 = 튜닝과 같은 정의 유지, 55 m/s 이상 탐색 한계), 제어기 모델과 플랜트의 큰 받음각 불일치(22절, 정보용), 표 7 ρ = 1.0 기준의 평가 창 길이 차이, 6자유도 트림(CG 행), +M_y = 기수 듦, 샘플링 500 Hz, 통합 임무 V_H만의 이유.

### e2. 본 실험 분산 도구 + `DISTRIBUTED_RUN.md` B절(PowerShell) — [x] 완료(보고서 24절)
- `control/main_distributed.py`: make-shards(LPT) · run-shard(시행별 원자 저장, 이어 돌리기, 해시 다르면 거부) · status · merge(누락·중복·해시·지문·플랫폼·코드 상태) · cross-select/compare(5%, 다른 컴퓨터, rtol 1e-3 + 판정).
- 맥 전 과정 점검 통과(`results/arena/e2_maccheck/REPORT.md`). **Windows에서는 아직 안 돌려 봄** — 첫 컴퓨터의 B.4·B.6이 절차 시험.
- 본 실험 태그 `<MAIN_TAG>`는 튜닝 확정·가드 통과 뒤 kj가 정한다. 그 전에 맥에서 `make-shards`로 `results/main/shards.json`을 만들어 태그에 넣는다.

### 팀원 확인 항목
- [x] **공력 모델의 받음각 유효 범위**(공력 담당, 2026-09-28 답): 큰 받음각에서도 신뢰할 수 있다 → 21.2절 근거 정정, 가용 가속도 계산·설정은 그대로.
- [x] **kj 판단**(2026-09-28 밤): 기울어진 경로가 맞다 → 보정항으로 고침(23절, arena_v2). M17·F13·GSLQR 재튜닝, V13·CPID 승계.

### 분석 단계에서 할 것
- 임무 ρ(t) 분리 보고(`control/arena_mission_rho.py`)에 '역유입 제약이 지배하는 구간(1~33 m/s 감속)'을 따로 표시한다. 임무 프로필은 바꾸지 않는다.
- 포화 경계 사례(공력 2배 0.969, 추력 0.7배 0.946, 공력 1.5배·CG y 0.941 — V_H)를 해석할 때 함께 인용한다.

### f0. tune-final-5 준비(M17·F13·GSLQR, arena_v2) — [ ]
- DISTRIBUTED_RUN.md A절을 tune-final-5·`--config configs/arena_v2.json`·새 run-dir 기준으로 고친다(설정 해시 `arena_v2`).
- env_check 기준 기록은 arena.json(V13) 그대로 쓸 수 있다(V13은 보정 대상 아님). 단 튜닝 명령은 반드시 `--config configs/arena_v2.json`.
- GSLQR도 재튜닝 대상이다(retune_v3 GSLQR 기록은 arena_v2에서 거부되는 것이 정상).

### f. V13·F13·M17 재튜닝이 끝난 뒤 — [ ]
- `configs/main_experiment.json`의 `tuned.run_dir`과 제어기별 `sha256`, `carryover.sha256`(V13 승계 뒤)을 채운다. V13·CPID는 retune_v3 기록, M17·F13·GSLQR은 tune-final-5 기록. 채운 뒤 `--plan`의 가드가 '통과'인지 확인한다.
- `arena_tune --summarize`: I-4 확인, 적분기 1%
- 설계점검: `arena_design_check --tuned …`
- 튜닝값 스모크
- 튜닝 기록 분석: `control/tuning_analysis.py`. A·B·C·first_nonfailed를 함께 인용한다(개선폭 비율만 쓰면 실패한 첫 평가에 유리하게 보인다).
- retune_v3 폴더 커밋(stdout 로그 *.log 는 관례대로 제외)

### g. 관문 통과 후 — [ ]
- control 브랜치로 병합, 태그 `main-exp-v1`. 병합과 태그 push 전에 kj 확인을 받는다.

## 완료 기록
- 2026-09-28 14:38: 관측 지연 버퍼(`add0ff5`), extra_params 배선(`f898a80`), 보고서 17절(`454349d`) push. 전체 회귀 213 passed, 1 xfailed.
