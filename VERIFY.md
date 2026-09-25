# 경기장 검증 방법 (VERIFY)

대상은 `protkjj/fast-drone` 저장소의 `protkjj/arena-completion` 브랜치다. 외부 검토자가 GitHub에서
받아 **명령 하나로** 독립적으로 확인하는 절차다. 여기서 확인하는 것은 경기장이 기울지 않았는지와
결과가 재현되는지다. 제어기 성능 결론은 확인 대상이 아니다. 스모크 수치는 전부 **SMOKE**다.

## 0. 준비

- Python 3.10 이상, 패키지 `numpy`·`scipy`·`casadi`·`matplotlib`(`requirements.txt`)
- 저장소 루트에서 실행한다.
- 스레드 수는 스크립트가 하위 프로세스마다 1로 고정한다(`OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`,
  `VECLIB_MAXIMUM_THREADS`, `MKL_NUM_THREADS`). 병렬 합산 순서가 바뀌어 결과가 달라지는 것을 막기
  위해서다. pytest를 직접 돌릴 때도 같게 설정하기를 권한다.

## 1. 명령

```bash
python scripts/verify_arena.py           # 전체 — 30분 이내(작업지시서)
python scripts/verify_arena.py --quick   # CPU 1개·메모리 3 GB에서 20분 이내(kj 2026-09-26)
```

종료코드 0은 모두 통과, 1은 하나 이상 실패다. 마지막에 통과/실패 표를 출력하고, 맨 아래 줄
'5 resource budget'에 실제 걸린 시간과 최대 메모리를 적는다. 예산을 넘기면 WARN으로 표시하지만
종료코드는 바꾸지 않는다(느린 컴퓨터를 실패로 만들지 않기 위해).

| 모드 | 실측 시간(Apple M4, 단일 스레드, 다른 백그라운드 작업 2개와 동시) | 최대 메모리 | 결과 |
|---|---|---|---|
| `--quick` | 2.1분(공정성 테스트 69초 + 재실행 2건 57초) | 1.23 GiB | ALL PASS, 재실행 비트 일치 |
| 전체 | 11.8분(공정성 테스트 458초 + 재실행 7건 248초) | 3.81 GiB | ALL PASS, 재실행 7건 비트 일치 |

모든 단계가 차례로 돌고 하위 프로세스가 단일 스레드라, CPU 1개 환경에서도 걸리는 시간은 CPU 속도에만
비례한다. `--quick`은 CPU가 5배 느려도 약 10분이다.

## 2. 단계별 내용과 기대 결과

| 단계 | 내용 | 통과 조건 |
|---|---|---|
| 1 환경 | OS·CPU·Python·패키지 버전을 출력한다 | Python 3.10 이상 |
| 2 공정성 테스트 | `control/test_arena_fairness.py`(불변식 I-1~I-11)를 불변식 묶음별 새 프로세스로 돌린다(NLP 메모리가 한 프로세스에 쌓이지 않게) | 모든 묶음의 pytest 종료코드 0. **xfailed 1건은 기대된 결과다** — CPID 85 m/s는 설계 영역(0~20 m/s) 밖이라 트림 유지에 실패하는 것이 정상이다(strict: 예상과 달리 통과하면 실패로 뜬다). `--quick`이면 NMPC 폐루프 사례를 건너뛰고 **M17 NLP를 아예 짓지 않는다**(한 번 짓는 데 최대 ~1.9 GiB라 3 GB 예산에 여유가 없다). M17은 전체 모드에서 확인한다 |
| 3 확정 사항 | `configs/arena.json`을 작업지시서 1절 확정 사항과 대조한다(기체·질량·해시, 이륙 없음, 15/15/15초, 돌풍 절대 풍속·공통 트림 출발, V_L 20·V_H 85 m/s 등) | 위반 0건 |
| 4 스모크 재실행 | 커밋된 참조 `results/arena/smoke_reference.json`의 사례 일부를 새 프로세스에서 다시 돌려 비교한다 | 아래 3절 |

재실행 사례는 전체 모드에서 순항 측풍 돌풍(`gust_lateral_p10_VH`) × 5종과 통합 임무(`mission_VH`) ×
GSLQR·CPID다. `--quick` 모드는 순항 측풍 돌풍 × GSLQR·V13이다.

## 3. 허용 오차

- **같은 실행 환경이면 비트 일치.** 환경 지문(OS·아키텍처·CPU·Python·numpy/scipy/casadi/matplotlib 버전)이
  참조와 같으면 궤적 해시(`trajectory_sha256`: 상태·명령 배열의 바이트 해시)가 같아야 한다.
  결과는 벽시계와 무관하다. 솔버는 반복 횟수 상한만 쓰고 시간 제한을 두지 않는다.
- **다른 환경이면 상대오차 1e-3과 판정 일치.** RMSE·|ω|max·시뮬레이션 시간은 상대오차 1e-3
  이내여야 하고, 판정(`passed`·`tracking_pass`·`paper_failed`)과 정지사유는 같아야 한다.
  1e-3은 **잠정치**다. 다른 컴퓨터(Ubuntu)에서 한 번 돌려 보정할 예정이다. 보정 전에 이 단계만 실패하면
  "플랫폼 차이로 인한 수치 차이"일 수 있다. 판정이 같은지를 먼저 보고, 차이를 보고해 달라.
  바꾸려면 `--rtol`을 쓴다.

## 4. 결과 해석

- 통과는 경기장이 공정성 불변식을 지키고 참조 결과가 재현된다는 뜻이다. 어느 제어기가 낫다는 뜻은 아니다.
- 스모크 사례의 `passed`(스위트 판정)와 `paper_failed`(논문 §5.10 판정)는 따로 기록된다.
- GSLQR·CPID 행의 `integrators` 필드는 적분기가 한계에 닿아 있던 스텝과 포화로 적분이 멈춘 스텝이다.
  한계 도달이 시행의 1%를 넘으면 현황 보고서의 결정 필요로 올린다.
- 참조를 만든 실행 환경은 `smoke_reference.json`의 `environment`에 있다.

## 5. 관련 문서와 참조 갱신(유지보수자용)

- 현황 보고서: `results/ARENA_STATUS_2026-09-26.md`
- 설계영역 점검: `python -m control.arena_design_check` → `results/arena/DESIGN_CHECK.md`
- 적분기 한계 점검: `python -m control.arena_integrators --binding --history --hold` → `results/arena/INTEGRATORS.md`
- 스모크 전체와 참조 갱신(수 시간):
  ```bash
  python -m control.validation_suite --config configs/arena.json --smoke \
      --reference-out results/arena/smoke_reference.json
  ```
  설정(`configs/arena.json`)이나 제어기 코드가 바뀌면 참조를 새로 만들어 커밋해야 한다. 4단계는 설정
  해시가 참조와 다르면 실패로 알린다.
