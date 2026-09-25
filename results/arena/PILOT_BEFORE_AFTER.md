# PILOT 전후 비교 — 제어기 모델 수정(프로펠러 곡선·x_cp(V)) 전후

**PILOT** — 예비 수치의 변화 기록. 우위·열위 결론 없음.

이전 값 출처: `results/SOLVER_FAILURE_REPORT_2026-09-25.md`(선형 프로펠러·상수 x_cp 시절). 재현: `python -m control.pilot_before_after --parts discriminate mission timing` · git `cb1841c93fdadb78ff94bdc2be032bf453e04451` dirty=True

## discriminate

이후 값 계산: git `cb1841c93fdadb78ff94bdc2be032bf453e04451` dirty=True

| 사례 | 시점 | 완주 | 추종통과 | 시간 s | z RMSE | v RMSE | \|ω\|max | 포화 % | 솔버실패 | 정지사유 |
|---|---|---|---|---:|---:|---:|---:|---:|---:|---|
| default@80 | 이전 | False | — | — | 0.6974 | 13.12 | — | 91.4 | 2/50 | velocity error limit |
| default@80 | current | True | False | 4.00 | 2.371 | 11.43 | 18.9 | 95.8 | 2/200 |  |
| default@80 | propeller_only | True | False | 4.00 | 2.371 | 11.43 | 18.9 | 95.8 | 2/200 |  |
| default@85 | 이전 | False | — | — | 0.6945 | 14.35 | — | 91.2 | 3/51 | velocity error limit |
| default@85 | current | False | False | 2.11 | 1.389 | 13.81 | 21.1 | 92.9 | 2/106 | velocity error limit |
| default@85 | propeller_only | False | False | 2.11 | 1.389 | 13.81 | 21.1 | 92.9 | 2/106 | velocity error limit |
| trim@80 | 이전 | False | — | — | 0.6975 | 13.12 | — | 91.2 | 2/50 | velocity error limit |
| trim@80 | current | True | False | 4.00 | 2.31 | 11.43 | 18.9 | 96.6 | 2/200 |  |
| trim@80 | propeller_only | True | False | 4.00 | 2.31 | 11.43 | 18.9 | 96.6 | 2/200 |  |
| trim@85 | 이전 | False | — | — | 0.6982 | 14.36 | — | 91.5 | 2/51 | velocity error limit |
| trim@85 | current | False | False | 1.88 | 1.447 | 14 | 19.2 | 89.4 | 2/95 | velocity error limit |
| trim@85 | propeller_only | False | False | 1.88 | 1.447 | 14 | 19.2 | 89.4 | 2/95 | velocity error limit |

## mission

이후 값 계산: git `cb1841c93fdadb78ff94bdc2be032bf453e04451` dirty=True

| 사례 | 시점 | 완주 | 추종통과 | 시간 s | z RMSE | v RMSE | \|ω\|max | 포화 % | 솔버실패 | 정지사유 |
|---|---|---|---|---:|---:|---:|---:|---:|---:|---|
| V13@70 | 이전 | True | — | 65.00 | 0.524 | 2.869 | 11.2 | 6.5 | 2/3250 |  |
| V13@70 | 이후 | True | — | 65.00 | 0.2849 | 2.749 | 10.6 | 5.8 | 2/3250 |  |
| V13@80 | 이전 | True | — | 65.00 | 1.307 | 3.538 | 16.2 | 7.5 | 2/3250 |  |
| V13@80 | 이후 | True | — | 65.00 | 0.6279 | 3.608 | 13.3 | 7.5 | 2/3250 |  |
| V13@85 | 이전 | True | — | 65.00 | 0.833 | 3.267 | 12.6 | 8.1 | 2/3250 |  |
| V13@85 | 이후 | True | — | 65.00 | 0.2685 | 4.017 | 13.9 | 7.1 | 2/3250 |  |

## timing

이후 값 계산: git `cb1841c93fdadb78ff94bdc2be032bf453e04451` dirty=True

| 사례 | 시점 | 완주 | 추종통과 | 시간 s | z RMSE | v RMSE | \|ω\|max | 포화 % | 솔버실패 | 정지사유 |
|---|---|---|---|---:|---:|---:|---:|---:|---:|---|
| original@70 | 이전 | True | — | 55.00 | 0.36 | 1.852 | 10 | 7.7 | 2/2750 |  |
| original@70 | 이후 | True | — | 55.00 | 0.2653 | 1.997 | 10.5 | 6.6 | 1/2750 |  |
| original@80 | 이전 | True | — | 55.00 | 0.217 | 1.771 | 16 | 8.3 | 4/2750 |  |
| original@80 | 이후 | False | — | 40.98 | 1.554 | 3.484 | 15.3 | 10.2 | 7/2050 | 5 consecutive optimizer failures |
| original@85 | 이전 | True | — | 55.00 | 2.374 | 4.37 | 14.6 | 14.6 | 15/2750 |  |
| original@85 | 이후 | True | — | 55.00 | 0.2548 | 3.758 | 13.8 | 8.7 | 2/2750 |  |
| relaxed@70 | 이전 | True | — | 40.00 | 0.136 | 1.275 | 12.1 | 8.6 | 2/2000 |  |
| relaxed@70 | 이후 | True | — | 40.00 | 0.183 | 1.126 | 13.4 | 6.1 | 2/2000 |  |
| relaxed@80 | 이전 | True | — | 40.00 | 0.414 | 1.841 | 15.4 | 12.7 | 5/2000 |  |
| relaxed@80 | 이후 | True | — | 40.00 | 0.2335 | 1.839 | 10.3 | 11.0 | 3/2000 |  |
| relaxed@85 | 이전 | True | — | 40.00 | 0.642 | 3.198 | 14.7 | 13.8 | 3/2000 |  |
| relaxed@85 | 이후 | True | — | 40.00 | 0.4037 | 1.986 | 15 | 12.2 | 2/2000 |  |

## 작업 A 판별 규칙 재적용(현재 모델)

| 속도 | 완주 기준(작업 A 표의 기준) | 추종 통과 기준(끝 오차·포화 < 1%) |
|---|---|---|
| 80 m/s | 둘 다 통과 — 판별의 전제(짧은 시험 실패)가 이 기준에선 사라짐 | H-시나리오 지지(기본·트림 웜스타트 둘 다 실패) |
| 85 m/s | H-시나리오 지지(기본·트림 웜스타트 둘 다 실패) | H-시나리오 지지(기본·트림 웜스타트 둘 다 실패) |
