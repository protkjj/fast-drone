# 공통 튜닝 집합 — 저속 사례 고르기(PILOT)

**PILOT** — kj 결정 6-10: 공통 튜닝 집합에 저속 사례를 더한다. 우위·열위 결론 없음.

재현: `python -m control.arena_tuning_candidates --controllers V13 M17 F13 GSLQR` · 설정 sha256 `7af0f188c485` · git `0f56535fc0d6e5704ff1431f27417ace70dee01b` dirty=True

선택 규칙(미리 정함): 사전 게인 CPID(경기장 설정 — 기울기 45°)가 1단계를 실패 없이 끝내면 채택, 실패하면 2단계(돌풍 −2 m/s, ρ 절반), 그래도 실패하면 뺀다. 실패 = stop_reason 또는 paper_failed(튜닝 목적함수와 같은 정의). 점수 = 창 RMSE v + z(실패면 벌점 1000).

| 사다리 | 제어기 | 사례 | 길이 s | 실패 | 정지 | 논문 사유 | 창 RMSE v | 창 RMSE z | 점수 | \|ω\|max | 적분 한계 도달 % |
|---|---|---|---:|---|---|---|---:|---:|---:|---:|---:|
| 측풍 @15 | CPID | tune_gust_lateral_p5_V15 | 4.00 | False | 끝까지 | — | 0.1684 | 0.0391 | 0.2075 | 0.12 | 0.00 |
| 수직풍 @20 | CPID | tune_gust_vertical_p3_V20 | 4.00 | False | 끝까지 | — | 0.06989 | 0.06541 | 0.1353 | 0.0899 | 0.00 |
| 가속 0→15 | CPID | tune_ramp_0_15_rho0.1 | 7.00 | False | 끝까지 | — | 2.716 | 0.1824 | 2.898 | 0.332 | 0.00 |
| 측풍 @15 | V13 | tune_gust_lateral_p5_V15 | 4.00 | False | 끝까지 | — | 0.1376 | 0.01056 | 0.1482 | 0.216 | — |
| 수직풍 @20 | V13 | tune_gust_vertical_p3_V20 | 4.00 | False | 끝까지 | — | 0.08209 | 0.0189 | 0.101 | 0.0998 | — |
| 가속 0→15 | V13 | tune_ramp_0_15_rho0.1 | 7.00 | False | 끝까지 | — | 0.03478 | 0.004932 | 0.03971 | 0.429 | — |
| 측풍 @15 | M17 | tune_gust_lateral_p5_V15 | 4.00 | False | 끝까지 | — | 0.1155 | 0.009694 | 0.1252 | 0.229 | — |
| 수직풍 @20 | M17 | tune_gust_vertical_p3_V20 | 4.00 | False | 끝까지 | — | 0.0527 | 0.01499 | 0.06769 | 0.145 | — |
| 가속 0→15 | M17 | tune_ramp_0_15_rho0.1 | 7.00 | False | 끝까지 | — | 0.07221 | 0.01178 | 0.08399 | 0.426 | — |
| 측풍 @15 | F13 | tune_gust_lateral_p5_V15 | 4.00 | False | 끝까지 | — | 0.1368 | 0.01345 | 0.1503 | 0.215 | — |
| 수직풍 @20 | F13 | tune_gust_vertical_p3_V20 | 4.00 | False | 끝까지 | — | 0.07788 | 0.0252 | 0.1031 | 0.101 | — |
| 가속 0→15 | F13 | tune_ramp_0_15_rho0.1 | 7.00 | False | 끝까지 | — | 0.03487 | 0.006936 | 0.0418 | 0.428 | — |
| 측풍 @15 | GSLQR | tune_gust_lateral_p5_V15 | 4.00 | False | 끝까지 | — | 0.08802 | 0.009161 | 0.09718 | 0.32 | 0.00 |
| 수직풍 @20 | GSLQR | tune_gust_vertical_p3_V20 | 4.00 | False | 끝까지 | — | 0.03615 | 0.01923 | 0.05538 | 0.2 | 0.00 |
| 가속 0→15 | GSLQR | tune_ramp_0_15_rho0.1 | 7.00 | False | 끝까지 | — | 0.5815 | 0.1354 | 0.7169 | 0.534 | 0.00 |

## 선택

- 측풍 @15: tune_gust_lateral_p5_V15
- 수직풍 @20: tune_gust_vertical_p3_V20
- 가속 0→15: tune_ramp_0_15_rho0.1

설정 대조(고른 사례가 `configs/arena.json` tuning.scenarios에 같은 명세로 있는가): tune_gust_lateral_p5_V15 True, tune_gust_vertical_p3_V20 True, tune_ramp_0_15_rho0.1 True
