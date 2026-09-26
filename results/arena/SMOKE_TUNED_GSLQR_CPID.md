# 튜닝값 스모크 결과 — 2026-09-26 (SMOKE)

**SMOKE** — 경기장 파이프라인이 모든 사례를 끝까지 도는지, 어디서 멈추는지 보는 실행이다. 성능 비교가 아니며 우위·열위 결론을 내리지 않는다.

재현: `python -m control.validation_suite --config configs/arena.json --smoke --tuned results/arena/tuning/main120 --only-controllers GSLQR CPID --reference-out results/arena/smoke_tuned_GSLQR_CPID.json` · 요약: `python -m control.arena_smoke_summary`

게인: **튜닝값** — `results/arena/tuning/main120` 기록의 최선값(같은 예산 튜닝, PILOT). 기록 sha256: GSLQR `779a4a52c09e`, CPID `a370b5a1cdfb`

설정 sha256 `19cd72375ef7` · git `ffa417ac` dirty=False · Darwin arm64 · Apple M4 · Python 3.13.7

시행 11개 + 설계 영역 밖 제외 7개(kj 결정 2026-09-26: CPID는 설계 영역 0~20 m/s 안의 V_L 사례만).

열: 스위트 판정 = 추종·안전(Acceptance) AND 추진 모델 도메인 · 추종 = Acceptance만 · 도메인 = 추진 모델 가정 범위(매우 엄격) · 논문 실패 = §5.10 · 창 RMSE = §5.8 평가창 · 솔버실패 = 실패/호출 · 적분 한계 % = GSLQR·CPID 적분기가 한계에 닿은 스텝 비율

| 사례 | 제어기 | 시뮬 s | 스위트 판정 | 추종 | 도메인 | 논문 실패 | 창 RMSE v | 창 RMSE z | \|ω\|max | 솔버실패 | 적분 한계 % | 정지·사유 |
|---|---|---:|---|---|---|---|---:|---:|---:|---:|---:|---|
| mission_VH | GSLQR | 46.17 | False | False | False | True | 4.67 | 13.8 | 35.5 | — | 0.0 | body-rate or altitude divergence stop |
| mission_VH | CPID | — | 제외 | — | — | — | — | — | — | — | — | outside design region 0-20 m/s (scenario reaches 85 m/s) |
| gust_lateral_p10_VH | GSLQR | 12.00 | True | True | True | False | 0.153 | 0.00428 | 0.194 | — | 0.0 |  |
| gust_lateral_p10_VH | CPID | — | 제외 | — | — | — | — | — | — | — | — | outside design region 0-20 m/s (scenario reaches 85 m/s) |
| gust_vertical_m5_VH | GSLQR | 12.00 | True | True | True | False | 0.178 | 0.0842 | 0.142 | — | 0.0 |  |
| gust_vertical_m5_VH | CPID | — | 제외 | — | — | — | — | — | — | — | — | outside design region 0-20 m/s (scenario reaches 85 m/s) |
| ref_accel_0_VH_rho1 | GSLQR | 7.50 | False | False | False | True | 51.2 | 13.9 | 17.6 | — | 0.0 | body-rate or altitude divergence stop |
| ref_accel_0_VH_rho1 | CPID | — | 제외 | — | — | — | — | — | — | — | — | outside design region 0-20 m/s (scenario reaches 85 m/s) |
| ref_brake_VH_VL_rho1 | GSLQR | 52.10 | True | True | True | False | 0.0112 | 0.00858 | 0.0882 | — | 0.0 |  |
| ref_brake_VH_VL_rho1 | CPID | — | 제외 | — | — | — | — | — | — | — | — | outside design region 0-20 m/s (scenario reaches 85 m/s) |
| mission_VH_mass1.3 | GSLQR | 45.51 | False | False | False | True | 4.57 | 12.2 | 35.8 | — | 0.0 | body-rate or altitude divergence stop |
| mission_VH_mass1.3 | CPID | — | 제외 | — | — | — | — | — | — | — | — | outside design region 0-20 m/s (scenario reaches 85 m/s) |
| mission_VH_tau2 | GSLQR | 46.16 | False | False | False | True | 4.67 | 13.8 | 35.4 | — | 0.0 | body-rate or altitude divergence stop |
| mission_VH_tau2 | CPID | — | 제외 | — | — | — | — | — | — | — | — | outside design region 0-20 m/s (scenario reaches 85 m/s) |
| gust_lateral_p10_VL | GSLQR | 12.00 | True | True | True | False | 0.122 | 0.0246 | 0.436 | — | 0.0 |  |
| gust_lateral_p10_VL | CPID | 12.00 | False | False | False | True | 20.1 | 5.08 | 11.6 | — | 0.0 |  |
| gust_vertical_m5_VL | GSLQR | 12.00 | True | True | True | False | 0.0483 | 0.0295 | 0.116 | — | 0.0 |  |
| gust_vertical_m5_VL | CPID | 12.00 | True | True | True | False | 0.0234 | 0.0118 | 0.133 | — | 0.0 |  |

## 사전값 참조 대비

사전값 참조: 설정 sha256 `19cd72375ef7` · git `4b20634f`. 해시가 다르면 궤적이 바뀐 것이다.

| 사례 | 제어기 | 해시 동일 | 시뮬 s 전 → 후 | 창 RMSE v 전 → 후 | 창 RMSE z 전 → 후 | \|ω\|max 전 → 후 | 논문 실패 전 → 후 | 정지 전 → 후 |
|---|---|---|---|---|---|---|---|---|
| mission_VH | GSLQR | False | 46.11 → 46.17 | 4.93 → 4.67 | 6.59 → 13.8 | 35.1 → 35.5 | True → True | body-rate or altitude divergence stop → body-rate or altitude divergence stop |
| mission_VH | CPID | 제외 | | | | | | |
| gust_lateral_p10_VH | GSLQR | False | 12.00 → 12.00 | 0.507 → 0.153 | 0.0306 → 0.00428 | 0.63 → 0.194 | False → False | 끝까지 → 끝까지 |
| gust_lateral_p10_VH | CPID | 제외 | | | | | | |
| gust_vertical_m5_VH | GSLQR | False | 12.00 → 12.00 | 0.205 → 0.178 | 0.0969 → 0.0842 | 0.148 → 0.142 | False → False | 끝까지 → 끝까지 |
| gust_vertical_m5_VH | CPID | 제외 | | | | | | |
| ref_accel_0_VH_rho1 | GSLQR | False | 6.64 → 7.50 | 73.1 → 51.2 | 10.1 → 13.9 | 35.8 → 17.6 | True → True | body-rate or altitude divergence stop → body-rate or altitude divergence stop |
| ref_accel_0_VH_rho1 | CPID | 제외 | | | | | | |
| ref_brake_VH_VL_rho1 | GSLQR | False | 52.10 → 52.10 | 0.0164 → 0.0112 | 0.00828 → 0.00858 | 0.0885 → 0.0882 | False → False | 끝까지 → 끝까지 |
| ref_brake_VH_VL_rho1 | CPID | 제외 | | | | | | |
| mission_VH_mass1.3 | GSLQR | False | 44.55 → 45.51 | 7.34 → 4.57 | 5.69 → 12.2 | 24.5 → 35.8 | True → True | body-rate or altitude divergence stop → body-rate or altitude divergence stop |
| mission_VH_mass1.3 | CPID | 제외 | | | | | | |
| mission_VH_tau2 | GSLQR | False | 46.15 → 46.16 | 4.93 → 4.67 | 6.6 → 13.8 | 35.4 → 35.4 | True → True | body-rate or altitude divergence stop → body-rate or altitude divergence stop |
| mission_VH_tau2 | CPID | 제외 | | | | | | |
| gust_lateral_p10_VL | GSLQR | False | 12.00 → 12.00 | 0.219 → 0.122 | 0.0152 → 0.0246 | 0.43 → 0.436 | False → False | 끝까지 → 끝까지 |
| gust_lateral_p10_VL | CPID | False | 12.00 → 12.00 | 0.397 → 20.1 | 0.0489 → 5.08 | 0.313 → 11.6 | False → True | 끝까지 → 끝까지 |
| gust_vertical_m5_VL | GSLQR | False | 12.00 → 12.00 | 0.0323 → 0.0483 | 0.0189 → 0.0295 | 0.139 → 0.116 | False → False | 끝까지 → 끝까지 |
| gust_vertical_m5_VL | CPID | False | 12.00 → 12.00 | 0.0861 → 0.0234 | 0.0681 → 0.0118 | 0.0887 → 0.133 | False → False | 끝까지 → 끝까지 |

이전 참조에만 있는 행: gust_lateral_p10_VH/F13, gust_lateral_p10_VH/M17, gust_lateral_p10_VH/V13, gust_lateral_p10_VL/F13, gust_lateral_p10_VL/M17, gust_lateral_p10_VL/V13, gust_vertical_m5_VH/F13, gust_vertical_m5_VH/M17, gust_vertical_m5_VH/V13, gust_vertical_m5_VL/F13, gust_vertical_m5_VL/M17, gust_vertical_m5_VL/V13, mission_VH/F13, mission_VH/M17, mission_VH/V13, mission_VH_mass1.3/F13, mission_VH_mass1.3/M17, mission_VH_mass1.3/V13, mission_VH_tau2/F13, mission_VH_tau2/M17, mission_VH_tau2/V13, ref_accel_0_VH_rho1/F13, ref_accel_0_VH_rho1/M17, ref_accel_0_VH_rho1/V13, ref_brake_VH_VL_rho1/F13, ref_brake_VH_VL_rho1/M17, ref_brake_VH_VL_rho1/V13
