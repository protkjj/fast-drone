# 발표 그림 색인

시뮬 조건은 전부 **참값 제어(센서모델 없음) + IPOPT NMPC**다.
`A` = 이 세션에서 재시뮬, `R` = 기록된 실측값을 그린 것.

## 3분 발표용 (연구제안)

| 슬라이드 | 그림 | 출처 | 핵심 숫자 |
|---|---|---|---|
| 1 문제 | `prop_fig_final.png` | A | **최종 그래프** — 감속 제외, 기수오차 20° 판정 |
| 1 보조 | `prop_fig_gust.png` | A | 수직돌풍 15 m/s: peak Δv_x **0.731 vs 0.068 (10.8×)** |
| 1 보조 | `prop_fig_crosswind.png` | A | 지속 측풍: 요각 **11.64° vs 8.30°**, 횡편차는 역전 |
| 2 왜 지금 | `prop_fig_solver.png` | R | IPOPT **21.4/28.1 ms** vs acados 0.09/**0.38 ms** (예산 20 ms) |
| 3 접근 | (없음) | — | 인터페이스 분리 개념도 — 직접 그려야 함 |
| 4 예비결과 | `prop_fig_robustness.png` | A | 공력오차 +30%: **2.851 vs 0.148 (19.3×)** |
| 4 보조 | `prop_fig_mc.png` | R | MC 10회: 분리 최악 2.82 < LQR 최선 4.42 |

## 역할 A (비행역학·공력) 자료

| 그림 | 출처 | 내용 |
|---|---|---|
| `plant_validation.png` | A | 플랜트 검증 6종 (2×3). 4번이 겹2 고유값과 0.3% 교차검증 |
| `trim_sweep.png` | R | 트림 속도 스윕, 최고 85 m/s (306 km/h) |
| `flight_envelope_modes.png` | A | ω_n ∝ V, ζ = 0.05 속도 불변 |
| `flight_envelope_authority.png` | A | 로터 사용률, 가용 각가속도 (피치/요) |
| `flight_envelope_gust.png` | A | 겹4 여유비 — ⚠ 해석 정정됨, `flight_envelope.txt` 참고 |
| `prop_fig_mission_crosswind.png` | A | 65초 전 구간 + 측풍. **맨 Hybrid 감속 텀블(\|ω\| 90)** |

## 참고 (기존 자료)

| 그림 | 내용 |
|---|---|
| `sweep_knee.png` | **고정 게인**이 속도에서 저하 (0.86 → 2.87). 스케줄링이 필요한 이유 |
| `sweep_fixed_vs_scheduled.png` | 스케줄링하면 회복(0.77). 단 테이블 데이터가 필요 |
| `mission_plot.png` | 기존 65초 미션 (수직돌풍만) |

---

## ⚠ 슬라이드에 올릴 때 반드시

**RMSE만 있는 표를 쓰지 말 것.** 65초 미션에서 맨 ProperHybrid 는 RMSE 가 셋 중
제일 좋은데(vx 2.08) **감속에서 텀블한다**(|ω| 90.3, 요 522°). `|ω|max` 를 같은
표에 넣어야 한다. 실기 실패 판정은 `|ω| > 35 rad/s` 또는 `|ω| > 25` 가 200 ms 이상.

**숫자 하나는 쓰지 말 것.** `MISSION_ANALYSIS.md` 의 "돌풍 응답 LQR 최대 Δv_x
15.226" 은 돌풍 응답이 **아니다** — 돌풍 전에 이미 16.35 벗어나 있던 가속 추종
실패다. 정정 주석을 달아 뒀다.

## 재현

```
python3 -m control.plot_proposal_figures      # gust · solver · mc · crosswind
python3 -m control.plot_plant_validation      # 플랜트 검증 6종
python3 -m control.flight_envelope            # 타당성 4겹
python3 -m control.robustness_crosswind       # 강건성 · 미션측풍 · 최종
```
