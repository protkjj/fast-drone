# GSLQR 적분 한계 — 스모크 사례 대조(PILOT)

**PILOT** — kj 결정 6-9: 한계 ∞로 영향부터 대조하고, 영향이 있으면 한계를 넓힌다. 결론 없음.

재현: `python -m control.arena_integrators --smoke-cases mission_VH_mass1.3 mission_VH_tau2 --limits 1e+09 50` · 설정 sha256 `60fba9acd26d` · git `2bf67e02abeb1dae12c78fbe9d3b2bc6a0b6d30e` dirty=True

영향 있음 = 정지 여부·사유 변화, 정지 시각 0.1 s 초과 변화, 창 RMSE(v·z) 5% 초과 변화, 논문 판정 변화 중 하나(미리 정한 규칙).

| 사례 | 적분 한계 | 정지 | 시간 s | 창 RMSE v | 창 RMSE z | 공통 구간 RMSE v / z | 처음 갈라진 시각 s | \|ω\|max | 논문 실패 | 한계 도달 % | ξ 최대(z, vx) | 참조 재현 |
|---|---:|---|---:|---:|---:|---|---:|---:|---|---:|---|---|
| mission_VH_mass1.3 | 5 | body-rate or altitude divergence stop | 44.33 | 7.957 | 5.173 | 7.957 / 5.173 (~44.33 s) | — | 26 | True | 10.97 | z 2.47, vx 5 | True |
| mission_VH_tau2 | 5 | body-rate or altitude divergence stop | 45.78 | 7.513 | 6.126 | 7.513 / 6.126 (~45.78 s) | — | 33.1 | True | 4.89 | z 2.75, vx 5 | True |
| mission_VH_mass1.3 | 1e+09 | body-rate or altitude divergence stop | 44.59 | 8.038 | 6.116 | 7.114 / 4.893 (~44.33 s) | 37.938 | 26 | True | 0.00 | z 2.47, vx 5.91 | — |
| mission_VH_tau2 | 1e+09 | body-rate or altitude divergence stop | 45.77 | 7.556 | 6.087 | 7.556 / 6.087 (~45.77 s) | 43.542 | 34.6 | True | 0.00 | z 2.74, vx 5.79 | — |
| mission_VH_mass1.3 | 50 | body-rate or altitude divergence stop | 44.59 | 8.038 | 6.116 | 7.114 / 4.893 (~44.33 s) | 37.938 | 26 | True | 0.00 | z 2.47, vx 5.91 | — |
| mission_VH_tau2 | 50 | body-rate or altitude divergence stop | 45.77 | 7.556 | 6.087 | 7.556 / 6.087 (~45.77 s) | 43.542 | 34.6 | True | 0.00 | z 2.74, vx 5.79 | — |

공통 구간 = 두 한계 실행이 모두 살아 있던 [3 s, 먼저 멈춘 시각]. 창 RMSE는 정지 시각이 다르면 구간 길이가 달라 직접 비교할 수 없어 함께 적는다.

## 영향 판정(미리 정한 규칙)

- mission_VH_mass1.3@1e+09: 영향 있음 — stop time 44.33 -> 44.59 s; window_rmse_z 5.173 -> 6.116
- mission_VH_tau2@1e+09: 영향 없음
- mission_VH_mass1.3@50: 영향 있음 — stop time 44.33 -> 44.59 s; window_rmse_z 5.173 -> 6.116
- mission_VH_tau2@50: 영향 없음
