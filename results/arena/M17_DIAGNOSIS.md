# M17 솔버 실패 진단 — PILOT

**PILOT** — 진단 전용(경기장 설정·코드 불변). 우위·열위 결론 없음.

재현: `python -m control.arena_m17_diagnosis` · git `7addbab4443383447f3fb117d8aec2f00bf36227` dirty=True

가설: H1 회전수 명령 한계 활성 · H2 NLP 속 C1 곡선(PCHIP) · H3 입력 편차 기준이 호버 회전수.
변형: V0 기준(+계측) · V1 max_iter 100 · V2 M17 NLP만 C2 곡선 · V3 입력 편차 기준 = 트림 회전수.

판정 규칙(미리 정함): H1 = V0 실패 풀이의 절반 이상에서 U 한계 활성 AND V1도 멈춤 · H2 = V2가 끝까지(곡선 차이 ≤ 2%) · H3 = V3가 끝까지 · 반복 부족 = V1이 끝까지.
한계 활성 = n_max 대비 0.0001 이내(내점법 반복점은 한계에 정확히 닿지 않는다. 처음 1e-6에서 V0 측풍 한 건을 본 뒤 조정). 추력 0 계획 = 계획 명령이 실측 축방향 속도 기준 J > 양추력 한계.

C2 곡선 대 PCHIP(운용 J 0.4~1.28): Ct 최대 상대차 0.70%(J=0.439), Cp 최대 상대차 1.93%(J=0.686) · 패치 전후 플랜트 xdot 비트 동일: True

| 사례 | 변형 | 끝까지 | 시간 s / 전체 | 정지사유 | 풀이 | 실패 | 연속 실패 최대 | 반복 중앙/최대 | 실패 중 한계 활성 | 실패 중 추력0 계획 | 성공 중 추력0 계획 | 명령 범위(n_max 비) | 플랜트 추력0 % | \|ω\|max | 참조 재현 |
|---|---|---|---|---|---:|---:|---:|---|---:|---:|---:|---|---:|---:|---|
| gust_lateral_p10_VH | V0 | False | 3.76 / 12.0 | 5 consecutive optimizer failures | 189 | 5 | 5 | 5/30 | 2/5 | 5/5 | 0.5% | 0.841~0.931 | 0.0 | 0.186 | True |
| gust_lateral_p10_VH | V1 | False | 3.78 / 12.0 | 5 consecutive optimizer failures | 190 | 5 | 5 | 5/100 | 2/5 | 5/5 | 1.1% | 0.841~0.931 | 0.0 | 0.218 | — |
| gust_lateral_p10_VH | V2 | False | 3.76 / 12.0 | 5 consecutive optimizer failures | 189 | 5 | 5 | 5/30 | 1/5 | 5/5 | 0.5% | 0.841~0.930 | 0.0 | 0.189 | — |
| gust_lateral_p10_VH | V3 | False | 3.76 / 12.0 | 5 consecutive optimizer failures | 189 | 5 | 5 | 5/30 | 2/5 | 5/5 | 0.5% | 0.841~0.932 | 0.0 | 0.194 | — |
| gust_lateral_p10_VH | V4 | True | 12.00 / 12.0 |  | 600 | 4 | 1 | 5/30 | 4/4 | 0/4 | 0.0% | 0.838~0.935 | 0.0 | 0.947 | — |
| gust_vertical_m5_VH | V0 | False | 4.08 / 12.0 | 5 consecutive optimizer failures | 205 | 5 | 5 | 5/30 | 1/5 | 5/5 | 6.5% | 0.861~0.929 | 0.0 | 0.106 | True |
| gust_vertical_m5_VH | V1 | False | 4.16 / 12.0 | 5 consecutive optimizer failures | 209 | 6 | 5 | 5/100 | 0/6 | 6/6 | 7.9% | 0.861~0.925 | 0.0 | 0.106 | — |
| gust_vertical_m5_VH | V2 | False | 4.22 / 12.0 | 5 consecutive optimizer failures | 212 | 9 | 5 | 5/30 | 0/9 | 9/9 | 7.9% | 0.861~0.927 | 0.0 | 0.106 | — |
| gust_vertical_m5_VH | V3 | False | 4.16 / 12.0 | 5 consecutive optimizer failures | 209 | 8 | 5 | 5/30 | 1/8 | 8/8 | 7.0% | 0.862~0.925 | 0.0 | 0.107 | — |
| gust_vertical_m5_VH | V4 | True | 12.00 / 12.0 |  | 600 | 0 | 0 | 5/10 | 0/0 | 0/0 | 0.0% | 0.855~0.925 | 0.0 | 0.156 | — |
| ref_brake_VH_VL_rho1 | V0 | False | 24.20 / 52.1 | 5 consecutive optimizer failures | 1211 | 19 | 5 | 5/30 | 0/19 | 19/19 | 27.8% | 0.377~0.910 | 2.5 | 0.233 | True |
| ref_brake_VH_VL_rho1 | V1 | False | 23.48 / 52.1 | 5 consecutive optimizer failures | 1175 | 13 | 5 | 5/100 | 0/13 | 13/13 | 25.9% | 0.324~0.910 | 0.9 | 0.457 | — |
| ref_brake_VH_VL_rho1 | V2 | False | 23.26 / 52.1 | 5 consecutive optimizer failures | 1164 | 6 | 5 | 5/30 | 0/6 | 6/6 | 25.6% | 0.547~0.910 | 0.1 | 0.0285 | — |
| ref_brake_VH_VL_rho1 | V3 | False | 22.66 / 52.1 | 5 consecutive optimizer failures | 1134 | 7 | 5 | 5/30 | 0/7 | 7/7 | 23.4% | 0.554~0.910 | 0.0 | 0.217 | — |
| ref_brake_VH_VL_rho1 | V4 | True | 52.10 / 52.1 |  | 2605 | 0 | 0 | 5/10 | 0/0 | 0/0 | 0.0% | 0.312~0.910 | 0.0 | 0.0662 | — |

## 판정(미리 정한 규칙 적용)

| 사례 | H1 | H2 | H2 교란 | H3 | 반복 부족 | H4(사후) | 비고 |
|---|---|---|---|---|---|---|---|
| gust_lateral_p10_VH | False | False | False | False | False | True |  |
| gust_vertical_m5_VH | False | False | False | False | False | True |  |
| ref_brake_VH_VL_rho1 | False | False | False | False | False | True |  |
