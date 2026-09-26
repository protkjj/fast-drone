# M17 양추력 하한 형태 비교(PILOT)

**PILOT** — kj 결정(2026-09-26 저녁)의 근거다. M17만 돌렸고 우위 결론은 없다. 결정은 **(A) 측정 상태 하한 유지**다. 노드별 정확한 부등식은 V_H 돌풍 두 사례를 새로 실패시켰고, 임무 실패는 어느 형태로도 고쳐지지 않았다(ρ > 1 구간이 원인 — MISSION_RHO.md).

재현: `python -m control.arena_m17_floor_variants <변형> <사례>`, 표는 `python -m control.arena_m17_floor_variants --aggregate` · git `16b32deb2a7299db7714b2d4749396b6880f2bce` dirty=True

칸 = 끝까지 또는 정지 시각 · 실패 풀이/전체 · 반복 최대 · |ω|max(rad/s). IPOPT 반복 상한은 per_node_maxiter100만 100, 나머지는 경기장 공통값 30이다.

| 변형 | gust_lateral_p10_VH | gust_vertical_m5_VH | mission_VH |
|---|---|---|---|
| per_state | 끝까지 · 4/600 · 30 · 0.95 | 끝까지 · 0/600 · 10 · 0.16 | 36.60 s 정지 · 5/1831 · 30 · 0.23 |
| per_node | 4.52 s 정지 · 22/227 · 30 · 0.78 | 4.26 s 정지 · 7/214 · 30 · 0.11 | 36.02 s 정지 · 5/1802 · 30 · 0.21 |
| per_node_maxiter100 | 끝까지 · 19/600 · 100 · 0.79 | — | — |
| predicted_box | 끝까지 · 9/600 · 30 · 0.52 | 끝까지 · 0/600 · 10 · 0.14 | 36.36 s 정지 · 5/1819 · 30 · 0.21 |
