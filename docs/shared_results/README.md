# 공유용 실험 결과

2026-09-25에 실행한 명목 모델 시험의 요약과 그림이다. 원시 궤적을 새로 계산한 결과가 아니라 저장된 실행에서 추출했다.

- `mission.png`: 동일한 40초 참조에 대한 GS-LQR, NMPC, NMPC+INDI 응답. 중단된 NMPC는 해당 시점에서 곡선이 끝난다.
- `mission_summary.json`: 세 제어기의 추종·솔버 실패·모델 적용 범위 및 정착 지표. 완주하지 않은 NMPC의 부분 구간 RMSE를 완주 결과와 직접 비교하면 안 된다.
- `gust_summary.json`: 4.5초 독립 돌풍 9회 결과.

## 원 실행

- Split: `timing_mission_20260925T001850705811Z`
- GS-LQR / NMPC: `timing_mission_20260925T001850771414Z`
- 그림 집계: `timing_selected_20260925T002756678357Z`
- 돌풍: `timing_gust_20260925T001241265619Z`

기본 미션의 모델 적용 범위 이탈은 그대로 보존했다. 물리적 검증 합격이나 강건성 입증으로 해석하지 않는다. 재현 방법은 [시간 선정 문서](../TIMING_OPTIMIZATION.md)를 참고한다.
