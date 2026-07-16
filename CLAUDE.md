# fast_drone 프로젝트 규칙

고속(목표 300km/h) 쿼드 추진 미사일형 드론의 제어기 연구 프로젝트.
macOS에서 코드 작성 → Ubuntu(ROS2 Jazzy + PX4 + Gazebo Harmonic)에서 실행하는 이중 환경.

## 세션 시작/종료

- 세션 시작 시 HANDOFF.md(있으면 TODO.md, MEMORY.md도) 먼저 정독할 것
- 작업 단위가 끝날 때마다 HANDOFF.md 갱신. 실험 결과는 MEMORY.md에 기록
- 버그 수정 후에는 영향받는 기존 실험을 재실행하고 MEMORY.md의 관련 수치를 갱신할 것

## 좌표계/부호 규약 — 이 프로젝트 최대 함정

- 관성 좌표계 z-up, **동체 좌표계 z-down (미사일 공력 관례)**, 호버 쿼터니언 `[1,0,0,0]`
- PX4 연동 시 NWU↔NED 변환은 `frame_utils.py`의 규칙을 따를 것
- **부호/축/회전방향(CW·CCW) 관련 결론은 원본 소스(dynamics.py)와 대조하기 전에 확정하지 말 것.**
  z-up/z-down 시점 차이로 스핀 방향 판단이 뒤집힌 전력, 독스트링과 실제 구현이 달라
  테스트 부호가 뒤집힌 전력이 있음

## 검증 규칙

- **이 프로젝트에서 극단적인 숫자는 거의 항상 버그였음** (+2596% 저하율 → `_last_t` 미초기화,
  Q_L 부호 오류, 순환논리 버그 등). 이상치가 나오면 보고 전에 재현·반대방향 테스트부터 할 것
- `dynamics.py` 변경 시 `test_plant.py`로 역호환 검증 필수
- 게인/파라미터 스윕은 한 조건에서 튜닝한 값을 고정한 채 진행 (속도마다 재튜닝하면 논점이 약해짐)
- 파라미터를 목표치(300km/h 등)에 맞춰 끼워맞추지 말고 물리적 타당성을 먼저 확인할 것

## 확정된 설계 사항

- 상태 17차원 `[p(3), v(3), q(4), ω(3), n(4)]`, 입력은 로터 속도 명령 4차원
- 추진 배치는 × 배치 (+ 배치는 할당행렬 특이로 불가)
- 제어기는 ProperHybrid(NMPC + INDI, 인터페이스 분리) 확정, 센서는 RTK GPS 확정
- CasADi 사용 이유: 자동미분 → acados/IPOPT 재사용

## 파일/실행 컨벤션

- 소스(`.py`)와 산출물(txt/md/png)은 분리: 결과물은 `results/`, 안 쓰는 코드는 `legacy/`
  (import 수정 없이 이동), Ubuntu 세팅 스크립트는 `scripts/`
- 물리 파라미터는 `vehicle_params.py`에만 (형상팀 값이 오면 이 파일만 교체)
- 긴 시뮬레이션(몬테카를로 등)은 `caffeinate -dims python3 xxx.py > results/xxx.txt 2>&1 &` 패턴으로
- 플롯 라벨은 영어로 (matplotlib 기본 폰트가 한글 미지원)

## 환경 주의사항

- PX4-Autopilot 관련 파일(커스텀 SDF 등)은 Ubuntu 로컬에만 있고 git에 없음 —
  macOS 세션에서 직접 수정 불가. 복붙 가능한 셸 명령/스크립트로 제공할 것
- Ubuntu 쪽 Claude Code 출력을 kj가 복사해 붙여넣는 경우가 있음 — 다른 에이전트의 진단이므로
  코드 대조로 검증할 것
- 이 폴더는 iCloud 동기화 대상 — `model 2.sdf`처럼 " 2" 붙은 파일은 동기화 충돌 사본이니
  발견 시 원본과 diff 후 정리 제안
- acados는 macOS 설치 이력 있음 (tera renderer, qpOASES rpath 트러블슈팅 기록 참고)
