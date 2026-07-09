# 고속 정찰 드론 제어 — TODO / 다음 단계

> 현재 상태: 파이썬 시뮬 **EKF 포함 전 시나리오 검증 완료**
> 최종 목표: **실기 비행** (시뮬 충분히 거친 후)
> 확정: **RTK GPS + ProperHybrid** (5개 시나리오 전부 1위)

---

## ✅ 완료된 것

### 제어 연구 (파이썬 시뮬)
- CasADi 6-DOF 플랜트(17D, 고속 공력 포함) + 검증
- 트림 (85 m/s / 300 km/h 가능, 여유 52%)
- 제어기 6종: PID / LQR / NMPC / INDI / 게인스케줄링 / **분리 하이브리드**
- 시나리오 5종: 순항 / 호버→고속 천이 / 발사천이 / 돌풍 / 모델 미스매치
- acados 실시간 (21ms, 50Hz)
- 문서화: 통합문서 / 제어기 원리·장단점 / NMPC 전공정리

### ESKF 상태추정 + 센서노이즈 검증 ★ 신규 완료
- ESKF 15D 오차상태 칼만필터 (estimator.py)
- 센서 모델: IMU(1kHz) + GPS(10Hz) (sensors.py)
- 가속도계 자세보정: 이중 게이팅 (|a|≈g + |ω|≈0)
- **전 제어기 × 전 시나리오 × 2 GPS 레벨 비교 완료**
- 버그 5건 수정 (NMPC _last_t, LQR K_r, Hybrid reset, NaiveHybrid 메서드 등)

### ESKF 핵심 결론
- **RTK(σ=0.02m) + Hybrid = 5개 시나리오 전부 1위**
  - 순항70: 0.118, 순항85: 0.190, 돌풍: 0.063, 호버: 0.049, 천이: 0.280
  - 돌풍에서 참값보다 -11% 개선 (INDI smoothing 효과)
- 표준GPS(σ=1.5m)에서는 LQR이 1위로 역전 (Hybrid/NMPC +500~1500% 저하)
- **"완벽 상태 가정이 깨지면 순위가 바뀌는가?" → 표준GPS면 Yes, RTK면 No**
- NMPC 단독이 노이즈에 가장 취약 (+1398%), 예측모델이 추정오차 증폭
- NMPC 천이 z=2.75는 버그 아닌 구조적 한계 (setpoint-tracking, 궤적추종 미지원)

### ROS2/PX4 연동 코드 (macOS에서 준비)
- 좌표 변환 (frame_utils.py) — NWU↔NED 검증 통과
- Offboard 제어 노드 (offboard_node.py) — 직접 액추에이터 제어
- 안전장치 (safety.py) — NaN/변화율/자세/고도 5중 보호
- Ubuntu 세팅 스크립트 (scripts/setup_ubuntu.sh)

---

## 📋 TODO (우선순위 순)

### [1] ROS/Gazebo 환경 세팅 ★ 다음 1순위
**목적**: 실기 경로 본격 진입. 파이썬 시뮬은 충분히 끝남.

- Phase 0: Ubuntu 24.04 + ROS2 Jazzy + PX4 + micro-XRCE-DDS + Gazebo Harmonic
- Phase 1: 기본 쿼드(x500)로 SITL 통신 확인 (**커스텀 기체 전에!**)
- ⚠️ 버전 짝 확정 먼저: ROS2 Jazzy↔Ubuntu 24.04↔Gazebo Harmonic
- ⚠️ micro-XRCE-DDS가 자주 말썽 → 안 붙으면 버전부터 의심
- 세팅 스크립트(scripts/setup_ubuntu.sh) + 실행 도우미(scripts/run_sitl.sh) 이미 작성됨

### [2] Gazebo 커스텀 통합
**⚠️ 숨은 대형 작업 두 개:**
- **커스텀 공력 플러그인**: Gazebo 기본 물리는 우리 고속 동체 공력을 안 담음.
  SDF는 형상만 정의, 공력(힘)은 C++ 플러그인으로 직접 짜야 연구 핵심이 살아남.
- **직접 모터제어 안전장치**: safety.py로 이미 구현 완료. Ubuntu 배포 시 controllers/ 파일 복사 필요.

### [3] 파라미터 현실화
- 형상팀 실제 공력값 도착 시 교체 → 결론(Hybrid 최강) 재검증

### [4] 실기체 준비
- RTK GPS 모듈 선정
- 컴패니언 컴퓨터 선정 (Jetson 등, 미정)
- PX4 + 컴패니언 시리얼/이더넷 통신
- acados RTI + ROS2 노드 실시간 구현

---

## 📌 나중에 (선택/여력 시)

- 몬테카를로 분석 (통계적 신뢰도)
- 모델 미스매치 심화 (±40%)
- NMPC를 궤적추종(trajectory-tracking)으로 개선 (천이 성능)
- 학습 기반 (RL/잔차)
- Isaac Sim (RL 대량학습 필요 시에만)

---

## 🗂 참고 — 확정 사항

- **제어기**: ProperHybrid (VirtualNMPC + INDI 분리) — 전 시나리오 1위
- **센서**: RTK GPS (σ≈0.02m) — 표준GPS에서는 고성능 제어 불가
- **형상**: 축대칭 미사일형 동체 + 쿼드 추진(로터 4개)
- **속도**: 300 km/h ≈ 85 m/s, M≈0.25 비압축성
- **이착륙**: 제자리 운용 (런처 추후)
- **상태**: 17차원 [위치3, 속도3, 쿼터니언4, 각속도3, 로터4]
- **제어입력**: [n1, n2, n3, n4] 로터 4개 속도 (핀 없음)
- **도구**: CasADi(동역학) + acados(실시간 NMPC). Simulink 불필요.
- **좌표계**: NWU(관성) + FRD(동체), PX4는 NED, 변환 = Rx(180°)
- **폐기**: 단일로터, 조종핀, 고정익/테일시터, 나이브 하이브리드, 표준GPS

---

## 🔑 세션 인계 팁

- Claude Code 세션이 무거워지면 HANDOFF + 이 TODO를 새 세션에 제공
- 큰 그림·방향 상의는 채팅 Claude, 코드 실행은 Claude Code로 분업
- NMPC/VirtualNMPC는 reset() 시 `_last_t = -inf` 초기화 필수 (버그 이력)
- 가속도계 자세보정: 동적가속도 분리는 순환논리로 불가, 이중게이팅이 정답
