# 고속 정찰 드론 제어 — TODO / 다음 단계

> 현재 상태: 파이썬 시뮬 검증 완료 + **Ubuntu에 PX4 SITL + Gazebo 스택 기동 성공 (2026-07-10)**
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

### Ubuntu 실환경 구축 (Phase 0 + Phase 1) ★ 신규 완료 (2026-07-10)
- **검증 환경**: Ubuntu 24.04.4 / ROS2 Jazzy / Gazebo Harmonic(gz-sim 8.11) / micro-XRCE-DDS
- PX4-Autopilot 클론 + 의존성 설치 + `make px4_sitl gz_x500` 빌드 성공
- Python 의존성 정리: requirements.txt(시뮬용) + PX4 Tools/setup/requirements.txt 설치
- **SITL + Gazebo 기동 확인**: x500 드론 로드, Gazebo GUI 표시
- **micro-XRCE-DDS Agent(8888) ↔ PX4 연결 확인**: 토픽/publisher 생성 성공
- 팀원 온보딩 문서 작성: **SETUP.md** (Ubuntu→Python→PX4 SITL 전 과정)
- 결정: **Docker 미사용** (사용자 3~4명, 네이티브 스택이 이미 검증됨)

### 커스텀 기체 fast_missile SITL 호버 성공 ★ 신규 완료 (2026-07-11)
- 커스텀 기체 `fast_missile{,_base}` (질량 8kg, 관성 0.02/0.70/0.70, 4100 에어프레임)
- **arm 직후 전복 문제 해결 → 30초+ 안정 호버 (PID, z=5m)**
- 전복 원인 = **모터맵/스핀/부호가 아님** (전부 실측 정상). 진짜 원인 2가지:
  1. **heading 미초기화** (기체 yaw≈−96° 스폰인데 CascadedPID가 절대 0° 목표) → heading 래치
  2. **자세게인 과대** (EKF omega 노이즈 증폭 → 진동 발산) → `att_gain_scale=0.35`
- 진단 방법: 단일모터/순수축 오픈루프 test(플랜트 검증) + 오프라인 파이썬 노이즈 재현
- 상세: HANDOFF.md "✅ 해결 (2026-07-11)" + 메모리 sitl-hover-flip-rootcause

### [1b] 제어기 노드 연결·강건화 + Hybrid 연결 ★ 신규 (2026-07-12)
- **노드 기능 추가** (offboard_node.py):
  - `att_gain_scale`(기본 0.35)를 **컨트롤러별 일반화** — pid(Kp/Kd), **lqr(K_r δφ·δω 열)**, indi(Kp/Kd_indi)
  - **LQR heading 재선형화** (`_relatch_lqr_heading`): 스폰 기수로 트림 재선형화 → δφ 오차 1.49→0 (프레임 정합)
  - **`hybrid` case 추가** — ProperHybrid(VirtualNMPC+INDI) 노드 연결. SITL 실시간용 `dt_ctrl=0.1(10Hz)/max_iter=5`
  - ProperHybrid INDI **ω̇를 실측 Δt로 계산**(하드코딩 dt 제거) → 가변 루프율 강건
- **SITL 결과(호버 z=5)**: PID 안정 ✓ / LQR·INDI·Hybrid **전부 롤(omega_x) 발산 → 전복**
  - heading 래치·게인축소·타이밍픽스로 개선(더 오래 버팀)됐으나 근본 미해결
  - ⚠️ **오프라인 모델(dynamics.py)로는 이 발산이 재현 안 됨**(너무 이상적: 무노이즈·무지연·무접촉).
    → **컨트롤러 튜닝은 Gazebo-in-the-loop로만 가능** (오프라인은 로드/버그 사전검증용)

### [1b-후속] 🔴 롤 불안정 + 장축 X/Z 결정 (LQR/Hybrid SITL의 블로커)
- **증상**: PID 외 3개 컨트롤러 전부 **롤축(Ixx=0.02, 피치/요의 1/35)**에서 폭발
- **후보 원인**: Ixx=0.02가 롤을 초민감하게 만듦. 단 **관성만 스왑 불가** —
  현재 공력(S_ref=코단면, C_A0 유선형)이 **장축=X(수평 미사일)** 가정이라 300km/h 성립.
  장축=Z(수직)로 보면 공력 전체 재정의 필요 + 300km/h 불가.
  → **장축 X vs Z는 공력+관성+외형을 함께 정하는 미해결 설계질문** (아래 [2]와 직결)
- 다음 세션: 이 결정을 먼저 확정한 뒤 그에 맞춰 관성/공력 정합 → 재시험

---

## 📋 TODO (우선순위 순)

### [1] ROS/Gazebo 환경 세팅 — ✅ 완료 (2026-07-10)
~~Phase 0/1 완료~~. 위 "Ubuntu 실환경 구축" 참고. 남은 세부 작업:

- ✅ ROS2 토픽 흐름 확인 (/fmu/out/* 수신, QoS=best_effort, v1.18은 토픽명 `_v1` 접미사)
- ✅ **ROS2 워크스페이스 빌드** — px4_msgs(main) + px4_ros_com + fast_drone_ctrl colcon 성공
- ✅ x500 이착륙 full cycle 검증 — 이륙→2.5m 호버→착륙→자동 disarm
  - 헤드리스(QGC 없이 ROS2)에선 arm 차단됨 → `NAV_RCL_ACT=0` 등 파라미터로 해결
  - 팀원은 콘솔+QGC 정상 방식이면 파라미터 불필요 → 문서화 생략 결정

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

## 🐛 코드 리뷰 결과 (2026-07-12, 4중 병렬 리뷰)

**총평**: arm-즉시-크래시급 치명 버그 없음 (제어기 수학·ESKF·좌표변환·모터맵·호버추력 전부 정합).

### ✅ 이번에 수정함
1. **INDI 전진비 누락** (controller.py `_compute_G`/`T_meas`) — 버그픽스 3-1이 ProperHybrid엔 있었으나
   노드가 쓰는 standalone INDIController엔 빠져 있었음. `dT/dn=k_T·n·(1+fac)`로 통일 (하위호환 v_body=None→fac=1)
2. **센서 RNG 미재시드** (sensors.py) — GPS/IMU `reset()`에서 `self.rng`를 저장seed로 재시드. 이제 제어기 간
   동일 노이즈 실현 = 공정 비교 (기존엔 "표준GPS LQR 역전" 결론이 노이즈 뽑기 운일 수 있었음).
   ※ 비교 루프는 **첫 제어기 전에도 reset() 호출** 필요(첫 vs 나머지 스트림 위치 일치).
3. **NMPC 무효 쿼터니언 초기추측** (nmpc.py) — `[0,0,0,0]`→`[1,0,0,0]` (첫 solve 반복 낭비 방지)
4. **mission_sim 발산판정** — NaN만 보던 것을 `|z-z_ref|>50m`도 발산 처리 (튄 궤적을 성공으로 집계 방지)
5. **launch 파라미터 타입** — `ParameterValue(..., value_type=float)` 래핑 (ros2 launch 경로 타입불일치 예방)

### ⏳ 이월 (설계결정/위험/저영향)
- **PX4 에어프레임 CA_ROTOR/KM가 x500 값** (4100_gz_fast_missile) — offboard direct-actuator엔 무해,
  단 offboard-loss failsafe 등 PX4 네이티브 복구가 잘못된 믹서로 비행 → 실기 전 수정 필요
- **관성텐서 vs 외형 vs 공력 축 불일치** (Ixx 장축=X vs 외형 세로미사일=Z) → 위 [1b-후속]/[2]에서 결정
- rotor_directions가 gz SDF turningDirection과 전부 반대부호(호버 성공=상쇄된 듯) — 건드리지 말 것(검증 후)
- att_gain_scale는 scheduled_pid엔 무효(노드가 경고) — 그 모드 쓰면 전복 재현 위험
- ESKF predict/update 1스텝(1ms) 지연 / fallback vel_mag 데드코드 — 무시가능
- safety 레이트리미터 pre-arm desync — 이륙 무해(재현 확인)

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
