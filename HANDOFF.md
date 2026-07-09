# HANDOFF — 고속 ISR 드론 제어 시뮬레이션

## 프로젝트 목표
축대칭 미사일형 동체 + 쿼드콥터 추진 드론의 고속(300 km/h) 비행 제어.
6개 제어기를 5개 시나리오에서 비교 → ESKF 상태추정 하 재검증까지 완료.

## 확정 사항
- **제어기: ProperHybrid** (VirtualNMPC + INDI 분리) — 5개 시나리오 전부 1위
- **센서: RTK GPS** (σ≈0.02m) — 표준GPS(1.5m)에서는 고성능 제어 불가
- **최종 목표: 실기체 비행** (PX4 + Gazebo → 실기)

## 핵심 결론

### 발견 1 — 완벽 상태 가정: "상황별 최적이 다르다"

```
시나리오         | PID고정  PID스케줄  LQR고정  LQR스케줄  NMPC     INDI단독
─────────────────┼──────────────────────────────────────────────────────────
순항 85 m/s      |
  RMSE z         | 1.85    1.55      0.35     0.34      0.21*    -
순항 60 m/s      |
  RMSE z         | 0.91    0.78      0.39     0.37      0.27*    -
─────────────────┼──────────────────────────────────────────────────────────
호버→70 천이     |
  RMSE vx        | 7.77    8.36      16.77    17.32     4.80*    -
  RMSE z         | 3.34    3.10      1.04     0.38*     2.23     -
─────────────────┼──────────────────────────────────────────────────────────
돌풍 (측풍 10m/s)|
  최대 Δz        | 1.28    1.18      0.05*    0.05*     0.10     1.35
돌풍 (수직 10m/s)|
  최대 Δz        | 1.16    1.12      0.29     0.26*     0.32     1.21
─────────────────┼──────────────────────────────────────────────────────────
모델미스매치     |        (C_Na ±20%, 순항+돌풍)
  NMPC           |                                       0.080~0.112
  NMPC+INDI(naive)|                                      0.263~0.472 (악화!)
  분리 하이브리드 |                                       0.038~0.079*

  * = 해당 시나리오 최고 성능
```

### 발견 2 — 인터페이스 분리 하이브리드가 전 조건 최강
- 나이브(둘 다 모터속도): 이중 보정 → NMPC보다 3.5배 악화
- **분리(NMPC→[T,ω̇], INDI→모터): 전 조건 최강**
- 핵심: "뭘 할지(NMPC)"와 "어떻게 할지(INDI)"의 역할 분리

### 발견 3 — ESKF: "센서 노이즈가 들어오면 순위가 바뀌는가?"

**RTK GPS (σ=0.02m) — 순위 유지, Hybrid 5전 5승:**

```
시나리오     | LQR스케줄       NMPC            Hybrid
─────────────┼─────────────────────────────────────────
순항 70 m/s  | 0.374 (+5%)    0.281 (+11%)    0.118* (+0%)
순항 85 m/s  | 0.359 (+7%)    0.238 (+11%)    0.190* (+13%)
돌풍         | 0.081 (+27%)   0.118 (+29%)    0.063* (-11%)
호버         | 0.305 (-4%)    0.181 (+7%)     0.049* (+47%)
호버→70 천이 | 0.454 (+21%)   2.824 (+3%)     0.280* (+26%)

(괄호) = 참값 대비 저하율.  * = RTK 1위
```

**표준 GPS (σ=1.5m) — 순위 역전, LQR이 1위:**

```
시나리오     | LQR스케줄       NMPC            Hybrid
─────────────┼─────────────────────────────────────────
순항 70+돌풍 | 0.396* (+522%) 1.374 (+1398%)  0.418 (+489%)
```

**핵심**: RTK면 Hybrid 최강 유지. 표준GPS면 LQR로 역전 (NMPC/Hybrid +500~1500% 저하).

### 발견 4 — NMPC 천이 z=2.8은 버그 아닌 구조적 한계
setpoint-tracking NMPC가 시간변화 기준을 추종할 때 예측 지평선(1초) 동안 목표가 ~4.7 m/s 변해 항상 뒤처짐. trajectory-tracking으로 개선 가능.

### 발견 5 — 가속도계 자세보정: 이중 게이팅이 정답
동적가속도 분리(Complementary Filter 방식) 시도 → IMU가 유일한 동적가속도 소스라 순환 논리 발생. 대신 |a|≈g + |ω|≈0 이중 게이팅으로 기동 시 자동 약화.

## 완료된 것 (전체)

### 플랜트 모델 (`dynamics.py`)
- CasADi 6-DOF: 미사일 동체 공력 + 4로터, 바람 입력
- 상태 17D, V=0 특이점 소거, 전진비 추력
- 검증: `test_plant.py` 6개 통과

### 제어기 (`controller.py`)
- CascadedPID, LQRController(Error-State 14D), ScheduledPID/LQR, INDI

### NMPC
- CasADi+IPOPT (`nmpc.py`), acados+HPIPM (`nmpc_acados.py`)

### 하이브리드 (`hybrid_comparison.py`)
- VirtualNMPC(13D) + ProperHybrid(INDI 실행자)
- NaiveHybrid(비교용, 이중보정 실패)
- `compute_control_effectiveness()` 공통 함수

### ESKF 상태추정 (신규)
- **`sensors.py`**: IMU(가속도+자이로, 1kHz) + GPS(위치+속도, 10Hz)
  - 바이어스 모델, 노이즈 레벨 파라미터
- **`estimator.py`**: 15D 오차상태 칼만필터
  - 오차상태: [δp(3), δv(3), δθ(3), δb_a(3), δb_g(3)]
  - predict(IMU), update_gps(위치+속도), update_accel(자세 보정, 이중 게이팅)
- **`ekf_sim.py`**: 센서+ESKF 통합 시뮬 루프
- **`ekf_comparison.py`**: 5제어기 × 2 GPS 레벨 비교
- **`ekf_full_comparison.py`**: 3제어기 × 5시나리오 × RTK 종합 비교

### ROS2/PX4 연동 (macOS에서 코드 준비)
- `ros2_ws/src/fast_drone_ctrl/`: ROS2 패키지
  - `frame_utils.py`: NWU↔NED 좌표 변환 (검증 통과)
  - `offboard_node.py`: PX4 Offboard 직접 액추에이터 제어
  - `safety.py`: NaN/변화율/자세/고도 5중 보호
- `scripts/setup_ubuntu.sh`: Ubuntu 환경 세팅 (PX4+Gazebo+DDS)
- `scripts/run_sitl.sh`: SITL 실행 도우미

### 비교 스크립트
- `sweep.py`, `sweep_scheduled.py`: 순항·천이·발사 종합
- `gust_comparison.py`: 돌풍 × 6제어기

### 수정된 버그
1. nmpc.py: 가짜 LQR에 K_r 미설정 → AttributeError
2. ekf_comparison.py: NMPC/VirtualNMPC _last_t 리셋 누락
3. hybrid_comparison.py: ProperHybrid.reset()에 VirtualNMPC 리셋 추가
4. hybrid_comparison.py: NaiveHybrid의 _compute_G 공통 함수로 분리

## 향후 과제

### 우선순위 1: ROS/Gazebo 환경 세팅
- Phase 0: Ubuntu 24.04 + ROS2 Jazzy + PX4 + micro-XRCE-DDS + Gazebo Harmonic
- Phase 1: 기본 쿼드(x500)로 SITL 통신 확인
- 세팅 스크립트 이미 작성 (scripts/setup_ubuntu.sh)

### 우선순위 2: Gazebo 커스텀 통합
- 커스텀 공력 플러그인 (C++) — Gazebo 기본 물리는 우리 공력 안 담음
- 안전장치 safety.py → Ubuntu 배포 시 controllers/ 복사 필요

### 우선순위 3: 실기체 준비
- RTK GPS 모듈 선정
- 컴패니언 컴퓨터 (Jetson 등, 미정)
- acados RTI + ROS2 노드 실시간 구현

### 나중에 (선택)
- 파라미터 현실화 (형상팀 값)
- 몬테카를로 분석
- NMPC trajectory-tracking 개선 (천이 성능)

## 주의사항

### 좌표계
- **관성**: NWU (z-up), **동체**: FRD (z-down)
- **PX4**: NED, 변환 = Rx(180°)
- 호버 q=[1,0,0,0], R=diag(1,-1,-1)

### 알려진 이슈
- NMPC/VirtualNMPC: reset() 시 `_last_t = -inf` 초기화 필수
- 가속도계 자세보정: 동적가속도 분리는 순환논리로 불가
- acados SQP 수렴률 ~30% (부분 수렴 해도 성능 유효)
- NMPC 천이 z=2.8: setpoint-tracking 한계 (버그 아님)

### 파일 구조
```
fast_drone/
├── vehicle_params.py        기체 파라미터 (플레이스홀더 v2)
├── dynamics.py              CasADi 6-DOF 플랜트 (바람 포함)
├── test_plant.py            플랜트 검증 6개
├── trim.py                  트림 탐색 + 속도 스윕
├── controller.py            PID, LQR, ScheduledPID/LQR, INDI
├── nmpc.py                  NMPC (CasADi+IPOPT)
├── nmpc_acados.py           NMPC (acados+HPIPM)
├── sweep.py                 PID/LQR 성능 저하 곡선
├── sweep_scheduled.py       고정/스케줄/NMPC 종합
├── gust_comparison.py       돌풍 비교 (6제어기)
├── hybrid_comparison.py     NMPC+INDI 하이브리드 + 모델미스매치
├── sensors.py               센서 모델 (IMU + GPS)
├── estimator.py             ESKF 15D 오차상태 칼만필터
├── ekf_sim.py               ESKF 통합 시뮬 루프
├── ekf_comparison.py        전 제어기 ESKF 비교 (표준/RTK GPS)
├── ekf_full_comparison.py   전 시나리오 RTK 종합 비교
├── TODO.md                  다음 단계 + 확정 사항
├── scripts/
│   ├── setup_ubuntu.sh      Ubuntu 환경 세팅
│   └── run_sitl.sh          SITL 실행 도우미
└── ros2_ws/src/fast_drone_ctrl/
    ├── frame_utils.py       NWU↔NED 좌표 변환
    ├── offboard_node.py     PX4 Offboard 제어 노드
    └── safety.py            안전장치 (5중 보호)
```
