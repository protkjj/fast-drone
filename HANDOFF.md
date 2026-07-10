# HANDOFF — 고속 ISR 드론 제어 시뮬레이션

## 프로젝트 목표
축대칭 미사일형 동체 + 쿼드콥터 추진 드론의 고속(300 km/h) 비행 제어.
파이썬 시뮬 완료. 다음 = Ubuntu/Gazebo.

## 확정 사항
- **제어기: ProperHybrid** (VirtualNMPC + INDI 분리) — 통합 미션 전체 1위
- **센서: RTK GPS** (sigma=0.02m) — 표준GPS에서는 고성능 제어 불가
- **안전: HybridWithFallback** — 이상 감지 시 LQR 자동 전환
- **최종 목표: 실기체 비행** (PX4 + Gazebo -> 실기)

## 최종 결과 (통합 미션, 모든 버그 수정 후)

미션: 이륙(10s)->가속(15s)->순항+돌풍(15s)->감속(15s)->호버(7s) = 65초

### 참값 제어

| 제어기 | RMSE vx | RMSE z | 구간 1위 |
|--------|---------|--------|---------|
| LQR | 16.78 | 4.44 | 0/6 |
| **Hybrid** | **1.88** | **1.75** | **5/6** |
| NMPC | 3.73 | 2.80 | 1/6 (감속) |

### RTK EKF

| 제어기 | 참값 z | RTK EKF z | 변화 |
|--------|-------|----------|------|
| LQR | 4.44 | 4.44 | 0% |
| **Hybrid** | **1.75** | **1.67** | **-4.7%** |

### MC 10회: Hybrid 최악(2.82) < LQR 최선(4.41) = 통계적 우위 확실

## 이번 세션 핵심 작업

1. **통합 미션 시뮬 구축** (mission_sim.py)
   - 전 비행구간을 하나로: 개별 시나리오 불필요
   - MC + RTK EKF + 시각화 포함

2. **감속 폭발 발견 및 해결**
   - 원인: INDI G 행렬이 전진비 무시 -> 감속 중 추력 23% 과대평가
   - 수정: dT/dn = k_T * n * (1 + fac) (3-1)
   - 결과: 감속 RMSE z 8.15 -> 1.45 (-82%)
   - RTK EKF 추가: 1.45 -> 0.83 (-43%)

3. **버그 5건 수정**
   - 1-1: 폴백 NaN 검사 위치 (안전)
   - 1-2: RTK R값 ESKF 전달 (RTK 결과 신뢰성)
   - 1-3: NMPC w0 warm start 리셋 (MC 독립성)
   - 2-2: VirtualNMPC T_ref 전달
   - 3-1: INDI G 전진비 반영 (감속 해결)

4. **폴더 정리**
   - legacy/: 이전 스크립트 11개
   - results/: 결과물 (분석.md, 플롯.png, 결과.txt)

## 파일 구조

```
fast_drone/
├── dynamics.py, vehicle_params.py, trim.py     플랜트 핵심
├── controller.py, nmpc.py                      제어기
├── hybrid_comparison.py                        ProperHybrid + VirtualNMPC
├── fallback_controller.py                      Hybrid+LQR 폴백
├── sensors.py, estimator.py, ekf_sim.py        ESKF
├── ekf_comparison.py, gust_comparison.py       유틸
├── mission_sim.py                              통합 미션 (메인 진입점)
├── test_plant.py                               플랜트 테스트
├── results/                                    결과물
│   ├── PROJECT_REPORT.md                       종합 보고서
│   ├── MISSION_ANALYSIS.md                     미션 분석
│   └── FILES.md                                파일 설명
├── legacy/                                     이전 스크립트
├── ros2_ws/                                    ROS2 패키지
└── scripts/                                    Ubuntu 세팅
```

## SITL 세션 (2026-07-10~11) — Ubuntu/Gazebo/PX4

### 완료 ✅
- 환경: Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic + micro-XRCE-DDS + **PX4 v1.18-beta**
- PX4 SITL 빌드 + x500 이착륙 검증 (`commander takeoff/land`)
- ROS2 워크스페이스 빌드: px4_msgs(**main**, 펌웨어와 버전 맞춤) + px4_ros_com + fast_drone_ctrl
- offboard_node.py PX4 v1.18 호환 수정 (아래 "주의: v1.18" 참고)
- **커스텀 기체 `fast_missile` 생성** (1+2층): x500 복제 후 우리 vehicle_params로
  - 모델: `PX4-Autopilot/Tools/simulation/gz/models/fast_missile{,_base}/`
    질량 8kg, 관성 0.02/0.70/0.70, motorConstant 6e-5, maxRotVel 1800, rotorVelSlowdown 50
  - 에어프레임: `.../init.d-posix/airframes/4100_gz_fast_missile`
    EC_MIN 0, EC_MAX 1800, MPC_THR_HOVER 0.318, 축별 rate게인, 헤드리스 arm 허용
  - 실행: `make px4_sitl gz_fast_missile`
- **버그 2건 수정**
  - safety.py rate-limiter 고착: `_last_valid_u`가 WARNING 시 미갱신 → 초기값 고착
    → 출력이 hover±max_delta에 갇혀 자세제어 불가. **항상 갱신하도록 수정**
  - PX4 v1.18 호환: 토픽명 `_v1`/`_v4` 접미사, actuator→direct_actuator,
    ActuatorMotors 12채널, QoS durability VOLATILE, vehicle_odometry에서 각속도

### ⚠️ 미해결 (다음 세션 1순위): 모터 매핑/부호 캘리브레이션
**증상**: 우리 PID로 fast_missile offboard 제어 시, 자세제어(모터 차동)가 실제로
작동하면 **arm 직후 <1초 만에 전복** → 물리 발산(속도 폭발).

**분석**:
- PX4 자체 제어기로는 이 극단적 비대칭 관성(롤0.02:피치0.70=1:35)을 못 잡음(예상됨,
  명제 검증). 우리 제어기가 답.
- 우리 direct-actuator 제어기는 PX4 할당을 우회 → **gz 모터의 물리 토크 부호가
  dynamics.py의 rotor_positions/rotor_directions와 정확히 일치해야 함.**
- offboard_node.py에 `MOTOR_MAP=(0,2,1,3)` (control[1]↔[2] 스왑) 넣었으나 **미검증**.
  (safety 고착으로 차동이 0이던 동안엔 똑바로 상승 → 매핑 검증된 걸로 착각했었음)
- 즉시 전복 = 롤/피치 부호 오류 신호 (느린 요 드리프트 아님).

**해결 절차 (추측 금지, 실측)**:
1. offboard 노드 대신 PX4 `actuator_test` 또는 개별 모터 명령으로 **모터 하나씩** 구동
2. gz에서 기체가 +롤/+피치/+요 중 어디로 기우는지 기록 → 4×(위치·스핀) 실측표 작성
3. dynamics.py 규약(FRD z-down, M_z=dir·k_Q·n², 추력 [0,0,-T])과 대조
4. gz 모델 turningDirection + offboard_node MOTOR_MAP 확정 → PID 재검증
5. PID 호버 성공 후 → LQR → INDI → NMPC → Hybrid 순으로 확장

### 참고: 헤드리스 SITL arm
QGC/RC 없이 offboard로 arm하려면 프리플라이트 통과 필요.
4100 에어프레임에 `NAV_RCL_ACT=0, NAV_DLL_ACT=0, COM_RCL_EXCEPT=7` 넣어둠.
런타임 강제: `build/px4_sitl_default/bin/px4-commander arm -f`

### 이후 순위
- Gazebo 커스텀 **공력 플러그인** (C++, 3층) — 형상팀 실제 공력값 도착 후 (지금은 임시값이라 보류)
- 실기체: RTK GPS + 컴패니언 컴퓨터 + acados RTI

## 주의사항

- NMPC/VirtualNMPC: reset() 시 _last_t + w0 초기화 필수
- INDI G: 전진비(advance ratio) 반영 필수 (compute_control_effectiveness)
- ESKF: update_gps에 실제 R_pos/R_vel 전달 필수
- 폴백: NaN 검사는 채터링 가드보다 항상 우선
- 감속 잔존 이슈: MC 3%에서 20m+ (폴백으로 대응)
- NMPC 단독 실기 부적합 (느리고 센서 취약)
