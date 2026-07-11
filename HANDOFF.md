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

### ✅ 해결 (2026-07-11): SITL PID 호버 성공 — 30초+ 안정
**과거 증상**: 우리 PID로 fast_missile offboard 제어 시 **arm 직후 <1초 전복**.

**진짜 원인은 모터 매핑/부호가 아니었음** (아래 전부 실측으로 정상 확인 — 건드리지 말 것):
- `MOTOR_MAP=(0,2,1,3)` 위치 매핑 정상 (gz motorNumber별 pose를 FRD 변환해 대조)
- 프레임(NWU world / FRD body) 변환 정상, omega·자세 피드백 부호 정상
  (순수-요 오픈루프 test로 확인: 물리 기수우측 = 노드 +omega_z 보고)
- 롤·피치·요 오픈루프 플랜트 응답 전부 정상 (단일모터 bump로 tilt 방향 실측)
- 관성(0.02/0.70/0.70) SDF↔vehicle_params 일치

**실제 원인 2가지 (실측 + 오프라인 파이썬 재현 완료):**
1. **heading 미초기화**: `CascadedPID`가 절대 기수 0°(북) 목표인데 기체는 yaw≈−96°로
   스폰 → 큰 초기 요 오차 → 공격적 게인이 tilt로 커플 → 텀블. 파이썬 sim은 항상
   heading=0(=hover_state)에서 시작해 이 경로가 미검증이었음.
   → **첫 상태에서 controller.heading을 현재 기수로 래치**해 해결.
2. **자세게인 과대**: Kp_att=[200,500,500]/Kd_att=[20,50,50]가 무노이즈 가정 →
   SITL EKF omega 노이즈를 증폭 → 롤/피치 진동 발산. 오프라인에 omega 노이즈만
   넣어도 재현(×1.0은 노이즈 0.02에도 발산, ×0.35 안정).
   → **`att_gain_scale` 파라미터(기본 0.35)로 게인 축소** 해결.

**수정 위치**: `ros2_ws/src/fast_drone_ctrl/.../offboard_node.py`
  (heading 래치 + att_gain_scale + 진단훅 test_motor/DIAG). SDF는 momentConstant
  0.016→0.0833만 유지(요 권한 정합, 부호 무관).

**검증 결과**: `-p controller_type:=pid -p z_ref:=5.0` → 5m로 상승 후 **30초+ 안정
호버** (tilt≈0, yaw −96° 고정, XY 드리프트 ~0.1m).

**교훈**: 파이썬 sim은 이상적(무노이즈·heading0)이라 두 결함이 잠재. SITL은 실노이즈+
임의 스폰 기수로 처음 노출됨. 고속에선 게인 재튜닝 필요할 수 있음.

### ✅ 추가 완료 (2026-07-11): 기체타입 확정 + 외형 + 저속 전진(1a)

**기체타입 확정 = 멀티로터 (테일시터 아님!)**
- **날개 없음 → 테일시터 불가.** 로터가 항상 양력 담당하는 **멀티로터**.
- 비행: 저속엔 거의 안 기울고, 속도↑ → **연속적으로 기울며 가속**, 초고속에 **~30-40°까지** 기욺. **90° 안 눕음.**
- 30-40°는 현재 제어기 틸트 한계(35-55°) 안 → 제어 로직 손댈 것 없음.
- ⚠️ 세션 중 "테일시터/수직동체" 논쟁 있었으나 결론: **멀티로터 확정.** 재론 말 것.

**외형 (visual only, 물리 무관)** — `fast_missile_base/model.sdf`
- x500 프레임 메시 제거 → **세로 미사일 동체 + 로터 X암(밑동 링)** 프리미티브로 교체.
  사용자가 AAMAT/Red Bull 세로 미사일 이미지로 형상 확정. **이 외형 그대로 유지.**
- 관성·충돌·모터·센서는 x500 것 그대로 → 호버 영향 0.

**저속 전진 (step 1a) 성공**
- `-p controller_type:=pid -p v_ref_x:=6.0 -p z_ref:=15.0` → 공중에서 전진 확인.

## SITL 세션 (2026-07-12) — [1b] 제어기 연결 + 롤 불안정 규명

### 완료 ✅
- **모든 제어기 노드 연결**: lqr/indi/nmpc + **hybrid(ProperHybrid) 신규 연결**
  - `hybrid_comparison.py`를 controllers/로 동기화(sync_controllers.sh에 등록), `_create_controller`에 case 추가
  - SITL 실시간: VirtualNMPC `dt_ctrl=0.1(10Hz)/max_iter=5`, INDI ω̇는 **실측 Δt**로 계산(하드코딩 dt 제거)
- **노드 강건화**: `att_gain_scale` 컨트롤러별 일반화(lqr=K_r δφ·δω열, indi=Kp/Kd_indi),
  **LQR heading 재선형화**(`_relatch_lqr_heading`: 스폰기수로 트림 재선형화, δφ오차 1.49→0)
- **코드리뷰 버그 5건 수정** (INDI 전진비, 센서 RNG 재시드, NMPC 쿼터니언 초기추측,
  mission_sim 발산판정, launch 파라미터 타입) — 상세는 TODO.md "🐛 코드 리뷰 결과"

### 🔴 미해결: 롤 불안정 (LQR/INDI/Hybrid 전복)
- **PID만 호버 안정**, 나머지 3개는 전부 **롤축(omega_x) 발산 → 전복**
  - LQR: heading래치+게인축소로 개선했으나 롤 진동 발산 여전
  - Hybrid: 타이밍픽스로 루프 18Hz→100Hz 회복(핵심 진전), 그래도 t≈0.8s 롤 폭발
- **핵심 교훈**: **오프라인 모델(dynamics.py)은 이 발산을 재현 못 함** (무노이즈·무지연·무접촉으로 너무 관대).
  노이즈/지연/모터추정/무공력 다 시도해도 오프라인은 안정 → **컨트롤러 튜닝은 Gazebo에서만 가능.**
- **유력 원인 = Ixx=0.02**(피치/요의 1/35)로 롤 초민감. **단 관성만 스왑 불가**:
  현재 공력(S_ref=코단면·C_A0 유선형)이 **장축=X 가정**이라 300km/h 성립. 장축=Z면 공력 전면 재정의+300km/h 불가.

### 다음 (새 세션 여기서 시작)
1. **[결정] 장축 X vs Z 확정** — 공력+관성+외형을 함께 정하는 설계질문. 이게 롤 불안정의 뿌리.
   - 장축=X(수평 미사일): 현재 관성/공력 정합, 300km/h 가능, 호버 시 기체 수평
   - 장축=Z(수직 로켓): 관성 스왑(Ixx=Iyy=0.7,Izz=0.02)+공력 재정의 필요, 300km/h 불가
2. **결정 후** 관성/공력/SDF 정합 → LQR/Hybrid Gazebo 재시험 (튜닝은 Gazebo-in-the-loop)
3. **[2] Gazebo 공력 플러그인**(C++, `_body_aerodynamics` 포팅) — 축 결정에 종속
4. **[3] 고속(83 m/s)** + 제어기 전환 + 돌풍

### 진단 훅 (offboard_node.py에 추가됨, 필요시 정리)
- `test_motor` (0~3 단일 / 10 순수롤 / 11 순수피치 / 12 순수요): 플랜트 오픈루프 특성화
- `[DIAG]` 로그 (10Hz, lean+yaw+omega): 자세 진단
- `att_gain_scale` (기본 0.35), heading 래치, 실패세이프 로그 스로틀
- 오프라인 재현 스크립트: repo 루트 `scratch_yawtest.py`, `scratch_delaytest.py` (정리 대상)

### 참고: 헤드리스 SITL arm
QGC/RC 없이 offboard로 arm하려면 프리플라이트 통과 필요.
4100 에어프레임에 `NAV_RCL_ACT=0, NAV_DLL_ACT=0, COM_RCL_EXCEPT=7` 넣어둠.
런타임 강제: `build/px4_sitl_default/bin/px4-commander arm -f`

### 이후 순위
- Gazebo 커스텀 **공력 플러그인** (C++) — **결정 번복: 지금 임시계수로 만든다**
  (위 "다음 [2]" 참고. SITL을 제어기 검증 테스트베드로 만들기 위해. 형상팀 값은 나중에 교체)
- 실기체: RTK GPS + 컴패니언 컴퓨터 + acados RTI

## 주의사항

- NMPC/VirtualNMPC: reset() 시 _last_t + w0 초기화 필수
- INDI G: 전진비(advance ratio) 반영 필수 (compute_control_effectiveness)
- ESKF: update_gps에 실제 R_pos/R_vel 전달 필수
- 폴백: NaN 검사는 채터링 가드보다 항상 우선
- 감속 잔존 이슈: MC 3%에서 20m+ (폴백으로 대응)
- NMPC 단독 실기 부적합 (느리고 센서 취약)
