# 코드 구조 및 파일 설명

> 최종 업데이트: 2026-07-10
> 상태: 파이썬 시뮬 완료, 다음 단계 = Ubuntu/Gazebo

---

## 전체 구조

```
fast_drone/
├── dynamics.py              CasADi 6-DOF 플랜트 (핵심)
├── vehicle_params.py        기체 파라미터
├── trim.py                  트림 탐색 + 속도 스윕
├── controller.py            PID / LQR / ScheduledPID/LQR / INDI
├── nmpc.py                  NMPC (CasADi+IPOPT)
├── hybrid_comparison.py     VirtualNMPC + ProperHybrid + 공통 G행렬
├── fallback_controller.py   Hybrid+LQR 폴백 (이상 감지 시 자동 전환)
├── sensors.py               센서 모델 (IMU + GPS)
├── estimator.py             ESKF 15D 오차상태 칼만필터
├── ekf_sim.py               EKF 통합 시뮬 루프
├── ekf_comparison.py        EKF 비교 + 유틸 (_reset_controller 등)
├── gust_comparison.py       돌풍 함수 (make_gust_fn) + 비교 스크립트
├── mission_sim.py           통합 미션 시뮬 (전 구간 + MC + RTK EKF)
├── test_plant.py            플랜트 검증 6개 테스트
│
├── results/                 결과물
│   ├── MISSION_ANALYSIS.md  최종 분석 문서
│   ├── FILES.md             이 파일
│   ├── mission_plot.png     미션 시뮬 플롯
│   └── *.txt                시뮬 결과 텍스트
│
├── legacy/                  이전 스크립트 (사용 안 함)
│   ├── sweep.py, sweep_scheduled.py
│   ├── launch_transition.py, takeoff_cruise.py
│   ├── ekf_full_comparison.py, monte_carlo.py, imu_sweep.py
│   ├── nmpc_acados.py, rotor.py, rotorpy_multirotor.py
│   └── FILES.md (이전 버전)
│
├── ros2_ws/                 ROS2 패키지 (Phase 0, macOS에서 작성)
│   └── src/fast_drone_ctrl/
│       ├── frame_utils.py   NWU↔NED 좌표 변환
│       ├── offboard_node.py PX4 Offboard 직접 액추에이터 제어
│       └── safety.py        NaN/변화율/자세/고도 5중 보호
│
└── scripts/
    ├── setup_ubuntu.sh      Ubuntu 환경 세팅
    └── run_sitl.sh          SITL 실행 도우미
```

---

## 의존성 흐름도

```
vehicle_params.py ──────────────────────────────────────┐
        │                                               │
        ▼                                               │
dynamics.py (CasADi 6-DOF)                              │
        │                                               │
   ┌────┼───────────────────┐                           │
   ▼    ▼                   ▼                           │
trim.py  test_plant.py   build_dynamics()               │
   │                        │                           │
   ▼                        ▼                           │
controller.py           nmpc.py                         │
 PID, LQR, INDI         CasADi+IPOPT                   │
 ScheduledPID/LQR                                       │
   │                        │                           │
   ▼                        ▼                           │
hybrid_comparison.py ◄──────┘                           │
 VirtualNMPC (13D)                                      │
 ProperHybrid (INDI 실행자)                              │
 compute_control_effectiveness()                        │
        │                                               │
        ▼                                               │
fallback_controller.py                                  │
 HybridWithFallback                                     │
        │                                               │
   ┌────┼───────────────────┐                           │
   ▼    ▼                   ▼                           │
sensors.py  estimator.py  gust_comparison.py            │
 IMU+GPS    ESKF 15D      make_gust_fn()                │
   │           │                │                       │
   ▼           ▼                │                       │
ekf_sim.py ◄───┘                │                       │
   │                            │                       │
   ▼                            ▼                       │
ekf_comparison.py ◄─────────────┘                       │
 _reset_controller()                                    │
 create_sensors()                                       │
        │                                               │
        ▼                                               │
mission_sim.py ◄────────────────────────────────────────┘
 MissionProfile (구간별 기준 궤적)
 MissionController (기준값 동적 갱신 래퍼)
 run_mission() / run_mission_ekf()
 run_monte_carlo()
 diagnose_deceleration()
 plot_mission()
```

---

## 파일별 상세 설명

### 1. dynamics.py — 6-DOF 플랜트 모델 (295줄)

프로젝트의 기반. CasADi 심볼릭으로 작성되어 acados/IPOPT에 직접 전달 가능.

**핵심 함수:**
- `_quat_to_rotmat(q)` — 쿼터니언 [x,y,z,w] -> 3x3 회전행렬
- `_quat_derivative(q, w)` — 쿼터니언 미분 + Baumgarte 안정화
- `_body_aerodynamics(v, w, p)` — 축대칭 동체 공력 (수직력, 축력, 모멘트, V=0 소거)
- `_rotor_forces_moments(v, n, w, p)` — 4로터 추력/반토크/자이로 (전진비 J 의존)
- `_compute_xdot(x, u, p, w)` — 전체 조립 -> x_dot = f(x, u, w)
- `build_dynamics(params)` — CasADi Function 생성
- `compute_allocation_matrix(params)` — [T1..T4] <-> [T, Mx, My, Mz]
- `AxialDronePlant` — 시뮬 래퍼 (RK4, 쿼터니언 정규화, 바람 입력)

**좌표계:**
- 관성: NWU (z-up), 동체: FRD (z-down)
- 호버 쿼터니언: q=[1,0,0,0] -> R=diag(1,-1,-1) (180도 about x)

**상태 x(17):** [pos(3), vel(3), quat(4), omega(3), n_rotor(4)]
**제어 u(4):** [n1_cmd, n2_cmd, n3_cmd, n4_cmd]

---

### 2. vehicle_params.py — 기체 파라미터 (85줄)

물리 상수 한 딕셔너리. 형상팀 값 확정 시 이 파일만 교체.

- 질량/관성: 8kg, 축대칭 (Iyy=Izz=0.70)
- 공력: C_A0=0.12 (유선형 오자이브), C_Na=6.0, x_cp=-0.10
- 로터: 4개 X 패턴, k_T=6e-5, n_max=1800 rad/s
- 모터: tau_m=20ms

**조정 이력:** C_A0=0.50(둔탁한 원통) -> 0.12(유선형). 85 m/s 트림 가능해짐.
**트림 여유:** 85 m/s에서 모터 52% 사용 (48% 여유). 실기 적정 T/W는 5~6.

---

### 3. trim.py — 트림 탐색 (196줄)

순항 속도 V에서 정상 수평비행 조건 [theta, n_eq, dn] 탐색.

- `find_trim(params, V)` — scipy.fsolve, 3변수 3방정식
- `trim_speed_sweep(params)` — 0~85 m/s 스윕
- 3방정식: v_dot_x=0, v_dot_z=0, omega_dot_y=0

결과: 0~85 m/s (306 km/h) 전 구간 수렴.

---

### 4. controller.py — 제어기 5종 (798줄)

**CascadedPID** — 외부(속도P+고도PID) -> 중간(힘->자세+추력) -> 내부(자세PD+자이로FF) -> 할당

**LQRController** — Error-State 14D. CasADi 야코비안 -> T(17x14) 축소 -> ARE -> K_r(4x14)

**ScheduledPID** — CascadedPID 상속. v_x에 따라 max_tilt(35->55도), Kp_att, Kd_att 연속 스케일링

**ScheduledLQR** — 0~80 m/s (10 간격) 게인 테이블. np.interp 선형 보간. 쿼터니언 재정규화.

**INDIController** — 외측(PID 동일) + 내측(INDI):
  omega_dot_meas = LPF(delta_omega/dt), G(4x4) = d[T,omega_dot]/dn, delta_n = G_inv @ dv

---

### 5. nmpc.py — NMPC CasADi+IPOPT (215줄)

Direct Multiple Shooting. N=20, dt=0.05s (1초 예측).

- `NMPCController` — dt_ctrl=0.02s 주기로 NLP 풀이, warm start 시프트
- `reset()` — _last_t, _u_current, w0 모두 초기화 (MC 독립성 보장)
- 파라미터: p = [x_init(17), v_ref(3), z_ref(1), u_ref(4)] = 25차원

계산시간: IPOPT ~165초/65초 미션 (실시간 불가). acados RTI는 21ms (50Hz 가능).

---

### 6. hybrid_comparison.py — 하이브리드 제어기 (~350줄)

**VirtualNMPC (13D)**
- 상태: [pos(3), vel(3), quat(4), omega(3)] — 로터 상태 제거
- 제어: [T_total, omega_dot_x, omega_dot_y, omega_dot_z] — 가상 명령
- NMPC가 "뭘 할지" 결정, 모터 할당은 INDI에게 위임
- `reset()` — _last_t + _u_current + w0 초기화

**ProperHybrid (인터페이스 분리)**
- VirtualNMPC -> [T_cmd, omega_dot_des] -> INDI -> [n1..n4]
- 이중 보정 없음: NMPC는 모터를 모르고, INDI만 모터 제어
- `reset()` — INDI 상태 + VirtualNMPC.reset() 호출

**compute_control_effectiveness(params, n_actual, v_body=None)**
- G(4x4) = d[T, omega_dot]/dn
- 전진비(advance ratio) 반영: dT/dn = k_T * n * (1 + fac)
- v_body 전달 시 V_axial에서 fac 계산, 미전달 시 fac=1 (호버 가정)
- 감속 중 ~23% 추력 과대평가 문제를 해결 (3-1 수정)

**NaiveHybrid** — 비교용. 이중 보정으로 3.5배 악화 확인.

---

### 7. fallback_controller.py — 폴백 제어기 (190줄)

**HybridWithFallback**
- 기본: Hybrid, 이상 감지 시 LQR 자동 전환
- 감지 기준: NaN/Inf, 극단 출력, 고도 오차, 속도 이상, 모터 진동
- NaN/Inf 검사는 채터링 가드보다 우선 (1-1 수정)
- 복귀: 쿨다운 2초 + 상태 안정 확인 후 Hybrid로 복귀

---

### 8. sensors.py — 센서 모델 (281줄)

**IMUSensor** — 가속도계(비력) + 자이로. 바이어스 + 가우시안 노이즈.
**GPSSensor** — 위치 + 속도. 10Hz 주기. 노이즈 레벨 조절 가능.
**SensorSuite** — IMU + GPS 묶음. `measure(t, x_true, acc_inertial)` -> SensorData
**create_default_sensors()** — noise_level로 스윕 가능 (0.5x ~ 2.0x)

| 센서 | 노이즈 (sigma) | 주파수 |
|------|--------------|--------|
| 가속도계 | 0.02 m/s2 | 1 kHz |
| 자이로 | 0.001 rad/s | 1 kHz |
| GPS 위치 | 1.5 m (표준) / 0.02 m (RTK) | 10 Hz |
| GPS 속도 | 0.5 m/s | 10 Hz |

---

### 9. estimator.py — ESKF 15D (342줄)

Error-State Kalman Filter.

**오차상태 15D:** [dp(3), dv(3), dtheta(3), db_a(3), db_g(3)]
**명목상태 16D:** [pos(3), vel(3), quat(4), bias_acc(3), bias_gyro(3)]

- `predict(acc_body, gyro_body, dt)` — IMU 적분 + 공분산 전파
- `update_gps(pos, vel, R_pos, R_vel)` — GPS 측정 업데이트 (Joseph form)
- `update_accel(acc_body, R_acc)` — 가속도계 자세 보정 (적응적 R, 기동 시 약화)
- `get_state()` -> x(17) 형태 반환
- `_inject(dx)` — 오차상태를 명목에 주입 (쿼터니언 exp map)

**R_pos/R_vel 전달 (1-2 수정):** RTK(0.02m) 사용 시 EKF가 GPS를 75배 더 신뢰.

---

### 10. ekf_sim.py — EKF 시뮬 루프 (197줄)

Plant -> Sensors -> ESKF -> Controller -> Plant 루프.

- `simulate_with_ekf(plant, ctrl, x0, T, sensors, seed, wind_fn)` — EKF 포함 시뮬
- `simulate_perfect(plant, ctrl, x0, T, wind_fn)` — 참값 제어 (비교 기준)
- `compute_metrics(result, z_ref, v_ref_x)` — RMSE, 최대 편차
- `compute_estimation_metrics(result)` — 추정 오차 (위치, 속도, 자세)

센서의 실제 노이즈(R_pos, R_vel)를 ESKF에 전달 (1-2 수정 반영).

---

### 11. ekf_comparison.py — EKF 유틸리티 (~100줄)

다른 파일들이 임포트하는 유틸 함수:

- `_reset_controller(ctrl)` — NMPC _last_t, _u_current, w0 + 체인 리셋
- `create_sensors(dt, noise_level, gps_noise_pos, seed)` — 센서 세트 생성
- `make_controllers(V, z_ref, dt, x_trim, u_trim)` — 5종 제어기 생성

---

### 12. gust_comparison.py — 돌풍 함수 + 비교 (322줄)

**make_gust_fn(direction, W_max, t_start, T_gust)** — 1-cosine 돌풍 생성 (FAR 25 표준)
- mission_sim.py, ekf_comparison.py 등에서 임포트

나머지는 6제어기 x 2돌풍 비교 스크립트 (main 실행 시).

---

### 13. mission_sim.py — 통합 미션 시뮬레이션 (~800줄)

전 비행구간을 하나로: 이륙 -> 가속 -> 순항(+돌풍) -> 감속 -> 호버링 (65초).

**MissionProfile** — 구간별 1-cos 스무스 기준 궤적
**MissionController** — 제어기 기준값 동적 갱신 래퍼
**run_mission()** — 참값 제어 시뮬
**run_mission_ekf()** — RTK EKF 포함 시뮬
**run_monte_carlo()** — MC (초기조건 + 돌풍 랜덤화)
**diagnose_deceleration(n_trials, nmpc_N)** — 감속 폭발 진단
**plot_mission()** — 4행 플롯 (v_x, z, dz, motor) + 구간 라벨 + 돌풍 표시

main() 실행 시: [1] 단일 미션 3제어기 -> [2] MC 10회 -> [3] RTK EKF

---

### 14. test_plant.py — 플랜트 검증 (180줄)

| # | 테스트 | 검증 |
|---|--------|------|
| 1 | 자유낙하 | 로터 0 -> v_z = -9.81 m/s (1초) |
| 2 | 호버 | n_hover -> dz < 0.1m (2초) |
| 3 | 복원 모멘트 | 양의 alpha -> M_y < 0 (기수 하강) |
| 4 | 감쇠 | C_mq < 0 -> 각속도 감쇠 |
| 5 | 전진비 | 상승 시 V_axial > 0 -> 추력 감소 |
| 6 | 할당 행렬 | A*A_inv = I |

---

## 실행 방법

```bash
# 플랜트 검증
python3 test_plant.py

# 트림 스윕
python3 trim.py

# 통합 미션 (3제어기 + MC + RTK EKF, ~25분)
python3 mission_sim.py > results/mission_result.txt 2>&1

# 감속 진단 (30회, ~25분)
python3 -c "from mission_sim import diagnose_deceleration; diagnose_deceleration(30)"

# 돌풍 비교 (6제어기, ~2분)
python3 gust_comparison.py
```

---

## 핵심 진입점 (개발자용)

```python
from dynamics import AxialDronePlant, build_dynamics
from vehicle_params import vehicle_params as P
from trim import find_trim
from controller import (CascadedPID, LQRController,
                        ScheduledPID, ScheduledLQR, INDIController)
from nmpc import NMPCController
from hybrid_comparison import VirtualNMPC, ProperHybrid
from fallback_controller import HybridWithFallback
from sensors import create_default_sensors
from estimator import ESKF

# 플랜트
plant = AxialDronePlant(P, dt=0.001)

# 트림
trim = find_trim(P, 70.0)

# 제어기
slqr = ScheduledLQR(P, v_ref=[70,0,0], z_ref=50)
vnmpc = VirtualNMPC(P, v_ref=[70,0,0], z_ref=50, N=20, dt_nmpc=0.05, dt_ctrl=0.02)
hybrid = ProperHybrid(vnmpc, P, dt=0.001)
fallback = HybridWithFallback(hybrid, slqr, z_ref=50)

# 시뮬 (바람 없이)
x0 = plant.hover_state(P); x0[2] = 50
ts, xs, us = plant.simulate(x0, hybrid, T=10.0)

# 시뮬 (돌풍 포함)
from gust_comparison import make_gust_fn
gust = make_gust_fn('vertical', 10.0, 3.0, 1.0)
ts, xs, us = plant.simulate(x0, hybrid, T=10.0, wind_fn=gust)

# 통합 미션
from mission_sim import MissionProfile, MissionController, run_mission
profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
ctrl = MissionController(hybrid, profile)
result = run_mission(plant, ctrl, x0, profile, wind_fn=gust)
```

---

## 수정 이력 (최종 세션)

| # | 수정 | 파일 | 영향 |
|---|------|------|------|
| 1-1 | 폴백 NaN 검사 -> 채터링 가드 위로 | fallback_controller.py | 1초간 무방비 해소 |
| 1-2 | RTK R값 ESKF에 전달 | ekf_sim.py | RTK 성능 정상 반영 |
| 1-3 | NMPC/VirtualNMPC w0 reset | nmpc.py, hybrid_comparison.py | MC 독립성 |
| 2-2 | VirtualNMPC T_ref 전달 | ekf_comparison.py | 비용함수 정확도 |
| 3-1 | INDI G 전진비 반영 | hybrid_comparison.py | 감속 폭발 -82% |

---

## 최종 결과 요약

**Hybrid + RTK EKF = RMSE z 1.670m** (전 구간 1위)

| 구간 | Hybrid RMSE z | 비고 |
|------|-------------|------|
| 이륙 | 4.08m | |
| 가속 | 0.28m | |
| 순항 | 0.46m | |
| 감속 | 0.83m | 수정 전 8.15 -> 수정 후 0.83 |
| 호버링 | 0.009m | 거의 완벽 |
| 돌풍 | Dz=0.15m | 거의 무반응 |

MC 10회: Hybrid 최악(2.82) < LQR 최선(4.41) = 통계적 우위 확실.
