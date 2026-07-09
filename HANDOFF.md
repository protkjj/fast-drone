# HANDOFF — 고속 ISR 드론 제어 시뮬레이션

## 프로젝트 목표
축대칭 미사일형 동체 + 쿼드콥터 추진 드론의 고속(300 km/h) 비행 제어.
6개 제어기(PID, PID스케줄, LQR, LQR스케줄, NMPC, INDI)를
5개 시나리오(순항, 천이, 발사, 돌풍, 모델미스매치)에서 비교.

## 핵심 결론 (논문 요약)

### 정량 결과 — 종합 표

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
  최대 z오차     | 6.4     5.9       2.0      0.5*      4.5      -
─────────────────┼──────────────────────────────────────────────────────────
발사천이 70 m/s  |
  고도 손실      | 2.46    1.99      0.26*    0.49      0.60     -
  안정화 [s]     | 1.57    1.21*     5.00✗    5.00✗     2.01     -
─────────────────┼──────────────────────────────────────────────────────────
돌풍 (측풍 10m/s)|
  최대 Δz        | 1.28    1.18      0.05*    0.05*     0.10     1.35
  회복 [s]       | 6.0✗    6.0✗      즉시*    즉시*     즉시     6.0✗
돌풍 (수직 10m/s)|
  최대 Δz        | 1.16    1.12      0.29     0.26*     0.32     1.21
  회복 [s]       | 6.0✗    6.0✗      즉시*    즉시*     즉시     6.0✗
─────────────────┼──────────────────────────────────────────────────────────
모델미스매치     |        (C_Na ±20%, 순항+돌풍)
  NMPC           |                                       0.080~0.112
  NMPC+INDI(naive)|                                      0.263~0.472 (악화!)
  LQR스케줄      |                                                   0.100~0.107

  * = 해당 시나리오 최고 성능,  ✗ = 미달/미회복
```

### 제어기별 평가

| 제어기 | 강점 | 약점 |
|--------|------|------|
| **PID 고정** | 단순, 저속 안정 | 65 m/s 무릎점, 돌풍 회복 느림 |
| **PID 스케줄** | 고정 대비 15~20% 개선 | 여전히 고속·돌풍에 약함 |
| **LQR 고정** | 풀스테이트 피드백 → 돌풍 즉각 응답 | 고속 궤적추종 약함 |
| **LQR 스케줄** | 돌풍·고도 유지 최고, 전 영역 안정 | 속도 추종 느림 (궤적 계획 없음) |
| **NMPC** | 순항 고도 최고, 속도 추종 최고 | Q_z 가중치 민감, 돌풍에서 LQR보다 느림 |
| **INDI 단독** | 이론적 강건성 | 외측(PID) 병목으로 PID급 성능 |
| **NMPC+INDI (naive)** | — | 이중 보정으로 순수 NMPC보다 악화 |
| **NMPC+INDI (분리)** | 전 조건 최강 (돌풍+모델오차) | 구현 복잡도 (VirtualNMPC 필요) |

### 핵심 발견 5가지

1. **"만능 제어기"는 없다** — 시나리오에 따라 최적이 다름
   - 순항 고도: NMPC (0.21) > LQR (0.34) > PID (1.55)
   - 돌풍 응답: LQR (0.05m) > NMPC (0.10m) > PID (1.18m)
   - 속도 추종: NMPC (4.80) > PID (7.77) > LQR (16.77)

2. **게인 스케줄링은 유효하지만 한계 있음**
   - PID: 85 m/s z RMSE 1.85→1.55 (16% 개선)
   - LQR: 85 m/s vx RMSE 0.67→0.43 (36% 개선)
   - 그래도 NMPC에는 못 미침 (순항)

3. **NMPC 고도 성능은 비용 가중치(Q_z)에 민감**
   - Q_z=20: z_max=4.5m, Q_z=200: z_max=1.6m (vx 거의 불변)
   - "속도 우선"과 "고도 우선"은 trade-off
   - NMPC가 만능이 아닌 이유

4. **돌풍 억제: 제어기 아키텍처 > 내측 알고리즘**
   - LQR(풀스테이트 피드백) ≫ INDI(센서기반 내측)
   - 병목은 외측 루프 (위치→자세 명령). 내측 개선은 무의미
   - INDI는 외측도 INDI여야 효과 (Incremental outer loop)

5. **NMPC+INDI: 나이브 실패 → 인터페이스 분리로 해결**
   - 나이브(둘 다 모터속도): 이중 보정 → NMPC보다 3.5배 악화
   - 분리(NMPC→[T,ω̇], INDI→모터): **전 조건 최강**
   - 돌풍: 분리 0.053 vs NMPC 0.092 vs LQR 0.064 (1.7배 개선)
   - C_Na-20%+돌풍: 분리 0.038 vs NMPC 0.112 (3배 개선)
   - 핵심: "뭘 할지"와 "어떻게 할지"의 명확한 역할 분리

## 완료된 것 (전체)

### 플랜트 모델 (`dynamics.py`)
- CasADi 6-DOF: 미사일 동체 공력 + 4로터 쿼드콥터 추진
- 상태 17D: [p(3), v(3), q(4), ω(3), n(4)], 제어 4D: [n1~n4]
- 좌표계: 관성 z-up, 동체 z-down (미사일 공력 관례)
- V=0 특이점: 대수적 소거, Baumgarte 쿼터니언 안정화
- 전진비 J 의존 추력 모델
- **바람 입력**: v_body = R^T @ (vel - w), simulate(wind_fn=...) 지원
- 검증: `test_plant.py` 6개 테스트 통과

### 파라미터 (`vehicle_params.py`)
- 플레이스홀더 v2: C_A0=0.12, n_max=1800 (팁마하 0.79)
- 85 m/s (306 km/h) 트림 가능

### 트림 (`trim.py`)
- scipy.fsolve: [θ, n_eq, Δn] 탐색, 0~85 m/s 전 구간 수렴

### 제어기 (`controller.py`)
- **CascadedPID**: 속도P + 고도PID → SO(3) 자세PD → 할당
- **LQRController**: Error-State 17D→14D + ARE (Q_L 부호 수정 완료)
- **ScheduledPID**: 속도별 max_tilt/Kp_att/Kd_att 연속 스케일링
- **ScheduledLQR**: np.interp 선형 보간 (0~80 m/s, 10 간격, K_r+x_trim+u_trim)
- **INDIController**: 외측PID + 내측INDI (LPF 50Hz, G(4×4) 실시간 계산)

### NMPC
- **CasADi+IPOPT** (`nmpc.py`): Q_z 파라미터화, IPOPT 풀이 통계
- **acados+HPIPM** (`nmpc_acados.py`): SQP + yref 파라미터화

### 비교 스크립트
- `sweep.py`: PID/LQR 성능 저하 곡선 (30 m/s 게인 고정)
- `sweep_scheduled.py`: 고정/스케줄/NMPC × 순항·천이·발사 종합
- `gust_comparison.py`: 1-cosine 돌풍 × 6제어기 (측풍/수직)
- `hybrid_comparison.py`: NMPC+INDI 모델미스매치(C_Na ±20%) + 돌풍

## 향후 과제

### 우선순위 1: 시뮬레이터 연동 (다음 단계)
- PX4 SITL + Gazebo/Isaac Sim
- 현재 제어기(ProperHybrid)를 ROS2 노드로 이식
- 3D 시각화 + 실기 비행 준비

### ✅ 완료: NMPC 아키텍처 분리
- VirtualNMPC(13D, [T,ω̇] 출력) + INDI(모터속도 실행) → `hybrid_comparison.py`
- 전 조건 최강 확인 (정확/미스매치/돌풍 모두)

### 우선순위 2: 실제 파라미터 반영
- 형상팀 확정 후 C_Na, C_A0, 질량특성 업데이트
- 트림·게인 스케줄 전체 재계산

### 우선순위 3: 실시간 구현
- acados SQP_RTI 고속 튜닝 (현재 30 m/s만 검증)
- INDI 1kHz 루프 + NMPC 50Hz 루프 분리
- ROS2 노드 구현

### 기타
- 센서 노이즈 모델 추가 (INDI LPF 실효성 검증)
- 6-DOF 풍동 데이터 반영 (현재는 해석적 공력 모델)
- INDI 외측 루프 (Incremental Backstepping 등)

## 주의사항

### 좌표계 (혼동 주의!)
- **관성**: z-up, 중력=[0,0,-mg]
- **동체**: x전방, y우측, **z하방** (우수: x×y=z)
- 양의 AoA: w_b > 0 (공기가 아래에서)
- 양의 M_y: 기수 상승 (x→-z, 우수법칙)
- 호버 R = [[1,0,0],[0,-1,0],[0,0,-1]]
- 로터 추력: body -z (위로 뜸)

### acados 실행 환경
```bash
export ACADOS_SOURCE_DIR=~/acados
export DYLD_LIBRARY_PATH=~/acados/lib
```

### 파일 구조
```
fast_drone/
├── vehicle_params.py        기체 파라미터 (플레이스홀더 v2)
├── dynamics.py              CasADi 6-DOF 플랜트 (바람 포함)
├── test_plant.py            플랜트 검증 6개
├── trim.py                  트림 탐색 + 속도 스윕
├── controller.py            PID, LQR, ScheduledPID/LQR, INDI
├── nmpc.py                  NMPC (CasADi+IPOPT, Q_z 조정 가능)
├── nmpc_acados.py           NMPC (acados+HPIPM)
├── sweep.py                 PID/LQR 성능 저하 곡선
├── sweep_scheduled.py       고정/스케줄/NMPC 종합 (순항·천이·발사)
├── gust_comparison.py       돌풍 비교 (측풍/수직 × 6제어기)
├── hybrid_comparison.py     NMPC+INDI 모델미스매치 검증
├── launch_transition.py     발사천이 (이전 버전)
├── takeoff_cruise.py        호버→순항 (이전 버전)
├── rotor.py                 이전 버전 동역학 (참고용)
└── rotorpy_multirotor.py    RotorPy 원본 (참고용)
```

### 핵심 함수 진입점
- `dynamics.build_dynamics(params)` → CasADi Function (NMPC/acados 직접 전달)
- `dynamics.AxialDronePlant(params).simulate(x0, ctrl, T, wind_fn=...)` → 시뮬레이션
- `trim.find_trim(params, V)` → 트림 상태/제어
- `controller.ScheduledLQR(params, v_ref, z_ref)` → 게인 스케줄링 LQR
- `controller.INDIController(params, v_ref, z_ref, dt)` → INDI
- `nmpc.NMPCController(params, v_ref, z_ref, Q_z=20)` → NMPC (Q_z 조정)

### 알려진 이슈
- acados SQP 수렴률 ~30% (부분 수렴 해도 성능 유효)
- NMPC IPOPT 계산시간 ~20ms (50Hz OK, 더 빠른 루프엔 RTI 전환)
- INDI 단독: 외측(PID) 병목으로 PID급 성능 (풀스테이트 외측 필요)
- NMPC+INDI naive: 이중 보정으로 악화 (인터페이스 분리 필수)
- NMPC Q_z=20: 속도 우선 설정. 고도 우선이면 Q_z=100~200 권장
