# 파일별 설명 — 고속 ISR 드론 제어 시뮬레이션

## 전체 흐름도

```
vehicle_params.py (파라미터)
        │
        ▼
dynamics.py (CasADi 6-DOF, 바람 포함)  ← 모든 것의 기반
        │
   ┌────┼───────────────┐
   ▼    ▼               ▼
trim.py  test_plant.py   build_dynamics() → acados/IPOPT
   │                     │
   ▼                     ▼
controller.py          nmpc.py / nmpc_acados.py
 PID, LQR,             CasADi+IPOPT, acados+HPIPM
 ScheduledPID/LQR,
 INDI
   │                     │
   ▼                     ▼
sweep.py              sweep_scheduled.py
(성능 저하 곡선)       (종합 비교: 순항·천이·발사)
                         │
                    ┌────┼────┐
                    ▼    ▼    ▼
    gust_comparison.py   hybrid_comparison.py
    (돌풍 × 6제어기)      (NMPC+INDI 모델미스매치)
```

---

## 1. vehicle_params.py — 기체 물리 파라미터

물리 상수를 한 딕셔너리에 집약. 형상팀 값이 확정되면 이 파일만 교체.

**내용물:**
- 질량·관성 (8kg, 축대칭 Iyy=Izz=0.70)
- 기체 형상 (길이 1m, 직경 0.15m, 기준면적 S_ref)
- 동체 공력계수 (C_Na, C_dc, C_A0, C_Aa2, x_cp, C_mq, C_lp)
- 로터 배치 (4개, × 패턴, arm=0.25m)
- 로터/프로펠러 (k_T, k_Q, D_prop, J_max)
- 모터 (tau_m=20ms, n_min/n_max)

**조정 이력:**
C_A0=0.50(둔탁한 원통) → 0.12(유선형 오자이브). 이유: 기존값으로는 60 m/s에서 26° 틸트 필요 → 동체 반양력 102N(무게 78N 초과) → 로터 포화. 유선형으로 조정 후 85 m/s(306 km/h)까지 트림 가능.

---

## 2. dynamics.py — 6-DOF 플랜트 모델 (핵심)

CasADi 심볼릭으로 작성된 프로젝트의 기반. 모든 제어기·비교 스크립트가 이 동역학을 공유.

**함수/클래스 구조:**
```
_quat_to_rotmat(q)          쿼터니언 [i,j,k,w] → 3×3 회전행렬
_quat_derivative(q, ω)      쿼터니언 미분 + Baumgarte 안정화
_body_aerodynamics(v, ω, p)  동체 공력 (수직력·축력·모멘트, V=0 소거)
_rotor_forces_moments(...)   4로터 추력/반토크/자이로 (전진비 J 의존)
_compute_xdot(x, u, p, w)   위 4블록 조립 → ẋ = f(x, u, w)
build_dynamics(params)       CasADi Function 생성 (acados 직접 전달)
compute_allocation_matrix()  [T1..T4] ↔ [T, Mx, My, Mz] 변환
AxialDronePlant              시뮬 래퍼 (RK4 적분, 쿼터니언 정규화)
  .step(x, u, w=None)       한 타임스텝
  .simulate(x0, ctrl, T, wind_fn=None)  궤적 시뮬
  .evaluate_xdot(x, u, w=None)  적분 없이 ẋ만 계산
  .hover_state(params)       호버 초기 상태
```

**왜 CasADi인가:**
numpy로 짜면 acados NMPC에 못 넣는다. CasADi는 심볼릭 자동미분이 가능해서 `build_dynamics()`가 반환하는 함수를 그대로 acados/IPOPT에 전달. 한 번 짜면 플랜트 시뮬 + NMPC 예측모델 두 가지로 재사용.

**바람 입력 (v3 추가):**
`_compute_xdot`에 w 인자 추가. v_body = R^T @ (vel - w). simulate()에 wind_fn 전달. 기존 코드는 w=None → 바람 없음 (역호환 유지, 6개 테스트 통과).

**좌표계:**
- 관성: z-up, 중력=[0,0,-mg]
- 동체: x전방, y우측, z하방 (우수: x×y=z)
- 호버 쿼터니언: q=[1,0,0,0] (180° about x)
- 로터 추력: body -z (위로 뜸)

---

## 3. test_plant.py — 플랜트 물리 검증

6개 테스트로 동역학의 물리적 정합성 확인:

| # | 테스트 | 검증 내용 |
|---|--------|-----------|
| 1 | 자유낙하 | 로터 0에서 v_z ≈ -9.81 m/s (1초) |
| 2 | 호버 | n_hover로 무게 지지, Δz < 0.1m (2초) |
| 3 | 복원 모멘트 | 양의 α → M_y < 0 (기수 하강), 음의 α → M_y > 0 |
| 4 | 감쇠 | C_mq < 0 → 초기 각속도 감쇠 |
| 5 | 전진비 | 상승 시 V_axial > 0 → 추력 감소 |
| 6 | 할당 행렬 | A·A⁻¹ = I, 균일 추력 → 순추력만 |

---

## 4. trim.py — 트림 조건 탐색

주어진 순항 속도 V에서 정상 수평비행(v̇=0, ω̇=0)을 만족하는 [θ_pitch, n_eq, Δn]을 찾는다.

**함수:**
```
find_trim(params, V)        scipy.fsolve로 3변수 3방정식 풀기
print_trim(trim, V, params)  결과 출력
trim_speed_sweep(params)     0→85 m/s 스윕, 물리적 최고 트림속도 특정
```

**3방정식:** [v̇_x=0 (수평 힘 균형), v̇_z=0 (수직 힘 균형), ω̇_y=0 (피치 모멘트 균형)]

**결과:** 0~85 m/s 전 구간 수렴. 파라미터 조정 전에는 60 m/s가 한계 (C_A0 과대 → 로터 포화).

---

## 5. controller.py — 제어기 5종

### CascadedPID
표준 캐스케이드 구조:
- 외부: 속도P + 고도PID → F_des (원하는 힘)
- 중간: F_des → R_des + T_cmd (자세+추력 분리, 틸트 제한)
- 내부: SO(3) 자세PD + 자이로 FF → 모멘트 → 할당 → 모터속도
- 30 m/s 게인 고정. 무릎점 65 m/s.

### LQRController (Error-State)
- 17D 풀스테이트 → 14D 오차상태 축소 (쿼터니언 4→3)
- CasADi 야코비안으로 A,B → 변환행렬 T(17×14) → 축소 ARE
- Q_L 부호 수정 완료 (이전 발산 원인이었음)
- u = u_trim - K_r @ δx_r

### ScheduledPID
CascadedPID 상속. 매 호출 시 v_x에 따라 게인 연속 스케일링:
- max_tilt: 35°→55° (고속 틸트 허용)
- Kp_att/Kd_att: 감소 (공력 감쇠 보상)
- Kp_vel: 감소 (고속 민감도 완화)

### ScheduledLQR
np.interp 선형 보간. 0~80 m/s (10 간격) 각 속도에서 트림+ARE → K_r 사전 계산.
런타임에 v_ref[0]으로 K_r(4×14), x_trim(17), u_trim(4) 보간. 쿼터니언 재정규화.

### INDIController
외측(PID와 동일) + 내측(INDI):
- 자세 오차 → ω̇_des (PD, gyro FF 불필요)
- ω̇_meas = LPF(Δω/Δt) (50Hz IIR)
- G(4×4) = ∂[T,ω̇]/∂n (현재 로터속도에서 실시간 계산)
- Δn = G⁻¹ @ ([T,ω̇]_des - [T,ω̇]_meas)
- n_cmd = n_actual + Δn

**돌풍 결과: INDI ≈ PID.** 외측(PID)이 병목. 내측만 개선해도 전체 성능 불변.

---

## 6. nmpc.py — NMPC (CasADi + IPOPT)

Direct Multiple Shooting NMPC. Q_z 파라미터로 외부 조정 가능.

**NLP 구조:**
- 결정변수: [U_0(4), X_1(17), U_1, X_2, ..., X_N] ≈ 420개
- 동역학 제약: X_{k+1} = RK4(X_k, U_k), 340개
- 입력 제약: n_min ≤ U_k ≤ n_max
- 비용: Q_v·속도 + Q_z·고도 + Q_w·각속도 + R·제어 + R_du·변화율
- 예측: N=20, dt=0.05s → 1초 앞

**Q_z 진단 결과:**
Q_z=20(기본): z_max=4.5m. Q_z=200: z_max=1.6m (vx 거의 불변).
원인: 가속 중 속도항(5·70²=24,500) >> 고도항(20·5²=500).

**IPOPT 풀이 통계:** get_solve_stats()로 수렴률 확인 (98-99%).

---

## 7. nmpc_acados.py — NMPC (acados + HPIPM)

실시간 NMPC. C 코드 생성으로 0.45ms/solve (IPOPT 대비 ~50배).

**특징:**
- SQP / SQP_RTI 선택 가능
- yref 런타임 파라미터화 (솔버 1회 빌드, 속도별 재빌드 불필요)
- Levenberg-Marquardt 정규화 (수렴 안정)
- 트림 warm start + 해 시프트

**수렴률:** ~30% (부분 수렴이지만 성능 유효).

---

## 8. sweep.py — 성능 저하 곡선

30 m/s 게인 고정 PID/LQR을 30→85 m/s로 밀며 RMSE, 포화율, 발산 기록.

**핵심 결과:** PID 무릎점 65 m/s (RMSE 3배 증가). "단일 동작점 튜닝의 한계" 정량 입증.

**주의:** 이 스크립트의 LQR은 error-state 리팩터 이전 버전 기준. 현재 controller.py와 인터페이스 불일치 있을 수 있음.

---

## 9. sweep_scheduled.py — 게인 스케줄링 종합 비교

5개 제어기(PID고정/스케줄, LQR고정/스케줄, NMPC) × 3 시나리오.

**시나리오:**
- [A] 순항 속도 스윕 (30~85 m/s, 교란 회복)
- [B] 호버→70 m/s 가속 천이 (20초)
- [B'] NMPC Q_z 진단 (Q_z=20,50,100,200 비교)
- [C] 발사천이 (70 m/s, 30° AoA, 로터 0)

**핵심 발견:**
- 게인 스케줄링은 고정 대비 확실히 개선 (LQR vx 36%↑)
- 그래도 순항 NMPC에는 못 미침
- 호버→70: NMPC vx 최고, LQR스케줄 z 최고 (trade-off)

---

## 10. gust_comparison.py — 돌풍 외란 비교

6개 제어기 × 2종 돌풍 (측풍 y + 수직돌풍 z, 10 m/s, 1-cosine).

**돌풍 모델:** FAR 25 1-cosine 펄스. w(t) = (W_max/2)(1 - cos(2π(t-t0)/T)).

**측정:** 최대 편차, 회복 시간, RMSE, 로터 포화.

**핵심 반전:**
- LQR(풀스테이트) > NMPC(예측) > PID ≫ INDI(PID급)
- "예측 불가 외란에서는 예측 장점 무력화, 즉각 피드백이 우세"
- INDI가 PID급인 이유: 외측 PID가 병목. 아키텍처 > 알고리즘.

---

## 11. hybrid_comparison.py — NMPC+INDI 하이브리드 + 모델 미스매치

**NMPCINDIHybrid 클래스:**
- NMPC → u_nmpc (nominal model)
- ω̇_pred = f_nom(x, u_nmpc) vs ω̇_meas = LPF(Δω/Δt)
- Δn = G⁻¹ @ [0, ω̇_pred - ω̇_meas] (추력 보존, 모멘트만 보정)
- n_cmd = u_nmpc + Δn

**모델 미스매치:** 플랜트 C_Na를 ±20% 변경, NMPC는 nominal 사용.

**결과 — 나이브 하이브리드 실패:**
- 정확+돌풍: NMPC 0.092 → NMPC+INDI 0.319 (3.5배 악화)
- C_Na+20%+돌풍: NMPC 0.080 → NMPC+INDI 0.263 (3.3배 악화)

**원인:** 이중 보정. NMPC(50Hz)와 INDI(1kHz)가 같은 오차를 각자 보정 → 과보정 → 진동.
NMPC는 INDI가 뭘 했는지 모르고, INDI는 NMPC가 뭘 할지 모름.

**올바른 해법:** NMPC가 모터속도 대신 가상명령[T_des, ω̇_des] 출력 → INDI가 유일한 실행자. 인터페이스 분리 필수. → 향후 과제.

---

## 12. launch_transition.py — 발사천이 시나리오 (이전 버전)

튜브 발사 직후 → 순항 안정화. 40 m/s, 30° AoA, 로터 0.
PID/LQR/NMPC(acados) 비교.

**참고:** sweep_scheduled.py의 [C]에서 70 m/s로 재실행됨 (이 파일은 이전 버전).

---

## 13. takeoff_cruise.py — 호버→순항 천이 (이전 버전)

정지 → 이륙 → 70 m/s 가속. 시간변화 기준 궤적 추종.
TrackingPID, TrackingLQR, TrackingNMPC 래퍼 포함.

**참고:** sweep_scheduled.py의 [B]에서 통합 재실행됨.

---

## 14. rotor.py — 이전 버전 동역학 (참고용)

추력 방향이 body +x (전방)인 이전 설계. 로터 배치도 y-z 평면 + 패턴.
현재 사용되지 않음. 설계 변경 이력 참고용.

---

## 15. rotorpy_multirotor.py — RotorPy 원본 (참고용)

RotorPy 라이브러리의 Multirotor + BatchedMultirotor(PyTorch) 클래스.
우리 dynamics.py의 구조적 참고가 됨. 프로젝트에서 직접 사용되지 않음.

---

## 실행 방법

```bash
# 플랜트 검증
python3 test_plant.py

# 트림 스윕
python3 trim.py

# 종합 비교 (5제어기 × 3시나리오, ~3분)
python3 sweep_scheduled.py

# 돌풍 비교 (6제어기 × 2돌풍, ~2분)
python3 gust_comparison.py

# 하이브리드 모델미스매치 (3조건 × 2시나리오 × 3제어기, ~5분)
python3 hybrid_comparison.py
```

## 핵심 진입점 (개발자용)

```python
from dynamics import AxialDronePlant, build_dynamics
from vehicle_params import vehicle_params as P
from trim import find_trim
from controller import (CascadedPID, LQRController,
                        ScheduledPID, ScheduledLQR, INDIController)
from nmpc import NMPCController

# 플랜트
plant = AxialDronePlant(P, dt=0.001)

# 트림
trim = find_trim(P, 70.0)  # 70 m/s

# 제어기
pid  = CascadedPID(P, v_ref=[70,0,0], z_ref=50, dt=0.001)
lqr  = LQRController(P, trim['state'], trim['control'])
slqr = ScheduledLQR(P, v_ref=[70,0,0], z_ref=50)
nmpc = NMPCController(P, v_ref=[70,0,0], z_ref=50, u_ref=trim['control'])
indi = INDIController(P, v_ref=[70,0,0], z_ref=50, dt=0.001)

# 시뮬레이션 (바람 없이)
ts, xs, us = plant.simulate(x0, pid, T=5.0)

# 시뮬레이션 (돌풍 포함)
def gust(t):
    w = np.zeros(3)
    if 2.0 <= t <= 3.0:
        w[2] = 5.0 * (1 - np.cos(2*np.pi*(t-2.0)))
    return w
ts, xs, us = plant.simulate(x0, pid, T=8.0, wind_fn=gust)
```
