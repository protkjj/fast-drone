# 다른 프로젝트에 새 기체를 연결하는 방법

## 1. 옮길 대상부터 확인

이 저장소의 최종 기체는 `light_rocket_v2_pack_forward` 하나다. 원본 저장소의 파일을 이 저장소 위에 덮어쓰거나, 질량·관성 숫자만 복사하면 안 된다. 기존 프로젝트의 사용자 수정은 보존하고 별도 브랜치에서 진행한다.

먼저 이 저장소 자체에서 아래 세 가지를 확인한다.

```text
python -m control.validate_model --authority
python -m pytest tests -q
python -m control.run_baseline_comparison --stage lqr
```

기체 ID와 파라미터 해시가 README와 같은지 확인한다. 파라미터 사전 전체는 `data/aircraft_snapshot.json`, 호버·70·80·85 m/s의 상태/입력 기준값은 `data/acceptance_fixtures.json`에 있다. JSON에서 읽은 리스트를 그대로 쓰는 대신 이 저장소의 팩토리를 쓰면 NumPy 배열 등의 형식도 맞춰진다.

## 2. 좌표계·입출력 계약

| 값 | 규약 |
|---|---|
| 세계 좌표 | z-up, 중력 `[0,0,-g]` |
| 동체 축 | +x가 기수·추력 방향, y/z는 동체 고정축 |
| 로터 배치 | 동체 yz 평면에 네 로터, 길이축에 평행한 고정 추력 |
| 자세 | `q=[qx,qy,qz,qw]`, body-to-world 회전 |
| 위치 / 속도 | 세계 좌표, m / m/s |
| 각속도 | 동체 좌표, rad/s |
| 모터 상태 / 명령 | 모두 **rad/s**, RPM·PWM·추력이 아님 |
| 바람 | 세계 좌표 m/s, 몸체 유입속도는 `R.T @ (v_world-w_world)` |
| 로터 방향 부호 | 동체에 작용하는 반작용 토크의 부호 `[+1,-1,+1,-1]` |

```text
x[ 0: 3] = world position
x[ 3: 6] = world velocity
x[ 6:10] = quaternion xyzw, body -> world
x[10:13] = body angular velocity
x[13:17] = actual rotor rates
u[ 0: 4] = commanded rotor rates
```

호버 자세는 단위 쿼터니언이 아니다. `geometry.hover_quaternion()` 또는 `AxialDronePlant.hover_state(p)`를 사용한다. 기수가 세계 +z를 향하며, 호버에서 가속도계 비력의 주축은 body +x다. NED/FRD 또는 wxyz를 쓰는 외부 코드에는 명시적인 변환이 필요하다. 이 저장소 자체의 규약을 조용히 바꾸지 않는다.

```python
from control.baseline_v2 import baseline_params
from control.dynamics import AxialDronePlant
from control.trim import find_trim

p = baseline_params()
plant = AxialDronePlant(p, dt=0.002)
trim = find_trim(p, 70.0)  # 유효한 평형이 없으면 예외
x = trim['state'].copy()
x[2] = 20.0
u = trim['control'].copy()
x_next = plant.step(x, u)  # 호환성 확인용 한 스텝; 제어 안정성 시험은 아님
```

## 3. 함께 옮겨야 하는 코드

최소 기체 계산 묶음:

```text
baseline_v2 -> light_rocket_curve -> airframe + data/apc_reference.json
                                -> propeller_curve + data/apc_curve.json
dynamics -> geometry + light_aero + propeller_curve
trim -> dynamics + geometry
curve_authority -> geometry + propeller_curve
serialization -> jsonable / 기존 LQR 설계 가중치
```

`airframe.py`의 질량 요소에서 전체 질량·CG·관성을 만들고, 공력 작용점과 로터 위치도 CG 기준으로 변환한다. 이 파일의 내부 생성값은 최종 추진 계수를 붙이기 전 단계이므로, 외부에서 `build_fixed_airframe()`만 호출해서 제어기에 넘기지 않는다. 최종 입력은 반드시 `baseline_params()` 또는 `make_curve_rocket('pack_forward')`로 얻는다.

동역학은 병진·회전 운동, 쿼터니언 미분, 실제 회전속도 상태와 `n_dot=(n_cmd-n)/tau_m`을 포함한다. 동체 자이로 항 및 로터 각운동량 효과를 포함하지만 로터 가속도에 따른 추가 반작용 `-h_dot`는 포함하지 않는다.

## 4. 추진식과 할당을 기존 코드에 연결할 때

1. NumPy 경로는 `values_and_derivatives`, CasADi 경로는 `symbolic_force_torque`를 사용한다. 두 경로는 같은 Ct/Cp 자료와 보간식을 쓴다.
2. 모터 회전속도는 rad/s이며 추진 계수 식에서는 초당 회전수로 바꾼다. Cp에서 토크를 만들 때 `2*pi`로 나눈다.
3. 속도에 따라 Q/T가 달라진다. 기존 `A^-1 @ [T,M]`를 정확한 할당으로 재사용하지 않는다. `allocate_wrench(p, desired, v_body)`가 현재 추진식에 맞춘 유계 비선형 할당이다.
4. `rotor_speeds_for_thrust`는 동일한 추력식의 역변환이다. 고속 경로에 정적 `sqrt(T/k_T)`만 남겨두지 않는다.
5. INDI의 `control_effectiveness`는 Ct/Cp 식의 실제 회전속도 미분을 사용한다. 로터 자이로에 대한 효과도 미분은 생략한 근사이며, 0추력 부근에는 랭크 저하가 가능하다. 포화·특이점 처리를 함께 확인한다.
6. `compute_allocation_matrix(..., static_reference=True)`와 플랜트의 `TM_to_f`는 정적 초기값용이다. 실제 고속 할당 행렬로 해석하지 않는다. 기본 호출은 가변 Q/T에서 오류를 발생시킨다.
7. 하드웨어 입력 제한과 추진 모델의 가정 범위를 구분한다. `domain_status`로 실제 공기 상대속도와 실제 회전속도의 범위 이탈을 기록한다.

## 5. 제어기 연결

검증된 연결 예시는 `run_baseline_comparison.Factory`다. 단순히 클래스 이름만 가져오기보다 이 생성·초기화·기준값 갱신을 함께 확인한다.

- **LQR:** 새 평형에서 상태방정식을 선형화한다. 오차상태는 14차원 `[dz,dv(3),dphi(3),domega(3),dn(4)]`이다. 예전 8 kg 게인을 재사용하지 않는다.
- **GS-LQR:** 0/10/20/30/40/50/60/70/75/80/85 m/s에서 같은 설계 원칙으로 게인을 구하고 목표 속도로 보간한다. 85 m/s 시험에 80 m/s까지만 있는 표를 쓰지 않는다.
- **INDI:** 고속에서 중력만의 호버 외부 루프를 쓰지 않도록 명목 트림의 세계 좌표 추진력 `trim_force_ref`를 연결한다. 실제 공력 오차나 돌풍의 참값을 주입하지 않는다. 첫 표본에는 실제 모터 상태를 유지하고 차분 이력을 시작한다. 비교에서는 기울기 제한 100도, 내부 갱신 2 ms, 각가속도 필터 50 Hz를 사용했다.
- **직접 NMPC:** `ComparisonNMPC(p, trim, virtual=False)`. 17상태 예측에 실제 모터 지연을 포함하고, 출력은 모터 명령이다.
- **분리형:** `ComparisonNMPC(p, trim, virtual=True)`를 `ProperHybrid`로 감싼다. 13상태 예측 출력은 `[T,omega_dot_x,omega_dot_y,omega_dot_z]`이며 실제 모터 명령은 INDI가 만든다.
- **나이브:** 직접 NMPC에 `NaiveHybrid`를 연결한다. 기존 비교 구조를 보존한 것이며 권장 최종 구조라는 뜻은 아니다.

```python
from control.run_baseline_comparison import Factory

factory = Factory(p)
controller = factory.make('Split', 70.0, 20.0)
u = controller(0.0, x)
```

분리형의 예측 단계에는 보조 회전수 4개가 추가된다. 실제 Ct/Cp 식으로 다음 조건을 만족하는 회전수가 존재하는지 확인한다.

```text
T_command = sum(T_i)
I * omega_dot_command = M_aero + M_rotor - omega x (I*omega)
0 <= auxiliary_rotor_rate <= n_max
양의 추력 전진비 영역
```

보조 회전수를 실제 모터 명령으로 바로 보내면 분리형 실험이 아니게 된다. 또한 이 제약은 단계별 정적 실현 가능성이지 모터 지연을 포함한 도달 가능성 보장이 아니다. 이전 상수 할당 기반 `VirtualNMPC`를 가져와 차단 조건만 지우면 안 된다. acados로 옮길 경우에도 이 제약을 별도로 구현·검증해야 한다.

## 6. 이식 완료 판정

- [ ] 모델 ID·해시·전체 파라미터가 snapshot과 일치한다.
- [ ] 호버 +x 추력, 상태 순서, quaternion, 바람·토크 부호, 단위가 일치한다.
- [ ] `acceptance_fixtures.json`의 0/70/80/85 m/s 트림과 전체 상태미분을 재현한다.
- [ ] NumPy/CasADi 추진식, 역변환, 효과도 미분, 비선형 할당 검증이 통과한다.
- [ ] 새 모델로 LQR을 재계산하고 70 m/s 작은 오차 응답을 확인한다.
- [ ] 직접·가상 NMPC의 제약과 입력 제한이 새 추진식과 일치한다.
- [ ] 각 시험은 같은 명목 기체·센서 조건·구동기 제한을 사용하고, 외란 시험에서만 시험 플랜트 사본을 바꾼다.
- [ ] 중단/미수렴/포화/추진 범위 이탈을 기록하고 기존 결과는 보존한다.

목표는 새 코드에서 과거의 제어기 순위를 억지로 재현하는 것이 아니다. 기체가 동일하고 연결이 맞는지 확인한 뒤, 제어기 문제와 모델 적용 범위 문제를 따로 개선한다.

## GPT/Codex에 전달할 요청문

> 이 저장소의 README.md, AGENTS.md, docs/INTEGRATION.md를 읽고 `light_rocket_v2_pack_forward`를 우리 시뮬레이션에 연결해줘. 대상은 고정한 기체 하나이며 형상·질량·배터리 위치·공력·추진 곡선을 다시 탐색하지 마. 기존 프로젝트의 사용자 변경과 결과는 보존하고, 먼저 파일별 변경 계획과 좌표계·단위 차이를 확인해줘. `vehicle_params.py`만 복사하지 말고, +x 추력축/17상태 규약/CG 기반 관성·공력/독립 Ct·Cp 보간/비선형 할당/INDI 효과도/NMPC 예측 및 가상입력 제약을 함께 맞춰줘. `aircraft_snapshot.json`, `acceptance_fixtures.json`, 테스트와 70 m/s LQR 응답으로 이식의 일관성을 검증해줘. 분리형은 보조 회전수 제약을 실제 모터 출력으로 대체하거나 옛 차단 조건만 삭제하지 마. 새 결과는 별도 폴더에 저장하고, 실패와 추진 모델 범위 이탈도 보고해줘. 이번에는 배터리 전기·열·실기 연동이나 새 제어 구조를 임의로 추가하지 마.
