> 원본 README의 보존 사본이다. 일부 과거 결과/문서는 이식 묶음에 포함하지 않았다.
> 실행은 프로젝트 루트 docs/VALIDATION.md 및 이 폴더 PROVENANCE.md를 따른다.

# fastdrone_kh — 고정 로켓형 기체 모델과 Python 사전 비교

팀에서 함께 사용할 **`light_rocket_v2_pack_forward` 한 기체**를 정리한 저장소다. 예전 8 kg 모델, 다른 배터리 위치 후보, 과거 기체의 결과는 현재 파일 목록에서 제외했다. 원본 팀 저장소는 [protkjj/fast-drone](https://github.com/protkjj/fast-drone)이며, 여기는 새 기체와 연결 코드를 공유하는 별도 저장소다.

기체는 고정했고, 제어기 비교와 통합 미션은 아직 보완 중이다. **트림이 된다는 것과 모든 제어기가 임무를 성공한다는 것은 다르다.** 실패한 시험도 결과에 포함했다. 실제 RBD1을 재현한 모델이나 실기 비행 검증 결과는 아니다.

다른 프로젝트에 옮기려면 [이식 가이드](docs/INTEGRATION.md)를 먼저 읽는다. GPT/Codex에 작업을 맡길 때 사용할 요청문도 이 문서 끝에 있다.

## 1. 무엇을 바꿨나

| 항목 | 이전 연구에서 사용한 방식 | 이번 공유본 |
|---|---|---|
| 배치 | KASA 당시 일반 쿼드 배치 + 축대칭 동체 | 동체 길이축 **body +x** 방향 고정 추력, yz 평면의 로터 4개, 기수 상향 호버 |
| 질량·관성 | 8 kg 기체의 파라미터 | 부품 질량 요소와 위치에서 **1.174 kg, CG, 관성**을 함께 계산 |
| 공력 | 기존 동체의 기준 면적·계수 | 새 동체 치수에 맞춘 동체 구간별 횡류력, 기수 수직력, 동체·지지대·모터 항력 |
| 추진식 | 초기 경량 모델에서는 추력·반토크에 같은 선형 감소율 | 공개 자료의 **Ct(J), Cp(J)를 각각 보간**해 추력과 반토크 계산 |
| 제어할당 | 일정한 반토크/추력 비율의 상수 행렬 | 실제 추진식에 맞춘 비선형 할당·추력 역변환·제어효과도 |
| 제어기 연결 | 기존 기체의 트림·게인·입력 제약 | 새 트림/LQR 재계산, INDI 명목 트림 보상, 분리형의 비선형 가상입력 제약 |

중간에 만들었던 8 kg 로켓형 및 다른 후보의 코드·결과를 함께 배포하는 것은 아니다. 이번 모델만 옮기려면 **질량 숫자나 `vehicle_params.py` 하나만 복사해서는 안 된다.** 추력축, 상태 규약, 공력, 추진식, 할당, 예측 모델까지 같이 맞춰야 한다.

## 2. 고정 기체

| 항목 | 값 |
|---|---|
| 모델 ID | `light_rocket_v2_pack_forward` |
| 질량 | **1.17407572 kg** |
| 동체 길이 / 직경 | **0.32 m / 0.075 m** |
| 배터리 중심 | 동체 중앙에서 기수 쪽 **0.020 m** |
| 무게중심 | 동체 중앙에서 기수 쪽 약 **0.001343 m** |
| 주관성 Ixx / Iyy / Izz | 약 **0.003560 / 0.005519 / 0.005533 kg·m²** |
| 로터 중심 반경 | 동체 중심축으로부터 **0.115 m** |
| 프로펠러 직경 / 피치 | **0.1397 m / 0.1651 m** (5.5×6.5 in 규모) |
| 물리적 회전수 제한 | **0~33,000 rpm** = 0~3455.752 rad/s |
| 모터 응답 | 1차 지연, 시정수 **20 ms** |
| 상태 / 입력 | **17상태 / 모터 회전속도 명령 4개** |

전체 수치는 [aircraft_snapshot.json](data/aircraft_snapshot.json)에 있다. 생성 코드는 [airframe.py](control/airframe.py), 최종 추진 곡선 연결은 [light_rocket_curve.py](control/light_rocket_curve.py)다. 배터리는 질량 요소이며 SOC·전압·전류·열 모델은 없다.

```python
from control.baseline_v2 import baseline_params
p = baseline_params()  # 매번 독립적인 dict; 파라미터 해시 확인
```

기존 호출 이름도 사용할 수 있다.

```python
from control.light_rocket_curve import make_curve_rocket
p = make_curve_rocket('pack_forward')  # 인자를 생략해도 같은 기체
```

다른 후보 이름은 오류로 처리한다. `from control.vehicle_params import vehicle_params` 역시 이제 이 기체를 반환한다. 모델이 조용히 바뀌는 것을 막기 위한 기준 해시는 다음과 같다.

```text
13a1dcc6332987eba93ea4653f4d6087741d1ac085701982ed94c22e6658c807
```

## 3. 추진 곡선과 공력의 근거·한계

[APC 5.5×6.5E 공개 성능 자료](https://www.apcprop.com/files/PER3_55x65E.dat)의 30,000 rpm 곡선에서 발췌한 10개 점으로 Ct와 Cp를 각각 PCHIP 보간한다. 이는 제조사의 **계산 자료**이지 이 기체의 실측 자료가 아니다. 출처·발췌점·보간에 쓰지 않은 확인점은 [apc_curve.json](data/apc_curve.json)에 남겼다.

```text
n_rps = omega_motor / (2*pi)
J = V_axial / (n_rps*D)
T = rho*n_rps^2*D^4*Ct(J)
Q = rho*n_rps^2*D^5*Cp(J)/(2*pi)
```

Cp는 동력계수다. 반토크 환산의 `2*pi`를 빠뜨리면 안 된다. 이제 **Q/T가 전진비에 따라 달라지므로 일정한 Q/T의 할당행렬을 그대로 쓸 수 없다.** `k_T`, `k_Q`는 호버와 초기값용 정적 기준으로만 남아 있다.

- 30,000 rpm의 계수 형상을 다른 회전수에도 재사용한다. RPM·Reynolds 수·Mach 수 의존성을 정밀하게 반영하지 않았다.
- 가정한 작업 범위는 10,000~33,000 rpm, 순방향 유입과 양의 추력 전진비 범위다. **10,000 rpm은 하드웨어 하한이 아니다.** 물리적 입력 하한은 0이다.
- 역류·풍차 상태·비스듬한 유입은 미검증이다. 범위 밖에서는 계수 경계 제한과 음의 추력 0 처리가 사용되므로, 수치적으로 계산됐다는 이유만으로 유효한 물리 결과로 보지 않는다.
- 공력은 저차 근사이며 CFD·풍동으로 보정하지 않았다. 85 m/s 트림에서는 수직 지지력의 약 **86%가 이 공력 모델**에서 나온다. 공력 가정의 영향이 크다.
- 실제 RBD1의 배치·질량 규모를 참고했지만, 장착 추진계·형상을 그대로 복원하지 않았다. 별도 날개·조종면·추력 편향 장치를 추가하지 않았다.

## 4. 설치와 빠른 확인

Python 3.12를 권장한다. 저장소 루트에서 실행한다. `control/`은 이 저장소의 패키지 이름이며, 별도 `python-control` 패키지를 설치할 필요가 없다.

```powershell
git clone https://github.com/KYUHYUNKANG03/fastdrone_kh.git
cd fastdrone_kh
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.\.venv\Scripts\python.exe -m control.validate_model --authority
.\.venv\Scripts\python.exe -m pytest tests -q
.\.venv\Scripts\python.exe -m control.run_baseline_comparison --stage lqr
```

Linux/macOS에서는 `.venv/bin/python`을 사용한다. 실제 확인에 사용한 버전은 [TESTED_ENVIRONMENT.json](docs/TESTED_ENVIRONMENT.json)에 기록했다. ROS2·PX4·acados·MATLAB·Simulink는 필요하지 않다.

새 기체의 트림 확인값:

| 속도 | 기수 상승각 | 최대 로터 회전수 | 회전수 상한까지의 여유 |
|---|---:|---:|---:|
| 호버 | 90.000° | 12,434 rpm | 62.32% |
| 70 m/s | 10.385° | 24,966 rpm | 24.35% |
| 80 m/s | 8.284° | 28,343 rpm | 14.11% |
| 85 m/s | 7.460° | 30,034 rpm | 8.99% |

85 m/s는 확인한 해석 범위의 상한이며 실제 최고속도 실증값이 아니다. 트림 존재, 정적 조종 여유, 폐루프 응답은 별도로 확인한다.

## 5. 비교 실험 실행

```powershell
# 70 m/s 명목 조건, 제어기 6종
.\.venv\Scripts\python.exe -m control.run_baseline_comparison --stage pilot

# 70 m/s 명목 + 공력 오차 ±15% + 수직/측방 2 m/s 돌풍
.\.venv\Scripts\python.exe -m control.run_baseline_comparison --stage short

# 위 시험 + 80/85 m/s 대표 조건 + 기존 70 m/s 통합 미션
.\.venv\Scripts\python.exe -m control.run_baseline_comparison --stage all
```

`--controllers GS-LQR NMPC Split`로 일부만 선택할 수 있다. 전체 실행은 PC에 따라 수십 분 걸릴 수 있다. 실행할 때마다 `results/baseline_v2/run_시각/`을 새로 만들며, 배포한 결과를 덮어쓰지 않는다. 새 실행 폴더는 기본적으로 Git에서 제외된다.

비교 코드는 [run_baseline_comparison.py](control/run_baseline_comparison.py)가 진입점이다. `mission_sim.py`는 미션 기준값만, `hybrid_comparison.py`는 결합 클래스만 포함한다. 이 두 파일을 직접 실행하는 옛 방식은 사용하지 않는다. 분리형 최적화는 **`ComparisonNMPC(..., virtual=True)`**를 사용하며, 상수 Q/T를 가정한 예전 `VirtualNMPC`와 acados 구현은 배포하지 않는다.

## 6. 함께 올린 결과

**고정한 새 기체로 실행한 61개 시험만** 포함했다. [전체 비교표](results/baseline_v2/run_20260924T161648152853Z/SUMMARY.md), [결과 해설](docs/BASELINE_V2_RESULTS.md), [구간별 지표와 추진 범위 점검](results/baseline_v2/run_20260924T161648152853Z/DIAGNOSTICS.md)을 확인한다.

- 70 m/s 명목 초기 오차에서는 여섯 제어기가 모두 복귀했다. GS-LQR 고도 RMSE는 0.059 m, 포화율은 0%였다.
- 공력 오차·돌풍 조건에서는 지표에 따라 순위가 달랐다. 분리형이 항상 가장 좋은 것은 아니었다.
- 80·85 m/s에서 현재 INDI는 진동·속도 오차가 남았고, 분리형은 중단 또는 큰 추종 오차가 발생했다.
- 통합 미션은 GS-LQR·INDI가 계산을 마쳤지만 포화 기준을 넘었고 추진 가정 범위 밖 구간도 있었다. 직접 NMPC·나이브·분리형은 감속 중 최적화 연속 실패로 중단했다. **모든 판정 조건을 충족한 통합 미션 결과는 아직 없다.**

중단 결과의 RMSE는 중단 전 구간 값이다. 완주 결과와 그대로 순위를 비교하면 안 된다. 직접·가상 NMPC는 상태 비용과 예측 구간을 맞췄지만 입력 비용·상태 차원·모터 지연 포함 여부가 달라, 인터페이스만의 효과를 분리한 최종 실험은 아니다.

![80 m/s 명목 응답 — 분리형 중단 포함](results/baseline_v2/run_20260924T161648152853Z/80_nominal_short.png)

결과의 NPZ에는 상태·모터 명령·기준값·돌풍, JSON에는 지표·솔버 상태가 있다. 원래 실행의 메타데이터는 보존했으며, 공유용으로 코드를 정리한 내역과 해시 관계는 [PROVENANCE.md](docs/PROVENANCE.md)를 참고한다.

## 7. 팀원이 읽을 파일

| 목적 | 파일 |
|---|---|
| 고정 모델 가져오기 | `control/baseline_v2.py`, `control/light_rocket_curve.py` |
| 부품 질량·CG·관성·형상 | `control/airframe.py`, `data/aircraft_snapshot.json` |
| 상태방정식·모터 지연·바람 | `control/dynamics.py` |
| 공력 / 추진 곡선 | `control/light_aero.py`, `control/propeller_curve.py`, `data/apc_*.json` |
| 추력축·추력 역변환·비선형 할당·INDI 효과도 | `control/geometry.py` |
| 트림 / 정적 조종 여유 | `control/trim.py`, `control/curve_authority.py` |
| LQR·GS-LQR·INDI | `control/controller.py` |
| 직접·가상입력 NMPC | `control/comparison_nmpc.py` |
| NMPC–INDI 내부 루프 연결 | `control/hybrid_comparison.py` |
| 공통 시험 조건·제어기 생성 | `control/run_baseline_comparison.py` |
| 이식 검증용 기준값 | `data/acceptance_fixtures.json`, `tests/` |

모델 이식이나 시뮬레이션 보완은 [INTEGRATION.md](docs/INTEGRATION.md)의 체크리스트를 기준으로 진행한다. 결과를 좋게 만들기 위해 기체를 다시 고르거나, 미수렴·범위 이탈 결과를 삭제하지 않는다.
