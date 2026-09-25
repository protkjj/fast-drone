# 기본 성능과 강건성 검증

저장소 루트에서 실행한다. Windows에서는 아래 `python` 대신
`.\.venv\Scripts\python.exe`를 사용할 수 있다.

```powershell
python -m pip install -r requirements-dev.txt
python -m control.mission_sim --help
```

## 기본 기체와 제어기

새 실행은 팀원 저장소의 **light_rocket_v2_pack_forward, 1.17407572 kg**를 쓴다.
원본: <https://github.com/KYUHYUNKANG03/fastdrone_kh>, `repro/kyuhyun` 브랜치,
커밋 `34981c5d64c102df5c2fd8b4b3d9a5e0e3db81f7`.
[이식 내역](../models/team_light/PROVENANCE.md)에 파라미터 해시와 파일 출처를 기록했다.

`models/team_light/control/`에는 원본의 기체 생성, 분포 공력, 독립 Ct/Cp 곡선,
비선형 할당, INDI 효과도, 직접/가상 NMPC와 테스트를 함께 이식했다.
원래 `control/`의 8 kg/rocket/selected 모델이나 상수 Q/T 최적화기를 새 기체에
끼워 넣지 않는다. 세계 z-up, body +x 추력, xyzw body-to-world quaternion,
17상태, 모터 rad/s 규약을 유지한다.

기본 비교군은 `GS-LQR Split`이다. 선택 가능한 이름은
`LQR70 GS-LQR INDI NMPC Naive Split`이다. `Split`은 팀원 제공
`ComparisonNMPC(virtual=True)` + `ProperHybrid`이며, 보조 로터 변수는 정적
명령 가능성 검사에만 쓰고 실제 모터 명령은 INDI가 생성한다.
예측 구간 N=15, 예측/상위 제어 주기 0.04초, max_iter=80, 플랜트/내부 주기
0.002초다. 현재 참조를 갱신하며 미래 궤적 미리보기는 쓰지 않는다.
직접/가상 NMPC의 입력 비용과 모터 지연 포함 여부는 다르므로 인터페이스만의
효과를 분리한 제거실험으로 해석하지 않는다. 폴백이나 acados를 임의로 추가하지 않았다.

## 1. 기본 성능: 40초, 무외란

```sh
python -m control.mission_sim --scenario baseline
```

| 구간 | 시간 | 참조 |
|---|---|---|
| 초기 호버 | 0–1 s | 속도 0, 고도 50 m |
| 가속 | 1–9 s | 0 → 70 m/s |
| 순항 | 9–12 s | 70 m/s |
| 감속 | 12–37 s | 70 → 0 m/s |
| 최종 호버 | 37–40 s | 속도 0 |

가감속은 기존 half-cosine 보간, 고도는 전 구간 일정하다. 이륙, 바람, EKF,
몬테카를로가 자동으로 섞이지 않는다. 초기 상태는 해당 플랜트의 호버 트림이다. 70 m/s 명목 모델에서 수행한
[시간 최적화 실험](TIMING_OPTIMIZATION.md)을 기본값에 반영했다. 다른 속도·돌풍 강도·
불확실성 범위에서 같은 정착시간이 보장되는 것은 아니다.

`--speed 80 --altitude 50` 또는 `--durations 3 20 15 20 7`로 바꿀 수 있다.
명목 트림 확인 범위인 0–85 m/s까지만 허용한다. 구간 길이는 0.002초의 배수다.
과도하게 짧은 구간은 실행 확인용이며 연구 성능 결과로 사용하지 않는다.

## 2. 독립 돌풍: 4.5초

```sh
python -m control.mission_sim --scenario gust
# 같은 진입점
python -m control.gust_comparison
# 강한 수직돌풍, 회복 관찰 시간 연장
python -m control.mission_sim --scenario gust --gust-directions vertical --gust-peak 10 --gust-recovery 10
```

70 m/s 크루즈 트림(자세·모터 실제 회전수 포함)에서 시작한다.
0–1초 정상 순항, 1–2초 1-cosine 돌풍, 2–4.5초 회복만 수행한다.
기본 세 조건은 무돌풍 대조군, 수직 2 m/s, 측방 2 m/s다. 기본 강도는 새 모델의
팀원 비교 조건을 따르며 `--gust-peak`로 부호·크기를 바꿀 수 있다.
`--gust-settle`, `--gust-duration`, `--gust-recovery`도 조정 가능하다.

제어기 참조·트림 추력·솔버 초기값과 INDI 이력을 매 시행 초기화한다.
돌풍 전 마지막 0.5초가 안정 기준 안에 있지 않으면 `pre_gust_not_settled`로
실패 처리한다. 초기화 과도응답을 돌풍 영향으로 간주하지 않는다.

회복 시간은 **돌풍 종료 시점부터** 측정한다. 전체 3축 속도 오차 ≤0.5 m/s,
고도 오차 ≤0.5 m, 각속도 ≤1 rad/s를 만족한 뒤 관찰 종료까지 유지하고, 그 시간이
적어도 0.5초여야 회복으로 기록한다. 미회복은 `null`과 실패 사유로 남긴다.
관찰 시간이 부족하면 `--gust-recovery`를 늘려 별도 시험한다.

## 3. 불확실성 스윕: 기본 미션 반복

```sh
python -m control.mission_sim --scenario sweep --ranges configs/uncertainty.example.json --points 3
```

제어기의 모델·게인·트림 테이블은 명목값으로 고정하고 **플랜트 사본만** 바꾼다.
초기 상태는 시험 플랜트의 평형 호버다. 따라서 초기 추력 불균형을 주는 시험이
아니라 모델 불일치 하의 기동 시험이다. 센서 오차는 포함하지 않는다.

| 인자 | 실제 변경 대상 |
|---|---|
| mass | 병진 운동의 질량 |
| Ixx / Iyy / Izz | 회전 운동의 각 주관성 |
| drag | 동체 C_pressure, C_f 및 부품 항력 cda |
| normal_force | 기수 C_Na 및 분포 횡류 C_dc |
| thrust | Ct(J) 곡선 전체와 정적 k_T |
| torque | Cp(J) 곡선 전체와 정적 k_Q |
| motor_tau | 모터 응답 시정수 |
| aero (선택) | 분포 공력의 전체 힘·모멘트 배율 |

기본값은 앞의 9인자 ×0.8–1.2다. **실측 공차가 아니라 탐색용 가정**이다.
명목 기체 형상/CG/부품 구성을 다시 설계하는 것이 아닌, 모델 계수 오차 시험이다.
Cp/Ct를 바꿀 때 k_Q/k_T만 바꾸는 실수를 막는 동역학 회귀 테스트가 있다.

기본 `--sweep-mode oat`는 한 번에 한 인자씩 바꾸고 명목 시행은 한 번만 포함한다.
기본 3점이면 총 19조건이다. 결합 효과는 `--sweep-mode grid`로 선택한 인자의
직접곱을 시험한다. 예를 들어 범위 JSON에 mass와 motor_tau만 넣으면 3×3=9조건이다.
100,000조건을 넘는 grid는 오류로 막으므로 인자를 줄이거나 MC를 사용한다.

## 4. 몬테카를로: 결합 불확실성

```sh
python -m control.mission_sim --scenario mc --trials 100 --seed 42 --ranges configs/uncertainty.example.json
```

각 시행에서 선택한 모든 인자를 **독립 균등분포**로 동시에 샘플링한다.
기본 무외란 40초 미션을 반복하며 같은 시행 조건을 모든 비교군에 적용한다.
시드·인자 이름 정렬로 재현성을 유지하고, 매 시행 제어 상태를 초기화한다.
기본 MC에는 돌풍이나 초기조건 랜덤화를 섞지 않아 불확실성만 평가한다.
물리적으로 상관된 공차가 있다면 현재 독립분포 가정을 그대로 쓰면 안 된다.

실패를 분모에 포함한 실패율과 Wilson 95% 구간을 기록한다. RMSE의 평균·표준편차·
P95·최악값은 **완주하고 지표가 유한한 시행**으로만 계산하며 표본 수도 남긴다.
실패 0건도 모든 조건에서의 강건성 증명은 아니다. 주장 범위는 모델, 공차 범위,
샘플링 분포와 판정 기준 안의 수치적 근거다.

## 판정과 산출물

`--criteria configs/acceptance.example.json`으로 기준을 지정한다. 기본 추종/안전 기준:
최대 고도 오차 10 m, 전체 속도 오차 norm 15 m/s, 각속도 35 rad/s 이하,
25 rad/s 초과가 0.2초 이상 지속하지 않음, 유한 상태와 완주.
기본 미션은 마지막 0.5초의 최종 호버도 돌풍 회복과 같은 오차 기준으로 확인한다.
이 값들은 연구용 설정이며 인증 기준이 아니다.

솔버 5회 연속 미수렴, 비유한 출력, 물리적 모터 한계 위반, 과대한 각속도/고도
이탈은 중단 사유로 저장한다. 모델 초기화 실패도 별도 실패 시행으로 집계한다.
`tracking_pass`와 `model_domain_valid`를 분리하며 **최종 passed는 둘 다 충족**해야 한다.
프로펠러 범위는 실제 모터 상태와 바람을 뺀 공기 상대속도로 확인한다.
범위 밖 결과를 좋은 RMSE만 보고 물리적 검증 성공으로 집계하지 않는다.

매 실행은 `results/validation/run_UTC시각/`을 생성한다.

- `manifest.json`: 모델 전체, 해시, 코드 해시, 버전, 기준, 타임라인, 실행 상태
- `cases.json`: 모든 시행의 인자값, 돌풍 조건
- `trials.jsonl`: 시행별 지표·실패 사유·구간별 RMSE(매 시행 즉시 저장)
- `summary.json`, `REPORT.md`: 집계와 사람이 읽는 결과표
- `*.solver.json`: 최적화 수렴 이력
- `*.npz`, `*.png`: 기본/돌풍 시계열·응답 그림; 스윕/MC의 NPZ는 `--save-traces`로 선택

`--dry-run`은 조건표와 설정만 저장한다. 대규모 실행 전 조건 수를 확인할 수 있다.
중단된 실행은 manifest의 `running` 상태와 이미 저장된 시행으로 식별한다.

## 검증 및 과거 실험

```sh
python -m models.team_light.control.validate_model --authority
python -m pytest models/team_light/tests control/test_validation_suite.py -q
```

과거 65초 미션을 쓰는 acados/측풍/legacy 스크립트는 명시적으로
`LegacyMissionProfile`을 사용한다. 기존 t=28/35/43초 조건을 새 타임라인으로
조용히 해석하지 않는다. `final_config_mission`도 과거 기체 재현용이다.
이번 변경은 Python 연구 시뮬레이션이며 ROS2/PX4 실기 경로는 새 기체로 이식하지 않았다.
