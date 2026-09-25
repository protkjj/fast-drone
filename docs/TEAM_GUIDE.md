# 팀 공유: 이번 변경과 실행 방법

공유 저장소: https://github.com/kms301111/fast-drone-control (비공개).
원본 저장소를 수정하지 않고, 원본의 커밋 이력과 이번 변경을 별도 저장소에서 공유한다.

## 무엇이 바뀌었나

- 기본 시험을 이륙·돌풍 없이 **호버 → 가속 → 크루즈 → 감속 → 호버**로 분리했다.
- 독립 돌풍 시험은 70 m/s 크루즈 트림에서 시작해 돌풍과 회복만 관찰한다.
- 기본 미션에 플랜트 불확실성 스윕과 시드가 있는 몬테카를로를 추가했다.
  제어기 내부 모델은 명목값을 유지하고 플랜트 사본의 인자만 바꾼다.
- 팀원 기체 `light_rocket_v2_pack_forward`(1.17407572 kg)를 공력·추진·제어기와
  함께 이식했다. 원본 코드의 동역학과 제어기 설정은 임의 조정하지 않았다.
- 주 제어기는 **NMPC+INDI (`Split`)**, 비교 대상은 GS-LQR과 단일 NMPC다.
- 매 시행의 완주 여부, 추종 오차, 솔버 실패, 모델 유효 범위와 정착 여부를 기록한다.
  실패 시행을 통계에서 성공처럼 처리하지 않는다.
- 과거 기체의 acados/legacy 시험은 이전 타임라인을 명시적으로 유지한다.

## 현재 기본 조건과 해석

기본 미션은 고도 50 m, 목표 속도 70 m/s(252 km/h), 구간별 **1/8/3/25/3초**로
총 40초다. 반 코사인 속도 참조의 최대 가속도는 13.74 m/s²,
최대 감속도 크기는 4.40 m/s²다. 독립 돌풍은 **1/1/2.5초**, 총 4.5초이며
무돌풍 대조군과 수직·측방 2 m/s 돌풍을 시험한다.

이 시간은 **시험한 후보 중 현재 하이브리드가 오차 예산을 만족하고 최적화 실패
없이 완주한 짧은 구성**이다. 제어기를 개선한 결과, 전역 최단 시간,
기체의 물리적 기동 한계 또는 확정된 연구 요구 조건으로 해석하지 않는다.
요구 가속도·감속도·허용 오차를 정한 뒤 같은 기동으로 제어기들을 비교해야 한다.

| 확인한 시험 | 결과 |
|---|---|
| 40초 기본 미션, NMPC+INDI | 완주, 최적화 실패 0회 |
| 40초 기본 미션, GS-LQR | 완주 |
| 40초 기본 미션, 단일 NMPC | 21.24초에서 중단, 마지막 5회 연속 최적화 실패 |
| 4.5초 돌풍, 3조건 × 3제어기 | 9회 모두 추종·모델 유효 범위 판정 통과 |
| 공유 전 회귀 테스트 | 73개 통과 |

기본 미션에는 프로펠러 데이터 하한 RPM 미만 및 역류 구간이 남아 있다.
따라서 하이브리드도 `tracking_pass=true`지만 `model_domain_valid=false`,
최종 `passed=false`다. 기본 미션의 물리적 검증이나 전체 불확실성 범위의
강건성 입증은 아직 완료되지 않았다. 스윕·몬테카를로 실행 기능과 대규모
강건성 검증 완료를 구분해야 한다. 솔버 중단은 NaN 발생을 뜻하지 않는다.

## 빠른 실행

Python 3.12를 기준으로 저장소 루트에서 실행한다. 현재 Python 기본/돌풍 시험에는
acados, ROS2, PX4가 필요하지 않다. 비공개 저장소이므로 초대받은 GitHub 계정으로
인증해야 복제할 수 있다.

```sh
git clone https://github.com/kms301111/fast-drone-control.git
cd fast-drone-control
python -m venv .venv
```

Windows PowerShell:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
$env:OPENBLAS_NUM_THREADS = '1'
$env:OMP_NUM_THREADS = '1'
.\.venv\Scripts\python.exe -X utf8 -m control.mission_sim --controllers Split
```

Linux/macOS:

```sh
source .venv/bin/activate
pip install -r requirements-dev.txt
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python -m control.mission_sim --controllers Split
```

이후 아래 `python`은 가상환경의 Python을 사용한다. PowerShell에서는
`.\.venv\Scripts\python.exe -X utf8`로 대체할 수 있다.

```sh
# 공통 참조로 세 제어기 비교 (단일 NMPC 중단도 결과에 기록)
python -m control.mission_sim --controllers GS-LQR NMPC Split
python -m control.gust_comparison --controllers GS-LQR NMPC Split

# 조건표 먼저 확인; --dry-run 제거 시 실제 시행
python -m control.mission_sim --scenario sweep --ranges configs/uncertainty.example.json --points 3 --dry-run
python -m control.mission_sim --scenario mc --ranges configs/uncertainty.example.json --trials 100 --seed 42 --dry-run

# 검증
python -m pytest models/team_light/tests control/test_validation_suite.py control/test_timing_study.py -q
```

## 문서와 결과 위치

- [실행 옵션·불확실성 인자·합격 판정](VALIDATION.md)
- [시간 후보 실험과 선정 근거](TIMING_OPTIMIZATION.md)
- [기체 출처와 원본 커밋](../models/team_light/PROVENANCE.md)
- [공유용 결과 요약](shared_results/README.md), [기본 미션 그림](shared_results/mission.png)

원본 기준은 `protkjj/fast-drone`의 `control` 커밋 `593b017`이다.
팀원 기체는 `KYUHYUNKANG03/fastdrone_kh`의 `repro/kyuhyun`,
커밋 `34981c5d64c102df5c2fd8b4b3d9a5e0e3db81f7`에서 가져왔다.
출처와 원본 저작 표시를 유지했다.

새 실행의 원시 로그·NPZ는 `results/validation/`에 생성되며 Git에는 올리지 않는다.
`.venv`, 캐시도 공유하지 않는다. `results/`의 다른 기존 그림과 문서는 과거 기체의
기록이므로 이번 기체의 성능으로 인용하지 않는다.
