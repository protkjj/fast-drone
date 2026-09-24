# PX4 연결 전 정합성 점검 — 2026-09-17

상태: 저장소 정적 점검 완료. **선정 기체 PX4 SITL 연동/실행은 미완료**.
웹의 시동 수정은 PX4 펌웨어를 실행하거나 이식한 것이 아니다.
사용자 선택: 이번 작업은 웹 수정·검증까지. 아래 연결 단계는 후속 작업 참고용이며
PX4 설치/실행/기체 파라미터 변경은 이번에 수행하지 않는다.

## 현재 경로의 차이

| 항목 | 메인 웹 `/sim.html` | 선정 기체 `research/` | 기존 ROS2/PX4 노드 |
| --- | --- | --- | --- |
| 질량 | 8 kg 시험 모델 | 1.711714 kg 개념설계 | 제어기 파라미터 사본 8 kg |
| 관성 kg·m² | 0.02 / 0.70 / 0.70 | 0.007651 / 0.034967 / 0.034967 | 0.02 / 0.70 / 0.70 |
| 추진 | 전진비 보정 + 20 ms 1차 모터 | CT/CQ 표, 전압·전류·회전자·배터리 모델 | 별도 Gazebo 모델과의 대조 필요 |
| 실행 주기 | NMPC 20 Hz / INDI 500 Hz | NMPC 50 Hz / INDI 1 kHz (시뮬레이션 시간) | 노드 기본 타이머 100 Hz |
| 시작 | 0 RPM → 공통 시동 → 선택 제어기 | 공중 호버 초기조건 | Offboard/ARM 요청 후 직접 모터 명령 |
| 회전수 피드백 | 플랜트 상태 | 실험의 정의된 센서/상태 경로 | 보낸 명령을 1차 필터링한 내부 추정값 |

근거 파일:

- `gz_aero/tools/build_sim.py`, `control/vehicle_params.py`
- `research/profiles/selected.json`, `research/model.py`, `research/runtime.js`
- `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`
- `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers/vehicle_params.py`
- `scripts/sitl_run.sh`

SITL 실행 스크립트는 `/home/kj/px4_drone/PX4-Autopilot`과
`/home/kj/px4_drone/ros2_ws`를 가리킨다. 이번 macOS 작업 환경에서는 첫 경로가 없고
`ros2`, `gz`, `MicroXRCEAgent`, `px4`도 현재 PATH에서 찾지 못했다.
사용자의 다른 Ubuntu 환경이나 다른 설치 경로까지 조사한 결과는 아니다.

README의 기존 기록에는 Gazebo 동체 공력 연결/고속 검증이 미완료라고 적혀 있다.
외부 PX4 트리의 현재 SDF/airframe 설정을 읽지 못했으므로, 이 기록만으로 현재
사용자 Ubuntu 모델의 물리나 실행 성공 여부를 확정할 수 없다.

## 연결할 때 지킬 제어 경계

- 속도만 PX4에 보내는 시험은 PX4 내부 제어기가 작동하는 baseline 시험이다.
  그것을 `NMPC → INDI → 개별 모터` Hybrid의 검증 결과로 기록하지 않는다.
- Hybrid에서는 NMPC의 총추력·각가속도 명령과 INDI의 모터 명령 소유권을 유지한다.
  INDI 출력을 직접 모터 인터페이스로 연결하거나, INDI를 PX4 모듈로 옮기는 선택이 필요하다.
- 기존 노드는 `direct_actuator=true`와 `ActuatorMotors`를 사용한다. 이 경로에서
  PX4 기본 위치/이륙 제어기의 램프가 적용된다고 가정하지 않는다. ARM 확인,
  공회전/이륙 상태 관리, 통신 실패 시 제어권 인계도 별도로 검증한다.
- 직접 모터 입력의 정규화 값은 물리적인 rad/s 그 자체가 아니다. Gazebo 플러그인의
  명령 변환, 최대 회전수, 모터 번호/회전 방향, NED/NWU/기체 좌표를 함께 대조한다.
- PX4 EKF2를 쓰는 시험과 현재 ESKF 시험은 추정기가 다른 시험이다. 모터 회전수도
  실제/모의 ESC 계측인지 명령 기반 추정인지 명시한다.
- Python ROS2 노드의 기본 100 Hz 타이머를 그대로 두고 INDI 1 kHz라고 표시하지 않는다.
  내부 고속 루프 실행 위치를 정한 뒤 주기·지연·지터·deadline miss를 측정한다.

## 다음 단계와 통과 조건

1. **실행 환경 확인:** 사용할 Ubuntu/PX4 트리와 버전, `px4_msgs`, airframe/SDF를
   읽고 기존 저속 호버를 재현한다. 재실행 스크립트가 기존 프로세스를 종료하므로,
   사용자의 실행 중 작업을 확인하지 않은 채 `sitl_run.sh`를 실행하지 않는다.
2. **선정 기체 물리 맞춤:** 질량/관성/로터 위치만 바꾸지 않고 CT/CQ, 명령 변환,
   전압·전류 한계, 모터 응답, 공력까지 맞춘다. 같은 상태·입력에서 힘/모멘트와
   모터 가속을 Python 연구 모델과 대조한다. STL은 외형이며 이 값을 대신하지 않는다.
3. **시동/제어 경계 시험:** 정지·공회전·이륙·중단, 모터 단독 입력, 좌표/회전 방향,
   포화, 상태/명령 통신 두절을 시험한다. 초기에는 저속·무풍부터 시작한다.
4. **동일 조건 비교:** 같은 기체/초기조건/시동 정책/센서/외란을 적용한 Hybrid와
   baseline을 비교한다. IPOPT 20 ms 계산 예산과 INDI 1 ms 실행은 별도 실측한다.

PX4 버전/실행 환경과 제어 루프 배치가 결정되기 전에는 기존 웹 UI나 연구 결과를
PX4 실행 결과로 바꾸어 표시하거나 공개 배포하지 않는다.

공식 참고:

- [PX4 Simulation](https://docs.px4.io/main/en/simulation/)
- [Offboard의 제어 입력 수준](https://docs.px4.io/main/en/flight_modes/offboard)
- [COM_SPOOLUP_TIME](https://docs.px4.io/main/en/advanced_config/parameter_reference#COM_SPOOLUP_TIME)
  — 무장 후 운용 대기 정책이며 특정 모터의 물리 시정수/실측 시동 곡선이 아니다.
