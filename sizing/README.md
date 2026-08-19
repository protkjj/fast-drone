# sizing/ — 중량·사이징 파트

고속(300 km/h) 미사일형 쿼드 드론의 **중량 산정(mass sizing)** 작업 폴더.
제어(controller.py, hybrid_comparison.py 등)는 이미 확정되어 있으므로 건드리지 않는다.

## 이 폴더의 목적

`control/vehicle_params.py`의 질량 특성은 현재 **플레이스홀더**다:

    'mass': 8.0,     # ← 근거 없는 임시값
    'Ixx':  0.02,
    'Iyy':  0.70,
    'Izz':  0.70,

이 값들을 **구성품 단위로 쌓아올린 근거 있는 숫자**로 대체하는 것이 최종 산출물이다.
즉 이 폴더의 출력 = `control/vehicle_params.py`의 입력.

> **주의:** `vehicle_params.py`는 `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers/`
> 에도 **동일 사본**이 있다 (ROS2 노드가 상대 import 로 자기 사본을 씀).
> 질량/관성을 갱신하면 **두 파일 모두** 고쳐야 파이썬 시뮬과 SITL 이 같은 기체를 난다.

## 산출 목표

1. **중량 분해표 (mass breakdown)** — 구조 / 추진(모터·프로펠러·ESC) / 배터리 / 페이로드(ISR 센서) / 아비오닉스 / 배선·체결
2. **CG 위치** — 공력 압력중심 `x_cp = -0.10 m`와의 상대 위치가 정적 안정성을 결정
3. **관성 모멘트 Ixx / Iyy / Izz** — 구성품 배치로부터 계산 (축대칭 가정이면 Iyy = Izz)
4. **에너지/항속 성립성** — 배터리 중량 ↔ 비행시간 트레이드오프

## 파일 규칙 (프로젝트 컨벤션 준수)

- 소스 `.py`는 이 폴더 바로 아래
- 산출물(txt / md / png / csv)은 `sizing/results/`
- 플롯 라벨은 영어 (matplotlib 기본 폰트가 한글 미지원)
- 물리 상수를 새로 정의하지 말고, 가능한 한 `from control.vehicle_params import vehicle_params` 로 대조할 것
- 실행은 저장소 루트에서 모듈 형태로: `python3 -m sizing.<모듈명>`

## 현재 구성 (게이트 1 완료)

```
sizing/
├── wght.py            WGHT 본체 — 순수 함수. 수렴 상태머신 4분할 + 질량 특성
├── strc_stub.py       STRC 스텁 5종 (선형·멱함수·양자화·노이즈·비순수)
├── test_wght.py       검증 15종.  python3 -m sizing.test_wght
├── measure_noise.py   resid_floor·양자화 실측.  python3 -m sizing.measure_noise
└── results/
    ├── GATE1_REPORT.md              게이트 1 보고 (이론 재현 + 발견 4건)
    └── gate1_noise_measurement.txt  실측 원출력
```

근거 문서는 C-1-ii-01-ICD-001 / WGHT 시트 rev.4 + 검토 지적 2건.
설계 원칙·기호·조항 번호(§)는 전부 그 문서를 따르며, `wght.py` 주석이 조항을 인용한다.

**핵심 원칙 셋만 요약하면:**

1. **순수 함수** — 상태를 안 든다. C-2 배치에서 설계점끼리 오염되면 안 되기 때문이다.
   주입되는 `strc_fn` 도 순수해야 하며, `options['check_strc_purity']=True` 로 대조할 수 있다.
2. **relax 격리** — `relax` 는 갱신식과 `Ŝ` 환산식 **두 줄에만** 등장한다.
   판정·가드·이력 어디에도 relax 에 의존하는 양을 두지 않는다 (같은 버그가 3회 재발한 뒤 세운 원칙).
3. **반환값은 Σbreakdown** — 완화된 반복값이 아니라 마지막 STRC 호출의 합을 보고한다.
   그래서 `MTOW == sum(breakdown.values())` 가 항등식이고 검산이 필요 없다.

## W_avio 산정 전제 (2026-08-19 확정)

`constants['W_avio']` 는 TBD 지만, **조사 범위는 확정됐다.** 제어 아키텍처 배치가
FC/CC 분담을 정하고, 그게 곧 CC 등급 = 아비오닉스 질량이기 때문이다.

> **INDI 를 포함한 모터 레벨 전체는 FC(H7). CC 는 50 Hz 가상명령 생성만.**
> 상세 근거는 HANDOFF "제어 아키텍처 배치 확정 (2026-08-19)".

따라서 CC 가 지는 부하는 **50 Hz VirtualNMPC 하나**(순항 솔브 0.55~0.65 ms = 20 ms 예산의 3%,
감속은 게인 룩업). → **20 g 급 가설**이 물리적으로 타당하다.

**중량 파트에 중요한 점 둘:**

1. **여기엔 "무게를 더 써서 푸는 선택지"가 없다.** CC 측 1 kHz INDI 가 막히는 이유는
   연산이 아니라 링크(DDS 왕복 지연·지터)라, CC 를 Jetson 급으로 올려도 안 풀린다.
   즉 이 항목은 트레이드가 아니라 **단일 해**이고, 질량은 그 결과로 따라온다.
2. **⚠ 0.55 ms 는 개발 머신 실측이지 타겟 SoC 실측이 아니다.** 20 g 급을 확정하려면
   타겟에서 acados 생성 C 를 크로스 컴파일해 한 번 재야 한다. 이게 남은 값싼 검증이다.

`W_avio` 에는 FC·CC 외에 **RTK 로버 + RTCM 보정 링크**(베이스국+텔레메트리 또는 NTRIP over LTE)와
**BEC** 가 들어간다. BEC 는 CC 확정 전에 못 고른다(6S→5V 에서 1 A 급과 5 A 급은 용량·발열이 전혀 다름).
RTK 를 포함하는 근거는 중량 시트가 아니라 제어 분석에 있다 —
*MISSION_ANALYSIS §6: 표준 GPS 시 Hybrid → LQR 역전(+500~1500% 저하)*. 즉 제어 아키텍처의 전제 조건이다.

## 주의 (프로젝트 최대 함정)

- 동체 좌표계는 **z-down** (미사일 공력 관례). CG·모멘트 부호 결론은
  `control/dynamics.py` 원본과 대조하기 전에 확정하지 말 것
- 극단적인 숫자가 나오면 보고 전에 버그부터 의심 (이 프로젝트 전력)
- 중량을 목표치(300 km/h)에 맞춰 끼워맞추지 말고 물리적 타당성을 먼저 확인
