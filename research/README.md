# 선정 기체 비행 시뮬레이터 — 관찰 / 정밀 비교

공개 `/research/index.html?mode=compare`에서 **선정 기체**를 관찰하거나 정밀 비교한다.
`/sim.html`은 사용자 요청으로 9월 15일의 원래 8 kg 시뮬레이터로 복원했다.
[복원 기준과 보존한 수정](../gz_aero/SIMULATOR_RESTORE.md)을 참고한다.
비행 화면에서는 **Hybrid(기본) / NMPC 단독 / PD–INDI**를 명시적으로 선택한다.
Hybrid와 NMPC는 관찰에서도 실제 IPOPT를 사용하며 느리다고 PD로 대체하지 않는다.
속도·고도·풍속·풍향은 슬라이더/숫자로 조작하고 적용된 20 ms 시각을 기록한다.
정밀 비교는 같은 목표·바람 일정을 두 제어기에 재사용한다.
PD–INDI는 빠른 조작용 baseline이며 Hybrid가 아니다. 연구 엔진은 메인 시뮬레이터와
다른 모델이며, `/sim-legacy.html`은 메인 8 kg 시뮬레이터와 같은 호환 주소다.
동일한 수식으로 Python과 브라우저의 물리·제어·추정 경로를 재현한다. **실물 검증, 제어기
우열 검증, 실제 20 ms 실시간 동작을 완료한 구현이 아니다.**

## 현재 상태와 알려진 문제

- **[선정안·STL·제약의 출처 점검](DESIGN_SOURCE_AUDIT.md):** CSV 계산값은 정확히
  재현되지만 원본 STL의 핀 위치·쓸림·두께는 계산 형상과 다르다. **사용자 결정으로
  `selected_design.csv`를 설계 기준으로 사용한다.** 연구 화면은 선정안 기준의
  STL 파생 형상을 표시하며, 원본 STL·물리 프로필·제어기는 보존한다. 아래의 평형
  제약 문제가 해결된 것은 아니다. 모터 40 A는 제품 정격이 아닌 시험 가정이다.
- **[선정 기체 힘·모멘트 평형 점검](TRIM_ENVELOPE_AUDIT.md):** 원본 사이징의
  순항 판정은 6DOF 자세 유지 보장이 아니다. 현재 모델의 무롤·등속 수평 조건에서
  중간 속도 구간은 음의 로터 추력을 요구하고, 83.3 m/s는 차등 추력 때문에
  모터 전류 상한을 초과한다. 실측 비행 불가능 판정은 아니며, 가속 전이와도
  구별한다. 제어기 비교 전에 모델/설계의 목표점 실현 가능성을 확인해야 한다.
  고받음각 공력의 실측 유효 범위는 미확인이다. UI·물리 계수는 변경하지 않았다.
- **[모터 능력 피드백 보강](ACTUATOR_FEEDBACK.md):** 현재 관측 회전수·전압에서
  20 ms 뒤의 명목 로터 힘 범위를 계산해 Hybrid의 첫 가상입력에 결합 제약으로
  전달한다. 13상태/모터 명령 역할 분리는 유지한다. 국소 근사이며, 전체 예측
  구간의 모터 지연 보상이나 엄밀한 실현 가능성 보장을 의미하지 않는다.
  비행 지표의 개선/악화가 섞여 있어 **기본 OFF인 실험 옵션**으로 보존한다.
  연구 주소에 `&actuator_feedback=1`을 붙이거나 실행 설정에서 명시적으로 켠다.
  [8회 전후 비교 결과와 실패·맵 밖 한계](ACTUATOR_FEEDBACK_VALIDATION.md)를 참고한다.
- **[연구 화면 배치 수정 기록](CLASSIC_UI_VALIDATION.md):** 왼쪽 슬라이더 / 가운데 큰 3D와
  하단 그래프·시간축 / 오른쪽 설정. Space 시작·정지·재개, R 초기화, F 따라가기,
  카메라 시점·Shift 이동·패널 폭 조절을 복원했다. 선정 기체의 물리·제어기는 유지한다.
- **[비행 조작성 복원](FLIGHT_CONTROLS_VALIDATION.md):** Hybrid 기본 선택, 슬라이더와
  숫자 입력, 풍속·방향의 실제 플랜트 반영, 적용 시각 기록, 비행 중심 3D 배치.
  사용자가 승인한 실제 브라우저 조작·재생 검사도 수행했다.
- **v2 시뮬레이터 보강:** 총추력 우선 비선형 INDI 할당, 전류/전압/모터 추종 제한
  진단, 센서 시각 정렬, 예고/비예고 목표, 풀이 경계 중지, 상세 v2 로그와
  시간축 재생을 제공한다. [변경 이유와 채널 정의](INTERFACE_V2.md)를 참고한다.
- **[화면 통합·공통 계산 검증](UNIFIED_VALIDATION.md):** 관찰 중 목표 변경·일시정지·중지,
  목표 일정의 정밀 비교 재사용, 수식 생성 평가기와 원래 WASM의 수치 일치를 검사했다.
  관찰/비교는 같은 플랜트·추정·INDI 함수를 사용한다. 화면 갱신 속도가 적분 간격을 바꾸지 않는다.
- **[이전 v2 검증](VALIDATION_V2.md):** 12개 시험 모두 종료, IPOPT 1,560회 중
  16회 반복 상한 도달. 외란 3개 seed에서 Hybrid는 평균 속도 오차가 작았지만
  에너지는 더 사용했고 평균 고도 오차도 더 작지 않았다. 109개 회귀 및 공개
  브라우저 검사를 통과했다. 절전이 섞인 경과 시간 통계는 성능 순위에 쓰지 않는다.
- 최적화는 실제 CasADi/IPOPT WebAssembly를 사용한다. 플랜트·센서·할당 수치는
  같은 CasADi 수식의 명령열에서 생성한 JavaScript로 평가하여 행렬 전달 비용을 줄인다.
  원래 WASM 평가 경로도 수치 대조용으로 유지한다. 서버에서 풀이하지 않는다.
- 짧은 호버에서 Hybrid와 NMPC 단독의 실행, Python/WASM 수치 일치,
  다중 주기, GPS 지연 보정, 모터 에너지 수지 및 STL 변환을 검사했다.
- **2026-09-15 수정 후 선정 기체 속도 변경 시험:** 같은 2초 명목 조건에서
  NMPC 단독의 풀이 실패가 초기 100/100회에서 0/100회로 줄었다. 상태·입력
  스케일링, 전류·전압의 명시적 제약, 20 ms에 맞는 웜스타트를 적용했다.
  실제 플랜트의 포화는 유지한다. 실패 판정을 느슨하게 바꾸지 않았다.
- 이전 v1의 동일 조건 6개 시험에서도 일부 풀이 실패와 맵 밖 표본이 남았다.
  Hybrid가 모든 지표에서 우세하지 않았다. [검증 결과와 한계](VALIDATION.md)를
  함께 읽고, 초기의 실패한 baseline을 Hybrid 우위의 근거로 사용하지 않는다.
- 로컬 브라우저 호버 검사에서도 실제 풀이 시간이 20 ms를 넘었다.
  시뮬레이션 시간은 풀이 중 멈춘다. 50 Hz 스케줄 구현은 실시간 성공을 뜻하지 않는다.
- baseline 중 선정 기체에 연결된 것은 NMPC 단독이다. 기존 페이지의 LQR 등은
  종전 기체이므로 여기의 선정 기체 Hybrid와 직접 비교하지 않는다.

## 구조와 단위

| 구성 | 정의 |
| --- | --- |
| 좌표 | body +x 추력, world +z 위쪽, body→world quaternion `[x,y,z,w]` |
| 플랜트 | 17개 기계 상태 `[p,v,q,omega,n]` + 배터리 SOC 1개 |
| Hybrid NMPC | 13상태, 가상 입력 `[T,angular acceleration]` |
| INDI | 자이로 미분/동기 필터, 총추력 등식·로터 힘 경계 할당, 공통 추진 맵 역산 |
| NMPC 단독 | 17상태, 네 로터 회전수 명령을 직접 최적화; INDI 없음 |
| 관찰용 PD–INDI | 명목 속도·고도·자세 PD 50 Hz → 기존 INDI 1 kHz; 최적화 없음 |
| NMPC 설정 | N=20, 예측 간격 0.05 s, 1 s 범위, 매 0.02 s, 최대 30회 반복 |
| 빠른 주기 | 플랜트 RK4 / IMU / INDI 0.001 s |
| 바람 | world 흐르는 방향: 0° +X 순풍, 90° +Y 측풍, 180° −X 역풍; 0–30 m/s |
| 추정 | 15D error-state EKF, 10 Hz GPS, 20 ms 지연 보정 + IMU 재적분 |

회전수 상태·명령의 단위는 **rad/s**다. 추진 계수식 안에서만 rev/s로 변환한다.
바람은 실제 플랜트에만 적용하고 제어기에 참 바람을 몰래 전달하지 않는다.
기존 gust 시나리오의 +Y 3 m/s와 모멘트 펄스는 기본 바람에 더해진다.
관찰 변경은 `configuration.commands`의 `wind_speed`/`wind_angle`과 trace의
6성분 `environment`에 기록된다. 과거 무풍 로그는 기본값 0 m/s로 호환한다.
센서 시험 제어기에는 추정 상태·자이로·가정한 RPM 텔레메트리를 전달한다.
참 상태는 센서 생성과 평가에 사용한다. 두 제어기는 같은 초기값, 난수 seed,
플랜트 오차, 외란, 상태 추종 가중치를 사용한다. 입력은 의미가 다르므로 각각
무차원화한 페널티를 적용한다. 체계적인 baseline 튜닝은 아직 완료하지 않았다.

## 기체 자료와 가정

선정 프로필: `profiles/selected.json`, CSV source ID 6931.

- 제공 `selected_design.csv` SHA-256:
  `326342d0e471bbb424eb558e4034f4e2458a7a2b669b19cfc2af59b96e8528ff`
- 원본 사이징 코드: [Rocket-Drone-Project 고정 리비전](https://github.com/gocksk/Rocket-Drone-Project/tree/db793a7039c76c9937597a240efc94f68c5a8af0).
  CSV의 값이 있는 모델 출력 70개를 재현했다. 개념설계 예측이며 측정값이 아니다.
- 질량 1.711714 kg, 관성 `[0.007651085,0.034966535,0.034966535]` kg·m²,
  길이 0.670499 m. CT/CP 맵과 모터·배터리 모델을 함께 가져왔다.
- 배터리 Wh/Ah를 공칭 3.7 V/cell 기준으로 통일했다. 모터 전기 입력,
  축동력, 동손, 무부하 손실, 로터 가속 에너지 및 전압 강하를 구분한다.
- 로터 관성 1e-5 kg·m², 속도 루프 20 ms, 모터별 전류 상한 40 A,
  공력 감쇠, 센서 잡음·바이어스·GPS 지연·RPM 텔레메트리는 **가정**이다.
- 맵 밖/역유입은 수동적인 연장 모델로 처리하고 비율을 기록한다. 풍차·회생
  검증 모델이 아니다. 열, 전기 인덕턴스, 나선형 팁 마하 제약은 미구현이다.
- `simple`은 2 kg의 별도 가정 모델이다. 선정 기체로 오해하지 않도록 STL을 숨긴다.

STL: `assets/drone_v2.stl`, 원본 SHA-256
`276045699ee8e9ee03fe5d75d4915d88a0688fbcb12a9e8d1e8f5c0702302e35`.
173,804개 삼각형. CAD 길이 670.499와 CSV 길이 일치를 근거로 mm 단위를
해석했다. 기수 X=0, CG=기수에서 428.251 mm로 변환한다. X와 Z를 함께
반전하는 정회전을 사용하여 반사 좌표계를 만들지 않는다. STL은 표시용이며
질량·관성·공력 계수를 자동 계산하지 않는다. 원본의 21개 연결 성분을 조사해
블레이드 8개와 허브 4개를 네 로터로 묶었다. 삼각형을 삭제하거나 원본 STL을
수정하지 않고 각 축 중심으로 회전한다. 파일 해시·분할·초기 형상 복원을 검사한다.
선정 연구 페이지에서는 `assets/selected_geometry.json`의 출처가 명시된 치수로
핀·하우징·로터 배치를 변환한 **별도 표시 메시**를 사용한다. 동체 표면과
블레이드·허브 세부 형상은 기존 STL을 재사용하며 제조용 CAD나 CFD 모델이 아니다.
원본 STL, CG·관성·CP·추진 모델은 이 표시 변환으로 수정하지 않는다.
화면의 회전은 실행 중 RPM 상태를 읽되 최대 약 2.9회/초로 시각적 감속하고
옅은 회전 잔상을 추가한다. 실제 RPM·추력·토크·적분 위상에는 영향을 주지 않는다.

## 로컬 실행 및 검사

저장소 루트에서 실행한다. Python CasADi 3.7.2와 공식 WASM 3.8.0 간의
직렬화 호환성은 실제 수치 회귀 검사로 확인한다. Node 22 이상을 권장한다.

```sh
python3 -m pip install -r research/requirements.txt
npm ci --prefix research --ignore-scripts --no-audit --no-fund
python3 -m research.build_bundle
python3 -m pytest research/test_model.py research/test_trim_envelope.py research/test_design_source_audit.py research/test_estimator.py research/test_nmpc.py control/test_research_parity.py -q
npm test --prefix research
python3 gz_aero/tools/build_site.py --research
python3 -m http.server 8765 --directory site
```

`http://127.0.0.1:8765/research/index.html`에서 선정 기체 관찰을 시작한다.
정밀 비교 모드는 같은 주소에 `?mode=compare`를 붙이고 먼저 기본 0.2초 호버를 실행한다.
브라우저 최초 실행은 WASM과 모델을 다운로드한다. STL과 런타임을 합쳐
수십 MB이며, 저장 공간이나 통신량이 제한된 기기에서는 주의한다.
풀이 실패와 실제 계산 시간을 반드시 확인한다. 중지는 현재 풀이를 마친 후 처리된다.

```sh
node research/run.cjs selected hybrid truth 0.2 hover
node research/run.cjs selected nmpc eskf 0.2 hover
node research/benchmark.cjs research/generated/benchmark-new.json
node research/browser_smoke.cjs http://127.0.0.1:8765/research/ 0.2
node research/browser_smoke.cjs http://127.0.0.1:8765/research/ 0.2 --extended
node research/browser_smoke.cjs http://127.0.0.1:8765/research/index.html .08 --classic
node research/validate_v2.cjs research/generated/validation-new.json
node research/summarize_v2.cjs research/generated/validation-new.json research/validation-new-summary.json
```

benchmark 명령은 동일 조건의 6개 속도 변경 시험을 순차 실행한다. 실제 WASM
최적화이므로 수십 분 걸릴 수 있다. 출력은 완료된 사례마다 갱신된다. 기존 로그를
덮어쓰지 않도록 새 출력 이름을 사용한다. 전체 생성물·의존성·실험 로그는 Git에
포함하지 않고, 출처 해시와 지표를 추린 `validation-2026-09-15.json`을 보관한다.
browser_smoke는 macOS의 별도 임시 Chrome 프로필에서 두 제어기·두 피드백
경로와 로터 회전/정지를 검사한다. `RESEARCH_CHROME`으로 실행 파일을 지정할 수 있다.

`--extended`는 실제 JSON 저장·가져오기·재생·설정 복원·중지·좁은 화면을 추가로
확인한다. 임시 브라우저 프로필과 다운로드를 보존하고 사용자 브라우저에는
접속하지 않는다. `validate_v2`는 6개 조건 × 2개 제어기를 동일 설정으로 실행하며,
외란에 seed 7/42/2026을 쓴다. 새 파일만 생성하도록 `wx`를 사용한다.
노트북 절전·다른 작업은 경과 시간 통계에 영향을 준다. 이를 포함한 수치를
IPOPT 계산 성능이나 실제 20 ms 마감 보장으로 해석하지 않는다.

속도 변경 시험은 1.1초 이상, 2–3초 외란 시험은 3.1초 이상이어야 한다.
이 최소 기간이 정착 시간이나 강건성 평가에 충분하다는 뜻은 아니다.
상세 1 kHz 로그는 20초 이하이며, JSON 파일은 최대 256 MiB까지 가져온다.
로그 재생은 기록 보간일 뿐 새 물리 적분이 아니다. 설정 재사용은 현재 버전의
코드를 실행하므로 원본 로그의 구현 해시와 비교해야 한다.

## 코드 위치와 배포

- `model.py`: 추진·모터·배터리·6DOF 및 공통 수식
- `trim_envelope.py`: 동일 플랜트의 힘·모멘트·모터 한계를 검사하는 오프라인 평형 진단
- `design_source_audit.py`: 선정 CSV/원본 코드·STL 형상·시험 가정의 출처와 차이 추적
- `assets/selected_geometry.json`: 선정안 사이징 치수와 출처를 기록한 표시 전용 명세
- `nmpc.py`: 두 IPOPT 최적화 문제
- `eskf.py`: 추정 전파·GPS 갱신·오차 리셋
- `runtime.js`: 다중 주기, 센서, INDI, 지연 GPS 재적분, 평가
- `numeric_export.py`: 공통 CasADi 명령열에서 수치 평가기 생성; 별도 물리식 없음
- `index.html`, `classic.css`, `page.js`, `worker.js`, `stl.js`: 웹 UI·Worker·외형
- `results.js`: v2 로그 검사·동일 조건 검사·시각적 재생 보간
- `build_bundle.py`, `build_site.py`: 직렬화 및 정적 배포 패키징
- `../gz_aero/tools/build_sim.py`: **기존** 실시간 데모 원본

GitHub Pages는 `bulnabi`의 `.github/workflows/pages.yml`로 배포한다.
연구 검사와 빌드가 성공해야 배포한다. 메인은 `/sim.html`의 원래 시뮬레이터,
선정 기체 연구는 `/research/index.html?mode=compare`로 분리한다.
배포는 연구 기능의 공개이지, 미완료 비교 실험의 성공 선언이 아니다.

보존된 `sim-legacy.html`에는 CSV 형상 보정 없이 원본 STL을 표시한다. 선정 CG를 원점으로 변환한
메시가 기존 플랜트의 위치·쿼터니언을 따라가며, 가시성을 위한 표시 배율은 ×5다.
기존 8 kg 계산 모델은 그대로이고 화면 머리글에 이를 명시한다. 모델의 크기나
모터 회전수 상태를 바꾸지 않는다. 연구 페이지와 같은 로터 분리·회전 표시를
사용하며 일시정지 또는 과거 기록 보기에서는 시각적 회전도 멈춘다.

향후 순서: NMPC 수렴/계산 비용 개선 → 동일 조건의 센서·오차·외란 검증 →
여러 seed 및 체계적인 baseline 튜닝 → 선정 기체의 강건성 결론 검토.
