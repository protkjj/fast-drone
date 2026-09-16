# 원래 시뮬레이터 복원 · 2026-09-16

기준은 9월 15일 마지막 커밋 `9b8bbfd`다. 새 연구 화면을 비슷하게 재디자인하는
방식이 아니라, 당시부터 보존한 `results/flight_sim.html`을 `/sim.html`에 다시 배포한다.
Git 기록 전체를 되돌리거나 이후 제어기·모델 수정 파일을 삭제하지 않는다.

## 어제 버전과 비교해 남긴 차이

1. **역류 공력 수정 유지** (`23163bd`): 축방향 속도가 음수일 때 항력 방향이
   반대로 작용하거나 횡력이 병진 에너지를 생성하지 않도록 한 수정이다.
   실제 플랜트·가상 예측·앞먹임·힘 표시·Python 기준식에 같은 처리를 유지한다.
   미검증 역류 영역의 연장식이지 실측 공력 모델은 아니다.
2. **Space 초점 문제 보완**: 슬라이더에 초점이 남아도 시작/정지가 된다.
   숫자 입력·버튼 기본 동작·반복키·조합키는 가로채지 않는다.
3. **모델 한계 설명 유지**: 무게중심 지면 판정과 STL 표면 충돌의 차이,
   역류 연장식의 한계를 밝힌다. 화면 배치나 기존 조작은 바꾸지 않는다.
4. **연구 엔진 별도 보존**: `/research/index.html?mode=compare`에 최신 선정 기체
   IPOPT/INDI/ESKF, 모터·배터리 제한, 로그·재생·추정 보강을 그대로 남긴다.
   `/sim-legacy.html`은 원래 시뮬레이터와 같은 내용의 호환 주소다.

어제 이미 반영된 Hybrid의 가상 명령/INDI 할당 수정, NMPC 단독의 모터 직접
최적화, 웜스타트·되감기 상태 복원, 실패 판정, STL·로터 회전은 **제거하지 않았다**.
`hybrid_interface.js`, `standalone_nmpc.js`는 어제 버전과 같고, 이번 복원에서
`research/model.py`, `runtime.js`, `nmpc.py`, `eskf.py`, `worker.js`는 변경하지 않았다.

주의: 원래 시뮬레이터는 **8 kg 시험 모델, JS 최적화**다. 원래 Hybrid는
NMPC 20 Hz / INDI 500 Hz로 실행된다. 선정 기체 1.712 kg와 실제 IPOPT
50 Hz / INDI 1 kHz / ESKF는 별도 연구 엔진이며 메인에 통합했다고 주장하지 않는다.
불확실한 ‘속도 300’ 변경은 적용하지 않고 원래 기본값 60 m/s·목표 고도 200 m로 복원했다.

## 확인

- 원래 시뮬레이터 34개 회귀 검사 통과: Hybrid SQP-RTI/NMPC 단독의 호버·60·83 m/s
  120초 검사, 할당/예측 정합성, 되감기, 힘 방향, 기록·실패 처리 포함.
- 추가 키보드 회귀 2개: 슬라이더/숫자/버튼 초점, 반복키, R/F.
- 배포 조립 시 `/sim.html`과 `/research/index.html`의 서로 다른 필수 조작부,
  CSS/JS/STL 경로와 호환 주소를 검사한다. 연구 페이지가 메인으로 리다이렉트되지 않는다.
- 별도 Chrome에서 원래 화면의 Space·방향키·R/F·되감기·풍속·공력 계수·채점,
  로터 정지 및 Hybrid PG/SQP-RTI/NMPC 초기 실행을 확인했다.
- 별도 연구 페이지의 참 상태/ESKF와 Hybrid/NMPC IPOPT 짧은 실행도 확인했다.
  0.02초 확인은 경로 점검이며 GPS 지연 융합이나 장시간 안정성 검증이 아니다.

공개본 추가 검사에서는 첫 실행의 PG 진행 대기(30초)와 다음 실행의 STL 준비
대기(30초)가 각각 시간 초과됐다. 원인 단정이나 제어기 수식 변경은 하지 않았다.
검사 실패 시 실행 상태/초점/페이지 가시성/표시 오류를 기록하도록 보강하고, 큰 STL의
최초 준비 대기만 90초로 분리했다. 비행 진행 기준과 30초 제한은 그대로 두고
같은 공개 산출물의 전체 입력·세 제어기 검사를 재통과했다. 첫 시간 초과의 원인은
재현되지 않았으며 확정하지 못했다.

```sh
python3 gz_aero/tools/build_sim.py
python3 gz_aero/tools/build_site.py --research
node --test gz_aero/test/*.test.cjs
node research/browser_smoke.cjs http://127.0.0.1:8765/sim.html .02 --original
node research/browser_smoke.cjs 'http://127.0.0.1:8765/research/index.html?mode=compare' .02
```
