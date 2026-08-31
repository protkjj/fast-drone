# fast-drone — 고속 ISR 드론 제어 연구

축대칭 미사일형 동체 + 쿼드 추진, 목표 순항 **300 km/h (83.3 m/s)**.
최종 목표는 실기 비행이고, 경로는 `파이썬 시뮬 → PX4 SITL/Gazebo → 실기`다.

이 문서는 **테스트를 어떻게 돌리는지**를 다룬다.
환경 구축은 `SETUP.md`, 진행 이력은 `HANDOFF.md`, 결과 해석은 `results/` 를 본다.

---

## 0. 설치

```
git clone https://github.com/protkjj/fast-drone.git
cd fast-drone
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`acados` 는 **선택**이다. pip 로 안 되고 소스 빌드가 필요하다(`SETUP.md` 참고).
없어도 아래 §2 의 대부분이 돈다 — 없으면 못 도는 것만 표에 따로 표시했다.

**모든 명령은 저장소 루트에서 실행한다.** 파이썬은 모듈 형태(`-m`)로 부른다.
그래야 `control/` 패키지의 상대 import 가 풀린다.

---

## 1. 테스트 지도 — 한눈에

빠른 것부터 순서대로. 위에서 아래로 내려가면서 실패하는 지점이 곧 원인 구간이다.

| # | 명령 | 무엇을 검증 | 소요 | 전제 |
|---|---|---|---|---|
| 1 | `python3 -m control.test_safety` | SafetyGuard(NaN·채터링·회전감쇠) | 0.05 s ✅ | — |
| 2 | `python3 -m sizing.test_wght` | 중량 수렴 알고리즘 | 0.01 s ✅ | — |
| 3 | `python3 -m control.trim` | 트림 성립 + 최고 트림속도 | 0.3 s ✅ | — |
| 4 | `python3 -m control.test_plant` | 6-DOF 플랜트 (자유낙하·모멘트 등) | 1.7 s ✅ | — |
| 5 | `python3 -m control.test_fallback` | Hybrid→LQR 폴백 전환 로직 | 6 s ✅ | — |
| 6 | `./gz_aero/verify.sh` | 공력 C++ 코어 ↔ 파이썬 (겹1) | 5.5 s ✅ | C++17 컴파일러 |
| 7 | `python3 gz_aero/tools/selftest_check_gz.py` | 겹2 검증 **도구 자체**의 판별력 | 5 s ✅ | — |
| 8 | `python3 -m control.mission_sim` | 통합 미션 65 s × 제어기 3종 | 수 분 📄 | matplotlib |
| 9 | `python3 -m control.gust_comparison` | 돌풍 외란 응답 | 수 분 📄 | — |
| 10 | `python3 -m control.ekf_comparison` | 센서 노이즈 하 제어기 순위 | 수 분 📄 | — |
| 11 | `python3 -m control.final_config_mission` | 확정 구성(acados) 통합 검증 | 수 분 📄 | **acados** |
| 12 | `ctest --test-dir gz_aero/build` | 공력 코어 (CMake 경유) | 0.1 s ✅ | cmake |
| 13 | `gz sim ... frame_check.world` | 좌표 변환 (겹2a) | — | **Gazebo** |
| 14 | `gz sim ... accel_check.world` | 힘 적용점 (겹2b) | — | **Gazebo** |
| 15 | `./scripts/sitl_run.sh` | PX4 SITL 이륙·호버 | — | **Ubuntu+PX4** |

✅ = 2026-08-27 macOS 에서 실측한 시간  📄 = 코드 주석에 적힌 값, 이 세션에서 미실측

> 1~7 은 **파이썬 의존성 + C++ 컴파일러만으로 20초 안에 다 돈다** (합계 실측 18.6 s). 저장소를 처음 받았으면 여기까지 먼저 하면 된다.

---

## 2. 파이썬 제어 시뮬 (`control/`)

### 2.1 단위 테스트 — 통과/실패가 명확한 것

```
python3 -m control.test_plant
python3 -m control.test_fallback
python3 -m control.test_safety
```

- `test_plant` — 플랜트 자체. 로터를 끄면 자유낙하하는가, 단일 모터 bump 가
  올바른 축으로 모멘트를 내는가 등. **플랜트가 틀리면 그 위의 모든 결과가 무의미**하므로
  제일 먼저 돌린다.
- `test_fallback` — 스텁 기반 전환 로직 4개 + 실제 Hybrid 스모크 1개.
  NMPC 미수렴 트리거 포함. 오탐(정상인데 폴백)이 없는지도 본다.
- `test_safety` — `ros2_ws` 의 `safety.py` 를 직접 import 한다.
  NaN 검사가 채터링 가드보다 **먼저** 오는지(순서가 중요), 최후 폴백에서
  회전 감쇠가 걸리는지.

전부 `ALL TESTS PASSED` 가 마지막 줄에 나와야 한다.

### 2.2 트림 — 이 기체가 그 속도로 날 수 있나

```
python3 -m control.trim
```

속도를 훑으며 정상 수평비행 `(θ, n_eq, Δn)` 를 찾는다. 잔차가 `1e-10` 수준으로
떨어져야 진짜 트림이다. 마지막에 **물리적 최고 트림속도**가 나온다.

기체 파라미터(`control/vehicle_params.py`)를 바꿨을 때 **제일 먼저 돌려야 하는 것**이다.
여기서 트림이 안 잡히면 제어기를 아무리 튜닝해도 소용없다.

### 2.3 통합 시뮬 — 성능 비교

```
python3 -m control.mission_sim
```

이륙(0~10s) → 안정화 → 가속(13~28s) → 순항(28~43s, t=35s 돌풍) → 감속 → 호버,
총 65초를 LQR / Hybrid / NMPC 세 제어기로 돌려 RMSE 표를 낸다.

⚠ **`results/mission_plot.png` 와 `results/mission_ekf_plot.png` 를 덮어쓴다.**
둘 다 git 추적 대상이라 실행하면 작업트리가 더러워진다. 비교 목적이면
그대로 두고, 아니면 `git checkout -- results/mission_plot.png` 로 되돌린다.

속도 감(코드 주석 기준): LQR ~5초/회, Hybrid ~50초/회. NMPC 는 더 느리다.

### 2.4 개별 비교 실험

```
python3 -m control.gust_comparison
python3 -m control.ekf_comparison
python3 -m control.hybrid_comparison
```

- `gust_comparison` — 70 m/s 순항 중 측풍/수직돌풍 주입. 요·롤 응답과 받음각 급변.
- `ekf_comparison` — 센서 노이즈를 넣었을 때 제어기 **순위가 뒤바뀌는지**.
  무노이즈 시뮬만 믿으면 안 된다는 것을 보이는 실험이다.
- `hybrid_comparison` — 나이브 하이브리드 vs 인터페이스 분리. 왜 분리해야 하는지의 근거.

### 2.5 acados 가 필요한 것

```
python3 -m control.final_config_mission
python3 -m control.bench_acados
python3 -m control.acados_fallback_mc
```

`acados` 미설치면 import 에서 죽는다. 실기 실시간 NMPC 판정용이라,
시뮬 검증 단계에서는 건너뛰어도 된다.

```
python3 -m control.bench_compute
```

이건 acados 없이 돈다. 플랫폼별 계산 성능 실측 — Ubuntu 로 옮겼을 때
재실행해서 macOS 수치와 비교하는 용도다.

---

## 3. 사이징 (`sizing/`)

```
python3 -m sizing.test_wght
```

중량 수렴(WGHT) 알고리즘 단위 테스트. 발산 설계점을 제대로 발산으로 분류하는지 포함.

```
python3 -m sizing.gate2_shat_bias
python3 -m sizing.measure_noise
```

게이트 검증 스크립트. 자세한 배경은 `sizing/README.md`.

> 팀 사이징 저장소(`rocket-drone`)는 **별개 저장소**다. `gz_aero` 가 그쪽 값을
> 참조할 때는 `ROCKET_DRONE_PATH` 환경변수로 경로를 준다 (기본
> `~/Desktop/dynamic/rocket-drone`).

---

## 4. Gazebo 공력 플러그인 (`gz_aero/`)

설계 계약은 `gz_aero/DESIGN.md`, 사용법 상세는 `gz_aero/USAGE.md`.

검증을 **2겹**으로 쪼갰다. 하나로 묶으면 실패했을 때 "수식이 틀렸나 좌표계가
틀렸나"를 못 가리기 때문이다.

### 겹1 — 수식 이식 (Gazebo 불필요, macOS 에서도 됨)

```
./gz_aero/verify.sh
```

표 생성 → 기준값 생성 → C++ 빌드 → 대조까지 한 번에. `146/146 통과` 가 나와야 한다.

- **test A** (격자점 위): C++ 코어가 `control/dynamics.py` 의
  `_body_aerodynamics()` 와 같은 값을 내는가. 상대오차 1e-9 요구.
- **test B** (격자칸 중점 전수): 표 격자가 충분히 촘촘한가. 힘 오차로 판정.

CMake 로도 같은 것을 돌릴 수 있다:

```
cmake -S gz_aero -B gz_aero/build && cmake --build gz_aero/build -j
ctest --test-dir gz_aero/build --output-on-failure
```

### 겹2 — 좌표 변환·적용점 (Gazebo 필요)

먼저 **검증 도구 자체**를 태운다 (Gazebo 없이):

```
python3 gz_aero/tools/selftest_check_gz.py
```

이걸 안 하면 Ubuntu 에서 실패했을 때 "플러그인이 틀렸나 대조 스크립트가
틀렸나"를 또 가려야 한다. 특히 겹2b 는 **틀린 구현을 일부러 넣어** 검사가
그걸 잡는지 확인한다 — 안 잡으면 통과해도 의미가 없다.

Ubuntu 에서:

```
python3 gz_aero/tools/gen_gz_test_world.py
mkdir -p /tmp/fast_drone_aero
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/gz_aero/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -s -r --iterations 2000 gz_aero/test/frame_check.world
python3 gz_aero/tools/check_gz_frames.py --source sized
gz sim -s -r --iterations 400 gz_aero/test/accel_check.world
python3 gz_aero/tools/check_gz_accel.py
```

`check_gz_frames.py` 는 단계 A~F 로 쪼개 잰다. 실패하면 어느 단계인지 나온다:
A 입력 변환 / B 각도·동압 / C 표 보간 / D 힘 조립 / E 출력 변환 / F 물리 대조.

### 공력 계수를 CSV 로 갈아끼우기

`gz_aero/data/aero_*.csv` 를 같은 형식으로 덮어쓰면 **코드 수정 없이** 반영된다.
그게 이 설계의 목적이다. 형식과 필수 조건은 `gz_aero/USAGE.md` §3.
갈아끼운 뒤에는 반드시 `./gz_aero/verify.sh` 를 다시 돌린다.

---

## 5. SITL (`ros2_ws/` + `scripts/`)

Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic + PX4 가 필요하다 (`SETUP.md`).

```
./scripts/sitl_run.sh
./scripts/sitl_run.sh -c lqr -z 10 -d 20
./scripts/sitl_run.sh -c hybrid --headless
./scripts/sitl_run.sh --build
```

DDS agent → PX4 SITL → offboard 노드를 **순서 기동**하고(각 단계 준비 완료를
폴링), 로그를 `results/sitl_runs/<타임스탬프>/` 에 모으고, 종료 시 역순으로 정리한다.
재실행 안전(이전 stale 프로세스와 8888 포트를 먼저 정리).

플래그: `-c` 제어기 / `-z` 고도 / `-x` 전진속도 / `-g` 자세게인 스케일 /
`-d` 시간 / `--headless` / `--build`

> ⚠ `control/vehicle_params.py` 는 `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers/`
> 에도 **동일 사본**이 있다. 질량·관성을 바꾸면 **두 파일 다** 고쳐야
> 파이썬 시뮬과 SITL 이 같은 기체를 난다. `scripts/sync_controllers.sh` 참고.

---

## 6. 무엇이 검증됐고 무엇이 아직인가

정직하게 적는다. 통과했다고 다 믿을 수 있는 게 아니다.

| 항목 | 상태 |
|---|---|
| 6-DOF 플랜트 | ✅ 단위 테스트 통과 |
| 제어기 6종 비교·폴백 | ✅ 시뮬에서 검증, SITL 호버 성공 |
| SafetyGuard | ✅ 단위 테스트 5종 |
| 중량 수렴(WGHT) | ✅ 단위 테스트. 단 `Ŝ` 추정기 편향은 미해결 |
| 공력 C++ 코어 ↔ 파이썬 | ✅ 겹1 146/146, 격자점 위 상대오차 다수 정확히 0 |
| 겹2 검증 **도구** | ✅ 자체 검증 완료 (판별력 200:1) |
| 겹2 **실측** (좌표 변환·적용점) | ❌ **미실행** — Gazebo 필요 |
| gz-sim API 실제 대조 | ❌ **미확인** — macOS 는 스텁 헤더 문법검사만 |
| 고속(83 m/s) SITL 비행 | ❌ 미도달. 저속 전진까지만 확인 |

### ⚠ 기체 구성이 2026-08-27 에 바뀌었다

**테일시터로 확정**했다. `HANDOFF.md` 의 2026-07-11 절에 있는
"기체타입 확정 = 멀티로터 … 재론 말 것" 은 이 결정으로 **대체됐다.**

그래서 §2 의 제어 시뮬 결과들은 **멀티로터 기체(8 kg 플레이스홀더) 기준**이고,
테일시터로 넘어가면 `controller.py` 할당행렬 · INDI G · `trim.py` ·
`vehicle_params.py` · PX4 에어프레임이 전부 재작업 대상이다.
근거와 수치는 `HANDOFF.md` 의 "공력 플러그인 세션 (2026-08-27)" 절에 있다.

공력 플러그인(`gz_aero/`)은 **기체 중립**이라 이 변경의 영향을 받지 않는다
(CSV 를 갈아끼우면 된다).

---

## 7. 저장소 구조

```
control/            제어 연구 본체 (플랜트·제어기·추정기·시뮬)
  dynamics.py         6-DOF 플랜트 (CasADi)
  vehicle_params.py   기체 파라미터 ★ 플레이스홀더 값 포함
  controller.py       PID/LQR/INDI/ScheduledLQR
  nmpc.py             NMPC (IPOPT)
  vnmpc_acados.py     VirtualNMPC acados 이식
  hybrid_comparison.py ProperHybrid (확정 제어기)
  fallback_controller.py  Hybrid + LQR 폴백
  trim.py             트림 탐색
  mission_sim.py      통합 미션 (메인 진입점)
  test_*.py           단위 테스트
sizing/             중량 산정 (WGHT 모듈)
gz_aero/            Gazebo 공력 플러그인 + 검증 (DESIGN.md / USAGE.md)
ros2_ws/            ROS2 패키지 (offboard 노드, SITL)
scripts/            환경 구축·SITL 런처
results/            산출물 (플롯·리포트·벤치 로그)
legacy/             판정 완료된 일회성 실험
model.sdf           Gazebo 모델 (로터 플러그인만. 동체는 PX4 트리에 있음)
```

## 8. 컨벤션

- 실행은 **저장소 루트에서 모듈 형태로**: `python3 -m control.<모듈>`
- 산출물은 `results/` (사이징은 `sizing/results/`)
- 플롯 라벨은 **영어** — matplotlib 기본 폰트가 한글을 못 그린다
- 물리 상수를 새로 정의하지 말고 `control/vehicle_params.py` 를 참조할 것
- 결과 숫자가 극단적이면 보고 전에 **먼저 버그를 의심**한다
- 평가에는 항상 `|ω|` 를 포함한다. RMSE 만으로는 텀블을 못 잡는다
  (실기 실패 판정: `|ω| > 35 rad/s` 또는 `|ω| > 25` 가 200 ms 이상 지속)
