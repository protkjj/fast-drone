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
| 6 | `python3 -m control.mission_sim` | 통합 미션 65 s × 제어기 3종 | 수 분 📄 | matplotlib |
| 7 | `python3 -m control.gust_comparison` | 돌풍 외란 응답 | 수 분 📄 | — |
| 8 | `python3 -m control.ekf_comparison` | 센서 노이즈 하 제어기 순위 | 수 분 📄 | — |
| 9 | `python3 -m control.final_config_mission` | 확정 구성(acados) 통합 검증 | 수 분 📄 | **acados** |
| 10 | `./scripts/sitl_run.sh` | PX4 SITL 이륙·호버 | — | **Ubuntu+PX4** |

✅ = 2026-08-27 macOS 에서 실측한 시간  📄 = 코드 주석에 적힌 값, 이 세션에서 미실측

> 1~5 는 **추가 의존성 없이 8초 안에 다 돈다** (합계 실측 8.1 s).
> 저장소를 처음 받았으면 여기까지 먼저 하면 된다.

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
python3 -m sizing.gate2_shat_bias
python3 -m sizing.measure_noise
```

중량 수렴(WGHT) 알고리즘. 발산 설계점을 제대로 발산으로 분류하는지 포함.
자세한 배경은 `sizing/README.md`.

> 팀 사이징 저장소(`rocket-drone`)는 **별개 저장소**다. 이 폴더는 그쪽에 낼
> WGHT 담당분의 로컬 작업 공간이다.

---

## 4. SITL (`ros2_ws/` + `scripts/`)

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

> ⚠ **Gazebo 기본 물리는 중력 + 로터 추력만 계산한다.** 축대칭 동체의 항력·법선력·
> 정적안정·감쇠는 아무도 안 넣어준다. 즉 지금 SITL 은 **공기 없는 우주에서 나는
> 쿼드**다. 저속 호버·전진 검증까지는 쓸 수 있지만 고속 결과는 믿으면 안 된다.
> 공력 플러그인은 `bulnabi` 브랜치에서 개발 중이다.

> ⚠ `control/vehicle_params.py` 는 `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers/`
> 에도 **동일 사본**이 있다. 질량·관성을 바꾸면 **두 파일 다** 고쳐야
> 파이썬 시뮬과 SITL 이 같은 기체를 난다. `scripts/sync_controllers.sh` 참고.

---

## 5. 기체 — 두 형상이 병행 중이다

헷갈리기 쉬운 지점이라 못박아 둔다. **둘 다 축대칭이다.**
축대칭(*형상*)과 테일시터/멀티로터(*추력축*)는 서로 다른 축의 이야기이고,
배타적이지 않다.

| | 축대칭 멀티로터 | 테일시터 |
|---|---|---|
| 어디 | **이 저장소** (`control/`) | 팀 저장소 `rocket-drone` (사이징) |
| 질량 | 8 kg (플레이스홀더) | 1.661 kg (사이징 확정) |
| 동체 | 1.0 m × 0.15 m, **핀 없음** | 0.72 m × 0.09 m, **핀 4매** |
| 추력축 | 장축에 **수직** | 장축 **방향** |
| 300 km/h | ✅ 트림 성립 (잔차 1e-10, 최고 85 m/s) | ✅ 사이징 g1~g9 전부 통과 |
| 제어기 | ✅ 6종 완성, SITL 호버 성공 | 없음 |

### ⚠ 팀 사이징 값을 이 저장소에 그대로 끼워 넣으면 안 된다

팀의 1.661 kg 은 **테일시터 트림**(`T·sinθ + L = W`, `T·cosθ = D`, `α = θ`)으로
성립한 값이다. 그 항등식은 추력이 장축에 있어야만 성립한다 (멀티로터면 `α = θ − 90°`).

같은 값을 멀티로터 구성으로 계산하면 300 km/h 가 **물리적으로 불가능**하다
(`control/trim.py` 실측):

| 속도 | 이 저장소 8 kg | 사이징 1.661 kg 을 멀티로터로 |
|---|---|---|
| 60 m/s | θ −3.5°, T/W 1.26 ✅ | θ −22.4°, T/W **12.1** |
| 83.3 m/s | θ −7.6°, T/W 2.02 ✅ | **트림 수렴 실패** |

핀을 떼도 최대 트림속도는 50 m/s (180 km/h)다. 원인은 무게가 1/4.8 인데 절대
항력은 비슷해서(CD0 0.12 → 0.44, 기저항력 0.186 포함) D/W 가 0.115 → 0.735 로
뛰기 때문이다. 36° 이상 기울여야 하고, 그 받음각의 크로스플로 항력이 무게의
5~10 배가 된다.

> 참고: 이 형상의 멀티로터가 300 km/h 를 내려면 **6 kg 이상**이 필요하다.
> `vehicle_params.py` 의 8 kg 은 임의값이 아니라 그 하한 근처의 여유값이었다.

---

## 6. 무엇이 검증됐고 무엇이 아직인가

정직하게 적는다. 통과했다고 다 믿을 수 있는 게 아니다.

| 항목 | 상태 |
|---|---|
| 6-DOF 플랜트 | ✅ 단위 테스트 통과 |
| 제어기 6종 비교·폴백 | ✅ 시뮬에서 검증, SITL 호버 성공 |
| SafetyGuard | ✅ 단위 테스트 5종 |
| 중량 수렴(WGHT) | ✅ 단위 테스트. 단 `Ŝ` 추정기 편향은 미해결 |
| SITL 공력 | ❌ **없음** — Gazebo 기본 물리에 동체 공력이 안 들어간다 (`bulnabi` 브랜치) |
| 고속(83 m/s) SITL 비행 | ❌ 미도달. 저속 전진까지만 확인 |

### 알려진 모델 결함

`control/vehicle_params.py` 의 `C_dc = 1.2` 는 **스케일이 빠져 있다.**
주석은 "원통 표준"인데 그 슬롯이 요구하는 건 `η·Cd_c·(A_plan/S_ref)` 라,
이 형상 기준 **≈ 5.6** 이어야 한다 (약 4.7배 과소).

영향: α=7.5°(순항)에선 C_N 의 2.6% → 12% 로 작지만,
**α=30°(고속 틸트)에선 C_N 2.90 → 4.01 (+38%)**. 즉 파이썬 시뮬이 고속 구간을
낙관적으로 본다. 고치면 기존 시뮬 결과가 전부 바뀌므로 **판단이 필요하다.**

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
ros2_ws/            ROS2 패키지 (offboard 노드, SITL)
scripts/            환경 구축·SITL 런처
results/            산출물 (플롯·리포트·벤치 로그)
legacy/             판정 완료된 일회성 실험
model.sdf           Gazebo 모델 (로터 플러그인만. 동체는 PX4 트리에 있음)
```

## 8. 브랜치

| 브랜치 | 내용 |
|---|---|
| `main` | 통합본 |
| `control` | 축대칭 멀티로터 제어 연구 (이 브랜치) |
| `bulnabi` | `control` + Gazebo 공력 플러그인. 두 기체(멀티로터·테일시터) 병행 지원 |

## 9. 컨벤션

- 실행은 **저장소 루트에서 모듈 형태로**: `python3 -m control.<모듈>`
- 산출물은 `results/` (사이징은 `sizing/results/`)
- 플롯 라벨은 **영어** — matplotlib 기본 폰트가 한글을 못 그린다
- 물리 상수를 새로 정의하지 말고 `control/vehicle_params.py` 를 참조할 것
- 결과 숫자가 극단적이면 보고 전에 **먼저 버그를 의심**한다
- 평가에는 항상 `|ω|` 를 포함한다. RMSE 만으로는 텀블을 못 잡는다
  (실기 실패 판정: `|ω| > 35 rad/s` 또는 `|ω| > 25` 가 200 ms 이상 지속)
