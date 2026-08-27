# gz_aero 사용법

설계 근거와 좌표계 계약은 `DESIGN.md` 에 있다. 이 문서는 **쓰는 법**만 적는다.

---

## 1. 빌드

### macOS (검증만, Gazebo 불필요)

```
./gz_aero/verify.sh
```

표 생성 → 기준값 생성 → C++ 빌드 → 대조까지 한 번에 한다. gz-sim 이 없으면
플러그인은 건너뛰고 검증 테스트만 빌드한다.

### Ubuntu 24.04 + Gazebo Harmonic

```
sudo apt install -y libgz-sim8-dev libgz-plugin2-dev
cd ~/fast-drone
cmake -S gz_aero -B gz_aero/build
cmake --build gz_aero/build -j$(nproc)
ctest --test-dir gz_aero/build --output-on-failure
```

`gz_aero/build/libFastDroneAero.so` 가 나오면 성공이다.

Gazebo 가 찾게 하려면:

```
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/fast-drone/gz_aero/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

---

## 2. 모델에 붙이기

```xml
<model name="fast_missile">
  <link name="base_link"> ... </link>

  <plugin filename="FastDroneAero" name="fast_drone::AeroPlugin">
    <link_name>base_link</link_name>
    <csv_file>aero_sized.csv</csv_file>
    <nose>1 0 0</nose>
    <right>0 -1 0</right>
    <rho>1.225</rho>
  </plugin>
</model>
```

### 파라미터 전체

| 태그 | 기본값 | 뜻 |
|---|---|---|
| `link_name` | `base_link` | 공력을 걸 링크 |
| `csv_file` | (필수) | 계수 표. 상대경로는 **SDF 파일 기준** |
| `nose` | `1 0 0` | 동체 +x(기수)를 링크 성분으로 |
| `right` | `0 -1 0` | 동체 +y(우현)를 링크 성분으로 |
| `s_ref` | CSV 값 | 기준면적 [m²] |
| `d_ref` | CSV 값 | 기준길이 [m] |
| `rho` | CSV `rho_ref` 또는 1.225 | 대기밀도 [kg/m³] |
| `wind` | `0 0 0` | 바람 [m/s], **월드 ENU** |
| `debug_csv` | (없음) | 매 스텝 진단 기록 파일 |
| `debug_csv_every` | `1` | 몇 스텝마다 기록할지 |
| `test_velocity_world` | (없음) | ⚠ 검증 전용 — 실제 속도 대신 이 값 사용 |
| `test_omega_world` | (없음) | ⚠ 검증 전용 |

`s_ref` / `d_ref` 를 SDF 에 적으면 CSV 의 값과 **0.1% 넘게 다를 때 에러**를
내고 힘을 하나도 걸지 않는다. 한쪽만 바꾸면 공력 전체가 그 비율로 조용히
스케일되기 때문이다.

### 세 가지 전제

1. **`<inertial><pose>` 가 진짜 무게중심이어야 한다.** 표의 `x_cp` 가 CG 기준이라,
   여기가 틀리면 정적 모멘트가 통째로 어긋난다.
2. **`nose`/`right` 는 직교해야 한다.** 아니면 Configure 가 에러를 내고 죽는다.
   세 번째 축은 플러그인이 `nose × right` 로 유도한다 (사람이 적으면 틀린다).
3. **중력·로터 추력은 이 플러그인이 안 건다.** Gazebo 와 모터 플러그인 몫이다.

### 테일시터로 쓸 때

추력축이 장축이므로, PX4 멀티콥터 믹서가 가정하는 "추력 = 링크 +z" 에 맞추면
기수가 링크 +z 가 된다. 그러면 **C++ 을 고치지 않고** 이것만 바꾸면 된다:

```xml
<nose>0 0 1</nose>
<right>0 -1 0</right>
```

---

## 3. 공력 담당이 준 CSV 로 갈아끼우기

`gz_aero/data/aero_*.csv` 를 같은 형식으로 덮어쓰면 끝이다. **코드 수정 없음.**

필수 조건:
- 첫 부분에 `# schema: axisym_v1` 이 있어야 한다 (축대칭 v1 표라는 선언)
- `# S_ref:` `# d_ref:` 메타데이터
- 헤더 `V_mps,alpha_rad,C_A,C_N,x_cp,C_lp,C_mq` (열 **순서는 바뀌어도 됨** —
  이름으로 찾는다)
- 정규격자: V 바깥 / alpha 안쪽, 둘 다 증가. 간격은 비균일이어도 된다
- `alpha` 는 `[0, π]` 만. 음의 받음각은 축대칭성으로 유도한다

Excel 이 붙이는 BOM 과 CRLF 는 플러그인이 걸러낸다.

갈아끼운 뒤에는 반드시:

```
./gz_aero/verify.sh
```

새 표의 격자가 성기면 test B 가 힘 오차로 알려준다.

---

## 4. 겹2 검증 (좌표 변환) — Ubuntu

```
python3 gz_aero/tools/gen_gz_test_world.py
mkdir -p /tmp/fast_drone_aero
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/gz_aero/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -s -r --iterations 2000 gz_aero/test/frame_check.world
python3 gz_aero/tools/check_gz_frames.py --source sized
```

중력 0 · 로터 없음 · 월드 속도 주입 상태에서 프로브 6개를 텀블링시키고,
로그에 찍힌 자세마다 변환을 다시 계산해 대조한다. 단계별로 나눠 잰다:

| 단계 | 무엇을 잰다 | 실패하면 |
|---|---|---|
| A | 월드 → 링크 → 동체 | 입력 변환. `nose`/`right` 또는 `RotateVectorReverse` |
| B | V, α, q̄ 계산 | 코어의 각도 계산 |
| C | 표 보간 | 보간기 또는 CSV 파싱 |
| D | 힘·모멘트 조립 | 코어의 수식 조립 |
| E | 동체 → 링크 → 월드 | 출력 변환 |
| F | `control/dynamics.py` 와 물리 대조 | 격자가 성긴 것 (A~E 가 통과했다면) |

A~E 는 기계정밀도로 맞아야 한다. F 만 표 보간오차를 허용한다.

---

## 5. 겹2b 검증 (적용점) — Ubuntu

```
gz sim -s -r --iterations 400 gz_aero/test/accel_check.world
python3 gz_aero/tools/check_gz_accel.py
```

**무엇이 걸려 있나.** 플러그인은 힘을 `AddWorldForce`(무게중심)로, 모멘트를
`AddWorldWrench(0, M)`(순수 우력)으로 건다. gz-sim 의 `ExternalWorldWrenchCmd` 가
힘을 **링크 원점**에 걸기 때문인데, 이건 소스를 읽고 내린 판단이지 실측이 아니다.
틀렸다면 실제 토크에 `r_cm × F` 가 여분으로 붙는다.

**어떻게 가르나.** `accel_check.world` 의 프로브는 **무게중심을 링크 원점에서
일부러 떼어 놨다.** 그래야 두 가설이 서로 다른 각가속도를 만든다:

| | 각운동 방정식 |
|---|---|
| 옳다면 | `J ω̇ + ω × Jω = M_L` |
| 틀렸다면 | `J ω̇ + ω × Jω = M_L + (−r_cm) × F_L` |

무게중심을 원점에 두면 `r_cm × F = 0` 이라 **두 가설이 구분이 안 된다.**
그래서 프로브를 따로 뒀다.

판정은 "절대오차가 작다" 가 아니라 **"틀린 가설보다 옳은 가설에 훨씬 가깝다"** 로
한다. 수치미분 오차가 얼마든 이 비교는 성립한다.

---

## 6. 검증 도구 자체 검증 (Gazebo 불필요)

```
python3 gz_aero/tools/selftest_check_gz.py
```

Ubuntu 에서 실패했을 때 "플러그인이 틀렸나 대조 스크립트가 틀렸나" 를 또
가리는 왕복이 아까워서, 도구를 먼저 태워 둔다.

- **겹2a**: 회전을 scipy 로 독립 생성해 손으로 쓴 `quat_to_R` 과 대조 →
  2e-16 일치 (쿼터니언 scalar-last 규약 확인)
- **겹2b**: 강체를 두 방식으로 굴린다 — `correct`(무게중심)와
  `at_origin`(원점, 흔한 실수). 검사가 앞은 통과시키고 뒤는 잡아내야 한다.
  **둘 다 통과하면 검사에 판별력이 없다는 뜻**이고, 그러면 Ubuntu 에서
  통과해도 아무 의미가 없다.

  측정된 판별력 (2026-08-27): 옳은가설 잔차 0.97 vs 틀린가설 205.3 rad/s²
  → 200 : 1. 수치미분 오차와 무관하게 구분된다.

---

## 7. 아직 안 한 것

- **`fast_missile_base` 정합.** 실제 기체 모델이 저장소 밖(Ubuntu PX4 트리)에
  있어 1단계(동체 링크 + 사이징 관성)를 못 했다. 겹2 는 최소 링크를 직접
  정의해 우회했다.
- **α > 90° 역류 계수.** `DESIGN.md` §7 참고. 표로 고칠 수 있으나 CFD 값이 필요하다.
- **gz API 실제 대조.** macOS 에서는 스텁 헤더로 문법만 확인했다.
