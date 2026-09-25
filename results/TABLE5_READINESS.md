# 표5 비교군 실행 가능성 점검 — 논문 기체(선정 프로파일, x축)

`python3 -m pytest` 로는 안 잡히는 문제를 찾기 위해, 표5의 모든 비교군을
**논문 기체**에서 폐루프로 돌려 봤다. 트림 상태에서 출발해 6초 유지
(목표 z=50 m, vx=8 m/s). 트림에서 출발했으니 정상 제어기는 아무것도 하지
않아야 한다 — 어긋나면 그 제어기가 트림을 모른다는 뜻이다.

| 비교군 | z 끝 | vx 끝 | \|ω\|max | 판정 | 원인 |
|---|---|---|---|---|---|
| CPID | **33.71** | **66.54** | 0.56 | ❌ | `thrust_axis` 미지원 |
| GSLQR | 50.00 | 8.00 | 0.00 | ✅ | |
| M17 (기본) | 50.59 | **11.99** | 0.48 | ❌ | legacy 입력가중 62,500배 과대 |
| M17 (`u_ref`=트림) | 50.00 | 8.00 | 0.00 | ✅ | 우회 가능 |
| F13 | 50.00 | 7.81 | 0.31 | ✅ | |
| V13 | 50.00 | 8.00 | 0.01 | ✅ | |
| GINDI | 48.98 | 7.19 | 9.79 | ❌ | 게인 미튜닝 |

핵심 비교군 4개(CPID·M17·F13·V13) 중 **CPID가 아예 못 돌고 M17은 기본
설정으로 틀린다.** 이 상태로 표5를 채우면 안 된다.

---

## ① CPID — `thrust_axis` 미지원 (핵심 비교군)

`control/controller.py`의 `CascadedPID`에 `thrust_axis` 문자열이 **0건**이다.
`_force_to_attitude`가 z축 전용으로 하드코딩돼 있다.

```python
b3_des = -F_des / F_norm          # 동체 3번째 축을 -F 에 정렬 (z-down 추력 전제)
cos_tilt = -b3_des[2]             # 틸트 제한도 z 기준
...                               # 한계에서 [0,0,-1] (z-down 기체의 '수직')
R_des = np.column_stack([b1, b2, b3_des])
```

로켓형은 추력이 동체 **+x**라서 정렬할 축이 1번째다. 그 결과 트림에서
피치 모멘트를 **아예 만들지 않는다**:

| | 트림 n | CPID 출력 |
|---|---|---|
| selected(x) V=8 | `[1433, 1433, 1612, 1612]` | `[1513, 1513, 1513, 1513]` |

4개가 같다 = 모멘트 0 = 호버 자세를 요구. 그래서 기체가 자세를 못 잡고
속도가 목표 8 m/s에서 **66.5 m/s로 폭주**한다.

`ScheduledPID(CascadedPID)`도 상속으로 같은 결함이다.

고치는 방향은 `control/gindi.py`의 `GeometricGuidance._desired_attitude`가
이미 하고 있는 것과 같다 — 추력축에 따라 정렬할 열과 헤딩 기준 축을 바꾸고,
틸트 제한도 추력축 기준으로 다시 정의한다. GINDI의 기하 외부 루프와 CPID의
외부 루프는 논문상 **같은 것**이므로(§4/§5.3, 306행 — 갈리는 지점은 정적
배분이냐 INDI냐뿐이다) 공유하는 게 맞다.

## ② M17 — legacy 입력가중이 호버로 끌어당긴다

`control/nmpc.py:84`가 `self.u_ref = np.full(4, n_hov)`(호버 회전수)이고
`R = 1e-4·I`로 그쪽 편차를 벌한다. 논문/research도 νh는 호버 회전수지만
**`Dν = max_n`으로 무차원화**한다(식15) — 실효 가중이 `0.02/3537² = 1.6e-9`다.
control의 `1e-4`는 그보다 **62,500배 강하다.**

왜 여태 안 보였나 — z축 기체는 트림 회전수가 호버와 거의 같기 때문이다:

| | 트림 n | `u_ref`=호버(기본) | `u_ref`=트림 |
|---|---|---|---|
| vehicle(z) V=12 | `[572, 572, 572.1, 572.1]` 대칭 | vx 12.00 ✅ | vx 12.00 ✅ |
| selected(x) V=8 | `[1433, 1433, 1612, 1612]` **비대칭** | vx **11.99** ❌ | vx **8.00** ✅ |

로켓형은 전후 쌍이 크게 갈려서 드러난다. 즉 **논문 기체로 옮기면서 생긴
노출**이지 새 회귀가 아니다.

`u_ref=트림 회전수`를 넘기면 우회된다(vx 8.00, |ω| 0.00). 그러나 그건 우회일
뿐이다 — 운전점이 참조에서 벗어나는 순간(램프·돌풍·섭동) 62,500배 과대한
가중이 다시 문다. 근본 해결은 식(15)의 `Dν` 정규화를 넣는 것, 즉
`cost_spec='paper'` 정렬이다.

## ③ GINDI — 게인 미튜닝

`results/GINDI_STATUS.md` 참조. 구조는 검증됐고 원인(축별 실효게인 490 대 107)도
규명됐다. §5.3 튜닝 필요.

---

## 재현

```bash
cd /Users/kj/Desktop/dynamic/fast_drone-control-paper
python3 - <<'PY'
import numpy as np
from control.dynamics import AxialDronePlant
from control.vehicle_params import load_selected_params
from control.trim import find_trim
P = load_selected_params(); V = 8.0
tr = find_trim(P, V, quiet=True)
x0 = np.zeros(17); x0[2] = 50.; x0[3:6] = [V, 0, 0]
x0[6:10] = tr['state'][6:10]; x0[13:17] = tr['state'][13:17]
from control.controller import CascadedPID
ts, X, U = AxialDronePlant(P, dt=0.001).simulate(
    x0, CascadedPID(P, v_ref=[V,0,0], z_ref=50.), 6.0)
print(f"CPID: z {X[-1,2]:.2f} (목표 50), vx {X[-1,3]:.2f} (목표 8)")
PY
```

트림 출발 유지 시험은 단위테스트보다 판별력이 높다 — 제어기가 자기 기체의
평형을 아는지 한 줄로 드러난다. 표5·표6을 돌리기 전에 모든 비교군에 대해
이 시험을 통과시킬 것.
