# 전체 코드베이스 코드리뷰 — 2026-08-04

> 이 문서는 `/code-review`(diff 기반)가 아니라 **활성 코드 전문(全文) 리뷰** 결과입니다.
> 자동 생성 + 사람(세션) 검증. 재생성 방법은 문서 끝 「재현」 절 참조.

## 1. 범위와 방법

| 항목 | 내용 |
|---|---|
| 대상 | 활성 Python 코드 약 6,900줄 (`legacy/` 3,600줄 **제외**) |
| 제외 | `legacy/` (CLAUDE.md상 "안 쓰는 코드"), 산출물(`results/*.png|txt`) |
| 방법 | 9개 서브시스템 병렬 정밀 리뷰 → 지적별 적대적 검증(반박 시도) → 종합 |
| 규모 | 에이전트 33개, 툴 호출 802회, 소요 80분 |
| 결과 | 제기 78건 → **검증 통과 63건 / 반박 기각 15건** → 중복 병합 후 **18개 그룹** |

검증자에게는 "기본 입장은 회의적, 확신이 없으면 기각 쪽으로 기울일 것"과
"부호 주장은 `dynamics.py`의 실제 수식 라인과 대조해서만 인정"을 지시했습니다.
CLAUDE.md의 「독스트링과 구현이 달라 테스트 부호가 뒤집힌 전력」 경고를 반영한 것입니다.

## 2. ⚠️ 코드 시점 주의 (중요)

리뷰는 `b784ab7` 시점에 시작했으나, **실행 중(01:32~02:52)에 워킹트리가 변경되었습니다.**

```
01:53  9566b83  HANDOFF/TODO 개인 문서 gitignore 처리
02:12  d5688e5  계산 성능 벤치마크 4종 + 결과
02:12  1e6d085  폴백 NMPC 미수렴 트리거 + VirtualNMPC cold-start w0 픽스 + 테스트
02:12  5f2f70d  sync_controllers.sh macOS 호환 (BSD sed → perl)
02:55  (미커밋) fallback_controller.py +30/-1, acados 작업물 untracked
```

따라서 **리뷰어와 검증자가 서로 다른 버전을 읽은 항목이 있습니다.** 확인된 영향:

| 항목 | 실제로 일어난 일 |
|---|---|
| `ros2-px4#6` (`sed -i -E`가 macOS에서 죽음) | 리뷰어는 구버전을 읽고 **정확히 지적**했으나, 검증자는 02:12에 커밋된 perl 버전을 읽고 기각. **kj가 이미 고친 문제** |
| `cross-cutting#6` (HybridWithFallback 미배선) | 검증자가 02:12에 추가된 `test_fallback.py`를 발견해 기각 |
| `nmpc-indi#1` (VirtualNMPC w0 쿼터니언 시딩) | 02:12 커밋으로 **이미 수정 완료** |
| `nmpc-indi#2` (NMPC NaN latching) | 검증자가 신 HEAD로 실험해 재현 실패 → 기각 |

**미커밋 상태인 `fallback_controller.py`의 v_ref/z_ref 패스스루 추가분은 리뷰 대상이 아닙니다**
(kj가 "커밋한 부분까지만"으로 범위를 지정).

---

## 3. 세션이 직접 재현 검증한 항목 (에이전트 주장 ≠ 확정)

에이전트 주장을 그대로 싣지 않고, **실기 손실로 직결되는 2건은 세션에서 코드를 직접 열고 수치 재현**했습니다.
재현 스크립트: `results/verify_review_claims.py`

### 3.1 Issue 1 — 쿼터니언 부호 뒤집힘 ✅ **사실 확인**

먼저 규약부터 확정했습니다. CLAUDE.md의 "호버 쿼터니언 `[1,0,0,0]`"이 스칼라-first로 읽히지만,
`dynamics.py:16`과 `_quat_to_rotmat`(`qx,qy,qz,qw = q[0],q[1],q[2],q[3]`)를 대조한 결과
**이 프로젝트는 스칼라-last `[x,y,z,w]`** 이고 `[1,0,0,0]`은 **x=1, w=0** 즉 x축 180° 회전입니다
(동체 z-down ↔ 관성 z-up). `controller.py`의 `_quat_mult` 독스트링·켤레·벡터부 추출 모두 이 규약과 정합합니다.
즉 리뷰어가 규약을 혼동한 것이 아니었습니다.

그 위에서 4개 검사:

| 검사 | 결과 |
|---|---|
| 트림 `q[3]`(스칼라부) | V = 0 / 10 / 30 / 50 / 85 m/s **전부 정확히 0.000e+00** |
| 롤 +10° vs −10° (`q_w>0` 정규화 통과 후) | 둘 다 `dphi_x = −0.17431` — **부호 정보 완전 소실** |
| 최단경로 보정 1줄 추가 후 | +10° → `+0.17431`, −10° → `−0.17431` — 정상 복구 |
| 파이썬 시뮬 경로 영향 | ±60°/3축 전수 스캔에서 `dq[3]` 최솟값 **0.866 > 0** → 보정 무작동 |

```
 V (m/s) |       q_trim (scalar-last [x,y,z,w])       |     w = q[3]
------------------------------------------------------------------------
     0.0 | [ 1.000000  0.000000  0.000000  0.000000] |    0.000e+00
    30.0 | [ 0.999972  0.000000 -0.007462  0.000000] |    0.000e+00
    85.0 | [ 0.997579  0.000000 -0.069549  0.000000] |    0.000e+00
```

**왜 정확히 0인가**: 공칭 자세가 180° 회전이라 순수 벡터 쿼터니언이고, 순항 피치가 붙어도
회전축이 기울 뿐 여전히 180° 회전이라 스칼라부는 0을 유지합니다.
즉 `offboard_node.py:380` / `frame_utils.py:156`의 `if q_nwu[3] < 0: q_nwu = -q_nwu` 는
**이 기체의 정상 비행 자세 바로 위에 부호 경계를 놓습니다.**

**결론**: 지적 성립. 단 **ROS2/PX4 경로 전용**이며 파이썬 시뮬 수치는 바뀌지 않아야 정상입니다.

### 3.2 Issue 3(d) — INDI의 NaN 영구 고착 ✅ **사실 확인 (현재 HEAD 기준)**

`hybrid_comparison.py`가 리뷰 도중 변경되었으므로 **변경 후 코드로** 재확인했습니다.

```
step 2: omega=[nan  0.  0.]  filt=[    nan -0.0177 -0.0177] <-- NaN 1회 주입
step 3: omega=[0.1 0.1 0.1]  filt=[   nan 0.2828 0.2828]
...
step 8: omega=[0.1 0.1 0.1]  filt=[   nan 0.2429 0.2429]
→ 주입 6스텝 뒤에도 finite? False
```

- 자기참조 LPF(`hybrid_comparison.py:277-279`, `407-409`)라 **NaN 1회로 영구 고착**
- `np.linalg.solve(G, dv)`는 NaN 우변에 `LinAlgError`를 **던지지 않고** `[nan nan nan nan]` 반환
  → `except np.linalg.LinAlgError` 절의 `_fallback`이 **발동하지 않음**
- `np.clip(nan, 0, 1800) = nan` → 최종 클립도 통과
- 현재 파일의 `isfinite`/`isnan` 등장은 **1회뿐**(433행 발산 판정)

**결론**: 지적 성립. `except`만으로는 NaN을 못 잡는다는 점이 핵심입니다.

### 3.3 검증하지 않은 것

Issue 2, 4, 5 및 Issue 6~18은 **에이전트 검증만 거쳤고 세션이 직접 재현하지 않았습니다.**
특히 confidence=medium인 Issue 7(로터 각운동량 부호)은 근거가
"할당행렬과 148행의 정합성" 하나에만 기대고 있고, 결정적 근거인 Gazebo SDF의 로터 조인트 축은
이 저장소(macOS)에서 확인 불가입니다. **착수 전 Ubuntu에서 직접 확인하십시오.**

---


## 4. 실기 비행 전 필수 — 5개 그룹

실기체 손실로 직결되는 항목입니다. **전부 `ros2_ws/` 안에서 완결되며 파이썬 실험 재실행이 필요 없습니다.**

| # | 심각도 | 제목 | 신뢰도 |
|---|---|---|---|
| 1 | 🔴 CRITICAL | 쿼터니언 최단경로 보정 부재 + PX4 상태조립의 w>0 정규화 → LQR 계열 자세 오차 부호가 절반의 기울기 방향에서 뒤집힘 | high |
| 2 | 🟠 HIGH | SafetyGuard 폴백이 4로터 동일 명령이라 자세 제어 모멘트를 정확히 0으로 만든다 (+ hover_cmd 값 자체가 매직넘버) | high |
| 3 | 🟠 HIGH | 상태 유효성·신선도 검사가 하나도 없고, 우리 실패가 PX4 failsafe로 승격되지 않는다 (워치독은 죽은 코드) | high |
| 4 | 🟠 HIGH | 노드가 제어기 생애주기와 시간 단조성을 관리하지 않는다 — 재arm 시 t가 되감기고 reset()이 호출되지 않음 | high |
| 5 | 🟠 HIGH | heading 정렬이 제어기별 임시방편 + duck-typing 분기라 scheduled_lqr / indi 는 경고 없이 통과하고, v_ref 좌표계도 어긋난다 | high |

### Issue 1 — 🔴 CRITICAL · 신뢰도 high

**쿼터니언 최단경로 보정 부재 + PX4 상태조립의 w>0 정규화 → LQR 계열 자세 오차 부호가 절반의 기울기 방향에서 뒤집힘**

- 파일: `controller.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers/controller.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/frame_utils.py`
- 원 지적 ID: `classical-ctrl#1`, `ros2-px4#1`

**무엇이 잘못됐나**

서로 다른 두 리뷰어(classical-ctrl#1, ros2-px4#1)가 같은 버그의 양쪽 끝을 잡았습니다. 하나로 묶으면 이렇습니다.

(a) controller.py:291 `dq = self._quat_mult(q_trim_inv, q)` / `dphi = 2.0 * dq[0:3]` 에 최단경로 보정(`if dq[3] < 0: dq = -dq`)이 없습니다. ScheduledLQR._compute_error_state(controller.py:476)에도 같은 코드가 복제돼 있습니다. 단독으로는 '자세 오차 180° 초과'라는 희귀 조건입니다.

(b) 그런데 실기 경로가 그 조건을 상시로 만듭니다. offboard_node.py:379-381 `if q_nwu[3] < 0: q_nwu = -q_nwu` (frame_utils.py:155-157에 동일 코드)입니다. 제가 직접 확인한 결과 이 기체의 트림 쿼터니언은 **모든 속도에서 w 성분이 정확히 0**입니다:
  V=  0.0 q=[1, 0, 0, 0]        w=0.000e+00
  V= 30.0 q=[0.999972, 0, -0.007462, 0]  w=0.000e+00
  V= 85.0 q=[0.997579, 0, -0.069549, 0]  w=0.000e+00
즉 'w>0 정규화'의 경계가 이 기체의 정상 비행 자세 위에 정확히 놓여 있습니다. 롤 +10°면 w = -sin(5°) < 0 → 부호 반전 → dphi 부호 뒤집힘.

검증자 실측: 롤 +10°와 -10°가 **같은** dphi_x = -0.1743을 냅니다(롤 오차 부호 정보 완전 소실). 폐루프는 10.00° → 15.85° → 32.70° → 59.10° → 62.39°로 발산하고, 1줄 수정 후 두 방향 모두 10.00 → 4.12 → 2.45 → 1.51 → 0.04°로 완전 복구됩니다.

**왜 중요한가**

LQRController/ScheduledLQR이 롤 방향의 절반에서 정피드백이 됩니다. ScheduledLQR은 offboard_node.py:604의 선택지이자 MEMORY.md가 확정한 안전 폴백(HybridWithFallback → LQR)의 실행 주체이므로, 'Hybrid가 이상해서 LQR로 넘어간' 바로 그 순간에 반대 방향으로 회전시킵니다. HANDOFF.md:154-156의 미해결 '롤 진동 발산'과 인과가 정확히 겹칩니다 — 다만 같은 줄에 Hybrid도 발산한다고 적혀 있고 Hybrid는 Rotation.from_quat의 회전행렬만 써서 R(-q)=R(q)로 면역이므로, 이것이 롤 불안정 **전체**를 설명하지는 않습니다. CascadedPID/INDI/ProperHybrid도 같은 이유로 면역입니다.

**수정**

1) controller.py:291과 476 두 곳, _quat_mult 직후에 `if dq[3] < 0.0: dq = -dq` 삽입. 이것이 정석이며 부호 규약이 바뀌어도 안전합니다.
2) ros2_ws/.../controllers/controller.py 사본에도 동일 적용(또는 scripts/sync_controllers.sh 재실행 — 현재 perl 버전으로 정상 동작 확인됨).
3) offboard_node.py:379-381과 frame_utils.py:155-157의 `q_w>0` 정규화는 이 기체에서 의미가 없으므로(트림 w=0) 제거하거나 직전 샘플 연속성 기준(`if np.dot(q_new, q_prev) < 0: q_new = -q_new`)으로 교체.
4) 회귀: 파이썬 시뮬은 부호 반전 상태를 만들지 않으므로 test_plant.py / mission_sim.py 수치가 **바뀌지 않아야 정상**입니다. 바뀌면 다른 문제가 있는 것이니 멈추고 확인.

---

### Issue 2 — 🟠 HIGH · 신뢰도 high

**SafetyGuard 폴백이 4로터 동일 명령이라 자세 제어 모멘트를 정확히 0으로 만든다 (+ hover_cmd 값 자체가 매직넘버)**

- 파일: `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/safety.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`
- 원 지적 ID: `safety-fallback#1`, `safety-fallback#4`, `ros2-px4#9`, `safety-fallback#2`, `cross-cutting#4`

**무엇이 잘못됐나**

네 리뷰어가 따로 낸 지적(safety-fallback#1/#2/#4, ros2-px4#9, cross-cutting#4)이 전부 같은 뿌리입니다: 폴백 명령을 [총추력, 3축 모멘트]로 분해하지 않고 로터 명령 벡터에 직접 원소별 연산을 합니다.

(a) safety.py:196-204 틸트>70° FAILSAFE → `return self.hover_cmd.copy()`. hover_cmd는 `np.full(4, hover_rpm)`(62행)이라 4개가 같습니다. 검증자가 dynamics.py:203-226의 실제 할당행렬로 계산: A@T = [79.6262, 0, 0, 0] — Mx=My=Mz가 **정확히 0**입니다. 자세 이상을 감지해서 자세 제어를 포기하는 뒤집힌 로직입니다.

(b) safety.py:217 저고도 보호 `u = np.maximum(u, self.hover_cmd)`도 원소별입니다. hover_cmd=576 > 실제 n_hov=571.84이므로, 지면 근처에서 트림 부근의 작은 자세 보정(예 [570,574,570,574])은 4개 전부 하한에 걸려 [576,576,576,576]이 되어 **차동이 통째로 소거**됩니다. 검증자 정정대로 '절반이 잘린다'가 아니라 '작은 보정은 완전히 사라진다'가 맞습니다. 이 레이어는 Layer 3 `_rate_limit` 뒤에 있어 변화율 제한도 우회합니다.

(c) 그 hover_rpm 자체가 safety.py:59-61 `hover_rpm = n_max * 0.32`입니다. 물리적으로 옳은 값은 sqrt(mg/(4·k_T)) = 571.84로 **n_max와 무관**합니다. 지금은 1800×0.32=576으로 우연히 +0.7% 오차지만, launch에서 n_max=1500을 주면 480 → T=55.3 N < mg=78.5 N, mass 8.0→12.0이면 필요값 700.4인데 576을 냅니다. safety.py는 vehicle_params를 import조차 하지 않습니다(CLAUDE.md 규약 위반).

**왜 중요한가**

실기에서 이상 상황(틸트 초과, 연속 NaN 50회)에 진입하는 순간 안전장치가 자세 회복 수단을 스스로 제거합니다. x_cp=-0.10(CG 뒤)으로 정적 안정한 동체는 낙하하며 상대풍에 정렬해 더 깊이 기울어지므로 물리적으로 자기유지됩니다. 발동 조건 자체가 이미 심각한 이상이라 '단독으로 추락을 만드는 결함'은 아니지만, **회복 가능성을 확정적으로 0으로 만드는** 결함입니다. 형상팀 값 교체 후에는 (c) 때문에 폴백이 추력 부족까지 겹칩니다. '안전장치가 발동하면 떨어진다'는 실기에서 가장 나쁜 실패 모드입니다.

정직한 반대 증거: 검증자 확인상 (b)는 통상 이륙에서 거의 물리지 않습니다(상승 명령 평균 ~812 rad/s ≫ 576), 실제로 SITL에서 PID가 이 구간을 통과해 5 m 상승 + 30초 호버에 성공했습니다(HANDOFF.md:121-122). (a)도 정상 트림 틸트가 V=85에서 7.98°뿐이라 오발동 위험은 없습니다.

**수정**

1) `_check_state`를 총추력/차동 분해 기반으로 재작성: compute_allocation_matrix로 u→[T,Mx,My,Mz] 분해 → T만 호버 추력으로 치환(틸트) 또는 하한 적용(저고도) → 재할당해 Mx/My/Mz 보존.
2) hover_rpm은 offboard_node.py:120의 SafetyGuard 생성 시 `hover_rpm=np.sqrt(P['mass']*P['g']/(4*P['k_T']))`로 명시 전달하고, 미지정이면 예외를 던져 매직넘버 경로를 없앨 것. 계산값이 n_max를 넘으면 launch 단계에서 잡히도록.
3) 저고도 하한에 '이륙 완료(z>min_altitude 1회 통과)' 래치를 두어 향후 착륙/하강을 막지 않게. 또는 min_altitude를 0.2 m로.
4) Layer 4의 출력도 rate limiter를 다시 통과시키도록 레이어 순서 재배치(NaN→상태체크→클램프→변화율제한).

---

### Issue 3 — 🟠 HIGH · 신뢰도 high

**상태 유효성·신선도 검사가 하나도 없고, 우리 실패가 PX4 failsafe로 승격되지 않는다 (워치독은 죽은 코드)**

- 파일: `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/safety.py`, `hybrid_comparison.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers/hybrid_comparison.py`
- 원 지적 ID: `ros2-px4#5`, `safety-fallback#3`, `safety-fallback#9`, `nmpc-indi#3`

**무엇이 잘못됐나**

세 리뷰어(ros2-px4#5, safety-fallback#3/#9, nmpc-indi#3)가 같은 구멍의 다른 면을 봤습니다.

(a) offboard_node.py:180-192 `_odom_callback`이 `self._pos_ned = np.array(msg.position, dtype=float)` 등을 유한성 검사 없이 대입하고 `_state_valid`를 한 번만 True로 래치합니다. 파일 전체 grep에서 isfinite/isnan/last_odom/stale이 0건입니다. PX4 VehicleOdometry는 메시지 계약상 무효 필드에 NaN을 넣습니다.

(b) 오도메트리가 끊겨도 `_control_loop`는 얼어붙은 `_pos_ned/_vel_ned/_q_ned_sf/_omega_body`로 계속 제어량을 계산해 발행하고, 218행 `self._publish_offboard_mode()`가 **무조건** 실행되므로 PX4는 offboard 신호가 건강하다고 판단합니다. 즉 우리 실패가 PX4 실패로 승격되지 않습니다. (검증자 정정: 링크가 통째로 끊기면 PX4 타임아웃이 정상 발동하므로, 위험한 경우는 '수신만 멈추고 송신은 살아있는' 비대칭 장애 — 단일스레드 executor에서 VirtualNMPC 솔브가 콜백을 블로킹하거나 odom 구독만 죽는 경우입니다.)

(c) safety.py 독스트링 13행이 '보호 레이어 4: 워치독 → 제어기 응답 없으면 호버 폴백'을 광고하지만, `_watchdog_count`는 66/133/225행에서 `= 0` 대입만 되고 증가 코드가 없습니다. `watchdog_limit`(84행)은 읽히는 곳이 0곳입니다. `SafetyLevel.EMERGENCY`도 정의(32행)와 주석(80행) 외 사용처 0곳이고, 80행 주석은 'NaN 한계 초과 시 EMERGENCY'라는데 145행은 FAILSAFE만 세팅합니다.

(d) 확정 제어기에 영구 고착 경로가 있습니다. hybrid_comparison.py의 각가속도 추정 `self._omega_dot_filt = alpha*raw + (1-alpha)*self._omega_dot_filt` / `self._omega_prev = omega.copy()`는 자기참조라, omega에 NaN이 한 번 들어오면 **그 뒤 20스텝을 완전히 깨끗한 상태로 돌려도 계속 [nan]×4를 반환**합니다(검증자 실측). 그리고 `np.linalg.solve(G, dv)`는 NaN 우변에 LinAlgError를 던지지 않고 NaN을 그대로 돌려주므로 except 절의 _fallback이 발동하지 않고, np.clip(nan)=nan이라 최종 반환까지 통과합니다.

**왜 중요한가**

ActuatorMotors 직접 제어(PX4 자세 안정화 전부 off)를 하면서 표준적인 입력 검증이 하나도 없습니다. 단 한 샘플의 무효 오도메트리로 확정 제어기(ProperHybrid)가 영구 무력화되고, 유일한 우아한 저하가 70~85 m/s 순항 중 4로터 동일 호버 명령(= 위 Issue 2의 제로 모멘트)입니다. 워치독/EMERGENCY가 죽은 코드라는 사실은 그 자체보다 '후속 작업자가 최후 수단이 존재한다고 오판하게 만든다'는 점이 더 나쁩니다.

**수정**

3층으로 나눠 고칠 것.
1) `_odom_callback` 진입부에서 `np.isfinite(...).all()`을 확인한 뒤에만 상태를 갱신하고 `self._last_odom_time = self.get_clock().now()` 기록.
2) `_control_loop` 시작부에서 (now - _last_odom_time) > 0.2 s 이거나 비유한이면 `_state_valid=False`로 되돌리고 **OffboardControlMode 발행을 중단**해 PX4의 offboard-loss failsafe(500 ms)에 인계. 이것이 '우리 실패를 PX4 실패로 승격'시키는 핵심입니다.
3) hybrid_comparison.py에서 `_omega_dot_filt`/`_omega_prev` 갱신 **이전에** omega 유한성 검사(고착 방지), 그리고 vc 수신 직후와 최종 반환 직전에 `if not np.all(np.isfinite(...)): return self._fallback(...)`. LinAlgError 포착만으로는 NaN을 못 잡는다는 점이 핵심입니다. 루트/ros2_ws 두 사본 모두.
4) safety.py의 워치독은 실제로 카운트를 증가시키거나, 구현하지 않을 거면 독스트링 13행과 80행 주석에서 삭제(허위 안전감 제거). EMERGENCY를 쓸 거면 그 레벨에서 (2)의 발행 중단과 배선할 것.

---

### Issue 4 — 🟠 HIGH · 신뢰도 high

**노드가 제어기 생애주기와 시간 단조성을 관리하지 않는다 — 재arm 시 t가 되감기고 reset()이 호출되지 않음**

- 파일: `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`, `hybrid_comparison.py`
- 원 지적 ID: `ros2-px4#4`, `ros2-px4#10`

**무엇이 잘못됐나**

offboard_node.py:200-202가 arm 상승엣지마다 `self._t_start = self.get_clock().now()`를 재설정해 제어기에 넘기는 t가 25.0 → 0.0으로 되돌아갑니다. 그런데 파일 전체 grep에서 `reset` 결과가 **0건**입니다 — VirtualNMPC.reset()과 ProperHybrid.reset()이 존재하는데도 ROS 노드에서 도달 불가입니다.

결과 (검증자 정정 반영): hybrid_comparison.py:191 `if t - self._last_t >= self.dt_ctrl - 1e-8:` 가 -25 < 0.1로 계속 거짓이 되어, **NMPC 외측 루프가 약 25초간 재솔브 없이** 직전 `_u_current`([T, ω̇_des])를 동결한 채 INDI가 100 Hz로 그것만 추종합니다. 즉 고도·속도 피드백이 사라진 개루프 비행입니다(리뷰어가 쓴 '25초 전 명령'은 부정확 — `_control_loop`에 armed 검사가 없어 disarm 중에도 계속 솔브하므로 값 자체는 ~0.1초 전 것입니다). 부수로 273-277행 `actual_dt = t - self._prev_t` = -25 → 1e-4 클램프로 각가속도 한 스텝 글리치가 납니다(단 LPF alpha≈0.03과 safety의 rate limit 180 rad/s/step 때문에 단발에 그침).

같은 뿌리의 부수 결함: ARM/SET_MODE를 한 번만 보내고 `/fmu/out/vehicle_command_ack`를 구독하지 않으며(저장소 전체 참조 0건), Phase 2가 `_vehicle_armed`가 아니라 `_arm_requested`로 게이트됩니다(offboard_node.py:222). arm이 거부돼도 재시도 없이 제어기를 계속 돌리고 적분기를 누적합니다(±5.0 클립이라 유계).

**왜 중요한가**

이 프로젝트의 SITL 워크플로가 정확히 '전복 → PX4 자동 disarm → 재arm' 반복입니다(HANDOFF.md:154-157). 노드를 재시작하지 않고 QGC/pxh로 재arm하면 확정 제어기의 외측 루프가 개루프화된 채 재이륙합니다. MEMORY.md 핵심 주의사항 #1('NMPC/VirtualNMPC reset() → _last_t + w0 초기화 필수')이 제어기 내부에는 남아 있는데 ROS 노드가 그 reset을 호출하지 않아 **실질적으로 회귀**한 사례입니다. 노드가 스스로 재arm하지는 않으므로 외부 재arm이 전제조건입니다.

**수정**

1) t를 arm 기준 상대시각이 아니라 **노드 기동 기준 단조 증가 시각**(`_t_node_start`)으로 바꾸고 arm 시각은 로깅용으로만 쓸 것. 제어기 3종이 모두 t의 단조성을 전제하므로 이게 정석입니다.
2) 그와 별개로 arm 상승엣지에서 `if hasattr(self.controller, 'reset'): self.controller.reset()` 와 `self.safety.reset()` 호출.
3) `/fmu/out/vehicle_command_ack` 구독해 ARM/SET_MODE 결과를 확인하고, 거부되면 경고 로그 + 1초 주기 재시도. Phase 2 진입 조건을 `self._vehicle_armed and nav_state == OFFBOARD`로 바꾸고, armed가 아닌 동안에는 `_publish_zero_motors()` + 제어기 reset 유지로 적분기 누적을 막을 것.

---

### Issue 5 — 🟠 HIGH · 신뢰도 high

**heading 정렬이 제어기별 임시방편 + duck-typing 분기라 scheduled_lqr / indi 는 경고 없이 통과하고, v_ref 좌표계도 어긋난다**

- 파일: `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`, `controller.py`, `launch/sitl_offboard.launch.py`
- 원 지적 ID: `cross-cutting#2`, `ros2-px4#3`

**무엇이 잘못됐나**

offboard_node.py:299-313을 직접 확인했습니다. 분기가 `hasattr(self.controller, 'heading')` → `elif hasattr(self.controller, 'x_trim')` 두 개뿐이고 **else가 없습니다**. 실제 인스턴스 속성 확인 결과: CascadedPID/ScheduledPID만 `heading`, LQRController만 `x_trim`을 가집니다. ScheduledLQR은 둘 다 없고(내부는 `_x_trim_arr`), INDIController도 둘 다 없습니다. 즉 `-p controller_type:=scheduled_lqr` 또는 `:=indi`로 띄우면 아무 래치도 없이 조용히 통과합니다.

- ScheduledLQR: `_compute_error_state`가 yaw=0 트림 기준으로 δφ·δv를 만듭니다(trim.py:49-51은 R_hover*R_pitch만, yaw 성분 없음). 스폰 기수 -96° 상황에서 이 코드 **자신의 주석**(302-304행)이 설명하는 '월드좌표 오차 피드백이 잘못된 동체축으로 매핑 → 롤 커플/발산'을 그대로 겪습니다.
- INDIController: controller.py:691 `c1 = np.array([1, 0, 0])`으로 목표 기수를 0°에 하드코딩해 -96° 요 오차가 SO(3) 오차에 통째로 들어갑니다.
- 별개로 래치는 자세 기준만 맞추고 `v_ref`는 월드 NWU로 남겨둡니다. launch/sitl_offboard.launch.py:39-40은 v_ref_x를 '목표 전진 속도'라고 문서화하는데, CascadedPID는 `e_vel = vel - self.v_ref`로 월드에서 오차를 잡으므로 pid/scheduled_pid는 기수 -96° 유지 + 북쪽 30 m/s = 96° 옆미끄럼이 됩니다. LQR만 offboard_node.py:506 `x_trim_new[3:6] = Rz.apply(c.x_trim[3:6])`로 이미 해결해 뒀습니다(같은 규약을 나머지에 안 적용).

**왜 중요한가**

확정 제어기 ProperHybrid는 VirtualNMPC 비용에 자세/요 항이 아예 없어 래치가 애초에 불필요하므로(615-618행 의도적 제외 확인) **실기 기체 손실 경로는 아닙니다**. 그럼에도 high로 두는 이유는 (a) SITL 비교 실험에서 두 제어기가 경고 한 줄 없이 잘못된 기준으로 돌아 전복하고, (b) HANDOFF.md:154-157의 '롤 발산 미해결' 원인 규명을 정면으로 방해하기 때문입니다. LQR은 래치를 받는데도 발산한다고 기록돼 있으니 이 항목이 원인 전체는 아닙니다 — 그래서 더더욱 교란변수부터 제거해야 판정이 가능합니다.

**수정**

1) duck-typing 대신 `ctrl_type` 문자열로 명시 분기. 어떤 분기에도 안 걸리면 `get_logger().warn('heading 래치 미적용')`을 남기되, nmpc/hybrid는 '불필요(비용함수에 자세항 없음)'로 구분 출력 — 그냥 warn하면 오탐입니다.
2) ScheduledLQR: V_table 각 점의 트림을 스폰 기수로 회전시켜 K_r을 재계산하거나, `_interpolate` 결과에 Rz를 적용.
3) INDIController에 `heading` 필드를 추가하고 controller.py:691의 c1을 `[cos(heading), sin(heading), 0]`로.
4) 래치 시점에 `self.v_ref = Rz(yaw_now) @ self.v_ref`로 v_ref도 회전(PID/INDI는 controller.v_ref, VirtualNMPC는 vnmpc.v_ref). 또는 제어기 생성 자체를 래치 이후로 미뤄 처음부터 회전된 값을 넣을 것.

---

## 5. 시뮬 충실도 · 보고 정확도 · 미래 대비 — 13개 그룹

| # | 심각도 | 제목 | 신뢰도 |
|---|---|---|---|
| 6 | 🟡 MEDIUM | test_plant.py 회귀 게이트가 4개 채널에서 완전히 무감각하다 (CLAUDE.md가 지정한 유일한 게이트) | high |
| 7 | 🟡 MEDIUM | 로터 각운동량 항 2건: h_net 부호가 반토크·할당행렬과 모순 + ḣ(로터 가감속 반작용) 항 누락 | medium |
| 8 | 🟡 MEDIUM | 트림 수렴 실패가 조용히 하류로 전파된다 — find_trim → ScheduledLQR 게인테이블 → 실기 LQR 폴백 | high |
| 9 | 🟡 MEDIUM | 제어 할당 포화가 로터별 독립 클램프라 총추력과 3축 모멘트를 동시에 왜곡한다 (진단 로그 없음) | high |
| 10 | 🟡 MEDIUM | 기준값(트림 피드포워드 / v_ref)이 물리적으로 일관되지 않아 제어기별로 불균등한 정상상태 오차가 생긴다 | high |
| 11 | 🟡 MEDIUM | '물리 파라미터는 vehicle_params.py에만' 규약이 선언만 되고 강제되지 않는다 — n_max·tau_m·g가 5곳에 하드코딩 | medium |
| 12 | 🟡 MEDIUM | 코드 중복이 수동으로 관리된다 — 루트/ros2_ws 이중 트리, 3개 스크립트의 각자 다른 파일 목록, INDI 이중 구현 | high |
| 13 | 🟡 MEDIUM | 발표용 지표가 물리적 의미와 어긋나 결론 일부가 지표 아티팩트다 (돌풍 회복시간·스윕 창·발산 판정·SAT%) | high |
| 14 | 🟡 MEDIUM | 공력·로터 모델의 유효범위가 문서화되지 않은 채 Gazebo C++ 플러그인으로 이식될 예정 | medium |
| 15 | ⚪ LOW | 문서·주석이 구현과 어긋난다 (없는 파일 참조, 적용됐다고 적힌 미적용 수정, 라벨 오류) | high |
| 16 | ⚪ LOW | ESKF·센서 모델 4종 (파이썬 시뮬 전용 — 실기 항법은 PX4 EKF2) | high |
| 17 | ⚪ LOW | NMPC 초기추측 — 주요 부분은 이미 수정됨, 남은 것은 상태 초기추측이 동작점과 무관하다는 점 | high |
| 18 | ⚪ LOW | 미배선·진단 코드가 실기 빌드에 그대로 남아 있다 (test_motor 안전장치 우회, fallback_controller 복귀 판정) | high |

### Issue 6 — 🟡 MEDIUM · 신뢰도 high

**test_plant.py 회귀 게이트가 4개 채널에서 완전히 무감각하다 (CLAUDE.md가 지정한 유일한 게이트)**

- 파일: `test_plant.py`
- 원 지적 ID: `sweep-test#2`, `sweep-test#3`, `sweep-test#4`, `sweep-test#9`

**무엇이 잘못됐나**

네 리뷰어 지적(sweep-test#2/#3/#4/#9)을 합치면 6개 테스트 중 절반이 사실상 비어 있습니다. 제가 파일에서 직접 확인:

(a) test_damping(95행)에 **assert가 하나도 없습니다** — 함수 시작 95행 다음 assert는 154행(test_advance_ratio)입니다. C_mq를 -10.0 → +10.0으로 뒤집으면 `omega_dot_y = 0.1044`(발산 방향)를 출력하면서도 `ALL TESTS PASSED`가 찍힙니다.

(b) test_allocation(159행)은 균일 입력 `A @ [10,10,10,10]`만 검사하는데 Mz = 10·k·Σdirs이고 Σ[1,-1,1,-1]=0이라 요 부호 규약이 무엇이든 통과합니다. 검증자가 원 지적을 정정했는데 이 정정이 중요합니다 — dirs 전역 반전은 자기일관적인 대안 형상이라 버그가 아니고(폐루프 응답 동일 확인), 진짜 공백은 **할당행렬↔플랜트 왕복 일관성**입니다. A[3,:]만 부호를 뒤집으면 Mz_cmd=+0.5에 대해 ω̇_z=-0.714(요 정피드백)가 나오는데도 6개 테스트 전부 통과합니다. 롤 축도 미검증인데 Ixx=0.02는 Iyy=0.70의 1/35이라 같은 모멘트 오차에 35배 민감합니다.

(c) test_advance_ratio(154행) `assert xd_c[5] < xd_s[5]` 는 부등호뿐이라, J_max를 2.0 → 1e12로 만들어 전진비 모델을 통째로 꺼도 통과합니다(-0.6494 vs 원본 -4.2424, 6.5배 차이를 못 봄). -0.6494는 100% 동체 교차류 항력입니다.

(d) 바람 경로(dynamics.py:171 `v_body = R.T @ (vel - w)`), tau_m, step()의 쿼터니언 재정규화는 한 번도 검증되지 않습니다. 검증자가 뮤테이션 테스트로 확인: 바람 부호 반전 + 재정규화 제거를 동시에 적용하고 tau_m을 100배로 해도 `ALL TESTS PASSED`.

**왜 중요한가**

CLAUDE.md가 'dynamics.py 변경 시 test_plant.py로 역호환 검증 필수'라고 지정한 **유일한** 게이트입니다. 아래 Issue 8(로터 각운동량)은 dynamics.py를 반드시 고쳐야 하는데, 지금 상태로는 회귀 통과가 아무것도 보장하지 않습니다. 게다가 vehicle_params.py는 형상팀 값으로 통째 교체될 예정이라(vehicle_params.py:5가 플레이스홀더로 명시) 사각지대가 현실화될 경로가 확실히 존재합니다. 현재 코드 자체는 정상이므로 라이브 버그가 아니라 '앞으로 들어올 버그를 못 잡는' 리스크입니다.

**수정**

각 assert는 반드시 dynamics.py 수식에서 직접 유도할 것 — 독스트링 문구를 근거로 삼지 말 것(CLAUDE.md의 '독스트링과 구현이 달라 테스트 부호가 뒤집힌 전력' 경고).
1) test_damping: `assert wdot_y < 0` + 해석해 0.25·ρ·V·S·d²·C_mq·q/Iyy (= -0.10437)와 상대오차 1% 비교 + ω_y=-2 반대 케이스 `assert wdot_y > 0`. 이 조건에서 정적 모멘트가 정확히 0이므로 강한 assert가 가능합니다(주석의 '순수 감쇠만은 아님' 유보는 틀렸습니다).
2) test_allocation: 비균일 입력 왕복 검증 — A로 [T,Mx,My,Mz] 지령 → 역할당 → evaluate_xdot의 ω̇ 부호가 지령과 일치하는지 롤/피치/요 3축 각각 assert.
3) test_advance_ratio: fac 해석해(dynamics.py:126의 EPS 포함 형태)와 1e-6 비교, 최소한 (xd_s[5]-xd_c[5]) > 3.0.
4) 바람은 **축방향이 아니라 z/y 성분**으로 검증할 것 — 검증자 확인상 순수 축방향 바람은 Fx가 u_b의 짝수 함수라 부호 반전을 못 잡습니다(w=[-10,0,0]과 [+10,0,0]의 xdot이 완전 동일). w=[0,0,-10] vs [0,0,+10]은 vdot_z가 -1.9588 vs +0.1624로 갈립니다.
5) tau_m: n=0에서 n_cmd=n_hov 스텝 후 t=tau_m에서 63.2%(±2%) 도달 assert. step(): 노름≠1인 쿼터니언 주입 후 노름=1 assert.
6) 보강 후 위 4개 뮤테이션(C_mq 부호, A[3,:] 부호, J_max=1e12, 바람 부호)을 각각 적용해 **테스트가 실패하는지** 확인할 것. 통과하면 assert가 아직 부족한 겁니다.

---

### Issue 7 — 🟡 MEDIUM · 신뢰도 medium

**로터 각운동량 항 2건: h_net 부호가 반토크·할당행렬과 모순 + ḣ(로터 가감속 반작용) 항 누락**

- 파일: `dynamics.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers/dynamics.py`, `vehicle_params.py`, `controller.py`, `hybrid_comparison.py`
- 원 지적 ID: `plant-physics#1`, `plant-physics#10`

**무엇이 잘못됐나**

같은 물리항(로터 각운동량 h = I_r·Σd_i·n_i)에 두 결함이 인접해 있어 하나로 묶습니다.

(a) 부호 모순: dynamics.py:148 `M_tot += ca.vertcat(0.0, 0.0, di * Qi)`(항력 반작용)와 150 `h_net += p['I_rotor'] * ni * di`는 서로 반대의 스핀축을 함의합니다. 스핀벡터를 s·ẑ라 하면 h_z = I·n·s, M_z = -Q·s이고 Q = k_Q·n² ≥ 0이므로 두 줄이 동시에 성립하는 s가 존재하지 않습니다(150행은 s=+d, 148행은 s=-d를 요구). 실측: n = n_hov + 50·d, ω_y = 1 rad/s에서 ω̇_x = -5.00 rad/s²(일관되면 +5.00).

(b) ḣ 누락: 강체+회전자 오일러식 J·ω̇ = M - ω×(Jω) - ω×h - **ḣ** 중 마지막 항(-I_r·Σd_i·ṅ_i)이 없습니다. dynamics.py:176-187에서 w_dot 계산 뒤에 n_dot이 산출되고 어디에도 더해지지 않습니다.

**왜 중요한가**

LQR 계열은 완전 무영향입니다 — 검증자가 150행 부호를 뒤집어 85 m/s 트림에서 수치 야코비안을 다시 구한 결과 A/B/xdot 차이가 전부 정확히 0.0이었습니다(트림에서 ω=0이고 h_net = I_r(n1-n2+n3-n4) = 0). 문제는 INDI입니다:
- (b)의 크기: 85 m/s + 측풍 10 m/s에서 max|I_r·Σd·ṅ| = 1.3923 N·m (LQR 대비 91배), 공력 요토크 RMS 대비 31%, Izz=0.7 기준 미모델링 요 가속 1.99 rad/s².
- 더 결정적인 것은 즉시 피드스루 비교입니다: I_rotor/(tau_m·Izz) = 0.0357 대 compute_control_effectiveness의 G[3,i] ≈ 0.0163 — 실기에서 **INDI 요 증분의 초기 응답이 부호가 반대이고 크기가 약 2배**가 되어 요축 한계주기 위험이 있습니다.
지금 파이썬 시뮬 결과가 무효화되는 것은 아닙니다(플랜트와 제어기가 같은 누락을 공유해 자기일관적). 위험은 Gazebo C++ 공력 플러그인 이식과 실기 시점에 나타납니다.

**부호 방향 판정의 한계**: 원 리뷰어는 model.sdf의 turningDirection과 PX4 x500 순서를 근거로 삼았는데, 검증자가 정정했듯 그 축 정의를 담은 `model://fast_missile_base`가 이 저장소에 없어 검증 불가입니다. 유효한 근거는 '할당행렬(dynamics.py:221 `A[3,i] = dirs[i]*k`)과 148행이 서로 정합하므로 150행만 뒤집는 것이 최소 수정'뿐입니다.

**수정**

1) dynamics.py:150을 `h_net -= p['I_rotor'] * ni * di`로. 동시에 vehicle_params.py:64-67의 CW/CCW 주석 라벨을 서로 바꿀 것(현 규약에서 r1(FR)=+1은 '위에서 볼 때 CCW').
2) `_compute_xdot`에서 n_dot을 먼저 구한 뒤 `M_body -= ca.vertcat(0, 0, p['I_rotor'] * ca.dot(dirs, n_dot))` 추가(부호는 1의 규약 확정 후 동일 규약으로).
3) **회귀 범위가 넓습니다**: 이 항을 넣으면 u → ω̇_z 대수적 피드스루가 생기므로 controller.py:155 linearize_error_state의 B 행렬과 hybrid_comparison.py:356 `G[3,i] = dirs[i]*dQ/Jz`를 함께 갱신해야 합니다. ros2_ws/.../controllers/dynamics.py 사본도 동일.
4) 순서: Issue 7(test_plant 보강) **이후에** 착수. 회귀는 test_plant.py 전체 + 요 기동 + MEMORY.md 수치 재실행. 트림은 정상상태 ṅ=0, h≈0이라 거의 불변이어야 정상이고, 크게 바뀌면 버그를 의심할 것.
5) 실기 판정이 필요하면 Ubuntu의 fast_missile_base SDF에서 로터 조인트 축을 확인해 148행 규약과 대조할 것(macOS 세션에서는 불가).

---

### Issue 8 — 🟡 MEDIUM · 신뢰도 high

**트림 수렴 실패가 조용히 하류로 전파된다 — find_trim → ScheduledLQR 게인테이블 → 실기 LQR 폴백**

- 파일: `trim.py`, `controller.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`, `sweep_plots.py`
- 원 지적 ID: `plant-physics#3`, `classical-ctrl#5`, `sweep-test#7`, `plant-physics#7`

**무엇이 잘못됐나**

세 지적(plant-physics#3, classical-ctrl#5, sweep-test#7, plant-physics#7)이 같은 뿌리입니다: 수렴 실패가 예외가 아니라 print로 처리되고, 하류가 그 정보를 읽지 않습니다.

(a) trim.py:78-83 `if ier != 1: print(f"  [경고] 트림 수렴 실패: {msg}")` 후 검사 없이 `theta_sol, n_eq_sol, dn_sol = sol`. find_trim(P, 105.0)은 θ=-19.20°, Δn=-248.7, residual=0.177인 상태를 정상 반환합니다.
(b) 'residual'은 반환 dict에 있는데 검사하는 곳(trim.py:145, nmpc.py:256, sweep_plots.py:202, gust_comparison.py:165)과 안 하는 곳(controller.py:414 ScheduledLQR, offboard_node.py:596/604, mission_sim.py:611, hybrid_comparison.py:432)이 섞여 있습니다. 원 리뷰어의 '어디서도 확인하지 않는다'는 과장이고, **일관성이 없다**가 정확합니다.
(c) ScheduledLQR은 `lqr.valid`(ARE 성공)만 봅니다. 검증자가 n_max=700으로 포화를 재현하니 V=70에서 res=0.205, V=80에서 res=0.575인데도 두 경우 모두 valid=True가 되어 'ScheduledLQR: 9/9 속도점 유효'를 출력했습니다. 같은 로그에 trim.py의 경고와 이 카운트가 모순으로 찍히고, 유효 카운트가 경고를 덮습니다.
(d) trim.py:168-169 `valid = [...]; V_max_trim = valid[-1]['V']`가 스윕 상한을 그대로 '물리적 최고 트림속도'로 승격시킵니다. 기본 V_range=arange(0,90,5)라 항상 85가 나오고 trim.py:194가 '✓ 300 km/h 트림 가능'을 출력하는데, 확장 스윕으로 확인한 실제 한계는 100~105 m/s 사이입니다(100 m/s res=4.69e-09 수렴, 105 m/s res=1.77e-01 비수렴).
(e) sweep_plots.py:250 `base = rows[0][key][metric]`가 '30 m/s 대비'라고 문서화됐지만 실제로는 '살아남은 첫 행'이라, 30 m/s가 스킵되면 모든 무릎점 라벨이 조용히 다른 기준이 됩니다. sweep_plots.py:186-189의 trim_30도 residual 검사 없이 K_r_30 설계에 쓰입니다.

**왜 중요한가**

현재 파라미터로는 전 구간 잔차가 8e-12 ~ 2e-9라 **지금 잘못된 수치는 없습니다**(직접 확인). 위험은 정확히 CLAUDE.md가 예고한 시점입니다 — '형상팀 값이 오면 vehicle_params.py만 교체'. 교체 직후 이 경로가 곧바로 발현하고, 그 결과가 offboard_node.py:604가 만드는 **실기 LQR 폴백 게인 테이블**까지 흘러갑니다. (d)는 별개로 CLAUDE.md의 '파라미터를 목표치에 맞춰 끼워맞추지 말 것' 원칙에 정면으로 걸리는 자기확증형 출력이라, 이 숫자가 보고서에 인용되면 잘못된 성능 한계가 고착됩니다.

**수정**

1) find_trim 반환 dict에 `'converged': (ier == 1 and res_norm < 1e-6)` 추가.
2) ScheduledLQR 루프에서 미수렴 속도점은 테이블에서 제외하고, 유효 카운트 출력에 트림 실패 수를 병기(sweep_plots.py:202의 `> 1e-3` 기준을 클래스 안으로 옮기면 됨).
3) **offboard_node 경로에서는 예외로 승격해 실기 arming을 막을 것.** 현재 경고가 `self.get_logger()`가 아닌 `print()`라 ROS2 노드에서는 더 묻힙니다.
4) trim.py:184: V_max가 V_range의 마지막 원소와 같으면 '스윕 상한에 걸림 — 한계 미특정'으로 표기하고, 기본 V_range를 실패가 관측되는 지점(arange(0,130,5))까지 확장. 이전 속도 해를 다음 초기추정으로 넘기는 continuation을 넣으면 105 m/s 비수렴이 물리적 부재인지 초기추정 문제인지 구분됩니다.
5) find_knee의 base는 V==30 행에서 찾고 없으면 명시적 경고, `if not rows: raise RuntimeError(...)`.

---

### Issue 9 — 🟡 MEDIUM · 신뢰도 high

**제어 할당 포화가 로터별 독립 클램프라 총추력과 3축 모멘트를 동시에 왜곡한다 (진단 로그 없음)**

- 파일: `controller.py`
- 원 지적 ID: `classical-ctrl#3`

**무엇이 잘못됐나**

controller.py:129-133 `f_ind = self.TM_to_f @ TM` 뒤 `n_cmd[i] = np.sqrt(max(f_ind[i], 0) / self.p['k_T'])` — 로터별 독립 클램프이고 재배분 로직이 없습니다. INDIController._fallback(controller.py:709-711)에도 동일 패턴이 있습니다(원 리뷰어가 놓친 곳).

sweep_plots.py와 동일 초기조건(트림 + Δvx=+1, Δvz=+2, z_ref=50)에서 전 속도가 t=0부터 약 155스텝(0.16 s) 동안 음수 f_ind를 내고 최소값이 V=0 -72.2 N, V=70 -113.9 N, V=80 -129.3 N입니다(로터 1개 정상 추력 ~29 N). V=70, t=0의 왜곡 정도를 검증자가 정량화했는데 원 지적보다 심합니다:
  명령 [T, Mx, My, Mz] = [47.16, 0, 88.85, 0]
  클램프 후 실제      = [274.88, 0, 48.59, 0]
총추력이 명령 대비 5.8배(무게의 3.5배)로 뜨고 피치 모멘트는 55%만 전달됩니다.

근인은 할당식보다 controller.py:49 `self.Kp_att = np.array([200, 500, 500])`입니다 — Iyy=0.7, 팔길이 s=0.25/√2=0.1768 m에서 10° 오차가 My≈89 N·m를 요구하고 이는 전후 차동 추력 약 500 N에 해당합니다. 물리적으로 낼 수 없는 명령이라 어떤 할당을 써도 포화합니다.

**왜 중요한가**

순항 중 상시 문제는 아닙니다 — 음수 f_ind는 전부 초기 0.16초 이내에만 발생하고 네 속도 모두 최종 수렴합니다. 문제는 (a) 10초 RMSE에서 초기 과도가 지배적이라 스윕 지표가 '설계된 제어법칙'이 아니라 '클램프 결과'를 반영하고, (b) 아무 로그·카운터도 없어 스윕 결과를 볼 때 이 사실을 알 수 없으며, (c) 실기에서 같은 상황이 그대로 재현되는데 진단이 남지 않는다는 것입니다(HANDOFF의 att_gain_scale 0.35 도입 배경과 같은 방향).

**수정**

1) 우선순위 기반 할당으로 교체: 모멘트 명령을 스칼라 s∈(0,1]로 축소해 모든 f_ind ≥ 0이 되게 하거나(모멘트 방향 보존), 총추력을 먼저 확보하고 남는 여유 내에서 모멘트를 배분.
2) 포화 발생 여부를 카운터로 노출해 실험 결과 해석 시 확인 가능하게 할 것.
3) controller.py:709-711(INDI _fallback)도 함께.
4) 단, 어떤 할당을 써도 Kp_att=500이 요구하는 모멘트는 낼 수 없으므로 자세 게인 재검토가 병행돼야 합니다. 할당 수정만으로 포화가 사라진다고 기대하지 말 것.

---

### Issue 10 — 🟡 MEDIUM · 신뢰도 high

**기준값(트림 피드포워드 / v_ref)이 물리적으로 일관되지 않아 제어기별로 불균등한 정상상태 오차가 생긴다**

- 파일: `controller.py`, `mission_sim.py`
- 원 지적 ID: `classical-ctrl#2`, `classical-ctrl#4`, `mission-sim#4`

**무엇이 잘못됐나**

세 지적(classical-ctrl#2, #4, mission-sim#4)의 공통 뿌리는 controller.py:82 `F_des = self.m * (a_des + np.array([0, 0, self.g]))` — 중력만 보상하고 공력(순항 시 동체 음의 받음각이 만드는 하향력)은 전혀 없습니다. 트림 총추력 실측(mg=78.48 N): V=0 78.48, V=70 116.49, V=80 146.37 N → 추가 수직가속 0.00 / 4.75 / 8.49 m/s².

(a) 고도: 적분항 최대 기여가 Ki_z·int_z_max = 0.5×5.0 = 2.5 m/s²뿐이라 나머지를 비례항이 떠맡고 고도가 눌립니다. 25초 후 z = 49.080(V=70) / 47.564(V=80), 두 경우 모두 `_int_ez`가 -5.000으로 포화. 반대방향 검증으로 int_z_max만 50으로 키우면 50.003 / 50.005로 정상 수렴(필요 적분값 -8.69 / -14.78) → 클램프가 구속임이 확정.
(b) 수평: controller.py:79 `a_des[0:2] = -self.Kp_vel * e_vel[0:2]`로 P만 있어 vx 정상상태 오차가 -0.150(V=30) / -1.282(V=70) / -2.144(V=80). ScheduledPID는 controller.py:374 `Kp_vel = 1.0 - 0.2*alpha`로 게인을 낮춰 더 나빠집니다(-2.700, 3.4%). **두 문제는 독립**입니다 — int_z_max를 키워 고도를 완전 수렴시켜도 vx는 68.718/77.853으로 불변.
(c) 같은 부류: mission_sim.py:72 `return np.array([vx, 0.0, 0.0]), z, name` — 이륙 구간 z_ref는 1-cos으로 최대 7.5398 m/s 상승을 요구하는데 v_ref의 z 성분은 항상 0입니다. v_ref를 직접 비용에 쓰는 VirtualNMPC(Q_v=diag([5,5,10]))는 상승 중 vz≈+7을 벌점해 스스로 억제합니다. A/B 실측: 현행 rmse_z 3.5796 → v_ref[2]에 해석적 미분을 넣으면 2.6103(-27%). ScheduledLQR은 오차상태를 v_ref가 아니라 보간 트림(vz=0)으로 잡아 비트 단위 동일 → **제어기별로 불균등**.

**왜 중요한가**

실기 안전이 아니라 세 가지가 걸립니다. (1) 저고도 ISR 임무에서 수 미터 고도 처짐(V=80에서 -2.44 m). (2) sweep_plots / gust_comparison / ekf_comparison의 RMSE z·vx가 속도에 따라 체계적으로 나빠져 'P-only 구조에서 오는 편향'이 '게인 스케줄링 효과'나 '제어기 종류 차이'로 오독됩니다. (3) 이륙 구간 비교에서 NMPC 계열만 손해를 봅니다.

정직한 반대 증거: (c)의 핸디캡을 안고도 기록된 이륙 RMSE z는 LQR 9.8889 / Hybrid 4.0746 / NMPC 5.7851로 Hybrid가 이미 1위입니다. 27% 개선을 반영해도 순위는 안 바뀌고 Hybrid 우위가 커질 뿐 — 현재 보고 수치는 Hybrid에 **보수적인** 방향입니다. 또 (a)(b)는 CascadedPID 독스트링(controller.py:26)이 '외부: 속도 P + 고도 PID'라고 명시한 문서화된 설계이지 코딩 버그가 아닙니다.

**수정**

세 개를 **하나의 수정**으로 처리하는 것이 효율적입니다.
1) 트림 추력/틸트를 v_ref 기반 feedforward로 a_des에 더해 적분항이 잔여 오차만 담당하게 할 것. 이것이 (a)(b)를 동시에 해결합니다.
2) int_z_max는 물리적으로 역산: 최대 순항 트림 T_max 기준 `int_z_max ≥ (T_max/m - g)/Ki_z` (V=85까지 커버하려면 약 20 이상). 더 나은 방법은 클램프를 액추에이터 포화 시에만 동작하는 조건부 anti-windup(back-calculation)으로 바꾸는 것. CascadedPID:46과 INDIController:542를 함께.
3) MissionProfile에 기준 궤적의 시간미분을 함께 생성: 1-cos이므로 `vz_ref = (z1-z0) * 0.5*pi/dur * sin(pi*tau)`. get_ref와 compute_refs 양쪽(v_refs[mask,2]).
4) Q_v[2,2] 재튜닝은 하지 말 것(CLAUDE.md: 조건마다 재튜닝 금지). 수정 후 MEMORY.md/HANDOFF.md의 미션 RMSE 표(LQR 4.44 / Hybrid 1.75 / NMPC 2.80) 재실행 갱신 필요.

---

### Issue 11 — 🟡 MEDIUM · 신뢰도 medium

**'물리 파라미터는 vehicle_params.py에만' 규약이 선언만 되고 강제되지 않는다 — n_max·tau_m·g가 5곳에 하드코딩**

- 파일: `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`, `launch/sitl_offboard.launch.py`, `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/frame_utils.py`, `sensors.py`, `estimator.py`, `model.sdf`
- 원 지적 ID: `cross-cutting#3`, `cross-cutting#5`, `cross-cutting#9`, `plant-physics#2`

**무엇이 잘못됐나**

(a) n_max=1800이 vehicle_params.py:83 외에 offboard_node.py:93 `declare_parameter('n_max', 1800.0)`, launch/sitl_offboard.launch.py:74, frame_utils.py:234(자체검증 전용), model.sdf 4곳 `<maxRotVelocity>`에 각각 박혀 있습니다. 제어기는 `np.clip(u, p['n_min'], p['n_max'])`로 vehicle_params를 보는데, 노드는 `SafetyGuard(n_max=self.n_max)`와 `motor_speed_to_normalized(motor_speeds, self.n_max)`(frame_utils.py:180이 0~1로 클립)로 자기 값을 씁니다.
(b) tau_m: offboard_node.py:347 `tau_m = 0.02  # 모터 시상수` 하드코딩. 같은 계산을 하는 ekf_sim.py:110은 `tau_m = P['tau_m']`를 씁니다. 이 값이 만드는 `_motor_speeds`가 `_assemble_state`를 통해 x[13:17]로 들어가 INDI(controller.py:577)와 ProperHybrid(hybrid_comparison.py:239)의 G·T_meas 작동점이 됩니다.
(c) g=9.81: sensors.py:104 `def measure(..., g=9.81)`, sensors.py:220 SensorSuite, estimator.py:64 ESKF 기본값에 각각 있고, 실제 호출부(sensors.py:288, ekf_comparison.py:51)가 전부 기본값을 씁니다. 반면 같은 루프의 ESKF는 ekf_sim.py:69 `ESKF(x0_est, g=P['g'])`로 vehicle_params를 참조합니다.

**[근거 약함 — 낮게 배치] model.sdf momentConstant**: plant-physics#2가 든 'model.sdf 0.016 vs k_Q/k_T 0.0833, 5.2배 어긋남'은 이 저장소만으로는 성립하지 않습니다. 제가 직접 확인: `.gitignore:38 *.sdf`로 model.sdf는 **git 미추적 로컬 스냅샷**이고, HANDOFF.md:118-119가 'SDF는 momentConstant 0.016→0.0833만 유지(요 권한 정합, 부호 무관)'로 Ubuntu 원본 수정 완료를 이미 기록합니다. 정확히 같은 이유로 cross-cutting#11은 기각됐는데 plant-physics#2만 살아남은 것은 판정 불일치입니다. 남는 실질 항목은 '루트의 stale 사본을 지우거나 갱신'뿐입니다. plant-physics#2의 나머지 논거(FM 0.22이므로 k_Q가 3배 크다)도 검증자가 '고피치 프로펠러는 정지 FM 0.3 이하가 정상'으로 무력화했고 vehicle_params.py:5가 플레이스홀더로 선언합니다.

**왜 중요한가**

현재는 모든 값이 우연히 일치해 오동작이 0입니다. 위험 시점이 명확합니다 — 형상팀 값이 와서 vehicle_params.py를 교체하는 순간입니다. 그때 (a) 제어기는 새 n_max까지 명령하는데 노드는 옛 값으로 정규화해 1.0에 포화시켜 추력 여유가 조용히 사라지고 INDI의 Δn이 반영되지 않는 구간이 생깁니다. (b) 노드의 로터 속도 옵저버만 옛 시상수로 남아 n_actual을 과대추정하면 INDI의 `n_cmd = n_actual + dn`이 실제 로터보다 앞서 나갑니다. (c) IMU 모델과 ESKF가 다른 중력을 쓰면 가속도계 바이어스 추정이 그 차이만큼 편향되고 속도 적분에 누적됩니다.

**수정**

1) offboard_node는 n_max/tau_m 기본값을 `from .controllers.vehicle_params import vehicle_params as P`로 읽고, launch에서는 n_max를 아예 넘기지 말 것(노드 기본값 = vehicle_params).
2) **단, 노드의 n_max는 순수 물리값이 아니라 PX4 ActuatorMotors 0~1 정규화 기준입니다** — Gazebo SDF의 `<maxRotVelocity>`와 반드시 같아야 합니다. vehicle_params ↔ SDF 일치를 기동 시 assert하거나, vehicle_params를 단일 출처로 SDF를 생성하는 스크립트를 둘 것. tau_m도 마찬가지로 SDF의 timeConstantUp/Down과 대조 후 결정할 것(우리 플랜트 모델 값이 자동으로 옳은 것은 아님).
3) SensorSuite/IMUSensor.measure/ESKF의 g 기본값을 제거해 반드시 넘기도록 강제하거나, 호출부에서 `g=P['g']` 명시.
4) 루트 model.sdf 스냅샷을 삭제하거나 Ubuntu 원본에서 재복사(현재 0.016으로 stale).

---

### Issue 12 — 🟡 MEDIUM · 신뢰도 high

**코드 중복이 수동으로 관리된다 — 루트/ros2_ws 이중 트리, 3개 스크립트의 각자 다른 파일 목록, INDI 이중 구현**

- 파일: `scripts/setup_ubuntu.sh`, `scripts/sync_controllers.sh`, `scripts/sitl_run.sh`, `controller.py`, `hybrid_comparison.py`
- 원 지적 ID: `ros2-px4#2`, `ros2-px4#7`, `ros2-px4#11`, `cross-cutting#1`, `cross-cutting#8`

**무엇이 잘못됐나**

(a) **setup_ubuntu.sh가 매번 동기화를 깨뜨립니다.** 160행이 이미 상대 import로 동기화된 `ros2_ws/src/fast_drone_ctrl`을 통째로 복사한 직후, 167-174행이 저장소 루트의 절대 import 버전을 그 위에 덮어씁니다. 그런데 sync_controllers.sh가 하는 상대 import 변환은 하지 않습니다. 결과: lqr 경로는 offboard_node.py:594 `from .controllers.trim import find_trim` → trim.py:18-19 **모듈 레벨** 절대 import로 즉시 ModuleNotFoundError, pid는 제어기 생성 시점에, hybrid는 덮어쓰인 trim.py를 끌어와 같이 죽습니다. 목록(5개)에 hybrid_comparison/fallback_controller가 빠진 것이 오히려 hybrid를 못 살립니다. 게다가 155-157행 `rm -rf "${CTRL_PKG}"`가 먼저라 재실행이 동작하던 설치를 파괴합니다.
(b) 같은 스크립트 181행 `pip3 install --user numpy scipy casadi`는 명시된 대상 OS인 Ubuntu 24.04(PEP 668)에서 exit 1이고, 28행 `set -euo pipefail` 때문에 colcon build(187-201행) 전에 죽습니다. --user로는 우회되지 않습니다.
(c) sitl_run.sh:143의 `kill_pattern 'gz sim' TERM` 등이 `pkill -f` 부분일치라 다른 프로젝트의 Gazebo까지 죽입니다. 기동 시 무조건 실행되고(--help만 예외), teardown 105-115행에도 같은 패턴이 있습니다.
(d) **알고리즘 자체가 두 벌입니다**: controller.py:605 INDIController는 `omega_dot_raw = (omega - self._omega_prev) / self.dt`로 고정 dt를 쓰는데, hybrid_comparison.py:273-278 ProperHybrid는 '하드코딩 dt 대신 실제 경과시간 사용 (SITL은 루프율 가변)'이라는 명시적 이유로 `actual_dt = t - self._prev_t`와 alpha 재계산을 씁니다. 같은 개선이 한쪽에만 적용됐습니다.
(e) **이미 해결됨**: sync_controllers.sh의 `sed -i -E`(macOS BSD sed 실패)는 확인 결과 이미 `perl -pi -e`로 수정돼 있고(36행 주석이 그 이유를 명시), 정상 동작합니다. cross-cutting#1은 종결 처리하십시오.

**왜 중요한가**

같은 모듈이 루트와 ros2_ws/controllers/ 두 벌 존재하고, 그 동기화를 스크립트 3개가 각자의 파일 목록으로 관리합니다(sync_controllers.sh MODULES 7개 vs setup_ubuntu.sh 5개). 실기로 나가는 코드가 시뮬 코드와 조용히 갈라질 수 있는 구조이고, setup_ubuntu.sh는 실제로 그 갈라짐을 매번 만들어냅니다. (d)는 '실기 루프율이 흔들리면 ω̇ 추정이 왜곡된다'는 이미 학습된 교훈이 사본 하나에만 반영된 사례입니다. 다만 (a)의 결과는 노드 기동 실패(ModuleNotFoundError)라 **조용한 오작동이 아니라 즉시 드러나는 큰 소리 실패**이고, 기체가 뜬 상태에서 발생하지 않습니다.

**수정**

1) setup_ubuntu.sh 167-177행 블록을 통째로 삭제(160행 복사로 이미 상대 import 버전이 들어감). 대신 배포 전에 `bash "${PROJECT_DIR}/scripts/sync_controllers.sh"`를 호출해 저장소 안에서 변환을 끝낸 뒤 복사하도록 순서를 바꿀 것. sync_controllers.sh의 MODULES를 **단일 진실 공급원**으로 삼아 파일 목록 중복 관리를 없앨 것.
2) numpy/scipy는 `sudo apt-get install -y python3-numpy python3-scipy`(ROS2 Python이 시스템 numpy를 보므로 이중 설치도 피함), casadi만 `pip3 install --break-system-packages --user casadi`. 실패 시 조치 안내 메시지 추가.
3) pkill 패턴을 `'gz sim.*gz_fast_missile'`처럼 프로젝트 전용으로 좁히고, 이전 실행 PID를 파일에 기록해 그 PID만 정리하는 방식으로.
4) INDIController.__call__에 ProperHybrid와 동일한 패턴 적용(`_prev_t` 추가, actual_dt 클램프, alpha 매 스텝 재계산). 더 나아가 ω̇ 측정+LPF를 두 클래스가 공유하는 헬퍼로 빼서 다시 갈라지지 않게 할 것. 참고: 근본적으로 올바른 분모는 ROS 타이머 주기도 t 차분도 아니라 오도메트리 샘플 타임스탬프 차이입니다.
5) 중기 과제: 루트/ros2_ws 이중 트리 자체를 없애는 방향(패키지 하나를 양쪽에서 import)을 검토할 것.

---

### Issue 13 — 🟡 MEDIUM · 신뢰도 high

**발표용 지표가 물리적 의미와 어긋나 결론 일부가 지표 아티팩트다 (돌풍 회복시간·스윕 창·발산 판정·SAT%)**

- 파일: `gust_comparison.py`, `sweep_plots.py`, `mission_sim.py`
- 원 지적 ID: `mission-sim#1`, `mission-sim#3`, `mission-sim#5`, `mission-sim#2`, `sweep-test#1`, `sweep-test#5`, `sweep-test#6`

**무엇이 잘못됐나**

일곱 지적(mission-sim#1/#2/#3/#5, sweep-test#1/#5/#6)이 같은 부류입니다: 지표가 물리적 정의 없이 정해지고 그 값이 곧바로 문서로 승격됐습니다.

(a) **돌풍 회복시간이 무의미** (가장 심각): gust_comparison.py:82-88이 탐색을 돌풍 시작 i0부터, 절대 기준(z_ref/V_ref)으로 합니다. 실행 확인 결과 LQR고정/LQR스케줄/NMPC는 첫 0.05초 창에서 조건이 즉시 성립해 settle=0.0000 s(실제 최대 고도편차는 t=2.728 s), PID고정/INDI는 vx 정상상태 오차 때문에 조건이 한 번도 성립 안 해 settle=6.0000 s(=T_sim-t_gust 기본값). 즉 '회복 시간 [s]' 열은 0.00 아니면 6.00 두 값뿐이고, 이 파일의 목적('왜 INDI가 필요한가')이 이 열에 걸려 있습니다.
(b) 같은 함수 70-76행에서 dy만 `xs[i0,1]` 기준선이고 dz/dvx는 절대 기준 + 창이 시뮬 끝까지 열려 있습니다. PID고정의 보고 max_dz=1.1637 m는 t=5.567 s(돌풍 종료 2.57초 뒤)에, max_dvx=1.2466은 t=8.000 s(마지막 샘플)에 잡힙니다. 돌풍 직전 이미 dz=-0.7994 드리프트 상태이고 돌풍 구간 내 기준선 대비 편차는 0.3877입니다.
(c) **sweep_plots.py:68 `T_SIM = 10.0`이 정착시간보다 훨씬 짧습니다.** V=85 StrictFixedLQR로 120초 적분하면 vx가 76.689(t=10) → 67.906(t=40) → 66.839(t=100)로 계속 흐릅니다(λ_max=-0.0584 → τ≈17 s). T=40 창으로 재집계하면 RMSE_z가 30 m/s 대비 3.53배가 되어 KNEE_FACTOR=3을 넘고 무릎점(z)이 85 m/s로 잡힙니다(T=120에서는 8.26배). 즉 results/sweep_knee.txt의 '무릎점(z)=없음'과 MEMORY.md의 '고정 LQR은 z 저하 없음'은 **10초 창의 함수**입니다.
(d) sweep_plots.py:144-152: 발산 판정은 NaN/|z-z_ref|>200/|vz|>100의 OR인데 절단 인덱스는 z 조건만 봅니다. NaN 케이스는 rmse=nan으로 찍히고 플롯에서 선도 마커도 사라집니다.
(e) sweep_plots.py:157의 SAT%가 (N,4) 전체 평균이라 단일 로터 포화가 1/4로 희석되는데 _conditions()는 '시간 비율'로 설명합니다(현실적 희석은 전후 쌍 단위라 2배). 이 기체는 trim.py:147-151대로 후방 로터가 먼저 포화합니다.
(f) mission_sim.py:218-222 compute_overall은 NaN만 보는데 compute_phase_metrics(198-201행)는 isfinite + |z-z_ref|>50까지 봅니다. run_monte_carlo가 compute_overall의 'diverged'를 그대로 써서 793-794행 'Hybrid 최악 < LQR 최선: 통계적 우위 확실' 판정을 내립니다. 커밋 1a8c53e가 한쪽만 고친 불완전 수정입니다.
(g) gust_comparison.py:197 '_FixedLQR' 독스트링은 '30 m/s 게인 + 해당 속도 트림'인데 호출부는 70 m/s 트림을 넘깁니다 = 게인만 고정. 실측으로 트림까지 고정하면 max_dvx가 0.450 → 16.846(37배)로 벌어져 '고정 vs 스케줄' 대비가 완전히 달라집니다.

**왜 중요한가**

이 지표들이 MEMORY.md / HANDOFF.md / results/*.txt로 승격되어 프로젝트 결론이 됐습니다. (a)(b)는 발표 결론이 실제로 무효인 경우이고, (c)는 핵심 서술이 창 길이의 함수라는 뜻입니다.

정직한 반대 증거도 같이 봐야 합니다: (c)에서 창을 늘려도 상대 결론은 유지됩니다(T=40에서 vx 28.89배 ≫ z 3.53배 — '저하는 vx 축이 지배적'은 그대로). (a)의 정성 결론(고전 피드백이 대등/우수하므로 INDI가 필요)도 기준선을 고쳐 재계산하면 방향이 유지됩니다(돌풍 직전 기준 max|dz_rel|: LQR고정 0.2891 < NMPC 0.3152 < PID고정 0.4015 < INDI 0.5633). (f)는 기록된 MC 20시행 rmse_z 최대 4.462로 발현 사례가 없습니다. 즉 **무효인 것은 숫자와 이분법적 판정이지 결론의 방향이 아닙니다.**

**수정**

(a)(b)는 한 번의 수정으로: 탐색 시작을 `j0 = np.searchsorted(ts, t_gust_start + T_gust)`(돌풍 종료 후)로, dz/dvx도 dy와 같은 돌풍 직전 기준선(`xs[i0,2]`, `xs[i0,3]`)으로 통일, '미회복'은 유한값 대신 np.inf/None으로 두고 표에 'N/R' 표기(249-252행 `{r[metric]:>10.2f}` 포맷도 함께 수정), 연속 판정 창을 0.05 s → 0.5~1.0 s로, 최대편차 창은 돌풍+회복 구간으로 한정.
(c) T_SIM을 정착시간의 3~5배(최소 60 s)로 늘리거나, 'RMSE(과도)'와 '마지막 1초 평균 오차(정상상태)'를 분리해 둘 다 보고. 창을 유지한다면 최소한 '10 s 창은 정착 전이며 RMSE_z 평탄성은 창 의존적'을 txt와 MEMORY.md에 명시할 것.
(d) 세 발산 기준 각각의 첫 True 인덱스 중 최솟값을 end로. 발산 케이스 RMSE는 곡선 위 숫자로 찍지 말 것.
(e) `np.max(np.mean(n_all >= 0.95*n_max, axis=0))` 또는 '어느 한 로터라도 포화한 시간 비율'로 바꾸고 _conditions() 문구를 정의와 일치시킬 것.
(f) `_is_diverged(xs, z_refs)` 헬퍼 하나를 모듈 상단에 두고 compute_overall/compute_phase_metrics가 공유.
(g) 코드를 바꾸지 말고(바꾸면 max_dvx가 37배 변해 표 전체 무효) 독스트링을 'K_r만 30 m/s 고정, 트림 ff는 현재 순항속도'로, 열 이름을 'LQR게인고정'으로 정정할 것. HANDOFF.md:206-208이 이미 이 구분을 문서화하고 있습니다.

**어느 것이든 수정 후 results/*.txt + MEMORY.md + HANDOFF.md 재실행 갱신이 필요하므로, Issue 10·11과 한 배치로 묶어 재실행을 한 번만 하십시오.**

---

### Issue 14 — 🟡 MEDIUM · 신뢰도 medium

**공력·로터 모델의 유효범위가 문서화되지 않은 채 Gazebo C++ 플러그인으로 이식될 예정**

- 파일: `dynamics.py`, `vehicle_params.py`, `trim.py`
- 원 지적 ID: `plant-physics#5`, `plant-physics#6`, `plant-physics#9`, `plant-physics#4`

**무엇이 잘못됐나**

(a) **로터 면내 항력(H-force)이 정확히 0입니다.** dynamics.py:110 `V_axial = ca.fmax(-v_body[2], 0.0)`, 126행 `J = V_axial/(n_rps*D_prop+EPS)`이고 로터가 만드는 힘은 133행 `F_tot += ca.vertcat(0.0, 0.0, -Ti)` 하나뿐이라 x/y 성분이 문자 그대로 없습니다. 85 m/s 트림에서 면내 전진비 μ = 84.178/(933.58×0.15) = 0.601인데(디스크가 거의 edgewise) 로터 항력이 0입니다. 검증자가 동체 축력으로 등가 치환해 재트림한 결과: +2.2 N(보수적 하한)만 더해도 θ가 -7.976° → -10.348°(+30%), n_rear 933.6 → 1022.7(+9.5%); +10.9 N(동체 항력과 동급)이면 **85 m/s 트림해가 아예 사라집니다**(잔차 7.1e-02). 즉 trim.py:194의 '✓ 300 km/h 트림 가능'이 여유 10 N 수준의 가정 위에 서 있습니다.
(b) dynamics.py:73-76의 교차류 항에 Jorgensen 식의 A_p/S_ref(=8.49)가 빠졌는데 vehicle_params.py:51은 C_dc=1.2를 '원통 표준'(raw 값)이라 명시합니다. 다만 실제 영향은 작습니다 — 순항 돌풍에서 Fz -18.25 → -19.75 N(8%), 트림 θ 불변, n_rear +5.3%. 원 리뷰어의 '호버 중 무게의 14%가 빠짐'은 틀렸습니다(mission_sim의 돌풍은 t=35 s로 순항 한복판). 유의미해질 구간은 고받음각 저속(감속 43~58 s)일 가능성이 크나 **미확인**입니다.
(c) dynamics.py:78-80 `C_A = C_A0 + C_Aa2*(v_b²+w_b²)/V_sq` / `Fx = -q_bar*S*C_A`가 u_b의 짝수 함수라 항상 Fx ≤ 0입니다(u_b=+20과 -20이 둘 다 -0.5195 N). 역류에서 항력이 추력처럼 작용하지만 현재 u_b<0을 만드는 경로가 없습니다(x축 바람 옵션 없음, vx_ref≥0). 실제로 발현되는 것은 α 의존성 쪽 — 이륙 구간(u_b≈0, w_b≈-9.6)에서 C_A가 최대값 1.120이 되어 0.14 m/s²의 가짜 후방 가속이 매번 생깁니다. 단 이 형태는 vehicle_params.py:53이 'C_Aa2 = 유도 축력'으로 명시 채택한 파라미터화라 버그가 아니라 모델 선택 비판입니다.
(d) vehicle_params.py:16-18의 n_max 팁 마하 근거가 회전 성분만 계산했습니다(1800×0.15=270 m/s, M 0.79). 전진 블레이드는 ΩR+V를 봅니다. 다만 검증자 재현 결과 실제 동작점은 훨씬 낮습니다 — 85 m/s 정상 트림 n_rear=933.6 → 225 m/s(M 0.66), INDI 과도 최대 명령 1233 rad/s에서만 270 m/s(M 0.79) 근접이고 n_max=1800에는 도달하지 않습니다. 원 리뷰어의 '전 구간 포화, 팁 M 1.04'와 '호버 여유 2 m/s'는 둘 다 재현되지 않았습니다(호버는 n_hov=571.8 → M 0.25).

**왜 중요한가**

이 항목들 각각은 시뮬 충실도 문제이고 실기 손실 경로가 아닙니다. 문제는 MEMORY.md의 다음 단계가 'C++ 공력 플러그인 (dynamics.py의 공력을 Gazebo에 이식)'이라는 것 — **같은 낙관이 SITL 예측과 실기 설계 판단으로 그대로 전파됩니다.** 특히 (a)는 '300 km/h 트림 가능'이라는 프로젝트의 핵심 주장을 직접 떠받치고 있고, 그 여유가 10 N밖에 안 됩니다(vehicle_params.py:47-49 주석이 설명한 양의 되먹임 — 항력↑ → 틸트↑ → 동체 반양력↑ → 추력↑ → 항력↑ — 때문).

**수정**

**모델 확장을 지금 하지 마십시오.** vehicle_params.py:5가 이 값들을 명시적 플레이스홀더로 선언한 상태에서 H-force·A_p/S_ref·cos²α 감쇠를 넣으면 CLAUDE.md의 '파라미터를 목표치에 맞춰 끼워맞추지 말 것'에 걸립니다. 지금 할 일은 유효범위를 명시하는 것입니다.
1) trim.py의 결론 문구에 '로터 면내 항력 미포함 / 압축성 손실 미포함'이라는 단서를 추가. '✓ 300 km/h 트림 가능'을 '현 모델 가정 하에서 가능'으로.
2) vehicle_params.py:16-18의 마하 근거를 ΩR + V_cruise 형태로 고쳐 적고, 형상팀 값 수령 시 재산정 항목으로 등록(M 0.8을 85 m/s에서 지키려면 n_max ≤ 1247 → T/W와 포화 여유가 줄어 k_T/D_prop 재검토가 연쇄됨을 함께 메모).
3) vehicle_params.py:51의 '원통 표준' 표현을 'S_ref 기준 유효값'으로 바꾸거나 A_p 인자를 추가(둘 중 하나로 명확히).
4) **dynamics.py 상단에 '이 모델이 담지 않는 것' 목록을 명시**: 로터 면내 항력, μ 의존 k_T 보정, 압축성 손실, 로터 ḣ 반작용(Issue 8), 역류 축력 부호. Gazebo 플러그인 이식 시 이 목록이 그대로 넘어가지 않도록.
5) (b)의 감속 구간 영향은 '영향 확인 필요'로 남기고, 여유가 생기면 감속 구간만 C_dc 8.49배로 A/B 재실행해 확인할 것.
6) 형상팀 값이 온 뒤 확장한다면, 추가 전후로 85 m/s 트림의 θ·n_eq 회귀 비교 필수.

---

### Issue 15 — ⚪ LOW · 신뢰도 high

**문서·주석이 구현과 어긋난다 (없는 파일 참조, 적용됐다고 적힌 미적용 수정, 라벨 오류)**

- 파일: `HANDOFF.md`, `results/MISSION_ANALYSIS.md`, `controller.py`, `hybrid_comparison.py`, `mission_sim.py`
- 원 지적 ID: `cross-cutting#7`, `cross-cutting#10`, `classical-ctrl#6`, `nmpc-indi#5`, `mission-sim#7`

**무엇이 잘못됐나**

(a) HANDOFF.md:69-72가 results/PROJECT_REPORT.md를 가리키는데 저장소 어디에도 없고, FILES.md는 legacy/에만 있습니다. 175행의 scratch_yawtest.py / scratch_delaytest.py도 없습니다(스스로 '정리 대상' 표기). MEMORY.md의 Key Files 절도 같은 PROJECT_REPORT.md를 참조합니다. 참고로 HANDOFF.md/TODO.md/CLAUDE.md는 .gitignore:41-43으로 git 미추적 개인 문서라 영향 범위는 kj 본인의 다음 세션입니다.
(b) results/MISSION_ANALYSIS.md:6과 159행이 '적용된 수정: ... 2-2(VirtualNMPC T_ref를 트림 추력으로 전달)'라고 적지만, mission_sim.py:625의 Hybrid 생성부에 T_ref 인자가 없어 hybrid_comparison.py:90의 기본값 `mass*g`=78.48 N이 쓰입니다(hybrid_comparison.py:504는 제대로 넘김). 같은 누락이 mission_sim.py:342에도 있습니다. 게다가 MissionController가 T_ref를 갱신하지 않아 단일 값을 넘겨도 구간별로는 여전히 틀립니다 → **코드가 아니라 문서 문구를 정정하는 것이 맞습니다.**
(c) controller.py의 506/537/675행이 세 번에 걸쳐 INDI 외측이 'PID와 동일'이라고 하는데 max_tilt만 45°(controller.py:543) vs 35°(controller.py:52)입니다. 현재는 요구 틸트 최대 9.41°라 클리핑 0회로 무해하지만, 시나리오를 키우면 조용히 불공정해집니다. 부수로 _force_to_attitude의 c1이 CascadedPID는 heading 파라미터(115행), INDI는 `[1,0,0]` 하드코딩(691행)이라 heading≠0에서 두 번째로 깨집니다.
(d) hybrid_comparison.py:435 `T_trim = float(np.sum(P['k_T'] * u_trim**2))`이 dynamics.py:129가 실제 적용하는 fac를 빠뜨려 출력값이 10.4% 과대(116.486 vs 실제 105.959 N). 검증자 실측상 T_ref가 최적해에 실제로 영향을 줍니다 — T_ref=105.959로 주면 트림이 정확한 부동점이 되고(u0=[105.959,0,0,0]), 3초 폐루프 rmse_z가 0.0064 → 0.0000. 약 8 mm의 지속적 고도 바이어스입니다. 같은 패턴이 ekf_comparison.py:71, bench_accel_experiments.py:99에 복제.
(e) mission_sim.py:114 `a_str = f"{np.pi/2*abs(z1-z0)/dur:.1f} m/s2(z)"` — 1-cos의 최대 '속도'(7.5398 m/s)를 계산해 놓고 m/s2로 라벨링합니다. 실제 최대 수직가속도는 0.5*(π/dur)²*|Δz| = 2.3687 m/s². 바로 위 vx 분기는 같은 식이 속도 변화율이라 m/s2가 맞습니다(단위만 복사한 형태). 미션 개요 표는 보고서에 그대로 인용됩니다.

**왜 중요한가**

이 프로젝트는 '독스트링과 실제 구현이 달라 테스트 부호가 뒤집힌 전력'을 CLAUDE.md에 명시적으로 경고합니다. (c)가 정확히 그 패턴이고, (b)는 '적용됐다'는 문서 주장 때문에 재현 시도가 실패합니다. (a)는 새 세션 에이전트가 CLAUDE.md 지침대로 HANDOFF를 정독하고 배경을 찾다가 헤매게 만듭니다.

**수정**

(a) 파일 구조 블록에서 PROJECT_REPORT.md 삭제, FILES.md는 `legacy/FILES.md`로 경로 정정, 175행은 '삭제됨(재작성 필요)' 표기. MEMORY.md의 같은 참조도 함께. 새로 생긴 bench_*.py / horizon_comparison.py / sweep_plots.py를 구조 블록에 추가.
(b) MISSION_ANALYSIS.md:6과 159행을 'hybrid_comparison.py에만 적용, mission_sim은 호버 기준 mg 사용(영향 무시 가능)'으로 정정.
(c) 45°가 의도인지 실수인지 코드만으로는 판단 불가하므로 **kj가 결정할 것**. 35°로 맞추면 gust_comparison 재실행 필요(현재 클리핑이 없어 안 바뀔 가능성이 높지만 확인). 어느 쪽이든 주석 3곳을 실제와 일치시킬 것. c1 하드코딩도 함께.
(d) ProperHybrid.__call__의 T_meas 로직(hybrid_comparison.py:266-271)과 같은 방식으로 fac를 곱하거나 `plant.evaluate_xdot`으로 트림 상태의 실제 추력을 뽑아 쓸 것. ekf_comparison.py:71도 같은 값을 제어기에 넣으므로 RTK/ESKF 비교가 동일한 미세 바이어스를 공유합니다.
(e) `m/s(z)`로 라벨을 고치거나 실제 최대 가속도 `0.5*(np.pi/dur)**2*abs(z1-z0)`를 계산할 것. 표 헤더 '최대가속'도 두 의미가 섞이지 않게 열을 나눌 것.

---

### Issue 16 — ⚪ LOW · 신뢰도 high

**ESKF·센서 모델 4종 (파이썬 시뮬 전용 — 실기 항법은 PX4 EKF2)**

- 파일: `ekf_sim.py`, `estimator.py`, `ekf_comparison.py`, `mission_sim.py`
- 원 지적 ID: `estimator#1`, `estimator#2`, `estimator#4`, `estimator#5`, `mission-sim#6`

**무엇이 잘못됐나**

(a) ekf_sim.py:96이 t_k에 샘플링한 측정을 공칭상태를 t_{k+1}로 전파한 뒤 적용합니다. 정상상태 편향이 정확히 -v·dt이고(칼만게인과 무관), 70 m/s에서 x축 평균 오차 +0.0717 m = 잡음 std의 약 58배입니다. **다만 검증자 정정이 중요합니다**: 제어기에 넘어가는 x_hat은 x_true[k] 대비 +0.0019 m로 사실상 정합하고, 편향은 오직 ekf_sim.py:125 `xs_est[k+1] = x_hat` 로깅 후 128행에서 같은 인덱스끼리 뺄 때만 나타납니다. 즉 update 순서 문제이자 동시에 로깅 오프-바이-원입니다. 손상 범위는 ekf_comparison.py:143의 '추정err' 열과 MEMORY의 RTK 추정정확도 수치.
(b) estimator.py:133-135의 H가 δp·δv만 관측하고 update_accel의 H(estimator.py:237 `_skew(g_body_expected)`)는 rank 2라 요와 요축 자이로 바이어스가 관측 불가입니다. sensors.py에 지자기계/헤딩 측정이 아예 없습니다. 20초 개루프에서 요 σ가 25°까지 커지고 P 자이로바이어스 z축이 다른 축보다 2~3자릿수 큽니다. **단** 실제 실행 경로(65초 폐루프)에서는 자세 오차 최대 2.16°로 유계이고(폐루프 자세 피드백 + 기동 중 수평 비력이 부분 관측성 제공), att_rmse_deg는 5초 호버 자체테스트에서만 출력됩니다. 원 리뷰어의 '세 시드 모두 20~24°'는 재현되지 않았고(시드에 따라 1.5°~46°) 인용하면 안 됩니다.
(c) ekf_comparison.py:48 `noise_vel=gps_noise_pos / 3.0` — 표준 GPS(1.5 → 0.5 m/s)에는 맞지만 RTK(0.02)에 외삽하면 6.7 mm/s가 됩니다. COTS 도플러 스펙(u-blox F9P급 0.03~0.05 m/s) 대비 5~8배 낙관입니다. mission_sim.py:811을 통해 대표 결과 경로에 들어갑니다. 감도 실측: rmse_z 0.0627 → 0.0662(+5.6%). 단 '비물리적'은 과장이고(TDCP 기반 RTK 속도는 mm/s급이 실제로 가능), 낙관 편향은 RTK 쪽에만 걸리며 RTK vs 표준GPS 순위는 안 뒤집힙니다.
(d) estimator.py:335 `Q = np.diag(self._Q_diag * dt)`가 이산 표본 σ(sensors.py:145가 매 스텝 더하는 0.02)에 dt만 곱해 분산 기준 1/dt(=1000)배 과대합니다. 그런데 '고치면' 바이어스 추정오차가 0.043 → 0.175 m/s²로 오히려 나빠집니다. 즉 사실상 튜닝 파라미터로 동작 중이고, estimator.py:80('센서 스펙과 매칭')과 102('연속시간 → 이산: Q_d ≈ Q_c*dt') 두 주석이 서로 모순됩니다.

**왜 중요한가**

**전부 시뮬 리포트 정확도 문제입니다.** ros2_ws 전체에 estimator/ESKF 문자열이 0건이고 offboard_node.py:162-164가 `/fmu/out/vehicle_odometry`를 구독하므로 실기 항법은 PX4 EKF2가 담당합니다. 기체 손실 경로가 아닙니다. 게다가 (a)의 방향은 보수적입니다 — RTK를 실제보다 13배 나쁘게 보이게 하므로 고치면 RTK 채택 결론이 오히려 강해집니다.

다만 정직하게 덧붙일 것: 검증자가 확인한 바로, results/MISSION_ANALYSIS.md의 Hybrid 1.752/1.670은 **현행 코드로 재현되지 않습니다**(동일 조건 참값 3회 재실행 시 rmse_z=2.774111로 비트 동일). hybrid_comparison.py가 2026-07-12와 08-04에 변경됐기 때문입니다. 이 그룹의 조치보다 **헤드라인 수치 재실행 갱신이 우선순위가 높습니다.**

**수정**

1) (a): predict 이전의 get_state()를 로깅에 쓰거나 로깅 인덱스를 k로 맞출 것. 리뷰어 제안대로 '측정→update→predict'로 바꾸면 이번엔 x_hat이 t_{k+1} 추정치가 되어 t_k 라벨로 제어기에 들어가는 1 ms 선행 불일치가 생깁니다. 어느 쪽이든 제어 지표는 불변(rmse_z 0.0627 확인).
2) (b): '요는 관측 불가'를 estimator.py 독스트링에 명시 + P[8,8]/P[14,14] 상한. compute_estimation_metrics의 att_rmse_deg를 롤·피치/요로 분리하는 제안은 효용이 거의 없습니다(5초 호버 자체테스트에서만 출력).
3) (c): create_sensors에 noise_vel을 독립 인자로 분리하고 RTK는 데이터시트 값 명시. 기존 비교를 위해 0.00667과 0.05 두 값으로 한 번씩 돌려 MEMORY.md에 **둘 다** 기록하는 것이 정직합니다.
4) (d): 값을 바꾸지 말고 estimator.py:80과 102의 주석을 실제 의미('이산 표본 σ를 그대로 쓰는 튜닝값')로 정정할 것. 지표 개선이 없는 변경을 감수하면 기존 EKF 실험 재실행 + MEMORY 갱신 비용만 발생합니다.

---

### Issue 17 — ⚪ LOW · 신뢰도 high

**NMPC 초기추측 — 주요 부분은 이미 수정됨, 남은 것은 상태 초기추측이 동작점과 무관하다는 점**

- 파일: `hybrid_comparison.py`, `nmpc.py`
- 원 지적 ID: `nmpc-indi#1`, `nmpc-indi#4`

**무엇이 잘못됐나**

**nmpc-indi#1(VirtualNMPC w0 쿼터니언 [0,0,0,0])은 이미 수정되어 HEAD에 반영돼 있습니다.** 제가 `git show HEAD:hybrid_comparison.py`로 직접 확인했습니다:
```
157:            _xg = [0.0]*nx
158:            _xg[6:10] = [1.0, 0.0, 0.0, 0.0]   # 유효 단위 쿼터니언(호버) 초기추측
159:            w0 += _xg                          # (nmpc.py와 동일 픽스 — cold 솔브 개선)
```
검증자도 리포트 말미에 '검증 도중 작업 트리에 이 픽스가 이미 반영되었습니다'라고 적었습니다. 이 항목은 종결 처리하십시오.

남는 것은 nmpc-indi#4입니다. 두 NMPC 모두 shooting node의 상태 초기추측이 p=0, v=0, n=0(쿼터니언만 단위)이라 실제 동작점(z=50, vx=70, n≈645)과 무관합니다. 검증자가 조합 실험으로 원 지적의 진단을 반증했는데 이게 중요합니다 — 수렴을 좌우하는 것은 제어 초기추측(nmpc.py:123 `w0 += [600.0] * nu`)이 아니라 **상태 초기추측**입니다:
  U=600,     X=0/q(현행) : Maximum_Iterations_Exceeded, iter=30
  U=u_trim,  X=0/q(제안) : Maximum_Iterations_Exceeded, iter=30, 오히려 악화
  U=600,     X=x0        : Solve_Succeeded, iter=6, 정확히 u_trim
따라서 제안된 수정(`w0 += list(self.u_ref)`)만 적용하면 문제가 해결되지 않습니다.

**왜 중요한가**

폐루프 영향이 첫 20 ms 한 주기로 한정됩니다(70 m/s에서 max|dz|=0.0194 m, max|ω_y|=0.0510 rad/s). NMPCController는 MEMORY.md 기준 'NMPC 단독' 폐기 결정으로 실험 비교 베이스라인 용도이고, 확정 경로인 ProperHybrid와 offboard_node.py:615-626은 VirtualNMPC만 씁니다. 실기 손실 경로가 아닙니다.

**수정**

1) `_build_nlp`를 파라미터화하지 말고, 첫 호출 시 현재 상태 x13을 모든 shooting node의 w0에 복사하도록 바꿀 것(두 파일 공통). 검증상 이 방식이 5~6 iter 수렴을 만듭니다.
2) 하드코딩 600.0은 vehicle_params 기반 n_hov(=571.9)나 u_ref로 교체 — Issue 12(단일 출처)와 같은 맥락이므로 함께 처리하면 됩니다.
3) reset() 후에도 같은 경로를 타므로, Issue 4에서 노드가 reset()을 호출하도록 고친 뒤 SITL에서 재확인할 것.

---

### Issue 18 — ⚪ LOW · 신뢰도 high

**미배선·진단 코드가 실기 빌드에 그대로 남아 있다 (test_motor 안전장치 우회, fallback_controller 복귀 판정)**

- 파일: `ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py`, `fallback_controller.py`
- 원 지적 ID: `ros2-px4#8`, `safety-fallback#6`

**무엇이 잘못됐나**

(a) offboard_node.py:254의 test_motor는 read_only가 아닌 ROS 파라미터를 **매 루프 다시 읽고**(패키지 전체에 ParameterDescriptor 사용 0건), 279행 `self._publish_motors(u_test)` 후 284행 return으로 336행 `self.safety.check(u_raw, state=x)`를 통째로 건너뜁니다. 비행 중 `ros2 param set /offboard_controller test_motor 0`만으로 NaN 검사·변화율 제한·틸트/고도 보호를 전부 우회한 개루프 명령이 즉시 주입됩니다. 진폭은 572+150=722이고 frame_utils.py:180이 0~1로 클립하므로 유계입니다('무제한'은 과장). 별개로 268행 `u_test = np.full(4, base)`(길이 4)인데 271-275행이 tm==10/11/12만 처리하고 278행 `else: u_test[tm] += bump`가 tm=4~9, 13 이상을 그대로 인덱싱해 IndexError → rclpy.spin 사망 → main의 except KeyboardInterrupt가 못 잡고 finally의 shutdown_disarm만 실행됩니다.
  **이것은 버그가 아니라 의도된 진단 기능입니다** — 95-97행 주석이 '컨트롤러/안전장치 우회 … gz 기울기로 부호 실측'이라고 명시합니다. 실기 빌드 전 제거/가드 대상입니다.
(b) fallback_controller.py:165 `return z_err < 2.0 and omega_mag < 1.0`이 생성자 파라미터 `self.z_err_limit`(66행) 대신 리터럴을 씁니다. 폐백 판정은 136행에서 파라미터를 쓰므로, 사용자가 z_err_limit=1.0으로 **조이면 복귀 문턱(2.0)이 폴백 문턱(1.0)보다 느슨해지는 역설**이 생깁니다. 161행 `vel_mag`는 계산만 되고 return 식에 없는 죽은 지역변수입니다.
  단 원 리뷰어의 '히스테리시스 0 → 무한 왕복'은 과장입니다 — 100행 쿨다운 2초와 130행 복귀 후 1초가 강제되어 최소 3초 주기 이하로는 전환이 불가능하고, 이 시간 히스테리시스는 모듈 독스트링(15-16행)에 명시된 의도된 설계입니다. HybridWithFallback은 offboard_node._create_controller의 어느 분기에도 없어 실기 미배선이고, 유일 사용처 legacy/monte_carlo.py:92가 기본값 2.0을 써서 현재 오작동은 없습니다.

**왜 중요한가**

둘 다 지금 나는 버그가 아니라 '실기로 넘어가기 전에 정리해야 할 것'입니다. (a)는 사람이 근처에 있는 실기 시험에서 파라미터 오타(`test_motor:=4`) 하나로 ARM 직후 노드가 죽는 경로가 있고, (b)는 HybridWithFallback을 실제로 배선하는 시점(MEMORY.md가 '확정'으로 적은 안전 아키텍처)에 파라미터를 조이는 순간 드러납니다.

**수정**

(a) test_motor를 __init__에서 한 번만 읽어 `self._test_motor`로 고정(런타임 변경 불가), 값이 {0,1,2,3,10,11,12}에 없으면 생성 시점에 ValueError로 거부. 최소한 오픈루프 명령도 `self.safety.check()`의 변화율 제한만은 통과시킬 것. 실기 빌드에서는 블록 전체를 별도 진단 노드로 분리. 참고: 271행 주석의 '순수 롤(+)'을 FRD 기준으로 틀렸다고 볼 수는 없습니다 — 상태 쿼터니언이 NWU(z-up) 기준이라 '우측 상승 = 롤(+)'이 맞습니다(검증자 정정). 이 주석은 건드리지 마십시오.
(b) 복귀 임계를 `z_err < 0.5 * self.z_err_limit`처럼 폴백 임계에서 파생시켜 명시적 히스테리시스(최소 2:1)를 만들 것. vel_mag를 return 조건에 포함하거나(예: `and vel_mag < 150`) 계산 자체를 제거해 의도를 코드에 드러낼 것. HybridWithFallback을 실기에 배선할 때 함께.

---

## 6. 기각된 지적 15건

검증 단계에서 반박된 항목입니다. **같은 오독을 되풀이하지 않기 위해 남깁니다.**
특히 좌표계·부호 관련 오독 패턴을 주목하십시오.

<details>
<summary><code>plant-physics#8</code> — trim.py:55 — build_trim_state 가 잔차 함수 안에서 로터 속도를 클리핑하기 때문에 포화 시 잔차가 n_eq/dn 에 대해 평평해지고, 반환되는 n_eq/dn 과 state[13:1</summary>

**기각 근거**

코드 인용(trim.py:55-56 `n_front = np.clip(n_eq + dn, ...)`, `n_rear = np.clip(n_eq - dn, ...)`)은 사실이지만, 실패 시나리오 '조용히 잘못된 트림점을 반환한다'는 실행 경로 확인 결과 성립하지 않습니다.

1) 조용하지 않습니다. trim.py:78-81 이 `sol, info, ier, msg = fsolve(..., full_output=True)` 로 받아 `if ier != 1: print('[경고] 트림 수렴 실패')` 를 출력하고, trim.py:95-96 이 반환 직전 클립 후 상태 x_trim 으로 잔차를 재계산해 'residual' 로 내보냅니다. trim.py:145 `converged = trim['residual'] < 1e-4` 가 이 값을 게이트로 씁니다. 실제로 105 m/s 를 돌려보니 경고 출력 + res=1.77e-01 이 그대로 나왔습니다.

2) 반환 'state' 는 잘못된 점이 아닙니다. residual(trim.py:64-72)도 반환 잔차(trim.py:95-96)도 똑같이 build_trim_state 를 거친 클립 후 상태를 평가합니다. 따라서 converged 로 통과한 x_trim 은 자기 자신에 대해 진짜 평형점입니다. 'state' 와 'control'(trim.py:87 `u_trim = x_trim[13:17].copy()`)은 서로 완전히 일관됩니다.

3) n_eq/dn 불일치는 하류 소비자가 없습니다. 저장소 전체에서 trim['n_eq'] / trim['dn'] 을 읽는 곳은 trim.py 내부의 print_trim(116-125)과 trim_speed_sweep(148-149) 뿐입니다. controller.py:415 `x_t, u_t = trim['state'].copy(), trim['control'].copy()`, gust_comparison.py:152, hybrid_comparison.py:432, offboard_node.py:596 등 실제 제어기 경로는 전부 state/control 만 씁니다.

4) 게다가 그 '불일치'는 의도적이고 기능상 필수입니다. trim.py:147 주석 '로터 포화 확인' 대로, 포화 판정(trim.py:150-151 `n_max_actual = max(abs(n_rear), abs(n_front))`, `saturated = n_max_actual > params['n_max'] * 0.95`)은 클립 전 값을 써야만 성립합니다. 리뷰어의 제안대로 반환 dict 를 클립 후 n_front/n_rear 로 통일하면 클립 후 값은 정의상 항상 n_max 이하이므로 saturated 가 영원히 False 가 되어 포화 감지가 죽습니다. 즉 제안된 수정이 오히려 회귀를 만듭니다.

5) 클리핑 제거 제안도 위험합니다. dynamics.py:129-130 `Ti = p['k_T'] * ni**2 * fac` 는 n 에 대해 우함수라 음의 n 에서도 양의 추력이 나오고, dynamics.py:125-127 은 n_rps<0 이면 J<0 → fac>1 이 되어 추력이 오히려 증폭됩니다. n_min=0 클립은 이 비물리 분기를 막는 가드 역할을 합니다.

6) 도달 가능성도 낮습니다. 현 파라미터에서 요구 n 은 85 m/s 933.6, 100 m/s 1332.3 으로 n_max=1800 과 n_min=0 양쪽에서 멀어, 클립이 발동한 사례가 없습니다.

남는 실질 이슈는 '하류 호출부(controller.py:414 등)가 trim['residual'] 을 검사하지 않는다'는 것인데, 이는 제출된 지적과 다른 항목이며 클리핑과도 무관합니다. 제출된 형태로는 반박합니다.

</details>

<details>
<summary><code>nmpc-indi#2</code> — hybrid_comparison.py:189 — VirtualNMPC._solve와 NMPCController._solve 모두 IPOPT 반환 상태를 검사하지 않고 해를 그대로 사용하며, 실패해 NaN이 나온 해를 warm s</summary>

**기각 근거**

핵심 주장인 'NaN latching'이 재현되지 않습니다. HEAD 사본(`git show HEAD:hybrid_comparison.py`)으로 다음을 모두 시험했습니다.
- 상태 NaN 주입 위치별(pos_x / vel_x / vel_z / quat 4개 전체 / omega 3개 / 13개 전부), cold-start와 warm-start 양쪽
- NaN 10회 연속 호출 후 정상 복귀
- Inf 주입, 1e300, 속도 1e5 발산 상태
결과: return_status는 `Invalid_Number_Detected` 또는 `Restoration_Failed`가 나오지만, **u_opt와 self.w0가 비유한값이 된 경우는 한 번도 없었고**(모든 케이스 `w0 finite: True`), 바로 다음 정상 호출에서 `Solve_Succeeded, u=[79.419,0,-0,0]`(정확한 트림)로 즉시 복구됐습니다. NMPCController도 동일: NaN 상태 주입 시 u=[600,600,600,600](초기추측 그대로), 다음 호출 정상.
원인은 명확합니다 — CasADi nlpsol은 실패 시 예외를 던지지 않고 '마지막 유한 반복해'를 sol['x']로 돌려줍니다. 따라서 `self.w0 = np.concatenate([w_opt[stride:], w_opt[-stride:]])` (hybrid_comparison.py:194)에 NaN이 저장되는 일이 발생하지 않고, 지적이 주장한 '이후 모든 솔브가 영구히 NaN → reset() 전까지 복구 불가'는 성립하지 않습니다.

부수 주장도 상당수 부정확합니다.
- 'NMPCController는 상태를 _solve_log에 적기만 하고': 사실이나, 이는 감지 수단이 있다는 뜻이고 nmpc.py:211-217 `get_solve_stats()`로 pct_ok를 뽑을 수 있습니다.
- 'VirtualNMPC는 상태를 기록조차 하지 않아': HEAD 기준으로는 맞지만, 현재 작업 트리에는 이미 `status = self.solver.stats().get('return_status','unknown')` / `self.consec_fail` / `last_status` 추적과 fallback_controller.py의 `nmpc_fail_limit` 전환 조건이 들어가 있어 유효하지 않습니다.
- '정상 상태에서도 max_iter=5에서는 모든 솔브가 Maximum_Iterations_Exceeded(호버·20·40 m/s 전부 확인)': 조건부로만 참입니다. 추종 오차가 큰 상태(z_ref와 10 m 차이)에서는 8회 내내 미수렴이지만, 오차가 없는 호버(z_ref 일치)에서는 3번째 솔브부터 `Solve_Succeeded`, 20 m/s는 3번째부터, 40 m/s는 6번째부터 수렴했습니다. '모든 솔브'가 아닙니다.

남는 실질 지적은 '미수렴 해를 상태 검사 없이 쓴다'인데, 이는 findings#1(cold 솔브 미수렴)과 #4가 이미 구체 수치로 다루는 내용이고, 제안된 `if not np.all(np.isfinite(w_opt)): ...` 가드는 발생하지 않는 시나리오에 대한 방어입니다. 안전 이슈로서의 실질 영향이 없어 refuted 처리합니다.

</details>

<details>
<summary><code>nmpc-indi#6</code> — hybrid_comparison.py:295 — ProperHybrid._fallback의 로터 속도 역산이 전진비 계수 fac를 무시하고 n=sqrt(f/k_T)로 계산해, 전진비행 중 폴백이 발동하면 요구 추력보다 적게 낸다</summary>

**기각 근거**

코드 인용 자체는 정확하다(HEAD:295 `n[i] = np.sqrt(max(f_ind[i], 0) / self.p['k_T'])`, 현재 작업트리 319행; controller.py:702-711의 동일 패턴도 실재). 그러나 실질 영향이 없어 반박한다. (1) 주장된 '급상승·급감속 V_axial≈25 → 37% 오차' 시나리오는 실행 경로상 도달 불가다. _fallback 진입로는 두 개뿐인데, LinAlgError 경로는 compute_control_effectiveness가 354행 `ni = max(n_actual[i], 1.0)`로 열을 0으로 만들지 않아 정확한 특이가 되지 않는다(실측 cond(G): 정상 75.07, 로터1 완전정지 4.87e4 — np.linalg.solve는 예외를 던지지 않음). 남은 경로는 `if not self._initialized`(257-261행) 한 스텝뿐이고, 그 시점 상태는 트림/준트림(V_axial=6.03, fac=0.902)이다. 비행 중 재진입도 fallback_controller.py:108-115가 `_check_stable`(z_err<2 m, ω<1 rad/s, 173-180행) 통과 후에만 hybrid.reset()을 부르므로 30도 틸트·V_axial=25 상태에서는 _fallback이 실행되지 않는다. (2) 폐루프 영향을 직접 측정했다. 리뷰어가 제안한 정확한 2차식 해(n=(b+sqrt(b²+4k_T·f))/(2k_T), b=k_T·2πV_axial/(D·J_max))로 _fallback을 교체해 3초 시뮬을 돌린 결과 rmse_z 0.006429 → 0.006443, max_dz 0.007996 → 0.007997, rmse_vx 0.000842 → 0.000844 로 개선이 없다(오히려 노이즈 수준으로 미세 악화). dt=0.001 한 스텝 명령이고 다음 스텝부터는 측정 n 기반 증분 INDI 경로(289-300행)가 즉시 덮어쓰며, 게다가 모터 1차 지연 tau_m 때문에 한 스텝 명령은 실제 n을 거의 못 움직인다. (3) fac는 그 한 스텝에서 지배적 오차도 아니다. 실측한 _fallback 출력은 n=[668.2,668.2,668.3,668.3]인데 트림은 [645.6,645.6,744.3,744.3] — TM=[T_cmd, J·ω̇_des]로 역산하면서 동체 정적 모멘트(dynamics.py:85 `M_static = [0, -xcp*Fz, xcp*Fy]`)를 상쇄하는 트림 차동 Δn(로터당 ±76 rad/s)을 통째로 버린다. 이는 총추력 10% 부족보다 훨씬 큰 근사이며 제안된 수정으로는 해결되지 않는다. 즉 지적이 _fallback의 실제 성격(의도적으로 거친 모델 기반 백업)을 오인해 부차적 항만 결함으로 지목했다. 실기 기체 손실 위험 경로도 아니다.

</details>

<details>
<summary><code>safety-fallback#5</code> — fallback_controller.py:95 — LQR 분기에서는 출력 검증도 예외 포착도 전혀 없어, 폴백 후 LQR이 NaN을 내면 그대로 플랜트로 나가고 영구히 그 상태에 갇힌다.</summary>

**기각 근거**

코드 관찰 자체(fallback_controller.py:95-96 `else:` / `u = self.lqr(t, x)` 에 try/except도 NaN 검사도 없음)는 사실이나, 주장된 실패 시나리오가 모든 실행 경로에서 이미 처리되거나 도달 불가능합니다.

(1) 실행 경로: `grep -rn "HybridWithFallback"` 결과 임포트처는 `legacy/monte_carlo.py:34` 단 하나. ROS2 `offboard_node.py:563 _create_controller`의 분기는 pid/scheduled_pid/lqr/scheduled_lqr/indi/nmpc/hybrid 뿐이고 else는 `raise ValueError(...)` 이므로, HybridWithFallback은 현재 실기/SITL 경로에 아예 존재하지 않습니다. `ros2_ws/.../controllers/fallback_controller.py` 사본도 어디서도 임포트되지 않습니다.

(2) '시뮬/노드가 죽는다' 반박: legacy/monte_carlo.py는 `try: res = simulate_with_ekf(...) ... except Exception: results[name] = float('nan')` 로 트라이얼 단위 예외를 이미 잡고, 그 바로 위에서 `if np.any(np.isnan(res['xs_true'])) or met['max_z_err'] > 50: results[name] = float('nan')` 로 NaN 궤적을 명시적으로 검출해 실패로 기록합니다. 즉 '전체 궤적이 NaN이 되어 조용히 통과'하는 일이 없습니다.

(3) 'LQR이 NaN을 낸다' 전제 반박: ScheduledLQR.__call__(controller.py:483-495)은 `V = self.v_ref[0]`(상수) 보간 → `_interpolate`에서 쿼터니언 재정규화 `if qn > 1e-10` 가드까지 있고, `_compute_error_state`는 유한 입력에 대해 유한, 마지막이 `return np.clip(u, self.p['n_min'], self.p['n_max'])`(controller.py:495, 리뷰어가 쓴 496은 1줄 오차). 즉 x가 유한하면 u는 유한합니다. LQR이 NaN을 내려면 입력 상태 x가 이미 NaN이어야 하고, 그 시점은 플랜트/추정기가 이미 발산해 어떤 출력 검증으로도 복구 불가한 상태입니다. '검증만 있었으면 살았다'가 성립하지 않습니다.

(4) 리뷰어 스스로 인정한 ROS2 경로: offboard_node.py:329-333 `try: u_raw = self.controller(t, x) except Exception as e: ... u_raw = np.full(4, nan)` + offboard_node.py:336 `u = self.safety.check(u_raw, state=x)`. 장차 이 클래스를 노드에 배선하더라도 예외/NaN은 이미 상위에서 잡힙니다.

결론: 방어적 코드 대칭성 개선 제안으로는 타당하나, 실행 경로상 실질 영향이 없어 high는 물론 결함으로 계상할 근거가 없습니다.

</details>

<details>
<summary><code>safety-fallback#7</code> — fallback_controller.py:126 — '극단 출력' 즉시 폴백 검사가 실행 경로상 절대 발동할 수 없어, 채터링 블라인드 구간 1초 동안 유일하게 살아있는 검사는 NaN/Inf뿐이다.</summary>

**기각 근거**

'도달 불가능하다'는 관찰 자체는 정확하지만, 그 결과가 '중복된 방어 코드'일 뿐 런타임 동작 차이가 전혀 없어 결함으로 계상할 수 없습니다.

(1) 도달 불가 근거는 확인됨: hybrid_comparison.py:284 `return np.clip(n_actual + dn, self.p['n_min'], self.p['n_max'])`, 같은 클래스의 예외 경로 `_fallback`도 `return np.clip(n, self.p['n_min'], self.p['n_max'])` 로 끝나고, controller.py:495 ScheduledLQR도 동일 클립. vehicle_params.py:82-83 `'n_min': 0.0, 'n_max': 1800.0`. 따라서 유한값이 -100 미만/2500 초과가 될 경로가 없습니다. 즉 fallback_controller.py:126 `if np.any(u < -100) or np.any(u > 2500):` 를 삭제해도, 유지해도 프로그램 거동은 비트 단위로 동일합니다. 안전 코드의 중복 가드는 결함이 아닙니다.

(2) 리뷰어가 '실제 문제인 이유'로 든 것은 이 죽은 검사가 아니라 :130 `if self._steps_since_switch < self.min_hybrid_steps and self._switch_count > 0: return False` 블라인드 창입니다. 그런데 이 창은 모듈 독스트링 :16 '채터링 방지: 복귀 후 최소 1초는 Hybrid 유지'로 명시된 의도된 트레이드오프이고, 애초에 임계가 -100/2500이 아니라 살아 있었더라도 n_max=1800 포화는 2500을 넘지 못하므로 '포화 발산 미감지'는 이 검사와 인과관계가 없습니다. 즉 지적된 코드를 고쳐도 지적된 피해가 사라지지 않습니다 — 근거와 결론이 연결되지 않습니다.

(3) 실행 경로: HybridWithFallback은 legacy/monte_carlo.py:34 에서만 임포트되며, offboard_node.py:563 `_create_controller` 의 분기(pid/scheduled_pid/lqr/scheduled_lqr/indi/nmpc/hybrid, else는 ValueError)에 없어 SITL/실기 경로에 배선돼 있지 않습니다. '85 m/s 순항 중 85 m를 흘려보낸다'는 현재 존재하지 않는 경로에 대한 서술입니다.

포화 감지 추가는 별건의 개선 제안으로 다룰 수는 있으나, 제출된 형태('죽은 검사 = medium 결함')는 실질 영향 없음으로 반박합니다.

</details>

<details>
<summary><code>safety-fallback#8</code> — ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/safety.py:65 — _last_valid_u를 hover_cmd로 초기화하는데 ARM 전에는 실제로 0이 발행되고 있어, ARM 직후 첫 스텝에서 변화율 제한이 사실상 무력화된다.</summary>

**기각 근거**

safety.py:65 `self._last_valid_u = self.hover_cmd.copy()` 와 offboard_node.py:245 `self._publish_zero_motors()` 가 guard를 우회한다는 관찰은 사실이나, 주장된 실패(‘ARM 직후 첫 스텝에 396~756 발행 = 변화율 제한 무력화’)는 같은 파일의 뒤쪽 레이어 때문에 실제로 발생하지 않습니다.

(1) 레이어 순서가 결정적입니다. check()는 :114-125 순서로 `_check_nan → np.clip(u, 0.0, n_max) → _rate_limit → _check_state` 를 적용합니다. 즉 변화율 제한이 먼저, 고도 하한이 나중입니다. `_check_state` :207-217
```
altitude = state[2]  # NWU z-up
if altitude < self.min_altitude:      # min_altitude = 1.0 (:78)
    ...
    u = np.maximum(u, self.hover_cmd)
```
지상(z≈0, EKF 원점이 이륙 지점)에서는 rate limiter 결과가 무엇이든 최종 출력이 576 이상으로 덮어써집니다. 제안된 수정(_last_valid_u=np.zeros(4))을 적용해도 첫 스텝 발행값은 max(180, 576)=576 으로 사실상 동일합니다 — 수정이 문제를 해결하지 못하므로 원인 지목이 틀렸습니다. 제어기가 600을 요구할 때 현행 600 vs 수정 후 576, 차이는 24 rad/s에 불과합니다.

(2) '변화율 제한이 무력화'라는 표현도 부정확합니다. :163-165 `max_delta = self.max_rate * self.dt` → 180, `delta_clipped = np.clip(delta, -max_delta, max_delta)` 는 그대로 동작합니다. 잘못된 것은 기준점이 0이 아닌 576이라는 점뿐이고, 그 오차는 최대 576 rad/s = 램프 약 3스텝(32 ms) 분량입니다.

(3) 'ARM 직후 첫 스텝'이라는 타이밍 전제도 어긋납니다. offboard_node.py:238 `self._arm_requested = True` 는 ARM **명령 송신** 시점에 세워지고 Phase 2(:248~)가 그때부터 돌기 시작합니다. PX4가 실제로 armed 되기 전부터 safety.check가 매 스텝 호출되며 :131 `self._last_valid_u = u.copy()` 로 갱신되므로, 실제 arming 순간에는 이미 실제 명령값으로 수렴해 있습니다. 반대로 실제 모터의 0→호버 점프는 PX4가 disarmed 동안 액추에이터 명령을 무시하다가 arming 시 반영하기 때문에 발생하는 것이고, 이는 SafetyGuard의 rate limiter가 통제할 수 있는 대상이 아닙니다.

(4) '자동 이륙' 부분은 이 지적(_last_valid_u 초기화)과 인과가 없습니다. 4×576에서 T = 4·6.0e-5·576² = 79.6 N vs mg = 8.0·9.81 = 78.5 N 계산 자체는 맞지만, 이는 :217 저고도 하한이 의도적으로 '호버 이상의 추력 보장'(:216 주석)을 하기 때문이며 초기화 값과 무관합니다. 저고도 하한의 적정성을 문제 삼으려면 별건으로 제출해야 합니다.

확신도 기준상 실질 영향 없음 + 원인 오지목이므로 refuted 처리합니다.

</details>

<details>
<summary><code>safety-fallback#10</code> — fallback_controller.py:134 — 상태 x에 NaN이 있으면 z_err/vel_mag 비교가 모두 False가 되어 감지기를 조용히 통과한다 (전형적인 NaN 전파 함정).</summary>

**기각 근거**

세 가지 이유로 반박한다.

(1) 리뷰어가 제시한 유일한 '실제 문제가 되는' 시나리오가 자기모순이다. 리뷰어는 `x[0:2] 위치만 NaN`인 경우 감지기가 무력하다고 했으나, 지적한 두 비교식은 fallback_controller.py:135 `z_err = abs(x[2] - self.z_ref)`와 :140 `vel_mag = np.linalg.norm(x[3:6])`로 x[2]와 x[3:6]만 쓴다. x[0:2]만 NaN이면 z_err도 vel_mag도 NaN이 아니고 두 검사는 정상 작동한다. 즉 '비교가 모두 False가 된다'는 실패 모드가 제시된 시나리오에서 발생하지 않는다.

(2) x[2] 또는 x[3:6]이 NaN인 경우는 리뷰어 본인이 인정했듯 :123 `if np.any(np.isnan(u)) or np.any(np.isinf(u)): return True`가 먼저 잡는 정상 경로다. 코드 대조로도 이 우선순위는 지켜져 있다 — :121-124 주석 `# ── 무조건 검사 (채터링 가드보다 우선) ──`, :130 채터링 가드가 그 뒤에 온다(프로젝트 알려진 함정 '폴백: NaN 검사 > 채터링 가드' 이미 반영됨). impact_verified=false도 리뷰어 스스로 붙였다.

(3) 도달성: HybridWithFallback은 활성 실행 경로에 없다. `grep -rn HybridWithFallback --include=*.py .` 결과 인스턴스화는 legacy/monte_carlo.py:92 한 곳뿐이고(legacy/ = 프로젝트 규약상 안 쓰는 코드), 메인 진입점 mission_sim.py의 import 목록(17-27행)에 fallback_controller가 없다. ROS2 사본 ros2_ws/.../controllers/fallback_controller.py도 offboard_node.py의 `_create_controller`(563-632)가 pid/scheduled_pid/lqr/scheduled_lqr/indi/nmpc/hybrid만 생성하므로 임포트조차 되지 않는다 — 실기 비행 경로와 무관하다.

추가로, x[0:2]는 VirtualNMPC 비용(hybrid_comparison.py:154-164, `e_v = X_k[3:6]-v_ref`, `e_z = X_k[2]-z_ref`)에도, 플랜트 동역학의 되먹임에도 들어가지 않아 물리적으로도 무해한 성분이다. 결국 '긍정형 비교로 쓰라'는 일반적 방어 코딩 권고일 뿐, 입증된 결함이 아니다.

</details>

<details>
<summary><code>estimator#3</code> — sensors.py:161 — GPS 측정 지연이 전혀 모델링되어 있지 않아, RTK(sigma=0.02 m)의 이점이 실제 수신기에서 재현될 수 없는 조건에서 측정되고 있다.</summary>

**기각 근거**

인용문 sensors.py:161 `- 지연: v1에서는 무시 (나중에 추가)` 는 실제로 존재하고, GPSSensor.measure(sensors.py:187-206)에 지연 버퍼가 없는 것도 사실이다. 그러나 이것은 결함이 아니라 코드가 자기 문서와 정확히 일치하는, 명시적으로 유예 선언된 v1 범위 결정이다. 리뷰 지적이 아니라 기능 요청에 가깝다.

반박 근거 4가지:
(1) 지연은 두 GPS 구성에 공통(common-mode)이다. ekf_comparison.py:44-50 의 GPSSensor 생성은 [1] 표준(ekf_comparison.py:198 gps_noise_pos=1.5)과 [2] RTK(ekf_comparison.py:217 gps_noise_pos=0.02)가 동일 클래스·동일 지연 0을 쓰므로, 지연을 넣어도 두 구성에 같은 v·latency 편향이 더해진다. 'RTK 확정'의 근거인 상대 비교(RTK 대비 표준GPS 마진)는 흔들리지 않는다.
(2) 센서 = RTK GPS 는 프로젝트 확정 사항이며 재론 대상이 아님.
(3) 실기 항법은 이 ESKF가 아니다. ros2_ws 전체 grep 에 estimator/ESKF 문자열 없음이고 offboard_node.py:162-164 는 `VehicleOdometry, '/fmu/out/vehicle_odometry'` 를 구독한다 — 즉 PX4 EKF2가 지연시각 지평선에서 융합을 수행한다. '하드웨어로 그대로 이전되지 않는다'는 전제 자체가 실기 경로에 성립하지 않는다.
(4) 리뷰어 스스로 impact_verified=false 로 두었고, '100 ms면 7 m' 는 측정이 아니라 finding 1의 1 ms 결과를 선형 외삽한 값이다. 게다가 finding 1에서 내가 확인했듯 그 1 ms 편향조차 제어기가 받는 상태에는 나타나지 않고(제어기 입력 편향 +0.0019 m) 로깅 지표에만 나타났으므로, 같은 메커니즘을 100배로 확대하는 외삽은 근거가 약하다.

</details>

<details>
<summary><code>estimator#6</code> — estimator.py:112 — update_gps의 R_pos/R_vel 기본값이 표준 GPS 값으로 하드코딩되어 있어, 인자를 빠뜨린 새 호출부가 RTK 센서에 75배 큰 R을 조용히 사용하게 된다.</summary>

**기각 근거**

인용은 정확하다(estimator.py:112 `def update_gps(self, pos_meas, vel_meas, R_pos=1.5, R_vel=0.5):`). 그러나 이 기본값이 실행 경로에서 도달 가능한 지점이 없다. 저장소 전체(legacy/ 포함) `grep -rn "update_gps" --include='*.py' .` 결과는 정의부(estimator.py:112)와 호출부(ekf_sim.py:103) 단 두 줄뿐이고, 그 유일한 호출부가 ekf_sim.py:103-105 `eskf.update_gps(sensor_data.gps_pos, sensor_data.gps_vel, R_pos=sensors.gps.noise_pos, R_vel=sensors.gps.noise_vel)` 로 센서 실제 sigma를 명시 전달한다. 리뷰어 본인도 '알려진 함정 3은 현재 회귀하지 않았다'고 인정했다. 즉 이것은 '호출부에서 이미 처리됨' + '도달 불가능한 분기'에 정확히 해당하며, 남는 것은 '미래에 누군가 인자를 빠뜨릴 수 있다'는 가정뿐이다. 그 가정에 붙인 실패 시나리오(0.01 m급 -> 0.75 m급)도 실제 코드가 아니라 가상의 새 호출부를 전제로 한 수치다. 또 기본 인자를 두는 것 자체는 결함이 아니다 — 같은 파일 estimator.py:190 `def update_accel(self, acc_body, R_acc=2.0):` 는 ekf_sim.py:102에서 의도적으로 기본값을 쓰고 있고(적응 게이팅 estimator.py:230 `scale = 1.0 + 20.0 * mag_deviation**2 + 5.0 * omega_mag**2` 로 스케일됨), 리뷰어도 그쪽은 문제 없다고 스스로 제외했다. 동일한 API 패턴을 한쪽만 결함으로 부르는 것은 일관성이 없다. 현존 버그가 아닌 방어적 리팩터링 제안이므로 지적으로는 반박한다.

</details>

<details>
<summary><code>estimator#7</code> — ekf_comparison.py:148 — 저하율의 분모가 되는 참값(perfect) 실행에는 발산 검사가 없어서, 참값 쪽이 발산하면 무의미한 기준값 위에서 퍼센트가 계산되어 표에 정상 숫자처럼 찍힌다.</summary>

**기각 근거**

인용 자체(ekf_comparison.py:148 `deg_z = ((me['rmse_z'] - mp['rmse_z']) / max(mp['rmse_z'], 1e-6)) * 100`, 151-152의 발산 체크가 res_e만 검사)는 사실이다. 그러나 핵심 실패 시나리오가 코드상 성립하지 않는다. (1) 저하율 0% 오독은 산술적으로 불가능하다 — deg_z는 else 분기(ekf_comparison.py:160-166)에서만 출력되고, 그 분기에 들어갔다는 것은 ekf_comparison.py:151-152 `np.max(np.abs(res_e['xs_true'][:, 2] - z_ref)) > 50` 이 거짓, 즉 EKF런의 최대 z 편차 ≤ 50이라는 뜻이다. RMSE ≤ max편차이므로 me['rmse_z'] ≤ 50이 강제된다. 참값이 진짜 발산해 mp['rmse_z']가 '수십~수백 m'이면 me < mp가 되어 deg_z는 -50% ~ -100% 사이의 큰 음수로 찍힌다. 리뷰어가 주장한 '0%에 가깝게 나와 노이즈에 강함으로 읽힌다'는 결과는 나올 수 없다. (2) 은닉되지도 않는다 — ekf_comparison.py:161-163이 참값 행에 `{mp['rmse_z']:>8.4f}` 로 원값을 그대로 찍고, 종합표 ekf_comparison.py:269도 `{z_perf:>8.4f}` 로 찍는다. 통상값이 0.0x인 칸에 100 단위 숫자가 뜨면 눈에 바로 띈다. (3) 임계값 경계 지적도 도달 불가능하다 — dynamics.py의 step()(235-262행)에는 지면 접촉/z 클램프가 없고 클립은 `xn[13:17] = np.clip(xn[13:17], self.params['n_min'], self.params['n_max'])` (로터뿐)이다. 추락 궤적은 z=0을 그냥 통과해 음수로 계속 내려가므로 |z-50| > 50이 즉시 성립한다. `>= 50`이어야만 잡히는 경우는 전체 궤적 최대편차가 정확히 50.0인 측도 0의 사건뿐이다. mission_sim.py:201 `_z_dev > 50.0`도 같은 이유로 무해하다. (4) 범위: ekf_comparison.py의 print_comparison_table/main은 standalone 리포트 경로이고, 다른 곳(mission_sim.py:27, 810)은 `_reset_controller`와 `create_sensors`만 임포트한다. 제어/추정 로직이 아니다. impact_verified=false이고 재현도 못 한 상태이므로, 남는 것은 '진단 스크립트를 더 방어적으로 짜면 좋겠다'는 수준의 제안이다.

</details>

<details>
<summary><code>mission-sim#8</code> — mission_sim.py:438 — diagnose_deceleration이 어디서도 호출되지 않는 죽은 코드이며, 결론 출력이 n_trials와 무관하게 '30회'와 '22m'를 하드코딩하고 있다.</summary>

**기각 근거**

인용 문자열 자체는 실제와 일치한다(mission_sim.py:437-439). 그러나 현행 코드에서 어떤 실패도 발생하지 않는다.

첫째, 도달 불가다. 저장소 전체 `grep -rn "diagnose_deceleration" .` 결과가 정의부 mission_sim.py:315 단 한 줄이다. main()(590-853줄)에도, `if __name__ == '__main__': main()`(856-857줄)에도 호출이 없고, 문서/스크립트에도 참조가 없다.

둘째, 기본값이 하드코딩 숫자와 일치한다. 시그니처가 `def diagnose_deceleration(n_trials=30, seed=0, nmpc_N=20)`이므로 인자 없이 부르는 유일한 호출 형태에서는 '30회'가 정확한 표기다. 실제로 과거 실행 산출물 3건(results/decel_diag_N10.txt:5, decel_diag_N20_rerun.txt:5, decel_fix_result.txt:5)이 모두 `Hybrid 감속 폭발 진단 (30회)`로 n_trials=30 기본값 실행이며, 유일하게 위험 케이스가 있었던 decel_fix_result.txt:34는 `판정: ... 6/30 (20%)`로 else 분기(443줄)를 탔다 — 문제의 437줄 n_danger==0 분기가 잘못된 표본 수를 출력한 기록이 한 번도 없다.

셋째, '22m' 역시 도달 불가 분기 안의 서술형 주석 성격 문자열이라 현재 어떤 결과도 오염시키지 않는다. 즉 실패 시나리오가 '누군가 나중에 n_trials!=30으로 직접 호출하면'이라는 가정에만 의존하며, 리뷰어 스스로 impact_verified=false로 표기했다. 결함이 아니라 정리 제안(legacy/ 이동 또는 argparse 연결)으로 분류하는 것이 맞다.

</details>

<details>
<summary><code>sweep-test#8</code> — sweep_plots.py:193 — 스케줄 LQR의 게인 테이블에만 85 m/s를 추가한 뒤, 발표용 헤드라인 비율(7.9배)을 하필 그 85 m/s에서 계산해 평가 격자와 스케줄 격자가 정렬된다.</summary>

**기각 근거**

세 가지 근거로 반박한다. (1) 제안된 수정의 사실 전제가 틀렸다 — '(현재는 소스 docstring에만 있음)'이라고 했지만 _conditions()(456-457행)가 '"LQR scheduled = V_table 0~80(+85) 선형 보간 (85 추가: 기본 테이블은 80에서 잘려 85 m/s에 인공 오차 발생)"' 를 산출물에 찍고, results/sweep_knee.txt와 results/sweep_fixed_vs_scheduled.txt 조건 블록 양쪽에 실제로 그 줄이 들어 있음을 파일에서 확인했다. 이미 공시되어 있다. (2) 85 추가는 격자 맞추기가 아니라 구현상 필수 보정이다. controller.py:437 `V_c = np.clip(V, self.V_table[0], self.V_table[-1])` 때문에 기본 테이블(controller.py:406 `V_table = np.arange(0, 90, 10)` = 0~80)이면 V=85가 80으로 클립되어 스케줄 LQR이 80 m/s 트림을 목표로 삼는다 — 이건 게인 스케줄링 기법의 성질이 아니라 테이블이 비행영역을 안 덮은 아티팩트이고, 실기 스케줄러라면 당연히 포락선을 덮는다. 게다가 30/40/50/60/70/80 정렬은 이 스크립트가 고른 게 아니라 기본 테이블 10 m/s 간격에서 상속된 것이고, 추가된 점은 85 하나뿐이다. (3) 85는 유리한 내부 격자점을 골라잡은 게 아니라 스윕 종점이자 프로젝트 목표속도(85 m/s ≈ 306 km/h)다. 비율 바로 위 _table 출력이 12개 속도 전 행을 그대로 싣고 있어(스케줄 RMSE_vx 0.86/0.95/0.92/0.98/0.90/0.95/0.85/0.88/0.77/0.81/0.68/0.62 — 지적한 지그재그가 독자에게 그대로 노출됨) 은폐가 없다. 지그재그 관측 자체는 사실이지만 크기가 ~0.1 m/s 수준이고, 80 m/s(격자점) 기준으로 계산해도 4.18/0.68 = 6.1배로 결론이 유지된다. 리뷰어 본인도 '결론 오류가 아니다', 긴 창에서는 격차가 오히려 커진다고 적었다. 실질 결함이 없고 공시도 이미 되어 있으므로 반박.

</details>

<details>
<summary><code>ros2-px4#6</code> — scripts/sync_controllers.sh:36 — `sed -i -E`는 GNU sed 전용 문법이라 코드 작성 환경인 macOS(BSD sed)에서 오류로 죽는데, 그 시점에 cp는 이미 실행된 뒤라 controllers/con</summary>

**기각 근거**

인용된 코드가 현재 파일에 존재하지 않는다. scripts/sync_controllers.sh 34-39행의 실제 내용은

    cp "$src" "$DST/$m.py"
    # 로컬 모듈 절대 import → 상대 import (들여쓰기 보존)
    # perl 사용: BSD(macOS)/GNU(Ubuntu) sed의 -i·정규식 비호환을 피함
    perl -pi -e \
      "s/^(\s*)from (${MODULES// /|}) import/\1from .\2 import/" \
      "$DST/$m.py"

으로, 36행 주석이 바로 이 BSD/GNU 비호환 문제를 명시적으로 회피했다고 적고 있다. `git diff -- scripts/sync_controllers.sh` 로 확인하면 HEAD(커밋본)에는 `sed -i -E`가 있었고 워킹트리에서 이미 `perl -pi -e`로 수정된 상태다 — 즉 리뷰어는 수정 전(커밋된) 버전을 인용했고, 지적된 결함은 리뷰 대상 코드에서 이미 해소돼 있다.

또한 대체 구현이 실제로 macOS에서 동작함을 직접 실행해 확인했다. BSD sed는 리뷰어 말대로 `sed: 1: "s/^(x)$/\1y/": \1 not defined in the RE`로 exit 1이지만, 현재 코드의 perl 명령은 macOS에서 exit 0이며 `from dynamics import ...` → `from .dynamics import ...`, 들여쓰기 있는 `    from trim import ...` → `    from .trim import ...`로 정상 변환되고 `import numpy as np`는 건드리지 않는다(perl은 `\s`도 지원하므로 [[:space:]] 치환도 불필요). 남은 개선 여지는 cp+in-place 대신 스트림 출력으로 부분 손상을 원천 차단하자는 정도인데, perl 명령이 실패하지 않으므로 실질 영향이 없다.

</details>

<details>
<summary><code>cross-cutting#6</code> — fallback_controller.py:27 — HANDOFF.md와 results/MISSION_ANALYSIS.md가 '확정된 안전장치'로 선언한 HybridWithFallback이 실제로는 어떤 실행 경로에서도 인스턴스화되</summary>

**기각 근거**

핵심 주장이 사실과 다르다. 리뷰어는 '저장소 전체(legacy 제외)를 HybridWithFallback으로 grep하면 정의부와 자기 독스트링, 그리고 문서/동기화 스크립트 목록에만 나온다'고 했으나, 실제 `grep -rn HybridWithFallback` 결과 루트의 test_fallback.py가 이 클래스를 두 번 인스턴스화한다: test_fallback.py:54 `ctrl = HybridWithFallback(hyb, _StubLQR(), z_ref=50.0)` (스텁 단위테스트), test_fallback.py:141 `ctrl = HybridWithFallback(hyb, lqr, z_ref=50.0)` 뒤에 `ts, xs, us = plant.simulate(x0, ctrl, 5.0)` 로 실제 ProperHybrid+ScheduledLQR+AxialDronePlant 5초 스모크까지 돌린다. 즉 dead-code가 아니라 테스트가 붙어 있는 살아있는 모듈이다. 더 결정적으로 이 모듈은 지금 활발히 개발 중이다 — `git status`상 fallback_controller.py는 미커밋 수정(+17줄)으로 `nmpc_fail_limit` 트리거가 추가되었고(`if getattr(getattr(self.hybrid, 'nmpc', None), 'consec_fail', 0) >= self.nmpc_fail_limit: return True`), hybrid_comparison.py도 미커밋 수정으로 VirtualNMPC에 `# 솔버 수렴 추적 (연속 미수렴 → HybridWithFallback의 전환 판단에 사용)` 주석과 함께 `self.consec_fail = 0`이 신설되었다. 폴백을 쓰기 위한 배선이 진행 중인 코드를 '죽은 코드'로 규정한 것은 오독이다. 문서 불일치 주장도 약하다: MISSION_ANALYSIS.md:105는 '폴백 제어기로 충분히 대응 가능'(가능성 서술), HANDOFF.md:264는 '감속 잔존 이슈: MC 3%에서 20m+ (폴백으로 대응)'(대응 방침)로, 어느 쪽도 'MC 수치가 폴백을 포함해 산출됐다'고 주장하지 않는다. MISSION_ANALYSIS.md:141의 `안전장치: HybridWithFallback`은 '### 실기 아키텍처' 절 안에 있어 명시적으로 미래 목표 구성이다. 또 리뷰어가 인용한 HANDOFF.md:254는 실제로는 264행이고, 254행 근처는 acados 문헌조사 내용이다. offboard_node.py:626 `return ProperHybrid(vnmpc, P, dt=dt)`로 폴백 미연결인 것은 맞으나, HANDOFF.md:150~157 기준 SITL은 아직 'LQR/INDI/Hybrid 롤 발산 미해결' 단계라 폴백 래퍼 배선은 로드맵 항목이지 결함이 아니다. dt=0.001 기본값 이슈도 '미래에 dt를 안 넘기고 붙이면'이라는 가정이며, 실제 호출부인 legacy/monte_carlo.py:92는 `dt=dt`를 넘기고 있고 docstring에도 'dt : float 제어 주기 [s]'로 명시돼 있다.

</details>

<details>
<summary><code>cross-cutting#11</code> — model.sdf:16 — 루트 model.sdf의 momentConstant가 0.016으로, vehicle_params의 k_Q/k_T=0.0833 및 HANDOFF.md가 적용했다고 기록한 값과 5.2</summary>

**기각 근거**

수치 관측 자체는 사실이다 — model.sdf:16 `<momentConstant>0.016</momentConstant>`(4개 로터 모두: 16, 33, 50, 67행)이고, dynamics.py:210 `k = params['k_Q'] / params['k_T']` + vehicle_params.py:75-76 `'k_T': 6.0e-5`, `'k_Q': 5.0e-6` → 0.0833이므로 5.2배 차이가 맞다. 그러나 이 파일은 이 저장소에서 실행 경로가 전혀 없다: (1) .gitignore에 `# Gazebo SDF (Ubuntu PX4 폴더에 원본 있음, 여기는 임시 복사본)` 주석과 함께 `*.sdf`가 있고 `git ls-files | grep -i sdf` 결과가 비어 있어 git 미추적이다. (2) 저장소 전체에서 momentConstant를 언급하는 곳은 model.sdf 자신과 HANDOFF.md:118뿐이고, model.sdf를 읽거나 복사하는 스크립트가 없다. (3) SITL 진입점 scripts/sitl_run.sh:176 `setsid bash -c "cd '${PX4_DIR}' && HEADLESS=${HEADLESS} exec make px4_sitl gz_fast_missile"` 는 Ubuntu PX4-Autopilot 트리의 모델을 로드하지 이 파일을 쓰지 않는다. (4) HANDOFF.md:118-119가 이미 `SDF는 momentConstant 0.016→0.0833만 유지(요 권한 정합, 부호 무관)`로 Ubuntu 쪽 수정 완료를 기록했고, 실측 결과 30초+ 안정 호버까지 확인된 상태다. 즉 '이미 다른 곳에서 처리됨 + 죽은 파일' 두 반박 조건에 해당하며, 리뷰어 스스로 impact_verified=false로 두었다. 리뷰어가 덧붙인 timeConstantUp=0.0125/Down=0.025 vs tau_m=0.02 지적도 같은 이유로 무영향이고, 애초에 Gazebo 모터 모델은 상승/하강 시상수를 분리해 받는 구조라 0.02를 사이에 두고 갈라진 값이 반드시 불일치라고 보기도 어렵다. 남는 것은 '오래된 로컬 스냅샷을 지우거나 갱신하자'는 하우스키핑 제안뿐이라 결함으로 인정하지 않는다.

</details>

## 7. 근본 원인

1. **부호·좌표계 규약이 주석과 관례로만 존재하고 코드로 강제되지 않는다.** CLAUDE.md가 '이 프로젝트 최대 함정'이라고 못박았는데도 같은 부류가 계속 나옵니다 — dynamics.py:148과 150이 서로 반대 스핀축을 함의하고(Issue 8), controller.py:291에 최단경로 보정이 없고(Issue 1), 할당행렬↔플랜트 왕복 부호를 검증하는 테스트가 없습니다(Issue 7). 규약을 코드로 강제하는 지점(단일 함수, assert, 왕복 테스트)이 한 곳도 없어서 각 파일이 각자 해석합니다.

2. **같은 코드가 여러 벌 존재하고 동기화가 사람 손에 달려 있다.** 루트/ros2_ws 두 트리 + 그 동기화를 스크립트 3개가 각자의 파일 목록으로 관리하고(sync_controllers.sh MODULES 7개 vs setup_ubuntu.sh 5개), 알고리즘 자체도 두 벌입니다(INDI가 controller.py와 hybrid_comparison.py에 별도 구현 — 실제로 '실기 루프율 가변 대응' 개선이 한쪽에만 들어감). 수정이 필요할 때마다 '어느 사본을 고쳐야 하나'를 매번 판단해야 하고, setup_ubuntu.sh는 그 갈라짐을 **매 실행마다 새로 만들어냅니다**.

3. **'물리 파라미터는 vehicle_params.py에만'이 선언만 되고 강제 수단이 없다.** n_max가 5곳, g가 3곳, tau_m이 2곳, hover_rpm이 n_max×0.32라는 매직넘버로 존재합니다. 지금은 값이 전부 우연히 일치해 오동작이 0이라 문제가 보이지 않는데, 위험 시점은 CLAUDE.md가 스스로 예고한 '형상팀 값 교체' 순간에 몰려 있습니다. 게다가 노드의 n_max는 Gazebo SDF와도 같아야 해서 단일 출처가 vehicle_params 하나로도 부족합니다.

4. **안전 계층이 [총추력, 3축 모멘트] 분해 없이 로터 명령 벡터에 직접 원소별 연산을 한다.** `np.full(4, hover_rpm)`, `np.maximum(u, hover_cmd)`, `np.sqrt(max(f_ind[i], 0) / k_T)` — 셋 다 같은 실수입니다. 로터 명령 공간에서 원소별로 자르면 총추력과 모멘트가 동시에, 예측 불가능하게 왜곡됩니다(Issue 2, 10). 자세 제어 권한이 사라지는 방향이라 실기에서 가장 위험한 패턴입니다.

5. **유일한 회귀 게이트(test_plant.py)가 assert 없는 print 위주라 커버리지 착시를 만든다.** 6개 테스트 중 감쇠는 assert가 0개, 할당은 균일 입력만, 전진비는 부등호만, 바람·tau_m·재정규화는 아예 미검증입니다. 뮤테이션을 3개 동시에 넣어도 `ALL TESTS PASSED`가 나옵니다. CLAUDE.md가 이 파일을 dynamics.py 변경의 필수 검증 수단으로 지정했기 때문에, '테스트 통과'가 안전 신호로 오독되는 구조입니다.

6. **지표가 물리적 정의 없이 임시로 정해지고, 그 값이 곧바로 MEMORY/HANDOFF/results로 승격되어 되돌리기 어려워진다.** 돌풍 회복시간이 0.00/6.00 두 값만 내는데 '최빠른 회복' 결론이 나가고, T_SIM=10 s가 정착시간의 1/6인데 '고정 LQR은 z 저하 없음'이 MEMORY에 박히고, SAT%가 로터 평균인데 문서는 '시간 비율'이라 적습니다. 지표를 고치면 전 실험 재집계 + 문서 갱신이 연쇄되므로 시간이 갈수록 고치기 어려워집니다.

7. **실패가 예외가 아니라 print/경고/플래그로 처리되고, 하류가 그 플래그를 읽지 않는다.** find_trim은 미수렴을 print만 하고 잘못된 해를 반환하며 ScheduledLQR은 residual을 안 봅니다. SafetyLevel은 로그 문자열 외에 어떤 분기도 타지 않고 EMERGENCY는 사용처 0곳, 워치독은 증가 코드 자체가 없습니다. '경고를 냈다'와 '실패를 막았다' 사이의 간격이 이 코드베이스 전반의 기본값입니다.

8. **파이썬 시뮬은 자기일관성으로 검증되지만 실기 경로에는 어떤 자동 검증도 없다.** 이번 리뷰의 critical 2건(쿼터니언 부호, 안전 폴백 제로 모멘트)이 **둘 다** ros2_ws 쪽에서 나왔습니다. 시뮬은 플랜트와 제어기가 같은 가정을 공유해 자기일관적이므로 결과가 깨끗한데, PX4 상태 조립·노드 생애주기·SafetyGuard는 그 자기일관성 밖에 있고 테스트가 없습니다.

9. **제어기 인터페이스가 duck-typing(hasattr)과 암묵 규약으로 연결돼 있다.** offboard_node의 heading 래치가 `hasattr(controller,'heading')` / `hasattr(controller,'x_trim')`로만 분기하고 else가 없어, 제어기를 추가하면 노드 쪽 처리가 조용히 누락됩니다(ScheduledLQR은 내부 이름이 `_x_trim_arr`라는 이유로 빠짐). 제어기 6종을 파라미터로 고를 수 있는 구조인데 각 제어기가 노드에 무엇을 요구하는지 명시된 계약이 없습니다.

## 8. 권장 수정 순서

1. **0단계 — 기준선 고정 (수정 전 반드시).** 지금 상태 그대로 test_plant.py, mission_sim.py, sweep_plots.py, gust_comparison.py, ekf_comparison.py를 한 번씩 돌려 results/에 타임스탬프 붙인 스냅샷을 남기십시오. 이후 모든 수정의 회귀 비교 기준이 됩니다. **중요**: results/MISSION_ANALYSIS.md의 Hybrid 1.752/1.670은 현행 코드로 재현되지 않고 2.774가 나온다는 검증 보고가 있으므로, 기존 문서 수치를 기준으로 삼지 말고 새로 뽑은 스냅샷을 쓰십시오.

2. **1단계 — Issue 1 (쿼터니언 최단경로). 단독, 2~3줄, 즉시.** controller.py:291과 476에 `if dq[3] < 0.0: dq = -dq` 추가 + ros2_ws 사본. 다른 어떤 수정과도 의존 관계가 없고, 파이썬 시뮬 수치는 **바뀌지 않아야 정상**이라 회귀 확인이 가장 쉽습니다(바뀌면 멈추고 확인). 실기 롤 발산 후보 3개 중 하나를 여기서 제거합니다.

3. **2단계 — Issue 3 (상태 유효성·신선도·PX4 인계).** 실기 안전 4종 중 이것부터인 이유: 입력 검증이 서면 아래 폴백들이 무의미하게 발동하는 경로가 먼저 줄고, ProperHybrid의 NaN 영구 고착이 막힙니다. 파이썬 시뮬 재실행 불필요(ros2_ws 안에서 완결).

4. **3단계 — Issue 2 (SafetyGuard 폴백 재설계).** 총추력/차동 분해 기반으로 `_check_state` 재작성 + hover_rpm을 vehicle_params에서 계산해 명시 전달. 2단계 후에 하는 이유는, 검증 없는 입력이 계속 들어오는 상태에서 폴백만 고치면 발동 빈도를 못 줄이기 때문입니다.

5. **4단계 — Issue 4 (노드 생애주기: t 단조성 + reset 호출 + arm ack).** 이걸 고쳐야 SITL 반복 시험(전복 → disarm → 재arm)이 신뢰 가능해집니다. 즉 5단계의 원인 판정을 위한 전제조건입니다.

6. **5단계 — Issue 5 (heading 래치 명시 분기 + v_ref 회전).** 마지막에 두는 이유: 1~4단계로 교란변수를 제거한 뒤에야 HANDOFF.md:154-157의 '롤 진동 발산'이 실제로 남는지 판정할 수 있습니다. 여기까지가 **실기 비행 전 반드시 끝내야 할 블록**이고, 검증은 SITL로 합니다(파이썬 재실행 불필요).

7. **6단계 — Issue 7 (test_plant.py 회귀 게이트 보강). dynamics.py를 건드리기 전 필수 선행.** 현재 코드는 정상이므로 보강 후에도 통과해야 하고, 추가로 4개 뮤테이션(C_mq 부호 반전, A[3,:] 반전, J_max=1e12, 바람 z성분 부호 반전)을 각각 적용해 **테스트가 실패하는지** 확인하십시오. 통과하면 assert가 아직 부족한 것입니다. 이 단계 없이 7단계로 가면 회귀 검증이 아무것도 보장하지 않습니다.

8. **7단계 — Issue 8 (로터 각운동량 2건).** dynamics.py:150 부호 + ḣ 항 추가. 회귀 범위가 넓습니다: controller.py:155의 B 행렬, hybrid_comparison.py:356의 G[3,i], vehicle_params.py 주석 라벨, ros2_ws 사본까지. 검증은 6단계에서 보강한 test_plant.py 전체 + 요 기동 + MEMORY.md 수치 재실행. 트림은 거의 불변이어야 정상이며(h≈0, ṅ=0), 크게 바뀌면 CLAUDE.md 규칙대로 버그를 먼저 의심하십시오.

9. **8단계 — Issue 9 (트림 수렴 검사) + Issue 12 (파라미터 단일 출처).** 두 개 모두 '형상팀 값 교체' 전에 반드시 끝나야 하는 준비 작업이고 서로 성격이 같아 한 세션에 묶는 게 좋습니다. 특히 offboard_node 경로의 트림 미수렴은 예외로 승격해 실기 arming을 막을 것. vehicle_params ↔ Gazebo SDF 일치 assert도 여기서.

10. **9단계 — Issue 13 (배포/중복).** setup_ubuntu.sh의 절대 import 덮어쓰기 제거 + sync_controllers.sh를 단일 출처로. 순서상 여기지만, **1~5단계 결과를 Ubuntu로 옮겨 SITL 검증할 때 이 경로를 쓰게 되므로 실제로는 2단계 착수 전에 sync 경로만 먼저 확정**해 두십시오(sync_controllers.sh는 이미 perl로 고쳐져 정상 동작 확인됨). INDI 이중 구현 통합은 나중에.

11. **10단계 — Issue 10 (할당 포화) + Issue 11 (트림 ff/기준값) + Issue 14 (지표 정의)를 한 배치로.** 여기부터는 파이썬 결과가 실제로 바뀌므로 재실행 비용이 듭니다. 세 개를 따로 하면 전 실험을 세 번 돌려야 하니, **수정을 모두 끝낸 뒤 한 번만 재실행**하고 results/*.txt + MEMORY.md + HANDOFF.md를 일괄 갱신하십시오. 배치 안에서의 순서는 11(기준값) → 10(할당) → 14(지표)가 자연스럽습니다(기준값을 고치면 포화 빈도가 줄고, 그 상태에서 지표를 재정의).

12. **11단계 — Issue 15 (공력 유효범위 문서화).** 코드 변경 없이 문구만입니다. **Gazebo C++ 공력 플러그인 착수 전에** 끝내야 같은 낙관이 실기 예측으로 전파되지 않습니다. dynamics.py 상단의 '이 모델이 담지 않는 것' 목록이 핵심 산출물입니다.

13. **12단계 — 정리 작업 (Issue 6, 16, 17, 18).** NMPC 상태 초기추측, 문서 드리프트, ESKF/센서, 미배선 진단 코드. 언제 해도 되지만 Issue 16(문서)의 (b)(d)는 재현성에 직접 걸리므로 10단계 재실행 직전에 함께 처리하면 효율적입니다. Issue 18(b)는 HybridWithFallback을 실제로 배선하는 시점에.

## 9. 구조적 리스크

- **리스크 1 — 실기 경로에 검증 계층이 0이다.** 이번 리뷰에서 나온 critical 2건(쿼터니언 부호 뒤집힘, 안전 폴백 제로 모멘트)이 **둘 다** ros2_ws 쪽에서 나왔다는 사실이 이 코드베이스 상태를 요약합니다. 파이썬 시뮬에는 test_plant.py(불완전하지만 존재)와 회귀 실행 비교 문화가 있는데, 실기로 나가는 코드(PX4 상태 조립, offboard_node 생애주기, SafetyGuard, frame_utils)에는 **어떤 자동 테스트도 없습니다**. 게다가 시뮬은 플랜트와 제어기가 같은 가정을 공유해 자기일관적이므로 결과가 깨끗하게 나오고, 그것이 '검증됐다'는 착각을 만듭니다. 트림 쿼터니언 w가 모든 속도에서 정확히 0이라는 사실 하나만 단위 테스트로 확인했어도 Issue 1은 진작 잡혔을 겁니다. **실기 첫 비행 전에 최소한 frame_utils 왕복 변환, SafetyGuard 각 레이어의 출력 모멘트, 트림→제어기 인터페이스 세 가지에 대한 테스트는 있어야 합니다.**

- **리스크 2 — 코드가 두 벌, 알고리즘도 두 벌이고 동기화가 사람 손에 달려 있다.** 루트와 ros2_ws/controllers/에 같은 모듈이 있고, 그 동기화를 스크립트 3개가 각자 다른 파일 목록으로 관리하며, setup_ubuntu.sh는 실행할 때마다 동기화를 **깨뜨립니다**. 여기에 INDI가 controller.py와 hybrid_comparison.py에 별도 구현으로 존재해 '실기 루프율 가변 대응' 같은 개선이 한쪽에만 들어갔습니다. 이 구조에서는 어떤 수정도 '몇 군데를 고쳐야 하나'라는 질문을 동반하고, 이번 리뷰의 수정 제안 상당수가 '루트/ros2_ws 사본 둘 다'라는 단서를 달고 있습니다. 실기 사고의 전형적 원인이 '시뮬에서 고친 걸 기체에 안 올렸다'인데, 지금 구조는 그 실수를 조직적으로 유도합니다.

- **리스크 3 — 안전 계층의 설계 방향이 '이상 감지 → 제어 권한 축소'다.** SafetyGuard의 4개 레이어 중 실제 동작하는 3개가 전부 자세 제어 권한을 깎는 방향입니다(틸트 초과 → 4로터 동일, 저고도 → 원소별 하한, NaN → 4로터 동일 호버). 워치독은 미구현이고, EMERGENCY 레벨은 사용처가 0곳이며, 우리 실패를 PX4 failsafe로 승격시키는 경로가 없어 오히려 PX4의 안전망을 억누릅니다. 즉 실기에서 이상이 나면 회복 확률이 구조적으로 낮고, 최후 수단이 존재하지 않습니다. 확정 설계인 HybridWithFallback(이상 감지 → LQR 전환)조차 offboard_node에 배선돼 있지 않아, MEMORY.md가 '확정'으로 적은 안전 아키텍처와 실제 실기 코드 사이에 간격이 있습니다.

- **[강점 — 아부가 아니라 사실] 이 프로젝트의 검증 문화는 실제로 작동하고 있고, 그게 이번 리뷰의 품질을 만들었다.** (1) 물리 코어가 견고합니다 — dynamics.py는 6-DOF + 로터 1차 지연 + 전진비 + 동체 공력을 CasADi 자동미분으로 담고 있고, 할당행렬·트림·플랜트가 서로 정합하며(요 부호 규약이 148행/221행에서 일치), 트림 잔차가 전 속도에서 1e-9 이하입니다. (2) CLAUDE.md의 '극단값은 거의 항상 버그' 규칙이 실효를 냈습니다 — 검증자들이 원 지적의 과장(전복·추락·무한부양 서술 다수)을 실측으로 걷어냈고 15건을 기각했습니다. (3) 리뷰 사이클이 실제로 돌고 있습니다 — VirtualNMPC w0 쿼터니언 시딩, sed→perl, heading 래치, att_gain_scale, 발산 판정 강화가 이미 HEAD에 들어 있습니다. (4) HANDOFF/MEMORY가 유지되어 이번 판정의 상당수가 그 문서 덕에 빠르게 끝났습니다(momentConstant 수정 이력, 롤 발산 미해결 기록 등). 구조적 리스크는 크지만 그걸 발견하고 고칠 프로세스는 이미 있습니다.

## 10. 정직한 총평

이 코드베이스는 **연구용 시뮬레이터로는 꽤 잘 만들어졌고, 실기 비행 소프트웨어로는 아직 검증되지 않았습니다.** 그 간극이 이번 리뷰의 핵심입니다.

**가장 정직한 요약 한 줄**: 살아남은 60여 건 중 critical 2건이 **둘 다** 파이썬 시뮬이 아니라 ros2_ws 쪽에서 나왔습니다. 시뮬 코드는 6종 제어기 비교, 몬테카를로, ESKF까지 돌리면서 반복 검증을 받아왔는데, 실기로 나가는 코드는 SITL에서 몇 번 띄워본 것이 검증의 전부입니다. dynamics.py에서 나온 지적들은 대부분 '모델 유효범위'나 '자기일관적이라 결과는 안 틀림' 수준인 반면, offboard_node.py/safety.py에서 나온 것들은 '10° 롤에 부호가 뒤집힌다', '안전장치가 발동하면 모멘트가 0이 된다' 같은 종류입니다.

**HANDOFF.md:154-157의 '롤 진동 발산 미해결'이 지금까지 안 풀린 것은 우연이 아닙니다.** 이번에 찾은 것만 독립 원인 후보가 셋입니다 — (1) 트림 쿼터니언 w=0 + `q_w>0` 정규화 + 최단경로 보정 부재의 3단 결합, (2) scheduled_lqr/indi가 heading 래치를 못 받고 조용히 통과, (3) 재arm 시 t 되감김으로 NMPC 외측 루프 25초 동결. 셋 다 로그에 아무것도 남기지 않습니다. 다만 정직하게 덧붙이면, HANDOFF는 Hybrid도 발산한다고 적고 있는데 Hybrid는 (1)에 면역이므로 이것들이 원인 전부는 아닙니다. **그래서 수정 순서를 '교란변수 제거 → 재판정'으로 짰습니다.**

**결과 수치는 방향은 대체로 맞고 숫자는 그대로 인용하면 안 됩니다.** 구체적으로: results/MISSION_ANALYSIS.md의 Hybrid 1.752/1.670은 현행 코드로 재현되지 않습니다(2.774). MEMORY.md의 '고정 LQR은 z 저하 없음'은 T_SIM=10 s의 함수이고 창을 40 s로 늘리면 3.53배가 됩니다. gust_comparison의 '회복 시간' 열은 0.00 아니면 6.00 두 값뿐이라 그 열로 내린 결론은 무효입니다. 반대로 **결론의 방향은 대부분 살아남습니다** — Hybrid 우위는 이륙 구간에서 오히려 보수적으로 측정됐고(NMPC 계열만 v_ref 불일치로 손해), RTK 우위는 낙관적 속도 잡음을 고쳐도 순위가 안 바뀌며, '저하는 vx 축이 지배적'은 창을 늘려도 유지됩니다. 즉 **논문/보고의 서사를 다시 쓸 필요는 없고, 표의 숫자와 이분법적 판정(무릎점 '없음', 최빠른 회복 'LQR')만 다시 뽑으면 됩니다.**

**규모 감각**: 실기 손실로 직결되는 것은 5개 그룹(Issue 1~5)이고, 이건 전부 ros2_ws 안에서 완결되며 파이썬 재실행이 필요 없습니다. 즉 며칠 단위 작업입니다. 나머지 13개는 시뮬 충실도·보고 정확도·미래 대비이고, 그중 재실행 비용이 큰 것들(Issue 10/11/14)은 한 배치로 묶으면 재실행 한 번으로 끝납니다. **절망할 상태가 전혀 아닙니다.** 다만 '실기 비행 전 필수'와 '보고용 정리'를 섞어서 진행하면 둘 다 늦어집니다.

**이 리뷰 자체에 대한 정직한 경고**: 저도 100% 신뢰하지 마십시오. 검증 단계에서 원 지적의 과장이 대량으로 걸러졌습니다 — '무한 부양', '즉시 전복', '초음속 팁', '무제한 개루프' 같은 서술이 실측에서 유계로 판명됐고 15건이 기각됐습니다. 저도 이 종합에서 두 건을 추가로 낮췄습니다: plant-physics#2의 model.sdf 논거는 `.gitignore:38`로 git 미추적 로컬 스냅샷이고 HANDOFF.md:118-119가 이미 Ubuntu 원본 수정을 기록하므로 **근거 약함**(같은 사실을 지적한 cross-cutting#11은 기각됐는데 이쪽만 살아남은 판정 불일치입니다), nmpc-indi#1의 VirtualNMPC w0 쿼터니언은 **이미 HEAD에 수정돼 있어 종결**입니다. Issue 8의 부호 방향 판정도 confidence를 medium으로 둔 이유가 있습니다 — Gazebo SDF 근거가 이 저장소만으로는 검증 불가라 할당행렬 정합성 하나에만 기대고 있습니다. **Issue 1~5 외에는 kj가 직접 파일을 열어 확인한 뒤 착수하십시오.** 특히 dynamics.py를 건드리는 Issue 8은 test_plant.py를 먼저 고치지 않으면 회귀 검증이 아무것도 보장하지 않습니다.

**마지막으로 잘한 것**: 이 프로젝트가 스스로 만든 규칙들이 실제로 값을 하고 있습니다. '극단값은 거의 항상 버그'는 이번 검증 라운드에서 열 건 넘게 걸러냈고, '원본 소스와 대조 전에 부호 결론 확정 금지'는 Issue 8의 판정 근거를 정확히 좁혀줬으며, HANDOFF/MEMORY 유지 덕에 momentConstant 수정 이력 같은 것이 즉시 확인됐습니다. 구조적 리스크는 크지만, 그걸 찾아내고 고칠 프로세스는 이미 이 프로젝트 안에 있습니다.

## 11. 각 리뷰어의 커버리지 자기보고

커버리지를 부풀리지 않게 하려고 스키마에서 강제로 받은 항목입니다.

<details>
<summary><b>plant-physics</b> — 플랜트 물리 (dynamics/params/trim)</summary>

끝까지 정독한 파일: dynamics.py(294줄 전체), vehicle_params.py(84줄 전체), trim.py(195줄 전체), test_plant.py(186줄 전체 — 담당 밖이지만 기존 부호 규약 검증용으로 통독).

부분 확인만 한 파일(grep + 특정 구간 Read): controller.py(ScheduledLQR 381-430, 클리핑 위치), nmpc.py/hybrid_comparison.py/mission_sim.py/gust_comparison.py(find_trim·n_max·wind 호출부만 grep), model.sdf(플러그인 블록 grep — 이 파일에는 링크/pose 정의가 없고 PX4 x500 SDF 를 참조하는 플러그인 스니펫뿐이라 로터 위치는 PX4 x500 표준 순서(0=FR,1=RL,2=FL,3=RR)를 가정해 판정했다), offboard_node.py(390-470 모터 매핑 구간만). 따라서 이들 파일 내부의 다른 결함은 이 리뷰 범위 밖이다.

코드로 직접 실행해 확인한 것:
- dynamics 의 R(q) 가 scipy Rotation 과 일치(호버 및 θ=-0.05 트림 자세 모두), 쿼터니언 미분은 scalar-last·body-rate·body→inertial 규약으로 손계산과 일치 → 부호 문제 없음.
- 정적 복원 모멘트 부호(x_cp=-0.10, α>0 → M_y<0)는 dynamics.py:85 의 실제 교차곱 전개로 검증했고 test_plant.py TEST3 과도 일치 → 문제 없음.
- 할당행렬(dynamics.py:203-223)은 M = (-r_y·T, r_x·T, 0) 로 dynamics.py:138-141 과 정확히 일치하고, 4행이 서로 직교해 cond(A) ≈ 12 로 정상 → 문제 없음.
- 알려진 함정 4건 회귀 여부: INDI G 의 전진비 반영은 T = k_T·n²·fac, fac = 1 - c/n 에서 dT/dn = k_T·n·(1+fac) 가 정확히 성립함을 대수적으로 확인(회귀 없음). 나머지 3건(reset, ESKF R 전달, 폴백 NaN 순서)은 담당 파일 밖이라 확인하지 않았다.
- 트림 스윕 실행(0~85, 85~195), 85 m/s 트림에서 h_net/자이로 항 크기, LQR 순항 시뮬에서 h_net·ḣ 실측, 프로펠러 FM/동력 계산, 팁 마하 계산 — 모두 본문 수치의 근거다.

확신이 낮거나 확인 못 한 부분:
- 1번(h_net 부호): 두 줄이 상호 모순이라는 점은 확정적이다. 다만 '어느 쪽을 고쳐야 하는가'는 model.sdf 의 turningDirection 과 Gazebo 의 cw/ccw 정의(조인트 축 +z, 모델 프레임 z-up)에 대한 내 해석에 의존한다. 실제 PX4 x500 SDF 의 rotor 링크 pose 를 Ubuntu 쪽에서 직접 확인해 FR/RL/FL/RR 배정을 대조한 뒤 확정할 것을 권한다.
- 6번(Jorgensen A_p/S_ref): C_dc 를 'S_ref 기준 유효계수'로 의도했다면 결함이 아니다. vehicle_params.py:51 의 '원통 표준' 주석을 근거로 raw 계수 의도라고 판단했으나 단정하지 않는다.
- 리뷰 중 부수적으로 관측한 것(추적하지 않음): INDIController 를 85 m/s 트림에서 단독으로 4초 돌리면 로터가 전 구간 포화하고 ω_x 가 93 rad/s 까지 발산한다. ProperHybrid 가 확정 설계이고 INDI 단독은 담당 밖이라 원인을 파고들지 않았다 — 별도 확인이 필요할 수 있다.

</details>

<details>
<summary><b>classical-ctrl</b> — 고전 제어기 (controller.py)</summary>

[끝까지 읽은 파일]\n- /Users/kj/Desktop/dynamic/fast_drone/controller.py (817줄 전체, 1회 통독 후 클래스별 재검토)\n- /Users/kj/Desktop/dynamic/fast_drone/dynamics.py (295줄 전체)\n- /Users/kj/Desktop/dynamic/fast_drone/trim.py (196줄 전체)\n- /Users/kj/Desktop/dynamic/fast_drone/vehicle_params.py (84줄 전체)\n- /Users/kj/Desktop/dynamic/fast_drone/fallback_controller.py (190줄 전체)\n\n[부분만 읽은 파일 — 호출 경로/사용법 확인 목적]\n- mission_sim.py (MissionProfile.get_ref, MissionController, run_mission 계열), ekf_sim.py (simulate_with_ekf 앞부분, 비교 main), ekf_comparison.py (make_controllers, _reset_controller), gust_comparison.py (_FixedLQR, main), sweep_plots.py (run_sweep), nmpc.py (LQRController.__new__ 사용부), ros2_ws/.../offboard_node.py (grep 결과만)\n\n[코드로 실제 검증한 것 — 스크래치패드 스크립트 실행]\n1. 트림 테이블 V=0~85: 전 구간 residual 8e-12~2e-9, ω_trim=0, 전체 xdot(17) 최대 1.1e-9 → 트림이 진짜 평형점임을 확인(LQR 선형화 전제 성립).\n2. 할당행렬 부호: +Mx/+My/+Mz 명령 → 플랜트 실제 모멘트 [2,0,0]/[0,2,0]/[0,0,0.2] 로 정확히 일치. 부호 오류 없음.\n3. INDI _compute_G: 플랜트 유한차분과 대조해 최대 상대오차 5.7e-10. 전진비 dT/dn=k_T·n·(1+fac) 도 T=k_T(n²-cn) 미분과 해석적으로 일치. 알려진 함정 2번(INDI G 전진비) 회귀 없음.\n4. 쿼터니언 오차상태: 10/90/170/179/181/190/270/350° 롤에 대해 dphi 계산 → 180° 초과에서 부호 반전 확인, 폐루프 시뮬로 발산 방향 확인(발견 1).\n5. int_z_max: V=30/70/80 에서 25초 시뮬, 5.0 vs 50.0 양방향 대조(발견 2).\n6. 할당 음수 추력: sweep_plots 초기조건으로 V=0~80 계측(발견 3).\n7. 정상상태 vx: CascadedPID/ScheduledPID/INDI 각각 계측(발견 4).\n8. 돌풍 시 요구 틸트각 계측(발견 6의 영향 없음 확인).\n9. LQR 노드별 폐루프 최대 실수부: -0.49(호버)~-0.057(80 m/s), 전 노드 안정. Q_L 행렬은 Hamilton 곱 전개와 손으로 대조해 일치 확인(알려진 함정 Q_L 부호 회귀 없음).\n10. 알려진 함정 재확인: CascadedPID.reset()/INDIController.reset() 은 각각 _int_ez / (_omega_prev, _omega_dot_filt, _int_ez, _initialized) 를 전부 초기화하여 누락 없음. LQRController/ScheduledLQR 은 내부 상태가 없어 reset() 이 없으며, 호출부(mission_sim, ekf_comparison, gust_comparison, fallback_controller)는 모두 hasattr 가드를 쓰므로 AttributeError 크래시는 발생하지 않음을 grep 으로 확인.\n\n[확신이 없거나 확인하지 못한 부분]\n- ScheduledLQR 의 트림 선형보간이 만드는 정상상태 오차는 계측 결과 V=75 에서 +0.146 m/s (0.2%) 로 작았다. MEMORY.md 의 '트림 ff 불일치로 vx에 나타남' 기록과 방향은 일치하지만 크기가 작아 버그가 아니라 게인 스케줄링 고유의 근사 오차로 판단해 발견 항목에서 제외했다.\n- 발견 1의 '자세 오차 > 180° 상황이 현재 시뮬에서 실제로 발생하는지'는 확인하지 못했다. 코드 경로(매 제어 스텝)는 확실히 도달하지만, 기존 mission_sim/gust_comparison 로그에서 180° 초과 자세 오차가 실제로 찍혔는지는 검사하지 않았다. severity high 는 '실기 비행 + 안전 폴백 제어기'라는 맥락을 근거로 매긴 것이다.\n- ekf_sim/estimator 가 제어기에 넘기는 추정 쿼터니언의 전역 부호가 플랜트 쪽과 항상 같은 부호로 유지되는지는 estimator.py 를 열지 않아 확인하지 못했다. 만약 어긋나면 발견 1은 180° 조건 없이도 즉시 터진다 — 별도 확인 권장.\n- INDIController 가 EKF 경로에서 받는 x[13:17](로터 속도)이 실제값인지 명령값인지는 ekf_sim.py 를 끝까지 읽지 않아 확인하지 못했다. 명령값이면 INDI 증분 계산의 기준점이 tau_m 만큼 앞서게 되어 별도 문제가 될 수 있다.\n- ros2_ws/.../controllers/controller.py 는 루트 controller.py 와 import 문 6줄 외에 완전 동일함을 diff 로 확인했으므로, 위 발견은 모두 실기(ROS2/PX4) 경로에도 그대로 적용된다.

</details>

<details>
<summary><b>nmpc-indi</b> — NMPC + INDI 하이브리드</summary>

[끝까지 읽은 파일] 담당 파일 2개는 전부 정독했습니다: nmpc.py(1~343줄 전체), hybrid_comparison.py(1~579줄 전체). 교차 확인용으로 dynamics.py(295줄 전체), controller.py(817줄 전체), vehicle_params.py(84줄 전체), fallback_controller.py(190줄 전체)도 완독했습니다. 부분만 읽은 파일: ros2_ws/.../offboard_node.py(280~400, 580~700줄 — 실행 경로 확인용), ekf_comparison.py(40~110줄), mission_sim.py(120~160줄), bench_accel_experiments.py(85~150줄). trim.py, sensors.py, estimator.py는 열지 않았습니다.

[동기화 확인] ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers/ 의 nmpc.py·hybrid_comparison.py·controller.py는 import 문(`from x` → `from .x`)만 다르고 본문은 루트 사본과 완전히 동일합니다(diff로 확인). 따라서 위 findings의 줄 번호가 ROS2 사본에도 그대로 적용되며, 수정 시 두 곳 모두 고쳐야 합니다.

[알려진 함정 회귀 여부 — 모두 정상] (1) reset(): nmpc.py:71-76과 hybrid_comparison.py:109-114 모두 _last_t=-inf 와 w0=_w0_init을 함께 복원 → 회귀 없음. (2) INDI G 전진비: compute_control_effectiveness(hybrid_comparison.py:348-351)의 dT/dn=k_T*n*(1+fac)를 dynamics.py:125-130의 T=k_T*n²*fac(fac=1-2πV/(n·D·J_max))로부터 직접 해석 미분해 대조 → 정확히 일치, 회귀 없음. (4) fallback_controller.py:123의 NaN 검사가 130줄 채터링 가드보다 먼저 옴 → 회귀 없음.

[부호 검증 — 오류 없음] compute_control_effectiveness의 G 각 행을 dynamics._rotor_forces_moments(133-148줄)의 실제 수식과 1:1 대조했습니다. M_x=r_y·(-T)→G[1,i]=-pos[i,1]·dT/Jx ✓, M_y=+r_x·T→G[2,i]=+pos[i,0]·dT/Jy ✓, M_z=dir·Q→G[3,i]=dirs[i]·dQ/Jz ✓, T_total→G[0,i]=+dT ✓. compute_allocation_matrix(dynamics.py:209-223)의 cross(pos,[0,0,-1])=(-r_y, r_x, 0)도 위와 부호 일치 ✓. build_virtual_dynamics의 추력 방향 vertcat(0,0,-T_cmd)·중력 [0,0,-g]·R@F_body/m·_quat_derivative 모두 dynamics._compute_xdot(179-180줄)과 동일 ✓. ProperHybrid의 Rotation.from_quat(q)는 scalar-last이고 dynamics._quat_to_rotmat도 q[0..3]=[qx,qy,qz,qw]로 scalar-last ✓. 상태 슬라이싱(13D/17D 모두 p=0:3, v=3:6, q=6:10, ω=10:13)도 순서 뒤바뀜 없음 ✓. 부호/축 관련 결함은 발견하지 못했습니다.

[검증했으나 근거 부족으로 보고하지 않은 항목 — 반대 증거 포함] 아래 4건은 처음에 의심했지만 실제로 측정해 보니 영향이 없거나 오히려 반대여서 findings에서 제외했습니다.
 (a) dt_ctrl(0.10) > dt_nmpc(0.05) 불일치(ROS2 설정): 돌풍 시나리오로 dt_ctrl=0.10/dt_nmpc=0.05 vs 0.05/0.05 vs 0.10/0.10을 비교했으나 RMSE z가 0.168 / 0.171 / 0.198로 현 설정이 오히려 근소하게 좋았음 → 결함 근거 없음.
 (b) INDI 증분의 u0(=n_actual)가 ω̇처럼 LPF를 거치지 않는 비동기 문제: 표준 INDI 이론상 결함이지만, n_actual을 동일 LPF에 통과시킨 변형과 비교해도 RMSE z 0.0786 vs 0.0790으로 차이가 없었음(dt=0.001, f_cut=50 Hz에서 필터 지연 3.2 ms ≪ tau_m=20 ms) → 보고 안 함.
 (c) compute_control_effectiveness의 `ni = max(n_actual[i], 1.0)` 정규화: 로터 1개가 n=0이 되면 cond(G)가 75 → 4.9e4로 악화되고 해당 dn이 3.2e4까지 튀지만, ×배치 대칭성 덕분에 나머지 3개 dn은 값이 전혀 변하지 않고(직접 확인) 튄 값도 np.clip으로 n_max/n_min에 잡히므로 실질 피해가 없음 → 보고 안 함. (W=25 m/s 돌풍에서 n=0 도달 자체는 실제로 재현됨)
 (d) nu_max=100 rad/s²를 세 축에 동일 적용: 호버 추력 제약 하 실제 각가속도 한계는 롤≈694, 피치≈19.8, 요≈9.3 rad/s²로 축마다 크게 다르지만, 현실적인 0→70 m/s 램프(10 s/20 s) 시뮬에서 nu가 한 번도 포화하지 않았고(0/2100 샘플) 요 비용항이 없어 nu_z≈0으로 유지됨 → 잠복 이슈로 판단해 보고 안 함.

[확신이 낮은 부분] findings#2의 NaN 입력 경로는 PX4 VehicleOdometry가 무효 시 NaN을 싣는다는 일반 지식과 offboard_node.py:182-184에 유효성 검사가 없다는 코드 사실에 근거했습니다. 실제 SITL에서 그 NaN이 관측되는지는 macOS에서 확인할 수 없었으므로 "재현 확인 필요"로 표시합니다. 다만 NaN이 들어왔을 때 w0가 영구 오염되어 스스로 복구하지 못한다는 부분은 파이썬에서 직접 재현했습니다. 또한 findings#1의 폐루프 수치는 모두 파이썬 6-DOF 플랜트(dt=0.001 또는 0.01)에서 제가 직접 실행한 결과이며, Gazebo/PX4에서의 실제 크기는 검증하지 못했습니다.

</details>

<details>
<summary><b>safety-fallback</b> — 폴백 / 안전 로직</summary>

담당 파일 2개를 처음부터 끝까지 전부 읽었습니다: /Users/kj/Desktop/dynamic/fast_drone/fallback_controller.py (1-189, 전체), /Users/kj/Desktop/dynamic/fast_drone/ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/safety.py (1-228, 전체).\n\n교차 확인을 위해 다음을 부분/전체 열람했습니다: dynamics.py(_quat_to_rotmat 30-46, _compute_xdot 164-187, compute_allocation_matrix 202-223), vehicle_params.py(1-90), controller.py(구조 grep + ScheduledLQR.__call__ 484-496), hybrid_comparison.py(ProperHybrid 202-292), nmpc.py(_last_t/reset grep), offboard_node.py(80-520 거의 전체), frame_utils.py(pos_ned_to_nwu, motor_speed_to_normalized), model.sdf(모터 상수).\n\n[부호 규약 대조 결과 — 오탐 아님을 확인한 항목]\nsafety.py:184 `qx, qy, qz, qw = q`는 dynamics.py:32 `qx, qy, qz, qw = q[0], q[1], q[2], q[3]`과 동일한 scalar-last 규약이고, bz_inertial 3개 성분도 dynamics.py:34-36 회전행렬의 3번째 열과 정확히 일치합니다. 호버 q=[1,0,0,0](qx=1)에서 R=diag(1,-1,-1) → bz_inertial=[0,0,-1] → cos_tilt=1 → tilt=0. 즉 틸트 계산 수식 자체는 올바르며, 부호 버그로 보고하지 않았습니다. offboard_node.py의 x[6:10]도 scipy from_quat(scalar-last)로 쓰이므로 정합합니다.\n\n[알려진 함정 회귀 확인 — 모두 회귀 없음]\n1. NMPC reset/_last_t: nmpc.py:67,73에서 -np.inf로 초기화, ProperHybrid.reset()이 nmpc.reset() 호출 → fallback_controller.py:173-179의 리셋 체인 정상 동작(178-179행은 중복이지만 무해).\n2. INDI G 전진비: hybrid_comparison.py:264-278에서 J/fac 반영 확인(담당 밖).\n3. ESKF R_pos/R_vel: 담당 밖으로 미검증.\n4. NaN 검사가 채터링 가드보다 먼저: fallback_controller.py:121-132에서 순서 정상 — 회귀 없음.\n\n[확신이 낮거나 미검증인 부분]\n- HybridWithFallback의 유일한 호출처는 legacy/monte_carlo.py:92뿐입니다(리뷰 제외 폴더). ros2_ws/.../controllers/fallback_controller.py는 루트 파일과 byte-identical 복사본이지만 offboard_node._create_controller에서 선택지로 배선돼 있는지는 563행 이후를 끝까지 보지 못해 확정하지 못했습니다. 따라서 fallback_controller.py 관련 5건은 '현재 실기 경로에 배선됨'이 아니라 '이 클래스를 쓰는 순간 발현됨'으로 읽어야 합니다. severity는 그 점을 감안해 매겼습니다(가장 높은 것이 high).\n- SafetyGuard는 offboard_node.py:120에서 생성되고 336행에서 매 제어 스텝 호출되므로 실기/SITL 실행 경로에 확실히 있습니다. safety.py 관련 5건은 impact_verified=true입니다.\n- Gazebo 측 actuator 정규화(control[i]가 rotor velocity에 선형인지 sqrt인지)는 PX4/gz 플러그인 쪽이라 확인하지 못했습니다. hover_cmd=576이 SITL에서 실제 호버가 되는지는 실행 검증이 필요합니다(model.sdf의 motorConstant=6.0e-05, maxRotVelocity=1800은 vehicle_params와 일치함을 확인).\n- 실행 검증(시뮬 재현)은 하지 않았고 전부 정적 코드 대조입니다.

</details>

<details>
<summary><b>estimator</b> — 추정기 / 센서</summary>

## 끝까지 읽은 파일
담당 4개 파일 전부 1행부터 끝까지 읽었다: estimator.py(349줄 전체), sensors.py(288줄 전체), ekf_sim.py(376줄 전체), ekf_comparison.py(276줄 전체).

교차 확인을 위해 추가로 읽은 파일: dynamics.py(294줄 전체), vehicle_params.py(84줄 전체), trim.py(195줄 전체), hybrid_comparison.py의 ProperHybrid 부분(200-310행), mission_sim.py의 관련 구간(130-250행, 300-360행, 786-846행). legacy/는 지시대로 완전히 제외했다.

## 부호/좌표계 규약 — dynamics.py 원본과 대조 완료 (결론: 문제 없음)
프로젝트 최대 함정 규칙에 따라 독스트링이 아니라 실제 수식 라인으로만 판정했다.
- 쿼터니언 순서: dynamics.py:32 `qx, qy, qz, qw = q[0], q[1], q[2], q[3]` → scalar-last. estimator.py가 쓰는 scipy `Rotation.from_quat`도 scalar-last이고, dynamics.py:33-36의 회전행렬 성분이 scipy의 as_matrix()와 원소별로 일치함을 손으로 전개해 확인했다. 호버 q=[1,0,0,0]은 x축 180°(body z-down)라는 규약과도 맞다.
- `ESKF._quat_mult`(estimator.py:339-349)를 dynamics.py:39-48 `_quat_derivative`의 0.5*G^T@omega와 4성분 모두 전개 비교했고 완전히 동일했다. 즉 q ⊗ exp(w·dt/2) 적분은 플랜트와 같은 규약이다.
- IMU 비력: sensors.py:139 `specific_force = acc_inertial + [0,0,g]`는 dynamics.py:179 `v_dot = [0,0,-g] + R@F/m`와 일관(호버 시 body 측정 = [0,0,-g], z-down에서 위쪽). estimator.py:215 `g_body_expected = R.T @ [0,0,self.g]`도 같은 값을 내므로 부호 뒤집힘 없음.
- ESKF 오차상태 야코비안: `_inject`가 q ← q ⊗ dq를 쓰므로 국소(body) 자세오차 파라미터화다. 이 정의로 직접 유도해보면 F[3:6,6:9] = -R[a_c]x·dt, F[3:6,9:12] = -R·dt, F[6:9,6:9] = I-[w_c]x·dt, F[6:9,12:15] = -I·dt 가 맞고, update_accel의 H[:,6:9] = +[g_body]x도 국소 정의와 부합한다(-[dθ]x·g = +[g]x·dθ). estimator.py:327-332, 237과 전부 일치 — 오류 없음.

## 명시적으로 확인했고 문제가 없던 항목
- Joseph form: update_gps(157-158), update_accel(247-248) 둘 다 IKH@P@IKH.T + K@R@K.T로 정확히 적용됨. 대칭화 P=(P+P.T)/2도 predict/update 전부에 있음(160, 249, 337).
- P 양정부호성: 70 m/s 20초 실행에서 갱신 시점 최소 고유값 5.661e-08 > 0. 붕괴 없음.
- 알려진 함정 회귀 검사 4건 전부 통과: (1) nmpc.py:71-76, hybrid_comparison.py:109-114의 reset()이 _last_t·_u_current·w0 모두 초기화하고 ProperHybrid.reset()이 내부 nmpc.reset()을 호출함. ekf_comparison.py:80-102 `_reset_controller`는 그 위에 덧씌우는 중복 안전장치라 무해. (2) INDI G행렬 전진비는 hybrid_comparison.py:303-310에 있고 담당 밖이라 상세 검토는 생략. (3) update_gps에 R_pos/R_vel 전달은 ekf_sim.py:104-105에서 실제로 이뤄짐(회귀 없음, 단 기본값 문제는 finding 6). (4) 폴백 NaN 순서는 fallback_controller.py 소관이라 담당 밖.
- 순환논리 검사: 제어기는 ekf_sim.py:116 `controller(t, x_hat)`로 추정값만 받는다. 참값 x_true는 센서 모델 입력(89, 93행)에만 쓰이고 추정기·제어기로 새지 않는다. compute_metrics/compute_estimation_metrics는 양쪽 다 xs_true를 기준으로 쓴다. 잡음 실현 재사용도 확인: IMU seed=42 / GPS seed=43으로 스트림이 분리되고, ESKF 초기 섭동은 seed+100=142로 또 분리되며, SensorSuite.reset()이 IMU·GPS 둘 다 재시드하므로 제어기 간 비교는 공정하다. 순환논리는 발견하지 못했다.
- 쿼터니언 부호 모호성(q vs -q): compute_estimation_metrics(ekf_sim.py:189)이 `np.abs(np.dot(...))`로 처리하고 있어 정상. 다만 반환 dict의 est_errors[:, 6:10]은 성분별 뺄셈이라 그대로 쓰면 의미가 없는데, 살아있는 호출부에서 그 슬라이스를 쓰는 곳은 없었다.
- xs_est 로깅 시각 정렬: 무잡음 dead-reckoning 실험으로 같은 인덱스 정렬이 정확함(오차 -0.000000)을 확인했다. finding 1은 로깅 오프셋이 아니라 갱신 순서 문제다.

## 확인하지 못했거나 확신이 없는 부분
- 오차상태 리셋 행렬 G(= I - 0.5[dθ]x)가 _inject 후 P에 적용되지 않는다(estimator.py:162-188). 교과서상 누락은 맞지만 dθ가 밀리라디안 수준이라 2차 효과이고, 20초 실행에서 P가 양정부호를 유지하는 것도 확인해서 finding으로 올리지 않았다. 지적 자체가 필요하면 별도로 알려달라.
- IMUSensor에는 dt 인자가 없어서 잡음이 '플랜트 스텝당 sigma'로 정의된다. dt=0.001이 아닌 값으로 시뮬하면 IMU의 물리적 잡음밀도가 조용히 바뀐다. 다만 살아있는 호출부는 전부 dt=0.001이라 잠재 위험으로만 남겨두고 finding에서 제외했다.
- 자이로 바이어스 추정이 20초 동안 수렴하지 않고 배회한다(오차 0.0007 -> 0.0365 -> 0.0116 -> 0.0024 rad/s). 원인은 finding 2의 요축 관측 불가로 보이지만, 나머지 두 축이 정상 수렴하는지 장시간(수 분) 실행으로는 확인하지 못했다.
- ekf_comparison.main()과 mission_sim의 Hybrid(NMPC 포함) 전체 실행은 시간이 오래 걸려 돌리지 않았다. 검증에 쓴 폐루프 실험은 전부 ScheduledLQR 기준이므로, NMPC/Hybrid에서 finding 1의 v·dt 편향이 제어 지표에 영향을 주는지는 확인하지 못했다. ProperHybrid는 x[6:10]과 x[3:6]으로 v_body를 계산하므로(hybrid_comparison.py:259-262) LQR보다 자세 추정오차에 민감할 가능성이 있다 — 영향 확인 필요.
- finding 3의 GPS 지연 크기(50~200 ms)는 일반적인 수신기 특성에 근거한 값이고 이 저장소 코드로 검증한 값이 아니다. 지연이 v·(지연시간)만큼의 계통 오차를 만든다는 메커니즘 자체는 finding 1에서 1 ms 스케일로 실측 확인했다.
- 검증에 쓴 프로브 스크립트 4개는 스크래치패드(/private/tmp/claude-501/-Users-kj-Desktop-dynamic-fast-drone/57a28c37-6ba7-4bd0-816c-fe0fffb5fd36/scratchpad/probe*.py)에만 두었고 저장소 파일은 하나도 수정하지 않았다.

</details>

<details>
<summary><b>mission-sim</b> — 미션 시뮬레이션 / 외란 실험</summary>

[끝까지 읽은 파일]\n- 담당: /Users/kj/Desktop/dynamic/fast_drone/mission_sim.py (857줄 전체), /Users/kj/Desktop/dynamic/fast_drone/gust_comparison.py (321줄 전체)\n- 교차확인용 전체 정독: dynamics.py(295줄), controller.py(817줄), hybrid_comparison.py(580줄), nmpc.py(343줄), ekf_sim.py(377줄), ekf_comparison.py(277줄), sensors.py(289줄), trim.py(196줄), vehicle_params.py(84줄), HANDOFF.md(앞 120줄)\n\n[실제 실행으로 검증한 것] — 모두 scratchpad에서만 실행, 저장소 파일은 일절 수정하지 않음\n1. gust_comparison settle 지표: CascadedPID / INDIController / _FixedLQR / ScheduledLQR 4종을 V=70, 수직돌풍 10m/s@t=2s, T_sim=8s로 실제 시뮬 → settle이 각각 6.00/6.00/0.00/0.00으로 두 값에만 고착됨을 확인. LQR 계열의 실제 최대편차 시각(t=2.70~2.73s)도 함께 측정.\n2. max_dz 기준선 문제: PID의 max_dz=1.1637이 t=5.567s(돌풍 종료 2.57초 뒤)에 발생, 돌풍 직전 이미 dz=-0.7994였음을 수치로 확인.\n3. mission_sim 이륙 지연: ScheduledLQR로 65초 미션 전체를 실행해 HANDOFF 기록값(RMSE vx=16.78, z=4.44)을 그대로 재현 → 내 실행 환경이 기준 결과와 일치함을 먼저 확인한 뒤 구간별 수치(이륙 rmse_z=9.889)를 얻음.\n4. v_z 피드포워드 반대방향 검증: get_ref만 오버라이드한 서브클래스로 (a) ScheduledLQR은 결과가 완전히 동일(= v_ref[1:3]을 아예 안 씀, controller.py:478 확인), (b) ProperHybrid는 이륙 13초 rmse_z 3.580→2.610으로 개선됨을 확인. 처음 LQR만 보고 '변화 없음'이 나왔을 때 원인을 코드에서 찾아 결론을 뒤집었음(v_ref[2]=0이 모든 제어기에 똑같이 작용한다는 초기 가정은 틀렸음).\n\n[확인했으나 문제 없다고 판단한 것 — 재지적 방지용 기록]\n- 적분/ZOH: dynamics.simulate가 dt=0.001로 매 스텝 controller(t,x) 호출, CasADi rk(number_of_finite_elements=4), step()에서 매 스텝 쿼터니언 정규화 + 로터 클립, wind_fn도 스텝 단위 ZOH — 정상.\n- reset 누락 없음: 단일 시뮬(642줄), MC(267줄), EKF(174줄) 모두 실행 직전 reset 호출. NMPC/VirtualNMPC의 _last_t·w0·_u_current 리셋 및 ProperHybrid→nmpc 전파 모두 살아 있음(알려진 함정 1, 2 회귀 없음).\n- 공정성: 3개 제어기 동일 x0.copy()/동일 gust_fn/동일 plant. MC는 제어기마다 seed=42로 rng를 새로 만들어 초기조건·돌풍 실현이 동일. EKF는 sensors.reset()이 IMU·GPS 양쪽 rng를 재시드(sensors.py:100,208,259)하고 ESKF 초기오차도 seed+100 고정 → 동일 노이즈 실현 확인.\n- 부호 규약: make_gust_fn의 w[2]=pulse(z-up 상승돌풍)가 dynamics.py:171 `v_body = R.T @ (vel - w)`와 부호 일관. 상태 인덱스(xs[:,1]=py, [:,2]=pz, [:,3]=vx, [10:13]=omega)도 dynamics.py 레이아웃과 일치. 회전방향/CW·CCW 관련해서는 이번 두 파일에 판단을 요하는 코드가 없었음.\n- 미션 구간 전환: 1-cos이라 경계에서 위치·속도 모두 연속(경계 기울기 0), get_ref와 compute_refs가 동일한 값을 산출함을 코드 대조로 확인.\n- 알려진 함정 3(ESKF R_pos/R_vel 전달), 4(폴백 NaN 검사 순서)는 담당 파일 밖이지만 ekf_sim.py:103-105에서 전달 유지 확인, fallback_controller.py는 이번 범위 밖이라 미확인.\n\n[확신이 낮거나 확인 못 한 부분]\n- NMPC(NMPCController)와 ScheduledPID는 비용 때문에 gust_comparison 전체를 돌려보지 못했다. settle 지표 결함은 PID/INDI/LQR고정/LQR스케줄 4종에서만 실측했고, NMPC의 settle 값이 0인지 6인지는 미확인이다(둘 중 어느 쪽이어도 '최빠른 회복' 결론이 틀리다는 판단은 바뀌지 않는다).\n- 발견 2(발산 판정 불일치)에서 '|Δz|>50m인 유한 궤적이 실제 MC 10회 안에서 발생하는가'는 확인하지 못했다. 두 함수가 서로 다른 기준을 쓴다는 사실과 '전체' 행이 구간 행과 모순될 수 있다는 점은 코드로 확정했지만, 현 파라미터에서 그 케이스가 실제로 나오는 빈도는 몬테카를로를 돌려봐야 안다(Hybrid 10회 ≈ 10분 소요라 실행하지 않았다).\n- 발견 6(RTK 속도 노이즈)의 '실기 영향'은 실제 RTK 수신기 데이터시트가 없어 일반적인 스펙 범위(0.03~0.05 m/s)를 근거로 했다. 계산식 자체는 담당 밖 파일(ekf_comparison.py:49)에 있으므로 수정 전 확인이 필요하다.\n- estimator.py(ESKF 내부)와 fallback_controller.py는 열지 않았다.\n- 저장소 파일은 하나도 수정하지 않았다. 다만 실행 중 __pycache__가 갱신되었을 수 있고, 세션 시작 시점과 달리 지금 git status에 HANDOFF.md 수정과 bench_*.py 미추적 파일이 보이는데 이는 내가 만든 것이 아니다.

</details>

<details>
<summary><b>sweep-test</b> — 스윕 플롯 / 회귀 테스트 (최신 커밋)</summary>

끝까지 읽은 파일: sweep_plots.py(537줄 전체), test_plant.py(185줄 전체), dynamics.py(295줄 전체), controller.py(817줄 전체), trim.py(195줄 전체), vehicle_params.py(84줄 전체), results/sweep_knee.txt, results/sweep_fixed_vs_scheduled.txt.

부호 관련 결론은 전부 dynamics.py의 실제 수식 라인과 대조했다. 호버 쿼터니언 [1,0,0,0]을 _quat_to_rotmat(30-36행)에 직접 대입해 R=diag(1,-1,-1)임을 확인했고, test_restoring_moment의 w_b>0 유도와 M_static=[0,-xcp*Fz,xcp*Fy](85행) + x_cp=-0.10 조합이 M_y<0을 주는 것을 확인했다. TEST 3은 양/음 α 양방향을 모두 assert하는 제대로 된 부호 테스트라 결함으로 보고하지 않았다. 요 반토크 부호는 dynamics.py:148(M_tot += di*Qi)과 dynamics.py:221(A[3,i]=dirs[i]*k)이 서로 일관되므로 시뮬 내부 결함은 아니며, "테스트가 부호 반전을 못 잡는다"는 커버리지 문제로만 보고했다.

또 확인 후 보고하지 않기로 한 것들: (1) 스윕의 제어기 오염 — CascadedPID/ScheduledPID는 각 속도마다 새로 생성 후 reset()되고, StrictFixedLQR/GainsOnlyFixedLQR은 무상태이며, 루프 밖에서 1회 생성되는 ScheduledLQR도 적분기·_last_t 같은 내부 상태가 전혀 없이 매 호출 _interpolate로 재계산하므로 조건 간 오염이 없다. (2) 조건별 재튜닝 — LQR Q/R과 PID 게인은 전 조건 고정이다. (3) 플롯 축/라벨 — METRIC_LABEL과 _plot_series의 metric 인자가 일관되어 "라벨은 z인데 데이터는 vx" 류 불일치는 찾지 못했다.

스크래치패드에서 실행해 검증한 것(모두 읽기 전용, 저장소 파일 미수정): rotor_directions 전역 반전 후 6개 테스트 전부 PASS / J_max=1e12로 전진비 무력화 후에도 TEST 5 통과하며 그 델타가 100% 교차류 항력임을 _body_aerodynamics 직접 호출로 확인 / simulate_and_measure에 합성 궤적을 넣어 |vz|>100·NaN·조기발산 3케이스 집계 결과 확인 / StrictFixedLQR V=85 120초 적분으로 정착시간 확인 및 T=10 vs T=40에서 무릎 판정 역전 확인 / V_table 10개 속도점의 트림 잔차와 LQRController.valid·max_real 확인(10/10 유효, 잔차 ≤1e-9) / ScheduledLQR·CascadedPID의 T=40 거동으로 1번 지적의 반대 방향 재확인.

하지 못한 것과 확신 없는 부분: (a) sweep_plots.py 자체는 실행하지 않았다 — 실행하면 git에 커밋된 results/*.png, *.txt를 덮어쓰기 때문이다. 대신 필요한 경로만 스크래치패드에서 재현했고, StrictFixedLQR V=30/85의 T=10 수치(0.864/0.298, 4.928/0.250)가 커밋된 txt와 소수점까지 일치해 재현성은 확인했다. (b) results/*.png 4장을 이미지로 열어보지 않아 축 라벨·범례의 실제 렌더링은 코드 독해로만 판단했다. (c) legacy/sweep.py, legacy/sweep_scheduled.py는 리뷰 대상 제외라 열지 않았다 — 따라서 "legacy와 동일 로직"이라는 docstring 주장(133행, 111-115행)과 T_SIM=10s의 출처는 검증하지 못했다. 1번 지적은 legacy가 어떠했든 무관하게 "정착시간 대비 창이 짧다"는 사실만으로 성립한다. (d) 6번(SAT% 정의)은 현재 파라미터에서 전 조건 SAT%=0.0이라 실행 경로상 영향을 실증하지 못해 impact_verified=false로 표시했다.

</details>

<details>
<summary><b>ros2-px4</b> — ROS2 / PX4 실기 경로</summary>

## 끝까지 읽은 파일 (담당)
- ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/offboard_node.py (651줄 전체)
- ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/frame_utils.py (243줄 전체)
- ros2_ws/src/fast_drone_ctrl/launch/sitl_offboard.launch.py (78줄 전체)
- ros2_ws/src/fast_drone_ctrl/setup.py, setup.cfg, package.xml (전체)
- scripts/run_sitl.sh, sitl_run.sh, setup_ubuntu.sh, sync_controllers.sh (전체)

## 교차 확인으로 읽은 파일 (담당 외)
- dynamics.py (부호 규약 원본): 25~200행 직접 확인. `_quat_to_rotmat`(scalar-last), `_rotor_forces_moments`(추력 body -z, `M_x = r_y*(-T)`, `M_y = r_x*T`, 반토크 `di*Qi`), `_body_aerodynamics`
- vehicle_params.py: rotor_positions / rotor_directions / n_max / k_T
- ros2_ws/.../controllers/controller.py (CascadedPID 40~134행, LQR 140~330행, ScheduledLQR 460~500행), hybrid_comparison.py (VirtualNMPC 175~200행, ProperHybrid 205~300행), safety.py (전체), trim.py·nmpc.py는 import부와 `_last_t` 관련부만
- 루트 controller.py / trim.py의 import 스타일

## 실제로 실행해서 검증한 것
1. `frame_utils.quat_ned_to_nwu` + `quat_scalar_first_to_last`로 PX4 수평 자세를 변환 → `q_nwu = [1,0,0,0]`(w=0). `find_trim(P, 0.0)`의 트림 쿼터니언도 정확히 `[1,0,0,0]`이라 부호 정규화 경계와 동작점이 정확히 일치함을 확인.
2. `LQRController._compute_error_state`에 롤 ±2°를 넣어 dphi_x 부호 반전 확인. 이어서 실제 `LQRController.__call__` 출력이 롤 +2°와 -2°에서 동일한 모터 명령 [570.5, 573.1, 573.1, 570.5]를 내는 것을 확인 (findings 1의 근거).
3. macOS에서 sync_controllers.sh의 `sed -i -E` 명령을 재현 → `\1 not defined in the RE`, exit 1 확인.
4. VirtualNMPC(max_iter=5) 솔브 시간 첫 33ms/평균 25ms, NMPCController 기본 첫 275ms/평균 63ms 실측.
5. LQRController(V=0)가 valid=True, max_real=-0.49로 정상 설계됨을 확인 (즉 LQR 경로는 죽은 코드가 아님).

## 확인했으나 문제 없다고 판단한 것 (정직한 반대 증거)
- **frame_utils의 좌표 변환 자체는 올바르다.** `quat_multiply`는 정상적인 Hamilton product(R(p⊗q)=R(p)R(q))이고, `_Q_NWU_TO_NED=[1,0,0,0]`(scalar-last)는 Rx(180°)가 맞으며, 위치·속도의 `[x,-y,-z]`와 일관된다. PX4 수평 자세 → 우리 호버 `[1,0,0,0]`이 `find_trim` 결과와 정확히 일치하는 것으로 교차 검증했다. findings 1은 변환식이 아니라 그 다음 단계의 **부호 정규화 기준**이 문제다.
- 각속도 무변환(FRD↔FRD)은 dynamics.py의 로터 배치(r0/r3의 y=+s가 우측)와 z-down 규약으로 볼 때 맞다.
- MOTOR_MAP=(0,2,1,3)은 주석이 밝힌 gz 순서(0=FR,1=RL,2=FL,3=RR)와 우리 순서(r0=FR,r1=FL,r2=RL,r3=RR — vehicle_params.py의 rotor_positions로 확인)에 대해 자기 일관적이다. 다만 **실제 gz SDF가 저장소에 없어 gz 쪽 순서·회전방향은 검증 불가**.
- test_motor의 tm==11(피치), tm==12(요) 분기는 dynamics.py 수식과 대조해 물리적으로 맞다(tm==12의 r0,r2는 rotor_directions=[1,-1,1,-1]에서 같은 회전군이고 롤/피치 모멘트가 상쇄됨). tm==10만 "순수 롤(+)" 라벨이 틀렸고(주석 안의 "우측 상승" 설명이 맞음), findings 8의 suggested_fix에 부기했다.
- OffboardControlMode는 100Hz로 루프 최상단에서 무조건 발행되므로 2Hz 요구를 만족하고, ARM 전에도 zero 모터 명령을 발행한다. 타임스탬프 μs 변환도 px4_ros_com 표준 관례와 같다.
- QoS(BEST_EFFORT/VOLATILE/KEEP_LAST 1)는 PX4 uXRCE-DDS out 토픽과 호환된다(TRANSIENT_LOCAL 구독이 오히려 불일치이며 주석대로 이미 수정됨).
- **스레드 안전성 문제는 없다**: rclpy 기본 SingleThreadedExecutor라 콜백과 타이머가 같은 스레드에서 직렬 실행된다. 공유 상태 경합은 발생하지 않는다.
- 알려진 함정 재확인: INDI G행렬 전진비(hybrid_comparison.py의 `fac` 반영 유지), 폴백의 NaN 검사 순서(safety.py `check`에서 Layer1이 최우선) 모두 회귀 없음. 다만 함정 #1(reset의 `_last_t` 초기화)은 제어기 내부에는 남아 있으나 ROS 노드가 reset을 전혀 호출하지 않아 실질적으로 재발했다 → findings 4.

## 확신이 없거나 확인 못 한 부분
- `/fmu/out/vehicle_status_v4` 토픽명, `VehicleOdometry.angular_velocity`가 PX4 v1.18에서 실제로 채워지는지(NaN이 아닌지), `ActuatorMotors.control`의 채널 수 — macOS에 px4_msgs가 없어 검증 불가. 노드의 [DIAG] 로그가 omega를 출력하도록 되어 있고 kj가 실제로 돌린 흔적이 있어 문제로 보고하지 않았다.
- `n_max=1800`이 커스텀 gz 모델 SDF의 `maxRotorVelocity`와 일치하는지 — SDF가 저장소에 없어(CLAUDE.md에 Ubuntu 로컬에만 있다고 명시) 검증 불가. 불일치하면 모든 추력이 상수배로 틀어지므로 Ubuntu에서 한 번 대조를 권한다.
- `controller_type:=nmpc`의 솔브 블로킹: 실측 평균 63ms/최대 275ms(macOS)로 PX4의 offboard timeout(500ms)에는 못 미치므로 failsafe 유발은 확인되지 않았다. 다만 dt_ctrl=0.02s보다 솔브가 오래 걸려 100Hz 루프가 실질 ~16Hz로 떨어지고, SafetyGuard의 변화율 제한과 offboard_node.py:347-349의 `_motor_speeds` 1차 지연 모델(alpha = dt/(dt+tau_m), dt=0.01 가정)이 그만큼 부정확해진다. **영향 확인 필요** 항목으로 남기고 findings에는 넣지 않았다.
- package.xml의 `buildtool_depend: ament_cmake` vs `build_type: ament_python` 불일치와 numpy/scipy/casadi exec_depend 누락은 colcon 빌드를 실제로 깨뜨리지 않아(빌드 타입은 export가 결정) 보고에서 제외했다.
- run_sitl.sh는 `set -euo pipefail`이 있고 경로도 `${HOME}` 기반이라 별다른 결함을 찾지 못했다. setup.py / setup.cfg도 표준 ament_python 템플릿과 동일하며 launch glob이 `sitl_offboard.launch.py`를 정상 매칭함을 확인했다.

</details>

<details>
<summary><b>cross-cutting</b> — 교차 검사 (중복·문서 불일치·구조)</summary>

## 끝까지 읽은 파일 (전문)\n- /Users/kj/Desktop/dynamic/fast_drone/dynamics.py (295행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/vehicle_params.py (85행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/controller.py (817행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/hybrid_comparison.py (580행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/nmpc.py (344행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/trim.py (196행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/fallback_controller.py (190행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/ekf_sim.py (377행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/ros2_ws/.../frame_utils.py (244행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/ros2_ws/.../safety.py (229행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/ros2_ws/.../offboard_node.py (652행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/ros2_ws/.../setup.py, launch/sitl_offboard.launch.py\n- /Users/kj/Desktop/dynamic/fast_drone/scripts/sync_controllers.sh (45행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/model.sdf (79행 전부)\n- /Users/kj/Desktop/dynamic/fast_drone/HANDOFF.md, results/MISSION_ANALYSIS.md (전부)\n- /Users/kj/Desktop/dynamic/fast_drone/bench_jit_diag.py, horizon_comparison.py (신규 파일, 전부)\n\n## 부분만 읽은 파일 (해당 섹션만 확인, 나머지 미확인)\n- mission_sim.py: 1~330행, 670~858행을 읽었고 330~670행은 진단/플롯/출력 헬퍼라 grep으로만 훑음\n- sweep_plots.py: 1~210행만 읽음. 211행 이후(플롯 생성부)는 미확인\n- sensors.py: 90~140행, 205~295행만 읽음. IMU/GPS 노이즈 모델 앞부분 미확인\n- estimator.py: 55~110행, update_gps(112~160행)만 읽음. predict/update_accel/_inject 본문 미확인\n- ekf_comparison.py, gust_comparison.py: grep + _reset_controller/make_gust_fn 주변만 확인\n- bench_compute.py, bench_accel_experiments.py: grep 수준만. 전문 미독\n- scripts/sitl_run.sh, run_sitl.sh, setup_ubuntu.sh: grep 수준만\n- TODO.md, SETUP.md: 섹션 존재 여부만 grep으로 확인, 내용 대조 안 함\n\n## 확신도에 대한 정직한 기술\n- **사전 조사 검증 결과: 맞았습니다.** 루트 7개 파일과 ros2_ws/controllers/ 동명 사본을 `diff -u`로 전부 대조한 결과, 차이는 상대임포트 접두사(`from x` → `from .x`)뿐이고 로직/수치는 완전히 동일했습니다. dynamics.py / fallback_controller.py / vehicle_params.py는 바이트 단위로 동일(diff exit 0)했습니다. 따라서 '사본 드리프트'는 현재 없습니다. 다만 그 동기화를 담당하는 sync_controllers.sh 자체가 macOS에서 깨지므로 이를 최우선으로 보고했습니다.\n- 1번 발견(sed)은 스크래치패드에 저장소 구조를 복제해 **동일 스크립트를 실제로 실행**해 재현했습니다(사용자 저장소는 건드리지 않음). 종료코드 1, DST에 controller.py만 절대 import 상태로 생성됨을 확인했습니다.\n- 2번 발견(heading 래치)은 추측이 아니라 5개 제어기 클래스를 **실제로 인스턴스화해 hasattr을 출력**해 확인했습니다.\n- **부호/좌표계는 규약대로 dynamics.py 원문 수식과 하나씩 대조했고, 결과적으로 결함을 발견하지 못했습니다.** 구체적으로: (a) _body_aerodynamics의 M_static = [0, -x_cp·Fz, x_cp·Fy]가 r_cp×F 전개와 일치하고 x_cp<0에서 복원모멘트가 나오는 것, (b) 로터 모멘트 [ry·(-T), -rx·(-T), 0]가 compute_allocation_matrix의 cross(pos, [0,0,-1])과 일치하는 것, (c) INDI G의 dT/dn = k_T·n·(1+fac)가 T=k_T·n²·fac의 해석적 미분과 정확히 같다는 것(2·k_T·n·fac + k_T·n·(1−fac) = k_T·n·(1+fac)), (d) _quat_derivative의 G^T가 scalar-last [x,y,z,w] 바디레이트 운동학과 일치하는 것, (e) frame_utils의 NWU↔NED 좌우곱 순서와 호버 쿼터니언 왕복, (f) vehicle_params의 dir=[1,-1,1,-1]과 model.sdf의 turningDirection(ccw,ccw,cw,cw) + gz motorNumber 배치(0=FR,1=RL,2=FL,3=RR)가 대각 짝을 이루어 요 부호가 정합하는 것 — 전부 코드 라인 기준으로 확인했고 독스트링과 어긋나는 곳은 없었습니다. **vehicle_params 주석이 dir=+1을 'CW'라 부르는 것은 z-down 시점 기준이라 gz의 'ccw'(위에서 본 기준)와 같은 회전이며, 부호 버그가 아닙니다.**\n- 이미 수정된 함정 4건은 모두 **회귀하지 않았음**을 확인했습니다: NMPC/VirtualNMPC reset()의 _last_t+w0 초기화(nmpc.py:71-76, hybrid_comparison.py:109-114), INDI G 전진비(controller.py:666, hybrid_comparison.py:350), ESKF update_gps의 R_pos/R_vel 전달(ekf_sim.py:103-105, 표준편차→분산 변환도 estimator.py:138에서 올바름), 폴백 NaN 검사가 채터링 가드보다 앞(fallback_controller.py:123 vs 130).\n- **확신이 낮은 부분**: 10번(model.sdf)은 git 미추적·미참조 파일이라 impact_verified=false로 표시했습니다. 8번(INDI 고정 dt)은 SITL 실제 루프 지터를 측정하지 않아 영향 크기를 확정하지 못했고 severity를 low로 낮췄습니다. 5번·3번은 코드 도달성은 확인했지만 실제 오작동은 vehicle_params가 바뀌어야 발현되는 잠복 결함입니다.\n- **순환 임포트/sys.path 조작은 발견되지 않았습니다.** 루트 스크립트들은 모두 같은 디렉토리의 형제 모듈을 평범한 절대 임포트로 참조하고, ros2_ws 패키지는 `from .controllers.x`로만 참조하며 루트 모듈을 잘못 가리키는 곳은 없습니다. setup.py의 find_packages가 controllers 서브패키지를 포함하는 것도 확인했습니다(__init__.py 존재). 활성 코드가 legacy/를 임포트하는 경우도 없었습니다(sweep_plots.py는 주석에서 legacy를 언급만 하고 재구현했습니다).

</details>

## 12. 재현

이 리뷰는 Claude Code의 Workflow 오케스트레이션으로 생성되었습니다.

```
스크립트: ~/.claude/projects/-Users-kj-Desktop-dynamic-fast-drone/
          <session>/workflows/scripts/fast-drone-full-review-wf_ca78d6c6-349.js
원본 JSON: 같은 세션의 tasks/wt7s3k8ql.output
저널:      .../subagents/workflows/wf_ca78d6c6-349/journal.jsonl
```

사람 검증 스크립트(§3)는 `results/verify_review_claims.py`로 승격했습니다.

```bash
python3 results/verify_review_claims.py
```
