#!/usr/bin/env python3
"""
전체 코드리뷰(results/CODE_REVIEW_2026-08-04.md) 중 실기 손실로 직결되는
2건을 코드로 직접 재현하는 검증 스크립트.

실행:
    python3 results/verify_review_claims.py

검증 대상
---------
[A] Issue 1 — 쿼터니언 부호 뒤집힘 (ROS2/PX4 경로 전용)
    offboard_node.py:380 / frame_utils.py:156 의 `if q_nwu[3] < 0: q_nwu = -q_nwu` 가
    controller.py:291/476 의 최단경로 보정 부재와 결합해 LQR 자세 오차 부호를 뒤집는가?

[B] Issue 3(d) — INDI 각가속도 LPF의 NaN 영구 고착
    hybrid_comparison.py:277-279 의 자기참조 LPF가 NaN 1회로 영구 고착되고,
    np.linalg.solve 의 except 절이 그것을 잡지 못하는가?

발견 요약
---------
[A] 성립. 이 기체의 트림 쿼터니언은 전 속도에서 스칼라부 w = q[3] 이 정확히 0이라
    'w>0 정규화'의 부호 경계가 정상 비행 자세 위에 놓인다. 롤 ±10° 가 동일한
    dphi_x = -0.17431 을 내어 부호 정보가 소실된다. 1줄 보정으로 완전 복구.
    파이썬 시뮬 경로는 정규화를 하지 않아 dq[3] >= 0.866 → 수치 불변이어야 정상.
[B] 성립. NaN 1회 주입 후 클린 입력 6스텝을 넣어도 필터가 NaN을 유지하고,
    np.linalg.solve 는 NaN 우변에 LinAlgError 를 던지지 않아 _fallback 이 미도달.

규약 메모 (CLAUDE.md 최대 함정)
------------------------------
이 프로젝트의 쿼터니언은 **스칼라-last [x, y, z, w]** 이다 (dynamics.py:16, _quat_to_rotmat).
호버 쿼터니언 [1,0,0,0] 은 x=1, w=0 즉 **x축 180° 회전** (동체 z-down <-> 관성 z-up).
스칼라-first 로 오독하면 이 스크립트의 결론이 통째로 뒤집히므로 주의.
"""
import os
import sys

import numpy as np

# 저장소 루트를 이 파일 위치에서 유도 (하드코딩 경로 금지 — CLAUDE.md 규약)
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from control.vehicle_params import vehicle_params as P          # noqa: E402
from control.trim import find_trim                              # noqa: E402
from control.controller import LQRController                    # noqa: E402


def banner(s):
    print("\n" + "=" * 76 + f"\n{s}\n" + "=" * 76)


# ══════════════════════════════════════════════════════════════════
# [A] 쿼터니언 부호 뒤집힘
# ══════════════════════════════════════════════════════════════════

def q_roll_from_hover(phi_deg):
    """호버 자세(x축 180°)에서 동체 x축으로 phi_deg 만큼 추가 롤. scalar-last."""
    q_hover = np.array([1.0, 0.0, 0.0, 0.0])            # [x,y,z,w]
    h = np.deg2rad(phi_deg) / 2.0
    q_roll = np.array([np.sin(h), 0.0, 0.0, np.cos(h)])
    return LQRController._quat_mult(q_hover, q_roll)


def normalize_w_positive(q):
    """offboard_node.py:380-381 / frame_utils.py:156-157 과 동일한 연산."""
    return -q if q[3] < 0 else q.copy()


def dphi_from(q, q_trim, shortest_path):
    """controller.py:289-293 의 오차각 계산. shortest_path=True 가 제안된 수정."""
    q_inv = np.array([-q_trim[0], -q_trim[1], -q_trim[2], q_trim[3]])
    dq = LQRController._quat_mult(q_inv, q)
    if shortest_path and dq[3] < 0.0:
        dq = -dq
    return 2.0 * dq[0:3], dq[3]


def check_quaternion_sign():
    banner("[A-1] 트림 쿼터니언의 스칼라부 w = q[3] 가 실제로 0인가")
    print(f"{'V (m/s)':>8} | {'q_trim (scalar-last [x,y,z,w])':^42} | {'w = q[3]':>11}")
    print("-" * 76)
    q_trim_hover = None
    for V in [0.0, 10.0, 30.0, 50.0, 85.0]:
        x_tr = np.asarray(find_trim(P, V)['state']).flatten().astype(float)
        q = x_tr[6:10]
        if V == 0.0:
            q_trim_hover = q.copy()
            x_hover = x_tr.copy()
        print(f"{V:8.1f} | [{q[0]:9.6f} {q[1]:9.6f} {q[2]:9.6f} {q[3]:9.6f}] | {q[3]:11.3e}")
    print("\n→ w = 0 이면 'w>0 정규화'의 부호 경계가 정상 비행 자세 위에 놓임.")

    banner("[A-2] 롤 ±10°/±30° → w>0 정규화 통과 후 오차각 dphi_x")
    print(f"{'roll':>7} | {'flip':^5} | {'dq[3]':>9} | {'dphi_x (현재)':>13} | {'dphi_x (수정後)':>15}")
    print("-" * 76)
    cur = {}
    for phi in [+10.0, -10.0, +30.0, -30.0]:
        q_raw = q_roll_from_hover(phi)
        q_px4 = normalize_w_positive(q_raw)
        flipped = not np.allclose(q_raw, q_px4)
        d_cur, dq3 = dphi_from(q_px4, q_trim_hover, shortest_path=False)
        d_fix, _ = dphi_from(q_px4, q_trim_hover, shortest_path=True)
        cur[phi] = d_cur[0]
        print(f"{phi:6.1f}° | {str(flipped):^5} | {dq3:9.5f} | "
              f"{d_cur[0]:13.5f} | {d_fix[0]:15.5f}")

    same_sign = np.sign(cur[+10.0]) == np.sign(cur[-10.0])
    print(f"\n롤 +10°: dphi_x = {cur[+10.0]:+.5f} / 롤 -10°: dphi_x = {cur[-10.0]:+.5f}")
    print("→ " + ("부호가 같음 = 롤 오차 부호 정보 소실. **지적 성립**"
                  if same_sign else "부호가 다름 = 지적 불성립"))

    banner("[A-3] 파이썬 시뮬 경로는 영향을 받는가 (기존 실험 수치가 바뀌는가)")
    print("시뮬은 적분기가 q를 연속 출력하고 w>0 정규화를 하지 않음.")
    print("dq[3] 가 항상 양수면 최단경로 보정은 무작동 = 수치 불변이어야 정상.\n")
    worst = 1.0
    for phi in np.arange(-60, 61, 5.0):
        for axis in range(3):
            h = np.deg2rad(phi) / 2.0
            v = np.zeros(3)
            v[axis] = np.sin(h)
            q = LQRController._quat_mult(np.array([1.0, 0.0, 0.0, 0.0]),
                                         np.array([v[0], v[1], v[2], np.cos(h)]))
            _, dq3 = dphi_from(q, q_trim_hover, shortest_path=False)
            worst = min(worst, dq3)
    print(f"정규화 없는 경로, 자세오차 ±60°/3축 전수 스캔 → dq[3] 최솟값 = {worst:.5f}")
    print("→ " + ("양수 = 시뮬 경로 무영향. 수정 후에도 test_plant/mission_sim 수치 불변이어야 정상."
                  if worst > 0 else "음수 = 시뮬도 영향 받음. 재실행 필요."))
    return same_sign, worst


# ══════════════════════════════════════════════════════════════════
# [B] INDI 각가속도 LPF 의 NaN 영구 고착
# ══════════════════════════════════════════════════════════════════

def check_nan_latching():
    banner("[B-1] 자기참조 LPF의 NaN 영구 고착 (hybrid_comparison.py:277-279 재현)")
    alpha, dt = 0.03, 0.01                  # 실제 코드와 동일 오더
    filt = np.zeros(3)
    prev = np.zeros(3)
    seq = ([np.array([0.1, 0.1, 0.1])] * 2
           + [np.array([np.nan, 0.0, 0.0])]          # NaN 1회 주입
           + [np.array([0.1, 0.1, 0.1])] * 6)        # 이후 완전히 깨끗한 입력
    for k, om in enumerate(seq):
        raw = (om - prev) / dt
        filt = alpha * raw + (1 - alpha) * filt
        prev = om.copy()
        tag = "  <-- NaN 1회 주입" if np.any(np.isnan(om)) else ""
        print(f"  step {k}: omega={np.array2string(om, precision=2):>18}  "
              f"filt={np.array2string(filt, precision=4)}{tag}")
    latched = not np.all(np.isfinite(filt))
    print(f"\n→ 주입 6스텝 뒤에도 NaN 잔존? {latched}  "
          + ("**지적 성립**" if latched else "(고착 없음 = 지적 불성립)"))

    banner("[B-2] np.linalg.solve 가 NaN 우변에 LinAlgError 를 던지는가")
    G = np.eye(4) * 0.5 + 0.01
    dv = np.array([1.0, np.nan, 0.0, 0.0])
    try:
        dn = np.linalg.solve(G, dv)
        caught = False
        print(f"  예외 없음. 반환값 = {dn}")
        print("  → `except np.linalg.LinAlgError` 절의 _fallback 은 발동하지 않음")
    except np.linalg.LinAlgError as e:
        caught = True
        print(f"  LinAlgError 발생: {e}")
    print(f"  np.clip(nan, 0, 1800) = {np.clip(np.nan, 0, 1800)} → 최종 클립도 NaN 통과")

    banner("[B-3] 현재 hybrid_comparison.py 에 유한성 가드가 있는가")
    src = open(os.path.join(REPO, 'hybrid_comparison.py')).read()
    n_guard = src.count('isfinite') + src.count('isnan')
    print(f"  isfinite/isnan 등장 횟수: {n_guard}")
    print("  (433행 발산 판정 1건만 있으면 제어 경로에는 가드가 없다는 뜻)")
    return latched, (not caught)


if __name__ == '__main__':
    same_sign, worst = check_quaternion_sign()
    latched, solve_silent = check_nan_latching()

    banner("종합")
    print(f"  [A] 쿼터니언 부호 뒤집힘        : {'재현됨 (지적 성립)' if same_sign else '재현 안 됨'}")
    print(f"      파이썬 시뮬 영향            : {'없음 (수치 불변 예상)' if worst > 0 else '있음 (재실행 필요)'}")
    print(f"  [B] INDI NaN 영구 고착          : {'재현됨 (지적 성립)' if latched else '재현 안 됨'}")
    print(f"      solve 가 NaN 을 조용히 통과 : {'예 (except 무효)' if solve_silent else '아니오'}")
