#!/usr/bin/env python3
"""겹 1 기준값 생성기 — DESIGN.md §4.

    python3 gz_aero/tools/gen_reference.py --source sized

**기준값은 표를 거치지 않는다.** 프로젝트가 이미 신뢰하는
`control.dynamics._body_aerodynamics()` 를 직접 불러서 만든다. 표를 써서 기준을
만들면 "표로 만든 값을 표로 검산"하는 순환논증이 된다.

C++ 코어는 같은 상태를 넣었을 때
  - 격자점 위(grid=1)  : 상대오차 < 1e-9   (보간오차가 0이므로 수식 이식만 잰다)
  - 격자 사이(grid=0)  : 오차를 **보고**만 한다 (격자가 촘촘한지를 잰다)
를 만족해야 한다.

상태 구성
---------
    u = V*cos(alpha)
    v = V*sin(alpha)*cos(phi)
    w = V*sin(alpha)*sin(phi)
phi 는 크로스플로의 방향각이다. phi=0 이면 순수 옆미끄럼(+y), phi=90 이면
순수 받음각(+z). **축대칭이므로 힘의 크기는 phi 에 무관해야 한다** — 그걸
swap_yz_a/b 쌍이 확인한다.

부호 사고 검출
--------------
`asym_*` 케이스는 u,v,w,p,q,r 여섯 성분이 전부 0이 아니고 서로 다르다.
대칭 상태(v=0, p=0)만으로는 y<->z 뒤바뀜이 그냥 통과한다 — 그게 이 프로젝트에서
이미 한 번 일어난 사고다(쿼터니언 scalar-last).
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import casadi as ca

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aero_sources import get_source                       # noqa: E402
from gen_aero_csv import adaptive_v_grid, adaptive_alpha_grid  # noqa: E402
from control.dynamics import _body_aerodynamics           # noqa: E402

# 표와 **같은** 적응격자를 main() 에서 채운다. 따로 적으면 언젠가 어긋나고,
# 어긋나면 test A 의 "격자점 위" 전제가 조용히 깨진다.
V_GRID: list = []
ALPHA_GRID: list = []


def _snap(value, grid):
    """요청값을 격자에서 가장 가까운 점으로 당긴다.

    격자가 적응식이라 케이스에 적은 alpha=17.5도 같은 값이 격자에 없을 수 있다.
    격자가 바뀔 때마다 케이스를 손보게 두면 언젠가 "격자점인 줄 알았는데 아닌"
    상태로 test A 가 통과해 버린다. 그래서 항상 당긴다.
    """
    return min(grid, key=lambda g: abs(g - value))

# (name, V[m/s], alpha[deg], phi[deg], p, q, r)
GRID_CASES = [
    ("zero",           0.0,    0.0,    0.0,  0.0,  0.0,  0.0),   # 0/0 특이점
    ("axial_low",     20.0,    0.0,    0.0,  0.0,  0.0,  0.0),   # alpha=0
    ("axial_cruise",  83.3,    0.0,    0.0,  0.0,  0.0,  0.0),
    ("pure_roll",     60.0,    0.0,    0.0,  3.0,  0.0,  0.0),   # 감쇠 x축만
    ("pure_pitch",    60.0,    0.0,    0.0,  0.0,  2.0,  0.0),   # 감쇠 y축만
    ("pure_yaw",      60.0,    0.0,    0.0,  0.0,  0.0, -1.5),   # 감쇠 z축만
    ("aoa_pitch_up",  83.3,    7.5,   90.0,  0.0,  0.0,  0.0),   # 크로스플로 +z
    ("aoa_pitch_dn",  83.3,    7.5,  -90.0,  0.0,  0.0,  0.0),   # 크로스플로 -z
    ("sideslip_pos",  83.3,    7.5,    0.0,  0.0,  0.0,  0.0),   # 크로스플로 +y
    ("sideslip_neg",  83.3,    7.5,  180.0,  0.0,  0.0,  0.0),   # 크로스플로 -y
    ("crossflow_90",  40.0,   90.0,    0.0,  0.0,  0.0,  0.0),   # alpha=90
    ("reversed_135",  30.0,  135.0,   30.0,  0.0,  0.0,  0.0),   # 역류
    ("reversed_180",  20.0,  180.0,    0.0,  0.0,  0.0,  0.0),   # 완전 역류
    ("swap_yz_a",     60.0,   25.0,    0.0,  0.0,  0.0,  0.0),   # 축대칭 쌍 A
    ("swap_yz_b",     60.0,   25.0,   90.0,  0.0,  0.0,  0.0),   # 축대칭 쌍 B
    ("asym_1",        50.0,   17.5,   37.0,  1.3, -2.7,  0.9),   # 부호사고 검출기
    ("asym_2",       100.0,   32.5, -113.0, -4.1,  0.6, -2.2),   # 부호사고 검출기
    ("asym_3",        10.0,   62.5,  200.0,  0.7,  5.5, -3.3),   # 저속 고받음각
    ("tilt_40",       83.3,   40.0,   90.0,  0.5, -0.5,  0.2),   # 고속 틸트
    ("hover_gust",     5.0,   85.0,  145.0,  2.0,  1.0, -1.0),   # 호버 돌풍
]

def offgrid_sweep():
    """격자 **모든 칸의 중점**을 훑는다 — 선형보간 오차가 최대가 되는 자리다.

    점 몇 개만 찍어 보고 "1% 안에 든다"고 말하는 건 운에 기대는 것이다.
    칸 전부를 훑는 비용이 어차피 없다.

    omega = 0 으로 둔다: C_lp/C_mq 가 격자 위에서 상수라 감쇠는 보간오차가
    0 인데, 그 항이 힘에 섞이면 지표가 희석돼 보간 품질이 좋아 보인다.
    """
    cases = []
    for iv in range(len(V_GRID) - 1):
        Vm = 0.5 * (V_GRID[iv] + V_GRID[iv + 1])
        for ia in range(len(ALPHA_GRID) - 1):
            am = math.degrees(0.5 * (ALPHA_GRID[ia] + ALPHA_GRID[ia + 1]))
            # 크로스플로 방향을 칸마다 돌린다 — 한 방향만 쓰면 y/z 편향이 안 걸린다
            phi = (37.0 + 13.0 * (iv + 2 * ia)) % 360.0
            cases.append((f"s{iv:02d}_{ia:03d}", Vm, am, phi, 0.0, 0.0, 0.0))
    return cases


COLUMNS = ["case", "grid", "u", "v", "w", "p", "q", "r",
           "Fx", "Fy", "Fz", "Mx", "My", "Mz"]


EPS = 1e-8          # control/dynamics.py:23 과 같은 값


def state_from(V: float, alpha_deg: float, phi_deg: float):
    """(V, alpha, phi) -> 동체 대기속도 (u, v, w).

    ⚠ 소박하게 u = V cos a, v = V sin a cos p 로 두면 안 된다.
    플러그인은 속도 크기를 `V = sqrt(u^2+v^2+w^2 + EPS)` 로 계산한다
    (파이썬 원본과 같은 관례). 그래서 그냥 V 를 쓰면 표를 조회하는 속도가
    격자점에서 EPS/(2V) 만큼 비껴가고, 보간오차가 섞여 들어와 "수식 이식
    오류"와 구분이 안 된다.

    그래서 EPS 를 미리 빼둔 크기로 성분을 만든다:
        V0 = sqrt(V^2 - EPS)
        u = V0 cos a,  v = V0 sin a cos p,  w = V0 sin a sin p
    이러면
        sqrt(u^2+v^2+w^2 + EPS) = sqrt(V0^2 + EPS) = V        (정확)
        atan2(sqrt(v^2+w^2), u) = a                           (정확)
    라서 조회점이 (V, alpha) 격자점에 **정확히** 떨어진다.

    V=0 만 예외다 (뺄 EPS 가 없다). 그 상태의 힘은 1e-11 N 수준이라
    절대 허용오차가 흡수한다.
    """
    a, ph = math.radians(alpha_deg), math.radians(phi_deg)
    V0 = math.sqrt(max(V * V - EPS, 0.0))
    return (V0 * math.cos(a),
            V0 * math.sin(a) * math.cos(ph),
            V0 * math.sin(a) * math.sin(ph))


def reference_wrench(src, V, u, v, w, p, q, r):
    """`_body_aerodynamics` 를 그대로 호출한다 — 여기가 기준의 단일 출처."""
    params = src.dyn_params(V)
    F, M = _body_aerodynamics(ca.DM([u, v, w]), ca.DM([p, q, r]), params)
    F = [float(x) for x in ca.DM(F).elements()]
    M = [float(x) for x in ca.DM(M).elements()]
    return F, M


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["placeholder", "sized"], default="sized")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    src = get_source(args.source)
    # ★ 표와 **같은** 격자를 써야 한다. 따로 적으면 언젠가 어긋나고,
    #   어긋나면 test A 의 "격자점 위" 전제가 조용히 깨진다.
    global V_GRID, ALPHA_GRID
    V_GRID = adaptive_v_grid(src)
    ALPHA_GRID = adaptive_alpha_grid(src)
    out = args.out or os.path.join(_REPO, "gz_aero", "test",
                                   f"aero_reference_{args.source}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    lines = [
        "# fast_drone aero reference v1  (DESIGN.md 4장 겹1)",
        "# 생성: gz_aero/tools/gen_reference.py — 기준은 control/dynamics.py "
        "_body_aerodynamics() 직접 호출",
        "# frame: body FRD (x=nose, y=right, z=down).  F [N], M [N m], omega [rad/s]",
        "# grid=1: (V,alpha) 가 표의 격자점 위 -> 상대오차 1e-9 요구",
        "# grid=0: 격자 사이 -> 보간오차 측정용, 실패 판정 안 함",
        f"# source: {src.name} / {src.meta.get('source', '?')}",
        f"# S_ref: {src.S_ref:.17g}",
        f"# d_ref: {src.d_ref:.17g}",
        f"# rho_ref: {src.rho:.17g}",
        ",".join(COLUMNS),
    ]

    def emit(cases, on_grid, verbose):
        out_lines = []
        for name, V, a_deg, phi_deg, p, q, r in cases:
            if on_grid:
                # 격자점 위로 당긴다 (위 _snap 주석 참고)
                V = _snap(V, V_GRID)
                a_deg = math.degrees(_snap(math.radians(a_deg), ALPHA_GRID))
            u, v, w = state_from(V, a_deg, phi_deg)
            F, M = reference_wrench(src, V, u, v, w, p, q, r)
            out_lines.append(
                f"{name},{on_grid},{u:.17g},{v:.17g},{w:.17g},"
                f"{p:.17g},{q:.17g},{r:.17g},"
                f"{F[0]:.17g},{F[1]:.17g},{F[2]:.17g},"
                f"{M[0]:.17g},{M[1]:.17g},{M[2]:.17g}")
            if verbose:
                print(f"{name:<14} {on_grid:>4} {a_deg:>7.2f} "
                      f"{F[0]:>11.4f} {F[1]:>11.4f} {F[2]:>11.4f} "
                      f"{M[0]:>11.4f} {M[1]:>11.4f} {M[2]:>11.4f}")
        return out_lines

    # ── test A 용: 이름 붙인 격자점 케이스 ──
    print(f"{'case':<14} {'grid':>4} {'alpha':>7} "
          f"{'Fx':>11} {'Fy':>11} {'Fz':>11} {'Mx':>11} {'My':>11} {'Mz':>11}")
    print("-" * 96)
    lines += emit(GRID_CASES, 1, True)
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("-" * 96)
    print(f"기록: {out}  (격자점 케이스 {len(GRID_CASES)}개)")

    # ── test B 용: 전 격자칸 중점 스윕 ──
    sweep = offgrid_sweep()
    sweep_out = out.replace("aero_reference_", "aero_interp_")
    sweep_lines = lines[:len(lines) - len(GRID_CASES)]
    sweep_lines = [l for l in sweep_lines if l.startswith("#") or l.startswith("case,")]
    with open(sweep_out, "w", encoding="utf-8") as f:
        f.write("\n".join(sweep_lines + emit(sweep, 0, False)) + "\n")
    print(f"기록: {sweep_out}  (격자칸 중점 {len(sweep)}개 = "
          f"V칸 {len(V_GRID)-1} x alpha칸 {len(ALPHA_GRID)-1})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
