#!/usr/bin/env python3
"""공력 계수 CSV 생성기 — DESIGN.md §5 스키마.

    python3 -m gz_aero.tools.gen_aero_csv --source sized
    python3 gz_aero/tools/gen_aero_csv.py --source placeholder --out /tmp/x.csv

격자를 (V, alpha) 정규격자로 훑어 무차원 계수를 뽑는다. 공력 담당이 CFD/풍동
결과를 줄 때는 **이 파일이 만든 CSV 를 같은 형식으로 덮어쓰면** 코드 수정 없이
반영된다 — 그게 이 설계의 목적이다.

격자 선택 근거
--------------
V : 비균일. 팀 CD0(V) 가 저속에서 가파르다 (0.783@1 -> 0.443@83 m/s. Re 의존).
    고속은 완만하므로 성기게 잡아도 된다. 보간기가 비균일 격자를 받으므로
    (이분탐색) 균일하게 맞출 이유가 없다.
alpha : 균일 2.5도, [0, 180]. C_N 의 2계도함수 크기가 ~2*(CN_a + K) ~ 40 이라
    선형보간 오차 ~ (h^2/8)*|f''| = (0.0436^2/8)*40 = 0.0095.
    C_N 최대값 ~11.5 대비 0.08% — 목표 1% 를 크게 밑돈다.
    (실제 오차는 test B 가 측정한다. 이 계산은 격자를 고르는 근거일 뿐이다.)
"""
from __future__ import annotations

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aero_sources import get_source          # noqa: E402

# ── 격자 ──────────────────────────────────────────────────────────────
#
# 두 축 다 **적응 격자**다. 손으로 점을 찍지 않는다.
#
# 기준은 계수 오차가 아니라 **힘 오차**다. 이게 핵심이다:
#   계수 기준(|dC| < tol)으로 쪼개면 저속에서 CD0(V) 가 가팔라 점이 잔뜩 생기는데,
#   V=1.5 m/s 에서 dC_A=1e-3 이 만드는 힘은 9e-6 N 이다 — 비행에 아무 영향이 없다.
#   반대로 고속 고받음각에서는 dC_N=7e-3 이 0.35 N (기체 무게의 2.2%) 을 만든다.
#   즉 "계수가 얼마나 틀렸나" 가 아니라 "그게 힘으로 얼마가 되나" 로 쪼개야 한다.
#
# 셀 [a,b] 의 중점 m 에서 C(m) 과 (C(a)+C(b))/2 를 비교한다 — 중점이 선형보간의
# 최악점이라 이 차이가 그 셀의 오차 상한이다.
F_TOL_N = 0.02            # [N] 허용 힘 오차. 기체 무게 16.3 N 의 0.12%
V_MAX_TABLE = 120.0       # [m/s] 표의 상한
_MAX_PTS = 400

_V_SEED = [0.0, 1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 60.0, 83.3, 100.0, V_MAX_TABLE]
_ALPHA_SEED_DEG = [0.0, 15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 150.0, 180.0]
_PROBE_ALPHA_DEG = (0.0, 15.0, 30.0, 45.0, 60.0, 90.0, 135.0, 180.0)


def _refine(seed, err_fn, max_pts=_MAX_PTS):
    """셀 중점의 오차가 허용치를 넘는 셀만 반으로 쪼개기를 반복한다.

    err_fn(a, b, m) -> 그 셀의 힘 오차 [N]. F_TOL_N 을 넘으면 쪼갠다.
    """
    grid = list(seed)
    for _ in range(24):
        add = [0.5 * (grid[i] + grid[i + 1])
               for i in range(len(grid) - 1)
               if err_fn(grid[i], grid[i + 1], 0.5 * (grid[i] + grid[i + 1])) > F_TOL_N]
        if not add or len(grid) + len(add) > max_pts:
            break
        grid = sorted(grid + add)
    return grid


def _qS(src, V):
    """동압 x 기준면적 [N] — 계수 오차를 힘 오차로 바꾸는 환산계수."""
    return 0.5 * src.rho * V * V * src.S_ref


def adaptive_v_grid(src):
    """V 축: 그 속도에서의 동압으로 잰다. 저속은 힘이 작아 성겨도 된다."""
    def err(a, b, m):
        worst = 0.0
        for adeg in _PROBE_ALPHA_DEG:
            aa = math.radians(adeg)
            cm, ca, cb = src.coeffs(m, aa), src.coeffs(a, aa), src.coeffs(b, aa)
            dCA = abs(cm["C_A"] - 0.5 * (ca["C_A"] + cb["C_A"]))
            dCN = abs(cm["C_N"] - 0.5 * (ca["C_N"] + cb["C_N"]))
            worst = max(worst, _qS(src, m) * max(dCA, dCN))
        return worst
    return _refine(_V_SEED, err)


def adaptive_alpha_grid(src):
    """alpha 축: C_N 은 V 무관이므로 **표 상한 속도**의 동압으로 잰다 (최악조건)."""
    qS_max = _qS(src, V_MAX_TABLE)

    def err(a, b, m):
        cm = src.coeffs(V_MAX_TABLE, m)
        ca = src.coeffs(V_MAX_TABLE, a)
        cb = src.coeffs(V_MAX_TABLE, b)
        dCN = abs(cm["C_N"] - 0.5 * (ca["C_N"] + cb["C_N"]))
        dCA = abs(cm["C_A"] - 0.5 * (ca["C_A"] + cb["C_A"]))
        return qS_max * max(dCA, dCN)

    return _refine([math.radians(d) for d in _ALPHA_SEED_DEG], err)


COLUMNS = ["V_mps", "alpha_rad", "C_A", "C_N", "x_cp", "C_lp", "C_mq"]


def _meta_lines(src) -> list[str]:
    """플러그인이 SDF 와 대조할 항목은 반드시 여기 적힌다 (DESIGN.md §5.1)."""
    out = [
        "# fast_drone aero table v1",
        # 축대칭 가정을 기계가 검사하는 선언 (DESIGN.md 5.2).
        # 일반화 표(asym_v2)를 v1 플러그인이 조용히 읽는 사고를 막는다.
        "# schema: axisym_v1",
        "# frame: body FRD (x=nose, y=right, z=down)",
        "# C_A, C_N: nondimensional on S_ref.  x_cp: body x from CG [m], negative = aft = stable",
        "# C_lp, C_mq: damping derivatives (nondimensional)",
        "# alpha: total angle of attack [rad], 0..pi. Negative alpha follows from axisymmetry.",
        f"# S_ref: {src.S_ref:.17g}",
        f"# d_ref: {src.d_ref:.17g}",
        f"# rho_ref: {src.rho:.17g}",
        # 격자를 만든 기준. 검증 test B 가 이 값으로 판정한다 — 표와 테스트가
        # 따로 숫자를 들고 있으면 언젠가 어긋난다.
        f"# F_tol_N: {F_TOL_N:.10g}",
        f"# source: {src.name} / {src.meta.get('source', '?')}",
    ]
    for key in ("MTOW", "x_cg", "J_xx", "J_yy", "J_zz", "l_body",
                "arm_rotor", "CN_alpha", "x_cp_nose", "SM_cal"):
        if key in src.meta:
            out.append(f"# {key}: {src.meta[key]:.10g}")
    if "design_point" in src.meta:
        dp = " ".join(f"{k}={v}" for k, v in src.meta["design_point"].items())
        out.append(f"# design_point: {dp}")
    return out


def generate(src, v_grid=None, alpha_grid=None) -> list[str]:
    v_grid = adaptive_v_grid(src) if v_grid is None else v_grid
    alpha_grid = adaptive_alpha_grid(src) if alpha_grid is None else alpha_grid

    lines = _meta_lines(src)
    lines.append(f"# n_V: {len(v_grid)}")
    lines.append(f"# n_alpha: {len(alpha_grid)}")
    lines.append(",".join(COLUMNS))

    # 순서 고정: V 바깥 루프, alpha 안쪽 루프 (DESIGN.md §5)
    for V in v_grid:
        for a in alpha_grid:
            c = src.coeffs(V, a)
            lines.append(
                # 계수는 %.17g (double 왕복 무손실). 이 파일은 기계가 만드는
                # 산출물이라 가독성보다 정확도가 먼저다 — 10자리로 자르면
                # 검증 test A 의 오차 바닥이 1e-10 에 걸려 진짜 이식 오류를 가린다.
                f"{V:.17g},{a:.17g},{c['C_A']:.17g},{c['C_N']:.17g},"
                f"{c['x_cp']:.17g},{c['C_lp']:.17g},{c['C_mq']:.17g}")
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["placeholder", "sized"], default="sized")
    ap.add_argument("--out", default=None,
                    help="기본: gz_aero/data/aero_<source>.csv")
    args = ap.parse_args(argv)

    src = get_source(args.source)
    out = args.out or os.path.join(_REPO, "gz_aero", "data",
                                   f"aero_{args.source}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    v_grid = adaptive_v_grid(src)
    alpha_grid = adaptive_alpha_grid(src)
    lines = generate(src, v_grid=v_grid, alpha_grid=alpha_grid)
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    n_rows = len(v_grid) * len(alpha_grid)
    print(f"{out}")
    print(f"  소스     {src.name} ({src.meta.get('source')})")
    print(f"  격자     V {len(v_grid)}점 x alpha {len(alpha_grid)}점 = {n_rows}행 (둘 다 적응)")
    print(f"  허용     힘 오차 {F_TOL_N} N (무게의 {F_TOL_N/(1.660778*9.81)*100:.2f}%)")
    print(f"  V 격자   {[round(v, 3) for v in v_grid]}")
    print(f"  a 격자   {len(alpha_grid)}점, "
          f"최소간격 {min(math.degrees(alpha_grid[i+1]-alpha_grid[i]) for i in range(len(alpha_grid)-1)):.3f}deg "
          f"최대간격 {max(math.degrees(alpha_grid[i+1]-alpha_grid[i]) for i in range(len(alpha_grid)-1)):.3f}deg")
    print(f"  S_ref    {src.S_ref:.8f} m^2    d_ref {src.d_ref} m    rho {src.rho:.6f}")
    print(f"  x_cp     {src.x_cp_body:+.6f} m (CG 기준, 음수=안정)")
    print(f"  C_A      {src.coeffs(83.3, 0.0)['C_A']:.6f} (83.3 m/s, alpha=0)")
    print(f"  C_N      {src.coeffs(83.3, math.radians(10))['C_N']:+.6f} (83.3 m/s, alpha=10deg)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
