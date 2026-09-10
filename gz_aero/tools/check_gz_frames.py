#!/usr/bin/env python3
"""겹2 대조 — Gazebo 가 남긴 디버그 로그를 파이썬 기준과 맞춰 본다.

    python3 gz_aero/tools/check_gz_frames.py --source sized

겹1 은 gz 없이 "수식을 옳게 옮겼나" 를 확인했다. 여기서 남은 것을 잡는다:
**좌표 변환**. 월드(ENU) -> 링크(FLU) -> 동체(FRD) -> 코어 -> 링크 -> 월드.

단계별로 쪼개 잰다 — 한 덩어리로 비교하면 실패했을 때 어디가 틀렸는지 모른다:

  A. 입력 변환   로그의 자세·월드속도로 v_B 를 다시 만들어 로그의 v_B 와 대조
  B. 각도/동압   로그의 v_B 로 V, alpha, q_bar 를 다시 계산해 대조
  C. 표 조회     로그의 (V, alpha) 에서 표를 파이썬으로 보간해 로그의 계수와 대조
  D. 힘 조립     로그의 계수로 F_B, M_B 를 만들어 로그의 것과 대조
  E. 출력 변환   로그의 F_B 를 월드로 돌려 실제로 건 f_world 와 대조
  F. 물리 대조   로그의 v_B 를 control/dynamics.py 에 넣어 대조 (보간오차만큼 차이)

A~E 는 기계정밀도로 맞아야 한다. F 만 표 보간오차(겹1 이 잰 값)를 허용한다.

⚠ 쿼터니언은 **scalar-last (qx,qy,qz,qw)** 다. 이 프로젝트에서 이미 한 번
  사고가 났던 자리라 로그 머리에도 규약을 적어 둔다.
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

EPS = 1e-8      # aero_core.hpp kEps 와 같은 값


# ── 표 읽기 + 이중선형 보간 (플러그인과 같은 규칙) ──────────────────────
class Table:
    def __init__(self, path):
        vs, als, rows, meta = [], [], [], {}
        cols = None
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip().lstrip("﻿")
                if not line:
                    continue
                if line.startswith("#"):
                    if ":" in line:
                        k, v = line[1:].split(":", 1)
                        meta[k.strip()] = v.strip()
                    continue
                cells = [c.strip() for c in line.split(",")]
                if cols is None:
                    cols = {n: i for i, n in enumerate(cells)}
                    continue
                rows.append([float(cells[cols[n]]) for n in
                             ("V_mps", "alpha_rad", "C_A", "C_N", "x_cp", "C_lp", "C_mq")])
        arr = np.array(rows)
        self.v_grid = np.unique(arr[:, 0])
        self.a_grid = np.unique(arr[:, 1])
        n_a = len(self.a_grid)
        self.data = arr[:, 2:].reshape(len(self.v_grid), n_a, 5)
        self.meta = meta

    @staticmethod
    def _locate(g, x):
        """플러그인 Locate() 와 같은 규칙: 격자 밖은 가장자리로 고정."""
        n = len(g)
        if n == 1:
            return 0, 0, 0.0
        hi = int(np.searchsorted(g, x, side="right"))
        if hi == 0:
            return 0, 1, 0.0
        if hi >= n:
            return n - 2, n - 1, 1.0
        i0, i1 = hi - 1, hi
        d = g[i1] - g[i0]
        return i0, i1, ((x - g[i0]) / d if d > 0 else 0.0)

    def lookup(self, V, alpha):
        iv0, iv1, tv = self._locate(self.v_grid, V)
        ia0, ia1, ta = self._locate(self.a_grid, alpha)
        d = self.data
        return ((1 - tv) * (1 - ta) * d[iv0, ia0] + (1 - tv) * ta * d[iv0, ia1]
                + tv * (1 - ta) * d[iv1, ia0] + tv * ta * d[iv1, ia1])


def quat_to_R(qx, qy, qz, qw):
    """scalar-last 쿼터니언 -> 회전행렬 (링크 -> 월드)."""
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]])


def read_log(path):
    meta, cols, rows = {}, None, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip().lstrip("﻿")
            if not line:
                continue
            if line.startswith("#"):
                if ":" in line:
                    k, v = line[1:].split(":", 1)
                    meta[k.strip()] = v.strip()
                continue
            cells = line.split(",")
            if cols is None:
                cols = {n.strip(): i for i, n in enumerate(cells)}
                continue
            rows.append([float(c) for c in cells])
    return meta, cols, np.array(rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["placeholder", "sized"], default="sized")
    ap.add_argument("--log-dir", default="/tmp/fast_drone_aero")
    ap.add_argument("--tol", type=float, default=1e-9,
                    help="A~E 단계 상대 허용오차 (기계정밀도여야 한다)")
    ap.add_argument("--phys-tol-N", type=float, default=0.05,
                    help="F 단계 힘 허용오차 [N] — 표 보간오차만큼은 허용한다")
    args = ap.parse_args(argv)

    logs = sorted(glob.glob(os.path.join(args.log_dir, "*.csv")))
    if not logs:
        print(f"로그가 없습니다: {args.log_dir}/*.csv")
        print("먼저 gen_gz_test_world.py 로 월드를 만들고 gz sim 을 돌리세요.")
        return 2

    from control.dynamics import _body_aerodynamics     # noqa: E402
    from aero_sources import get_source                 # noqa: E402
    import casadi as ca                                 # noqa: E402
    src = get_source(args.source)

    n_fail = 0
    print(f"{'probe':<14} {'rows':>6} | {'A 입력':>9} {'B 각도':>9} {'C 표':>9} "
          f"{'D 힘':>9} {'E 출력':>9} | {'F 물리[N]':>10}")
    print("-" * 88)

    for path in logs:
        meta, cols, R = read_log(path)
        if R.size == 0:
            print(f"{os.path.basename(path):<14} {'0':>6} | (빈 로그)")
            n_fail += 1
            continue
        table = Table(meta["table"])
        S = float(meta["S_ref"]); d = float(meta["d_ref"]); rho = float(meta["rho"])
        nose = np.array([float(x) for x in meta["nose"].split()])
        right = np.array([float(x) for x in meta["right"].split()])
        down = np.cross(nose, right)
        R_LB = np.column_stack([nose, right, down])     # 링크 <- 동체

        c = cols
        worst = dict(A=0.0, B=0.0, C=0.0, D=0.0, E=0.0, F=0.0)

        for row in R:
            q = quat_to_R(row[c["qx"]], row[c["qy"]], row[c["qz"]], row[c["qw"]])
            v_air_w = row[[c["vaWx"], c["vaWy"], c["vaWz"]]]
            om_w = row[[c["owWx"], c["owWy"], c["owWz"]]]
            vB_log = row[[c["u"], c["v"], c["w"]]]
            wB_log = row[[c["p"], c["q"], c["r"]]]
            FB_log = row[[c["Fx"], c["Fy"], c["Fz"]]]
            MB_log = row[[c["Mx"], c["My"], c["Mz"]]]
            fW_log = row[[c["fWx"], c["fWy"], c["fWz"]]]
            mW_log = row[[c["mWx"], c["mWy"], c["mWz"]]]

            def rel(a, b, scale):
                return float(np.max(np.abs(a - b)) / max(scale, 1e-12))

            # A. 월드 -> 링크 -> 동체
            vB = R_LB.T @ (q.T @ v_air_w)
            wB = R_LB.T @ (q.T @ om_w)
            worst["A"] = max(worst["A"],
                             rel(vB, vB_log, np.abs(vB_log).max()),
                             rel(wB, wB_log, max(np.abs(wB_log).max(), 1e-9)))

            # B. 속도 크기 / 전받음각 / 동압
            u_, v_, w_ = vB_log
            vcf = math.hypot(v_, w_)
            Vs = u_*u_ + v_*v_ + w_*w_ + EPS
            V = math.sqrt(Vs)
            alpha = math.atan2(vcf, u_)
            qbar = 0.5 * rho * Vs
            worst["B"] = max(worst["B"],
                             abs(V - row[c["V"]]) / max(V, 1e-9),
                             abs(alpha - row[c["alpha"]]) / max(alpha, 1e-9),
                             abs(qbar - row[c["q_bar"]]) / max(qbar, 1e-9))

            # C. 표 조회
            CA, CN, xcp, Clp, Cmq = table.lookup(row[c["V"]], row[c["alpha"]])
            worst["C"] = max(worst["C"],
                             abs(CA - row[c["C_A"]]) / max(abs(CA), 1e-9),
                             abs(CN - row[c["C_N"]]) / max(abs(CN), 1e-9),
                             abs(xcp - row[c["x_cp"]]) / max(abs(xcp), 1e-9))

            # D. 힘·모멘트 조립 (로그의 계수를 그대로 써서 조립만 검증)
            qb, CAl, CNl, xl = row[c["q_bar"]], row[c["C_A"]], row[c["C_N"]], row[c["x_cp"]]
            vcf_l = row[c["V_cf"]]
            inv = 1.0 / vcf_l if vcf_l > 0 else 0.0
            FN = qb * S * CNl
            F = np.array([-qb * S * CAl, -FN * v_ * inv, -FN * w_ * inv])
            df = 0.25 * rho * row[c["V"]] * S * d * d
            M = np.array([df * Clp * wB_log[0],
                          -xl * F[2] + df * Cmq * wB_log[1],
                          xl * F[1] + df * Cmq * wB_log[2]])
            sc = max(np.abs(FB_log).max(), 1e-9)
            worst["D"] = max(worst["D"], rel(F, FB_log, sc),
                             rel(M, MB_log, max(np.abs(MB_log).max(), 1e-9)))

            # E. 동체 -> 링크 -> 월드
            fW = q @ (R_LB @ FB_log)
            mW = q @ (R_LB @ MB_log)
            worst["E"] = max(worst["E"],
                             rel(fW, fW_log, max(np.abs(fW_log).max(), 1e-9)),
                             rel(mW, mW_log, max(np.abs(mW_log).max(), 1e-9)))

            # F. 물리 대조 — control/dynamics.py 원본에 넣어 본다.
            #    표 보간오차만큼은 차이가 나는 게 정상이다 (겹1 test B 가 잰 값).
            Fp, Mp = _body_aerodynamics(ca.DM(list(vB_log)), ca.DM(list(wB_log)),
                                        src.dyn_params(row[c["V"]]))
            Fp = np.array([float(x) for x in ca.DM(Fp).elements()])
            worst["F"] = max(worst["F"], float(np.max(np.abs(Fp - FB_log))))

        ok = all(worst[k] < args.tol for k in "ABCDE") and worst["F"] < args.phys_tol_N
        if not ok:
            n_fail += 1
        print(f"{os.path.basename(path):<14} {len(R):>6} | "
              + " ".join(f"{worst[k]:>9.2e}" for k in "ABCDE")
              + f" | {worst['F']:>10.3e}  {'OK' if ok else 'FAIL'}")

    print("-" * 88)
    print(f"A~E 허용 {args.tol:.0e} (기계정밀도)   F 허용 {args.phys_tol_N} N (표 보간오차)")
    if n_fail == 0:
        print("✅ 겹2 통과 — 좌표 변환이 월드↔링크↔동체 왕복에서 전부 맞습니다")
    else:
        print(f"❌ {n_fail}개 프로브 실패")
        print("   A 실패 -> 입력 변환(월드->동체).  E 실패 -> 출력 변환(동체->월드).")
        print("   C 실패 -> 표 보간기.  D 실패 -> 힘 조립.  F만 실패 -> 격자가 성긴 것.")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
