#!/usr/bin/env python3
"""실기체 공력 로그를 그림으로. PPT 용.

    python3 gz_aero/tools/plot_real_vehicle.py [로그.csv] [-o 출력.png]

축 이름은 영문이다. 우분투에 한글 폰트가 없으면 네모로 깨지는데, 그러면
그림 자체를 못 쓴다. 공력 용어는 어차피 영문이 표준이라 손해가 없다.
"""
import argparse
import math
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from check_real_vehicle import load, load_table   # noqa: E402


def table_curve(tbl, V, n=181):
    """표에서 주어진 속도의 (alpha, C_A, C_N) 곡선을 뽑는다."""
    Vs = sorted({r[0] for r in tbl})
    Vn = min(Vs, key=lambda v: abs(v - V))
    sel = sorted((r for r in tbl if r[0] == Vn), key=lambda r: r[1])
    return Vn, [math.degrees(r[1]) for r in sel], [r[2] for r in sel], [r[3] for r in sel]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default="/tmp/fast_drone_aero/real_vehicle.csv")
    ap.add_argument("-o", "--out", default="results/gz_real_vehicle.png")
    args = ap.parse_args()

    if not pathlib.Path(args.log).is_file():
        print(f"로그가 없습니다: {args.log}")
        return 1
    meta, cols, rows, bad = load(args.log)
    if not rows:
        print("데이터 줄이 없습니다")
        return 1
    ix = {c: i for i, c in enumerate(cols)}
    S, rho = float(meta["S_ref"]), float(meta["rho"])
    W = float(meta["mass"]) * 9.80665

    t = [r[ix["t"]] for r in rows]
    alpha = [math.degrees(r[ix["alpha"]]) for r in rows]
    V = [r[ix["V"]] for r in rows]
    F = [math.dist((0, 0, 0), (r[ix["fWx"]], r[ix["fWy"]], r[ix["fWz"]])) for r in rows]
    M = [math.dist((0, 0, 0), (r[ix["mWx"]], r[ix["mWy"]], r[ix["mWz"]])) for r in rows]

    yaw, tilt = [], []
    for r in rows:
        qx, qy, qz, qw = (r[ix[c]] for c in ("qx", "qy", "qz", "qw"))
        yaw.append(math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                           1 - 2 * (qy * qy + qz * qz))))
        tilt.append(math.degrees(math.acos(max(-1.0, min(1.0,
                    1.0 - 2.0 * (qx * qx + qy * qy))))))

    # 이륙 시점 = 처음으로 눈에 띄게 기운 순간. 그 앞은 바닥이 힘을 받는 구간이라
    # 회색으로 덮어 "공력이 걸려도 안 움직이는 게 정상" 임을 보이게 한다.
    lift = next((tt for tt, z in zip(t, tilt) if z > 1.0), None)

    C_A = [r[ix["C_A"]] for r in rows]
    C_N = [r[ix["C_N"]] for r in rows]
    qb = [r[ix["q_bar"]] for r in rows]

    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.25, 1.0])
    ax = [fig.add_subplot(gs[i, 0]) for i in range(4)]
    for a_ in ax[:3]:
        a_.sharex(ax[3])
    bx = [fig.add_subplot(gs[0:2, 1]), fig.add_subplot(gs[2:4, 1])]
    wind = meta.get("wind", "?")
    fig.suptitle(f"Aero plugin on PX4 SITL   wind = ({wind}) m/s ENU,  W = {W:.1f} N"
                 f"   |   V max {max(V):.1f} m/s = {max(V) * 3.6:.0f} km/h",
                 fontsize=12)

    def ground(a):
        if lift:
            a.axvspan(t[0], lift, color="0.9", zorder=0)

    ground(ax[0])
    ax[0].plot(t, alpha, lw=1.4, label="angle of attack")
    ax[0].plot(t, yaw, lw=1.4, label="yaw")
    ax[0].axhline(0, color="0.6", lw=0.6)
    ax[0].set_ylabel("deg")
    ax[0].legend(loc="upper right", fontsize=9)
    ax[0].set_title("Weathervane: nose turns into the relative wind", fontsize=10)

    ground(ax[1])
    ax[1].plot(t, tilt, lw=1.4, color="tab:red", label="measured tilt")
    ax[1].plot(t, [math.degrees(math.atan2(f, W)) for f in F], lw=1.0,
               ls="--", color="0.4", label="tilt needed to hold position")
    ax[1].set_ylabel("deg")
    ax[1].legend(loc="upper right", fontsize=9)
    ax[1].set_title("Tilt: PX4 does not know about wind, so this means it was pushed",
                    fontsize=10)

    ground(ax[2])
    ax[2].plot(t, F, lw=1.4, color="tab:blue", label="|F| aero")
    ax[2].set_ylabel("N")
    ax[2].legend(loc="upper right", fontsize=9)
    a2 = ax[2].twinx()
    a2.plot(t, M, lw=1.0, color="tab:green", alpha=0.7)
    a2.set_ylabel("|M|  [N m]", color="tab:green")
    a2.tick_params(axis="y", colors="tab:green")
    ax[2].set_title("Aerodynamic force and moment actually applied", fontsize=10)

    ground(ax[3])
    ax[3].plot(t, V, lw=1.4, color="tab:purple")
    ax[3].set_ylabel("V air-rel  [m/s]")
    ax[3].set_xlabel("t  [s]")
    ax[3].set_title("Airspeed relative to the injected wind", fontsize=10)

    if lift:
        ax[0].text(t[0] + 0.3, ax[0].get_ylim()[1] * 0.85, "on ground",
                   fontsize=9, color="0.35")

    # ── 오른쪽: 공력계수 시각화 ────────────────────────────────────────
    tbl_path = meta.get("table", "")
    tbl = load_table(tbl_path) if tbl_path and pathlib.Path(tbl_path).is_file() else None

    # (1) 속도-공력.  V^2 로 자라는지가 한눈에 보여야 한다.
    sc = bx[0].scatter(V, F, c=alpha, s=8, cmap="viridis", vmin=0, vmax=180)
    plt.colorbar(sc, ax=bx[0], label="alpha [deg]")
    if tbl:
        Vgrid = [i * max(max(V), 1.0) / 60.0 for i in range(61)]
        for adeg, ls, lab in ((0.0, "--", "table, alpha=0"),
                              (90.0, ":", "table, alpha=90")):
            cs = []
            for v in Vgrid:
                Vn, aa, ca, cn = table_curve(tbl, v)
                j = min(range(len(aa)), key=lambda k: abs(aa[k] - adeg))
                cs.append(0.5 * rho * v * v * S * math.hypot(ca[j], cn[j]))
            bx[0].plot(Vgrid, cs, ls, color="0.35", lw=1.2, label=lab)
        # alpha=90 이론곡선은 측정점 밑에 정확히 겹쳐 안 보이는 것이 정상이다.
        # 플러그인이 같은 표에서 계수를 뽑기 때문이다.
        bx[0].legend(fontsize=8, loc="center left")
    bx[0].axhline(W, color="tab:red", lw=0.9, alpha=0.6)
    bx[0].text(0.98, W, "weight ", color="tab:red", fontsize=8, ha="right",
               va="bottom", transform=bx[0].get_yaxis_transform())
    bx[0].set_xlabel("V air-relative  [m/s]")
    bx[0].set_ylabel("|F| aero  [N]")
    bx[0].set_title("Aero force vs speed: does it grow as V^2", fontsize=10)
    bx[0].grid(alpha=0.3)

    # (2) 표 위에 비행이 실제로 지나간 자리를 덮어 그린다.
    #     표 전체가 아니라 **쓰인 구간**이 어디인지가 중요하다.
    if tbl:
        Vn, aa, ca, cn = table_curve(tbl, max(V))
        bx[1].plot(aa, ca, "-", color="tab:blue", lw=1.2, label=f"table C_A @ V={Vn:g}")
        bx[1].plot(aa, cn, "-", color="tab:orange", lw=1.2, label=f"table C_N @ V={Vn:g}")
    bx[1].scatter(alpha, C_A, s=10, color="tab:blue", alpha=0.5, label="flight C_A")
    bx[1].scatter(alpha, C_N, s=10, color="tab:orange", alpha=0.5, label="flight C_N")
    bx[1].set_xlabel("alpha [deg]")
    bx[1].set_ylabel("coefficient")
    bx[1].set_title("Which part of the table the flight actually used", fontsize=10)
    bx[1].legend(fontsize=8)
    bx[1].grid(alpha=0.3)

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out, dpi=150)
    print(f"저장 {out}")
    print(f"  이륙 {lift:.1f} s" if lift else "  이륙 안 함")
    print(f"  공력 최대 {max(F):.1f} N ({100*max(F)/W:.0f}% of W), "
          f"모멘트 최대 {max(M):.2f} N·m")
    print(f"  받음각 {min(alpha):.0f} ~ {max(alpha):.0f} deg, "
          f"요 {min(yaw):.0f} ~ {max(yaw):.0f} deg")
    print(f"  속도 최대 {max(V):.1f} m/s = {max(V) * 3.6:.0f} km/h")
    print(f"  C_A {min(C_A):.3f} ~ {max(C_A):.3f}, C_N {min(C_N):.3f} ~ {max(C_N):.3f}")
    print(f"  동압 최대 {max(qb):.0f} Pa")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
