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

    fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    wind = meta.get("wind", "?")
    fig.suptitle(f"Aero plugin on PX4 SITL   wind = ({wind}) m/s ENU,  W = {W:.1f} N",
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
