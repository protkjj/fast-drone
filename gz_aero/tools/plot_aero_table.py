#!/usr/bin/env python3
"""공력표의 특성 곡선을 그린다. 두 소스를 겹쳐 본다.

    python3 gz_aero/tools/plot_aero_table.py [-o 출력.png]

표는 숫자 3800 줄이라 눈으로 못 읽는다. 곡선으로 보면 공력 담당자가
"이 형상이 이런 값이 나올 리 없다" 를 즉시 판단할 수 있다. CSV 를 바꾸면
다시 돌려서 확인하는 것이 이 도구의 쓰임이다.
"""
import argparse
import math
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402

SOURCES = [("placeholder", "tab:blue", "-"), ("sized", "tab:red", "--")]


def load(path):
    meta, cols, rows = {}, None, []
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        if line.startswith("#"):
            if ":" in line:
                k, v = line[1:].split(":", 1)
                meta[k.strip()] = v.strip()
            continue
        p = [x.strip() for x in line.split(",")]
        if cols is None:
            cols = p
            continue
        if p and p[0] != "":
            rows.append([float(x) for x in p])
    ix = {c: i for i, c in enumerate(cols)}
    return meta, ix, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default="results/gz_aero_table.png")
    ap.add_argument("--speed", type=float, default=83.3,
                    help="곡선을 뽑을 속도 [m/s]")
    args = ap.parse_args()

    here = pathlib.Path(__file__).resolve().parents[1]
    data = {}
    for name, _, _ in SOURCES:
        p = here / "data" / f"aero_{name}.csv"
        if p.is_file():
            data[name] = load(p)

    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    ax = ax.ravel()
    fig.suptitle(f"Aero table characteristics at V = {args.speed:g} m/s "
                 "(solid = placeholder, dashed = sized)", fontsize=12)

    for name, color, ls in SOURCES:
        if name not in data:
            continue
        meta, ix, rows = data[name]
        S = float(meta["S_ref"])
        d = float(meta["d_ref"])
        Vs = sorted({r[ix["V_mps"]] for r in rows})
        V = min(Vs, key=lambda v: abs(v - args.speed))
        sel = sorted((r for r in rows if r[ix["V_mps"]] == V),
                     key=lambda r: r[ix["alpha_rad"]])
        a = [math.degrees(r[ix["alpha_rad"]]) for r in sel]
        lab = f"{name}  (d={d:g} m, S={S:.5f})"

        ax[0].plot(a, [r[ix["C_A"]] for r in sel], color=color, ls=ls, label=lab)
        ax[1].plot(a, [r[ix["C_N"]] for r in sel], color=color, ls=ls, label=lab)
        ax[2].plot(a, [math.hypot(r[ix["C_A"]], r[ix["C_N"]]) for r in sel],
                   color=color, ls=ls, label=lab)
        ax[3].plot(a, [r[ix["x_cp"]] for r in sel], color=color, ls=ls, label=lab)
        ax[4].plot(a, [r[ix["C_mq"]] for r in sel], color=color, ls=ls,
                   label=f"{name} C_mq")
        ax[4].plot(a, [r[ix["C_lp"]] for r in sel], color=color, ls=":",
                   alpha=0.7, label=f"{name} C_lp")
        # 속도 의존성은 alpha=0 축력으로 본다
        z = [min((r for r in rows if r[ix["V_mps"]] == v),
                 key=lambda r: abs(r[ix["alpha_rad"]])) for v in Vs]
        ax[5].plot(Vs, [r[ix["C_A"]] for r in z], color=color, ls=ls,
                   marker="o", ms=3, label=lab)

    titles = [
        ("C_A  axial force coefficient", "alpha [deg]", "C_A"),
        ("C_N  normal force coefficient", "alpha [deg]", "C_N"),
        ("|C| = sqrt(C_A^2 + C_N^2)   resultant", "alpha [deg]", "|C|"),
        ("x_cp  centre of pressure from CG\n(negative = aft = statically stable)",
         "alpha [deg]", "x_cp [m]"),
        ("damping derivatives", "alpha [deg]", "C_mq (solid), C_lp (dotted)"),
        ("speed dependence at alpha = 0", "V [m/s]", "C_A"),
    ]
    for a_, (t, xl, yl) in zip(ax, titles):
        a_.set_title(t, fontsize=10)
        a_.set_xlabel(xl)
        a_.set_ylabel(yl)
        a_.grid(alpha=0.3)
        a_.legend(fontsize=7)
    ax[3].axhline(0, color="0.5", lw=0.8)

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=140)
    print(f"저장 {out}")
    for name in data:
        meta, ix, rows = data[name]
        Vs = sorted({r[ix["V_mps"]] for r in rows})
        V = min(Vs, key=lambda v: abs(v - args.speed))
        sel = [r for r in rows if r[ix["V_mps"]] == V]
        peak = max(sel, key=lambda r: math.hypot(r[ix["C_A"]], r[ix["C_N"]]))
        print(f"  {name:12s} |C| 최대 {math.hypot(peak[ix['C_A']], peak[ix['C_N']]):6.2f} "
              f"@ alpha {math.degrees(peak[ix['alpha_rad']]):5.1f} deg, "
              f"x_cp {peak[ix['x_cp']]:+.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
