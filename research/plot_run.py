"""시뮬 결과 CSV(웹 UI 'CSV 저장' 또는 export_csv.cjs)를 matplotlib PNG로 그린다.

사용법:
    python3 -m research.plot_run <csv> [--out out.png] [--title "제목"]

라벨은 전부 영어로 쓴다 -- matplotlib 기본 폰트가 한글을 지원하지 않는다
(fast_drone 프로젝트 CLAUDE.md 규칙).
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def tilt_from_vertical_deg(qx, qy, qz, qw):
    """Angle between body +x (thrust axis) and world +z, degrees.
    0 = hover (nose straight up), 90 = nose horizontal, >90 = nose points down --
    this crosses exactly the pitch-overshoot boundary found in the 2026-09-22
    step-vs-ramp investigation (HANDOFF.md)."""
    nose_z = 2*(qx*qz - qy*qw)
    return np.degrees(np.arccos(np.clip(nose_z, -1, 1)))


def plot_run(csv_path, out_path=None, title=None):
    df = pd.read_csv(csv_path)
    t = df["t_s"]
    tilt = tilt_from_vertical_deg(df["qx"], df["qy"], df["qz"], df["qw"])
    omega_norm = np.hypot(np.hypot(df["wx_rads"], df["wy_rads"]), df["wz_rads"])

    fig, axes = plt.subplots(4, 2, figsize=(12, 14), sharex=True)
    fig.suptitle(title or Path(csv_path).stem, fontsize=13)

    ax = axes[0, 0]
    ax.plot(t, df["vx_mps"], label="vx")
    ax.plot(t, df["vy_mps"], label="vy")
    ax.plot(t, df["vz_mps"], label="vz")
    ax.plot(t, df["ref_vx_mps"], "k--", lw=1, label="ref vx")
    ax.set_ylabel("velocity [m/s]")
    ax.set_title("Velocity tracking")
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    ax.plot(t, df["pos_z_m"], label="z")
    ax.plot(t, df["ref_z_m"], "k--", lw=1, label="ref z")
    ax.set_ylabel("altitude [m]")
    ax.set_title("Altitude tracking")
    ax.legend(fontsize=8)

    ax = axes[1, 0]
    ax.plot(t, tilt)
    ax.axhline(90, color="r", lw=.8, ls=":", label="nose horizontal (90 deg)")
    ax.set_ylabel("tilt from vertical [deg]")
    ax.set_title("Attitude: nose tilt from hover-up")
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    ax.plot(t, df["wx_rads"], label="wx")
    ax.plot(t, df["wy_rads"], label="wy")
    ax.plot(t, df["wz_rads"], label="wz")
    ax.plot(t, omega_norm, "k", lw=1.2, label="|omega|")
    ax.set_ylabel("body rate [rad/s]")
    ax.set_title("Angular rate")
    ax.legend(fontsize=8)

    ax = axes[2, 0]
    for i in range(1, 5):
        ax.plot(t, df[f"n{i}_rads"], label=f"n{i}")
    ax.set_ylabel("rotor speed [rad/s]")
    ax.set_title("Rotor speed")
    ax.legend(fontsize=8, ncol=2)

    ax = axes[2, 1]
    for i in range(1, 5):
        ax.plot(t, df[f"thrust{i}_N"], label=f"T{i}")
    ax.set_ylabel("thrust [N]")
    ax.set_title("Per-rotor thrust")
    ax.legend(fontsize=8, ncol=2)

    ax = axes[3, 0]
    ax2 = ax.twinx()
    ax.plot(t, df["voltage_V"], "tab:blue", label="voltage")
    ax2.plot(t, df["bus_current_A"], "tab:orange", label="current")
    ax.set_ylabel("bus voltage [V]", color="tab:blue")
    ax2.set_ylabel("bus current [A]", color="tab:orange")
    ax.set_xlabel("t [s]")
    ax.set_title("Electrical")

    ax = axes[3, 1]
    ax.plot(t, df["soc"])
    ax.set_ylabel("SOC [-]")
    ax.set_xlabel("t [s]")
    ax.set_title("Battery state of charge")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out_path = out_path or str(Path(csv_path).with_suffix(".png"))
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out", default=None)
    ap.add_argument("--title", default=None)
    args = ap.parse_args()
    written = plot_run(args.csv, args.out, args.title)
    print(f"wrote {written}")
