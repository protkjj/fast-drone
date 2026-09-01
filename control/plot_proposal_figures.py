"""연구제안 3분 발표용 그림 3장.

시뮬 조건 (연구계획서 예비단계와 동일):
  · **참값 제어** — 센서모델·상태추정기 없음. plant 가 참 상태를 제어기에 직접 준다
  · **IPOPT** — VirtualNMPC 는 ca.nlpsol(..., 'ipopt', ...). acados 미사용
    (그림 B 의 acados 수치는 기록된 벤치마크이지 이 시뮬의 솔버가 아니다)

    python3 -m control.plot_proposal_figures

  A. results/prop_fig_gust.png    슬라이드 1 — 강외란에서 스케줄 선형이 무너진다
  B. results/prop_fig_solver.png  슬라이드 2 — 왜 지금 비선형이 가능한가
  C. results/prop_fig_mc.png      슬라이드 4 — 통계적 우위

A 는 **이 자리에서 다시 시뮬레이션**한다 (70 m/s 순항 + 1-cosine 돌풍 10 m/s).
B, C 는 이미 실측해 둔 값을 그린다 (results/bench_*.txt, results/MISSION_ANALYSIS.md).
어느 쪽인지 각 그림에 표시해 둔다 — 재현 가능성이 제안서의 주장 중 하나라서다.
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from control.vehicle_params import vehicle_params as P
from control.dynamics import AxialDronePlant
from control.trim import find_trim
from control.controller import ScheduledLQR
from control.hybrid_comparison import VirtualNMPC, ProperHybrid
from control.gust_comparison import make_gust_fn

V_CRUISE, Z_REF = 70.0, 50.0
T_GUST, D_GUST, W_GUST = 2.0, 1.0, 10.0
T_SIM = 8.0


# ══════════════════════════════════════════════════════════════════════
GUSTS = [5.0, 10.0, 15.0]          # 연구계획서 §2 과제 2 조건


def fig_gust():
    """A — 70 m/s 순항 중 수직돌풍. 스케줄 선형 vs 인터페이스 분리.

    ⚠ MISSION_ANALYSIS.md 의 "LQR 최대 dv_x = 15.226" 을 그대로 쓰면 안 된다.
      그 값은 65 초 통합 미션의 t=35s 시점 값인데, LQR 은 미션 전체 RMSE v_x 가
      16.8 m/s 다 — 즉 **가속 구간에서 이미 추종에 실패한 상태**였고, 15.2 는
      돌풍 응답이 아니라 그 누적 오차다. (Hybrid 는 0.059 로 여기서도 같다)

      돌풍 배제 능력만 재려면 **순항 트림에서 출발해 돌풍만 인가**해야 한다.
      그게 이 함수가 하는 것이다.
    """
    plant = AxialDronePlant(P, dt=0.001)
    trim = find_trim(P, V_CRUISE)
    x0 = trim["state"].copy(); x0[2] = Z_REF

    def build():
        lqr = ScheduledLQR(P, v_ref=[V_CRUISE, 0, 0], z_ref=Z_REF)
        vn = VirtualNMPC(P, v_ref=[V_CRUISE, 0, 0], z_ref=Z_REF,
                         N=20, dt_nmpc=0.05, dt_ctrl=0.02)
        return {"Gain-scheduled LQR": lqr,
                "NMPC+INDI (interface split)": ProperHybrid(vn, P, dt=plant.dt)}

    peaks = {k: [] for k in build()}
    series = {}
    print("  [A] 돌풍 세기 스윕 (순항 트림 출발, 돌풍만 인가)")
    for W in GUSTS:
        gust = make_gust_fn("vertical", W, T_GUST, D_GUST)
        for nm, c in build().items():
            ts, xs, _ = plant.simulate(x0, c, T=T_SIM, wind_fn=gust)
            dvx = xs[:, 3] - V_CRUISE
            peaks[nm].append(float(np.max(np.abs(dvx))))
            if W == GUSTS[-1]:
                series[nm] = (ts, dvx, xs[:, 2] - Z_REF)
            print(f"      W={W:4.1f}  {nm:<30} |dv_x| = {peaks[nm][-1]:.3f} m/s",
                  flush=True)

    colors = {"Gain-scheduled LQR": "C3", "NMPC+INDI (interface split)": "C0"}
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))

    for nm, (ts, dvx, dz) in series.items():
        ax[0].plot(ts, dvx, color=colors[nm], lw=2, label=nm)
    ax[0].axvspan(T_GUST, T_GUST + D_GUST, color="k", alpha=.10)
    ax[0].text(T_GUST + D_GUST / 2, ax[0].get_ylim()[1], "gust", ha="center",
               va="top", fontsize=8)
    ax[0].axhline(0, color="k", lw=.8)
    ax[0].set_xlabel("t [s]"); ax[0].set_ylabel(r"$\Delta v_x$ [m/s]")
    ax[0].set_title(f"Response to a {GUSTS[-1]:.0f} m/s vertical gust")
    ax[0].legend(fontsize=9); ax[0].grid(alpha=.3)

    for nm, pk in peaks.items():
        ax[1].plot(GUSTS, pk, "o-", color=colors[nm], lw=2, ms=7, label=nm)
    for i, W in enumerate(GUSTS):
        r = peaks["Gain-scheduled LQR"][i] / max(
            peaks["NMPC+INDI (interface split)"][i], 1e-9)
        ax[1].annotate(f"{r:.0f}$\\times$", (W, peaks["Gain-scheduled LQR"][i]),
                       textcoords="offset points", xytext=(6, 6), fontsize=9)
    ax[1].set_xlabel("gust strength [m/s]")
    ax[1].set_ylabel(r"peak $|\Delta v_x|$ [m/s]")
    ax[1].set_title("Gap widens with gust strength")
    ax[1].set_xticks(GUSTS); ax[1].legend(fontsize=9); ax[1].grid(alpha=.3)

    fig.suptitle(f"Pure gust rejection at {V_CRUISE:.0f} m/s cruise — "
                 "ground-truth state, IPOPT NMPC, started from trim", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig("results/prop_fig_gust.png", dpi=130); plt.close(fig)
    return peaks


# ══════════════════════════════════════════════════════════════════════
def fig_solver():
    """B — 솔버 속도. results/bench_compute.txt · bench_acados.txt 실측값."""
    # (이름, 중앙[ms], p95[ms])
    bars = [("IPOPT\n(VirtualNMPC, warm)", 21.4, 28.1),
            ("acados\nfull solve", 0.27, 0.51),
            ("acados RTI\nfeedback only", 0.09, 0.38)]
    fig, ax = plt.subplots(figsize=(7, 4.2))
    x = np.arange(len(bars))
    med = [b[1] for b in bars]; p95 = [b[2] for b in bars]
    ax.bar(x, med, 0.55, color=["C3", "C0", "C2"], label="median")
    ax.errorbar(x, med, yerr=[np.zeros(len(bars)), np.array(p95) - np.array(med)],
                fmt="none", ecolor="k", capsize=6, lw=1.4, label="up to p95")
    ax.axhline(20.0, color="k", ls="--", lw=1.5)
    ax.text(len(bars) - 0.45, 22, "50 Hz budget = 20 ms", ha="right", fontsize=9)
    ax.set_yscale("log"); ax.set_ylim(0.05, 60)
    ax.set_xticks(x); ax.set_xticklabels([b[0] for b in bars], fontsize=9)
    ax.set_ylabel("solve time [ms]  (log)")
    ax.set_title("Why nonlinear is feasible now\n"
                 "this study runs IPOPT (on the budget); acados is the path to hardware")
    for xi, (m, p) in enumerate(zip(med, p95)):
        ax.text(xi, p * 1.25, f"{m:.2f} / {p:.2f}", ha="center", fontsize=8.5)
    ax.legend(loc="lower left", fontsize=9); ax.grid(alpha=.3, axis="y", which="both")
    fig.tight_layout(); fig.savefig("results/prop_fig_solver.png", dpi=130)
    plt.close(fig)
    print("  [B] 솔버 속도 (기록된 실측값)")


# ══════════════════════════════════════════════════════════════════════
def fig_mc():
    """C — 몬테카를로 10회. results/MISSION_ANALYSIS.md 기록값."""
    # (이름, 평균, 표준편차, 최대)
    data = [("Gain-scheduled LQR", 4.434, 0.016, 4.462),
            ("NMPC+INDI (split)", 1.929, 0.386, 2.822)]
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    x = np.arange(len(data))
    ax.bar(x, [d[1] for d in data], 0.5, yerr=[d[2] for d in data],
           capsize=8, color=["C3", "C0"], label="mean $\\pm$ 1 s.d.")
    for xi, d in enumerate(data):
        ax.plot(xi, d[3], "kv", ms=9)
        ax.text(xi + 0.16, d[3], f"worst {d[3]:.2f}", va="center", fontsize=9)
    lo_worst, hi_best = data[1][3], data[0][1] - data[0][2]
    ax.axhspan(lo_worst, hi_best, color="C2", alpha=.12)
    ax.text(0.5, (lo_worst + hi_best) / 2,
            f"no overlap\n(split worst {lo_worst:.2f} < LQR best {hi_best:.2f})",
            ha="center", va="center", fontsize=9, color="C2")
    ax.set_xticks(x); ax.set_xticklabels([d[0] for d in data])
    ax.set_ylabel("mission RMSE $z$ [m]")
    ax.set_title("Monte Carlo, 10 runs (ground-truth state)\n"
                 "randomised initial state + gust 5-15 m/s + gust timing")
    ax.grid(alpha=.3, axis="y"); ax.legend(fontsize=9)
    fig.tight_layout(); fig.savefig("results/prop_fig_mc.png", dpi=130)
    plt.close(fig)
    print("  [C] 몬테카를로 (기록된 실측값)")


def main():
    print("연구제안 발표용 그림 생성")
    fig_gust(); fig_solver(); fig_mc()
    for f in ("gust", "solver", "mc"):
        print(f"  저장: results/prop_fig_{f}.png")


if __name__ == "__main__":
    main()
