"""측풍 통합미션 + 모델오차 강건성 — 연구계획서 과제 2 조건.

    python3 -m control.robustness_crosswind

  E. results/prop_fig_mission_crosswind.png   65초 통합미션에 지속 측풍 추가
  F. results/prop_fig_robustness.png          공력계수 오차 +-10/20/30% 강건성

⚠ 강건성 시험의 핵심 — **플랜트만 틀고 제어기는 공칭 모델을 믿게 한다.**
   둘 다 틀면 "다른 기체를 정확히 아는 채로 나는 것"이라 강건성이 아니다.
     plant      = AxialDronePlant(P_perturbed)   실제 기체
     controller = ScheduledLQR(P_nominal, ...)   제어기가 믿는 모델
   초기 상태도 **공칭 트림**에서 시작한다 — 제어기가 그게 트림이라고 믿으니까.

시뮬 조건: 참값 제어(센서모델 없음), IPOPT NMPC.
"""
from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from control.vehicle_params import vehicle_params as P
from control.dynamics import AxialDronePlant
from control.trim import find_trim
from control.controller import ScheduledLQR
from control.hybrid_comparison import VirtualNMPC, ProperHybrid
from control.gust_comparison import make_gust_fn
from control.mission_sim import MissionProfile, MissionController, run_mission
from control.fallback_controller import HybridWithFallback
from control.flight_envelope import scaled_params


class PreemptiveFB(HybridWithFallback):
    """확정 아키텍처 — 예방형 전환. HANDOFF '최종 확정' 절 그대로.

    맨 ProperHybrid 는 **감속 구간에서 텀블한다** (65 초 미션에서 t=51.7 s,
    |omega| = 90.3 rad/s, |omega|>35 이 1.75 초 지속 -> 실기 판정 FAIL).
    RMSE 는 2.08/2.77 로 멀쩡해 보여서 RMSE 만 보면 못 잡는다.

    확정 구성은 감속 시작 시각(t=43 s)에 **계획적으로** ScheduledLQR 에 넘기고
    복귀하지 않는다. omega_limit=15 는 그 전에 터질 때를 위한 반응형 백스톱이다.
    (final_config_mission.py 의 LoggedFB 와 같은 구성이나, 그쪽은 acados 를
     끌고 오므로 IPOPT 연구용으로 여기 최소 구현을 둔다)
    """

    def __init__(self, *a, preempt_t=None, **k):
        super().__init__(*a, **k)
        self.preempt_t = preempt_t

    def __call__(self, t, x):
        if (self.preempt_t is not None and self._using_hybrid
                and t >= self.preempt_t):
            self._using_hybrid = False
        return super().__call__(t, x)

_RES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "results")
AERO_ERRORS = [-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30]
CROSSWINDS = [0.0, 10.0, 15.0]
COL = {"LQR": "C3", "Hybrid": "C7", "HybridFB": "C0"}


def _yaw_deg(xs):
    y = np.array([Rotation.from_quat(q).as_euler("zyx")[0] for q in xs[:, 6:10]])
    return np.degrees(np.unwrap(y) - np.unwrap(y)[0])


def _mk(name, params, v_ref, z_ref, dt):
    """제어기 생성. params 는 **제어기가 믿는 모델** — 공칭을 넣어야 한다."""
    if name == "LQR":
        return ScheduledLQR(params, v_ref=list(v_ref), z_ref=z_ref)
    vn = VirtualNMPC(params, v_ref=list(v_ref), z_ref=z_ref,
                     N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    hyb = ProperHybrid(vn, params, dt=dt)
    if name == "Hybrid":                      # 폴백 없는 맨 하이브리드 (진단용)
        return hyb
    # "HybridFB" — 확정 아키텍처
    return PreemptiveFB(hyb, ScheduledLQR(params, v_ref=list(v_ref), z_ref=z_ref),
                        z_ref=z_ref, z_err_limit=10.0, omega_limit=15.0,
                        cooldown_sec=1e6, dt=dt, preempt_t=43.0)


# ══════════════════════════════════════════════════════════════════════
# E — 65초 통합미션 + 지속 측풍
# ══════════════════════════════════════════════════════════════════════
# 측풍 진입 시각. **순항 시작(28s)** 이 기본이다.
#
# 처음엔 이륙 직후(5s)부터 걸었는데, 그러면 기수오차 판정이 무의미해진다:
# 풍향계 복원 강성이 q_bar ~ V^2 라 V~0 인 이륙·호버에서 0 이고, 제어기가
# 무엇이든 기체가 그냥 돌아간다 (LQR 도 측풍 5 m/s 에서 35.8 도).
# 그건 제어 문제가 아니라 물리라서, 제어기를 가르는 지표가 못 된다.
# 저속 측풍은 별도 케이스(T_ON_TAKEOFF)로 따로 본다.
T_ON_CRUISE = 28.0
T_ON_TAKEOFF = 5.0


def _wind_mission(W_cross, t_on=T_ON_CRUISE):
    """지속 측풍 + 기존 수직돌풍(t=35s, 10 m/s).

    실제 비행에서 바람은 계속 불고 돌풍이 그 위에 얹힌다. 기존 미션은 돌풍만
    있었으므로 '바람 없는 날의 돌풍' 이었다.

    t_on=28 이면 순항 진입과 함께 측풍이 들어오고, 7 초 뒤 수직돌풍이 겹친다
    — **조합 외란 시험은 그대로 유지된다.**
    """
    gust = make_gust_fn("vertical", 10.0, 35.0, 1.0)

    def f(t):
        w = np.array(gust(t), float)
        if t >= t_on:
            w = w + np.array([0.0, W_cross * min((t - t_on) / 1.0, 1.0), 0.0])
        return w
    return f


def mission_crosswind():
    plant = AxialDronePlant(P, dt=0.001)
    prof = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    x0 = AxialDronePlant.hover_state(P); x0[2] = 2.0

    rows, series = [], {}
    fails = []
    print("  [E] 65초 통합미션 + 지속 측풍  (수 분 걸립니다)")
    for W in CROSSWINDS:
        wf = _wind_mission(W)
        for nm in ("LQR", "Hybrid", "HybridFB"):
            c = MissionController(_mk(nm, P, [0, 0, 0], 2.0, plant.dt), prof)
            r = run_mission(plant, c, x0, prof, wind_fn=wf)
            ts, xs, vr, zr = r["ts"], r["xs"], r["v_refs"], r["z_refs"]
            yw = _yaw_deg(xs)
            rec = dict(W=W, ctrl=nm,
                       rmse_vx=float(np.sqrt(np.mean((xs[:, 3] - vr[:, 0]) ** 2))),
                       rmse_z=float(np.sqrt(np.mean((xs[:, 2] - zr) ** 2))),
                       yaw_max=float(np.max(np.abs(yw))),
                       dy_max=float(np.max(np.abs(xs[:, 1]))),
                       w_max=float(np.max(np.linalg.norm(xs[:, 10:13], axis=1))))
            rec["fail"] = rec["w_max"] > 35.0
            rows.append(rec)
            if rec["fail"]:
                fails.append((W, nm))
            if abs(W - CROSSWINDS[-1]) < 1e-9:
                series[nm] = (ts, xs, vr, zr, yw)
            print(f"      W={W:4.0f}  {nm:<7} RMSE vx {rec['rmse_vx']:6.2f}  "
                  f"z {rec['rmse_z']:5.2f}  요 {rec['yaw_max']:6.2f}deg  "
                  f"dy {rec['dy_max']:7.2f}m  |w|max {rec['w_max']:6.2f}"
                  f"  {'FAIL(텀블)' if rec['fail'] else 'PASS'}", flush=True)

    fig, ax = plt.subplots(2, 2, figsize=(13, 7.5))
    for nm, (ts, xs, vr, zr, yw) in series.items():
        ax[0, 0].plot(ts, xs[:, 3], color=COL[nm], lw=1.8, label=nm)
        ax[0, 1].plot(ts, xs[:, 2] - zr, color=COL[nm], lw=1.8, label=nm)
        ax[1, 0].plot(ts, yw, color=COL[nm], lw=1.8, label=nm)
        ax[1, 1].plot(ts, xs[:, 1], color=COL[nm], lw=1.8, label=nm)
    ax[0, 0].plot(series["LQR"][0], series["LQR"][2][:, 0], "k--", lw=1.2,
                  label="reference")
    for a in ax.ravel():
        a.axvspan(35, 36, color="k", alpha=.12)
        a.grid(alpha=.3); a.set_xlabel("t [s]"); a.legend(fontsize=8)
    ax[0, 0].set_ylabel("$v_x$ [m/s]"); ax[0, 0].set_title("Forward speed tracking")
    ax[0, 1].set_ylabel(r"$\Delta z$ [m]"); ax[0, 1].set_title("Altitude error")
    ax[1, 0].set_ylabel(r"$\Delta\psi$ [deg]"); ax[1, 0].set_title("Heading")
    ax[1, 1].set_ylabel("$y$ [m]"); ax[1, 1].set_title("Lateral position")
    fig.suptitle(f"65 s mission with sustained {CROSSWINDS[-1]:.0f} m/s crosswind "
                 "(+ vertical gust at t=35 s, shaded)  —  ground truth, IPOPT",
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(_RES, "prop_fig_mission_crosswind.png"), dpi=130)
    plt.close(fig)
    return rows


# ══════════════════════════════════════════════════════════════════════
# F — 공력계수 오차 강건성 (플랜트만 틀고 제어기는 공칭)
# ══════════════════════════════════════════════════════════════════════
def robustness(V=70.0, Z=50.0, T=8.0, W_gust=15.0):
    trim_nom = find_trim(P, V)                 # 제어기가 믿는 트림
    x0 = trim_nom["state"].copy(); x0[2] = Z
    gust = make_gust_fn("vertical", W_gust, 2.0, 1.0)

    out = {nm: {"pk": [], "rmse": [], "wmax": []} for nm in ("LQR", "Hybrid")}
    print(f"  [F] 공력계수 오차 강건성 (플랜트만 섭동, 제어기는 공칭)")
    for e in AERO_ERRORS:
        plant = AxialDronePlant(scaled_params(P, e), dt=0.001)   # 실제 기체
        for nm in ("LQR", "Hybrid"):
            c = _mk(nm, P, [V, 0, 0], Z, plant.dt)              # 공칭 모델
            ts, xs, _ = plant.simulate(x0, c, T=T, wind_fn=gust)
            d = xs[:, 3] - V
            out[nm]["pk"].append(float(np.max(np.abs(d))))
            out[nm]["rmse"].append(float(np.sqrt(np.mean(d ** 2))))
            out[nm]["wmax"].append(float(np.max(np.linalg.norm(xs[:, 10:13], axis=1))))
            print(f"      err {e:+5.0%}  {nm:<7} peak |dvx| {out[nm]['pk'][-1]:6.3f}  "
                  f"RMSE {out[nm]['rmse'][-1]:6.3f}  |w|max {out[nm]['wmax'][-1]:5.2f}",
                  flush=True)

    x = np.array(AERO_ERRORS) * 100
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    for nm in out:
        ax[0].plot(x, out[nm]["pk"], "o-", color=COL[nm], lw=2, label=nm)
        ax[1].plot(x, out[nm]["rmse"], "o-", color=COL[nm], lw=2, label=nm)
        ax[2].plot(x, out[nm]["wmax"], "o-", color=COL[nm], lw=2, label=nm)
    for a, lab, ttl in ((ax[0], r"peak $|\Delta v_x|$ [m/s]", "Peak excursion"),
                        (ax[1], r"RMSE $v_x$ [m/s]", "Tracking error"),
                        (ax[2], r"$|\omega|_{max}$ [rad/s]", "Rate (tumble check)")):
        a.axvline(0, color="k", lw=.8)
        a.set_xlabel("aero coefficient error [%]"); a.set_ylabel(lab)
        a.set_title(ttl); a.grid(alpha=.3); a.legend(fontsize=8)
        a.set_xticks(x)
    fig.suptitle(f"Robustness to aero model error — plant perturbed, controller keeps "
                 f"the nominal model\n{V:.0f} m/s cruise + {W_gust:.0f} m/s vertical gust, "
                 "ground truth, IPOPT", fontsize=11.5)
    fig.tight_layout(rect=[0, 0, 1, 0.90])
    fig.savefig(os.path.join(_RES, "prop_fig_robustness.png"), dpi=130)
    plt.close(fig)
    return out


def main():
    r = robustness()
    m = mission_crosswind()
    print("\n  저장: results/prop_fig_robustness.png")
    print("        results/prop_fig_mission_crosswind.png")
    return r, m


if __name__ == "__main__":
    main()


# ══════════════════════════════════════════════════════════════════════
# G — 최종 비교: 감속 제외 (이륙~순항), 허용 기수오차 20도
# ══════════════════════════════════════════════════════════════════════
YAW_LIMIT_DEG = 20.0        # kj 지정 허용 기수오차
OMEGA_FAIL = 35.0           # 자이로 포화 = 실기 실패
T_END = 43.0                # 감속 시작 직전까지 (이륙·안정화·가속·순항+돌풍)
CW_FINAL = [0.0, 5.0, 10.0, 15.0]


def final_comparison():
    """감속 구간을 뺀 최종 비교.

    감속(43~58s)에서 맨 ProperHybrid 가 텀블한다(|omega| 90). 그건 제어 구조가
    아니라 **감속 과도구간의 알려진 문제**이고 확정 아키텍처가 예방형 전환으로
    따로 다룬다. 여기서는 그 구간을 빼고 **인터페이스 분리 자체**를 본다.
    그래서 폴백 없이 LQR vs ProperHybrid 순수 비교가 된다.

    판정: 기수오차 20도 이내 **그리고** |omega| < 35 rad/s.
    """
    plant = AxialDronePlant(P, dt=0.001)
    prof = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    x0 = AxialDronePlant.hover_state(P); x0[2] = 2.0

    res, series = [], {}
    print(f"  [G] 최종 비교 — 감속 제외(t<{T_END:.0f}s), 기수오차 {YAW_LIMIT_DEG:.0f}deg 판정")
    for W in CW_FINAL:
        wf = _wind_mission(W)
        for nm in ("LQR", "Hybrid", "HybridFB"):
            c = MissionController(_mk(nm, P, [0, 0, 0], 2.0, plant.dt), prof)
            ts, xs, _ = plant.simulate(x0, c, T=T_END, wind_fn=wf)
            vr, zr = prof.compute_refs(ts)
            yw = _yaw_deg(xs)
            r = dict(W=W, ctrl=nm,
                     rmse_vx=float(np.sqrt(np.mean((xs[:, 3] - vr[:, 0]) ** 2))),
                     rmse_z=float(np.sqrt(np.mean((xs[:, 2] - zr) ** 2))),
                     yaw_max=float(np.max(np.abs(yw))),
                     dy_max=float(np.max(np.abs(xs[:, 1]))),
                     w_max=float(np.max(np.linalg.norm(xs[:, 10:13], axis=1))))
            r["pass"] = (r["yaw_max"] <= YAW_LIMIT_DEG) and (r["w_max"] < OMEGA_FAIL)
            res.append(r)
            if abs(W - CW_FINAL[-1]) < 1e-9:
                series[nm] = (ts, xs, vr, zr, yw)
            print(f"      W={W:4.0f}  {nm:<7} RMSEvx {r['rmse_vx']:6.2f}  z {r['rmse_z']:5.2f}"
                  f"  요 {r['yaw_max']:6.2f}deg  dy {r['dy_max']:6.2f}m"
                  f"  |w| {r['w_max']:5.2f}  {'PASS' if r['pass'] else 'FAIL'}", flush=True)

    fig, ax = plt.subplots(2, 2, figsize=(13, 8))
    for nm, (ts, xs, vr, zr, yw) in series.items():
        ax[0, 0].plot(ts, xs[:, 3], color=COL[nm], lw=1.8, label=nm)
        ax[0, 1].plot(ts, np.linalg.norm(xs[:, 10:13], axis=1),
                      color=COL[nm], lw=1.8, label=nm)
    ax[0, 0].plot(series["LQR"][0], series["LQR"][2][:, 0], "k--", lw=1.3,
                  label="reference")
    ax[0, 1].axhline(OMEGA_FAIL, color="C3", ls="--", lw=1.6)
    ax[0, 1].text(1, OMEGA_FAIL + 2, rf"tumble limit $|\omega|$ = {OMEGA_FAIL:.0f} rad/s",
                  color="C3", fontsize=9)
    ax[0, 1].set_yscale("symlog", linthresh=1)
    for a in (ax[0, 0], ax[0, 1]):
        a.axvspan(35, 36, color="k", alpha=.12)
        a.axvline(T_ON_CRUISE, color="C2", lw=1.0, alpha=.6)
        a.set_xlabel("t [s]"); a.grid(alpha=.3); a.legend(fontsize=8)
    ax[0, 0].set_ylabel("$v_x$ [m/s]")
    ax[0, 0].set_title(f"Speed tracking, {CW_FINAL[-1]:.0f} m/s crosswind")
    ax[0, 1].set_ylabel(r"$|\omega|$ [rad/s]")
    ax[0, 1].set_title("Body rate — tumble check (log)")

    for nm in ("LQR", "Hybrid", "HybridFB"):
        v = [r for r in res if r["ctrl"] == nm]
        ax[1, 0].plot(CW_FINAL, [r["rmse_vx"] for r in v], "o-",
                      color=COL[nm], lw=2, ms=7, label=nm)
        ax[1, 1].plot(CW_FINAL, [r["yaw_max"] for r in v], "o-",
                      color=COL[nm], lw=2, ms=7, label=nm)
    ax[1, 1].axhline(YAW_LIMIT_DEG, color="C2", ls="--", lw=1.6)
    ax[1, 1].text(0.2, YAW_LIMIT_DEG + .7, f"tolerance {YAW_LIMIT_DEG:.0f}$\\degree$",
                  color="C2", fontsize=9)
    for a, lab, ttl in ((ax[1, 0], r"RMSE $v_x$ [m/s]", "Speed tracking error"),
                        (ax[1, 1], r"peak $|\Delta\psi|$ [deg]", "Heading held")):
        a.set_xlabel("sustained crosswind [m/s]"); a.set_ylabel(lab)
        a.set_title(ttl); a.set_xticks(CW_FINAL); a.grid(alpha=.3); a.legend(fontsize=8)

    fig.suptitle("Take-off to cruise (deceleration excluded)  —  crosswind enters at "
                 f"cruise (t={T_ON_CRUISE:.0f} s), vertical gust at t=35 s\n"
                 "ground truth, IPOPT NMPC  —  "
                 f"pass = heading within {YAW_LIMIT_DEG:.0f}$\\degree$ and "
                 rf"$|\omega|$ < {OMEGA_FAIL:.0f} rad/s", fontsize=11.5)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(os.path.join(_RES, "prop_fig_final.png"), dpi=130)
    plt.close(fig)
    return res
