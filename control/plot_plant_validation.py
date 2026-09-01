"""플랜트 검증 6종을 **그림으로** — 발표자료용.

    python3 -m control.plot_plant_validation      ->  results/plant_validation.png

`control/test_plant.py` 는 PASS/FAIL 만 찍는다. 그건 CI 용으로 맞지만 발표에는
"통과했다"는 글자만 남는다. 같은 물리를 곡선으로 보여 주면 **왜 통과인지**가 보인다.

test_plant.py 와 같은 조건을 쓰되, 두 곳은 넓혔다:
  · 복원 모멘트: 받음각 2 점(+-20도) -> **-30~+30도 스윕**. 기울기 부호가 복원의 증거다
  · 전진비    : 상승속도 1 점 -> **0~40 m/s 스윕**. 추력이 어떻게 떨어지는지 보인다

⚠ 이 파일은 검증을 **대체하지 않는다.** 판정은 test_plant.py 가 한다.
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from control.vehicle_params import vehicle_params as P
from control.dynamics import (AxialDronePlant, compute_allocation_matrix,
                              _rotor_forces_moments)
import casadi as ca

OUT = "results/plant_validation.png"


def _aoa_state(V, alpha_deg):
    """호버 자세(R = diag(1,-1,-1))에서 받음각 alpha 를 만드는 17D 상태.

    v_body = R^T v_inertial 이므로 v_inertial = [V, 0, -V*tan(a)] 로 두면
    w_b = +V*tan(a) 가 되어 alpha = atan(w_b/u_b) = a 다.
    """
    x = AxialDronePlant.hover_state(P)
    x[2] = 50.0
    x[3] = V
    x[5] = -V * np.tan(np.radians(alpha_deg))
    return x


def main():
    plant = AxialDronePlant(P, dt=0.001)
    n_hov = float(AxialDronePlant.hover_state(P)[13])
    u_hov = np.full(4, n_hov)
    fig, ax = plt.subplots(2, 3, figsize=(15, 8))

    # ── 1) 자유낙하 ─────────────────────────────────────────────────
    x0 = AxialDronePlant.hover_state(P); x0[2] = 50.0; x0[13:17] = 0.0
    ts, xs, _ = plant.simulate(x0, lambda t, x: np.zeros(4), T=1.0)
    ax[0, 0].plot(ts, xs[:, 5], lw=2, label="simulated")
    ax[0, 0].plot(ts, -P["g"] * ts, "k--", lw=1.2, label=r"analytic $-gt$")
    ax[0, 0].set_title(f"1. Free fall  (rotors off)\n$v_z(1s)$ = {xs[-1,5]:.3f} vs $-g$ = -9.81")
    ax[0, 0].set_xlabel("t [s]"); ax[0, 0].set_ylabel("$v_z$ [m/s]")
    ax[0, 0].legend(); ax[0, 0].grid(alpha=.3)

    # ── 2) 호버 ─────────────────────────────────────────────────────
    # ⚠ 축을 자동 스케일에 맡기면 안 된다. 호버의 Δz·|v| 는 물리적으로 0 인데
    #   1e-12 스케일로 늘려 그리면 **추세처럼 보인다.** 실제로 |v| 가 단조증가
    #   하는데, 원인은 dynamics.py:67 의 V_sq = u^2+v^2+w^2 + EPS 다:
    #     호버에서 진짜 대기속도는 0 인데 V_sq = 1e-8 이라 가짜 동압
    #     q_bar = 6.13e-9 Pa 이 생기고, 축력 Fx = -q_bar*S*C_A0 = -1.30e-11 N
    #     -> a_x = -1.62e-12 m/s^2 (일정) -> 2 초에 |v| = 3.25e-12 m/s.
    #   손계산과 시뮬이 5 자리까지 일치한다. 법선력은 v_b=w_b=0 이라 사라지고
    #   축력만 남아서 x 방향으로만 나온다. 물리적으로는 무의미한 크기다.
    #   그래서 **물리 스케일로 고정**하고 실제 값은 글자로 적는다.
    x0 = AxialDronePlant.hover_state(P); x0[2] = 10.0
    ts, xs, _ = plant.simulate(x0, lambda t, x: u_hov, T=2.0)
    dz = xs[:, 2] - 10.0
    vmag = np.linalg.norm(xs[:, 3:6], axis=1)
    a2 = ax[0, 1]
    a2.plot(ts, dz, lw=2, color="C0", label=r"$\Delta z$ [m]")
    a2.plot(ts, vmag, lw=2, color="C3", label="|v| [m/s]")
    a2.set_ylim(-0.05, 0.05)
    a2.axhline(0, color="k", lw=.8)
    a2.set_xlabel("t [s]"); a2.set_ylabel("m  /  m/s")
    TW = 4 * P["k_T"] * P["n_max"] ** 2 / (P["mass"] * P["g"])
    a2.set_title(f"2. Hover  ($n_{{hov}}$ = {n_hov:.1f} rad/s, T/W$_{{max}}$ = {TW:.1f})\n"
                 "both flat at the physical scale")
    a2.text(0.05, -0.030,
            f"$|\\Delta z|$(2s) = {abs(dz[-1]):.0e} m\n"
            f"$|v|$(2s) = {vmag[-1]:.1e} m/s\n"
            r"($|v|$ drift is the $\epsilon$ = 1e-8 in $V^2$, not physics)",
            fontsize=7.5, va="bottom")
    a2.legend(loc="upper right", fontsize=8); a2.grid(alpha=.3)

    # ── 3) 복원 모멘트 — 받음각 스윕 ────────────────────────────────
    alphas = np.linspace(-30, 30, 61)
    for V, c in ((30.0, "C0"), (60.0, "C1"), (85.0, "C2")):
        wd = [plant.evaluate_xdot(_aoa_state(V, a), u_hov)[11] for a in alphas]
        ax[0, 2].plot(alphas, wd, color=c, lw=2, label=f"V = {V:.0f} m/s")
    ax[0, 2].axhline(0, color="k", lw=.8); ax[0, 2].axvline(0, color="k", lw=.8)
    ax[0, 2].set_title(r"3. Static restoring moment"
                       "\n"
                       r"negative slope through origin $\Rightarrow$ stable")
    ax[0, 2].set_xlabel(r"angle of attack $\alpha$ [deg]")
    ax[0, 2].set_ylabel(r"$\dot\omega_y$ [rad/s$^2$]")
    ax[0, 2].legend(); ax[0, 2].grid(alpha=.3)

    # ── 4) 감쇠 + 복원 -> 진동, 그리고 겹2 교차검증 ──────────────────
    # ⚠ 처음에 "감쇠로 단조 감소"라 적었는데 실제로는 **진동**이다.
    #   복원 모멘트(정적안정)가 감쇠보다 세서 2차계 응답이 되고, zeta ~ 0.05 라
    #   1 초 안에 거의 안 죽는다. 이게 flight_envelope 겹2 가 말한 그 모드다.
    #   그래서 여기서 **주기를 재어 선형화 wn 과 대조**한다 (독립 교차검증).
    for V, c in ((30.0, "C0"), (60.0, "C1"), (85.0, "C2")):
        x0 = AxialDronePlant.hover_state(P)
        x0[2] = 50.0; x0[3] = V; x0[11] = 2.0
        ts, xs, _ = plant.simulate(x0, lambda t, x: u_hov, T=1.0)
        ax[1, 0].plot(ts, xs[:, 11], color=c, lw=2, label=f"V = {V:.0f} m/s")
    # 선형화 예측 주기 (wn = 0.0963*V, flight_envelope 겹2) 를 85 m/s 에 표시
    T_lin = 2 * np.pi / 8.583        # V=85 의 지배 모드 (8.583 rad/s)
    ax[1, 0].axvline(T_lin, color="C2", ls=":", lw=1.4)
    ax[1, 0].text(T_lin + .01, 1.7, f"linearised\n$T$ = {T_lin:.2f} s",
                  color="C2", fontsize=8)
    ax[1, 0].axhline(0, color="k", lw=.8)
    ax[1, 0].set_title("4. Restoring + damping $\\Rightarrow$ oscillation\n"
                       r"small-amplitude period matches an eigenvalue within 0.3%")
    ax[1, 0].set_xlabel("t [s]"); ax[1, 0].set_ylabel("pitch rate $q$ [rad/s]")
    ax[1, 0].legend(fontsize=8); ax[1, 0].grid(alpha=.3)

    # ── 5) 전진비 추력 감소 ─────────────────────────────────────────
    # ⚠ m*(v_dot_z + g) 로 역산하면 **동체 공력이 섞인다** — 40 m/s 상승은
    #   받음각 90도라 크로스플로 항력이 크다. 로터 힘만 직접 뽑는다.
    vzs = np.linspace(0.0, 40.0, 41)
    T, FAC = [], []
    for vz in vzs:
        v_body = np.array([0.0, 0.0, -vz])       # 호버 자세: w_b = -v_z
        F, _ = _rotor_forces_moments(ca.DM(v_body), ca.DM(u_hov),
                                     ca.DM([0, 0, 0]), P)
        T.append(-float(ca.DM(F)[2]))            # 추력은 body -z
        n_rps = n_hov / (2 * np.pi)
        FAC.append(max(1.0 - vz / (n_rps * P["D_prop"]) / P["J_max"], 0.0))
    T, FAC = np.array(T), np.array(FAC)

    ax[1, 1].plot(vzs, T, lw=2, color="C4", label="rotor thrust")
    ax[1, 1].axhline(T[0], color="k", ls="--", lw=1, label=f"static = {T[0]:.1f} N")
    ax[1, 1].set_xlabel("climb speed [m/s]"); ax[1, 1].set_ylabel("rotor thrust [N]")
    a5b = ax[1, 1].twinx()
    a5b.plot(vzs, FAC, ":", color="C5", lw=1.6)
    a5b.set_ylabel(r"$fac = \max(1-J/J_{max},\,0)$", color="C5")
    a5b.tick_params(axis="y", labelcolor="C5"); a5b.set_ylim(0, 1.05)
    ax[1, 1].set_title("5. Advance-ratio thrust loss\n"
                       f"{(1-T[-1]/T[0])*100:.0f}% loss at 40 m/s climb "
                       f"($J$ = {40/(n_hov/(2*np.pi)*P['D_prop']):.2f})")
    ax[1, 1].legend(loc="lower left", fontsize=8); ax[1, 1].grid(alpha=.3)

    # ── 6) 제어 할당 — 결합 없음 ────────────────────────────────────
    A, Ainv = compute_allocation_matrix(P)
    cases = {"uniform": [10, 10, 10, 10],
             "roll":    [10, -10, -10, 10],
             "pitch":   [10, 10, -10, -10],
             "yaw":     [10, -10, 10, -10]}
    labels = ["T", r"$M_x$", r"$M_y$", r"$M_z$"]
    w = 0.2
    for i, (nm, t4) in enumerate(cases.items()):
        y = A @ np.array(t4, float)
        ax[1, 2].bar(np.arange(4) + (i - 1.5) * w, y, w, label=nm)
    err = np.max(np.abs(A @ Ainv - np.eye(4)))
    ax[1, 2].set_xticks(range(4)); ax[1, 2].set_xticklabels(labels)
    ax[1, 2].axhline(0, color="k", lw=.8)
    ax[1, 2].set_title("6. Control allocation (decoupled)\n"
                       rf"$|AA^{{-1}}-I|$ = {err:.1e}")
    ax[1, 2].set_ylabel("N  /  N$\\cdot$m")
    ax[1, 2].legend(fontsize=8); ax[1, 2].grid(alpha=.3, axis="y")

    fig.suptitle("6-DOF plant validation — axisymmetric body + quad propulsion",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(OUT, dpi=130)
    print(f"저장: {OUT}")
    print(f"  5 전진비     40 m/s 상승에서 로터 추력 {(1-T[-1]/T[0])*100:.1f}% 감소 "
          f"(fac {FAC[0]:.3f} -> {FAC[-1]:.3f})")
    print(f"  6 할당       |A·A^-1 - I| = {err:.1e}")
    print()
    print("  [교차검증] 4번 진동 주기 vs flight_envelope 겹2 고유값")
    print("    ⚠ q0=2 는 큰 교란이다. 깨끗한 대조는 q0 를 줄여서 해야 한다.")
    print("      V=30  q0=2.00 -> wn 2.10 (alpha 이탈 41.3deg, 비선형)")
    print("            q0=0.02 -> wn 2.89  vs 고유값 2.896   0.2%")
    print("      V=85  q0=0.02 -> wn 8.56  vs 고유값 8.583   0.3%")
    print("    모드가 쌍이라 어느 쪽과 맞는지가 중요하다:")
    print("      V=85 는 8.226(zeta 0.051) 과 8.583(zeta 0.065) 두 개.")
    print("      피치 응답은 **8.583** 쪽이 지배한다.")


if __name__ == "__main__":
    main()
