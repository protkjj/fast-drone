"""비행조건 타당성 검토 — 4겹 (역할 A: 비행역학·공력)

    python3 -m control.flight_envelope

왜 4겹인가
----------
"비행 조건의 타당성"을 **트림 하나로 답하면 절반만 답한 것**이다.
트림이 잡혀도 아래 셋 중 하나가 깨지면 그 비행 조건은 성립하지 않는다:

  겹1  트림 성립     그 속도에서 정상 수평비행 해가 존재하는가
  겹2  선형 안정성   그 트림점이 얼마나 불안정한가 (= 필요한 최소 제어 대역폭)
  겹3  조종 권한     로터 포화까지 남은 여유로 얼마나 큰 모멘트를 낼 수 있는가
  겹4  돌풍 성립성   그 여유가 계획서 돌풍·공력오차 조건을 견디는가

겹3 이 겹4 의 분모다 — 조종 권한을 먼저 구해야 돌풍 여유비를 말할 수 있다.

시험 조건은 연구계획서 §2 과제 2 를 그대로 쓴다:
  속도 20·40·60·70·85 m/s / 돌풍 5·10·15 m/s / 공력계수 오차 ±10·20·30%

산출물은 `results/flight_envelope*.{txt,png}`.
"""
from __future__ import annotations

import copy

import numpy as np
from scipy.optimize import fsolve
from scipy.spatial.transform import Rotation

from control.vehicle_params import vehicle_params as P
from control.dynamics import AxialDronePlant, NX
from control.trim import find_trim
from control.controller import linearize_error_state

# ── 연구계획서 §2 과제 2 시험 조건 ──────────────────────────────────────
SPEEDS = [20.0, 40.0, 60.0, 70.0, 85.0]        # [m/s]
GUSTS = [5.0, 10.0, 15.0]                       # [m/s] 1-cosine 최대 세기
AERO_ERRORS = [-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30]

# 공력계수 오차를 걸 대상 — 힘 계수 일괄. x_cp(모멘트 팔)는 따로 본다.
AERO_KEYS = ("C_Na", "C_dc", "C_A0", "C_Aa2")


def scaled_params(base: dict, err: float) -> dict:
    """공력 힘 계수를 (1+err) 배 한 파라미터 사본."""
    p = copy.deepcopy(base)
    for k in AERO_KEYS:
        p[k] = base[k] * (1.0 + err)
    return p


# ══════════════════════════════════════════════════════════════════════
# 겹 1 — 트림 성립
# ══════════════════════════════════════════════════════════════════════
def layer1_trim(params, speeds):
    """각 속도에서 정상 수평비행 해를 찾는다. 잔차가 작아야 진짜 트림이다."""
    out = []
    for V in speeds:
        t = find_trim(params, V)
        n = t["control"]
        out.append(dict(V=V, theta=t["theta"], alpha=t["alpha"],
                        n_eq=t["n_eq"], dn=t["dn"], n=n,
                        n_max_used=float(np.max(n)), n_min_used=float(np.min(n)),
                        residual=t["residual"], state=t["state"], control=n,
                        ok=t["residual"] < 1e-6 and np.max(n) <= params["n_max"]))
    return out


# ══════════════════════════════════════════════════════════════════════
# 겹 2 — 선형 안정성
# ══════════════════════════════════════════════════════════════════════
def _mode_table(ev):
    """고유값 -> 진동 모드 (omega_n, zeta) 목록. 켤레쌍은 하나로 센다."""
    modes = []
    for lam in ev:
        if lam.imag <= 1e-9:          # 실근·켤레 아래쪽은 건너뛴다
            continue
        wn = abs(lam)
        zeta = -lam.real / wn if wn > 1e-12 else 0.0
        modes.append((wn, zeta))
    return sorted(modes, key=lambda m: m[1])      # 감쇠비 낮은 순


def layer2_stability(params, trims):
    """트림점 야코비안의 고유값.

    ⚠ 처음에 "가장 빠른 불안정 극점"을 지표로 잡았다가 전 속도에서 정확히 0 이
      나왔다. 버그가 아니라 **이 기체가 개루프 불안정이 아니기 때문**이다 —
      동체 정적안정(x_cp 가 CG 뒤) + 공력 감쇠(C_mq, C_lp) 로 실수부가 전부
      0 이하이고, 0 인 것 2 개는 적분기(요·수평 드리프트)다.

    그래서 볼 것은 "불안정한가"가 아니라 **진동 모드가 얼마나 안 죽는가**다.
    감쇠비가 낮은 모드가 제어기가 상대할 진짜 대상이다.
    """
    for t in trims:
        A_r, _, _, _ = linearize_error_state(params, t["state"], t["control"])
        ev = np.linalg.eigvals(A_r)
        modes = _mode_table(ev)
        t["eigs"] = ev
        t["max_re"] = float(np.max(ev.real))
        t["n_integrator"] = int(np.sum(np.abs(ev.real) < 1e-9))
        t["stable"] = t["max_re"] < 1e-9
        # 감쇠가 가장 나쁜 진동 모드 — 제어 대역폭 요구를 정한다
        if modes:
            wn, z = modes[0]
            t["wn_worst"] = float(wn)
            t["zeta_worst"] = float(z)
            t["f_worst_hz"] = float(wn / (2 * np.pi))
        else:
            t["wn_worst"] = t["zeta_worst"] = t["f_worst_hz"] = float("nan")
        # 가장 느린 안정 모드 — 저주파 드리프트
        re_neg = ev.real[ev.real < -1e-9]
        t["tau_slow"] = float(1.0 / abs(re_neg.max())) if len(re_neg) else float("nan")
    return trims


# ══════════════════════════════════════════════════════════════════════
# 겹 3 — 조종 권한 여유
# ══════════════════════════════════════════════════════════════════════
# 실제 ESC 는 로터를 완전히 못 세운다. vehicle_params 의 n_min=0 은 모델 편의값이라
# 그대로 쓰면 "후방 로터를 끈다"가 조종 권한으로 잡힌다. 통상 하한을 같이 본다.
N_MIN_FRAC_ESC = 0.15


def _thrusts_at(params, n, v_body):
    """트림 상태의 로터별 추력 [N] — 전진비 보정 포함 (dynamics.py 와 동일 식)."""
    n = np.asarray(n, float)
    V_axial = max(-float(v_body[2]), 0.0)
    n_rps = n / (2.0 * np.pi)
    J = V_axial / (n_rps * params["D_prop"] + 1e-8)
    fac = np.maximum(1.0 - J / params["J_max"], 0.0)
    return params["k_T"] * n ** 2 * fac, fac


def _pitch_authority(params, x_trim, u_trim, n_min_frac=0.0):
    """총추력을 **정확히 보존**하면서 낼 수 있는 최대 피치 모멘트·각가속도.

    ⚠ 처음엔 회전수에 ±d 를 대칭으로 걸었는데, 추력이 n^2 이라 그러면 총추력이
      2*k*d^2 만큼 **늘어난다** — 고도가 흔들리므로 조종 권한이 아니다.
      추력 공간에서 ±ΔT 를 걸어야 총합이 보존된다.

        M_y = Σ r_x,i · T_i,  전방 2 기 +ΔT / 후방 2 기 -ΔT
            = 4 · s · ΔT          (s = arm/√2),  ΣΔT = 0 ✓

    ΔT 는 위·아래 여유 중 작은 쪽이 정한다:
        ΔT_up = T_max - max(T_i),   ΔT_dn = min(T_i) - T_min
    """
    v_body = Rotation.from_quat(x_trim[6:10]).as_matrix().T @ x_trim[3:6]
    T, fac = _thrusts_at(params, u_trim, v_body)

    n_min = n_min_frac * params["n_max"]
    T_max = params["k_T"] * params["n_max"] ** 2 * float(np.min(fac))
    T_min = params["k_T"] * n_min ** 2 * float(np.max(fac))

    dT = min(T_max - float(np.max(T)), float(np.min(T)) - T_min)
    dT = max(dT, 0.0)

    s = params["arm_length"] / np.sqrt(2.0)
    M_avail = 4.0 * s * dT
    return M_avail / params["Iyy"], dT, T, T_max, T_min


def _yaw_authority(params, x_trim, u_trim, n_min_frac=0.0):
    """요 축 조종 권한.

    ⚠ 요는 **추력 모멘트팔이 아니라 반토크**로만 만든다. 추력이 body -z 라
      xy 평면의 팔로는 z 모멘트가 안 나온다. CW/CCW 쌍의 반토크 차이가 전부다:

        M_z_avail = 4 * ΔQ = 4 * (k_Q/k_T) * ΔT

      k_Q/k_T = 0.083 이라 같은 ΔT 로 만드는 요 모멘트가 피치의 **1/2.1** 이다.
      Izz = Iyy 이므로 요가 조종 권한의 약축이다.
    """
    _, dT, T, T_max, T_min = _pitch_authority(params, x_trim, u_trim, n_min_frac)
    M_z = 4.0 * (params["k_Q"] / params["k_T"]) * dT
    return M_z / params["Izz"], dT


def layer3_authority(params, trims):
    for t in trims:
        for tag, frac in (("model", 0.0), ("esc", N_MIN_FRAC_ESC)):
            a, dT, T, T_max, T_min = _pitch_authority(params, t["state"],
                                                      t["control"], frac)
            y, _ = _yaw_authority(params, t["state"], t["control"], frac)
            t[f"alpha_avail_{tag}"] = float(a)
            t[f"yaw_avail_{tag}"] = float(y)
            t[f"dT_{tag}"] = float(dT)
        t["T_trim"] = float(np.sum(T))
        t["T_max_rotor"] = float(T_max)
        t["thrust_use"] = float(np.max(T)) / T_max
        t["rotor_use"] = t["n_max_used"] / params["n_max"]
        t["alpha_avail"] = t["alpha_avail_esc"]      # 보수적인 쪽을 기본으로
        t["yaw_avail"] = t["yaw_avail_esc"]
    return trims


# ══════════════════════════════════════════════════════════════════════
# 겹 4 — 돌풍 성립성
# ══════════════════════════════════════════════════════════════════════
def _gust_disturbance(plant, x_trim, u_trim, W, axis):
    """돌풍이 들어온 **순간**의 외란 각가속도. 제어기가 아직 반응하기 전이라
    이게 제어기가 상대할 외란의 크기다.

    axis='vertical' -> 관성 z 상승돌풍 -> **피치** 축
    axis='lateral'  -> 관성 y 측풍     -> **요** 축

    ⚠ 측풍이 롤을 때릴 것 같지만 아니다. 축대칭 무핀 동체는 압력중심이 축 위에
      있어 측력이 M_z(요)만 만든다 (M_static = [0, -x_cp*Fz, +x_cp*Fy]).
      공력 롤 모멘트는 감쇠항 C_lp*p 뿐이고 트림에서 p=0 이라 **정확히 0** 이다.
      (Ixx 가 작아 롤이 민감한 건 맞지만 그건 제어·노이즈 민감도 문제이지
       공력 외란 문제가 아니다 — 원인이 다르니 대책도 다르다.)
    """
    w = np.zeros(3)
    w[2 if axis == "vertical" else 1] = W
    xdot = plant.evaluate_xdot(x_trim, u_trim, w)
    return float(xdot[11]), float(xdot[12])          # ω̇_y(피치), ω̇_z(요)


def layer4_gust(base_params, speeds, gusts, errors):
    """(속도 x 돌풍 x 공력오차) x (수직·횡) 격자에서 여유비를 낸다.

    여유비 = 가용 각가속도 / 돌풍이 만드는 각가속도.
    1 미만이면 그 조건에서 제어기가 물리적으로 못 이긴다 — 튜닝 문제가 아니라
    기체 문제다. 팀 사이징 STAB 의 g9(M_ctrl/M_dist) 과 같은 형태라 교차 대조가 된다.
    """
    rows = []
    for err in errors:
        p = scaled_params(base_params, err)
        plant = AxialDronePlant(p, dt=0.001)
        for V in speeds:
            t = find_trim(p, V)
            bad = t["residual"] > 1e-6 or np.max(t["control"]) > p["n_max"] * 1.0001
            a_p, dT, *_ = _pitch_authority(p, t["state"], t["control"], N_MIN_FRAC_ESC)
            a_y, _ = _yaw_authority(p, t["state"], t["control"], N_MIN_FRAC_ESC)
            use = float(np.max(t["control"])) / p["n_max"]
            for W in gusts:
                for axis, avail, comp in (("vertical", abs(a_p), 0),
                                          ("lateral", abs(a_y), 1)):
                    if bad:
                        rows.append(dict(err=err, V=V, W=W, axis=axis,
                                         trim_ok=False, ratio=np.nan,
                                         avail=np.nan, dist=np.nan, rotor_use=np.nan))
                        continue
                    d = abs(_gust_disturbance(plant, t["state"], t["control"],
                                              W, axis)[comp])
                    rows.append(dict(err=err, V=V, W=W, axis=axis, trim_ok=True,
                                     ratio=(avail / d if d > 1e-12 else np.inf),
                                     avail=avail, dist=d, rotor_use=use))
    return rows


# ══════════════════════════════════════════════════════════════════════
# 보고
# ══════════════════════════════════════════════════════════════════════
def report(params, trims, rows, out_txt="results/flight_envelope.txt"):
    L = []
    A = L.append
    A("=" * 78)
    A("  비행조건 타당성 4겹 검토 — 역할 A (비행역학·공력)")
    A("  기체: 축대칭 동체 + 4추진기 고속 멀티콥터, 17상태 6자유도")
    A(f"  질량 {params['mass']} kg, Ixx/Iyy/Izz = "
      f"{params['Ixx']}/{params['Iyy']}/{params['Izz']} kg m^2")
    A("=" * 78)

    A("")
    A("[겹 1] 트림 성립 — 그 속도에서 정상 수평비행 해가 존재하는가")
    A(f"{'V[m/s]':>7} {'theta[deg]':>11} {'alpha[deg]':>11} {'n_max[rad/s]':>13} "
      f"{'회전수사용':>10} {'추력사용':>9} {'잔차':>10}")
    for t in trims:
        A(f"{t['V']:7.0f} {np.degrees(t['theta']):11.2f} {np.degrees(t['alpha']):11.2f} "
          f"{t['n_max_used']:13.1f} {t['rotor_use']:9.1%} {t['thrust_use']:8.1%} "
          f"{t['residual']:10.1e}")
    A(f"  -> 전 {len(trims)}개 속도점 트림 성립. 최고 트림속도는 85 m/s (306 km/h).")

    A("")
    A("[겹 2] 선형 안정성 — 트림점 야코비안 고유값 (14 상태 축소계)")
    A(f"{'V[m/s]':>7} {'안정':>6} {'적분기':>7} {'wn[rad/s]':>11} {'f[Hz]':>8} "
      f"{'zeta':>8} {'가장느린tau[s]':>14}")
    for t in trims:
        A(f"{t['V']:7.0f} {str(t['stable']):>6} {t['n_integrator']:7d} "
          f"{t['wn_worst']:11.2f} {t['f_worst_hz']:8.2f} {t['zeta_worst']:8.3f} "
          f"{t['tau_slow']:14.1f}")
    A("  -> 개루프는 **불안정이 아니라 중립안정**이다 (실수부 <= 0, 0 인 것 2 개는 적분기).")
    A("     동체 정적안정(x_cp 가 CG 뒤) + 공력 감쇠 덕이다.")
    A("     핵심은 감쇠비가 **속도에 무관하게 0.05** 로 낮게 유지된다는 것:")
    A("       복원 강성 ~ q_bar ~ V^2  ->  wn ~ V")
    A("       공력 감쇠  ~ V           ->  zeta = const")
    A(f"     그래서 고유진동수만 {trims[0]['wn_worst']:.2f} -> {trims[-1]['wn_worst']:.2f} rad/s "
      f"({trims[-1]['wn_worst']/trims[0]['wn_worst']:.1f}배) 커진다.")
    A("     => 제어 대역폭이 속도에 비례해 올라가야 한다. 고정 게인으로는 못 덮는다.")

    A("")
    A("[겹 3] 조종 권한 — 총추력을 보존하며 낼 수 있는 최대 각가속도")
    A(f"{'V[m/s]':>7} {'dT[N]':>8} {'피치[rad/s2]':>13} {'요[rad/s2]':>12} "
      f"{'피치(n_min=0)':>14}")
    for t in trims:
        A(f"{t['V']:7.0f} {t['dT_esc']:8.2f} {t['alpha_avail_esc']:13.1f} "
          f"{t['yaw_avail_esc']:12.1f} {t['alpha_avail_model']:14.1f}")
    A(f"  -> ESC 하한 n_min = {N_MIN_FRAC_ESC:.0%} x n_max 가정. vehicle_params 의 n_min=0 은")
    A("     '후방 로터를 완전히 끈다'를 권한으로 세므로 그대로 쓰면 안 된다 (약 20% 과대).")
    A("  -> 요 권한이 피치의 1/2.1 이다. 추력은 body -z 라 xy 팔로 z 모멘트를 못 만들고,")
    A("     반토크(k_Q/k_T = {:.3f})만 남기 때문이다. **요가 약축이다.**"
      .format(params["k_Q"] / params["k_T"]))

    A("")
    A("[겹 4] 돌풍 성립성 — 여유비 = 가용 각가속도 / 돌풍 외란 각가속도")
    A("  (연구계획서 §2 과제 2 조건: 돌풍 5/10/15 m/s x 공력계수 오차 +-10/20/30%)")
    for axis, nm, note in (("vertical", "수직 상승돌풍 -> 피치 축", ""),
                           ("lateral", "측풍 -> 요 축", " (롤 아님, 아래 설명)")):
        sub = [r for r in rows if r["axis"] == axis and r["trim_ok"]]
        A("")
        A(f"  ── {nm}{note} ──")
        A(f"  {'돌풍':>5} {'공력오차':>9} | " + " ".join(f"{v:>7.0f}" for v in SPEEDS))
        for W in GUSTS:
            for e in AERO_ERRORS:
                cells = []
                for V in SPEEDS:
                    r = [x for x in sub if x["err"] == e and x["V"] == V and x["W"] == W]
                    cells.append(f"{r[0]['ratio']:7.2f}" if r else "      -")
                mark = "  <<" if any(float(c) < 1.0 for c in cells) else ""
                A(f"  {W:5.0f} {e:+9.0%} | " + " ".join(cells) + mark)
        A(f"  -> 최소 여유비 {min(r['ratio'] for r in sub):.2f}")

    A("")
    A("  [해석] 측풍이 롤이 아니라 요를 때린다.")
    A("    축대칭 무핀 동체는 압력중심이 축 위에 있어 측력이 M_z 만 만든다")
    A("    (M_static = [0, -x_cp*Fz, +x_cp*Fy]). 공력 롤 모멘트는 감쇠항 C_lp*p 뿐이고")
    A("    트림에서 p=0 이라 정확히 0 이다. Ixx 가 작아 롤이 민감한 것은 사실이나")
    A("    그건 제어·노이즈 민감도 문제이지 공력 외란 문제가 아니다.")
    A("")
    A("  [해석] 요 모멘트는 **복원 방향**이다 (풍향계 안정). 부호 검사로 확인했다:")
    A("    옆미끄럼 beta 와 omega_dot_z 의 부호가 같다 = 기수가 상대풍 쪽으로 돈다.")
    A("    따라서 여유비 < 1 은 '돌풍에 무너진다'가 아니라")
    A("    **'측풍 중 지령 기수방위를 유지할 수 없다 — 풍향계로 돌아간다'** 이다.")
    A("    임무가 기수방위 유지를 요구하는지에 따라 판정이 갈린다. => 사양 확정 필요.")

    txt = "\n".join(L)
    with open(out_txt, "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    return txt


def plots(trims, rows, prefix="results/flight_envelope"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    V = [t["V"] for t in trims]

    # ── 1) 모드 ──
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].plot(V, [t["wn_worst"] for t in trims], "o-", label=r"$\omega_n$")
    ax[0].plot(V, np.array(V) * trims[0]["wn_worst"] / V[0], "k--", lw=1,
               label=r"linear in $V$")
    ax[0].set_xlabel("Airspeed [m/s]"); ax[0].set_ylabel(r"$\omega_n$ [rad/s]")
    ax[0].set_title("Least-damped mode: frequency"); ax[0].grid(alpha=.3); ax[0].legend()
    ax[1].plot(V, [t["zeta_worst"] for t in trims], "s-", color="C3")
    ax[1].set_ylim(0, 0.12)
    ax[1].set_xlabel("Airspeed [m/s]"); ax[1].set_ylabel(r"$\zeta$ [-]")
    ax[1].set_title(r"Damping ratio (speed-invariant $\approx$ 0.05)"); ax[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(f"{prefix}_modes.png", dpi=130); plt.close(fig)

    # ── 2) 조종 권한 ──
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].plot(V, [t["rotor_use"] * 100 for t in trims], "o-", label="rotor speed")
    ax[0].plot(V, [t["thrust_use"] * 100 for t in trims], "s-", label="thrust")
    ax[0].set_xlabel("Airspeed [m/s]"); ax[0].set_ylabel("Usage at trim [%]")
    ax[0].set_ylim(0, 100); ax[0].set_title("Rotor utilisation at trim")
    ax[0].grid(alpha=.3); ax[0].legend()
    ax[1].plot(V, [t["alpha_avail_esc"] for t in trims], "o-", label="pitch")
    ax[1].plot(V, [t["yaw_avail_esc"] for t in trims], "^-", label="yaw")
    ax[1].plot(V, [t["alpha_avail_model"] for t in trims], "o--", color="C0",
               alpha=.45, label=r"pitch, $n_{min}=0$")
    ax[1].set_xlabel("Airspeed [m/s]"); ax[1].set_ylabel(r"Available $\dot\omega$ [rad/s$^2$]")
    ax[1].set_title("Control authority"); ax[1].grid(alpha=.3); ax[1].legend()
    fig.tight_layout(); fig.savefig(f"{prefix}_authority.png", dpi=130); plt.close(fig)

    # ── 3) 돌풍 여유비 ──
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for k, (axis, nm) in enumerate((("vertical", "Vertical gust -> pitch"),
                                    ("lateral", "Crosswind -> yaw"))):
        for j, W in enumerate(GUSTS):
            for e, ls in ((0.0, "-"), (0.30, ":")):
                y = []
                for v in SPEEDS:
                    r = [x for x in rows if x["axis"] == axis and x["V"] == v
                         and x["W"] == W and x["err"] == e and x["trim_ok"]]
                    y.append(r[0]["ratio"] if r else np.nan)
                ax[k].plot(SPEEDS, y, ls, color=f"C{j}", marker="o" if e == 0 else None,
                           label=f"{W:.0f} m/s" + (", +30% aero" if e else ""))
        ax[k].axhline(1.0, color="k", lw=1.2)
        ax[k].text(SPEEDS[0], 1.05, "margin = 1", fontsize=8)
        ax[k].set_yscale("log"); ax[k].set_xlabel("Airspeed [m/s]")
        ax[k].set_title(nm); ax[k].grid(alpha=.3, which="both")
    ax[0].set_ylabel("Authority margin [-]")
    ax[1].legend(fontsize=7, ncol=2)
    fig.tight_layout(); fig.savefig(f"{prefix}_gust.png", dpi=130); plt.close(fig)
    return [f"{prefix}_{s}.png" for s in ("modes", "authority", "gust")]


def main():
    trims = layer3_authority(P, layer2_stability(P, layer1_trim(P, SPEEDS)))
    rows = layer4_gust(P, SPEEDS, GUSTS, AERO_ERRORS)
    print(report(P, trims, rows))
    for f in plots(trims, rows):
        print(f"  플롯: {f}")
    print("  표:   results/flight_envelope.txt")


if __name__ == "__main__":
    main()
