#!/usr/bin/env python3
"""브라우저 시뮬의 기준값을 만든다. 검증 사슬의 가운데 고리.

    python3 gz_aero/tools/gen_sim_reference.py

왜 필요한가:
  브라우저에서 6자유도를 직접 푸는 것은 같은 물리의 **세 번째 구현**이다
  (파이썬 CasADi / C++ 플러그인 / JS). 구현이 하나 늘면 어긋날 자리가 하나
  는다. 이 프로젝트에서 좌표계 부호 사고를 그렇게 경계한 이유가 그것이다.

검증 사슬:
    control/dynamics.py (CasADi)        ← 원본. 파이썬 시뮬이 이걸 쓴다
        ↕  이 스크립트가 대조
    plain_xdot (여기, 순수 파이썬)       ← JS 가 따라 할 대상
        ↕  대시보드가 페이지에서 대조
    JS 포트                              ← 브라우저

  CasADi 는 연산 순서를 재배치하므로 비트 단위 일치를 기대할 수 없다.
  대신 순수 파이썬을 JS 와 **연산 순서까지 똑같이** 써서 그 고리는 비트 단위로
  맞추고, CasADi 와의 고리는 수치로 대조한다.
"""
import argparse
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from control.vehicle_params import vehicle_params as VP_Z   # noqa: E402
from control.vehicle_params import rocket_params as VP     # noqa: E402

EPS = 1e-8
NX = 17


def params():
    """JS 로 넘길 평평한 파라미터. numpy 를 걷어낸다."""
    p = {k: VP[k] for k in (
        "mass", "Ixx", "Iyy", "Izz", "S_ref", "d_ref", "rho", "g",
        "C_Na", "C_dc", "C_A0", "C_Aa2", "x_cp", "C_mq", "C_lp",
        "num_rotors", "D_prop", "k_T", "k_Q", "J_max", "I_rotor",
        "tau_m", "n_min", "n_max")}
    p["thrust_axis"] = VP.get("thrust_axis", "z")
    p["rotor_positions"] = [[float(v) for v in r] for r in VP["rotor_positions"]]
    p["rotor_directions"] = [float(v) for v in VP["rotor_directions"]]
    p["S_ref"] = float(p["S_ref"])
    return p


def plain_xdot(x, u, w, p):
    """x_dot. **JS 포트가 그대로 따라 할 기준이다.**

    연산 순서를 바꾸면 JS 와 마지막 자리가 어긋나므로 손대지 말 것.
    dynamics.py 의 _compute_xdot 을 순수 파이썬으로 옮긴 것이다.
    """
    vel = x[3:6]
    qx, qy, qz, qw = x[6], x[7], x[8], x[9]
    om = x[10:13]
    nv = x[13:17]

    # 쿼터니언 -> 회전행렬 (동체 -> 관성)
    R = ((1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
         (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
         (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)))

    # 대기속도를 동체 좌표로:  v_body = R^T (vel - w)
    d0, d1, d2 = vel[0] - w[0], vel[1] - w[1], vel[2] - w[2]
    ub = R[0][0] * d0 + R[1][0] * d1 + R[2][0] * d2
    vb = R[0][1] * d0 + R[1][1] * d1 + R[2][1] * d2
    wb = R[0][2] * d0 + R[1][2] * d1 + R[2][2] * d2

    # ── 동체 공력 ────────────────────────────────────────────────────
    V_sq = ub * ub + vb * vb + wb * wb + EPS
    V = math.sqrt(V_sq)
    V_cf = math.sqrt(vb * vb + wb * wb + EPS)
    q_bar = 0.5 * p["rho"] * V_sq

    F_N_fac = 0.5 * p["rho"] * p["S_ref"] * (p["C_Na"] * ub + p["C_dc"] * V_cf)
    Fy = -F_N_fac * vb
    Fz = -F_N_fac * wb
    C_A = p["C_A0"] + p["C_Aa2"] * (vb * vb + wb * wb) / V_sq
    Fx = -q_bar * p["S_ref"] * C_A

    xcp = p["x_cp"]
    Mx = 0.0
    My = -xcp * Fz
    Mz = xcp * Fy

    df = 0.25 * p["rho"] * V * p["S_ref"] * p["d_ref"] * p["d_ref"]
    Mx += df * p["C_lp"] * om[0]
    My += df * p["C_mq"] * om[1]
    Mz += df * p["C_mq"] * om[2]

    # ── 로터 ────────────────────────────────────────────────────────
    # thrust_axis 'x' = 로터가 동체축에 수직인 평면에 놓이고 추력이 기수 방향
    #                   (로켓형, 추진까지 축대칭)
    # thrust_axis 'z' = 추력이 동체 -z (어뢰 동체를 수평으로 단 일반 쿼드, 역호환)
    ax_is_x = p["thrust_axis"] == "x"
    V_axial = (ub if ub > 0.0 else 0.0) if ax_is_x else (-wb if -wb > 0.0 else 0.0)
    h_net = 0.0
    Frx = Frz = 0.0
    for i in range(int(p["num_rotors"])):
        ni = nv[i]
        di = p["rotor_directions"][i]
        ri = p["rotor_positions"][i]
        n_rps = ni / (2.0 * math.pi)
        J = V_axial / (n_rps * p["D_prop"] + EPS)
        fac = 1.0 - J / p["J_max"]
        if fac < 0.0:
            fac = 0.0
        Ti = p["k_T"] * ni * ni * fac
        Qi = p["k_Q"] * ni * ni * fac
        if ax_is_x:
            Frx += Ti
            My += ri[2] * Ti
            Mz += -ri[1] * Ti
            Mx += di * Qi
        else:
            Frz += -Ti
            Mx += ri[1] * (-Ti)
            My += -ri[0] * (-Ti)
            Mz += di * Qi
        # 반토크를 +d_i*Q_i 로 쓰므로 로터는 -d_i 로 돈다 -> h = -I_r*sum(d_i*n_i).
        # dynamics.py 와 같은 부호. 연산 순서까지 같게 둔다.
        h_net -= p["I_rotor"] * ni * di

    if ax_is_x:
        My += -om[2] * h_net
        Mz += om[1] * h_net
    else:
        Mx += -om[1] * h_net
        My += om[0] * h_net

    Fbx, Fby, Fbz = Fx + Frx, Fy, Fz + Frz

    # ── 조립 ────────────────────────────────────────────────────────
    m = p["mass"]
    ax = (R[0][0] * Fbx + R[0][1] * Fby + R[0][2] * Fbz) / m
    ay = (R[1][0] * Fbx + R[1][1] * Fby + R[1][2] * Fbz) / m
    az = -p["g"] + (R[2][0] * Fbx + R[2][1] * Fby + R[2][2] * Fbz) / m

    # q_dot = 0.5 G^T ω  - (|q|^2 - 1) q   (Baumgarte)
    p_, q_, r_ = om[0], om[1], om[2]
    qd0 = 0.5 * (qw * p_ - qz * q_ + qy * r_)
    qd1 = 0.5 * (qz * p_ + qw * q_ - qx * r_)
    qd2 = 0.5 * (-qy * p_ + qx * q_ + qw * r_)
    qd3 = 0.5 * (-qx * p_ - qy * q_ - qz * r_)
    nrm = qx * qx + qy * qy + qz * qz + qw * qw - 1.0
    qd0 -= nrm * qx
    qd1 -= nrm * qy
    qd2 -= nrm * qz
    qd3 -= nrm * qw

    Jx, Jy, Jz = p["Ixx"], p["Iyy"], p["Izz"]
    wdx = (Mx - (q_ * (Jz * r_) - r_ * (Jy * q_))) / Jx
    wdy = (My - (r_ * (Jx * p_) - p_ * (Jz * r_))) / Jy
    wdz = (Mz - (p_ * (Jy * q_) - q_ * (Jx * p_))) / Jz

    tau = p["tau_m"]
    return [vel[0], vel[1], vel[2], ax, ay, az,
            qd0, qd1, qd2, qd3, wdx, wdy, wdz,
            (u[0] - nv[0]) / tau, (u[1] - nv[1]) / tau,
            (u[2] - nv[2]) / tau, (u[3] - nv[3]) / tau]


def rk4_step(x, u, w, p, dt, sub=4):
    """CasADi 'rk' + number_of_finite_elements=4 와 같은 구조."""
    h = dt / sub
    for _ in range(sub):
        k1 = plain_xdot(x, u, w, p)
        x2 = [x[i] + 0.5 * h * k1[i] for i in range(NX)]
        k2 = plain_xdot(x2, u, w, p)
        x3 = [x[i] + 0.5 * h * k2[i] for i in range(NX)]
        k3 = plain_xdot(x3, u, w, p)
        x4 = [x[i] + h * k3[i] for i in range(NX)]
        k4 = plain_xdot(x4, u, w, p)
        x = [x[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
             for i in range(NX)]
    # dynamics.py 의 step() 이 하는 뒤처리를 그대로
    qn = math.sqrt(sum(x[6 + i] ** 2 for i in range(4)))
    if qn > 1e-10:
        for i in range(4):
            x[6 + i] /= qn
    lo, hi = p["n_min"], p["n_max"]
    for i in range(13, 17):
        x[i] = lo if x[i] < lo else (hi if x[i] > hi else x[i])
    return x


def hover_state(p):
    """호버 자세. 추력축이 월드 +z 를 보게 한다.

    'z' 배치: 추력이 동체 -z 이므로 x 축 180도 (q = [1,0,0,0]).
    'x' 배치: 추력이 동체 +x 이므로 y 축 -90도 -> 기수가 위를 본다.
    """
    n = math.sqrt(p["mass"] * p["g"] / (4.0 * p["k_T"]))
    x = [0.0] * NX
    if p["thrust_axis"] == "x":
        c = math.cos(-math.pi / 4.0)
        x[6], x[7], x[8], x[9] = 0.0, math.sin(-math.pi / 4.0), 0.0, c
    else:
        x[6] = 1.0
    for i in range(13, 17):
        x[i] = n
    return x


def seg_defs(p, dt):
    """검증 구간들. **운용영역을 실제로 훑어야** 배지가 뜻을 갖는다.

    예전에는 호버에서 정지 상태로 0.8 초를 도는 구간 하나뿐이었다. 그 궤적의
    실측 범위가 대기속도 3.0~3.2 m/s, fac 0.999~1.000 이었다 — 즉 **전진비
    분기가 한 번도 실행되지 않았다.** 그래서 할당이 전진비를 무시하던 버그가
    이 배지를 그대로 통과했다. 그 일을 되풀이하지 않으려고 구간을 늘린다.

    각 구간은 하나씩 '이 항이 죽어 있지 않다' 를 담당한다.
    """
    n_h = math.sqrt(p["mass"] * p["g"] / (4.0 * p["k_T"]))
    segs = []

    # 1) 호버 비대칭 — 예전 구간 그대로. 네 로터가 다 다르고 바람도 비스듬하다.
    segs.append(("호버 비대칭", hover_state(p),
                 [n_h * 1.03, n_h * 0.98, n_h * 1.01, n_h * 0.99],
                 [3.0, -1.0, 0.5]))

    # 2~5) 고속 구간. 트림을 못 구하면 손으로 자세를 만들어서라도 넣는다 —
    #      트림이 없다고 검증 범위를 좁히면 본말이 전도된다.
    def tilted(V, deg, wz=0.0):
        """기수를 deg 만큼 눕히고 월드 +x 로 V 로 나는 상태."""
        x = [0.0] * NX
        a = -math.pi / 2.0 + math.radians(deg)      # y 축 둘레
        x[7], x[9] = math.sin(a / 2.0), math.cos(a / 2.0)
        x[2] = 200.0
        x[3] = V
        x[5] = wz
        for i in range(13, 17):
            x[i] = n_h
        return x

    # 2) 고속 순항 — 전진비가 실제로 물린다 (fac < 1).
    x2 = tilted(45.0, 60.0)
    for i in range(13, 17):
        x2[i] = 900.0
    segs.append(("고속 순항 45 m/s", x2,
                 [930.0, 880.0, 870.0, 920.0], [0.0, 0.0, 0.0]))

    # 3) 고속 + 큰 차동 — 로터마다 fac 가 갈리고 자이로 항이 산다.
    x3 = tilted(60.0, 70.0)
    for i in range(13, 17):
        x3[i] = 950.0
    x3[10], x3[11], x3[12] = 1.5, -2.5, 0.8
    segs.append(("고속 차동 + 각속도", x3,
                 [1300.0, 600.0, 1250.0, 620.0], [0.0, 0.0, 0.0]))

    # 4) 저회전 고속 — fac 가 0 으로 내려가는 영역. 오늘 버그가 살던 자리다.
    x4 = tilted(50.0, 65.0)
    for i in range(13, 17):
        x4[i] = 500.0
    segs.append(("저회전 고속 (fac→0)", x4,
                 [520.0, 430.0, 510.0, 440.0], [0.0, 0.0, 0.0]))

    # 5) 측풍 — 횡류 항(C_dc)과 큰 받음각.
    x5 = tilted(35.0, 45.0, wz=-4.0)
    for i in range(13, 17):
        x5[i] = 800.0
    segs.append(("측풍 + 받음각", x5,
                 [850.0, 760.0, 830.0, 780.0], [0.0, 18.0, -3.0]))

    # 6) 큰 각속도 — 자이로 항과 쿼터니언 정규화가 일하는 구간.
    x6 = hover_state(p)
    x6[2] = 100.0
    x6[10], x6[11], x6[12] = 4.0, -6.0, 3.0
    for i in range(13, 17):
        x6[i] = n_h
    segs.append(("큰 각속도", x6,
                 [n_h * 1.15, n_h * 0.85, n_h * 1.10, n_h * 0.90],
                 [0.0, 0.0, 0.0]))
    return segs


def coverage(p, x, u):
    """이 상태에서 무엇이 실제로 물리는지. 배지가 보증하는 범위를 숫자로 남긴다."""
    qx, qy, qz, qw = x[6], x[7], x[8], x[9]
    R = ((1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
         (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
         (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)))
    d = [x[3], x[4], x[5]]
    ub = R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2]
    vb = R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2]
    wb = R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2]
    V = math.sqrt(ub * ub + vb * vb + wb * wb)
    alpha = math.degrees(math.atan2(math.sqrt(vb * vb + wb * wb), ub))
    om = math.sqrt(x[10] ** 2 + x[11] ** 2 + x[12] ** 2)
    axial = ub if ub > 0 else 0.0
    facs = []
    for i in range(4):
        ni = x[13 + i]
        nr = ni / (2.0 * math.pi)
        J = axial / (nr * p["D_prop"] + 1e-12)
        facs.append(max(0.0, 1.0 - J / p["J_max"]))
    return {"V": V, "alpha": alpha, "om": om,
            "fac_min": min(facs), "fac_max": max(facs),
            "n_min": min(x[13:17]), "n_max": max(x[13:17])}


def crosscheck_seg(p, dt, steps, x0, u, w):
    """CasADi 원본과 대조. 없으면 건너뛴다."""
    try:
        import numpy as np
        from control.dynamics import AxialDronePlant
    except Exception as e:
        return None, f"CasADi 대조 생략: {type(e).__name__}"
    plant = AxialDronePlant(VP, dt=dt)
    xs = list(x0)
    xc = np.array(x0, dtype=float)
    worst = 0.0
    for _ in range(steps):
        xs = rk4_step(xs, u, w, p, dt)
        xc = plant.step(xc, np.array(u), np.array(w))
        worst = max(worst, max(abs(a - b) for a, b in zip(xs, xc)))
    return worst, ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out",
                    default=str(ROOT / "gz_aero" / "data" / "sim_reference.json"))
    ap.add_argument("--dt", type=float, default=0.005)
    ap.add_argument("--steps", type=int, default=400)
    ap.add_argument("--every", type=int, default=20,
                    help="검문소 간격. 전 스텝을 담으면 파일이 커지는데, "
                         "오차는 쌓이므로 간격을 둬도 판별력은 같다.")
    a = ap.parse_args()

    p = params()
    n_h = math.sqrt(p["mass"] * p["g"] / (4.0 * p["k_T"]))

    segs_out, cov_all, worst_all, note = [], [], 0.0, ""
    for name, x0, u, w in seg_defs(p, a.dt):
        x = list(x0)
        checks, cov = [], [coverage(p, x, u)]
        for k in range(1, a.steps + 1):
            x = rk4_step(x, u, w, p, a.dt)
            if k % a.every == 0:
                checks.append(list(x))
            cov.append(coverage(p, x, u))
        worst, note = crosscheck_seg(p, a.dt, a.steps, x0, u, w)
        if worst is not None:
            worst_all = max(worst_all, worst)
        segs_out.append({"name": name, "x0": list(x0), "u": u, "w": w,
                         "checks": checks})
        cov_all.append((name, cov))

    from control.dynamics import compute_allocation_matrix   # noqa: E402
    _A, A_inv = compute_allocation_matrix(VP)
    alloc_inv = [[float(v) for v in row] for row in A_inv]

    hov = hover_state(p)
    ref = {"dt": a.dt, "steps": a.steps, "every": a.every,
           "segs": segs_out,
           "x0": hov, "n_hover": n_h,
           "casadi_max_diff": (worst_all if note == "" else None),
           "casadi_note": note,
           "alloc_inv": alloc_inv, "params": p}

    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(ref, separators=(",", ":")), encoding="utf-8")
    print(f"저장 {out}  ({out.stat().st_size / 1024:.0f} KB)")
    print(f"  구간 {len(segs_out)} 개 × {a.dt} s × {a.steps} 스텝"
          f" = 각 {a.dt * a.steps:.1f} s, 검문소 {a.steps // a.every} 곳")
    print()
    print(f"  {'구간':<20s} {'대기속도':>12s} {'받음각°':>12s} {'|ω|':>10s} {'fac':>14s}")
    g = {"V": [1e9, -1e9], "alpha": [1e9, -1e9], "om": [1e9, -1e9], "fac": [1e9, -1e9]}
    for name, cov in cov_all:
        V = (min(c["V"] for c in cov), max(c["V"] for c in cov))
        al = (min(c["alpha"] for c in cov), max(c["alpha"] for c in cov))
        om = (min(c["om"] for c in cov), max(c["om"] for c in cov))
        fc = (min(c["fac_min"] for c in cov), max(c["fac_max"] for c in cov))
        for key, v in (("V", V), ("alpha", al), ("om", om), ("fac", fc)):
            g[key][0] = min(g[key][0], v[0]); g[key][1] = max(g[key][1], v[1])
        print(f"  {name:<20s} {V[0]:5.1f}~{V[1]:5.1f} {al[0]:5.1f}~{al[1]:5.1f}"
              f" {om[0]:4.1f}~{om[1]:4.1f} {fc[0]:6.3f}~{fc[1]:6.3f}")
    print(f"  {'── 전체 ──':<20s} {g['V'][0]:5.1f}~{g['V'][1]:5.1f}"
          f" {g['alpha'][0]:5.1f}~{g['alpha'][1]:5.1f}"
          f" {g['om'][0]:4.1f}~{g['om'][1]:4.1f}"
          f" {g['fac'][0]:6.3f}~{g['fac'][1]:6.3f}")
    if g["fac"][0] > 0.99:
        print("  ❌ fac 가 전 구간 1.0 입니다 — 전진비 분기가 한 번도 안 돕니다.")
        return 1
    print()
    if note:
        print(f"  {note}")
    else:
        ok = worst_all < 1e-9
        print(f"  {'✅' if ok else '❌'} CasADi 원본과 최대차 {worst_all:.3e}"
              f"  (구간 {len(segs_out)} 개 전체)")
        if not ok:
            print("     순수 파이썬 포트가 원본과 어긋납니다. JS 로 넘기기 전에 고쳐야 합니다.")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
