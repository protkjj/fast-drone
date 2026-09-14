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
        h_net += p["I_rotor"] * ni * di

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


def crosscheck(p, dt, steps):
    """CasADi 원본과 대조. 없으면 건너뛴다."""
    try:
        import numpy as np
        from control.dynamics import AxialDronePlant
    except Exception as e:
        return None, f"CasADi 대조 생략: {type(e).__name__}"
    plant = AxialDronePlant(VP, dt=dt)
    n_h = math.sqrt(p["mass"] * p["g"] / (4.0 * p["k_T"]))
    u = [n_h * 1.03, n_h * 0.98, n_h * 1.01, n_h * 0.99]
    w = [3.0, -1.0, 0.5]
    xs = hover_state(p)
    xc = np.array(xs)
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
    ap.add_argument("--dt", type=float, default=0.002)
    ap.add_argument("--steps", type=int, default=400)
    a = ap.parse_args()

    p = params()
    n_h = math.sqrt(p["mass"] * p["g"] / (4.0 * p["k_T"]))

    # 일부러 비대칭인 입력. 네 로터가 다 다르고 바람도 비스듬해야
    # 부호가 틀린 항이 숨지 못한다.
    u = [n_h * 1.03, n_h * 0.98, n_h * 1.01, n_h * 0.99]
    w = [3.0, -1.0, 0.5]

    x = hover_state(p)
    traj = [list(x)]
    for _ in range(a.steps):
        x = rk4_step(x, u, w, p, a.dt)
        traj.append(list(x))

    # 제어 할당 [T_total, Mx, My, Mz] -> [T1..T4].
    # JS 에서 4x4 역행렬을 풀 필요 없게 여기서 미리 구해 넘긴다.
    from control.dynamics import compute_allocation_matrix   # noqa: E402
    _A, A_inv = compute_allocation_matrix(VP)
    alloc_inv = [[float(v) for v in row] for row in A_inv]

    worst, note = crosscheck(p, a.dt, a.steps)
    ref = {"dt": a.dt, "steps": a.steps, "u": u, "w": w,
           "x0": hover_state(p), "traj": traj,
           "n_hover": n_h,
           "casadi_max_diff": worst, "casadi_note": note,
           "alloc_inv": alloc_inv,
           "params": p}

    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(ref, separators=(",", ":")), encoding="utf-8")
    print(f"저장 {out}  ({out.stat().st_size / 1024:.0f} KB)")
    print(f"  dt {a.dt} s × {a.steps} 스텝 = {a.dt * a.steps:.2f} s")
    print(f"  호버 로터속도 {n_h:.2f} rad/s")
    print(f"  마지막 상태  pos ({x[0]:+.4f} {x[1]:+.4f} {x[2]:+.4f})  "
          f"vel ({x[3]:+.4f} {x[4]:+.4f} {x[5]:+.4f})")
    if worst is None:
        print(f"  {note}")
    else:
        ok = worst < 1e-9
        print(f"  {'✅' if ok else '❌'} CasADi 원본과 최대차 {worst:.3e}")
        if not ok:
            print("     순수 파이썬 포트가 원본과 어긋납니다. JS 로 넘기기 전에 고쳐야 합니다.")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
