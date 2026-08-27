#!/usr/bin/env python3
"""check_gz_frames.py 자체를 검증한다 — Gazebo 없이.

    python3 gz_aero/tools/selftest_check_gz.py

왜 필요한가: 겹2 대조 스크립트를 Ubuntu 에서 처음 돌려 보면, 실패했을 때
"플러그인이 틀렸나 대조 스크립트가 틀렸나" 를 또 가려야 한다. 그 왕복이 아깝다.
그래서 **플러그인이 낼 법한 로그를 여기서 만들어** 대조 스크립트를 먼저 태운다.

독립성: 회전은 scipy Rotation 으로 만든다. 대조 스크립트는 자기 손으로 쓴
quat_to_R 을 쓰므로, 둘이 어긋나면 A/E 단계에서 걸린다. 쿼터니언 규약 사고가
이 프로젝트에서 이미 한 번 났던 자리다 (scalar-last).

여기서 통과한다고 플러그인이 맞다는 뜻은 아니다. **대조 도구가 쓸 만하다**는
뜻이고, 그래야 Ubuntu 에서 나온 실패를 플러그인 탓으로 읽을 수 있다.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from check_gz_frames import Table, EPS          # noqa: E402
from gen_gz_test_world import PROBES            # noqa: E402
from aero_sources import get_source             # noqa: E402

HEADER_COLS = ("t,qx,qy,qz,qw,vaWx,vaWy,vaWz,owWx,owWy,owWz,"
               "u,v,w,p,q,r,V,V_cf,alpha,q_bar,C_A,C_N,x_cp,"
               "Fx,Fy,Fz,Mx,My,Mz,fWx,fWy,fWz,mWx,mWy,mWz")

NOSE = np.array([1.0, 0.0, 0.0])
RIGHT = np.array([0.0, -1.0, 0.0])


def fabricate(path, table_path, src, pose_rpy, vel_w, omega_w, wind_w, n_rows=40):
    """플러그인 형식 그대로의 디버그 로그를 만든다."""
    tab = Table(table_path)
    down = np.cross(NOSE, RIGHT)
    R_LB = np.column_stack([NOSE, RIGHT, down])
    S, d, rho = src.S_ref, src.d_ref, src.rho

    lines = [
        "# fast_drone aero debug log (fabricated by selftest_check_gz.py)",
        f"# table: {table_path}",
        f"# S_ref: {S:.17g}", f"# d_ref: {d:.17g}", f"# rho: {rho:.17g}",
        "# nose: 1 0 0", "# right: 0 -1 0", "# down: 0 0 -1",
        f"# wind: {wind_w[0]:.17g} {wind_w[1]:.17g} {wind_w[2]:.17g}",
        "# quaternion: scalar-last (qx,qy,qz,qw), 링크 -> 월드",
        HEADER_COLS,
    ]

    v_air_w = np.array(vel_w, float) - np.array(wind_w, float)
    om_w = np.array(omega_w, float)

    for i in range(n_rows):
        # 자세를 조금씩 굴린다 — 실제로도 기체가 텀블링하며 여러 자세를 지난다
        rot = (Rotation.from_euler("xyz", pose_rpy)
               * Rotation.from_rotvec(np.array([0.17, -0.23, 0.31]) * i * 0.05))
        q = rot.as_quat()                    # scipy 도 scalar-last
        Rm = rot.as_matrix()                 # 링크 -> 월드

        v_B = R_LB.T @ (Rm.T @ v_air_w)
        w_B = R_LB.T @ (Rm.T @ om_w)

        u_, v_, w_ = v_B
        vcf = math.hypot(v_, w_)
        Vs = u_*u_ + v_*v_ + w_*w_ + EPS
        V = math.sqrt(Vs)
        alpha = math.atan2(vcf, u_)
        qbar = 0.5 * rho * Vs
        CA, CN, xcp, Clp, Cmq = tab.lookup(V, alpha)

        FN = qbar * S * CN
        inv = 1.0 / vcf if vcf > 0 else 0.0
        F_B = np.array([-qbar * S * CA, -FN * v_ * inv, -FN * w_ * inv])
        df = 0.25 * rho * V * S * d * d
        M_B = np.array([df * Clp * w_B[0],
                        -xcp * F_B[2] + df * Cmq * w_B[1],
                        xcp * F_B[1] + df * Cmq * w_B[2]])
        f_W = Rm @ (R_LB @ F_B)
        m_W = Rm @ (R_LB @ M_B)

        vals = ([i * 0.005, q[0], q[1], q[2], q[3]] + list(v_air_w) + list(om_w)
                + list(v_B) + list(w_B) + [V, vcf, alpha, qbar, CA, CN, xcp]
                + list(F_B) + list(M_B) + list(f_W) + list(m_W))
        lines.append(",".join(f"{x:.17g}" for x in vals))

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    src = get_source("sized")
    table_path = os.path.join(_REPO, "gz_aero", "data", "aero_sized.csv")
    with tempfile.TemporaryDirectory() as tmp:
        for name, pose, vel, omega, wind, _com in PROBES:
            fabricate(os.path.join(tmp, f"{name}.csv"), table_path, src,
                      pose[3:6], vel, omega, wind)
        print(f"가짜 로그 {len(PROBES)}개 생성 -> 대조 스크립트 실행\n")
        r = subprocess.run(
            [sys.executable, os.path.join(_HERE, "check_gz_frames.py"),
             "--source", "sized", "--log-dir", tmp],
            env={**os.environ, "PYTHONPATH": _REPO})
    if r.returncode != 0:
        print("\n❌ 겹2a 대조 도구가 자기가 만든 로그도 못 맞춥니다. 도구부터 고쳐야 합니다")
        return r.returncode
    print("\n✅ 겹2a 대조 도구 검증 통과")
    return accel_selftest()


# ══════════════════════════════════════════════════════════════════════
# 겹2b 도구 자체 검증 — check_gz_accel.py 가 적용점 오류를 정말 잡는가
# ══════════════════════════════════════════════════════════════════════
#
# 통과만 확인하면 의미가 없다. "틀린 구현을 넣었을 때 실패하는가" 를 같이 봐야
# 시험에 판별력이 있다는 게 증명된다 (반대방향 시험).
#
# 여기서는 강체를 파이썬으로 굴린다. 두 가지 방식으로:
#   correct : 힘을 무게중심에, 모멘트를 우력으로     -> 플러그인이 하는 것
#   at_origin : 힘을 링크 원점에                     -> 흔한 실수
# 그리고 check_gz_accel.py 에 각각 먹여 본다. correct 는 통과, at_origin 은
# 실패해야 한다. 둘 다 통과하면 시험이 아무것도 못 가리는 것이다.

ACCEL_HEADER_COLS = HEADER_COLS + ",vLx,vLy,vLz,oLx,oLy,oLz"


def simulate(path, table_path, src, pose_rpy, vel_w, omega0_w, com, mass,
             J_diag, mode, n=200, dt=1e-3):
    tab = Table(table_path)
    down = np.cross(NOSE, RIGHT)
    R_LB = np.column_stack([NOSE, RIGHT, down])
    S, d, rho = src.S_ref, src.d_ref, src.rho
    J = np.diag(J_diag)
    Jinv = np.linalg.inv(J)
    r_cm = np.array(com, float)

    rot = Rotation.from_euler("xyz", pose_rpy)
    om_W = np.array(omega0_w, float)
    v_cm = np.zeros(3)
    v_air_w = np.array(vel_w, float)      # 주입 모드 + 바람 0

    lines = [
        "# fabricated by selftest_check_gz.py (accel)",
        f"# table: {table_path}",
        f"# S_ref: {S:.17g}", f"# d_ref: {d:.17g}", f"# rho: {rho:.17g}",
        "# nose: 1 0 0", "# right: 0 -1 0", "# down: 0 0 -1",
        "# wind: 0 0 0",
        f"# mass: {mass:.17g}",
        f"# r_cm: {r_cm[0]:.17g} {r_cm[1]:.17g} {r_cm[2]:.17g}",
        f"# J_diag: {J_diag[0]:.17g} {J_diag[1]:.17g} {J_diag[2]:.17g}",
        "# J_off: 0 0 0",
        "# quaternion: scalar-last (qx,qy,qz,qw), 링크 -> 월드",
        ACCEL_HEADER_COLS,
    ]

    for i in range(n):
        Rm = rot.as_matrix()
        q = rot.as_quat()
        v_B = R_LB.T @ (Rm.T @ v_air_w)
        w_B = R_LB.T @ (Rm.T @ om_W)

        u_, v_, w_ = v_B
        vcf = math.hypot(v_, w_)
        Vs = u_*u_ + v_*v_ + w_*w_ + EPS
        V = math.sqrt(Vs)
        alpha = math.atan2(vcf, u_)
        qbar = 0.5 * rho * Vs
        CA, CN, xcp, Clp, Cmq = tab.lookup(V, alpha)
        FN = qbar * S * CN
        inv = 1.0 / vcf if vcf > 0 else 0.0
        F_B = np.array([-qbar * S * CA, -FN * v_ * inv, -FN * w_ * inv])
        df = 0.25 * rho * V * S * d * d
        M_B = np.array([df * Clp * w_B[0],
                        -xcp * F_B[2] + df * Cmq * w_B[1],
                        xcp * F_B[1] + df * Cmq * w_B[2]])
        f_W = Rm @ (R_LB @ F_B)
        m_W = Rm @ (R_LB @ M_B)

        # 링크 원점 속도 = 무게중심 속도 - w x (R r_cm)
        v_link = v_cm - np.cross(om_W, Rm @ r_cm)
        vals = ([i * dt, q[0], q[1], q[2], q[3]] + list(v_air_w) + list(om_W)
                + list(v_B) + list(w_B) + [V, vcf, alpha, qbar, CA, CN, xcp]
                + list(F_B) + list(M_B) + list(f_W) + list(m_W)
                + list(v_link) + list(om_W))
        lines.append(",".join(f"{x:.17g}" for x in vals))

        # ── 적분 ──
        F_L = Rm.T @ f_W
        tau_L = Rm.T @ m_W
        if mode == "at_origin":
            # 힘이 링크 원점에 걸리면 무게중심 기준 토크에 (-r_cm) x F 가 붙는다
            tau_L = tau_L + np.cross(-r_cm, F_L)
        om_L = Rm.T @ om_W
        om_L = om_L + dt * (Jinv @ (tau_L - np.cross(om_L, J @ om_L)))
        rot = rot * Rotation.from_rotvec(om_L * dt)
        om_W = rot.as_matrix() @ om_L
        v_cm = v_cm + dt * f_W / mass

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def accel_selftest():
    from gen_gz_test_world import ACCEL_PROBES
    src = get_source("sized")
    table_path = os.path.join(_REPO, "gz_aero", "data", "aero_sized.csv")
    m = src.meta
    mass = m.get("MTOW", 1.660778087)
    Jd = [m.get("J_xx", 0.00604071), m.get("J_yy", 0.04045326),
          m.get("J_zz", 0.04045326)]

    results = {}
    for mode in ("correct", "at_origin"):
        with tempfile.TemporaryDirectory() as tmp:
            for name, pose, vel, omega, wind, com in ACCEL_PROBES:
                simulate(os.path.join(tmp, f"{name}.csv"), table_path, src,
                         pose[3:6], vel, omega, com, mass, Jd, mode)
            print(f"\n── 강체를 '{mode}' 방식으로 굴려서 검사 태우기 ──")
            r = subprocess.run(
                [sys.executable, os.path.join(_HERE, "check_gz_accel.py"),
                 "--log-dir", tmp],
                env={**os.environ, "PYTHONPATH": _REPO})
            results[mode] = r.returncode

    print()
    if results["correct"] == 0 and results["at_origin"] != 0:
        print("✅ 겹2b 검사에 판별력이 있습니다 "
              "— 옳은 구현은 통과, 원점 적용은 잡아냄")
        return 0
    if results["correct"] != 0:
        print("❌ 옳은 구현인데 실패했습니다. 검사가 너무 빡빡하거나 틀렸습니다")
    else:
        print("❌ 틀린 구현(원점 적용)도 통과했습니다. 검사에 판별력이 없습니다 "
              "— 이대로면 Ubuntu 에서 통과해도 아무 의미가 없습니다")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
