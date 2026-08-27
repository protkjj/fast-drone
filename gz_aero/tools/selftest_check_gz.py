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
        for name, pose, vel, omega, wind in PROBES:
            fabricate(os.path.join(tmp, f"{name}.csv"), table_path, src,
                      pose[3:6], vel, omega, wind)
        print(f"가짜 로그 {len(PROBES)}개 생성 -> 대조 스크립트 실행\n")
        r = subprocess.run(
            [sys.executable, os.path.join(_HERE, "check_gz_frames.py"),
             "--source", "sized", "--log-dir", tmp],
            env={**os.environ, "PYTHONPATH": _REPO})
    if r.returncode == 0:
        print("\n✅ 대조 도구 자체 검증 통과 — 이제 Ubuntu 결과를 플러그인 탓으로 읽어도 된다")
    else:
        print("\n❌ 대조 도구가 자기가 만든 로그도 못 맞춥니다. 도구부터 고쳐야 합니다")
    return r.returncode


if __name__ == "__main__":
    raise SystemExit(main())
