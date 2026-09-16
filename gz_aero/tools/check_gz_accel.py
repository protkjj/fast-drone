#!/usr/bin/env python3
"""겹2b — 적용점 검증. "건 wrench 가 정말 그 가속도를 만들었나"

    gz sim -s -r --iterations 400 gz_aero/test/accel_check.world
    python3 gz_aero/tools/check_gz_accel.py

무엇이 걸려 있나
----------------
플러그인은 힘을 `AddWorldForce`(무게중심)로, 모멘트를 `AddWorldWrench(0, M)`
(순수 우력)로 건다. 근거는 이렇다:

  gz-sim 의 ExternalWorldWrenchCmd 는 힘을 **링크 원점**에 건다. 우리 모멘트는
  x_cp 가 CG 기준이라 **무게중심 기준**이다. AddWorldForce 가 원점↔CG 보정을
  내부에서 해 주고, 우력은 자유벡터라 적용점이 아예 의미가 없다.

이건 gz-sim 소스를 읽고 내린 판단이지 실측이 아니다. 틀렸다면 실제 토크에
`r_cm x F` 가 여분으로 붙는다. 그래서 이 시험이 필요하다.

어떻게 가르나
-------------
`accel_check.world` 의 프로브는 **무게중심을 링크 원점에서 일부러 떼어 놨다.**
그러면 두 가설이 서로 다른 각가속도를 만든다:

    옳다면   J w_dot + w x Jw = M_L
    틀렸다면 J w_dot + w x Jw = M_L + (-r_cm) x F_L

측정한 각가속도가 어느 쪽에 붙는지 보면 된다. 무게중심이 원점에 있으면
`r_cm x F = 0` 이라 두 가설이 **구분이 안 된다** — 그래서 프로브를 따로 뒀다.

판정은 "절대오차가 작다" 가 아니라 **"틀린 가설보다 옳은 가설에 훨씬 가깝다"** 로
한다. 수치미분 오차가 얼마든 이 비교는 성립한다.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from check_gz_frames import read_log, quat_to_R      # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log-dir", default="/tmp/fast_drone_aero")
    ap.add_argument("--pattern", default="q*.csv")
    ap.add_argument("--skip", type=int, default=20,
                    help="초반 몇 행을 버릴지 (컴포넌트 초기화 과도구간)")
    ap.add_argument("--ratio", type=float, default=0.1,
                    help="옳은 가설의 잔차가 판별력의 이 비율보다 작아야 통과")
    args = ap.parse_args(argv)

    logs = sorted(glob.glob(os.path.join(args.log_dir, args.pattern)))
    if not logs:
        print(f"로그가 없습니다: {args.log_dir}/{args.pattern}")
        print("gen_gz_test_world.py 로 accel_check.world 를 만들고 gz sim 을 돌리세요.")
        return 2

    n_fail = 0
    print(f"{'probe':<16} {'rows':>5} | {'옳은가설 잔차':>13} {'틀린가설 잔차':>13} "
          f"{'판별력':>10} | {'선형 a':>10}")
    print("-" * 84)

    for path in logs:
        meta, c, R = read_log(path)
        if len(R) < args.skip + 5:
            print(f"{os.path.basename(path):<16} {len(R):>5} | 행이 너무 적습니다")
            n_fail += 1
            continue

        m = float(meta["mass"])
        r_cm = np.array([float(x) for x in meta["r_cm"].split()])
        Jd = np.array([float(x) for x in meta["J_diag"].split()])
        Jo = np.array([float(x) for x in meta["J_off"].split()])
        J = np.array([[Jd[0], Jo[0], Jo[1]],
                      [Jo[0], Jd[1], Jo[2]],
                      [Jo[1], Jo[2], Jd[2]]])
        Jinv = np.linalg.inv(J)

        t = R[:, c["t"]]
        res_ok, res_bad, discr, res_lin = [], [], [], []

        for k in range(args.skip, len(R) - 1):
            if t[k + 1] - t[k - 1] <= 0:
                continue
            dt2 = t[k + 1] - t[k - 1]

            def Rm(i):
                return quat_to_R(R[i, c["qx"]], R[i, c["qy"]],
                                 R[i, c["qz"]], R[i, c["qw"]])

            # 각속도를 링크 좌표로. d(w_L)/dt = R^T (dw_W/dt) 이므로 중앙차분이 그대로 맞다
            wL = [Rm(i).T @ R[i, [c["oLx"], c["oLy"], c["oLz"]]]
                  for i in (k - 1, k, k + 1)]
            wdot_meas = (wL[2] - wL[0]) / dt2

            Rk = Rm(k)
            M_L = Rk.T @ R[k, [c["mWx"], c["mWy"], c["mWz"]]]
            F_L = Rk.T @ R[k, [c["fWx"], c["fWy"], c["fWz"]]]
            w = wL[1]

            gyro = np.cross(w, J @ w)
            wdot_ok = Jinv @ (M_L - gyro)
            wdot_bad = Jinv @ (M_L + np.cross(-r_cm, F_L) - gyro)

            res_ok.append(np.linalg.norm(wdot_meas - wdot_ok))
            res_bad.append(np.linalg.norm(wdot_meas - wdot_bad))
            discr.append(np.linalg.norm(Jinv @ np.cross(r_cm, F_L)))

            # 선형: a_cm = F/m (적용점과 무관하지만 힘의 크기·부호를 검증한다)
            # v_cm = v_origin + w_W x (R r_cm)
            vcm = []
            for i in (k - 1, k + 1):
                Ri = Rm(i)
                wW = R[i, [c["oLx"], c["oLy"], c["oLz"]]]
                vcm.append(R[i, [c["vLx"], c["vLy"], c["vLz"]]]
                           + np.cross(wW, Ri @ r_cm))
            a_meas = (vcm[1] - vcm[0]) / dt2
            a_exp = R[k, [c["fWx"], c["fWy"], c["fWz"]]] / m
            res_lin.append(np.linalg.norm(a_meas - a_exp)
                           / max(np.linalg.norm(a_exp), 1e-9))

        if not res_ok:
            print(f"{os.path.basename(path):<16} {len(R):>5} | 비교할 구간 없음")
            n_fail += 1
            continue

        ok_m = float(np.median(res_ok))
        bad_m = float(np.median(res_bad))
        dis_m = float(np.median(discr))
        lin_m = float(np.median(res_lin))
        passed = (dis_m > 1e-6) and (ok_m < args.ratio * dis_m) and (lin_m < 0.05)
        if not passed:
            n_fail += 1
        print(f"{os.path.basename(path):<16} {len(res_ok):>5} | "
              f"{ok_m:>13.4g} {bad_m:>13.4g} {dis_m:>10.4g} | "
              f"{lin_m:>10.3%}  {'OK' if passed else 'FAIL'}")

    print("-" * 84)
    print("잔차 단위 rad/s^2 (중앙값). 판별력 = |J^-1 (r_cm x F)| — 두 가설의 차이 크기.")
    if n_fail == 0:
        print("✅ 겹2b 통과 — 힘이 무게중심에 걸리고 모멘트가 우력으로 걸립니다")
        print("   (옳은가설 잔차 << 틀린가설 잔차 이므로 수치미분 오차와 무관하게 구분됩니다)")
    else:
        print(f"❌ {n_fail}개 실패")
        print("   옳은가설 잔차 ~ 판별력 이면 -> 힘이 링크 원점에 걸리고 있습니다.")
        print("     AddWorldForce 대신 r_cm x F 를 직접 토크에 더하도록 고쳐야 합니다.")
        print("   선형 a 만 틀리면 -> WorldLinearVelocity 가 원점이 아니라 무게중심")
        print("     속도일 수 있습니다 (AeroPlugin.cc 의 v_cm 보정 재확인).")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
