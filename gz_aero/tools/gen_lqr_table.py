"""로켓 배치 오차상태 LQR 게인표를 구워 브라우저에 심는다.

왜 표인가. 브라우저에는 solver 가 없다. ARE(대수 리카티)도 CasADi 야코비안도
런타임에 못 돈다. 그런데 LQR 은 게인이 **설계 시점에 한 번** 정해지는
제어기라, 속도별로 미리 풀어 표로 넘기면 런타임에는 보간과 행렬곱만 남는다.
그래서 이 저장소에서 브라우저로 옮길 수 있는 유일한 '최적제어' 축이다.

무엇이 나오나. 속도 격자마다
    x_trim(17)  u_trim(4)  K_r(4x14)  maxRe
를 담고, JS 가 그대로 못 읽었는지 보려고 **기준 샘플**도 같이 담는다.
샘플은 격자점과 격자 사이(보간 검사) 양쪽에서 뽑는다.

한계를 먼저 적는다. 로켓 배치는 **52 m/s 위에 정상비행 트림이 없다**
(후방 로터가 음추력을 요구한다). 그래서 표는 0~50 m/s = 180 km/h 까지다.
그 위에서는 마지막 게인을 유지하는데, 그건 '빠르게 못 난다' 를 숨기는 것이
아니라 **기체에 그 속도의 평형점이 없다**는 사실을 그대로 드러내는 것이다.
"""
import json
import pathlib
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore")

# 형제 도구(gen_sim_reference.py)와 같은 방식으로 저장소 뿌리를 잡는다.
ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from control.vehicle_params import rocket_params
from control.trim import find_trim
from control.controller import LQRController

V_STEP = 2.5
V_MAX = 85.0      # 동체 90 mm 로 트림 상한이 84 m/s 까지 열렸다
RES_TOL = 1e-6          # 트림 잔차 상한. 이보다 크면 평형점이 아니다.
OUT = ROOT / "gz_aero" / "data" / "lqr_rocket.json"


def build_table():
    rows = []
    V, guess = 0.0, None
    while V <= V_MAX + 1e-9:
        # ★ 연속법. 냉시동 fsolve 는 고속에서 엉뚱한 가지로 빠진다.
        tr = find_trim(rocket_params, V, guess=guess, quiet=True)
        res = float(tr["residual"])
        if tr.get("converged"):
            guess = tr["guess"]
        x, u = np.asarray(tr["state"], float), np.asarray(tr["control"], float)
        if res > RES_TOL:
            print(f"  V={V:5.1f}  잔차 {res:.2e} > {RES_TOL:.0e} — 평형점이 아니라 버립니다")
            V += V_STEP
            continue
        lq = LQRController(rocket_params, x, u)
        if not lq.valid or lq.max_real >= 0.0:
            print(f"  V={V:5.1f}  LQR valid={lq.valid} maxRe={lq.max_real:.4f} — 버립니다")
            V += V_STEP
            continue
        rows.append({
            "V": float(V), "res": res, "maxRe": float(lq.max_real),
            "x": x.tolist(), "u": u.tolist(),
            "K": np.asarray(lq.K_r, float).tolist(),
        })
        V += V_STEP
    return rows


def interp(rows, V):
    """표에서 V 에 해당하는 (x_trim, u_trim, K). JS 와 **같은 식**이어야 한다."""
    Vs = [r["V"] for r in rows]
    if V <= Vs[0]:
        r = rows[0]
        return np.array(r["x"]), np.array(r["u"]), np.array(r["K"])
    if V >= Vs[-1]:
        r = rows[-1]
        return np.array(r["x"]), np.array(r["u"]), np.array(r["K"])
    i = 0
    while i + 1 < len(Vs) and Vs[i + 1] < V:
        i += 1
    a, b = rows[i], rows[i + 1]
    t = (V - a["V"]) / (b["V"] - a["V"])
    x = np.array(a["x"]) + t * (np.array(b["x"]) - np.array(a["x"]))
    # 쿼터니언은 선형 보간 후 정규화. 격자가 2.5 m/s 라 이웃 사이 자세차가
    # 몇 도뿐이고, 그 정도면 slerp 와 선형이 사실상 같다.
    q = x[6:10]
    n = np.linalg.norm(q)
    if n > 1e-12:
        x[6:10] = q / n
    u = np.array(a["u"]) + t * (np.array(b["u"]) - np.array(a["u"]))
    K = np.array(a["K"]) + t * (np.array(b["K"]) - np.array(a["K"]))
    return x, u, K


def quat_mult(p, q):
    return np.array([
        p[3]*q[0] + p[0]*q[3] + p[1]*q[2] - p[2]*q[1],
        p[3]*q[1] - p[0]*q[2] + p[1]*q[3] + p[2]*q[0],
        p[3]*q[2] + p[0]*q[1] - p[1]*q[0] + p[2]*q[3],
        p[3]*q[3] - p[0]*q[0] - p[1]*q[1] - p[2]*q[2]])


def law(rows, V, x, z_ref):
    """controller.py 의 LQRController 와 같은 제어법. 고도만 명령값을 쓴다."""
    xt, ut, K = interp(rows, V)
    q, qt = np.asarray(x[6:10], float), xt[6:10]
    qti = np.array([-qt[0], -qt[1], -qt[2], qt[3]])
    dq = quat_mult(qti, q)
    if dq[3] < 0.0:
        dq = -dq
    dx = np.concatenate([
        [x[2] - z_ref],
        np.asarray(x[3:6], float) - xt[3:6],
        2.0 * dq[0:3],
        np.asarray(x[10:13], float) - xt[10:13],
        np.asarray(x[13:17], float) - xt[13:17],
    ])
    u = ut - K @ dx
    return np.clip(u, rocket_params["n_min"], rocket_params["n_max"]), dx


def main():
    print("로켓 배치 LQR 게인표")
    rows = build_table()
    if not rows:
        print("표가 비었습니다."); return 1
    print(f"  격자 {len(rows)} 점  ({rows[0]['V']:.1f} ~ {rows[-1]['V']:.1f} m/s"
          f" = {rows[-1]['V']*3.6:.0f} km/h)")
    print(f"  트림 잔차 최대 {max(r['res'] for r in rows):.2e}")
    print(f"  폐루프 최대 실수부 {max(r['maxRe'] for r in rows):.4f}  (< 0 이어야 안정)")

    # 기준 샘플. 격자점과 격자 **사이** 양쪽에서 뽑아 보간까지 검사한다.
    rng = np.random.default_rng(20260915)
    samples = []
    for k in range(16):
        V = rows[0]["V"] + rng.random() * (rows[-1]["V"] - rows[0]["V"])
        if k % 2 == 0:                      # 절반은 격자점에 정확히 올린다
            V = rows[rng.integers(len(rows))]["V"]
        xt, _, _ = interp(rows, V)
        x = xt.copy()
        x[2] += rng.normal(0, 20)           # 고도 오차
        x[3:6] += rng.normal(0, 3, 3)
        dphi = rng.normal(0, 0.15, 3)       # 자세 교란
        dq = np.concatenate([dphi / 2.0, [1.0]])
        dq /= np.linalg.norm(dq)
        x[6:10] = quat_mult(x[6:10], dq)
        x[10:13] += rng.normal(0, 0.5, 3)
        x[13:17] += rng.normal(0, 30, 4)
        z_ref = 200.0 + rng.normal(0, 30)
        u, _ = law(rows, V, x, z_ref)
        samples.append({"V": float(V), "x": x.tolist(),
                        "zref": float(z_ref), "u": u.tolist()})

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({
        "note": "로켓 배치 오차상태 LQR. 14D [dz, dv3, dphi3, domega3, dn4], u = u_trim - K dx",
        "V_step": V_STEP, "V_max_table": rows[-1]["V"],
        "n_min": float(rocket_params["n_min"]), "n_max": float(rocket_params["n_max"]),
        "rows": rows, "samples": samples,
    }), encoding="utf-8")
    print(f"  저장 {OUT}  ({OUT.stat().st_size/1024:.0f} KB)")
    print(f"  기준 샘플 {len(samples)} 개 (절반은 격자 사이 — 보간까지 검사)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
