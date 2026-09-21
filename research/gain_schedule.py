"""GSLQR (표5) 이득 스케줄 사전계산 — 식(40)-(42).

속도별 수평 트림점(LevelTrimAudit, 선정 프로파일에서 이미 검증됨)에서
17상태 예측모델(functions["full"], M17과 동일 소스)을 CasADi로 선형화하고,
쿼터니언을 국소 회전벡터로 축소한 14차원 오차상태 ζ=[δz,δv(3),δφ(3),δω(3),δn(4)]
에서 연속시간 ARE를 풀어 K_j(4x14)를 얻는다. 런타임(runtime.js의 GSLQR
클래스)은 이 표를 실시간 속도로 선형보간(식42)한다.

control/controller.py의 linearize_error_state와 같은 축소 방식이지만,
여기선 트림 자체가 이미 thrust_axis=+x(선정 프로파일)에 맞게 검증돼 있어
control/trim.py를 다시 쓰지 않는다(find_trim은 z축 하드코딩, 로켓 배치에서
수렴 실패 기록이 있음 — lqr-fallback-thrust-axis-gap 메모 참고).
"""
import numpy as np
import casadi as ca
from scipy.linalg import solve_continuous_are

from research.model import build, initial_state
from research.trim_envelope import LevelTrimAudit

NR = 14  # 축소 오차상태 차원: dz(1)+dv(3)+dphi(3)+domega(3)+dn(4)


def _quat_left_mult_matrix(q):
    """q ⊗ p = Q_L(q) @ p, scalar-last [x,y,z,w]. control/controller.py와 동일."""
    qx, qy, qz, qw = q
    return np.array([
        [qw, -qz, qy, qx],
        [qz, qw, -qx, qy],
        [-qy, qx, qw, qz],
        [-qx, -qy, -qz, qw]])


def trim_point(audit, speed):
    """LevelTrimAudit.evaluate()의 결과에서 (x_trim[18], u_trim[4], voltage)를 재구성."""
    result = audit.evaluate(speed, soc=1.0)
    if not result["model_feasible"]:
        raise ValueError(f"V={speed} m/s 트림이 model_feasible=False: {result['rejection_reasons']}")
    theta = np.deg2rad(result["theta_deg"])
    n = np.array(result["rotor_rpm"]) * 2 * np.pi / 60
    x = audit.state(speed, theta, 1.0)
    x[13:17] = n
    return x, n, result["bus_voltage_V"]


def linearize(f, x18, n, voltage):
    """functions["full"](x17, cmd, wind, bus_v)를 트림점에서 선형화 → (A17, B17)."""
    xm = ca.SX.sym("xm", 17)
    cmd = ca.SX.sym("cmd", 4)
    wind = ca.SX.sym("wind", 3)
    bus = ca.SX.sym("bus")
    rhs = f["full"](xm, cmd, wind, bus)
    A_fn = ca.Function("A", [xm, cmd, wind, bus], [ca.jacobian(rhs, xm)])
    B_fn = ca.Function("B", [xm, cmd, wind, bus], [ca.jacobian(rhs, cmd)])
    x17 = x18[:17]
    A17 = np.asarray(A_fn(x17, n, [0, 0, 0], voltage)).astype(float)
    B17 = np.asarray(B_fn(x17, n, [0, 0, 0], voltage)).astype(float)
    return A17, B17


def reduce(A17, B17, q_trim):
    """17x17/17x4 -> 14x14/14x4 오차상태 축소 (control/controller.py와 동일 패턴)."""
    Q_L = _quat_left_mult_matrix(q_trim)
    dq_dphi = 0.5 * Q_L[:, 0:3]
    T = np.zeros((17, NR))
    T[2, 0] = 1.0
    T[3:6, 1:4] = np.eye(3)
    T[6:10, 4:7] = dq_dphi
    T[10:13, 7:10] = np.eye(3)
    T[13:17, 10:14] = np.eye(4)
    T_pinv = np.linalg.pinv(T)
    A_r = T_pinv @ A17 @ T
    B_r = T_pinv @ B17
    return A_r, B_r, T_pinv


def design(A_r, B_r, Q=None, R=None):
    if Q is None:
        Q = np.diag([100, 10, 10, 20, 50, 50, 50, 5, 5, 5, .01, .01, .01, .01])
    if R is None:
        R = np.eye(4) * .05
    P = solve_continuous_are(A_r, B_r, Q, R)
    K_r = np.linalg.inv(R) @ B_r.T @ P
    eig = np.linalg.eigvals(A_r - B_r @ K_r)
    return K_r, float(np.max(np.real(eig)))


def build_schedule(p, speeds=None):
    """반환: JSON 직렬화 가능한 dict. 실패한 속도점은 건너뛰고 기록한다."""
    speeds = [0, 10, 20, 30, 40, 50, 60, 70, 83.3] if speeds is None else list(speeds)
    audit = LevelTrimAudit(p)
    f = build(p)
    rows, skipped = [], []
    for v in speeds:
        try:
            x18, n, voltage = trim_point(audit, v)
        except ValueError as e:
            skipped.append({"speed_mps": v, "reason": str(e)})
            continue
        A17, B17 = linearize(f, x18, n, voltage)
        A_r, B_r, _ = reduce(A17, B17, x18[6:10])
        K_r, max_real = design(A_r, B_r)
        if max_real >= -1e-6:
            skipped.append({"speed_mps": v, "reason": f"폐루프 불안정(max_real={max_real:.4f})"})
            continue
        rows.append({"speed_mps": float(v), "K_r": K_r.tolist(),
                     "x_trim": x18[:17].tolist(), "u_trim": n.tolist(),
                     "closed_loop_max_real": max_real})
    if not rows:
        raise RuntimeError("모든 속도점에서 GSLQR 설계 실패 — speeds 인자를 확인할 것")
    return {"nr": NR, "speeds_mps": [r["speed_mps"] for r in rows],
            "K_r": [r["K_r"] for r in rows], "x_trim": [r["x_trim"] for r in rows],
            "u_trim": [r["u_trim"] for r in rows], "skipped": skipped}
