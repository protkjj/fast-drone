"""GSLQR (표5) 이득 스케줄 사전계산 — 식(40)-(42).

속도별 수평 트림점(LevelTrimAudit, 선정 프로파일에서 이미 검증됨)에서
17상태 예측모델(functions["full"], M17과 동일 소스)을 CasADi로 선형화하고,
쿼터니언을 국소 회전벡터로 축소한 14차원 오차상태 ζ=[δz,δv(3),δφ(3),δω(3),δn(4)]
에서 연속시간 ARE를 풀어 K_j(4x14)를 얻는다. 런타임(runtime.js의 GSLQR
클래스)은 이 표를 **지령(reference) 속도**로 선형보간(식42)한다.

⚠ "실시간 속도로 보간"이라고 적혀 있었으나 코드는 `ref[0]`(지령)을 쓴다. 코드가
아니라 이 문구가 틀린 것이었다 — 2026-09-22에 실측으로 확인했다(V=8~18, ramp):

    보간 입력            V=8            V=12           V=13 이상
    지령(현재 구현)      v=0.67         v=4.24         FAIL
    실측 속도            v=8.00(정지)   v=12.0(정지)   v=13.0(정지)
    이득만 실측/목표는 지령  v=9.10       v=8.83         FAIL

`x_trim`이 **선형화점이자 동시에 목표점**이라 실측 속도로 보간하면 목표가
현재 상태를 따라가 버려 오차가 0이 되고, 기체가 아예 가속하지 않는다(위 표의
"정지" = RMSE가 지령 전체와 같음). 이득만 실측으로 떼어내도 더 나빠진다.
표준 이득 스케줄링 관례(스케줄 변수는 실측)를 그대로 적용할 수 없는 구조이므로,
문구를 코드에 맞춘다. 고속 실패의 원인은 보간 입력이 아니라 램프 중 자세
오버슈트다(V=13에서 기울기가 수평을 넘어 70°까지 감 -> 추력이 낙하를 가속).

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


# 튜닝 2026-09-22 — research/tune_gains.py, 논문 §5.3 프로토콜(독립 시나리오·동일
# 예산 120평가·본시험 비참조). 이력·선택 규칙은 results/tuned_gains.json.
# 튜닝 전 placeholder: Q diag [100,10,10,20, 50,50,50, 5,5,5, .01x4], R 0.05I.
#
# 개정3 채택값. 개정1(q_v_h x64)과 개정2(q_z x128)가 각각 한 축만 찾고 **그 조합을
# 한 번도 방문하지 못한 것**이 나침반 탐색의 경로의존성 때문이었다 — 출발점만
# 기존 선택값으로 옮겨 같은 절차·같은 예산으로 다시 돌리자 조합을 찾았다.
# 튜닝 세트 점수 8.427 -> 0.854(9.9배), 실패 0/6, z RMSE 6.240 -> 0.347.
# 고도 추종이 나빴던 것은 GSLQR 구조 한계(적분기 부재)가 아니라 가중치 문제였다 —
# 근거는 HANDOFF "GSLQR 고도추종 조사" 절(램프 중 과도손실이지 정적 편차가 아님).
# §5.3 이 본시험을 보고 가중치를 다시 고르는 것을 금지하므로 손으로 고치지 말 것.
TUNED_Q_DIAG = [6400, 1280, 1280, 20, 50, 50, 50, 5, 5, 5, .01, .01, .01, .01]
TUNED_R_SCALE = .00625


def design(A_r, B_r, Q=None, R=None):
    if Q is None:
        Q = np.diag(TUNED_Q_DIAG)
    if R is None:
        R = np.eye(4) * TUNED_R_SCALE
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
