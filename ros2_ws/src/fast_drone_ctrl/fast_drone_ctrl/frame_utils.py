"""
좌표계 변환: 우리 시뮬레이션 ↔ PX4/NED
==========================================

■ 우리 시뮬레이션 (dynamics.py 기준)
  관성: NWU — x=전방, y=좌측, z=상방
  동체: FRD — x=전방, y=우측, z=하방
  쿼터니언: scalar-last [qx, qy, qz, qw] (scipy 관례)
  호버: q=[1,0,0,0] → R=diag(1,-1,-1) → 180° about x
        (동체 z-down이 관성 -z=아래를 가리킴)

■ PX4 (px4_msgs 기준)
  관성: NED — x=북, y=동, z=하방
  동체: FRD — x=전방, y=우측, z=하방 (우리와 동일!)
  쿼터니언: scalar-first [w, x, y, z]
  호버: q=[1,0,0,0](scalar-first) = 항등 (동체=NED, z-down 일치)

■ 변환 핵심
  NWU → NED: [x, -y, -z] (= Rx(180°) 회전)
  동체 프레임: 동일 (FRD ↔ FRD)
  쿼터니언: R_ned = Rx(180°) @ R_nwu
            q_ned = q_Rx180 ⊗ q_nwu  (q_Rx180 = [1,0,0,0] scalar-last)
"""

import numpy as np


# ════════════════════════════════════════════════════
# 상수
# ════════════════════════════════════════════════════

# NWU → NED 변환 쿼터니언: 180° about x
# scalar-last: [sin(90°), 0, 0, cos(90°)] = [1, 0, 0, 0]
_Q_NWU_TO_NED = np.array([1.0, 0.0, 0.0, 0.0])


# ════════════════════════════════════════════════════
# 쿼터니언 연산
# ════════════════════════════════════════════════════

def quat_multiply(p, q):
    """
    Hamilton product p ⊗ q, scalar-last [x, y, z, w].

    결과 쿼터니언은 "먼저 q 회전, 그 다음 p 회전"을 의미.
    """
    px, py, pz, pw = p
    qx, qy, qz, qw = q
    return np.array([
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
        pw*qw - px*qx - py*qy - pz*qz,
    ])


def quat_conjugate(q):
    """쿼터니언 켤레 (역회전). scalar-last [x, y, z, w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_scalar_last_to_first(q):
    """[x, y, z, w] → [w, x, y, z] (우리 → PX4)."""
    return np.array([q[3], q[0], q[1], q[2]])


def quat_scalar_first_to_last(q):
    """[w, x, y, z] → [x, y, z, w] (PX4 → 우리)."""
    return np.array([q[1], q[2], q[3], q[0]])


# ════════════════════════════════════════════════════
# 프레임 변환: 우리 ↔ PX4
# ════════════════════════════════════════════════════

def pos_nwu_to_ned(pos_nwu):
    """위치 NWU → NED: [x, y, z] → [x, -y, -z]."""
    return np.array([pos_nwu[0], -pos_nwu[1], -pos_nwu[2]])


def pos_ned_to_nwu(pos_ned):
    """위치 NED → NWU: [x, y, z] → [x, -y, -z]. (자기역원)"""
    return np.array([pos_ned[0], -pos_ned[1], -pos_ned[2]])


def vel_nwu_to_ned(vel_nwu):
    """속도 NWU → NED. 위치와 동일한 변환."""
    return pos_nwu_to_ned(vel_nwu)


def vel_ned_to_nwu(vel_ned):
    """속도 NED → NWU."""
    return pos_ned_to_nwu(vel_ned)


def quat_nwu_to_ned(q_nwu):
    """
    쿼터니언 NWU → NED (둘 다 scalar-last).

    R_ned = R_nwu_to_ned @ R_nwu
    q_ned = q_Rx180 ⊗ q_nwu

    여기서 q_Rx180 = [1, 0, 0, 0] (scalar-last, 180° about x).
    """
    return quat_multiply(_Q_NWU_TO_NED, q_nwu)


def quat_ned_to_nwu(q_ned):
    """
    쿼터니언 NED → NWU (둘 다 scalar-last).

    R_nwu = R_ned_to_nwu @ R_ned
    q_nwu = q_Rx180_inv ⊗ q_ned = q_Rx180_conj ⊗ q_ned

    Rx(180°)의 역 = Rx(-180°) = Rx(180°) (자기역원).
    → 같은 쿼터니언 적용.
    """
    return quat_multiply(_Q_NWU_TO_NED, q_ned)


# ════════════════════════════════════════════════════
# 복합 변환: PX4 메시지 ↔ 우리 상태 벡터
# ════════════════════════════════════════════════════

def px4_to_state(pos_ned, vel_ned, q_ned_sf, omega_body, motor_speeds):
    """
    PX4 메시지 데이터 → 우리 상태 벡터 x(17).

    Parameters
    ----------
    pos_ned : array(3)
        NED 위치 [m].
    vel_ned : array(3)
        NED 속도 [m/s].
    q_ned_sf : array(4)
        PX4 쿼터니언 [w, x, y, z] (scalar-first, body→NED).
    omega_body : array(3)
        동체 각속도 [rad/s] (FRD, 우리와 동일).
    motor_speeds : array(4)
        로터 속도 [rad/s].

    Returns
    -------
    x : array(17)
        [pos(3), vel(3), quat(4), omega(3), motors(4)]
        우리 좌표계 (NWU, scalar-last).
    """
    pos_nwu = pos_ned_to_nwu(pos_ned)
    vel_nwu = vel_ned_to_nwu(vel_ned)

    # PX4 quaternion: scalar-first → scalar-last → NED→NWU
    q_ned_sl = quat_scalar_first_to_last(q_ned_sf)
    q_nwu = quat_ned_to_nwu(q_ned_sl)

    # 쿼터니언 부호 정규화 (w > 0 관례)
    if q_nwu[3] < 0:
        q_nwu = -q_nwu

    return np.concatenate([pos_nwu, vel_nwu, q_nwu, omega_body, motor_speeds])


def state_to_px4_quat(q_nwu):
    """
    우리 쿼터니언 → PX4 쿼터니언 (scalar-first).

    q_nwu: scalar-last [x,y,z,w] (body→NWU)
    반환: scalar-first [w,x,y,z] (body→NED)
    """
    q_ned_sl = quat_nwu_to_ned(q_nwu)
    return quat_scalar_last_to_first(q_ned_sl)


def motor_speed_to_normalized(motor_speeds, n_max):
    """
    모터 속도 [rad/s] → PX4 정규화 [0, 1].

    PX4의 ActuatorMotors 메시지는 [-1, 1] 범위지만,
    일반 모터는 [0, 1]만 사용.
    """
    return np.clip(motor_speeds / n_max, 0.0, 1.0)


def normalized_to_motor_speed(normalized, n_max):
    """PX4 정규화 [0, 1] → 모터 속도 [rad/s]."""
    return np.clip(normalized, 0.0, 1.0) * n_max


# ════════════════════════════════════════════════════
# 검증 함수
# ════════════════════════════════════════════════════

def verify_conversions():
    """
    좌표 변환 왕복 검증.

    이 함수를 Ubuntu에서 한 번 실행해서
    변환이 올바른지 확인하기.
    """
    print("=== 좌표 변환 검증 ===\n")

    # 1. 위치 왕복
    pos = np.array([10.0, 5.0, 50.0])
    pos_rt = pos_ned_to_nwu(pos_nwu_to_ned(pos))
    assert np.allclose(pos, pos_rt), f"위치 왕복 실패: {pos} → {pos_rt}"
    print(f"[OK] 위치 왕복: {pos} → NED {pos_nwu_to_ned(pos)} → NWU {pos_rt}")

    # 2. 쿼터니언 왕복
    q = np.array([0.1, 0.2, 0.3, 0.9])
    q = q / np.linalg.norm(q)
    q_rt = quat_ned_to_nwu(quat_nwu_to_ned(q))
    # q와 -q는 같은 회전
    if q_rt[3] < 0:
        q_rt = -q_rt
    if q[3] < 0:
        q = -q
    assert np.allclose(q, q_rt, atol=1e-10), f"쿼터니언 왕복 실패: {q} → {q_rt}"
    print(f"[OK] 쿼터니언 왕복: {q[:2]}... → {q_rt[:2]}...")

    # 3. 호버 쿼터니언 변환
    q_hover_nwu = np.array([1.0, 0.0, 0.0, 0.0])  # 우리 호버: Rx(180°)
    q_hover_ned_sl = quat_nwu_to_ned(q_hover_nwu)
    q_hover_ned_sf = quat_scalar_last_to_first(q_hover_ned_sl)
    # PX4 호버 = 항등 쿼터니언 = [1, 0, 0, 0] (scalar-first)
    # 또는 [-1, 0, 0, 0] (같은 회전)
    expected = np.array([1.0, 0.0, 0.0, 0.0])
    sign = np.sign(q_hover_ned_sf[0]) if abs(q_hover_ned_sf[0]) > 0.5 else 1.0
    q_check = sign * q_hover_ned_sf
    assert np.allclose(abs(q_check), expected, atol=1e-10), \
        f"호버 변환 실패: NWU {q_hover_nwu} → NED(sf) {q_hover_ned_sf}"
    print(f"[OK] 호버: NWU {q_hover_nwu} → NED(sf) {q_hover_ned_sf}")

    # 4. 모터 정규화 왕복
    n = np.array([500.0, 600.0, 500.0, 600.0])
    n_max = 1800.0
    n_rt = normalized_to_motor_speed(motor_speed_to_normalized(n, n_max), n_max)
    assert np.allclose(n, n_rt), f"모터 정규화 왕복 실패"
    print(f"[OK] 모터 정규화 왕복: {n} → norm {motor_speed_to_normalized(n, n_max)} → {n_rt}")

    print("\n=== 전체 통과 ===")


if __name__ == '__main__':
    verify_conversions()
