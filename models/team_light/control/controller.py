"""Controllers retained for the fixed pack_forward v2 comparison.

Extracted from the tested local research branch; no legacy demo entry points.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import solve_continuous_are
import casadi as ca
from models.team_light.control.geometry import (force_to_attitude, axial_speed, control_effectiveness,
    rotor_speeds_for_thrust, rotor_thrusts, allocate_wrench)
from models.team_light.control.propeller_curve import uses_curve


class CascadedPID:
    """
    캐스케이드 PID 속도/자세 제어기.

    외부: 속도 P + 고도 PID → 원하는 힘 F_des
    중간: F_des → 원하는 자세 R_des + 추력 T
    내부: 자세 PD + 자이로 FF → 모멘트 → 할당 → 모터속도
    """

    def __init__(self, params, v_ref=None, z_ref=0.0, heading=0.0, dt=0.001):
        self.p = params
        self.m = params['mass']
        self.g = params['g']
        self.dt = dt

        self.v_ref = np.array(v_ref) if v_ref is not None else np.zeros(3)
        self.z_ref = z_ref
        self.heading = heading

        # ── 외부 게인 ──
        self.Kp_vel = 1.0
        self.Kp_z   = 2.0
        self.Kd_z   = 2.0
        self.Ki_z   = 0.5       # 고도 적분 게인 (정상상태 오차 제거)
        self.int_z_max = 5.0    # 안티와인드업 한계

        # ── 내부 게인 ──
        self.Kp_att = np.array([200, 500, 500])
        self.Kd_att = np.array([20, 50, 50])

        self.max_tilt = np.radians(35)

        from models.team_light.control.dynamics import compute_allocation_matrix
        self.f_to_TM, self.TM_to_f = compute_allocation_matrix(params, static_reference=True)
        self.J = np.diag([params['Ixx'], params['Iyy'], params['Izz']])

        # 적분기 상태
        self._int_ez = 0.0

    def reset(self):
        """적분기 초기화."""
        self._int_ez = 0.0

    def __call__(self, t, x):
        pos, vel = x[0:3], x[3:6]
        q, omega = x[6:10], x[10:13]
        R = Rotation.from_quat(q).as_matrix()

        # ── 외부: 속도/고도 → 원하는 힘 ──
        e_vel = vel - self.v_ref
        e_z   = pos[2] - self.z_ref

        # 고도 적분기 + 안티와인드업
        self._int_ez += e_z * self.dt
        self._int_ez = np.clip(self._int_ez, -self.int_z_max, self.int_z_max)

        a_des = np.zeros(3)
        a_des[0:2] = -self.Kp_vel * e_vel[0:2]
        a_des[2]   = -self.Kp_z * e_z - self.Kd_z * vel[2] - self.Ki_z * self._int_ez

        F_des = self.m * (a_des + np.array([0, 0, self.g]))

        # ── 중간: 힘 → 추력 + 자세 ──
        T_cmd, R_des = self._force_to_attitude(F_des)

        # ── 내부: 자세 PD → 모멘트 ──
        S_err = 0.5 * (R_des.T @ R - R.T @ R_des)
        att_err = np.array([-S_err[1, 2], S_err[0, 2], -S_err[0, 1]])
        gyro_ff = np.cross(omega, self.J @ omega)
        M_cmd = self.J @ (-self.Kp_att * att_err - self.Kd_att * omega) + gyro_ff

        # ── 할당 → 모터속도 ──
        return self._allocate(T_cmd, M_cmd, R.T @ vel)

    def _force_to_attitude(self, F_des):
        """World force -> body +x attitude reference."""
        return force_to_attitude(F_des, self.heading, self.max_tilt)


    def _allocate(self, T_cmd, M_cmd, v_body=None):
        """[T, Mx, My, Mz] → 모터 속도."""
        TM = np.array([T_cmd, M_cmd[0], M_cmd[1], M_cmd[2]])
        if uses_curve(self.p):
            return allocate_wrench(self.p, TM, v_body)
        f_ind = self.TM_to_f @ TM
        return rotor_speeds_for_thrust(self.p, f_ind, v_body)


def _quat_left_mult_matrix(q):
    """
    쿼터니언 왼쪽 곱셈 행렬 Q_L: q ⊗ p = Q_L(q) @ p.
    q, p 모두 scalar-last [x,y,z,w].

    유도: (q⊗p)_x = qw·px - qz·py + qy·pz + qx·pw  등.
    """
    qx, qy, qz, qw = q
    return np.array([
        [ qw, -qz,  qy,  qx],
        [ qz,  qw, -qx,  qy],
        [-qy,  qx,  qw,  qz],
        [-qx, -qy, -qz,  qw]])


def linearize_error_state(params, x_trim, u_trim):
    """
    Error-State 선형화: 17D 풀 야코비안 → 14D 축소 시스템.

    축소 상태 δx_r(14):
      [δz(1), δv(3), δφ(3), δω(3), δn(4)]
      위치 x,y 제거 (속도 제어기라 불필요)
      쿼터니언 4 → 오차 각도 3 (δφ via 오차 쿼터니언 벡터부)

    변환: δx_full(17) = T(17×14) @ δx_reduced(14)
    축소: A_r = T⁺ A T,  B_r = T⁺ B
    """
    from models.team_light.control.dynamics import build_dynamics

    # 풀 야코비안 (17×17, 17×4)
    f, x_sym, u_sym = build_dynamics(params)
    xdot = f(x_sym, u_sym)
    A_fn = ca.Function('A', [x_sym, u_sym], [ca.jacobian(xdot, x_sym)])
    B_fn = ca.Function('B', [x_sym, u_sym], [ca.jacobian(xdot, u_sym)])

    A_full = np.array(A_fn(x_trim, u_trim)).astype(float)  # 17×17
    B_full = np.array(B_fn(x_trim, u_trim)).astype(float)  # 17×4

    # ── 변환 행렬 T (17×15) ──
    # 풀 상태: [px,py,pz, vx,vy,vz, qx,qy,qz,qw, wx,wy,wz, n1,n2,n3,n4]
    # 축소:    [   δz,     δv(3),     δφ(3),       δω(3),     δn(4)]  = 15D
    #
    # 매핑:
    #   δpz → idx 2
    #   δv  → idx 3:6
    #   δq = ∂q/∂φ @ δφ  (4×3), 트림 쿼터니언에서 계산
    #   δω  → idx 10:13
    #   δn  → idx 13:17

    q_trim = x_trim[6:10]
    Q_L = _quat_left_mult_matrix(q_trim)
    dq_dphi = 0.5 * Q_L[:, 0:3]   # 4×3: ∂q/∂δφ

    T = np.zeros((17, 15))
    # δz(1)     → 풀 상태 idx 2
    T[2, 0] = 1.0
    # δv(3)     → 풀 상태 idx 3:6
    T[3:6, 1:4] = np.eye(3)
    # δφ(3)     → 풀 상태 idx 6:10 (through dq_dphi)
    T[6:10, 4:7] = dq_dphi
    # δω(3)     → 풀 상태 idx 10:13
    T[10:13, 7:10] = np.eye(3)
    # δn(4)     → 풀 상태 idx 13:17
    T[13:17, 10:14] = np.eye(4)
    # 미사용 15번째 열은 0 (패딩, 실질 14D지만 15로 맞춤)
    # 실제로는 14D. 15번째 = 미사용.
    # → 14D로 하자.

    T = T[:, :14]  # 17×14

    # 축소: A_r = T⁺ A T,  B_r = T⁺ B  (T⁺ = pseudo-inverse)
    T_pinv = np.linalg.pinv(T)  # 14×17

    A_r = T_pinv @ A_full @ T   # 14×14
    B_r = T_pinv @ B_full       # 14×4

    return A_r, B_r, T, T_pinv


class LQRController:
    """
    Error-State LQR — 쿼터니언 축소 (17D → 14D 오차상태).

    오차상태 δx_r(14): [δz, δv(3), δφ(3), δω(3), δn(4)]

    u = u_trim - K_r @ δx_r
    여기서 δx_r은 현재 상태에서 트림으로의 오차를 14D로 변환한 것.
    """

    def __init__(self, params, x_trim, u_trim, Q=None, R=None):
        self.p = params
        self.x_trim = x_trim.copy()
        self.u_trim = u_trim.copy()

        # Error-state 선형화
        A_r, B_r, T, T_pinv = linearize_error_state(params, x_trim, u_trim)
        self.A_r, self.B_r = A_r, B_r
        self.T, self.T_pinv = T, T_pinv
        self.n_reduced = A_r.shape[0]  # 14

        # ── 비용 행렬 (14D) ──
        if Q is None:
            Q = np.diag([
                100,               # δz (고도) — 강하게
                10, 10, 20,        # δv (속도)
                50, 50, 50,        # δφ (자세 오차) — 강하게
                5, 5, 5,           # δω (각속도)
                0.01, 0.01, 0.01, 0.01
            ])
        if R is None:
            R = np.eye(4) * 0.05

        self.Q, self.R_cost = Q, R

        # ── ARE 풀이 ──
        try:
            P = solve_continuous_are(A_r, B_r, Q, R)
            self.K_r = np.linalg.inv(R) @ B_r.T @ P   # 4×14
            self.valid = True

            A_cl = A_r - B_r @ self.K_r
            self.eigvals = np.linalg.eigvals(A_cl)
            self.max_real = np.max(np.real(self.eigvals))

            # 풀 상태용 K: K_full(4×17) = K_r(4×14) @ T_pinv(14×17)
            self.K = self.K_r @ self.T_pinv

        except np.linalg.LinAlgError as e:
            print(f"  [경고] ARE 풀이 실패: {e}")
            self.K = np.zeros((4, 17))
            self.K_r = np.zeros((4, self.n_reduced))
            self.valid = False
            self.eigvals = np.array([])
            self.max_real = float('inf')

    def set_position_ref(self, pos):
        self.x_trim[0:3] = pos

    def _compute_error_state(self, x):
        """현재 상태 → 14D 오차상태."""
        # 위치 오차 (z만)
        dz = x[2] - self.x_trim[2]
        # 속도 오차
        dv = x[3:6] - self.x_trim[3:6]
        # 쿼터니언 오차 → 오차 각도 3D
        q = x[6:10]
        q_trim = self.x_trim[6:10]
        # 오차 쿼터니언: δq = q_trim⁻¹ ⊗ q
        # q_trim⁻¹ = conjugate (단위 쿼터니언)
        q_trim_inv = np.array([-q_trim[0], -q_trim[1], -q_trim[2], q_trim[3]])
        # Hamilton product: q_trim_inv ⊗ q
        dq = self._quat_mult(q_trim_inv, q)
        # 최단경로 보정: q와 -q는 같은 자세. 이 기체는 트림 w≡0(180° about x)
        # 이라 부호 경계가 정상 비행자세 바로 위 — SITL/실기의 w>0 정규화
        # (frame_utils/offboard_node)가 롤 ± 절반에서 q 부호를 뒤집으면
        # δφ가 반전되어 정피드백이 됨 (2026-08-04 코드리뷰 검증).
        if dq[3] < 0.0:
            dq = -dq
        # 소교란: δφ ≈ 2 * dq[0:3] (벡터부)
        dphi = 2.0 * dq[0:3]
        # 각속도 오차
        dw = x[10:13] - self.x_trim[10:13]
        # 로터 오차
        dn = x[13:17] - self.x_trim[13:17]

        return np.concatenate([[dz], dv, dphi, dw, dn])

    @staticmethod
    def _quat_mult(p, q):
        """Hamilton product p ⊗ q, scalar-last [x,y,z,w]."""
        return np.array([
            p[3]*q[0] + p[0]*q[3] + p[1]*q[2] - p[2]*q[1],
            p[3]*q[1] - p[0]*q[2] + p[1]*q[3] + p[2]*q[0],
            p[3]*q[2] + p[0]*q[1] - p[1]*q[0] + p[2]*q[3],
            p[3]*q[3] - p[0]*q[0] - p[1]*q[1] - p[2]*q[2]])

    def __call__(self, t, x):
        dx_r = self._compute_error_state(x)
        u = self.u_trim - self.K_r @ dx_r
        return np.clip(u, self.p['n_min'], self.p['n_max'])

    def print_info(self):
        if not self.valid:
            print("  LQR 설계 실패!")
            return

        print(f"  오차상태 차원: {self.n_reduced}D (17D → {self.n_reduced}D)")
        print(f"  K_r 크기: {self.K_r.shape}")
        print(f"  폐루프 최대 실수부: {self.max_real:.4f}",
              "(안정)" if self.max_real < -1e-6 else "(⚠ 불안정!)")

        eigs = self.eigvals
        real_neg = eigs[np.real(eigs) < -1e-10]
        if len(real_neg) > 0:
            slowest = real_neg[np.argmax(np.real(real_neg))]
            fastest = real_neg[np.argmin(np.real(real_neg))]
            print(f"  가장 느린 모드: λ = {slowest:.3f}  (τ = {-1/np.real(slowest):.3f} s)")
            print(f"  가장 빠른 모드: λ = {fastest:.3f}")


class ScheduledLQR:
    """
    속도별 게인 스케줄링 LQR — np.interp 선형 보간.

    초기화:
      V_table(0,10,...,80 m/s) 각 속도에서
      find_trim → linearize_error_state → ARE → K_r(4×14) 사전 계산.

    런타임:
      v_ref[0]으로 K_r, x_trim, u_trim을 np.interp 선형 보간.
      인접 게인 간 부드러운 천이 (계단형 불연속 없음).

    왜 선형 보간인가:
      최근접(nearest) 선택은 속도가 테이블 경계를 넘을 때 게인이
      불연속적으로 점프 → 제어 입력 튐. 선형 보간은 이를 방지.
    """

    def __init__(self, params, v_ref, z_ref=0.0, V_table=None, Q=None, R=None):
        from models.team_light.control.trim import find_trim

        self.p = params
        self.v_ref = np.array(v_ref, dtype=float)
        self.z_ref = z_ref

        automatic_table = V_table is None
        if V_table is None:
            V_table = np.arange(0, 90, 10).astype(float)
        self.V_table = np.array(V_table, dtype=float)
        if (self.V_table.ndim != 1 or self.V_table.size == 0
                or not np.all(np.isfinite(self.V_table))
                or np.any(self.V_table < 0) or np.any(np.diff(self.V_table) <= 0)):
            raise ValueError('V_table must be finite, nonnegative and strictly increasing.')

        # ── 각 속도에서 게인 사전 계산 ──
        K_r_list, x_trim_list, u_trim_list = [], [], []
        n_valid = 0

        for V in self.V_table:
            trim = find_trim(params, float(V), strict=False)
            if not trim['valid']:
                if automatic_table:
                    break  # Do not interpolate across an invalid equilibrium.
                raise ValueError(f'Invalid trim at scheduled speed {V:g} m/s.')
            x_t, u_t = trim['state'].copy(), trim['control'].copy()

            lqr = LQRController(params, x_t, u_t, Q, R)
            if lqr.valid:
                K_r_list.append(lqr.K_r.flatten())   # 4×14 = 56개 원소
                n_valid += 1
            else:
                raise ValueError(f'LQR design failed at {V:g} m/s.')

            if x_trim_list and np.dot(x_t[6:10], x_trim_list[-1][6:10]) < 0:
                x_t[6:10] *= -1
            x_trim_list.append(x_t)
            u_trim_list.append(u_t)

        if n_valid == 0:
            raise ValueError('No valid rocket trim available for ScheduledLQR.')
        self.V_table = self.V_table[:n_valid]
        # 보간용 배열: 각 행이 한 속도점의 값
        self._K_r_flat = np.array(K_r_list)       # (N_speeds, 56)
        self._x_trim_arr = np.array(x_trim_list)  # (N_speeds, 17)
        self._u_trim_arr = np.array(u_trim_list)  # (N_speeds, 4)
        self._nr = 14  # 축소 상태 차원

        print(f"  ScheduledLQR: {n_valid}/{len(V_table)} 속도점 유효")
        # Reject an out-of-range initial request before starting a simulation.
        self._interpolate(float(self.v_ref[0]))

    def _interpolate(self, V):
        """V에서 K_r(4×14), x_trim(17), u_trim(4) 선형 보간."""
        if V < self.V_table[0] - 1e-9 or V > self.V_table[-1] + 1e-9:
            raise ValueError(f'Reference {V:g} m/s outside validated LQR table '
                             f'[{self.V_table[0]:g}, {self.V_table[-1]:g}].')
        V_c = np.clip(V, self.V_table[0], self.V_table[-1])

        # K_r: 56개 원소 각각 보간 → 4×14로 reshape
        K_r = np.array([
            np.interp(V_c, self.V_table, self._K_r_flat[:, j])
            for j in range(self._K_r_flat.shape[1])
        ]).reshape(4, self._nr)

        # x_trim: 17개 원소 각각 보간
        x_trim = np.array([
            np.interp(V_c, self.V_table, self._x_trim_arr[:, j])
            for j in range(17)])
        # 보간된 쿼터니언 재정규화 (선형 보간은 단위구 벗어남)
        q = x_trim[6:10]
        qn = np.linalg.norm(q)
        if qn > 1e-10:
            x_trim[6:10] = q / qn

        # u_trim: 4개 원소 각각 보간
        u_trim = np.array([
            np.interp(V_c, self.V_table, self._u_trim_arr[:, j])
            for j in range(4)])

        return K_r, x_trim, u_trim

    @staticmethod
    def _compute_error_state(x, x_trim):
        """
        14D 오차상태: [δz, δv(3), δφ(3), δω(3), δn(4)].

        LQRController._compute_error_state와 동일하지만
        x_trim을 인자로 받아 외부에서 사용 가능.
        """
        dz = x[2] - x_trim[2]
        dv = x[3:6] - x_trim[3:6]

        # 쿼터니언 오차 → 오차 각도 3D
        q, q_t = x[6:10], x_trim[6:10]
        q_t_inv = np.array([-q_t[0], -q_t[1], -q_t[2], q_t[3]])
        dq = LQRController._quat_mult(q_t_inv, q)
        if dq[3] < 0.0:
            dq = -dq          # 최단경로 보정 (LQRController와 동일 — 위 주석 참고)
        dphi = 2.0 * dq[0:3]

        dw = x[10:13] - x_trim[10:13]
        dn = x[13:17] - x_trim[13:17]

        return np.concatenate([[dz], dv, dphi, dw, dn])

    def __call__(self, t, x):
        # v_ref[0]으로 스케줄링 (목표 속도의 게인/트림 사용)
        V = self.v_ref[0]
        K_r, x_trim, u_trim = self._interpolate(V)

        # 위치 x,y는 동역학에 무관 → 현재 값 사용
        x_trim[0:2] = x[0:2]
        x_trim[2] = self.z_ref

        dx_r = self._compute_error_state(x, x_trim)
        u = u_trim - K_r @ dx_r
        return np.clip(u, self.p['n_min'], self.p['n_max'])


class INDIController:
    """
    INDI — 증분 비선형 동적 역전.

    외측: 속도P + 고도PID → F_des → R_des + T_cmd (PID와 동일)
    내측: INDI
      1) 자세 오차 → 원하는 각가속도 ω̇_des  (PD)
      2) 측정 각가속도 ω̇_meas = LPF(Δω/Δt)  (센서 기반)
      3) 제어 효과 G(n) = ∂[T,ω̇]/∂n          (현재 작동점)
      4) 증분: Δn = G⁻¹ @ ([T,ω̇]_des - [T,ω̇]_meas)
      5) n_cmd = n_actual + Δn

    핵심 원리:
      기존 제어: 모델에서 '필요한 제어 전체'를 계산 → 모델 오차/외란에 취약
      INDI:      '현재 센서 측정'과 '원하는 것'의 차이만 보정
               → ω̇_meas가 바람·모델오차·자이로를 이미 포함
               → 측정·필터·효과도 근사가 맞는 범위에서 외란 억제를 의도

    구조:
      [속도/고도 PID] → F_des → [R_des + T_cmd]
                                      ↓
      [SO(3) 자세 오차] → ω̇_des → [INDI: Δn = G⁻¹(ν_des - ν_meas)]
                                      ↓
                                  n_cmd = n_actual + Δn
    """

    def __init__(self, params, v_ref=None, z_ref=0.0, dt=0.001, f_cut=50.0):
        self.p = params
        self.m = params['mass']
        self.g = params['g']
        self.dt = dt

        self.v_ref = np.array(v_ref) if v_ref is not None else np.zeros(3)
        self.z_ref = z_ref

        # ── 외측 게인 (PID 외측과 동일 → 비교 공정성) ──
        self.Kp_vel = 1.0
        self.Kp_z = 2.0
        self.Kd_z = 2.0
        self.Ki_z = 0.5
        self.int_z_max = 5.0
        self.max_tilt = np.radians(45)

        # ── 내측 INDI 게인 ──
        # 2차 응답 설계: φ̈ + Kd·φ̇ + Kp·φ = 0
        # 기존 초기 게인 ωn=20 rad/s, ζ=0.7; 전 속도 안정성 보장은 아님.
        # INDI에서 게인은 관성과 무관 (G가 관성을 자동 보정)
        wn = 20.0
        zeta = 0.7
        self.Kp_indi = np.full(3, wn**2)           # 400 rad/s²/rad
        self.Kd_indi = np.full(3, 2 * zeta * wn)   # 28 rad/s²/(rad/s)

        # ── LPF (1차 IIR, 각가속도 필터링) ──
        # f_cut=50Hz: 기계적 동역학은 통과, 수치 노이즈 차단
        self._alpha = dt / (dt + 1.0 / (2 * np.pi * f_cut))

        # ── 폴백용 할당 행렬 (로터 정지 시) ──
        from models.team_light.control.dynamics import compute_allocation_matrix
        _, self._TM_to_f = compute_allocation_matrix(params, static_reference=True)

        # ── 내부 상태 ──
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._int_ez = 0.0
        self._initialized = False

    def reset(self):
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._int_ez = 0.0
        self._initialized = False

    def __call__(self, t, x):
        pos, vel = x[0:3], x[3:6]
        q, omega = x[6:10], x[10:13]
        n_actual = x[13:17]
        R = Rotation.from_quat(q).as_matrix()

        # ━━ 1. 외측: 속도/고도 → T_cmd + R_des ━━
        e_vel = vel - self.v_ref
        e_z = pos[2] - self.z_ref
        self._int_ez += e_z * self.dt
        self._int_ez = np.clip(self._int_ez, -self.int_z_max, self.int_z_max)

        a_des = np.zeros(3)
        a_des[0:2] = -self.Kp_vel * e_vel[0:2]
        a_des[2] = -self.Kp_z * e_z - self.Kd_z * vel[2] - self.Ki_z * self._int_ez
        F_des = self.m * (a_des + np.array([0, 0, self.g]))

        T_cmd, R_des = self._force_to_attitude(F_des)

        # ━━ 2. 자세 오차 → 원하는 각가속도 ━━
        # Optional nominal trim feedforward for the v2 experiment adapter.
        # It uses the nominal reference only, never the true gust/aero mismatch.
        if getattr(self, 'trim_force_ref', None) is not None:
            F_des = self.trim_force_ref + self.m*a_des
            T_cmd, R_des = self._force_to_attitude(F_des)
        S_err = 0.5 * (R_des.T @ R - R.T @ R_des)
        att_err = np.array([-S_err[1, 2], S_err[0, 2], -S_err[0, 1]])
        # gyro FF 불필요: ω̇_meas에 이미 자이로·공력 효과 포함
        omega_dot_des = -self.Kp_indi * att_err - self.Kd_indi * omega

        # ━━ 3. 각가속도 측정 (LPF) ━━
        # 측정 NaN 가드 (리뷰 3d): 자기참조 LPF는 NaN 1회로 영구 고착
        if not (np.all(np.isfinite(omega)) and np.all(np.isfinite(n_actual))):
            return self._fallback(T_cmd, omega_dot_des, R.T @ vel)

        if not self._initialized:
            self._omega_prev = omega.copy()
            self._initialized = True
            if uses_curve(self.p):
                return np.clip(n_actual, self.p['n_min'], self.p['n_max'])
            return self._fallback(T_cmd, omega_dot_des, R.T @ vel)

        omega_dot_raw = (omega - self._omega_prev) / self.dt
        self._omega_dot_filt = (self._alpha * omega_dot_raw
                                + (1 - self._alpha) * self._omega_dot_filt)
        self._omega_prev = omega.copy()

        # ━━ 4. INDI 증분 ━━
        # 전진비(advance ratio) 반영 — 버그픽스 3-1을 standalone INDI에도 적용.
        # (수직상승/이륙 등 축방향 유입 V_axial>0 이면 fac<1 → 추력·자세효과 과대평가 방지.
        #  ProperHybrid.compute_control_effectiveness와 동일 규칙.)
        v_body = R.T @ vel
        T_meas = float(np.sum(rotor_thrusts(self.p, n_actual, v_body)))

        # 가상 제어 오차: [추력, 각가속도] 기대 - 측정
        dv = np.array([T_cmd - T_meas,
                       omega_dot_des[0] - self._omega_dot_filt[0],
                       omega_dot_des[1] - self._omega_dot_filt[1],
                       omega_dot_des[2] - self._omega_dot_filt[2]])

        # G: 현재 로터 속도에서의 제어 효과 (4×4, 전진비 반영)
        G = self._compute_G(n_actual, v_body)

        try:
            dn = np.linalg.solve(G, dv)
        except np.linalg.LinAlgError:
            return self._fallback(T_cmd, omega_dot_des, v_body)

        n_cmd = n_actual + dn
        return np.clip(n_cmd, self.p['n_min'], self.p['n_max'])

    def _compute_G(self, n_actual, v_body=None):
        """Derivative of the rocket rotor map, shared with the hybrid."""
        return control_effectiveness(self.p, n_actual, v_body)


    def _force_to_attitude(self, F_des):
        """World force -> body +x attitude reference."""
        return force_to_attitude(F_des, 0.0, self.max_tilt)


    def _fallback(self, T_cmd, omega_dot_des, v_body=None):
        """G 특이(로터 정지 등) 시 모델 기반 폴백."""
        J = np.diag([self.p['Ixx'], self.p['Iyy'], self.p['Izz']])
        M_cmd = J @ omega_dot_des
        TM = np.array([T_cmd, M_cmd[0], M_cmd[1], M_cmd[2]])
        if uses_curve(self.p):
            return allocate_wrench(self.p, TM, v_body)
        f_ind = self._TM_to_f @ TM
        return rotor_speeds_for_thrust(self.p, f_ind, v_body)
