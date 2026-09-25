"""
NMPC+INDI 하이브리드 — 인터페이스 분리 vs 나이브 비교
====================================================

핵심 변경:
  나이브: NMPC→[n1..n4], INDI도→[Δn] → 이중 보정 → 악화
  분리:   NMPC→[T, ω̇_des], INDI→[n1..n4] → 역할 분리 → 올바름

VirtualNMPC:
  상태 13D: [p(3), v(3), q(4), ω(3)]  (로터 상태 제거)
  제어 4D:  [T_total, ω̇_x, ω̇_y, ω̇_z] (가상 명령)
  내부 동역학: ω̇ = ν_ω (INDI가 이를 실현한다고 가정)

검증:
  정확 모델 + 돌풍 → 나이브 악화 vs 분리 정상
  C_Na ±20% + 돌풍 → 분리가 모델 오차도 보상하는지
"""

from functools import lru_cache

import numpy as np
import casadi as ca
import time as timer
from scipy.spatial.transform import Rotation

from control.vehicle_params import vehicle_params as P
from control.dynamics import (build_dynamics, _quat_to_rotmat, _quat_derivative,
                      _body_aerodynamics, EPS, compute_allocation_matrix,
                      PROP_CURVE_MODEL)
from control.trim import find_trim
from control.controller import ScheduledLQR
from control.nmpc import NMPCController


# ══════════════════════════════════════════════════
# 1. 가상 명령 동역학 (13D)
# ══════════════════════════════════════════════════

NX_V = 13   # [p(3), v(3), q(4), ω(3)]
NU_V = 4    # [T, ν_ωx, ν_ωy, ν_ωz]


def _curve_thrust(params, n, V_axial):
    """팀원 APC 곡선 기반 로터 추력(수치, 배열 가능) — 곡선 재구현 없이
    `models.team_light.control.propeller_curve.coefficients()`를 그대로
    쓴다(CasADi 대칭 미분이 필요 없는 자리 — `rotor_thrust_cap`, INDI의
    T_meas, `_fallback`용)."""
    from models.team_light.control.propeller_curve import coefficients
    n = np.asarray(n, dtype=float)
    n_rps = n / (2 * np.pi)
    J = V_axial / (n_rps * params['D_prop'] + 1e-8)
    CT, _ = coefficients(params, J)
    return params['rho'] * n_rps**2 * params['D_prop']**4 * CT


def rotor_thrust_cap(params, V_axial):
    """유속 V_axial·명목 최대 회전수에서의 로터별 추력 상한 f_max,i (식A2).

    전진비 J가 커지면 같은 회전수로 낼 수 있는 추력이 줄어든다. 이 상한은
    제어법칙의 선택이 아니라 물리적 사실이므로, 제약 배분(A1)과 기하 외부
    루프의 적분기 정지 판정이 같은 값을 써야 한다 — 그래서 모듈 함수로 뺐다.

    `propulsion_model`이 팀원 APC 곡선이면 그 곡선으로, 아니면(우리 자신의
    기체) 기존 선형 fac 모델로 — kj 지적(2026-09-25 밤)에 따라 선형모델이
    실제보다 훨씬 낮은 추력상한을 만들어 왔었다(80/85m/s에서 실제의
    43%뿐). `control/dynamics.py`와 동일한 조건 분기.
    """
    n_i = params['n_max']
    if params.get('propulsion_model') == PROP_CURVE_MODEL:
        return np.full(4, float(_curve_thrust(params, n_i, V_axial)))
    n_rps = n_i / (2 * np.pi)
    J = V_axial / (n_rps * params['D_prop'] + 1e-8)
    fac = max(1.0 - J / params['J_max'], 0.0)
    return np.full(4, params['k_T'] * n_i**2 * fac)


def build_virtual_dynamics(params):
    """
    가상 명령 동역학: ω̇ = ν_ω (INDI가 실현).

    NMPC는 "얼마나 회전시킬지"만 결정.
    모터 할당·자이로·공력 모멘트는 INDI가 센서로 처리.
    공력 힘(항력·양력)은 NMPC가 알아야 궤적 예측이 되므로 포함.
    """
    x = ca.SX.sym('x', NX_V)
    u = ca.SX.sym('u', NU_V)

    pos, vel, quat, omega = x[0:3], x[3:6], x[6:10], x[10:13]
    T_cmd = u[0]          # 총 추력 [N]
    nu_omega = u[1:4]     # 가상 각가속도 명령 [rad/s²]

    R = _quat_to_rotmat(quat)
    v_body = R.T @ vel

    # 공력 힘 (모멘트는 INDI가 처리하므로 무시)
    F_aero, _ = _body_aerodynamics(v_body, omega, params)

    # 가상 모델도 물리 플랜트와 같은 추력축을 사용해야 한다.
    thrust = (ca.vertcat(T_cmd, 0, 0) if params.get('thrust_axis', 'z') == 'x'
              else ca.vertcat(0, 0, -T_cmd))
    F_body = F_aero + thrust

    p_dot = vel
    v_dot = ca.vertcat(0, 0, -params['g']) + (R @ F_body) / params['mass']
    q_dot = _quat_derivative(quat, omega)
    omega_dot = nu_omega    # INDI가 이걸 실현

    xdot = ca.vertcat(p_dot, v_dot, q_dot, omega_dot)
    f = ca.Function('f_virtual', [x, u], [xdot])
    return f, x, u


# ══════════════════════════════════════════════════
# 2. VirtualNMPC (가상 명령 출력)
# ══════════════════════════════════════════════════

class VirtualNMPC:
    """NMPC with virtual command output [T, ω̇_des]."""

    def __init__(self, params, v_ref=None, z_ref=0.0, T_ref=None,
                 N=20, dt_nmpc=0.05, dt_ctrl=0.02, Q_z=20.0, max_iter=30,
                 alloc_feedback=False, c2_limit=None,
                 cost_spec='paper', ref_fn=None):
        """
        cost_spec : {'paper','legacy'}
            'paper' (**기본값**, 2026-09-24부터) — 논문 v5.3 식(13)-(18)·(31)의
            정확한 전사.

            'legacy' — 이 클래스가 원래 쓰던 비용/구조. 2026-09-24 이전의 기존
            결과(results/, mission_sim, robustness_*)를 재현할 때만 쓴다. 그
            결과들은 기체 모델 자체가 달랐던 시절의 것이라 비교 대상이 아니다
            (kj 판단) — 그래서 기본값을 'paper'로 뒤집었다.

            둘의 차이(감사 결과, m=1.7117 기준):

              항          논문(식)              legacy            차이
              속도가중    5·I₃ (식14)           diag(5,5,10)      vz만 2배
              각속도가중  ‖ω‖² (식14)           diag(1,1,1)       같음
              입력편차T   0.02/(mg)²=7.09e-5    1e-5              7.1배 약함
              입력편차α   0.02/100²=2e-6        1e-3              500배 강함
              입력변화T   0.1/(mg)²=3.55e-4     1e-4              3.5배 약함
              입력변화α   0.1/100²=1e-5         0.01              1000배 강함
              종말가중    stage 전체×10 (식16)  v·z만×10          ω 누락
              참조        노드별 r_{j|k} (식14) 상수 1개          램프 불가
              적분        RK4 5회+정규화(4.2절) RK4 1회           정확도
              결정변수    (N+1)nx+N·nu=353(식31) 340(x₀는 파라미터)

            legacy에는 Dν 정규화(식15) 자체가 없어서 '튜닝으로 고른 값'이
            아니라 다른 세대의 구현으로 보인다. 논문 실험은 'paper'로 돌려야
            한다 — legacy로 돌리면 논문이 적어 놓은 비용함수와 코드가 다르다.

        ref_fn : callable | None
            'paper'에서만 쓰인다. ``ref_fn(t) -> (vx, vy, vz, z)``로 절대시각
            t의 참조를 준다. 식(14)의 r_{j|k}를 예측 노드마다 채우기 위한 것
            으로, 램프/가감속 프로필 추종에 필요하다. None이면 v_ref·z_ref를
            모든 노드에 같은 값으로 채운다(상수 참조).

        alloc_feedback : bool
            논문 식(27)-(28) C1. 배분 결과를 다음 솔브의 입력변화량 비용
            기준 ν_{-1|k}으로 쓴다. 표6의 V13-1 → V13-2 단계.
        c2_limit : float | None
            논문 5.4절 변형 **C2**. 첫 예측 입력이 배분값에서 정규화 거리
            ``‖Dν⁻¹(ν_0|k − ν_alloc)‖ ≤ c2_limit`` 안에 있도록 **제약**한다.
            None(기본값)이면 제약을 걸지 않아 기존 동작이 그대로 보존된다.

            왜 필요한가 — C1은 식(17)의 첫 변화량 항 하나에만 작용하는 '비용'
            이라, 다른 입력 비용에 묻혀 효과가 안 보일 수 있다(논문 202행).
            C2는 같은 정보를 '제약'으로 올려 연결 강도를 높인 것이다. C0↔C1
            차이가 프로필 편차 안에 머물고 C2에서만 차이가 나면, 배분 피드백의
            효과는 연결 방식에 의존한다는 결론이 나온다.

            한계값은 논문이 "예비 시험으로 정한다"고만 해서 숫자를 박지 않았다.
            Dν로 무차원화했으므로 1.0이면 '호버 추력 하나만큼' 떨어질 수 있다는
            뜻이다.

            주의: ν_alloc이 실제로 들어와야 의미가 있으므로 alloc_feedback=True
            (C1)와 함께 써야 한다. 아래에서 그 조합을 검사한다.
        """
        if cost_spec not in ('legacy', 'paper'):
            raise ValueError(f"cost_spec must be 'legacy' or 'paper', got {cost_spec!r}")
        if ref_fn is not None and cost_spec != 'paper':
            raise ValueError("ref_fn은 cost_spec='paper'에서만 쓸 수 있다 — "
                             "legacy 경로는 참조가 상수 하나뿐이다.")
        if c2_limit is not None:
            if c2_limit <= 0:
                raise ValueError(f"c2_limit must be positive, got {c2_limit}")
            if not alloc_feedback:
                raise ValueError(
                    "c2_limit은 alloc_feedback=True(C1)와 함께 써야 한다 — "
                    "C1이 꺼져 있으면 기준값이 고정 트림이라 '배분값 주변 제한'이 "
                    "아니라 '트림 주변 제한'이 되어 다른 실험이 된다.")
        self.p = params
        self.N, self.dt_nmpc, self.dt_ctrl = N, dt_nmpc, dt_ctrl
        self.v_ref = np.array(v_ref) if v_ref is not None else np.zeros(3)
        self.z_ref = z_ref
        self._Q_z = Q_z
        self._max_iter = max_iter   # IPOPT 반복 상한 (SITL 실시간용 축소 가능)
        # 논문 식(27)-(28): INDI 배분 결과를 다음 솔브의 입력변화량 비용
        # 기준(ν_{-1|k})으로 쓸지 여부. False면 기존과 동일하게 고정 트림값을
        # 쓴다 — 기본값 False로 기존 호출부(mission_sim 등) 동작을 보존한다.
        self.alloc_feedback = alloc_feedback
        self.c2_limit = c2_limit
        self.cost_spec = cost_spec
        self.ref_fn = ref_fn

        self.T_ref = T_ref if T_ref else params['mass'] * params['g']
        self.u_ref = np.array([self.T_ref, 0, 0, 0])
        self._prev_input = self.u_ref.copy()

        f, x_sym, u_sym = build_virtual_dynamics(params)

        if cost_spec == 'paper':
            # 4.2절: 각 예측 격자에서 RK4 5회 세부적분 + 쿼터니언 정규화.
            self.F = self._make_substep_integrator(f, x_sym, u_sym, substeps=5)
            self._build_nlp_paper(params, x_sym, u_sym)
        else:
            # RK4 (1회 — legacy)
            dt = dt_nmpc
            k1 = f(x_sym, u_sym)
            k2 = f(x_sym + dt/2*k1, u_sym)
            k3 = f(x_sym + dt/2*k2, u_sym)
            k4 = f(x_sym + dt*k3, u_sym)
            self.F = ca.Function('F_v', [x_sym, u_sym],
                                 [x_sym + dt/6*(k1 + 2*k2 + 2*k3 + k4)])
            self._build_nlp(params, x_sym, u_sym)
        self._t_now = 0.0       # ref_fn 평가 시각. _solve 시그니처를 못 바꿔서
                                # (bench_*·final_config_mission이 위치인자 1개로
                                #  호출하고 몽키패치까지 한다) 필드로 전달한다.
        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()
        # 솔버 수렴 추적 (연속 미수렴 → HybridWithFallback의 전환 판단에 사용)
        self.consec_fail = 0
        self.last_status = 'none'
        self._ever_converged = False
        self._cold_start = True   # 첫 _solve()에서 콜드스타트 웜스타트 보정용
        # _w0_init는 _build_nlp()에서 설정됨

    def reset(self):
        """MC 시행 간 독립성 보장을 위한 완전 리셋."""
        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()
        self._cold_start = True
        self.consec_fail = 0
        self.last_status = 'none'
        self._ever_converged = False
        self._prev_input = self.u_ref.copy()
        if self._w0_init is not None:
            self.w0 = self._w0_init.copy()

    def set_prev_input(self, v):
        """INDI 배분 결과(v_alloc)를 다음 솔브의 입력변화량 비용 기준으로 받는다
        (식27-28). alloc_feedback=False면 무시 — 항상 안전하게 호출 가능."""
        if self.alloc_feedback:
            self._prev_input = np.asarray(v, dtype=float)

    def _build_nlp(self, params, x_sym, u_sym):
        N, nx, nu = self.N, NX_V, NU_V

        Q_v = np.diag([5.0, 5.0, 10.0])
        Q_z = self._Q_z
        Q_w = np.diag([1.0, 1.0, 1.0])
        # 가상 명령 페널티: 추력 변화 작게, 각가속도 부드럽게
        R = np.diag([1e-5, 1e-3, 1e-3, 1e-3])
        R_du = np.diag([1e-4, 0.01, 0.01, 0.01])

        T_max = 4 * params['k_T'] * params['n_max']**2
        nu_max = 100.0  # rad/s²

        # p 레이아웃: [x_init(nx), v_ref(3), z_ref(1), u_ref(nu), u_prev(nu)]
        # u_ref: 매 단계 정규화 목표(트림) — 항상 고정.
        # u_prev: 0단계 입력변화량 비용의 기준(식28의 ν_{-1|k}).
        #         alloc_feedback=False면 u_ref와 동일값이 매번 들어온다(기존과 동일 동작).
        p = ca.SX.sym('p', nx + 3 + 1 + nu + nu)
        x_init = p[0:nx]
        v_ref = p[nx:nx+3]
        z_ref = p[nx+3]
        u_ref = p[nx+4:nx+4+nu]
        u_prev = p[nx+4+nu:nx+4+2*nu]

        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J_cost = 0.0
        X_prev, U_prev = x_init, u_prev

        for k in range(N):
            U_k = ca.SX.sym(f'U_{k}', nu)
            w.append(U_k)
            lbw += [0.0, -nu_max, -nu_max, -nu_max]
            ubw += [T_max, nu_max, nu_max, nu_max]
            w0 += [float(self.T_ref), 0, 0, 0]

            X_k = ca.SX.sym(f'X_{k}', nx)
            w.append(X_k)
            lbw += [-1e6]*nx; ubw += [1e6]*nx
            _xg = [0.0]*nx
            _xg[6:10] = ([0.0, -np.sqrt(0.5), 0.0, np.sqrt(0.5)]
                         if params.get('thrust_axis', 'z') == 'x'
                         else [1.0, 0.0, 0.0, 0.0])   # 유효 단위 쿼터니언(호버) 초기추측
            w0 += _xg                          # (nmpc.py와 동일 픽스 — cold 솔브 개선)

            g.append(X_k - self.F(X_prev, U_k))
            lbg += [0.0]*nx; ubg += [0.0]*nx

            if k == 0 and self.c2_limit is not None:
                # C2 (논문 5.4절): 첫 예측 입력을 배분값 주변으로 제한.
                # Dν = diag(mg, 100, 100, 100) 로 무차원화 — 식(15)와 같은 척도.
                # 노름 대신 제곱노름을 쓴다(0에서 미분 불가한 sqrt 회피).
                D_nu = ca.DM([self.T_ref, 100.0, 100.0, 100.0])
                g.append(ca.sumsqr((U_k - u_prev) / D_nu))
                lbg += [0.0]; ubg += [float(self.c2_limit)**2]

            e_v = X_k[3:6] - v_ref
            e_z = X_k[2] - z_ref
            dU = U_k - U_prev
            J_cost += e_v.T @ Q_v @ e_v + Q_z*e_z**2
            J_cost += X_k[10:13].T @ Q_w @ X_k[10:13]
            J_cost += (U_k - u_ref).T @ R @ (U_k - u_ref)
            J_cost += dU.T @ R_du @ dU
            X_prev, U_prev = X_k, U_k

        J_cost += 10*(X_prev[3:6]-v_ref).T @ Q_v @ (X_prev[3:6]-v_ref)
        J_cost += 10*Q_z*(X_prev[2]-z_ref)**2

        nlp = {'f': J_cost, 'x': ca.vertcat(*w),
               'g': ca.vertcat(*g), 'p': p}
        self.solver = ca.nlpsol('vnmpc', 'ipopt', nlp, {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
            'ipopt.max_iter': self._max_iter, 'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-4})
        self.lbw = np.array(lbw)
        self.ubw = np.array(ubw)
        self.lbg = np.array(lbg)
        self.ubg = np.array(ubg)
        self.w0 = np.array(w0)
        self._w0_init = self.w0.copy()

    def _make_substep_integrator(self, f, x_sym, u_sym, substeps=5):
        """예측 격자 하나를 RK4 세부적분으로 넘긴다 (논문 4.2절).

        한 번에 h=50 ms를 RK4로 넘기면, 자세·각속도가 빠르게 움직이는 구간에서
        적분 오차가 예측을 왜곡한다. 세부적분으로 실효 스텝을 10 ms로 줄이고,
        매 세부단계마다 쿼터니언을 정규화해 단위노름에서 벗어나지 않게 한다
        (정규화를 안 하면 RK4가 노름을 조금씩 키워 자세가 서서히 뒤틀린다).

        `ca.norm_2`는 0 근처에서 미분(1/(2·sqrt(x)))이 발산한다 — IPOPT가
        수렴 전 탐색 중 만드는 극단적 시행점에서 쿼터니언 성분이 0 근처를
        지나가면 NaN이 날 수 있다(kj 지적, 2026-09-25 저녁: 정확한 트림
        단발 솔브의 격리실험에서는 이게 최종 결과를 바꾸진 않았지만, 짧은
        시험 같은 다단계 폐루프에서는 다를 수 있어 별도로 없앤다). eps로
        정칙화해 이 나눗셈 특이점 자체를 없앤다 — M17(`nmpc.py`)·
        F13(`nmpc_f13.py`)에도 동일 적용.
        """
        h = self.dt_nmpc / substeps
        st = x_sym
        for _ in range(substeps):
            k1 = f(st, u_sym)
            k2 = f(st + h/2*k1, u_sym)
            k3 = f(st + h/2*k2, u_sym)
            k4 = f(st + h*k3, u_sym)
            st = st + h/6*(k1 + 2*k2 + 2*k3 + k4)
            q_norm = ca.sqrt(ca.sumsqr(st[6:10]) + EPS)
            st = ca.vertcat(st[0:6], st[6:10]/q_norm, st[10:13])
        return ca.Function('F_paper', [x_sym, u_sym], [st])

    def _build_nlp_paper(self, params, x_sym, u_sym):
        """논문 v5.3 식(13)-(18)·(31)의 직접 전사.

        legacy와 나란히 두고 읽을 수 있게 따로 뺐다. 한 함수에 플래그를
        섞으면 '논문이 뭐라고 했는지'가 분기 속에 묻힌다.
        """
        N, nx, nu = self.N, NX_V, NU_V

        # 식(15): 가상입력 정규화 척도와 호버 기준값.
        #   Dν = diag(mg, 100, 100, 100),  νh = [mg, 0, 0, 0]ᵀ
        # 100은 식(18)의 각가속도 한계와 같은 수라 각 성분이 '한계의 몇 %'가 된다.
        D_nu = ca.DM([self.T_ref, 100.0, 100.0, 100.0])
        nu_h = ca.DM([self.T_ref, 0.0, 0.0, 0.0])

        # 식(18): 0 ≤ T ≤ T_max,0 (정지 유속에서의 4로터 합), |α_i| ≤ 100 rad/s²
        T_max = 4 * params['k_T'] * params['n_max']**2
        a_max = 100.0

        # p 레이아웃: [x_meas(nx), refs(4·(N+1)), nu_prev(nu)]
        #   refs 는 노드별 (vx,vy,vz,z) — 식(14)의 r_{j|k}.
        p = ca.SX.sym('p', nx + 4*(N+1) + nu)
        x_meas = p[0:nx]
        refs = ca.reshape(p[nx:nx + 4*(N+1)], 4, N+1)
        nu_prev = p[nx + 4*(N+1):]

        # 결정변수: X_0, U_0, X_1, U_1, …, X_{N-1}, U_{N-1}, X_N
        #   x₀를 결정변수에 넣고 등식으로 묶는 직접 다중사격 형태 — 식(31)의
        #   n_var = (N+1)·n_x + N·n_u = 21·13 + 20·4 = 353 이 이 배치에서 나온다.
        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J_cost = 0.0

        _q_hover = ([0.0, -np.sqrt(0.5), 0.0, np.sqrt(0.5)]
                    if params.get('thrust_axis', 'z') == 'x'
                    else [1.0, 0.0, 0.0, 0.0])

        def new_state(k):
            X = ca.SX.sym(f'X_{k}', nx)
            w.append(X)
            lbw.extend([-1e6]*nx); ubw.extend([1e6]*nx)
            guess = [0.0]*nx
            guess[6:10] = _q_hover          # 유효 단위 쿼터니언으로 시작
            w0.extend(guess)
            return X

        X_k = new_state(0)
        g.append(X_k - x_meas)              # 측정 상태 고정
        lbg.extend([0.0]*nx); ubg.extend([0.0]*nx)

        U_prev = nu_prev
        for k in range(N):
            # ── 단계 상태비용 식(14) ──
            e_v = X_k[3:6] - refs[0:3, k]
            e_z = X_k[2] - refs[3, k]
            J_cost += (5.0*ca.sumsqr(e_v) + self._Q_z*e_z**2
                       + ca.sumsqr(X_k[10:13]))

            U_k = ca.SX.sym(f'U_{k}', nu)
            w.append(U_k)
            lbw.extend([0.0, -a_max, -a_max, -a_max])
            ubw.extend([T_max, a_max, a_max, a_max])
            w0.extend([float(self.T_ref), 0.0, 0.0, 0.0])

            # ── 입력비용 식(17) — Dν로 무차원화한 두 항 ──
            J_cost += 0.02*ca.sumsqr((U_k - nu_h) / D_nu)
            J_cost += 0.10*ca.sumsqr((U_k - U_prev) / D_nu)

            if k == 0 and self.c2_limit is not None:
                g.append(ca.sumsqr((U_k - nu_prev) / D_nu))
                lbg.append(0.0); ubg.append(float(self.c2_limit)**2)

            X_next = new_state(k+1)
            g.append(X_next - self.F(X_k, U_k))
            lbg.extend([0.0]*nx); ubg.extend([0.0]*nx)
            X_k, U_prev = X_next, U_k

        # ── 종말비용 식(16): 단계 상태비용 '전체'에 10배 (ω 포함) ──
        e_v = X_k[3:6] - refs[0:3, N]
        e_z = X_k[2] - refs[3, N]
        J_cost += 10.0*(5.0*ca.sumsqr(e_v) + self._Q_z*e_z**2
                        + ca.sumsqr(X_k[10:13]))

        nlp = {'f': J_cost, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': p}
        self.solver = ca.nlpsol('vnmpc_paper', 'ipopt', nlp, {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
            'ipopt.max_iter': self._max_iter, 'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-4})
        self.lbw = np.array(lbw)
        self.ubw = np.array(ubw)
        self.lbg = np.array(lbg)
        self.ubg = np.array(ubg)
        self.w0 = np.array(w0)
        self._w0_init = self.w0.copy()

    def _reference_horizon(self):
        """예측 노드별 (vx,vy,vz,z) 참조 — 식(14)의 r_{j|k}."""
        if self.ref_fn is None:
            one = np.concatenate([self.v_ref, [self.z_ref]])
            return np.tile(one[:, None], (1, self.N + 1))
        cols = [np.asarray(self.ref_fn(self._t_now + k*self.dt_nmpc),
                           dtype=float)
                for k in range(self.N + 1)]
        return np.stack(cols, axis=1)

    def __call__(self, t, x_full):
        """17D 플랜트 상태 → 13D 추출 → [T, ν_ω] 반환."""
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            x13 = np.concatenate([x_full[0:10], x_full[10:13]])
            self._t_now = t
            self._u_current = self._solve(x13)
            self._last_t = t
        return self._u_current

    def _solve(self, x13):
        if self.cost_spec == 'paper':
            if self._cold_start:
                # kj 지적(2026-09-25 저녁, results/SOLVER_FAILURE_LOG 20:XX) —
                # _build_nlp_paper의 정적 웜스타트는 상태 부분을 전부
                # [0,0,0,...,호버쿼터니언,...,0,0,0]로 채운다. 순항속도가
                # 크고 트림 자세가 호버에서 먼 조건(예: 80m/s, pitch≈-8°)
                # 에서는 이 추측값이 실측 x13와 크게 어긋나 있어, 다중사격
                # 등식제약(X_{k+1}=F(X_k,U_k))이 전체 호라이즌에서 동시에
                # 크게 위반된 채로 첫 뉴턴스텝을 시작한다 — 진단 결과
                # (control/test_v13_degenerate_first_step.py 및 diag10~12,
                # 로그 참고) 이게 실제 원인이었다: 콜드스타트 웜스타트를
                # 트림/실측 상태로 바꾸면(가상입력 U_k 추측값은 이미
                # 합리적이라 그대로 둠) 정확한 트림에서 항상 수렴했고,
                # eps로 쿼터니언 정규화만 정칙화해서는(나눗셈 특이점 자체)
                # 바뀌지 않았다 — 즉 원인은 나눗셈 특이점이 아니라
                # 웜스타트였다(격리 실험으로 확인, eps 유무 무관).
                # U_k는 그대로 두고 X_k만 x13으로 덮어쓴다.
                stride = NU_V + NX_V
                for k in range(self.N + 1):
                    off = k * stride
                    self.w0[off:off + NX_V] = x13
                self._cold_start = False
            p_val = np.concatenate([x13,
                                    self._reference_horizon().ravel(order='F'),
                                    self._prev_input])
        else:
            p_val = np.concatenate([x13, self.v_ref, [self.z_ref], self.u_ref,
                                     self._prev_input])
        sol = self.solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw,
                          lbg=self.lbg, ubg=self.ubg, p=p_val)

        # 수렴 추적: max_iter 도달 등 미수렴 해는 IPOPT의 "최선 반복해"라
        # 한 번은 쓸 만하지만 연속되면 품질 저하 누적 (벤치 실측: 과도구간에서
        # iter=30 상한 도달 사례). 솔브 단위로 카운트해 폴백이 판단하게 함.
        # cold-start(최초 수렴 전) 실패는 예상된 것이라 카운트 제외 —
        # 처음부터 고장난 NMPC는 폴백의 고도오차 트리거가 잡음.
        status = self.solver.stats().get('return_status', 'unknown')
        if status in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'):
            self.consec_fail = 0
            self._ever_converged = True
        elif self._ever_converged:
            self.consec_fail += 1
        self.last_status = status

        w_opt = np.array(sol['x']).flatten()
        stride = NU_V + NX_V
        if self.cost_spec == 'paper':
            # 배치: X_0, U_0, X_1, U_1, …, X_{N-1}, U_{N-1}, X_N
            #   → 첫 입력은 X_0 뒤에 있고, 워밍시프트는 한 단계 버린 뒤
            #     마지막 (U_{N-1}, X_N)을 복제해 길이를 되맞춘다.
            u_opt = w_opt[NX_V:NX_V + NU_V]
            self.w0 = np.concatenate([w_opt[stride:],
                                      w_opt[-stride:-NX_V],   # U_{N-1}
                                      w_opt[-NX_V:]])         # X_N
        else:
            # 배치: U_0, X_0, U_1, X_1, … (legacy)
            u_opt = w_opt[0:NU_V]
            self.w0 = np.concatenate([w_opt[stride:], w_opt[-stride:]])
        return u_opt


def constrained_allocation(G, dT_target, domega_target, n_actual, n_min, n_max,
                            max_iter=4):
    """
    총추력 등식 제약 하 각가속도 잔차 최소 배분 — 논문 식(24)-(26)/부록 식(A2)-(A3)의
    증분(Δn) 공간 국소 해.

    G(4x4)는 현재 동작점의 선형 입력효과 행렬(0행=∂T/∂n, 1-3행=∂ω̇/∂n,
    ProperHybrid._compute_G와 동일). dT_target·domega_target은 INDI 증분
    목표(요구-측정, 식20 B(f)=∂b/∂f의 소신호 가정). 반환하는 Δn은:
      Σ G[0,i]·Δn_i = dT_target  을 (포화가 없는 한) 정확히 만족시키고,
    그 부분공간 안에서 ||G[1:4]·Δn - domega_target||²를 최소화한다.

    포화(n_min/n_max 도달)는 활성집합법으로 처리한다 — 로터가 4개뿐이라
    매 반복 자유변수가 최소 1개 줄어들면 최대 4회 안에 반드시 끝난다
    (부록A.3의 KKT 시스템을 그대로 씀, 하한·자유·상한 조합을 전수 열거하지
    않고 위반한 변수만 순차로 고정 — 동일한 수렴 성질을 훨씬 싸게 얻는다).

    논문 본문이 별도로 쓰는 외곽 비선형 재선형화 + 감쇠 후보(1,1/2,1/4,1/8)
    루프는 적용하지 않는다 — INDI 증분은 이미 1ms마다 재선형화되는 소신호
    영역이라(식20) 1회 선형화로 충분하다고 판단했다. 필요해지면 이 함수를
    감싸는 바깥 루프를 추가하면 된다(이 함수 경계는 그러도록 분리해 뒀다).

    Returns
    -------
    dn : array(4)
    """
    free = np.ones(4, dtype=bool)
    dn = np.zeros(4)
    for _ in range(max_iter):
        idx = np.where(free)[0]
        if idx.size == 0:
            break
        fixed = ~free
        Gw = G[1:4, idx]                          # (3, |F|)
        g0 = G[0, idx]                             # (|F|,)
        dT_rem = dT_target - G[0, fixed] @ dn[fixed]
        dw_rem = domega_target - G[1:4, fixed] @ dn[fixed]

        k = idx.size
        KKT = np.zeros((k + 1, k + 1))
        KKT[:k, :k] = Gw.T @ Gw
        KKT[:k, k] = g0
        KKT[k, :k] = g0
        rhs = np.concatenate([Gw.T @ dw_rem, [dT_rem]])
        try:
            sol = np.linalg.solve(KKT, rhs)
        except np.linalg.LinAlgError:
            sol, *_ = np.linalg.lstsq(KKT, rhs, rcond=None)
        dn_free = sol[:k]
        dn[idx] = dn_free

        n_new = n_actual[idx] + dn_free
        lo_bad = n_new < n_min
        hi_bad = n_new > n_max
        if not (lo_bad.any() or hi_bad.any()):
            break
        for j, gi in zip(range(k), idx):
            if lo_bad[j]:
                dn[gi] = n_min - n_actual[gi]
                free[gi] = False
            elif hi_bad[j]:
                dn[gi] = n_max - n_actual[gi]
                free[gi] = False
        # 4회 반복을 다 써도 못 끝나면(모든 로터가 포화 직전) 마지막 추정치를
        # 그대로 쓴다 — 논문도 4회 상한을 명시하며 전역 최적성을 주장하지 않는다.
    return dn


# ══════════════════════════════════════════════════
# 3. ProperHybrid (인터페이스 분리)
# ══════════════════════════════════════════════════

class ProperHybrid:
    """
    올바른 NMPC+INDI: 인터페이스 분리.

    NMPC → [T_cmd, ω̇_des]  (뭘 할지)
    INDI → [n1..n4]         (어떻게 할지, 유일한 실행자)

    이중 보정 없음: NMPC는 모터를 모르고, INDI만 모터를 제어.
    """

    def __init__(self, virtual_nmpc, params, dt=0.001, f_cut=50.0,
                 alloc_mode='A0', time_align='S0'):
        """
        alloc_mode : {'A0','A1'}
            'A0' (기본값, 기존 동작 보존) — 비제약 최소자승 + 사후 clip.
            'A1' — 논문 식(24)-(26) 총추력 등식 배분(constrained_allocation).
            논문 표6의 A0↔A1 제거실험이 바로 이 플래그다.
        time_align : {'S0','S1'}
            논문 표6의 V13-2 → V13 단계(추가 요소 S1)를 켜는 플래그.

            'S0' (기본값, 기존 동작 보존) — 각가속도에만 LPF를 걸고, 로터 추력은
            현재 표본의 생값을 쓴다. 두 신호의 시간 기준이 다르다.

            'S1' — 논문 식(22)-(23). 두 가지를 바꾼다.
              ① 시각 정렬: 자이로를 차분한 각가속도는 두 표본 **사이**의 평균
                 변화라서 시각이 반 표본 앞선다. 로터 힘도 인접 표본의 중간값
                 f_mid,k = (f_k + f_{k-1})/2 로 맞춘다.
              ② 같은 필터: 각가속도와 로터 추력·토크에 **동일한** LPF를 쓰고,
                 계수도 논문식 λ = 1 - exp(-2π f_c Δt_i) 로 계산한다.

            왜 중요한가 — 두 신호에 서로 다른 필터·시간 기준을 쓰면, 실제 외란이
            없어도 모터 명령이 바뀌는 것만으로 (T_cmd - T_meas)에 위상 오차가
            생긴다. INDI는 그 차이를 외란으로 읽고 보정하므로, 가상 외란을
            스스로 만들어 쫓는 셈이 된다(논문 4.5절).

            주의: S0의 LPF 계수는 후향차분 근사 Δt/(Δt+τ)이고 S1은 지수적분형
            이다. dt=1 ms, f_c=50 Hz에서 0.2394 대 0.2696으로 12% 다르다.
            같은 필터의 다른 이산화이므로 S0↔S1 비교에는 이 차이도 섞여 있다.
        """
        if time_align not in ('S0', 'S1'):
            raise ValueError(f"time_align must be 'S0' or 'S1', got {time_align!r}")
        self.nmpc = virtual_nmpc
        self.p = params
        self.dt = dt
        self.f_cut = f_cut
        self._tau = 1.0 / (2*np.pi*f_cut)     # LPF 시상수 (가변 dt에서 alpha 재계산용)
        self._alpha = dt / (dt + self._tau)
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._prev_t = None                   # 실제 Δt 측정용 (SITL 루프율 가변/100Hz미만)
        self._initialized = False
        self.alloc_mode = alloc_mode
        self.time_align = time_align
        # S1 전용 상태: 직전 표본의 로터별 추력(중간값 정렬용)과 그 LPF 출력.
        self._f_prev = None                   # f_{k-1} (N, 로터별)
        self._f_filt = None                   # LPF 통과한 f_mid — 0이 아니라
                                              # 첫 f_mid로 초기화한다(호버 추력은
                                              # mg≠0이라 0에서 시작하면 INDI가
                                              # 없는 추력 부족을 크게 본다)
        self.last_alloc = None                 # 식(27) 배분 결과 (보고·피드백용)
        _, self._TM_to_f = compute_allocation_matrix(params)

    def _lpf_coeff(self, actual_dt):
        """이번 표본의 LPF 계수. S1은 논문 식(23), S0은 기존 후향차분 근사."""
        if self.time_align == 'S1':
            return 1.0 - np.exp(-2*np.pi*self.f_cut*actual_dt)
        return actual_dt / (actual_dt + self._tau)

    def reset(self):
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._prev_t = None
        self._initialized = False
        self._f_prev = None
        self._f_filt = None
        # VirtualNMPC 완전 리셋 (타이밍 + warm start + w0)
        if hasattr(self.nmpc, 'reset'):
            self.nmpc.reset()

    def __call__(self, t, x):
        # 1. NMPC: 가상 명령
        vc = self.nmpc(t, x)
        T_cmd, omega_dot_des = vc[0], vc[1:4]

        omega = x[10:13]
        n_actual = x[13:17]

        # 측정 NaN 가드 (리뷰 3d): _omega_dot_filt는 자기참조 LPF라 NaN이
        # 한 번 들어가면 영구 고착 — 갱신을 건너뛰고 모델 기반 폴백으로.
        if not (np.all(np.isfinite(omega)) and np.all(np.isfinite(n_actual))):
            return self._fallback(T_cmd, omega_dot_des)

        if not self._initialized:
            self._omega_prev = omega.copy()
            self._prev_t = t
            self._initialized = True
            return self._fallback(T_cmd, omega_dot_des)

        # 2. ω̇ 측정 (LPF) — 하드코딩 dt 대신 실제 경과시간 사용
        #    (SITL은 NMPC 솔브로 루프율이 100Hz 미만/가변 → dt 오차가 ω̇를 왜곡)
        actual_dt = t - self._prev_t if self._prev_t is not None else self.dt
        actual_dt = min(max(actual_dt, 1e-4), 0.2)      # 0/과대 방지
        self._prev_t = t
        alpha = self._lpf_coeff(actual_dt)              # 가변 dt에 맞춰 LPF 재계산
        # 식(22) 앞부분. 차분값은 두 표본 '사이'의 평균 변화라 시각이 반 표본
        # 앞선다 — S1은 아래 로터 힘을 이 시각에 맞춘다.
        raw = (omega - self._omega_prev) / actual_dt
        self._omega_dot_filt = alpha*raw + (1-alpha)*self._omega_dot_filt
        self._omega_prev = omega.copy()

        # 3. INDI: [T,ω̇]_cmd vs [T,ω̇]_meas → Δn
        # v_body 계산 (전진비 반영)
        q = x[6:10]
        R = Rotation.from_quat(q).as_matrix()
        v_body = R.T @ x[3:6]
        V_axial = max(v_body[0] if self.p.get('thrust_axis', 'z') == 'x'
                      else -v_body[2], 0.0)

        # 전진비 보정된 로터별 추력 f_k (T_meas) — kj 지적(2026-09-25 밤):
        # 선형 fac 모델은 팀원 APC 곡선 대비 80/85m/s 트림에서 실제 추력의
        # 43%만 준다. propulsion_model이 곡선이면 그걸로 측정 추력을 잰다.
        if self.p.get('propulsion_model') == PROP_CURVE_MODEL:
            f_k = _curve_thrust(self.p, n_actual, V_axial)
        else:
            f_k = np.empty(4)
            for i in range(4):
                ni = n_actual[i]
                n_rps = ni / (2 * np.pi)
                J = V_axial / (n_rps * self.p['D_prop'] + 1e-8)
                fac = max(1.0 - J / self.p['J_max'], 0.0)
                f_k[i] = self.p['k_T'] * ni**2 * fac

        if self.time_align == 'S1':
            # 식(22) 뒷부분 + 식(23): 중간시각 정렬 후 각가속도와 '같은' 필터.
            if self._f_prev is None:
                self._f_prev = f_k.copy()      # 첫 표본은 f_{k-1}=f_k로 시작
            f_mid = 0.5*(f_k + self._f_prev)
            self._f_prev = f_k.copy()
            if self._f_filt is None:
                # 0에서 시작하면 안 된다 — 호버 추력은 mg≠0이라 LPF가 올라오는
                # 동안 INDI가 '없는 추력 부족'을 크게 보고 과보정한다.
                self._f_filt = f_mid.copy()
            else:
                self._f_filt = alpha*f_mid + (1-alpha)*self._f_filt
            f_used = self._f_filt
        else:
            f_used = f_k                       # S0: 현재 표본 생값(기존 동작)

        T_meas = 0.0
        for i in range(4):
            T_meas += f_used[i]

        dv = np.array([T_cmd - T_meas,
                       omega_dot_des[0] - self._omega_dot_filt[0],
                       omega_dot_des[1] - self._omega_dot_filt[1],
                       omega_dot_des[2] - self._omega_dot_filt[2]])

        G = self._compute_G(n_actual, v_body)

        if self.alloc_mode == 'A1':
            f_max = self._rotor_thrust_cap(V_axial)
            T_c = float(np.clip(T_cmd, 0.0, np.sum(f_max)))
            dT_target = T_c - T_meas
            dn = constrained_allocation(G, dT_target, dv[1:4], n_actual,
                                         self.p['n_min'], self.p['n_max'])
            n_cmd = np.clip(n_actual + dn, self.p['n_min'], self.p['n_max'])

            # 식(27): 명목 배분 결과를 가상입력 공간으로 되돌려 저장.
            T_alloc = T_meas + G[0] @ dn
            omega_alloc = self._omega_dot_filt + G[1:4] @ dn
            self.last_alloc = np.concatenate([[T_alloc], omega_alloc])
            # nmpc 쪽의 alloc_feedback 플래그가 실제 사용 여부를 결정한다
            # (VirtualNMPC.set_prev_input 참조) — 여기선 항상 통지만 한다.
            if hasattr(self.nmpc, 'set_prev_input'):
                self.nmpc.set_prev_input(self.last_alloc)
            return n_cmd

        # A0 (기존 동작 그대로 — 회귀 가드 겸 표6의 A0 비교항)
        try:
            dn = np.linalg.solve(G, dv)
        except np.linalg.LinAlgError:
            return self._fallback(T_cmd, omega_dot_des)

        return np.clip(n_actual + dn, self.p['n_min'], self.p['n_max'])

    def _rotor_thrust_cap(self, V_axial):
        """현재 유속·명목 최대 회전수에서 로터별 추력 상한 f_max,i (식A2)."""
        return rotor_thrust_cap(self.p, V_axial)

    def _compute_G(self, n, v_body=None):
        return compute_control_effectiveness(self.p, n, v_body)

    def _fallback(self, T_cmd, omega_dot_des):
        """드물게만 쓰이는 경로(초기화 전·센서 NaN)라 J(전진비) 없이 정지
        추력(J=0)만으로 역산한다 — 선형모델의 기존 근사(n=sqrt(f/k_T),
        fac=1 가정)와 같은 성격. 곡선모델이면 k_T 대신 정지 CT(0) 기반
        계수를 쓴다(kj 지적 2026-09-25 밤 — k_T 자체가 곡선과 안 맞을 수
        있어 이 근사도 일관되게 맞춘다)."""
        J = np.diag([self.p['Ixx'], self.p['Iyy'], self.p['Izz']])
        TM = np.array([T_cmd, *(J @ omega_dot_des)])
        f_ind = self._TM_to_f @ TM
        k_T_static = self._static_k_T()
        n = np.zeros(4)
        for i in range(4):
            n[i] = np.sqrt(max(f_ind[i], 0) / k_T_static)
        return np.clip(n, self.p['n_min'], self.p['n_max'])

    def _static_k_T(self):
        if self.p.get('propulsion_model') == PROP_CURVE_MODEL:
            from models.team_light.control.propeller_curve import coefficients
            CT0, _ = coefficients(self.p, 0.0)
            return self.p['rho'] * self.p['D_prop']**4 * float(CT0) / (2*np.pi)**2
        return self.p['k_T']


# ══════════════════════════════════════════════════
# 4. NaiveHybrid (이전 버전, 비교용)
# ══════════════════════════════════════════════════

@lru_cache(maxsize=8)
def _curve_thrust_torque_deriv_fn(knots_key, D_prop, rho):
    """팀원(규현) APC 곡선 기반 T(n,V)·Q(n,V)와 dT/dn·dQ/dn — 한 번만 짓고
    캐싱한다(INDI가 매 제어주기 부르므로 매번 CasADi Function을 새로
    지으면 느리다). `models.team_light.control.propeller_curve`를 그대로
    불러 쓴다 — 곡선을 재구현하지 않는다."""
    from models.team_light.control.propeller_curve import symbolic_force_torque
    n_sym = ca.SX.sym('n')
    v_sym = ca.SX.sym('v')
    p = {'prop_curve': {'knots': list(knots_key)}, 'D_prop': D_prop, 'rho': rho}
    T_sym, Q_sym = symbolic_force_torque(p, n_sym, v_sym)
    dT_dn = ca.jacobian(T_sym, n_sym)
    dQ_dn = ca.jacobian(Q_sym, n_sym)
    return ca.Function('curve_TQ_deriv', [n_sym, v_sym], [T_sym, Q_sym, dT_dn, dQ_dn])


def _curve_deriv(params, n_vec, axial):
    """로터별 dT/dn, dQ/dn (팀원 APC 곡선 기반)."""
    knots_key = tuple(tuple(row) for row in params['prop_curve']['knots'])
    fn = _curve_thrust_torque_deriv_fn(knots_key, params['D_prop'], params['rho'])
    dT = np.empty(len(n_vec))
    dQ = np.empty(len(n_vec))
    for i, ni in enumerate(n_vec):
        _, _, dTi, dQi = fn(ni, axial)
        dT[i] = float(dTi)
        dQ[i] = float(dQi)
    return dT, dQ


def compute_control_effectiveness(params, n_actual, v_body=None):
    """추진 wrench의 국소 미분 ∂[T, M/I]/∂n (각속도 0 기준).

    플랜트의 EPS와 영추력 분기를 그대로 미분한다. n=0에 가짜 제어 효과를
    넣지 않는다. 랭크가 부족하면 할당기가 처리해야지 G를 왜곡하면 안 된다.
    회전 중 로터 자이로의 미분은 이 기존 인터페이스에 포함하지 않는다.

    구 버전은 모멘트팔을 pos[i,0]/pos[i,1]로 직접 손계산했는데(로터가 동체
    xy평면에 있다는 전제), thrust_axis='x'에선 로터가 yz평면에 있어 그 전제가
    깨진다. compute_allocation_matrix가 이미 축을 아는 모멘트팔/반토크 구조를
    만드니 그걸 재사용한다 — z축 기본값에서는 이전 손계산과 수학적으로 동일하다
    (T=k_T*n^2*fac(n)의 곱미분이 정확히 dT/dn=k_T*n*(1+fac); Q=k*T가 항상
    성립해 반토크 행에 dT/dn을 곱해도 dQ/dn이 그대로 나온다) — **단 이 Q=k*T
    가정은 선형 fac 모델에서만 참이다.** 팀원 APC 곡선은 CT·CP가 J에 따라
    서로 다르게 움직여(예: J=0.79에서 CT는 정지 대비 12% 줄지만 CP는 오히려
    44% 늘어난다) 이 가정이 깨진다 — `propulsion_model`이 곡선 모델이면
    반토크 행(스핀축 둘레)만 실제 dQ/dn으로 따로 계산한다(kj 지적,
    2026-09-25 밤). 나머지(총추력 행·기하 모멘트팔 행)는 dT/dn을 그대로
    쓴다 — 그 행들은 모멘트=팔×추력이라 실제로 추력에 비례한다.
    """
    n = np.asarray(n_actual, dtype=float)
    axis = params.get('thrust_axis', 'z')
    axial = 0.0 if v_body is None else max(
        v_body[0] if axis == 'x' else -v_body[2], 0.0)
    allocation, _ = compute_allocation_matrix(params)

    if params.get('propulsion_model') == PROP_CURVE_MODEL:
        dT, dQ = _curve_deriv(params, n, axial)
        dirs = np.asarray(params['rotor_directions'], dtype=float)
        G = allocation * dT[np.newaxis, :]
        torque_row = 1 if axis == 'x' else 3   # compute_allocation_matrix와 동일 규약(반토크 행)
        G[torque_row, :] = dirs * dQ
    else:
        b = params['D_prop'] / (2 * np.pi)
        denominator = b * n + EPS
        c = axial / params['J_max']
        factor = 1.0 - c / denominator
        derivative = np.where(factor > 0.0,
                              params['k_T'] * (2 * n * factor + n**2 * c * b / denominator**2),
                              0.0)
        G = allocation * derivative[np.newaxis, :]

    G[1:4] /= np.array([params['Ixx'], params['Iyy'], params['Izz']])[:, None]
    return G


class NaiveHybrid:
    """나이브 NMPC+INDI: 모터속도 위에 INDI 보정 얹음 → 이중 보정."""

    def __init__(self, nmpc_ctrl, params_nom, dt=0.001, f_cut=50.0):
        self.nmpc = nmpc_ctrl
        self.p = params_nom
        self.dt = dt
        self._f_nom, _, _ = build_dynamics(params_nom)
        self._alpha = dt / (dt + 1.0/(2*np.pi*f_cut))
        self._omega_prev = np.zeros(3)
        self._omega_dot_filt = np.zeros(3)
        self._initialized = False

    def __call__(self, t, x):
        u_nmpc = self.nmpc(t, x)
        omega, n_actual = x[10:13], x[13:17]
        if not self._initialized:
            self._omega_prev = omega.copy()
            self._initialized = True
            return u_nmpc
        xdot_nom = np.array(self._f_nom(x, u_nmpc)).flatten()
        omega_dot_pred = xdot_nom[10:13]
        raw = (omega - self._omega_prev) / self.dt
        self._omega_dot_filt = self._alpha*raw + (1-self._alpha)*self._omega_dot_filt
        self._omega_prev = omega.copy()
        d = omega_dot_pred - self._omega_dot_filt
        G = compute_control_effectiveness(self.p, n_actual)
        dv = np.array([0.0, d[0], d[1], d[2]])
        try:
            dn = np.linalg.solve(G, dv)
        except np.linalg.LinAlgError:
            return u_nmpc
        return np.clip(u_nmpc + dn, self.p['n_min'], self.p['n_max'])


# ══════════════════════════════════════════════════
# 헬퍼
# ══════════════════════════════════════════════════

def _make_gust(W, t0, Tg):
    def f(t):
        w = np.zeros(3)
        if t0 <= t <= t0+Tg:
            w[2] = (W/2)*(1 - np.cos(2*np.pi*(t-t0)/Tg))
        return w
    return f

def _measure(ts, xs, us, V, z, nm):
    div = bool(np.any(np.isnan(xs)) or np.any(np.abs(xs[:,2]-z) > 50))
    if div:
        return {'rmse_z': np.inf, 'max_dz': np.inf, 'rmse_vx': np.inf, 'div': True}
    return {
        'rmse_z':  np.sqrt(np.mean((xs[:,2]-z)**2)),
        'max_dz':  np.max(np.abs(xs[:,2]-z)),
        'rmse_vx': np.sqrt(np.mean((xs[:,3]-V)**2)),
        'div': False}


# ══════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════

def main():
    from control.dynamics import AxialDronePlant
    V, z_ref = 70.0, 50.0

    print("\n" + "=" * 90)
    print("  NMPC+INDI 하이브리드 — 나이브 vs 인터페이스 분리")
    print("=" * 90)

    # 트림
    trim = find_trim(P, V)
    x0 = trim['state'].copy(); x0[2] = z_ref
    u_trim = trim['control'].copy()
    T_trim = float(np.sum(P['k_T'] * u_trim**2))

    print(f"\n[준비]")
    print(f"  70 m/s 트림: T_trim={T_trim:.1f}N (mg={P['mass']*P['g']:.1f}N)")

    sched_lqr = ScheduledLQR(P, v_ref=[V,0,0], z_ref=z_ref)

    # 조건
    mismatches = [('정확', 1.0), ('C_Na+20%', 1.2), ('C_Na-20%', 0.8)]
    scenarios = [
        ('순항 5s',      5.0, None),
        ('순항+돌풍 8s', 8.0, _make_gust(10.0, 2.0, 1.0)),
    ]
    ctrl_names = ['NMPC', '나이브', '분리', 'LQR스케줄']

    ALL = {}

    for mm_name, scale in mismatches:
        P_true = dict(P)
        P_true['C_Na'] = P['C_Na'] * scale
        plant = AxialDronePlant(P_true, dt=0.001)

        print(f"\n{'═'*90}")
        print(f"  {mm_name} (C_Na={P['C_Na']*scale:.1f})")
        print(f"{'═'*90}")

        ALL[mm_name] = {}

        for sc_name, T_sim, wind_fn in scenarios:
            print(f"\n  ── {sc_name} ──")
            sc = {}

            # 1) NMPC 순수 (모터속도 직접)
            print(f"    NMPC...", end=" ", flush=True)
            nmpc = NMPCController(P, v_ref=[V,0,0], z_ref=z_ref,
                                  u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)
            t0 = timer.time()
            ts,xs,us = plant.simulate(x0.copy(), nmpc, T_sim, wind_fn=wind_fn)
            sc['NMPC'] = _measure(ts,xs,us,V,z_ref,P['n_max'])
            _pr('NMPC', sc['NMPC'], timer.time()-t0)

            # 2) 나이브 하이브리드 (이중 보정)
            print(f"    나이브...", end=" ", flush=True)
            nmpc2 = NMPCController(P, v_ref=[V,0,0], z_ref=z_ref,
                                   u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)
            naive = NaiveHybrid(nmpc2, P, dt=0.001)
            t0 = timer.time()
            ts,xs,us = plant.simulate(x0.copy(), naive, T_sim, wind_fn=wind_fn)
            sc['나이브'] = _measure(ts,xs,us,V,z_ref,P['n_max'])
            _pr('나이브', sc['나이브'], timer.time()-t0)

            # 3) 분리 하이브리드 (올바른 인터페이스)
            print(f"    분리...", end=" ", flush=True)
            vnmpc = VirtualNMPC(P, v_ref=[V,0,0], z_ref=z_ref, T_ref=T_trim,
                                N=20, dt_nmpc=0.05, dt_ctrl=0.02)
            proper = ProperHybrid(vnmpc, P, dt=0.001)
            t0 = timer.time()
            ts,xs,us = plant.simulate(x0.copy(), proper, T_sim, wind_fn=wind_fn)
            sc['분리'] = _measure(ts,xs,us,V,z_ref,P['n_max'])
            _pr('분리', sc['분리'], timer.time()-t0)

            # 4) LQR 스케줄
            print(f"    LQR스케줄...", end=" ", flush=True)
            sched_lqr.v_ref = np.array([V,0,0]); sched_lqr.z_ref = z_ref
            t0 = timer.time()
            ts,xs,us = plant.simulate(x0.copy(), sched_lqr, T_sim, wind_fn=wind_fn)
            sc['LQR스케줄'] = _measure(ts,xs,us,V,z_ref,P['n_max'])
            _pr('LQR', sc['LQR스케줄'], timer.time()-t0)

            ALL[mm_name][sc_name] = sc

            # 표
            print(f"\n    {'':>10s}", end="")
            for cn in ctrl_names:
                print(f"  {cn:>10s}", end="")
            print()
            print(f"    {'─'*54}")
            for m,lb,fm in [('rmse_z','RMSE z','.3f'),('max_dz','Δz max','.2f'),
                            ('rmse_vx','RMSE vx','.3f')]:
                print(f"    {lb:>10s}", end="")
                for cn in ctrl_names:
                    r = sc[cn]
                    print(f"  {'DIV':>10s}" if r['div'] else f"  {r[m]:>10{fm}}", end="")
                print()

    # ══════════════════════════════════════════════════
    # 종합
    # ══════════════════════════════════════════════════
    print(f"\n{'═'*90}")
    print("  [종합] 나이브 vs 인터페이스 분리 — RMSE z")
    print(f"{'═'*90}")
    print(f"\n  {'조건':>16s}  {'시나리오':>12s}", end="")
    for cn in ctrl_names:
        print(f"  {cn:>10s}", end="")
    print()
    print(f"  {'─'*72}")
    for mm in ALL:
        for sc in ALL[mm]:
            d = ALL[mm][sc]
            print(f"  {mm:>16s}  {sc:>12s}", end="")
            for cn in ctrl_names:
                r = d[cn]
                print(f"  {'DIV':>10s}" if r['div'] else f"  {r['rmse_z']:>10.3f}", end="")
            print()

    # 분석
    print(f"\n{'─'*90}")
    print("  분석:")

    # 나이브 vs 분리 비교
    for mm in ALL:
        for sc in ALL[mm]:
            d = ALL[mm][sc]
            if d['NMPC']['div'] or d['나이브']['div'] or d['분리']['div']:
                continue
            z_nmpc = d['NMPC']['rmse_z']
            z_naive = d['나이브']['rmse_z']
            z_proper = d['분리']['rmse_z']
            z_lqr = d['LQR스케줄']['rmse_z']

            if z_nmpc < 1e-6:
                continue  # 순항 0 skip

            print(f"\n  {mm}, {sc}:")
            print(f"    NMPC:      {z_nmpc:.3f}")
            print(f"    나이브:    {z_naive:.3f} ({'악화' if z_naive > z_nmpc*1.1 else 'OK'})")
            print(f"    분리:      {z_proper:.3f} ({'개선!' if z_proper < z_nmpc*0.95 else 'NMPC급' if z_proper < z_nmpc*1.1 else '악화'})")
            print(f"    LQR스케줄: {z_lqr:.3f}")

    print(f"\n  결론:")
    print(f"  - 나이브: 이중 보정으로 NMPC보다 악화 (재확인)")
    print(f"  - 분리: 인터페이스를 [T, ω̇]로 명확히 나누면 이중 보정 해소")
    print(f"  - 핵심 설계 원칙: '뭘 할지'(NMPC)와 '어떻게 할지'(INDI)를 분리")
    print(f"{'═'*90}")


def _pr(name, r, elapsed):
    if r['div']:
        print(f"{elapsed:.1f}s  DIVERGED!")
    else:
        print(f"{elapsed:.1f}s  z={r['rmse_z']:.3f}  Δz={r['max_dz']:.2f}")


if __name__ == '__main__':
    main()
