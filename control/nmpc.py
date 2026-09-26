"""
Nonlinear MPC — CasADi + IPOPT (Direct Multiple Shooting)
==========================================================

acados 없이 CasADi의 IPOPT 솔버로 NMPC 구현.
수학적으로 acados와 동일한 formulation:
  - Multiple shooting discretization (RK4)
  - 비선형 동역학 예측 모델
  - 모터 속도 제약 명시적 처리
  - 예측 지평선(horizon) 내 최적 제어 시퀀스

PID/LQR 대비 NMPC의 핵심 이점:
  1) 미래 예측 → 선제적 제어 (틸트 전환 등)
  2) 제약 처리 → 모터 포화 방지
  3) 비선형 모델 → 고속 영역에서도 정확한 예측
"""

import numpy as np
import casadi as ca
import time as timer

from control.dynamics import build_dynamics, NX, NU, EPS
from control.nmpc_common import ipopt_options, cost_weights as nmpc_cost_weights

# 양추력 하한의 여유: 하한 회전수에서 전진비가 추력 0 전진비보다 조금 작아 추력이 0이 아니라 양수다
ZERO_THRUST_MARGIN = 1e-3


def zero_thrust_advance_ratio(params):
    """추력이 0이 되는 전진비 J0 — 팀 곡선이면 양추력 한계(곡선의 근), 옛 선형 모델이면 J_max."""
    from models.team_light.control.propeller_curve import positive_thrust_j_limit, uses_curve
    return positive_thrust_j_limit(params) if uses_curve(params) else float(params['J_max'])


def positive_thrust_rate_floor(params, x, j0=None):
    """현재 상태 x에서 명목 모델로 본 **양추력 회전수** [rad/s] — 이보다 느리면 로터 추력이 0이다.

    kj 결정(2026-09-26 오후, 경기장 보고서 8.7-1): M17(로터 회전수 입력) 명령의 하한으로 쓴다.
    V13·F13은 NLP 입력이 추력이라 '추력 ≥ 0'이 상자 제약이다(V13 총추력 T ≥ 0, F13 로터별 f ≥ 0).
    M17은 회전수로 계획해서 같은 제약이 추진 곡선의 추력 0 평탄 구간(fmax(Ct,0), 기울기 0)이 되고,
    계획이 그 구간에 들어가면 IPOPT가 막혔다(results/arena/M17_DIAGNOSIS.md, H4). 이 하한은 그
    '추력 ≥ 0'을 회전수 좌표로 옮긴 것이다.

    쓰는 정보는 관측 상태(관성 속도 — 바람은 모른다)와 명목 제어기 모델뿐이다.
      va    = 동체 x축(추력축) 속도, 음수면 0
      J0    = 추력이 0이 되는 전진비(zero_thrust_advance_ratio)
      floor = 2π·va/(D·J0)·(1 + 여유), 단 0.999·n_max를 넘지 않는다
    진단 V4(control/arena_m17_diagnosis.py::SolverProxy)와 같은 식·같은 계산 순서다. 결과를 비트
    단위로 재현하려고 그대로 옮겼다. 한계: 예측 구간 내내 이 값(현재 상태 기준)으로 고정한다.
    팀 propeller_curve.supported_rate_bounds는 작동 범위 하한(10k rpm)까지 더하지만, 그것은 모델
    영역의 한계라(V13·F13에도 없다) 넣지 않는다.
    """
    from scipy.spatial.transform import Rotation
    if j0 is None:
        j0 = zero_thrust_advance_ratio(params)
    va = max(float((Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6])[0]), 0.0)
    return min(2*np.pi*va/(params['D_prop']*j0)*(1 + ZERO_THRUST_MARGIN), 0.999*float(params['n_max']))


def positive_thrust_floor_function(params, j0=None):
    """positive_thrust_rate_floor의 CasADi 판 — 상태 x(17) → 양추력 회전수. 노드별 하한(NLP)과 테스트가 같이 쓴다.

    kj 결정(2026-09-26 저녁): M17 하한을 예측 노드마다 U_k ≥ floor(X_k)로 둔다(F13의 노드별 f ≥ 0과 대칭).
    측정 상태 하나로 구간 전체를 고정하던 방식(positive_thrust)은 예측 상태의 축방향 속도가 측정값보다
    커지면 뒤 노드가 추력 0 평탄 구간에 들어갔다(results/arena/M17_PROBE.md).

    축방향 속도 v_a = R(q)[:,0]·v(쿼터니언 정규화, scalar-last)다. numpy 도우미와 달리 0에서 자르지
    않는다 — v_a < 0이면 floor < 0이라 U ≥ floor가 저절로 만족해, 제약으로서는 같다. 상한 0.999·n_max로
    자르는 것은 같다(v_a가 아주 커도 U ≤ n_max와 모순되지 않게).
    """
    if j0 is None:
        j0 = zero_thrust_advance_ratio(params)
    x = ca.SX.sym('x', NX)
    q = x[6:10]/ca.sqrt(ca.sumsqr(x[6:10]))
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    body_x = ca.vertcat(1 - 2*(qy**2 + qz**2), 2*(qx*qy + qz*qw), 2*(qx*qz - qy*qw))
    va = ca.dot(body_x, x[3:6])
    floor = ca.fmin(2*np.pi*va/(params['D_prop']*j0)*(1 + ZERO_THRUST_MARGIN), 0.999*float(params['n_max']))
    return ca.Function('positive_thrust_floor', [x], [floor])


class NMPCController:
    """
    Direct Multiple Shooting NMPC.

    사용법:
        nmpc = NMPCController(params, v_ref=[83,0,0], z_ref=50)
        u = nmpc(t, x)  # plant.simulate()와 호환
    """

    def __init__(self, params, v_ref=None, z_ref=0.0, u_ref=None,
                 N=20, dt_nmpc=0.05, dt_ctrl=0.02, Q_z=20.0,
                 electrical_constraints=False, V_b=None,
                 cost_spec='paper', ref_fn=None, max_iter=30, tol=1e-4,
                 cost_weights=None, rotor_floor=None):
        """
        Parameters
        ----------
        rotor_floor : None | 'positive_thrust' | 'positive_thrust_per_node'
            양추력 하한(논문 모드만). 기본값 None이면 기존과 비트 동일하다.
            'positive_thrust'        매 풀이마다 모든 노드의 회전수 하한(lbx)을 **측정 상태**의 양추력
                                     회전수로 올린다(2026-09-26 오후 — 진단 V4·스모크 e0f3581 재현용).
            'positive_thrust_per_node' 노드마다 U_k ≥ floor(**예측 상태 X_k**)를 부등식 제약으로 둔다
                                     (2026-09-26 저녁 kj 결정, F13의 노드별 f ≥ 0과 대칭).
        max_iter, tol, cost_weights : control/nmpc_common.py 참고.
            V13·F13과 같은 함수로 IPOPT 옵션·비용 가중치를 만들고 보관한다
            (경기장 불변식 I-3). 예전엔 max_iter가 30으로 박혀 있어 이 클래스만
            설정할 수 없었다. 기본값은 기존과 비트 단위로 같다.
        N        : int    예측 지평선 스텝 수 (N * dt_nmpc = 예측 시간)
        dt_nmpc  : float  NMPC 내부 적분 스텝 [s]
        dt_ctrl  : float  제어 주기 (이 간격마다 NLP 재풀이) [s]
        electrical_constraints : bool
            모터 전류·전압 제약을 예측 격자마다 부과할지.
            ⚠ 논문 v5.3 범위 밖 — 132행이 "전류·전압 한계와 배터리 전압
            강하는 모델에 포함하지 않는다 … 고정된 회전수 한계로 대체한다"고
            명시한다(7.4절 한계에도 재기술). 기본값 False를 유지할 것.
            실기(PX4) 쪽에서 쓸 수 있어 코드는 남겨 둔다.
            기본값 False로 기존 M17 동작·테스트를 보존한다(표6의 E0).
            True가 실제 논문 M17(E1) — 이게 표6의 E0↔E1 제거실험 플래그다.
        V_b : float or None
            예측 구간 내내 고정해서 쓸 버스 전압(논문 범위 밖: "예측 전압은
            현재 측정된 V_b로 유지, SOC의 미래 변화는 직접 예측하지 않음").
            None이면 params['V_oc'](만충 가정)로 시작하고, 매 호출 전
            self.V_b를 갱신해 실제 배터리 상태를 반영할 수 있다.
        """
        self.p = params
        self.N = N
        self.dt_nmpc = dt_nmpc
        self.dt_ctrl = dt_ctrl
        self.nx, self.nu = NX, NU
        self.electrical_constraints = electrical_constraints
        self.V_b = V_b if V_b is not None else params.get('V_oc', 44.4)

        self.v_ref = np.array(v_ref) if v_ref is not None else np.zeros(3)
        self.z_ref = z_ref
        self._Q_z = Q_z
        # cost_spec : {'paper','legacy'}  (2026-09-25 밤, 야간지시 3-a)
        #   'paper' (기본값) — 논문 v5.3 식(14)-(18)의 정확한 전사. M17(비분리)의
        #   입력은 로터 속도 자체이므로 Dν 정규화는 research/nmpc.py 가 kind="nmpc"
        #   에 쓰는 척도(max_n)를 그대로 따른다 — V13 의 [mg,100,100,100] 과는
        #   다른 척도지만 같은 원칙(식15: 물리량마다 자기 한계로 무차원화)이다.
        #   'legacy' — 이 클래스가 원래 쓰던 비용. 옛 결과 재현용으로만 남긴다.
        #
        #   2026-09-24에 V13 만 'paper' 로 기본값을 뒤집어 M17·F13 과 비대칭이
        #   됐었다 — 이 파일이 그 비대칭을 없앤다. 차이(legacy 기준):
        #     속도가중    5·I₃(식14)              diag(5,5,10)      vz만 2배
        #     입력편차    0.02/max_n²             1e-4              척도 자체가 다름
        #     입력변화    0.1/max_n²              1e-3              척도 자체가 다름
        #     종말가중    stage 전체×10(식16)      v·z만×10          ω 누락
        #     참조        노드별 r_{j|k}(식14)     상수 1개          ref_fn 없으면 동일
        if cost_spec not in ('paper', 'legacy'):
            raise ValueError(f"cost_spec must be 'paper' or 'legacy', got {cost_spec!r}")
        if ref_fn is not None and cost_spec != 'paper':
            raise ValueError("ref_fn은 cost_spec='paper'에서만 쓸 수 있다.")
        self.cost_spec = cost_spec
        self.ref_fn = ref_fn
        if rotor_floor not in (None, 'positive_thrust', 'positive_thrust_per_node'):
            raise ValueError("rotor_floor must be None, 'positive_thrust' or 'positive_thrust_per_node', "
                             f"got {rotor_floor!r}")
        if rotor_floor is not None and cost_spec != 'paper':
            raise ValueError("rotor_floor는 cost_spec='paper'에서만 쓸 수 있다.")
        self.rotor_floor = rotor_floor
        self._floor_j0 = zero_thrust_advance_ratio(params) if rotor_floor else None
        self.last_lbx = None          # 마지막 풀이에 실제로 넘긴 하한(테스트가 읽는다)
        self.last_solution = None     # 마지막 풀이의 결정변수 w(테스트·계측이 읽는다)
        self._max_iter = max_iter
        self.ipopt_options = ipopt_options(max_iter, tol)
        self.cost_weights = nmpc_cost_weights(cost_weights)
        self.soft_constraints = {}
        self._solve_log = []
        if u_ref is not None:
            self.u_ref = np.array(u_ref)
        else:
            n_hov = np.sqrt(params['mass'] * params['g'] / (4 * params['k_T']))
            self.u_ref = np.full(4, n_hov)

        # RK4 적분기 (예측용)
        f, x_sym, u_sym = build_dynamics(params)
        if self.cost_spec == 'paper':
            self.F = self._build_rk4_substeps(f, x_sym, u_sym, substeps=5)
            self._build_nlp_paper(params)
        else:
            self._build_rk4(f, x_sym, u_sym)
            self._build_nlp(params)

        # 상태
        self._last_t = -np.inf
        self._t_now = 0.0
        self._u_current = self.u_ref.copy()
        self._cold_start = True   # V13와 동일 콜드스타트 웜스타트 보정 플래그
        # _w0_init는 _build_nlp()에서 설정됨

    def reset(self):
        """MC 시행 간 독립성 보장을 위한 완전 리셋."""
        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()
        self._cold_start = True
        if self._w0_init is not None:
            self.w0 = self._w0_init.copy()

    def solver_settings(self):
        """경기장 불변식 I-3 비교용 요약(control/nmpc_common.solver_settings)."""
        from control.nmpc_common import solver_settings
        return solver_settings(self)

    def _build_rk4(self, f, x_sym, u_sym):
        """예측용 RK4 한 스텝 함수."""
        dt = self.dt_nmpc
        k1 = f(x_sym, u_sym)
        k2 = f(x_sym + dt/2 * k1, u_sym)
        k3 = f(x_sym + dt/2 * k2, u_sym)
        k4 = f(x_sym + dt * k3, u_sym)
        x_next = x_sym + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        self.F = ca.Function('F_rk4', [x_sym, u_sym], [x_next])

    def _build_rk4_substeps(self, f, x_sym, u_sym, substeps=5):
        """논문 4.2절: 예측 격자당 RK4 5회 세부적분 + 쿼터니언 정규화.

        control/hybrid_comparison.py::VirtualNMPC._make_substep_integrator 와
        같은 패턴(그쪽 주석 참고 — 실효 스텝을 줄여 빠른 자세 변화의 적분
        오차를 줄이고, 정규화 없이 RK4 만 쓰면 쿼터니언 노름이 서서히 벌어진다).
        `ca.norm_2`의 0 근처 나눗셈 특이점도 그쪽과 동일하게 eps로 정칙화
        (kj 지적, 2026-09-25 저녁 — NMPC 계열 전체 동일 적용).
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
            st = ca.vertcat(st[0:6], st[6:10]/q_norm, st[10:17])
        return ca.Function('F_paper', [x_sym, u_sym], [st])

    def _build_nlp_paper(self, params):
        """논문 v5.3 식(14)-(18)의 직접 전사 — M17(비분리, 로터속도 입력).

        VirtualNMPC._build_nlp_paper 와 나란히 읽을 수 있게 같은 구조로 쓴다.
        차이는 상태가 17D(로터 4개 포함)이고 입력이 로터 속도 자체라는 것뿐 —
        Dν 정규화 척도가 research/nmpc.py 의 kind="nmpc" 와 같은 max_n 하나다
        (표5 kind별 척도: nmpc→max_n, f13→mg/4, hybrid→[mg,100,100,100]).
        """
        N, nx, nu = self.N, self.nx, self.nu
        n_max = params['n_max']
        n_hov = self.u_ref[0]   # __init__ 에서 이미 트림/명시값으로 계산됨

        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J_cost = 0.0

        # p 레이아웃: [x_meas(17), refs(4·(N+1))]  — refs 는 노드별 (vx,vy,vz,z).
        p = ca.SX.sym('p', nx + 4*(N+1))
        x_meas = p[0:nx]
        refs = ca.reshape(p[nx:nx + 4*(N+1)], 4, N+1)

        def new_state(k):
            X = ca.SX.sym(f'X_{k}', nx)
            w.append(X)
            lbw.extend([-1e6]*nx); ubw.extend([1e6]*nx)
            guess = [0.0]*nx
            guess[6:10] = [1.0, 0.0, 0.0, 0.0]
            guess[13:17] = [float(n_hov)]*4
            w0.extend(guess)
            return X

        X_k = new_state(0)
        g.append(X_k - x_meas)
        lbg.extend([0.0]*nx); ubg.extend([0.0]*nx)

        W = self.cost_weights          # 논문 식(14)·(16)·(17) 가중치(nmpc_common)
        U_prev = ca.DM(np.full(nu, n_hov))
        floor_fn = (positive_thrust_floor_function(params, self._floor_j0)
                    if self.rotor_floor == 'positive_thrust_per_node' else None)
        for k in range(N):
            e_v = X_k[3:6] - refs[0:3, k]
            e_z = X_k[2] - refs[3, k]
            J_cost += (W['w_v']*ca.sumsqr(e_v) + self._Q_z*e_z**2
                       + W['w_omega']*ca.sumsqr(X_k[10:13]))

            U_k = ca.SX.sym(f'U_{k}', nu)
            w.append(U_k)
            lbw.extend([params['n_min']]*nu); ubw.extend([n_max]*nu)
            w0.extend([float(n_hov)]*nu)
            if floor_fn is not None:
                # 노드별 양추력 하한: 로터마다 U_k ≥ floor(X_k). X_0은 측정 상태와 같다(아래 등식).
                g.append(U_k - floor_fn(X_k))
                lbg.extend([0.0]*nu); ubg.extend([ca.inf]*nu)

            J_cost += W['r_dev']*ca.sumsqr((U_k - n_hov)/n_max)
            J_cost += W['r_rate']*ca.sumsqr((U_k - U_prev)/n_max)

            X_next = new_state(k+1)
            g.append(X_next - self.F(X_k, U_k))
            lbg.extend([0.0]*nx); ubg.extend([0.0]*nx)
            X_k, U_prev = X_next, U_k

        e_v = X_k[3:6] - refs[0:3, N]
        e_z = X_k[2] - refs[3, N]
        J_cost += W['terminal']*(W['w_v']*ca.sumsqr(e_v) + self._Q_z*e_z**2
                                 + W['w_omega']*ca.sumsqr(X_k[10:13]))

        nlp = {'f': J_cost, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': p}
        self.solver = ca.nlpsol('nmpc_paper', 'ipopt', nlp, dict(self.ipopt_options))
        self.lbw = np.array(lbw); self.ubw = np.array(ubw)
        self.lbg = np.array(lbg); self.ubg = np.array(ubg)
        self.w0 = np.array(w0); self._w0_init = self.w0.copy()

    def _reference_horizon(self):
        if self.ref_fn is None:
            one = np.concatenate([self.v_ref, [self.z_ref]])
            return np.tile(one[:, None], (1, self.N + 1))
        cols = [np.asarray(self.ref_fn(self._t_now + k*self.dt_nmpc), dtype=float)
               for k in range(self.N + 1)]
        return np.stack(cols, axis=1)

    def _build_nlp(self, params):
        """Multiple shooting NLP 구성."""
        N, nx, nu = self.N, self.nx, self.nu

        # ── 비용 가중치 ──
        Q_v  = np.diag([5.0, 5.0, 10.0])       # 속도 추종
        Q_z  = self._Q_z                          # 고도 (외부 조정 가능)
        Q_w  = np.diag([1.0, 1.0, 1.0])         # 각속도 (안정성)
        R    = np.eye(nu) * 0.0001               # 제어 편차 (트림 대비)
        R_du = np.eye(nu) * 0.001                # 제어 변화율

        n_min = params['n_min']
        n_max = params['n_max']

        # ── 파라미터: [x_init(17), v_ref(3), z_ref(1), u_ref(4), V_b(1)] = 26 ──
        # V_b: 전기제약(electrical_constraints)용 — 예측 구간 내내 고정해서 쓰는
        # 현재 측정 버스전압(논문 범위 밖). 제약을 안 쓸 때도 자리만 차지, 계산엔 안 씀.
        p = ca.SX.sym('p', nx + 3 + 1 + nu + 1)
        x_init = p[0:nx]
        v_ref  = p[nx:nx+3]
        z_ref  = p[nx+3]
        u_ref  = p[nx+4:nx+4+nu]
        V_b_param = p[nx+4+nu]

        # ── 결정 변수 + 제약 ──
        w, w0, lbw, ubw = [], [], [], []
        g, lbg, ubg = [], [], []
        J = 0.0

        X_prev = x_init
        U_prev = u_ref

        for k in range(N):
            # 제어 입력 (결정 변수)
            U_k = ca.SX.sym(f'U_{k}', nu)
            w.append(U_k)
            lbw += [n_min] * nu
            ubw += [n_max] * nu
            w0  += [600.0] * nu

            # 다음 상태 (결정 변수, shooting node)
            X_k = ca.SX.sym(f'X_{k}', nx)
            w.append(X_k)
            lbw += [-1e6] * nx
            ubw += [1e6] * nx
            _xg = [0.0] * nx
            _xg[6:10] = [1.0, 0.0, 0.0, 0.0]   # 유효 단위 쿼터니언(호버) — 무효 [0,0,0,0] 초기추측 방지
            w0  += _xg

            # 동역학 제약: X_k = F(X_prev, U_k)
            X_pred = self.F(X_prev, U_k)
            g.append(X_k - X_pred)
            lbg += [0.0] * nx
            ubg += [0.0] * nx

            # ── 전류·전압 제약 (논문 v5.3 범위 밖, electrical_constraints=True일 때만) ──
            # M17만의 특징 — 모터 상태(X_k[13:17]=n)와 명령(U_k=n_c)이 이미
            # 결정변수에 있어 추가 상태 없이 바로 부과할 수 있다.
            if self.electrical_constraints:
                n_i = X_k[13:17]
                Q_i = self.p['k_Q'] * n_i**2   # 마찰 Q_f=0 (battery.py와 동일 가정)
                I_req = (Q_i / self.p['k_t']
                         + self.p['I_rotor'] * (U_k - n_i)
                           / (self.p['k_t'] * self.p['tau_m']))
                g.append(I_req)
                lbg += [0.0] * 4
                ubg += [float(self.p['I_lim'])] * 4
                g.append(V_b_param - (self.p['k_e'] * n_i + self.p['R_m'] * I_req))
                lbg += [0.0] * 4
                ubg += [1e6] * 4

            # ── 스테이지 비용 ──
            e_v = X_k[3:6] - v_ref
            e_z = X_k[2] - z_ref
            e_u = U_k - u_ref
            dU  = U_k - U_prev

            J += e_v.T @ Q_v @ e_v           # 속도 추종
            J += Q_z * e_z**2                  # 고도
            J += X_k[10:13].T @ Q_w @ X_k[10:13]  # 각속도 안정
            J += e_u.T @ R @ e_u               # 제어 편차
            J += dU.T @ R_du @ dU              # 제어 변화율

            X_prev = X_k
            U_prev = U_k

        # ── 종단 비용 (10배 강화) ──
        e_v_N = X_prev[3:6] - v_ref
        e_z_N = X_prev[2] - z_ref
        J += 10 * (e_v_N.T @ Q_v @ e_v_N)
        J += 10 * Q_z * e_z_N**2

        # ── NLP 솔버 생성 ──
        w_cat = ca.vertcat(*w)
        g_cat = ca.vertcat(*g)

        nlp = {'f': J, 'x': w_cat, 'g': g_cat, 'p': p}
        opts = dict(self.ipopt_options)
        self.solver = ca.nlpsol('nmpc', 'ipopt', nlp, opts)

        self.lbw = np.array(lbw, dtype=float)
        self.ubw = np.array(ubw, dtype=float)
        self.lbg = np.array(lbg, dtype=float)
        self.ubg = np.array(ubg, dtype=float)
        self.w0  = np.array(w0, dtype=float)
        self._w0_init = self.w0.copy()

    def __call__(self, t, x):
        """dt_ctrl 주기마다 NLP를 풀고 첫 제어 반환."""
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            self._t_now = t
            self._u_current = self._solve(x)
            self._last_t = t
        return self._u_current

    def _solve(self, x_current):
        """NLP 풀이 → 첫 제어 추출."""
        if self.cost_spec == 'paper':
            if self._cold_start:
                # control/hybrid_comparison.py::VirtualNMPC._solve와 동일한
                # 콜드스타트 웜스타트 보정(kj 지적, 2026-09-25 저녁) — 같은
                # 수정을 NMPC 계열 전체에 동일하게 적용한다(V13만 고치지
                # 않는다). X_k(상태) 부분만 실측값으로 덮어쓰고 U_k(로터
                # 속도) 추측값은 이미 n_hov라 그대로 둔다.
                stride = self.nu + self.nx
                for k in range(self.N + 1):
                    off = k * stride
                    self.w0[off:off + self.nx] = x_current
                self._cold_start = False
            p_val = np.concatenate([x_current, self._reference_horizon().ravel(order='F')])
        else:
            p_val = np.concatenate([x_current, self.v_ref, [self.z_ref], self.u_ref,
                                     [self.V_b]])

        # 측정 상태 하한(positive_thrust)만 lbx를 올린다. 노드별 하한은 NLP 안의 부등식 제약이다.
        lbx = self._rotor_floor_bounds(x_current) if self.rotor_floor == 'positive_thrust' else self.lbw
        self.last_lbx = lbx
        sol = self.solver(
            x0=self.w0, lbx=lbx, ubx=self.ubw,
            lbg=self.lbg, ubg=self.ubg, p=p_val)

        self._solve_log.append(self.solver.stats().get('return_status', 'unknown'))
        w_opt = np.array(sol['x']).flatten()
        self.last_solution = w_opt
        stride = self.nu + self.nx   # 21

        if self.cost_spec == 'paper':
            # 배치: X_0, U_0, X_1, U_1, …, X_{N-1}, U_{N-1}, X_N (x0 가 결정변수)
            u_opt = w_opt[self.nx:self.nx + self.nu]
            self.w0 = np.concatenate([w_opt[stride:], w_opt[-stride:-self.nx],
                                      w_opt[-self.nx:]])
        else:
            # 배치: U_0, X_0, U_1, X_1, … (legacy)
            u_opt = w_opt[0:self.nu]
            self.w0 = np.concatenate([w_opt[stride:], w_opt[-stride:]])

        return np.clip(u_opt, self.p['n_min'], self.p['n_max'])

    def _rotor_floor_bounds(self, x_current):
        """U 칸 하한을 양추력 회전수로 올린 lbx **사본** — self.lbw는 그대로 둔다.

        제자리에서 고치면 하한이 풀이마다 누적돼 한 번 올라간 값이 내려오지 않는다. 배치는 논문 모드의
        [X_0, U_0, X_1, U_1, …, U_{N-1}, X_N]이다. 진단 V4와 같은 순서로 만든다(비트 재현).
        """
        floor = positive_thrust_rate_floor(self.p, np.asarray(x_current, dtype=float), self._floor_j0)
        lbx = np.array(self.lbw, dtype=float).ravel()
        stride = self.nx + self.nu
        for k in range(self.N):
            sl = slice(k*stride + self.nx, k*stride + self.nx + self.nu)
            lbx[sl] = np.maximum(lbx[sl], floor)
        return lbx

    def get_solve_stats(self):
        """IPOPT 풀이 통계."""
        if not self._solve_log:
            return {}
        n = len(self._solve_log)
        ok = sum(1 for s in self._solve_log if s == 'Solve_Succeeded')
        return {'n_solves': n, 'n_ok': ok, 'pct_ok': 100.0 * ok / n}


# ══════════════════════════════════════════════════════
# 비교 시뮬레이션
# ══════════════════════════════════════════════════════

def run_nmpc_comparison():
    from control.vehicle_params import vehicle_params as P
    from control.dynamics import AxialDronePlant
    from control.trim import find_trim, print_trim
    from control.controller import CascadedPID, LQRController

    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    print("\n" + "=" * 70)
    print("  NMPC vs PID vs LQR 비교")
    print("=" * 70)

    # 비교 속도 포인트
    test_speeds = [30, 50, 70, 85]
    T_sim = 10.0

    # 30 m/s 기준 게인 (PID/LQR)
    trim_30 = find_trim(P, 30.0)
    pid_base_gains = True   # 30 m/s 게인 고정
    lqr_30 = LQRController(P, trim_30['state'], trim_30['control'])
    K_fixed = lqr_30.K.copy()
    K_r_fixed = lqr_30.K_r.copy()
    n_reduced = lqr_30.n_reduced

    results = []

    for V in test_speeds:
        print(f"\n{'─'*70}")
        print(f"  V = {V} m/s ({V*3.6:.0f} km/h)")

        trim = find_trim(P, V)
        if trim['residual'] > 1e-3:
            print(f"  트림 실패, 건너뜀")
            continue

        print_trim(trim, V, P)
        x_trim = trim['state']
        u_trim = trim['control']

        # 초기 상태: 트림 + 교란
        x0 = x_trim.copy()
        x0[2] = 50.0
        x0[5] += 2.0
        x0[3] += 1.0

        # ── PID (30 m/s 게인 고정) ──
        pid = CascadedPID(P, v_ref=[V, 0, 0], z_ref=50.0, dt=dt)
        pid.reset()
        t0 = timer.time()
        ts, xs_pid, us_pid = plant.simulate(x0.copy(), pid, T_sim)
        t_pid = timer.time() - t0

        # ── LQR (30 m/s K 고정) ──
        lqr = LQRController.__new__(LQRController)
        lqr.p = P
        lqr.K = K_fixed
        lqr.K_r = K_r_fixed
        lqr.n_reduced = n_reduced
        lqr.valid = True
        lqr.x_trim = x_trim.copy()
        lqr.u_trim = u_trim.copy()
        lqr.x_trim[2] = 50.0
        lqr.x_trim[0:2] = x0[0:2]

        t0 = timer.time()
        ts, xs_lqr, us_lqr = plant.simulate(x0.copy(), lqr, T_sim)
        t_lqr = timer.time() - t0

        # ── NMPC ──
        nmpc = NMPCController(P, v_ref=[V, 0, 0], z_ref=50.0,
                              u_ref=u_trim, N=20, dt_nmpc=0.05, dt_ctrl=0.02)
        t0 = timer.time()
        ts, xs_nmpc, us_nmpc = plant.simulate(x0.copy(), nmpc, T_sim)
        t_nmpc = timer.time() - t0

        # ── 결과 ──
        def metrics(xs, us, label):
            rmse_vx = np.sqrt(np.mean((xs[:, 3] - V)**2))
            rmse_z  = np.sqrt(np.mean((xs[:, 2] - 50.0)**2))
            sat = 100 * np.mean(us >= 0.95 * P['n_max'])
            return rmse_vx, rmse_z, sat

        m_pid  = metrics(xs_pid, us_pid, 'PID')
        m_lqr  = metrics(xs_lqr, us_lqr, 'LQR')
        m_nmpc = metrics(xs_nmpc, us_nmpc, 'NMPC')

        print(f"\n  {'':>12s}  {'PID':>10s}  {'LQR':>10s}  {'NMPC':>10s}")
        print(f"  {'─'*46}")
        print(f"  {'RMSE vx':>12s}  {m_pid[0]:10.3f}  {m_lqr[0]:10.3f}  {m_nmpc[0]:10.3f}")
        print(f"  {'RMSE z':>12s}  {m_pid[1]:10.3f}  {m_lqr[1]:10.3f}  {m_nmpc[1]:10.3f}")
        print(f"  {'SAT %':>12s}  {m_pid[2]:10.1f}  {m_lqr[2]:10.1f}  {m_nmpc[2]:10.1f}")
        print(f"  {'계산시간':>12s}  {t_pid:10.2f}s  {t_lqr:10.2f}s  {t_nmpc:10.2f}s")

        results.append({
            'V': V, 'pid': m_pid, 'lqr': m_lqr, 'nmpc': m_nmpc,
            'time': (t_pid, t_lqr, t_nmpc)
        })

    # ── 종합 요약 ──
    print(f"\n{'='*70}")
    print("  NMPC 종합 요약")
    print(f"{'='*70}")
    print(f"\n  {'V':>5s}  {'--- RMSE vx ---':^30s}  {'--- RMSE z ---':^30s}")
    print(f"  {'m/s':>5s}  {'PID':>8s} {'LQR':>8s} {'NMPC':>8s}"
          f"  {'PID':>8s} {'LQR':>8s} {'NMPC':>8s}")
    print(f"  {'─'*65}")
    for r in results:
        print(f"  {r['V']:5.0f}  {r['pid'][0]:8.3f} {r['lqr'][0]:8.3f} {r['nmpc'][0]:8.3f}"
              f"  {r['pid'][1]:8.3f} {r['lqr'][1]:8.3f} {r['nmpc'][1]:8.3f}")

    print(f"\n  결론:")
    print(f"  - NMPC: 비선형 예측 + 제약 처리로 전 속도 영역에서 최적 성능")
    print(f"  - PID: 튜닝 속도에서 OK, 고속에서 성능 저하")
    print(f"  - LQR: 최적 게인이지만 선형화 모델의 한계")
    print(f"{'='*70}")


if __name__ == '__main__':
    run_nmpc_comparison()
