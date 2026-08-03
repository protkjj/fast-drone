"""
AcadosVirtualNMPC — VirtualNMPC(13D)의 acados SQP-RTI 이식
==========================================================

왜 acados인가 (2026-08-04 실측, results/bench_*.txt):
  - IPOPT: warm 20.6ms / 미션 p95 39ms → 실기 20ms 예산 불가. jit 후에도 최악 58.5ms
  - CasADi sqpmethod 1-iter "가난한 RTI"는 실패 (exact Hessian 부정정칙)
  - 진짜 RTI = Gauss-Newton(항상 PSD) + OCP 구조 QP(HPIPM) = acados

실행 주의 (macOS):
  dyld는 프로세스 시작 시점의 환경만 읽으므로 반드시 셸에서 지정해야 함:
      DYLD_LIBRARY_PATH=$HOME/acados/lib python3 vnmpc_acados.py
  (libacados.dylib에 LC_RPATH 없음 — 과거 qpOASES rpath 이슈의 재발 형태.
   파이썬 코드 안에서 os.environ으로 설정하는 것은 무효.)

설계:
  - VirtualNMPC(IPOPT)와 동일 인터페이스: __call__(t, x17) → [T, ν_ω(3)],
    reset(), consec_fail/last_status → ProperHybrid·HybridWithFallback 무수정 주입
  - 비용 가중치 VirtualNMPC._build_nlp와 동일. 단 Δu 페널티는 v1에서 생략
    (acados NONLINEAR_LS는 Δu 직접 표현 불가 — 채터링 여부는 폐루프
     모터 명령 플롯으로 검증하고, 문제 시 상태확장으로 구현 예정)
  - cost_scaling=1로 고정: acados 기본은 스테이지 비용에 dt_k를 곱하는데
    IPOPT판은 무스케일 합이므로 의미를 맞춤
  - 비균일 그리드: time_steps 인자 (근거리 촘촘·원거리 성글게 — 첫 스텝을
    dt_ctrl과 일치시키는 것이 의도된 설계)
  - legacy/nmpc_acados.py의 검증된 패턴 이식: yref 런타임 파라미터화(솔버
    1회 빌드), 트림/호버 전노드 웜스타트, 해 시프트, status≠0 시 부분해+NaN 가드
  - codegen 산출물은 ~/.cache/fast_drone_acados/ (iCloud 동기화 폴더 오염 방지)

게이트 1 (이 파일 __main__): 30/50/70/85 m/s 10s 순항 — NaN 없이 수렴.
"""
import os
import time as timer

import numpy as np
import casadi as ca

os.environ.setdefault('ACADOS_SOURCE_DIR', os.path.expanduser('~/acados'))

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

from hybrid_comparison import build_virtual_dynamics, NX_V, NU_V

CODE_DIR_BASE = os.path.expanduser('~/.cache/fast_drone_acados')


class AcadosVirtualNMPC:
    """acados SQP-RTI 가상 명령 NMPC. VirtualNMPC 대체 주입용."""

    def __init__(self, params, v_ref=None, z_ref=0.0, T_ref=None,
                 N=20, dt_nmpc=0.05, time_steps=None, dt_ctrl=0.02,
                 hpipm_mode='BALANCE', qp_cond_N=None,
                 levenberg_marquardt=1e-2, regularize_method=None,
                 rate_aug=True, du_weight_scale=1.0,
                 nlp_solver_type='SQP_RTI', nlp_max_iter=5,
                 T_min=0.0, qp_iter_max=20, integrator='ERK',
                 as_rti_iter=None, as_rti_level=None,
                 code_suffix='default'):
        """
        Parameters
        ----------
        time_steps : array or None
            비균일 그리드 [s]. None이면 균일 N×dt_nmpc.
        hpipm_mode : 'SPEED' | 'BALANCE' | 'ROBUST'
        qp_cond_N : int or None
            partial condensing 지평선 (None = acados 기본).
        levenberg_marquardt : float
            GN 헤시안 정규화 (legacy 검증값 1e-2).
        regularize_method : None | 'PROJECT' | 'MIRROR' | 'CONVEXIFY'
            NaN 재발 시 시도할 손잡이.
        nlp_solver_type : 'SQP_RTI' | 'SQP'
            RTI(1-iter)가 감속 대천이에서 발산하면 'SQP'+nlp_max_iter로
            수 회 반복 (문헌 표준: 드론레이싱 LMPC는 SQP max 5 iter).
        nlp_max_iter : int
            nlp_solver_type='SQP'일 때 반복 상한.
        T_min : float
            총추력 하한 [N]. 감속 중 최적화기가 T→0 근처로 내려가면
            모터 n→0에서 INDI G(∝n) 특이화 → 제어권한 상실 → 텀블
            (2026-08-04 플롯 진단). 0보다 크게 주면 특이 영역 진입 차단.
        rate_aug : bool
            Δu(변화율) 페널티를 상태확장으로 구현 (x=[x13,u], 입력=u̇).
            **기본 True** — 없으면 감속 과도구간에서 명령 bang-bang →
            폐루프 발산 실측 (2026-08-04 진단: |ω| 37 rad/s 텀블링).
            IPOPT판 R_du와 등가가 되도록 변화율 가중치 = R_du·dt_k².
        du_weight_scale : float
            변화율 가중치 배율 (1.0 = IPOPT R_du 등가).
        code_suffix : str
            codegen 디렉토리 구분자 (그리드 변형별로 다르게 줄 것).
        """
        self.p = params
        self.dt_ctrl = dt_ctrl
        self.v_ref = (np.array(v_ref, dtype=float)
                      if v_ref is not None else np.zeros(3))
        self.z_ref = float(z_ref)
        self.T_ref = float(T_ref) if T_ref else params['mass'] * params['g']
        self.u_ref = np.array([self.T_ref, 0.0, 0.0, 0.0])

        if time_steps is None:
            time_steps = np.full(N, dt_nmpc)
        self.time_steps = np.asarray(time_steps, dtype=float)
        self.N = len(self.time_steps)

        self.rate_aug = bool(rate_aug)
        self.du_weight_scale = float(du_weight_scale)
        self.nlp_solver_type = nlp_solver_type
        self.nlp_max_iter = int(nlp_max_iter)
        self.qp_iter_max = int(qp_iter_max)
        self.integrator = integrator
        self.as_rti_iter = as_rti_iter
        self.as_rti_level = as_rti_level
        self.nx_solver = NX_V + (NU_V if self.rate_aug else 0)
        self._u_state = self.u_ref.copy()      # rate_aug: 적분된 현재 명령
        T_max = 4 * params['k_T'] * params['n_max']**2
        self._u_lb = np.array([float(T_min), -100.0, -100.0, -100.0])
        self._u_ub = np.array([T_max, 100.0, 100.0, 100.0])

        self._build_solver(params, hpipm_mode, qp_cond_N,
                           levenberg_marquardt, regularize_method, code_suffix)

        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()
        self.consec_fail = 0
        self.last_status = -1
        self._ever_converged = False
        self.solve_times = []
        self.statuses = []
        self._init_trajectory()

    # ══════════════════════════════════════════════
    # 빌드
    # ══════════════════════════════════════════════

    def _build_solver(self, params, hpipm_mode, qp_cond_N, lm,
                      regularize_method, code_suffix):
        f, x13_sym, u4_sym = build_virtual_dynamics(params)

        model = AcadosModel()
        model.name = f'vnmpc_{code_suffix}'

        base_W = [5.0, 5.0, 10.0,          # Q_v
                  20.0,                     # Q_z
                  1.0, 1.0, 1.0,            # Q_w
                  1e-5, 1e-3, 1e-3, 1e-3]   # R (u 편차)

        if self.rate_aug:
            # ── Δu 상태확장: x=[x13, u4], 입력=u̇ ──
            # Δu 페널티 없이는 감속 과도구간에서 명령 bang-bang → 발산 (실측).
            # IPOPT판의 節間 페널티 dUᵀR_du dU와 등가: u̇가 노드에서 일정하므로
            # dU = u̇·dt_k → 가중치 = R_du·dt_k² (노드별, 빌드 후 cost_set).
            xa = ca.SX.sym('xa', NX_V + NU_V)
            du = ca.SX.sym('du', NU_V)
            x13, u4 = xa[0:NX_V], xa[NX_V:NX_V + NU_V]
            model.x = xa
            model.u = du
            model.f_expl_expr = ca.vertcat(f(x13, u4), du)
            y_expr = ca.vertcat(xa[3:6], xa[2], xa[10:13], u4, du)   # 15
            y_e_expr = ca.vertcat(xa[3:6], xa[2], xa[10:13])
            W = np.diag(base_W + [1e-7, 1e-5, 1e-5, 1e-5])  # 임시 — 노드별 재설정
        else:
            model.x = x13_sym
            model.u = u4_sym
            model.f_expl_expr = f(x13_sym, u4_sym)
            y_expr = ca.vertcat(model.x[3:6], model.x[2],
                                model.x[10:13], model.u)             # 11
            y_e_expr = ca.vertcat(model.x[3:6], model.x[2], model.x[10:13])
            W = np.diag(base_W)

        ocp = AcadosOcp()
        ocp.model = model
        ocp.model.cost_y_expr = y_expr
        ocp.model.cost_y_expr_e = y_e_expr
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.cost.W = W
        # 종단 = 10×(Q_v, Q_z) — IPOPT판과 동일. ω는 수치용 미소값
        ocp.cost.W_e = np.diag([50.0, 50.0, 100.0, 200.0, 1e-6, 1e-6, 1e-6])
        self._ny = W.shape[0]
        ocp.cost.yref = np.zeros(self._ny)
        ocp.cost.yref_e = np.zeros(7)

        # ── 제약 ──
        if self.rate_aug:
            # 명령 bounds는 상태 박스 (노드 1..N + 종단), 입력은 변화율 bounds
            ocp.constraints.idxbx = np.arange(NX_V, NX_V + NU_V)
            ocp.constraints.lbx = self._u_lb.copy()
            ocp.constraints.ubx = self._u_ub.copy()
            ocp.constraints.idxbx_e = np.arange(NX_V, NX_V + NU_V)
            ocp.constraints.lbx_e = self._u_lb.copy()
            ocp.constraints.ubx_e = self._u_ub.copy()
            rate_max = np.array([2000.0, 5000.0, 5000.0, 5000.0])
            ocp.constraints.lbu = -rate_max
            ocp.constraints.ubu = rate_max
            ocp.constraints.idxbu = np.arange(NU_V)
            x0 = np.zeros(self.nx_solver)
            x0[6] = 1.0
            x0[NX_V:] = self.u_ref
        else:
            ocp.constraints.lbu = self._u_lb.copy()
            ocp.constraints.ubu = self._u_ub.copy()
            ocp.constraints.idxbu = np.arange(NU_V)
            x0 = np.zeros(NX_V)
            x0[6] = 1.0                              # 호버 쿼터니언 qx=1
        ocp.constraints.x0 = x0

        # ── 솔버 옵션 ──
        so = ocp.solver_options
        so.N_horizon = self.N
        so.tf = float(np.sum(self.time_steps))
        so.time_steps = self.time_steps
        so.cost_scaling = np.ones(self.N + 1)        # IPOPT판과 의미 일치
        so.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        so.hpipm_mode = hpipm_mode
        so.hessian_approx = 'GAUSS_NEWTON'
        so.integrator_type = self.integrator
        so.sim_method_num_stages = 4
        so.sim_method_num_steps = 3
        if self.integrator == 'IRK':
            xdot_sym = ca.SX.sym('xdot', model.x.shape[0])
            model.xdot = xdot_sym
            model.f_impl_expr = xdot_sym - model.f_expl_expr
        if self.as_rti_iter is not None:
            so.as_rti_iter = int(self.as_rti_iter)
        if self.as_rti_level is not None:
            so.as_rti_level = int(self.as_rti_level)
        so.nlp_solver_type = self.nlp_solver_type
        if self.nlp_solver_type == 'SQP':
            so.nlp_solver_max_iter = self.nlp_max_iter
            # 반복 수렴 판정은 느슨하게 (제어 용도 — IPOPT판 tol과 동급)
            so.nlp_solver_tol_stat = 1e-3
            so.nlp_solver_tol_eq = 1e-3
            so.nlp_solver_tol_ineq = 1e-3
            so.nlp_solver_tol_comp = 1e-3
        so.levenberg_marquardt = float(lm)
        so.qp_solver_iter_max = self.qp_iter_max
        so.qp_solver_warm_start = 1
        if qp_cond_N is not None:
            so.qp_solver_cond_N = int(qp_cond_N)
        if regularize_method is not None:
            so.regularize_method = regularize_method

        os.makedirs(CODE_DIR_BASE, exist_ok=True)
        ocp.code_export_directory = os.path.join(CODE_DIR_BASE,
                                                 f'c_{code_suffix}')
        json_path = os.path.join(CODE_DIR_BASE, f'ocp_{code_suffix}.json')
        try:
            self.solver = AcadosOcpSolver(ocp, json_file=json_path)
        except OSError as e:
            if ('rpath' in str(e) or 'qpOASES' in str(e)
                    or 'Library not loaded' in str(e)):
                raise OSError(
                    'acados dylib 로드 실패 — macOS는 프로세스 시작 시점에 '
                    '환경을 지정해야 함:\n'
                    '  DYLD_LIBRARY_PATH=$HOME/acados/lib python3 <스크립트>'
                ) from e
            raise

        if self.rate_aug:
            # 변화율 가중치 = R_du·dt_k² (IPOPT판 節間 Δu 페널티와 등가, 노드별)
            R_DU = np.array([1e-4, 0.01, 0.01, 0.01]) * self.du_weight_scale
            base = np.array(base_W)
            for k in range(self.N):
                Wk = np.diag(np.concatenate(
                    [base, R_DU * self.time_steps[k]**2]))
                self.solver.cost_set(k, 'W', Wk)

    # ══════════════════════════════════════════════
    # 상태 관리
    # ══════════════════════════════════════════════

    def _init_trajectory(self):
        """전 노드 웜스타트: 현재 기준값의 호버/순항 근방 (legacy 검증 패턴)."""
        x_ws = np.zeros(self.nx_solver)
        x_ws[2] = self.z_ref
        x_ws[3:6] = self.v_ref
        x_ws[6] = 1.0
        if self.rate_aug:
            x_ws[NX_V:] = self.u_ref
        u_node = np.zeros(NU_V) if self.rate_aug else self.u_ref
        for k in range(self.N + 1):
            self.solver.set(k, 'x', x_ws)
        for k in range(self.N):
            self.solver.set(k, 'u', u_node)

    def reset(self):
        """MC 시행 간 독립성 보장 (VirtualNMPC.reset과 동일 의미)."""
        self._last_t = -np.inf
        self._u_current = self.u_ref.copy()
        self._u_state = self.u_ref.copy()
        self.consec_fail = 0
        self.last_status = -1
        self._ever_converged = False
        self.solve_times = []
        self.statuses = []
        self._init_trajectory()

    def set_refs(self, v_ref, z_ref, T_ref=None):
        """기준값 일괄 변경 (속도 스윕용). yref는 다음 솔브에서 반영."""
        self.v_ref = np.array(v_ref, dtype=float)
        self.z_ref = float(z_ref)
        if T_ref is not None:
            self.T_ref = float(T_ref)
            self.u_ref = np.array([self.T_ref, 0.0, 0.0, 0.0])

    def _push_refs(self):
        """v_ref/z_ref → yref 반영 (MissionController가 속성을 바꾸므로 매 솔브)."""
        yref = np.zeros(self._ny)
        yref[0:3] = self.v_ref
        yref[3] = self.z_ref
        yref[7:11] = self.u_ref            # rate_aug면 11:15(u̇ 기준)는 0
        for k in range(self.N):
            self.solver.cost_set(k, 'yref', yref)
        self.solver.cost_set(self.N, 'yref', yref[:7])

    def _shift(self):
        """해 시프트 웜스타트 (오름차순이라 get이 set보다 먼저 읽힘)."""
        for k in range(self.N - 1):
            self.solver.set(k, 'x', self.solver.get(k + 1, 'x'))
            self.solver.set(k, 'u', self.solver.get(k + 1, 'u'))
        self.solver.set(self.N - 1, 'x', self.solver.get(self.N, 'x'))

    def _reinit_from(self, x13):
        """실패 회복: 현재 측정 상태로 전 노드 재시드.

        QP 실패/NaN 후에 해 시프트를 하면 오염된 궤적이 웜스타트로 남아
        연쇄 실패(cascade)가 됨 — 실측: 재시드 없이는 한 번의 NaN이
        이후 전 솔브를 무너뜨림 (grid 비교에서 status OK 1067/3250).
        """
        seed = (np.concatenate([x13, self._u_state])
                if self.rate_aug else x13)
        u_node = np.zeros(NU_V) if self.rate_aug else self.u_ref
        for k in range(self.N + 1):
            self.solver.set(k, 'x', seed)
        for k in range(self.N):
            self.solver.set(k, 'u', u_node)

    # ══════════════════════════════════════════════
    # 솔브
    # ══════════════════════════════════════════════

    def __call__(self, t, x_full):
        """17D 플랜트 상태 → [T, ν_ω] (VirtualNMPC와 동일 시그니처)."""
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            x13 = np.concatenate([x_full[0:10], x_full[10:13]])
            self._u_current = self._solve(x13)
            self._last_t = t
        return self._u_current

    def _solve(self, x13):
        self._push_refs()
        x0s = (np.concatenate([x13, self._u_state])
               if self.rate_aug else x13)
        self.solver.set(0, 'lbx', x0s)
        self.solver.set(0, 'ubx', x0s)

        t0 = timer.perf_counter()
        status = self.solver.solve()
        self.solve_times.append(timer.perf_counter() - t0)
        self.statuses.append(status)

        # status 처리: 0=수렴, 2=max iter 도달(반복해 사용 가능 — IPOPT도
        # max_iter 해를 그대로 쓰는 관행) → 둘 다 "소프트 성공"으로 해 사용+시프트.
        # 그 외(1/4 = NaN/QP실패)만 하드 실패로 재시드+consec_fail.
        # ⚠ 과거 버그: status 2를 실패 취급 → 매 솔브 웜스타트 파괴 →
        #   SQP 모드가 이륙부터 연쇄 NaN (규명: 2026-08-04)
        self.last_status = status
        raw = self.solver.get(0, 'u')
        ok = status in (0, 2) and np.all(np.isfinite(raw))
        if ok:
            self.consec_fail = 0
            self._ever_converged = True
        elif self._ever_converged:
            self.consec_fail += 1

        if self.rate_aug:
            if ok:
                # 적용 명령 = 이전 명령 + u̇₀·dt_ctrl (변화율 적분)
                self._u_state = np.clip(self._u_state + raw * self.dt_ctrl,
                                        self._u_lb, self._u_ub)
                self._shift()
            else:
                # 실패: u_ref(호버 추력·ν=0)로 지수 감쇠 — 마지막 명령을
                # 그대로 동결하면 나쁜 명령(예: ν=-78)이 고착됨 (플롯 실측)
                self._u_state = 0.9 * self._u_state + 0.1 * self.u_ref
                self._reinit_from(x13)
            if np.any(~np.isfinite(self._u_state)):
                self._u_state = self.u_ref.copy()
            return self._u_state.copy()

        if ok:
            self._shift()
            return raw
        if np.any(~np.isfinite(raw)):
            raw = self.u_ref.copy()
        self._reinit_from(x13)              # 오염 궤적 폐기 → 자가 회복
        return raw

    def presolve(self, x_full, n=3):
        """arm 전 워밍업 (cold 첫 솔브 대책 — 실기/SITL 필수 관행)."""
        x13 = np.concatenate([x_full[0:10], x_full[10:13]])
        for _ in range(n):
            self._solve(x13)
        del self.solve_times[-n:]
        del self.statuses[-n:]

    def solve_split(self, x13):
        """RTI preparation/feedback 분리 — 벤치마크용.

        실기 배치에서는 preparation을 상태 측정 전에 미리 돌리므로
        실효 제어 지연은 feedback 시간만이다. Returns (t_prep, t_fb, status).
        """
        self._push_refs()
        x0s = (np.concatenate([x13, self._u_state])
               if self.rate_aug else x13)
        self.solver.set(0, 'lbx', x0s)
        self.solver.set(0, 'ubx', x0s)

        self.solver.options_set('rti_phase', 1)
        t0 = timer.perf_counter()
        self.solver.solve()
        t_prep = timer.perf_counter() - t0

        self.solver.options_set('rti_phase', 2)
        t0 = timer.perf_counter()
        status = self.solver.solve()
        t_fb = timer.perf_counter() - t0

        self.solver.options_set('rti_phase', 0)
        self._shift()
        return t_prep, t_fb, status

    def get_stats(self):
        if not self.solve_times:
            return {}
        st = np.array(self.solve_times)
        ok = sum(1 for s in self.statuses if s == 0)
        soft = sum(1 for s in self.statuses if s == 2)
        return {
            'median_ms': float(np.median(st) * 1e3),
            'p95_ms': float(np.percentile(st, 95) * 1e3),
            'max_ms': float(np.max(st) * 1e3),
            'n_solves': len(st),
            'n_ok': ok,
            'n_maxiter': soft,          # status 2 (반복해 사용)
        }


# ══════════════════════════════════════════════════
# 게이트 1: 전 속도 수렴 스모크
# ══════════════════════════════════════════════════

if __name__ == '__main__':
    from vehicle_params import vehicle_params as P
    from dynamics import AxialDronePlant
    from trim import find_trim
    from hybrid_comparison import ProperHybrid

    print("게이트 1 — acados RTI 전 속도 수렴 (10s 순항, ProperHybrid 폐루프)")
    print("=" * 70)

    plant = AxialDronePlant(P, dt=0.001)
    t0 = timer.perf_counter()
    vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=50.0,
                           code_suffix='gate')
    print(f"솔버 빌드: {timer.perf_counter()-t0:.1f} s (1회, yref로 속도 변경)\n")

    all_pass = True
    for V in [30.0, 50.0, 70.0, 85.0]:
        trim = find_trim(P, V)
        if trim['residual'] > 1e-4:
            print(f"  V={V:.0f}: 트림 실패 — 건너뜀")
            continue
        T_trim = float(np.sum(P['k_T'] * trim['control']**2))
        x0 = trim['state'].copy()
        x0[2] = 50.0
        x0[5] += 2.0                       # 수직 교란 (과거 발산 유발 조건)

        vn.set_refs([V, 0, 0], 50.0, T_trim)
        vn.reset()
        hyb = ProperHybrid(vn, P, dt=plant.dt)

        ts, xs, us = plant.simulate(x0, hyb, 10.0)
        st = vn.get_stats()
        nan = bool(np.any(np.isnan(xs)))
        rmse_z = float(np.sqrt(np.mean((xs[:, 2] - 50.0)**2)))
        rmse_vx = float(np.sqrt(np.mean((xs[:, 3] - V)**2)))
        ok = (not nan) and st['n_ok'] == st['n_solves'] and rmse_z < 2.0
        all_pass &= ok
        print(f"  V={V:3.0f} m/s | rmse_z={rmse_z:6.3f} rmse_vx={rmse_vx:6.3f}"
              f" | solve {st['median_ms']:.2f}/{st['p95_ms']:.2f}/{st['max_ms']:.2f} ms"
              f" (med/p95/max) | status OK {st['n_ok']}/{st['n_solves']}"
              f" | NaN={nan} | {'PASS' if ok else 'FAIL'}")

    print("\n게이트 1:", "ALL PASS" if all_pass else "FAIL — 정규화 조정 필요")
