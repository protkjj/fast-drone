"""
NMPC (acados) v3 — 전 속도 수렴 버전
=====================================

핵심 수정:
  v_ref/u_ref를 비용 함수에 상수로 넣지 않고,
  yref 런타임 파라미터로 처리 → 솔버 1회 생성, 속도별 재빌드 불필요.
  + full SQP + LM 정규화 + 트림 warm start.
"""

import os
os.environ['ACADOS_SOURCE_DIR'] = os.path.expanduser('~/acados')
os.environ['DYLD_LIBRARY_PATH'] = os.path.expanduser('~/acados/lib')
os.environ['LD_LIBRARY_PATH'] = os.path.expanduser('~/acados/lib')

import numpy as np
import casadi as ca
import time as timer
import shutil

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from dynamics import build_dynamics, NX, NU


def build_ocp_solver(params, N=20, T_horizon=1.0, use_rti=False):
    """
    솔버를 한 번만 생성. yref로 기준 변경 가능.

    비용 잔차:
      스테이지 y = [v(3), z(1), omega(3), u(4)] = 11
      종단   y_e = [v(3), z(1), omega(3)] = 7
    yref = [v_ref, z_ref, 0,0,0, u_ref] 로 런타임에 설정
    """
    f_func, x_sym, u_sym = build_dynamics(params)

    model = AcadosModel()
    model.name = 'mq'
    model.x = x_sym
    model.u = u_sym
    model.f_expl_expr = f_func(x_sym, u_sym)

    ocp = AcadosOcp()
    ocp.model = model
    nx, nu = NX, NU
    x, u = model.x, model.u

    # ── 비용: y = [v, z, omega, u] (기준값 없이 원래 상태) ──
    ocp.model.cost_y_expr = ca.vertcat(x[3:6], x[2], x[10:13], u)
    ocp.model.cost_y_expr_e = ca.vertcat(x[3:6], x[2], x[10:13])

    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    ocp.cost.W = np.diag([5, 5, 10, 20, 1, 1, 1,
                           0.0001, 0.0001, 0.0001, 0.0001])
    ocp.cost.W_e = np.diag([50, 50, 100, 200, 5, 5, 5])

    # yref는 런타임에 설정 (여기선 더미)
    ocp.cost.yref = np.zeros(11)
    ocp.cost.yref_e = np.zeros(7)

    # ── 제약 ──
    ocp.constraints.lbu = np.full(nu, params['n_min'])
    ocp.constraints.ubu = np.full(nu, params['n_max'])
    ocp.constraints.idxbu = np.arange(nu)
    ocp.constraints.x0 = np.zeros(nx)

    # ── 솔버 옵션 ──
    ocp.solver_options.tf = T_horizon
    ocp.solver_options.N_horizon = N

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.levenberg_marquardt = 1e-2

    if use_rti:
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.nlp_solver_max_iter = 1
    else:
        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.nlp_solver_max_iter = 50

    ocp.solver_options.qp_solver_iter_max = 100

    code_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'c_generated_code')
    ocp.code_export_directory = code_dir

    solver = AcadosOcpSolver(ocp, json_file=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'acados_ocp.json'))

    return solver, N


class AcadosNMPC:
    """acados NMPC. 솔버를 공유하고 yref만 변경."""

    def __init__(self, solver, N, params, v_ref, z_ref, u_ref, x_trim,
                 dt_ctrl=0.02):
        self.solver = solver
        self.N = N
        self.p = params
        self.dt_ctrl = dt_ctrl

        # yref 설정: [v_ref(3), z_ref(1), 0,0,0(omega), u_ref(4)] = 11
        yref = np.concatenate([v_ref, [z_ref], [0,0,0], u_ref])
        yref_e = np.concatenate([v_ref, [z_ref], [0,0,0]])

        for k in range(N):
            solver.cost_set(k, 'yref', yref)
        solver.cost_set(N, 'yref', yref_e)

        # 전 노드 트림 warm start
        x_ws = x_trim.copy()
        x_ws[2] = z_ref
        for k in range(N+1):
            solver.set(k, 'x', x_ws)
        for k in range(N):
            solver.set(k, 'u', u_ref)

        self.u_ref = np.array(u_ref)
        self._last_t = -np.inf
        self._u_current = u_ref.copy()
        self.solve_times = []
        self.statuses = []

    def __call__(self, t, x):
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            self._u_current = self._solve(x)
            self._last_t = t
        return self._u_current

    def _solve(self, x_current):
        t0 = timer.time()

        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        status = self.solver.solve()
        self.solve_times.append(timer.time() - t0)
        self.statuses.append(status)

        if status == 0:
            u_opt = self.solver.get(0, 'u')
        else:
            # 실패 시: 솔버의 현재 해(부분 수렴)를 사용하되 fallback
            u_opt = self.solver.get(0, 'u')
            if np.any(np.isnan(u_opt)):
                u_opt = self.u_ref.copy()

        # warm start shift
        for k in range(self.N - 1):
            self.solver.set(k, 'x', self.solver.get(k+1, 'x'))
            self.solver.set(k, 'u', self.solver.get(k+1, 'u'))
        self.solver.set(self.N-1, 'u', self.solver.get(self.N-1, 'u'))
        self.solver.set(self.N, 'x', self.solver.get(self.N, 'x'))

        return np.clip(u_opt, self.p['n_min'], self.p['n_max'])

    def get_stats(self):
        if not self.solve_times:
            return {}
        st = np.array(self.solve_times)
        ok = sum(1 for s in self.statuses if s == 0)
        return {
            'mean_ms': np.mean(st) * 1000,
            'max_ms':  np.max(st) * 1000,
            'n_solves': len(st),
            'n_ok': ok,
        }


# ══════════════════════════════════════════════════════
# 비교 실행
# ══════════════════════════════════════════════════════

def run_comparison():
    from vehicle_params import vehicle_params as P
    from dynamics import AxialDronePlant
    from trim import find_trim, print_trim
    from controller import CascadedPID, LQRController

    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    # 30 m/s 기준 게인 (PID/LQR)
    trim_30 = find_trim(P, 30.0)
    K_fixed = LQRController(P, trim_30['state'], trim_30['control']).K.copy()

    # ── 솔버 1회 생성 ──
    print("acados 솔버 빌드 중...")
    solver, N = build_ocp_solver(P, N=20, T_horizon=1.0, use_rti=False)
    print("빌드 완료.\n")

    test_speeds = [30, 50, 70, 85]
    T_sim = 10.0

    print("=" * 70)
    print("  acados NMPC (v3: 공유 솔버 + yref 파라미터) vs PID vs LQR")
    print("=" * 70)

    all_results = []

    for V in test_speeds:
        print(f"\n{'─'*70}")
        print(f"  V = {V} m/s ({V*3.6:.0f} km/h)")

        trim = find_trim(P, V)
        if trim['residual'] > 1e-3:
            print("  트림 실패"); continue
        print_trim(trim, V, P)

        x_trim, u_trim = trim['state'], trim['control']
        x0 = x_trim.copy()
        x0[2] = 50.0
        x0[5] += 2.0
        x0[3] += 1.0

        # PID
        pid = CascadedPID(P, v_ref=[V,0,0], z_ref=50.0, dt=dt)
        pid.reset()
        t0 = timer.time()
        _, xs_pid, us_pid = plant.simulate(x0.copy(), pid, T_sim)
        t_pid = timer.time() - t0

        # LQR
        lqr = LQRController.__new__(LQRController)
        lqr.p, lqr.K, lqr.valid = P, K_fixed, True
        lqr.x_trim = x_trim.copy(); lqr.u_trim = u_trim.copy()
        lqr.x_trim[2] = 50.0; lqr.x_trim[0:2] = x0[0:2]
        t0 = timer.time()
        _, xs_lqr, us_lqr = plant.simulate(x0.copy(), lqr, T_sim)
        t_lqr = timer.time() - t0

        # NMPC (솔버 재사용, yref만 변경)
        try:
            nmpc = AcadosNMPC(solver, N, P,
                              v_ref=np.array([V,0,0]), z_ref=50.0,
                              u_ref=u_trim, x_trim=x_trim,
                              dt_ctrl=0.02)
            t0 = timer.time()
            _, xs_nmpc, us_nmpc = plant.simulate(x0.copy(), nmpc, T_sim)
            t_nmpc = timer.time() - t0
            stats = nmpc.get_stats()
            nmpc_ok = True
        except Exception as e:
            print(f"  NMPC 오류: {e}")
            xs_nmpc, us_nmpc = xs_pid, us_pid
            t_nmpc, stats, nmpc_ok = 0, {}, False

        # 결과
        def m(xs, us):
            return (np.sqrt(np.mean((xs[:,3]-V)**2)),
                    np.sqrt(np.mean((xs[:,2]-50)**2)),
                    100*np.mean(us >= 0.95*P['n_max']))

        mp, ml = m(xs_pid, us_pid), m(xs_lqr, us_lqr)
        mn = m(xs_nmpc, us_nmpc) if nmpc_ok else (float('nan'),)*3

        print(f"\n  {'':>14s}  {'PID':>8s}  {'LQR':>8s}  {'NMPC':>8s}")
        print(f"  {'─'*42}")
        print(f"  {'RMSE vx':>14s}  {mp[0]:8.3f}  {ml[0]:8.3f}  {mn[0]:8.3f}")
        print(f"  {'RMSE z':>14s}  {mp[1]:8.3f}  {ml[1]:8.3f}  {mn[1]:8.3f}")
        print(f"  {'SAT %':>14s}  {mp[2]:8.1f}  {ml[2]:8.1f}  {mn[2]:8.1f}")
        print(f"  {'시뮬 [s]':>14s}  {t_pid:8.2f}  {t_lqr:8.2f}  {t_nmpc:8.2f}")
        if stats:
            print(f"  {'NMPC 풀이':>14s}  {stats['mean_ms']:.2f}ms avg, "
                  f"{stats['n_ok']}/{stats['n_solves']} 수렴")

        all_results.append({
            'V': V, 'pid': mp, 'lqr': ml, 'nmpc': mn,
            'nmpc_ok': nmpc_ok, 'stats': stats})

    # 종합
    print(f"\n{'='*70}")
    print("  종합")
    print(f"{'='*70}")
    print(f"  {'V':>5s}  {'PID vx':>7s} {'LQR vx':>7s} {'NMPC vx':>7s}"
          f"  {'PID z':>7s} {'LQR z':>7s} {'NMPC z':>7s}")
    for r in all_results:
        p, l, n = r['pid'], r['lqr'], r['nmpc']
        def f(v): return f"{v:7.3f}" if not np.isnan(v) else "  FAIL"
        print(f"  {r['V']:5.0f}  {f(p[0])} {f(l[0])} {f(n[0])}"
              f"  {f(p[1])} {f(l[1])} {f(n[1])}")
    print(f"{'='*70}")

    # 정리
    for p in ['c_generated_code', 'acados_ocp.json']:
        fp = os.path.join(os.path.dirname(os.path.abspath(__file__)), p)
        if os.path.isdir(fp): shutil.rmtree(fp)
        elif os.path.isfile(fp): os.remove(fp)


if __name__ == '__main__':
    run_comparison()
