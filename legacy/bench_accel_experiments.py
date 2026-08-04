"""NMPC 가속 실험 3종 — codegen(JIT)과 "가난한 RTI"의 실측 검증.

목적:
  acados로 가기 전에 CasADi 안에서 가능한 가속 수단을 실측으로 확인.
  (결과 요약은 results/bench_accel_experiments.txt, HANDOFF.md 참고)

실행:
  python3 bench_accel_experiments.py > results/bench_accel_experiments.txt 2>&1

실험 구성 (위험한/오래 걸리는 것을 뒤로 배치):
  [A] RK4 스텝 함수 평가: CasADi VM vs JIT(C codegen)
      → 커널 속도향상의 상한 실측. map(1000)으로 파이썬 호출 오버헤드 분리.
  [B] sqpmethod(max_iter=1) + qpOASES/OSQP — RTI 흉내
      → 2026-08-04 실측 결과 **실패**: exact Hessian 부정정칙으로
        qpOASES 3.2s/해 49% 이탈, OSQP는 해 발산.
        RTI에는 Gauss-Newton Hessian + OCP 구조화 QP(HPIPM)가 필요하다는
        것을 보여주는 반례로 보존 (= acados로 가야 하는 근거).
  [C] IPOPT + jit=True (NLP 전체 C 컴파일)
      → 2026-08-04 실측 2.68x (20.7→7.7ms), 컴파일 77s(1회성).
        함수평가 비중 71% × 커널 10x → Amdahl 예측 2.8x와 일치.

주의:
  아래 NLP 구성은 hybrid_comparison.VirtualNMPC._build_nlp의 의도적 복사본.
  (실험 대상과 동일한 문제를 솔버 옵션만 바꿔 비교해야 하므로,
   프로덕션 클래스를 건드리지 않고 여기서 복제함. 본체 수정 시 여기도 갱신할 것.)
"""
import time

import numpy as np

import casadi as ca
from vehicle_params import vehicle_params as P
from trim import find_trim
from hybrid_comparison import build_virtual_dynamics, NX_V, NU_V

rng = np.random.default_rng(0)

# ══════════════════════════════════════════════════
# [A] 함수 평가: VM vs JIT
# ══════════════════════════════════════════════════
print("[A] RK4 스텝 함수 평가: CasADi VM vs JIT(clang -O2)", flush=True)

f, xs, us = build_virtual_dynamics(P)
dt = 0.05
k1 = f(xs, us)
k2 = f(xs + dt/2*k1, us)
k3 = f(xs + dt/2*k2, us)
k4 = f(xs + dt*k3, us)
xn = xs + dt/6*(k1 + 2*k2 + 2*k3 + k4)
F = ca.Function('F', [xs, us], [xn])

t0 = time.perf_counter()
Fj = ca.Function('Fj', [xs, us], [xn],
                 {'jit': True, 'compiler': 'shell',
                  'jit_options': {'flags': ['-O2']}})
print(f"  JIT 컴파일(RK4 1스텝): {time.perf_counter()-t0:.2f} s", flush=True)

x0 = np.zeros(NX_V)
x0[6] = 1.0                      # 호버 쿼터니언 qx=1
x0[3] = 70.0
u0 = np.array([P['mass']*P['g'], 0.0, 0.0, 0.0])


def b(fn, n=20000, w=100):
    for _ in range(w):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n


tF = b(lambda: F(x0, u0))
tFj = b(lambda: Fj(x0, u0))
print(f"  단건 호출: VM {tF*1e6:7.2f} us | JIT {tFj*1e6:7.2f} us"
      f" | {tF/tFj:4.1f}x", flush=True)

try:
    Fm = F.map(1000)
    Fjm = Fj.map(1000)
    X = ca.DM(np.tile(x0, (1000, 1)).T)
    U = ca.DM(np.tile(u0, (1000, 1)).T)
    tFm = b(lambda: Fm(X, U), n=200, w=5) / 1000
    tFjm = b(lambda: Fjm(X, U), n=200, w=5) / 1000
    print(f"  map(1000): VM {tFm*1e6:7.2f} us | JIT {tFjm*1e6:7.2f} us"
          f" | 커널 {tFm/tFjm:4.1f}x  (파이썬 호출 오버헤드 제거판)", flush=True)
except Exception as e:
    print(f"  map 비교 실패: {e}", flush=True)

# ══════════════════════════════════════════════════
# NLP 구성 (VirtualNMPC._build_nlp 복사본 — 헤더 주의사항 참고)
# ══════════════════════════════════════════════════
N = 20
dt_nmpc = 0.05
trim70 = find_trim(P, 70.0)
x70 = trim70['state'].copy()
x70[2] = 50.0
u70 = trim70['control'].copy()
T_ref = float(np.sum(P['k_T'] * u70**2))
x13 = np.concatenate([x70[0:10], x70[10:13]])
u_ref_v = np.array([T_ref, 0, 0, 0])

k1 = f(xs, us)
k2 = f(xs + dt_nmpc/2*k1, us)
k3 = f(xs + dt_nmpc/2*k2, us)
k4 = f(xs + dt_nmpc*k3, us)
Fn = ca.Function('Fn', [xs, us], [xs + dt_nmpc/6*(k1 + 2*k2 + 2*k3 + k4)])

Q_v = np.diag([5.0, 5.0, 10.0])
Q_z = 20.0
Q_w = np.diag([1.0, 1.0, 1.0])
R = np.diag([1e-5, 1e-3, 1e-3, 1e-3])
R_du = np.diag([1e-4, 0.01, 0.01, 0.01])
T_max = 4 * P['k_T'] * P['n_max']**2
nu_max = 100.0

p = ca.SX.sym('p', NX_V + 3 + 1 + NU_V)
x_init, v_ref = p[0:NX_V], p[NX_V:NX_V+3]
z_ref, u_ref = p[NX_V+3], p[NX_V+4:NX_V+4+NU_V]

w, w0, lbw, ubw = [], [], [], []
g, lbg, ubg = [], [], []
J_cost = 0.0
X_prev, U_prev = x_init, u_ref
for k in range(N):
    U_k = ca.SX.sym(f'U_{k}', NU_V)
    w.append(U_k)
    lbw += [0.0, -nu_max, -nu_max, -nu_max]
    ubw += [T_max, nu_max, nu_max, nu_max]
    w0 += [T_ref, 0, 0, 0]
    X_k = ca.SX.sym(f'X_{k}', NX_V)
    w.append(X_k)
    lbw += [-1e6]*NX_V
    ubw += [1e6]*NX_V
    w0 += [0.0]*NX_V
    g.append(X_k - Fn(X_prev, U_k))
    lbg += [0.0]*NX_V
    ubg += [0.0]*NX_V
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

nlp = {'f': J_cost, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': p}
lbw, ubw = np.array(lbw), np.array(ubw)
lbg, ubg = np.array(lbg), np.array(ubg)
w0 = np.array(w0, dtype=float)
p_val = np.concatenate([x13, [70, 0, 0], [50.0], u_ref_v])

IPOPT_OPTS = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
              'ipopt.max_iter': 30, 'ipopt.warm_start_init_point': 'yes',
              'ipopt.tol': 1e-4}

# 기준 ipopt 솔버 + 수렴해 (warm start 소스)
solver_ip = ca.nlpsol('vnmpc', 'ipopt', nlp, IPOPT_OPTS)
sol_ref = solver_ip(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=p_val)
w_ref = np.array(sol_ref['x']).flatten()
stride = NU_V + NX_V


def perturbed_states(n):
    outs = []
    r2 = np.random.default_rng(7)
    for _ in range(n):
        x = x13.copy()
        x[2] += r2.normal(0, 0.3)
        x[3] += r2.normal(0, 0.5)
        x[5] += r2.normal(0, 0.3)
        outs.append(x)
    return outs


STATES = perturbed_states(20)


def bench_solver(solver, label, warm_from=w_ref, with_lam=False, sol0=None):
    """receding-horizon 흉내: 교란 상태 시퀀스를 해 시프트 warm start로 풀기."""
    times, u0s = [], []
    wk = np.concatenate([warm_from[stride:], warm_from[-stride:]])
    lam_x = sol0['lam_x'] if (with_lam and sol0 is not None) else None
    lam_g = sol0['lam_g'] if (with_lam and sol0 is not None) else None
    for x in STATES:
        pv = np.concatenate([x, [70, 0, 0], [50.0], u_ref_v])
        kw = dict(x0=wk, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=pv)
        if lam_x is not None:
            kw['lam_x0'] = lam_x
            kw['lam_g0'] = lam_g
        t0 = time.perf_counter()
        sol = solver(**kw)
        times.append(time.perf_counter() - t0)
        wo = np.array(sol['x']).flatten()
        u0s.append(wo[0:NU_V])
        wk = np.concatenate([wo[stride:], wo[-stride:]])
        if lam_x is not None:
            lam_x, lam_g = sol['lam_x'], sol['lam_g']
    times = np.array(times)
    print(f"  {label}: warm 중앙값 {np.median(times)*1e3:6.1f} ms"
          f" | p95 {np.percentile(times, 95)*1e3:6.1f} ms", flush=True)
    return np.array(u0s), float(np.median(times))


print("\n[기준] IPOPT (동일 NLP, 실험 스크립트 재구성판)", flush=True)
u_ip, t_ip = bench_solver(solver_ip, "ipopt max_iter=30")

# ══════════════════════════════════════════════════
# [B] sqpmethod 1-iter = RTI 흉내
# ══════════════════════════════════════════════════
print("\n[B] sqpmethod(max_iter=1) + qpOASES — RTI 흉내", flush=True)
sqp_variants = [
    ("convexify+qpoases", {'qpsol': 'qpoases', 'max_iter': 1,
                           'convexify_strategy': 'regularize',
                           'print_time': 0, 'print_header': False,
                           'print_iteration': False,
                           'qpsol_options': {'printLevel': 'none',
                                             'error_on_fail': False}}),
    ("plain+qpoases",     {'qpsol': 'qpoases', 'max_iter': 1,
                           'print_time': 0, 'print_header': False,
                           'print_iteration': False,
                           'qpsol_options': {'printLevel': 'none',
                                             'error_on_fail': False}}),
    ("convexify+osqp",    {'qpsol': 'osqp', 'max_iter': 1,
                           'convexify_strategy': 'regularize',
                           'print_time': 0, 'print_header': False,
                           'print_iteration': False,
                           'qpsol_options': {'error_on_fail': False,
                                             'osqp': {'verbose': False}}}),
]
for name, opts in sqp_variants:
    try:
        s = ca.nlpsol('rti', 'sqpmethod', nlp, opts)
        u_sqp, t_sqp = bench_solver(s, f"sqpmethod {name}",
                                    with_lam=True, sol0=sol_ref)
        du = np.linalg.norm(u_sqp - u_ip, axis=1)
        scale = np.linalg.norm(u_ip, axis=1).mean()
        print(f"    vs ipopt 해: |Δu| 중앙값 {np.median(du):.3f}"
              f" (u 크기 ~{scale:.1f} 대비 {100*np.median(du)/scale:.1f}%)",
              flush=True)
    except Exception as e:
        print(f"  sqpmethod {name}: 실패 — {type(e).__name__}: {e}", flush=True)

# ══════════════════════════════════════════════════
# [C] IPOPT + jit (NLP 전체 codegen) — 컴파일이 1분+ 걸릴 수 있음
# ══════════════════════════════════════════════════
print("\n[C] IPOPT + jit=True (NLP 전체 C 컴파일) — 컴파일 시작...", flush=True)
try:
    t0 = time.perf_counter()
    opts_j = dict(IPOPT_OPTS)
    opts_j.update({'jit': True, 'compiler': 'shell',
                   'jit_options': {'flags': ['-O2']}})
    solver_j = ca.nlpsol('vnmpc_j', 'ipopt', nlp, opts_j)
    print(f"  컴파일 시간: {time.perf_counter()-t0:.1f} s", flush=True)
    u_j, t_j = bench_solver(solver_j, "ipopt+jit max_iter=30")
    print(f"  속도향상: {t_ip/t_j:.2f}x (vs 기준 {t_ip*1e3:.1f} ms)", flush=True)
except Exception as e:
    print(f"  실패 — {type(e).__name__}: {e}", flush=True)
