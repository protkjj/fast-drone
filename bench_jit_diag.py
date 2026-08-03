"""jit 벤치마크 공정성 검증 + p95 피크 원인 진단 + N=10+jit 조합 실측.

배경 (bench_accel_experiments.py 후속):
  jit 행(7.7ms)의 iter 수가 미수집이라 "같은 반복 수로 푼 공정 비교인지" 미확인,
  p95(18.3ms)가 중앙값의 2.4배로 벌어진 원인 미규명, N=10+jit 조합 미실측.

실험:
  N=20/N=10 × plain/jit 4개 솔버를 **동일한 상태 시퀀스·동일한 웜스타트 프로토콜**로
  풀고 per-솔브 (시간, iter)를 수집. plain vs jit의 iter를 인덱스별로 대조.

실행:
  # ⚠ jit이 임시 .c 파일을 cwd에 만들 수 있음 — iCloud 동기화 중인 repo에서 직접 돌리지 말고
  #   비동기화 폴더를 cwd로 두고 PYTHONPATH로 실행할 것:
  cd /tmp && PYTHONPATH=~/Desktop/dynamic/fast_drone \
    python3 ~/Desktop/dynamic/fast_drone/bench_jit_diag.py > ~/Desktop/dynamic/fast_drone/results/bench_jit_diag.txt 2>&1
"""
import time

import numpy as np

import casadi as ca
from vehicle_params import vehicle_params as P
from trim import find_trim
from hybrid_comparison import build_virtual_dynamics, NX_V, NU_V

f, xs, us = build_virtual_dynamics(P)

trim70 = find_trim(P, 70.0)
x70 = trim70['state'].copy()
x70[2] = 50.0
u70 = trim70['control'].copy()
T_ref = float(np.sum(P['k_T'] * u70**2))
x13 = np.concatenate([x70[0:10], x70[10:13]])
u_ref_v = np.array([T_ref, 0, 0, 0])

IPOPT_OPTS = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
              'ipopt.max_iter': 30, 'ipopt.warm_start_init_point': 'yes',
              'ipopt.tol': 1e-4}
JIT_OPTS = {'jit': True, 'compiler': 'shell', 'jit_options': {'flags': ['-O2']}}


def build_nlp(N, dt_nmpc):
    """VirtualNMPC._build_nlp와 동일 구조 (N, dt만 가변)."""
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

    return {
        'nlp': {'f': J_cost, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': p},
        'lbw': np.array(lbw), 'ubw': np.array(ubw),
        'lbg': np.array(lbg), 'ubg': np.array(ubg),
        'w0': np.array(w0, dtype=float),
    }


def make_states(n=20):
    r2 = np.random.default_rng(7)          # bench_accel_experiments와 동일 시드
    outs = []
    for _ in range(n):
        x = x13.copy()
        x[2] += r2.normal(0, 0.3)
        x[3] += r2.normal(0, 0.5)
        x[5] += r2.normal(0, 0.3)
        outs.append(x)
    return outs


STATES = make_states()
stride = NU_V + NX_V


def run_seq(solver, pack, w_start):
    """동일 웜스타트 프로토콜(해 시프트)로 STATES를 순회, (시간, iter) 수집."""
    times, iters = [], []
    wk = np.concatenate([w_start[stride:], w_start[-stride:]])
    for x in STATES:
        pv = np.concatenate([x, [70, 0, 0], [50.0], u_ref_v])
        t0 = time.perf_counter()
        sol = solver(x0=wk, lbx=pack['lbw'], ubx=pack['ubw'],
                     lbg=pack['lbg'], ubg=pack['ubg'], p=pv)
        times.append(time.perf_counter() - t0)
        iters.append(solver.stats().get('iter_count', -1))
        wo = np.array(sol['x']).flatten()
        wk = np.concatenate([wo[stride:], wo[-stride:]])
    return np.array(times), np.array(iters)


def report(label, times, iters):
    print(f"  {label}")
    print(f"    중앙값 {np.median(times)*1e3:6.1f} ms | p95 {np.percentile(times,95)*1e3:6.1f} ms"
          f" | 최악 {times.max()*1e3:6.1f} ms | iter {iters.min()}~{iters.max()}"
          f" (중앙 {np.median(iters):.0f})")
    order = np.argsort(times)[::-1][:3]
    worst = ", ".join(f"#{i}: {times[i]*1e3:.1f}ms/iter{iters[i]}" for i in order)
    print(f"    최악 3건: {worst}", flush=True)


for N, dtn in [(20, 0.05), (10, 0.1)]:
    print(f"\n════ N={N}, dt_nmpc={dtn} (지평선 {N*dtn:.1f}s) ════", flush=True)
    pack = build_nlp(N, dtn)

    sp = ca.nlpsol('vp', 'ipopt', pack['nlp'], IPOPT_OPTS)
    # 수렴해 확보 (양쪽 솔버의 공통 웜스타트 출발점)
    p0 = np.concatenate([x13, [70, 0, 0], [50.0], u_ref_v])
    sol0 = sp(x0=pack['w0'], lbx=pack['lbw'], ubx=pack['ubw'],
              lbg=pack['lbg'], ubg=pack['ubg'], p=p0)
    w_ref = np.array(sol0['x']).flatten()

    t_p, i_p = run_seq(sp, pack, w_ref)
    report(f"plain ipopt", t_p, i_p)

    t0 = time.perf_counter()
    opts_j = dict(IPOPT_OPTS)
    opts_j.update(JIT_OPTS)
    sj = ca.nlpsol('vj', 'ipopt', pack['nlp'], opts_j)
    print(f"  jit 컴파일: {time.perf_counter()-t0:.1f} s", flush=True)

    t_j, i_j = run_seq(sj, pack, w_ref)
    report(f"jit ipopt", t_j, i_j)

    same = int(np.sum(i_p == i_j))
    print(f"  ── 공정성: 20건 중 iter 일치 {same}건"
          f" (불일치 인덱스: {list(np.where(i_p != i_j)[0])})")
    print(f"  ── per-iter 시간: plain {np.median(t_p/np.maximum(i_p,1))*1e3:.2f} ms/iter"
          f" | jit {np.median(t_j/np.maximum(i_j,1))*1e3:.2f} ms/iter")
    # 피크-iter 상관: 시간 상위 5건의 iter가 상위인지
    top_t = np.argsort(t_j)[::-1][:5]
    print(f"  ── jit 시간 상위 5건의 iter: {i_j[top_t]} (전체 중앙 {np.median(i_j):.0f})",
          flush=True)
