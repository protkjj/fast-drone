"""계산 성능 벤치마크 — 플랜트/제어기/NMPC 솔브 시간 실측.

목적:
  "NMPC가 실시간이 되는가"를 추정이 아니라 숫자로 답하기 위한 스크립트.
  플랫폼(macOS/Ubuntu)별로 재실행해서 수치를 비교하는 용도.

실행 (결과는 results/에 저장하는 컨벤션):
  python3 bench_compute.py > results/bench_compute.txt 2>&1

주의:
  - 순차 실행 전제. 다른 무거운 프로세스와 병렬로 돌리면 타이밍이 오염됨.
  - NMPC warm 솔브는 트림 근방 교란 상태로 receding-horizon을 흉내낸 것.
    발산 직전 같은 어려운 상태에서는 iter가 늘어 더 느릴 수 있음 (p95 참고).

측정 항목:
  [startup]  1회성 빌드 비용 (플랜트/트림/LQR/ScheduledLQR)
  [per-call] 1kHz 루프에서 매 스텝 드는 비용 (예산 1000us)
  [NMPC]     cold/warm 솔브 시간 + IPOPT 내부 함수평가 비중
             (함수평가 비중 = codegen(jit)으로 줄일 수 있는 부분의 상한 근거)
  [스루풋]   미션 시뮬 1초당 벽시계 → MC 소요시간 추정
"""
import time
import platform
import sys

import numpy as np

import casadi as ca
from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import CascadedPID, LQRController, ScheduledLQR, INDIController
from nmpc import NMPCController
from hybrid_comparison import VirtualNMPC, ProperHybrid


def bench_call(fn, n, warmup=5):
    for _ in range(warmup):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n


def pr(label, sec):
    print(f"  {label:<40s} {sec*1e6:10.1f} us", flush=True)


print(f"platform: {platform.platform()} / {platform.machine()}")
print(f"python {sys.version.split()[0]}, casadi {ca.__version__}, numpy {np.__version__}",
      flush=True)

rng = np.random.default_rng(0)

# ══════════════════════════════════════════════════
# 시작 비용 (1회성)
# ══════════════════════════════════════════════════
print("\n[startup 비용 (1회성)]", flush=True)

t0 = time.perf_counter()
plant = AxialDronePlant(P, dt=0.001)
print(f"  AxialDronePlant build:           {time.perf_counter()-t0:6.2f} s", flush=True)

t0 = time.perf_counter()
trim70 = find_trim(P, 70.0)
print(f"  find_trim(70) (플랜트 재빌드 포함): {time.perf_counter()-t0:6.2f} s", flush=True)

x70 = trim70['state'].copy()
x70[2] = 50.0
u70 = trim70['control'].copy()
T_trim70 = float(np.sum(P['k_T'] * u70**2))
x13_70 = np.concatenate([x70[0:10], x70[10:13]])

t0 = time.perf_counter()
lqr = LQRController(P, trim70['state'], u70)
t_lqr_build = time.perf_counter() - t0
print(f"  LQRController build (선형화+ARE):  {t_lqr_build:6.2f} s", flush=True)

t0 = time.perf_counter()
slqr = ScheduledLQR(P, v_ref=[70, 0, 0], z_ref=50.0)
print(f"  ScheduledLQR build (9x 트림+ARE):  {time.perf_counter()-t0:6.2f} s", flush=True)

# ══════════════════════════════════════════════════
# 제어기/플랜트 per-call 비용 (1kHz 루프 = 1000us 예산)
# ══════════════════════════════════════════════════
print("\n[per-call 비용]  (시뮬 1kHz 루프 예산 = 1000 us/step)", flush=True)

t_step = bench_call(lambda: plant.step(x70, u70), 2000)
pr("plant.step (RK4 1ms)", t_step)

pid = CascadedPID(P, v_ref=[70, 0, 0], z_ref=50.0)
t_pid = bench_call(lambda: pid(0.0, x70), 2000)
pr("CascadedPID.__call__", t_pid)

t_lqr = bench_call(lambda: lqr(0.0, x70), 2000)
pr("LQRController.__call__", t_lqr)

t_slqr = bench_call(lambda: slqr(0.0, x70), 2000)
pr("ScheduledLQR.__call__ (np.interp 77회)", t_slqr)

indi = INDIController(P, v_ref=[70, 0, 0], z_ref=50.0)
indi(0.0, x70)
indi(0.001, x70)
t_indi = bench_call(lambda: indi(0.0, x70), 2000)
pr("INDIController.__call__", t_indi)

# ProperHybrid의 INDI 경로만 측정 (dt_ctrl을 크게 줘서 NMPC는 1회 솔브 후 캐시)
vn_cached = VirtualNMPC(P, v_ref=[70, 0, 0], z_ref=50.0, T_ref=T_trim70,
                        dt_ctrl=1e9)
hyb = ProperHybrid(vn_cached, P, dt=0.001)
tick = {'t': 0.0}
hyb(tick['t'], x70)


def call_hyb():
    # ProperHybrid는 실측 Δt를 쓰므로 t를 전진시켜야 함
    tick['t'] += 0.001
    hyb(tick['t'], x70)


call_hyb()
call_hyb()
t_hyb = bench_call(call_hyb, 2000)
pr("ProperHybrid INDI-경로 (NMPC 캐시시)", t_hyb)

# ══════════════════════════════════════════════════
# NMPC 솔브 시간
# ══════════════════════════════════════════════════
print("\n[NMPC 솔브 시간]  (오프라인 예산 20ms, SITL 예산 100ms)", flush=True)

EVAL_KEYS = ('t_wall_nlp_f', 't_wall_nlp_g', 't_wall_nlp_grad_f',
             't_wall_nlp_jac_g', 't_wall_nlp_hess_l')


def bench_mpc(mpc, x_base, label, n_warm=30):
    t0 = time.perf_counter()
    mpc._solve(x_base.copy())
    cold = time.perf_counter() - t0

    times, iters, fracs = [], [], []
    for _ in range(n_warm):
        x = x_base.copy()
        x[2] += rng.normal(0, 0.3)
        x[3] += rng.normal(0, 0.5)
        x[5] += rng.normal(0, 0.3)
        t0 = time.perf_counter()
        mpc._solve(x)
        dtb = time.perf_counter() - t0
        s = mpc.solver.stats()
        tw = s.get('t_wall_total', dtb)
        fe = sum(s.get(k, 0.0) for k in EVAL_KEYS)
        times.append(dtb)
        iters.append(s.get('iter_count', -1))
        fracs.append(fe / tw if tw > 0 else 0.0)

    times = np.array(times)
    print(f"  {label}")
    print(f"    cold {cold*1e3:7.1f} ms | warm 중앙값 {np.median(times)*1e3:6.1f} ms"
          f" | p95 {np.percentile(times, 95)*1e3:6.1f} ms"
          f" | iter 중앙값 {np.median(iters):.0f}"
          f" | 함수평가 비중 {100*np.median(fracs):.0f}%", flush=True)
    return float(np.median(times))


nmpc17 = NMPCController(P, v_ref=[70, 0, 0], z_ref=50.0, u_ref=u70,
                        N=20, dt_nmpc=0.05, dt_ctrl=0.02)
t_nmpc17 = bench_mpc(nmpc17, x70, "NMPC 17D N=20 max_iter=30 (오프라인 설정)")

vn = VirtualNMPC(P, v_ref=[70, 0, 0], z_ref=50.0, T_ref=T_trim70,
                 N=20, dt_nmpc=0.05, dt_ctrl=0.02)
t_vn = bench_mpc(vn, x13_70, "VirtualNMPC 13D N=20 max_iter=30 (오프라인 설정)")

vn5 = VirtualNMPC(P, v_ref=[70, 0, 0], z_ref=50.0, T_ref=T_trim70,
                  N=20, dt_nmpc=0.05, dt_ctrl=0.1, max_iter=5)
t_vn5 = bench_mpc(vn5, x13_70, "VirtualNMPC 13D N=20 max_iter=5 (SITL 설정)")

vn10 = VirtualNMPC(P, v_ref=[70, 0, 0], z_ref=50.0, T_ref=T_trim70,
                   N=10, dt_nmpc=0.1, dt_ctrl=0.02)
t_vn10 = bench_mpc(vn10, x13_70, "VirtualNMPC 13D N=10 dt=0.1 (지평선 1s 유지)")

# ══════════════════════════════════════════════════
# 미션 시뮬 스루풋 추정
# ══════════════════════════════════════════════════
print("\n[미션 시뮬 스루풋]  1 시뮬-초당 벽시계 (1kHz: 플랜트+제어기, NMPC 50Hz)",
      flush=True)

rows = [
    ("PID",              t_pid,  0.0,      0),
    ("ScheduledLQR",     t_slqr, 0.0,      0),
    ("INDI",             t_indi, 0.0,      0),
    ("Hybrid (50Hz솔브)", t_hyb,  t_vn,     50),
    ("NMPC17D (50Hz)",   2e-6,   t_nmpc17, 50),
]
for name, tc, tsv, ns in rows:
    tot = 1000 * (t_step + tc) + ns * tsv
    print(f"  {name:<18s} {tot:7.2f} s/시뮬-초  (실시간의 {tot:5.1f}배 소요)")

t_mission_hyb = 65 * (1000 * (t_step + t_hyb) + 50 * t_vn)
print(f"\n  65초 미션 1회(Hybrid) 추정: {t_mission_hyb/60:.1f} 분"
      f"  /  MC 10회: {10*t_mission_hyb/60:.0f} 분", flush=True)
