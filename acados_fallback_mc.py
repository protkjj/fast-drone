"""폴백 결합 MC — EXT5 vs GN-RTI × (ω15 + 복귀금지): 최종 구성 판정.

배경: ω15+복귀금지 하에서 두 솔버의 완주 성능이 비슷 (2.33 vs 2.60)
→ EXT5(10.7ms)의 값어치는 "폴백 트리거를 덜 밟는가"로 판정 (kj 지정).

지표:
  1) 감속 max_dz 5m+ 발생률 (기존 기준)
  2) 폴백 전환 빈도 + 전환 시각·전환 시 |ω|
  3) 완주율 (NaN 없이 65s)
  4) |ω| 통계 + 실기 실패 판정: |ω|>35(자이로 포화) 또는 |ω|>25 지속 200ms+
     — RMSE만으로는 텀블을 못 잡음 (EXT5 단독이 |ω|79 텀블에도 z 1.46이었던 사례)

실행 (구성별 병렬, caffeinate 금지):
  DYLD_LIBRARY_PATH=$HOME/acados/lib python3 acados_fallback_mc.py ext5 > results/fallback_mc_ext5.txt 2>&1
  DYLD_LIBRARY_PATH=$HOME/acados/lib python3 acados_fallback_mc.py gnrti > results/fallback_mc_gnrti.txt 2>&1
"""
import sys
import time as timer

import numpy as np

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from hybrid_comparison import ProperHybrid
from vnmpc_acados import AcadosVirtualNMPC
from fallback_controller import HybridWithFallback
from controller import ScheduledLQR
from mission_sim import MissionProfile, MissionController, run_mission
from gust_comparison import make_gust_fn

CONFIGS = {
    'ext5': dict(rate_aug=True, cost_variant='EXT_EXACT',
                 nlp_solver_type='SQP', nlp_max_iter=5),
    'gnrti': dict(rate_aug=True),
    # 예방형: 감속 시작(t=43)에 계획 전환 — 텀블 진입 자체를 회피해
    # 반응형 ω15의 핸드오버 딥(~6.7m)을 줄이는지 검증. ω15는 백스톱 유지
    'gnrti_pre': dict(rate_aug=True),
}
PREEMPTIVE_T = {'gnrti_pre': 43.0}


class LoggedFB(HybridWithFallback):
    """전환 시각·전환 시 |ω| 기록. preempt_t 설정 시 계획 전환(예방형)."""

    def __init__(self, *a, preempt_t=None, **k):
        super().__init__(*a, **k)
        self.switch_events = []
        self.preempt_t = preempt_t

    def __call__(self, t, x):
        if (self.preempt_t is not None and self._using_hybrid
                and t >= self.preempt_t):
            # 계획 전환: 텀블 진입 전에 LQR 인수 (복귀 금지 전제)
            self._using_hybrid = False
            self._switch_count += 1
            self._cmd_history.clear()
            self.switch_events.append((float(t),
                                       float(np.linalg.norm(x[10:13]))))
            return self.lqr(t, x)
        was = self._using_hybrid
        u = super().__call__(t, x)
        if was and not self._using_hybrid:
            self.switch_events.append((float(t),
                                       float(np.linalg.norm(x[10:13]))))
        return u


def omega_metrics(xs, dt=0.001):
    om = np.linalg.norm(xs[:, 10:13], axis=1)
    om = om[np.isfinite(om)]
    if len(om) == 0:
        return {'om_max': np.inf, 'ms25': np.inf, 'run25': np.inf,
                'real_fail': True}
    above = om > 25.0
    runs, cur = [], 0
    for a in above:
        if a:
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    run25 = (max(runs) if runs else 0) * dt * 1e3       # ms
    om_max = float(np.max(om))
    return {'om_max': om_max, 'ms25': int(np.sum(above)) * dt * 1e3,
            'run25': run25,
            'real_fail': om_max > 35.0 or run25 >= 200.0}


def main(config_name, n_trials=30, seed=0):
    kw = dict(N=20, dt_nmpc=0.05, dt_ctrl=0.02, **CONFIGS[config_name])
    plant = AxialDronePlant(P, dt=0.001)
    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    gust_fn = make_gust_fn('vertical', 10.0, 35.0, 1.0)
    rng = np.random.default_rng(seed)

    print(f"폴백 MC — {config_name} + ω15 + 복귀금지 ({n_trials}회, seed={seed})")
    print(f"구성: {kw}")
    print("=" * 60, flush=True)

    rows = []
    t0_all = timer.time()
    for i in range(n_trials):
        x0 = AxialDronePlant.hover_state(P)
        x0[2] = 2.0 + rng.normal(0, 0.5)
        x0[3] = rng.normal(0, 0.5)
        x0[5] = rng.normal(0, 0.3)

        vn = AcadosVirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                               code_suffix=f'fmc_{config_name}', **kw)
        hyb = ProperHybrid(vn, P, dt=plant.dt)
        lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=2.0)
        fb = LoggedFB(hyb, lqr, z_ref=2.0, z_err_limit=10.0,
                      omega_limit=15.0, cooldown_sec=1e6, dt=plant.dt,
                      preempt_t=PREEMPTIVE_T.get(config_name))
        ctrl = MissionController(fb, profile)
        res = run_mission(plant, ctrl, x0, profile, wind_fn=gust_fn)

        ts, xs, z_refs = res['ts'], res['xs'], res['z_refs']
        completed = not bool(np.any(np.isnan(xs)))
        mask = (ts >= 43.0) & (ts < 58.0)
        dz = (np.inf if np.any(np.isnan(xs[mask]))
              else float(np.max(np.abs(xs[mask, 2] - z_refs[mask]))))
        omm = omega_metrics(xs)
        sw = fb.switch_events[0] if fb.switch_events else None
        rows.append({'dz': dz, 'completed': completed,
                     'switched': fb.fallback_count > 0, 'sw': sw, **omm})
        sw_str = f"t={sw[0]:.1f}s ω={sw[1]:.1f}" if sw else "없음"
        print(f"  [{i+1}/{n_trials}] dz={dz:6.2f} 완주={completed}"
              f" 전환={sw_str} |ω|max={omm['om_max']:5.1f}"
              f" run25={omm['run25']:.0f}ms 실기판정="
              f"{'FAIL' if omm['real_fail'] else 'ok'}", flush=True)

    print(f"\n완료 [{timer.time()-t0_all:.0f}s]")
    n = len(rows)
    dzs = np.array([r['dz'] for r in rows])
    print(f"\n════ {config_name} 요약 ════")
    print(f"  완주율        : {sum(r['completed'] for r in rows)}/{n}")
    print(f"  전환 빈도     : {sum(r['switched'] for r in rows)}/{n}")
    sws = [r['sw'] for r in rows if r['sw']]
    if sws:
        print(f"  전환 시 |ω|   : 평균 {np.mean([s[1] for s in sws]):.1f}"
              f" (범위 {min(s[1] for s in sws):.1f}~{max(s[1] for s in sws):.1f})")
        print(f"  전환 시각     : 평균 t={np.mean([s[0] for s in sws]):.1f}s")
    print(f"  감속 5m+      : {int(np.sum(dzs >= 5))}/{n}"
          f" ({100*np.mean(dzs >= 5):.0f}%)")
    fin = dzs[np.isfinite(dzs)]
    if len(fin):
        print(f"  감속 max_dz   : 중앙 {np.median(fin):.2f} | P95"
              f" {np.percentile(fin, 95):.2f} | 최악 {np.max(fin):.2f} m")
    print(f"  실기판정 FAIL : {sum(r['real_fail'] for r in rows)}/{n}"
          f"  (|ω|>35 또는 >25 지속 200ms+)")
    print(f"  |ω|max        : 중앙 {np.median([r['om_max'] for r in rows]):.1f}"
          f" | 최악 {max(r['om_max'] for r in rows):.1f}")


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'ext5')
