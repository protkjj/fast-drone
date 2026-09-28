"""arena_v2(모멘트 보정) 실제 시행 점검 — M17·F13·GSLQR, 튜닝 시나리오 2개, 기본 게인. 성능이 아니라 NaN·설정 오류 확인용."""
import numpy as np
from control.arena import load_config, build_scenarios
from control.arena_factory import ArenaFactory
from control.validation_suite import baseline_params, run_trial
from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
cfg = load_config('configs/arena_v2.json'); n = baseline_params(); f = ArenaFactory(cfg, n)
ids = ['tune3_step_altitude_p0.7_V0', 'tune2_gust_vertical_p3.5_V14']
sc = [s for s in build_scenarios(cfg, f.cp, n, scenarios=cfg['tuning']['scenarios']) if s.id in ids]
lim = Acceptance(**cfg['acceptance']); paper = PaperCriteria(**cfg['paper_criteria'])
for label in ('GSLQR', 'F13', 'M17'):
    for s in sc:
        try:
            m, r, log = run_trial(f, label, s.profile, s.cases[0], lim)
            pe = paper_evaluate(r, s.profile, paper, window=s.window, solve_log=log, n_max=n['n_max'])
            print(f"{label} {s.id}: stop={m['stop_reason']} paper_failed={pe['paper_failed']} rmse_v={pe['window_rmse_velocity']:.4f} rmse_z={pe['window_rmse_z']:.4f} max_omega={m['max_omega']:.2f} opt_fail={m['optimizer_failures']}/{m['optimizer_calls']} finite={np.all(np.isfinite(r['xs']))}", flush=True)
        except Exception as e:
            print(f"{label} {s.id}: EXCEPTION {type(e).__name__}: {e}", flush=True)
print('DONE')
