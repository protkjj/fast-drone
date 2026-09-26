"""GSLQR·CPID 설계 영역 확인 — kj 작업지시서(2026-09-25) 작업 C. 결과는 PILOT.

"두 제어기 모두 '설계 영역에서 잘 된다'는 근거(정착 시간, 오버슈트)를 표로
남긴다." 비교가 아니라 **기준선이 제 영역에서 정상인지** 확인하는 진단이다 —
기준선이 망가진 채 비교하면 경기장이 기운다.

시험: 팀 플랜트(명목)의 정확한 트림에서 출발해 0.5초부터 계단 참조를 준다.
  고도 +1 m   (속도 참조는 그대로)
  속도 +1 m/s (고도 참조는 그대로)
계단은 논문 §5.5가 말하는 '진단 시험' 용도다(주 기동 비교에는 연속 참조).
운용점: GSLQR 0·20·40·60·85 m/s, CPID 0·10·20 m/s(CPID 설계 영역, 작업지시서).

계단 모양(kj 결정 2026-09-26 오후):
  smooth(기본) 모든 제어기에 같은 **매끄러운 계단** — 논문 식(32) smoothstep으로 1 s에 걸쳐 바뀐다.
               PX4가 설정값을 궤적 생성기로 매끄럽게 만든 뒤 가속도를 피드포워드하는 구조와
               맞추려는 것이다. 참조 가속도 피드포워드(GSLQR·CPID)가 들어간 뒤로는 원계단을 차분하면
               계단 직전 0.05 s 동안 20 m/s² 펄스가 되어, 기준선의 루프가 아니라 그 펄스를 재게 된다.
  step         옛 원계단(t=0.5 s에 순간 변화) — 2026-09-26 오전까지의 기록을 재현할 때만 쓴다.
               출력 파일을 따로 쓴다(DESIGN_CHECK_step.md).

판정 기준(정착 대역): 계단 크기의 ±5%. 정착시간은 계단 시작(0.5 s)부터 그 대역에 들어가
끝까지 머무는 첫 시각까지다 — 매끄러운 계단이면 1 s 전이가 그 안에 들어 있다. 오버슈트는 목표를
넘어선 최대량 / 계단 크기.

실행: python -m control.arena_design_check [--out results/arena]
      python -m control.arena_design_check --shape step --feedforward off
        → 옛 원계단·피드포워드 전 기록(DESIGN_CHECK_step.md)을 다시 만든다
      python -m control.arena_design_check --tuned results/arena/tuning/pilot24 --out <dir>
        → 튜닝 기록(<label>.record.json)의 최선값으로 같은 점검(kj: 튜닝 뒤에도 CPID가
          설계 영역에서 수렴하지 못하면 결정 필요로 올린다)
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from control.arena import load_config, DEFAULT_CONFIG, ROOT
from control.arena_factory import ArenaFactory
from control.validation_metrics import Acceptance

STEP_TIME = 0.5
DURATION = 15.0          # 적분기(GSLQR LQI, CPID 속도 적분)의 정상오차 제거까지 보려면 8초는 짧다
BAND = 0.05                    # 계단 크기의 ±5%
POINTS = {'GSLQR': (0.0, 20.0, 40.0, 60.0, 85.0), 'CPID': (0.0, 10.0, 20.0)}
STEPS = {'altitude_+1m': (0.0, 1.0), 'speed_+1mps': (1.0, 0.0)}   # (Δvx, Δz)
# 설계 영역의 윗끝에서는 속도 계단을 아래로(-1 m/s) 준다. 처음엔 +1로 줬는데 GSLQR
# 85 m/s에서 참조 86이 스케줄 표(0~85) 밖이라 85로 잘려 오차상태가 0이 됐다 —
# 제어기가 아니라 점검 설계의 결함이었다(2026-09-25).
REGION_TOP = {'GSLQR': 85.0, 'CPID': 20.0}
SMOOTH_S = 1.0                 # 매끄러운 계단의 전이 시간(논문 식(32) smoothstep)
SHAPES = {'smooth': SMOOTH_S, 'step': 0.0}


class StepProfile:
    """트림 속도·고도에서 t=STEP_TIME부터 계단 하나. 스위트 profile 규약을 따른다.

    ramp_s > 0이면 식(32) smoothstep으로 ramp_s초에 걸쳐 바뀌는 매끄러운 계단, 0이면 옛 원계단이다.
    """

    def __init__(self, V, z, dv, dz, duration=DURATION, ramp_s=SMOOTH_S):
        self.V, self.z, self.dv, self.dz = float(V), float(z), float(dv), float(dz)
        self.ramp_s = float(ramp_s)
        self.T_total = float(duration)
        self.phases = [('트림', 0.0, STEP_TIME, V, V, z, z),
                       ('계단', STEP_TIME, duration - STEP_TIME, V + dv, V + dv, z + dz, z + dz)]
        self.gust_interval = None
        self.cruise_start, self.cruise_end = 0.0, self.T_total
        self.decel_start = self.decel_end = None

    def _progress(self, ts):
        """계단 진행도 s ∈ [0, 1] — 원계단이면 0/1, 매끄러운 계단이면 smoothstep((t−0.5)/전이 시간)."""
        from control.mission_profiles import SmoothstepProfile
        ts = np.asarray(ts, dtype=float)
        return SmoothstepProfile.shape(np.clip((ts - STEP_TIME)/self.ramp_s, 0.0, 1.0))

    def get_ref(self, t):
        if not self.ramp_s:                         # 옛 원계단 — 기록을 비트 단위로 재현하려고 그대로 둔다
            after = t >= STEP_TIME
            return (np.array([self.V + self.dv*after, 0.0, 0.0]), self.z + self.dz*after,
                    '계단' if after else '트림')
        s = float(self._progress(t))
        return (np.array([self.V + self.dv*s, 0.0, 0.0]), self.z + self.dz*s,
                '계단' if t >= STEP_TIME else '트림')

    def compute_refs(self, ts):
        if not self.ramp_s:
            after = np.asarray(ts) >= STEP_TIME
            v = np.zeros((len(after), 3))
            v[:, 0] = self.V + self.dv*after
            return v, self.z + self.dz*after
        s = self._progress(ts)
        v = np.zeros((len(s), 3))
        v[:, 0] = self.V + self.dv*s
        return v, self.z + self.dz*s

    def get_phase_boundaries(self):
        return [(name, t, t + d) for name, t, d, *_ in self.phases]


def step_metrics(ts, y, y0, target):
    """정착시간·오버슈트·정상오차 — y는 계단이 걸린 채널(고도 또는 전진속도)."""
    step = target - y0
    band = BAND*abs(step)
    after = ts >= STEP_TIME
    t, e = ts[after], y[after] - target
    inside = np.abs(e) <= band
    settled_from = None
    if inside[-1]:
        last_out = np.flatnonzero(~inside)
        settled_from = float(t[last_out[-1] + 1] if len(last_out) else t[0])
    overshoot = max(0.0, float(np.max(np.sign(step)*e))) / abs(step)
    tail = t >= ts[-1] - 1.0
    return dict(settling_s=None if settled_from is None else settled_from - STEP_TIME,
                overshoot_pct=100.0*overshoot,
                steady_error=float(abs(np.mean(e[tail]))),
                band=band)


def run(config, factory, label, V, step_name, shape='smooth', duration=DURATION):
    import control.validation_suite as suite
    dv, dz = STEPS[step_name]
    if dv and V >= REGION_TOP[label] - 1e-9:
        dv = -dv
    z = float(config['altitude_m'])
    profile = StepProfile(V, z, dv, dz, duration=duration, ramp_s=SHAPES[shape])
    case = dict(case_id=f'design_{label}_{V:g}_{step_name}', factors={})
    row, result, _ = suite.run_trial(factory, label, profile, case, Acceptance())
    ts, xs = result['ts'], result['xs']
    if dz:
        channel, cross = xs[:, 2], np.abs(xs[:, 3] - V)
        metrics = step_metrics(ts, channel, z, z + dz)
        metrics['cross_coupling_max'] = float(np.max(cross))          # 속도 이탈 [m/s]
    else:
        channel, cross = xs[:, 3], np.abs(xs[:, 2] - z)
        metrics = step_metrics(ts, channel, V, V + dv)
        metrics['cross_coupling_max'] = float(np.max(cross))          # 고도 이탈 [m]
    step_label = f'altitude_{dz:+g}m' if dz else f'speed_{dv:+g}mps'
    metrics.update(controller=label, speed=V, step=step_label, shape=shape, stop_reason=row['stop_reason'],
                   simulated_seconds=row['simulated_seconds'], max_omega=row['max_omega'],
                   trajectory_sha256=row['trajectory_sha256'], integrators=row.get('integrators'))
    return metrics


def write_markdown(path, rows, meta):
    lines = [f'# 설계 영역 확인 (GSLQR·CPID) — {meta["label"]}', '',
             '**PILOT** — 기준선이 제 설계 영역에서 정상인지 보는 진단. 비교 결론 없음.', '',
             f'재현: `{meta["command"]}` · 설정 sha256 `{meta["config_sha256"][:12]}` · '
             f'git `{meta["git_revision"]}` dirty={meta["git_dirty"]}', '',
             *([f'게인: 튜닝 기록 최선값 — ' + ', '.join(
                 f'{k} 목적함수 {v["prior_objective"]:.4g} → {v["best_objective"]:.4g}(예산 {v["budget"]})'
                 for k, v in meta['tuned'].items()), ''] if meta.get('tuned') else
               ['게인: configs/gains 사전값', '']),
             (f'계단 모양: **매끄러운 계단**(논문 식(32) smoothstep, {meta["ramp_s"]:g} s 전이 — 모든 제어기에 '
              '같다, kj 결정 2026-09-26 오후). 정착시간은 계단 시작부터 재서 전이 시간이 들어 있다.'
              if meta.get('shape', 'step') == 'smooth' else
              '계단 모양: **원계단**(순간 변화) — 2026-09-26 오전까지의 기록. 지금 근거는 매끄러운 계단(DESIGN_CHECK.md)이다.'),
             f'참조 가속도 피드포워드(GSLQR·CPID): {meta.get("feedforward", "on")}', '',
             f'팀 플랜트 명목, 트림 출발, t={STEP_TIME}s부터 계단, {meta.get("duration", DURATION):g}s 관찰. '
             f'정착 대역 = 계단의 ±{100*BAND:.0f}%. 교차결합 = 다른 채널의 최대 이탈.', '',
             '적분 한계 % = 어느 적분 채널이든 한계에 닿아 있던 스텝의 비율, 적분 정지 % = 포화로 '
             '조건부 적분이 멈춘 스텝의 비율(GSLQR·CPID 같은 형식, arena_factory.IntegratorLog).', '',
             '| 제어기 | 속도 m/s | 계단 | 정착시간 s | 오버슈트 % | 정상오차 | 교차결합 | |ω|max | '
             '적분 한계 % | 적분 정지 % | 정지 |',
             '|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---|']
    for r in rows:
        settle = '미정착' if r['settling_s'] is None else f'{r["settling_s"]:.2f}'
        integ = r.get('integrators') or {}
        at_limit = f'{100*integ["at_limit_fraction"]:.2f}' if integ else '-'
        frozen = f'{100*integ["frozen_fraction"]:.2f}' if integ else '-'
        lines.append(f'| {r["controller"]} | {r["speed"]:g} | {r["step"]} | {settle} | '
                     f'{r["overshoot_pct"]:.1f} | {r["steady_error"]:.3g} | '
                     f'{r["cross_coupling_max"]:.3g} | {r["max_omega"]:.3g} | {at_limit} | {frozen} | '
                     f'{r["stop_reason"] or ""} |')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def tuned_overrides(config, run_dir, labels=('GSLQR', 'CPID')):
    """튜닝 기록의 최선값을 팩토리 overrides로 바꾼다(arena_tune.parameter_space와 같은 적용)."""
    from control.arena_tune import parameter_space
    gains = ArenaFactory(config).gains
    overrides, used = {}, {}
    for label in labels:
        path = Path(run_dir)/f'{label}.record.json'
        record = json.loads(path.read_text(encoding='utf-8'))
        if record.get('status') != 'complete':
            raise ValueError(f'{path}: tuning not complete ({record.get("status")})')
        _, _, apply = parameter_space(config, label, gains)
        overrides.update(apply(record['best_values']))
        used[label] = dict(record=str(path), best_values=record['best_values'],
                           best_objective=record.get('best_objective'),
                           prior_objective=record.get('prior_objective'), budget=record['budget'])
    return overrides, used


def main(argv=None):
    from control.validation_suite import write_json, environment_fingerprint, git_state
    from control.arena import config_sha256
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=ROOT/'results'/'arena')
    parser.add_argument('--tuned', type=Path, help='tuning run-dir: use each record\'s best values')
    parser.add_argument('--shape', choices=tuple(SHAPES), default='smooth',
                        help='smooth = eq.(32) smoothstep over 1 s (default); step = old instantaneous step')
    parser.add_argument('--feedforward', choices=('on', 'off'), default='on',
                        help='off drops reference_feedforward from GSLQR/CPID (pre-2026-09-26 afternoon)')
    parser.add_argument('--duration', type=float, default=DURATION,
                        help='observation seconds per point (diagnostic; the record uses 15 s)')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    if args.feedforward == 'off':
        from control.arena_feedforward_check import without_feedforward
        config = without_feedforward(config)
    overrides, tuned = tuned_overrides(config, args.tuned) if args.tuned else (None, None)
    factory = ArenaFactory(config, overrides=overrides)
    rows = []
    for label, speeds in POINTS.items():
        for V in speeds:
            for step_name in STEPS:
                print(f'{label} V={V:g} {step_name}', flush=True)
                rows.append(run(config, factory, label, V, step_name, shape=args.shape,
                                duration=args.duration))
    revision, dirty = git_state()
    command = 'python -m control.arena_design_check' + ''.join(
        f' --{k} {v:g}' if isinstance(v, float) else f' --{k} {v}'
        for k, v in (('shape', args.shape), ('feedforward', args.feedforward), ('duration', args.duration))
        if v != parser.get_default(k)) + (f' --tuned {args.tuned}' if args.tuned else '')
    meta = dict(label='PILOT', command=command, tuned=tuned, shape=args.shape, ramp_s=SHAPES[args.shape],
                feedforward=args.feedforward,
                created_utc=datetime.now(timezone.utc).isoformat(),
                config_sha256=config_sha256(config), git_revision=revision, git_dirty=dirty,
                controller_model_sha256=factory.controller_model_sha256,
                environment=environment_fingerprint(), gains=factory.gains,
                band_fraction=BAND, step_time=STEP_TIME, duration=args.duration)
    # 원계단 기록은 이름을 따로 둔다 — 지금 근거(매끄러운 계단)를 덮어쓰지 않게
    suffix = '' if args.shape == 'smooth' else '_step'
    args.out.mkdir(parents=True, exist_ok=True)
    write_json(args.out/f'design_check{suffix}.json', dict(meta=meta, rows=rows))
    write_markdown(args.out/f'DESIGN_CHECK{suffix}.md', rows, meta)
    print(json.dumps([{k: r[k] for k in ('controller', 'speed', 'step', 'settling_s',
                                         'overshoot_pct', 'steady_error')} for r in rows],
                     indent=1, ensure_ascii=False))
    return rows


if __name__ == '__main__':
    main()
