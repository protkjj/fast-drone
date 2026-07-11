"""
전 비행구간 통합 시뮬레이션 — 이륙->가속->순항(+돌풍)->감속->호버
================================================================

전체 비행을 하나로 이어붙여서 제어기 성능을 본다.
순항 구간에 1-cosine 수직돌풍을 삽입하여 외란 복원도 함께 검증.

미션 프로파일 (총 65초):
  이륙(0~10s) -> 안정화(10~13s) -> 가속(13~28s)
  -> 순항(28~43s, 돌풍 t=35s) -> 감속(43~58s) -> 호버링(58~65s)

몬테카를로:
  초기조건 + 돌풍 강도/시각을 랜덤화하여 N회 반복.
  통계적 성능 분포(평균, 표준편차, 최악) 확인.
"""

import numpy as np
import time as timer

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import ScheduledLQR
from nmpc import NMPCController
from hybrid_comparison import VirtualNMPC, ProperHybrid
from gust_comparison import make_gust_fn
from ekf_comparison import _reset_controller


# ════════════════════════════════════════════════════
# 1. 미션 프로파일
# ════════════════════════════════════════════════════

class MissionProfile:
    """
    비행 구간별 기준 궤적 생성.

    각 구간 내에서 1-cos 보간: 시작/끝 가속도 = 0 (매끄러운 전환).
    """

    def __init__(self, cruise_speed=70.0, cruise_alt=50.0):
        V = cruise_speed
        Z = cruise_alt

        # (이름, t_start, duration, vx_start, vx_end, z_start, z_end)
        self.phases = [
            ('이륙',    0.0,  10.0,   0.0,  0.0,   2.0,    Z),
            ('안정화',  10.0,   3.0,   0.0,  0.0,     Z,    Z),
            ('가속',   13.0,  15.0,   0.0,    V,     Z,    Z),
            ('순항',   28.0,  15.0,     V,    V,     Z,    Z),
            ('감속',   43.0,  15.0,     V,  0.0,     Z,    Z),
            ('호버링',  58.0,   7.0,   0.0,  0.0,     Z,    Z),
        ]
        self.T_total = sum(p[2] for p in self.phases)  # 65초
        self.cruise_speed = cruise_speed
        self.cruise_alt = cruise_alt

        # 순항 구간 시간 범위 (돌풍 랜덤화에 사용)
        cruise = self.phases[3]
        self.cruise_start = cruise[1]
        self.cruise_end = cruise[1] + cruise[2]

    def get_ref(self, t):
        """시각 t에서의 기준값. Returns: (v_ref[3], z_ref, phase_name)"""
        for name, t_start, dur, vx0, vx1, z0, z1 in self.phases:
            t_end = t_start + dur
            if t < t_end or name == self.phases[-1][0]:
                tau = np.clip((t - t_start) / dur, 0.0, 1.0)
                s = 0.5 * (1.0 - np.cos(np.pi * tau))
                vx = vx0 + (vx1 - vx0) * s
                z = z0 + (z1 - z0) * s
                return np.array([vx, 0.0, 0.0]), z, name

        last = self.phases[-1]
        return np.array([last[4], 0.0, 0.0]), last[6], last[0]

    def compute_refs(self, ts):
        """시간 배열에서 기준 궤적 벡터화 계산."""
        v_refs = np.zeros((len(ts), 3))
        z_refs = np.zeros(len(ts))

        for name, t_start, dur, vx0, vx1, z0, z1 in self.phases:
            t_end = t_start + dur
            mask = (ts >= t_start) & (ts < t_end)
            if not np.any(mask):
                continue
            tau = np.clip((ts[mask] - t_start) / dur, 0.0, 1.0)
            s = 0.5 * (1.0 - np.cos(np.pi * tau))
            v_refs[mask, 0] = vx0 + (vx1 - vx0) * s
            z_refs[mask] = z0 + (z1 - z0) * s

        last = self.phases[-1]
        mask = ts >= (last[1] + last[2])
        v_refs[mask, 0] = last[4]
        z_refs[mask] = last[6]

        return v_refs, z_refs

    def get_phase_boundaries(self):
        return [(name, t0, t0 + dur)
                for name, t0, dur, *_ in self.phases]

    def print_profile(self):
        print(f"\n  {'구간':>6s}  {'시간':>10s}  "
              f"{'v_x [m/s]':>12s}  {'z [m]':>10s}  {'최대가속':>10s}")
        print(f"  {'─'*56}")
        for name, t0, dur, vx0, vx1, z0, z1 in self.phases:
            t1 = t0 + dur
            v_str = f"{vx0:.0f} -> {vx1:.0f}" if vx0 != vx1 else f"{vx0:.0f}"
            z_str = f"{z0:.0f} -> {z1:.0f}" if z0 != z1 else f"{z0:.0f}"
            if dur > 0 and vx0 != vx1:
                a_str = f"{np.pi/2*abs(vx1-vx0)/dur:.1f} m/s2"
            elif dur > 0 and z0 != z1:
                a_str = f"{np.pi/2*abs(z1-z0)/dur:.1f} m/s2(z)"
            else:
                a_str = "-"
            print(f"  {name:>6s}  {t0:>4.0f}~{t1:>4.0f}s  "
                  f"{v_str:>12s}  {z_str:>10s}  {a_str:>10s}")
        print(f"\n  총 시뮬 시간: {self.T_total:.0f}초")


# ════════════════════════════════════════════════════
# 2. 미션 제어기 래퍼
# ════════════════════════════════════════════════════

class MissionController:
    """미션 프로파일에 따라 제어기 기준값을 동적 갱신하는 래퍼."""

    def __init__(self, inner_ctrl, profile):
        self.inner = inner_ctrl
        self.profile = profile

    def __call__(self, t, x):
        v_ref, z_ref, _ = self.profile.get_ref(t)

        if hasattr(self.inner, 'v_ref'):
            self.inner.v_ref = v_ref
        if hasattr(self.inner, 'z_ref'):
            self.inner.z_ref = z_ref

        # Hybrid: 내부 VirtualNMPC의 기준도 갱신
        if hasattr(self.inner, 'nmpc'):
            if hasattr(self.inner.nmpc, 'v_ref'):
                self.inner.nmpc.v_ref = v_ref
            if hasattr(self.inner.nmpc, 'z_ref'):
                self.inner.nmpc.z_ref = z_ref

        return self.inner(t, x)

    def reset(self):
        _reset_controller(self.inner)


# ════════════════════════════════════════════════════
# 3. 시뮬레이션 + 지표
# ════════════════════════════════════════════════════

def run_mission(plant, ctrl, x0, profile, wind_fn=None):
    """전 구간 통합 시뮬레이션. wind_fn으로 돌풍 주입 가능."""
    ts, xs, us = plant.simulate(x0, ctrl, profile.T_total, wind_fn=wind_fn)
    v_refs, z_refs = profile.compute_refs(ts)
    return {'ts': ts, 'xs': xs, 'us': us, 'v_refs': v_refs, 'z_refs': z_refs}


def run_mission_ekf(plant, ctrl, x0, profile, sensors, wind_fn=None, seed=42):
    """
    RTK EKF 포함 미션 시뮬레이션.

    Plant(참값) → Sensors(노이즈) → ESKF(추정) → Controller → Plant.
    성능 평가는 참값(xs_true) 기준.
    """
    from ekf_sim import simulate_with_ekf
    sensors.reset()
    ctrl.reset()
    res = simulate_with_ekf(plant, ctrl, x0, profile.T_total,
                            sensors=sensors, seed=seed, wind_fn=wind_fn)
    v_refs, z_refs = profile.compute_refs(res['ts'])
    return {
        'ts': res['ts'],
        'xs': res['xs_true'],       # 참값 기준 성능 평가
        'xs_est': res['xs_est'],
        'us': res['us'],
        'v_refs': v_refs,
        'z_refs': z_refs,
    }


def compute_phase_metrics(result, profile):
    """구간별 RMSE/최대편차."""
    ts, xs = result['ts'], result['xs']
    v_refs, z_refs = result['v_refs'], result['z_refs']
    metrics = []
    for name, t_start, t_end in profile.get_phase_boundaries():
        mask = (ts >= t_start) & (ts < t_end)
        if not np.any(mask):
            continue
        _seg = xs[mask]
        _z_dev = np.nanmax(np.abs(_seg[:, 2] - z_refs[mask])) if len(_seg) else 0.0
        # 발산 판정: NaN/비유한 또는 고도가 참조에서 >50m 이탈
        # (유한하지만 튄 궤적을 '성공'으로 집계하지 않도록; hybrid_comparison |z-z_ref|>50 기준과 일치)
        if (not np.all(np.isfinite(_seg))) or _z_dev > 50.0:
            metrics.append({'name': name, 'diverged': True,
                            'rmse_vx': np.inf, 'rmse_z': np.inf,
                            'max_vx_err': np.inf, 'max_z_err': np.inf})
            continue
        vx, z = xs[mask, 3], xs[mask, 2]
        vr, zr = v_refs[mask, 0], z_refs[mask]
        metrics.append({
            'name': name, 'diverged': False,
            'rmse_vx': np.sqrt(np.mean((vx - vr)**2)),
            'rmse_z': np.sqrt(np.mean((z - zr)**2)),
            'max_vx_err': np.max(np.abs(vx - vr)),
            'max_z_err': np.max(np.abs(z - zr)),
        })
    return metrics


def compute_overall(result):
    """전체 구간 통합 RMSE."""
    xs = result['xs']
    if np.any(np.isnan(xs)):
        return {'rmse_vx': np.inf, 'rmse_z': np.inf, 'diverged': True}
    return {
        'rmse_vx': np.sqrt(np.mean((xs[:, 3] - result['v_refs'][:, 0])**2)),
        'rmse_z': np.sqrt(np.mean((xs[:, 2] - result['z_refs'])**2)),
        'max_vx_err': np.max(np.abs(xs[:, 3] - result['v_refs'][:, 0])),
        'max_z_err': np.max(np.abs(xs[:, 2] - result['z_refs'])),
        'diverged': False,
    }


# ════════════════════════════════════════════════════
# 4. 몬테카를로
# ════════════════════════════════════════════════════

def run_monte_carlo(plant, ctrl, profile, n_trials=20, seed=0):
    """
    몬테카를로: 초기조건 + 돌풍을 랜덤화하여 n_trials 반복.

    랜덤화 항목:
      - 초기 고도: z0 = 2.0 + N(0, 0.5) m
      - 초기 속도: vx ~ N(0, 0.5), vz ~ N(0, 0.3) m/s
      - 돌풍 강도: W_max ~ U(5, 15) m/s
      - 돌풍 시각: t_gust ~ U(순항시작+2, 순항끝-3) s
      - 돌풍 방향: 수직(z) 고정 (가장 까다로운 조건)

    Returns: list of dict (rmse_vx, rmse_z, diverged, W_max, t_gust)
    """
    rng = np.random.default_rng(seed)
    results = []

    cs, ce = profile.cruise_start, profile.cruise_end

    for i in range(n_trials):
        # ── 랜덤 초기 조건 ──
        x0 = AxialDronePlant.hover_state(P)
        x0[2] = 2.0 + rng.normal(0, 0.5)
        x0[3] = rng.normal(0, 0.5)
        x0[5] = rng.normal(0, 0.3)

        # ── 랜덤 돌풍 ──
        W = float(rng.uniform(5, 15))
        t_g = float(rng.uniform(cs + 2, ce - 3))
        gust_fn = make_gust_fn('vertical', W, t_g, 1.0)

        # ── 시뮬 ──
        ctrl.reset()
        res = run_mission(plant, ctrl, x0, profile, wind_fn=gust_fn)
        ov = compute_overall(res)

        results.append({
            **ov,
            'W_max': W,
            't_gust': t_g,
        })

        status = '발산' if ov['diverged'] else f"z={ov['rmse_z']:.3f}"
        print(f"\r    [{i+1}/{n_trials}] {status}      ", end="", flush=True)

    print()  # 줄바꿈
    return results


def print_mc_summary(name, mc_results):
    """몬테카를로 결과 통계 출력."""
    valid = [r for r in mc_results if not r['diverged']]
    n_div = len(mc_results) - len(valid)

    if not valid:
        print(f"  {name:>8s}: 전부 발산 ({n_div}/{len(mc_results)})")
        return

    z_vals = np.array([r['rmse_z'] for r in valid])
    vx_vals = np.array([r['rmse_vx'] for r in valid])

    print(f"  {name:>8s}  ({len(valid)}/{len(mc_results)} 성공"
          f"{f', {n_div} 발산' if n_div else ''})")
    print(f"    RMSE z:  평균={np.mean(z_vals):.3f}  "
          f"std={np.std(z_vals):.3f}  "
          f"최소={np.min(z_vals):.3f}  최대={np.max(z_vals):.3f}")
    print(f"    RMSE vx: 평균={np.mean(vx_vals):.3f}  "
          f"std={np.std(vx_vals):.3f}  "
          f"최소={np.min(vx_vals):.3f}  최대={np.max(vx_vals):.3f}")

    # 돌풍 강도 vs 성능 상관
    W_vals = np.array([r['W_max'] for r in valid])
    corr = np.corrcoef(W_vals, z_vals)[0, 1] if len(valid) > 2 else 0
    print(f"    돌풍강도-RMSE z 상관: r={corr:.2f}")


# ════════════════════════════════════════════════════
# 5. 감속 폭발 진단
# ════════════════════════════════════════════════════

def diagnose_deceleration(n_trials=30, seed=0, nmpc_N=20):
    """
    Hybrid 감속 폭발 재현 조건 특정.

    고정: 돌풍 10 m/s at t=35s (단일 시뮬과 동일)
    랜덤: 초기 z(±0.5m), vx(±0.5), vz(±0.3)

    "얼마나 자주, 얼마나 심한가"를 정량화.
    감속 구간(43~58s)의 max_z_err 분포를 본다.

    Parameters
    ----------
    nmpc_N : int
        VirtualNMPC 예측 지평선 스텝 수.
        N=20 (기본, 1.0초), N=40 (확장, 2.0초).
    """
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt
    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    gust_fn = make_gust_fn('vertical', 10.0, 35.0, 1.0)

    horizon_sec = nmpc_N * 0.05
    print(f"\n[준비] Hybrid 제어기 생성 (N={nmpc_N}, 지평선={horizon_sec:.1f}s)...")
    vnmpc = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                         N=nmpc_N, dt_nmpc=0.05, dt_ctrl=0.02)
    hybrid = ProperHybrid(vnmpc, P, dt=dt)
    ctrl = MissionController(hybrid, profile)

    rng = np.random.default_rng(seed)

    # 감속 구간 시간
    decel_start, decel_end = 43.0, 58.0

    print(f"\n{'='*60}")
    print(f"  Hybrid 감속 폭발 진단 ({n_trials}회)")
    print(f"  VirtualNMPC N={nmpc_N} (지평선 {horizon_sec:.1f}s)")
    print(f"  고정: 돌풍 10 m/s at t=35s")
    print(f"  랜덤: 초기조건 (z±0.5, vx±0.5, vz±0.3)")
    print(f"  관찰: 감속 구간 (t={decel_start:.0f}~{decel_end:.0f}s) max Dz")
    print(f"{'='*60}")

    decel_max_dz = []
    decel_rmse_z = []
    overall_rmse_z = []

    t0_all = timer.time()
    for i in range(n_trials):
        x0 = AxialDronePlant.hover_state(P)
        x0[2] = 2.0 + rng.normal(0, 0.5)
        x0[3] = rng.normal(0, 0.5)
        x0[5] = rng.normal(0, 0.3)

        ctrl.reset()
        res = run_mission(plant, ctrl, x0, profile, wind_fn=gust_fn)

        ts, xs = res['ts'], res['xs']
        z_refs = res['z_refs']

        # 감속 구간 추출
        mask = (ts >= decel_start) & (ts < decel_end)
        if np.any(np.isnan(xs[mask])):
            decel_max_dz.append(np.inf)
            decel_rmse_z.append(np.inf)
        else:
            dz = np.abs(xs[mask, 2] - z_refs[mask])
            decel_max_dz.append(np.max(dz))
            decel_rmse_z.append(np.sqrt(np.mean((xs[mask, 2] - z_refs[mask])**2)))

        ov = compute_overall(res)
        overall_rmse_z.append(ov['rmse_z'])

        print(f"\r  [{i+1}/{n_trials}] 감속 max_dz={decel_max_dz[-1]:.2f}m  "
              f"전체 z={ov['rmse_z']:.3f}      ", end="", flush=True)

    elapsed = timer.time() - t0_all
    print(f"\n  완료 [{elapsed:.0f}s]")

    dz_arr = np.array(decel_max_dz)
    rz_arr = np.array(decel_rmse_z)
    oz_arr = np.array(overall_rmse_z)

    # ── 결과 ──
    print(f"\n{'='*60}")
    print(f"  결과 — 감속 구간 max Dz 분포 ({n_trials}회)")
    print(f"{'='*60}")

    # 심각도 구간 분류
    bins = [
        ('< 2m (양호)',     dz_arr < 2),
        ('2~5m (주의)',     (dz_arr >= 2) & (dz_arr < 5)),
        ('5~10m (위험)',    (dz_arr >= 5) & (dz_arr < 10)),
        ('10~20m (심각)',   (dz_arr >= 10) & (dz_arr < 20)),
        ('>= 20m (대참사)', dz_arr >= 20),
    ]

    print(f"\n  심각도 분포:")
    for label, mask in bins:
        count = np.sum(mask)
        pct = 100 * count / n_trials
        if count > 0:
            vals = dz_arr[mask]
            print(f"    {label:>20s}: {count:>3d}회 ({pct:>5.1f}%)  "
                  f"평균={np.mean(vals):.2f}m")
        else:
            print(f"    {label:>20s}: {count:>3d}회 ({pct:>5.1f}%)")

    print(f"\n  통계:")
    finite = dz_arr[np.isfinite(dz_arr)]
    if len(finite) > 0:
        print(f"    평균: {np.mean(finite):.2f} m")
        print(f"    중앙: {np.median(finite):.2f} m")
        print(f"    std:  {np.std(finite):.2f} m")
        print(f"    최소: {np.min(finite):.2f} m")
        print(f"    최대: {np.max(finite):.2f} m")
        print(f"    P90:  {np.percentile(finite, 90):.2f} m")
        print(f"    P95:  {np.percentile(finite, 95):.2f} m")

    # 위험 비율
    n_danger = np.sum(dz_arr >= 5)
    print(f"\n  판정: 감속 max Dz >= 5m 발생률 = "
          f"{n_danger}/{n_trials} ({100*n_danger/n_trials:.0f}%)")

    if n_danger == 0:
        print(f"  => 30회 중 위험 케이스 없음. 단일 시뮬의 22m는 극히 드문 케이스.")
        print(f"     폴백으로 충분, 지평선 확장 불필요.")
    elif n_danger <= n_trials * 0.1:
        print(f"  => 10% 이하 드문 케이스. 폴백으로 대응 가능.")
    else:
        print(f"  => {100*n_danger/n_trials:.0f}% 빈도 — 구조적 문제.")
        print(f"     지평선 확장(N=40) 또는 switching 필요.")

    print(f"\n{'='*60}")


# ════════════════════════════════════════════════════
# 6. 시각화
# ════════════════════════════════════════════════════

def plot_mission(results, profile, save_path='results/mission_plot.png'):
    """
    미션 시뮬레이션 결과 시각화.

    4행 플롯:
      1) v_x 추종 (실선=실제, 점선=기준)
      2) z 추종
      3) 고도 오차 Δz
      4) 모터 평균 속도

    구간 경계를 수직선으로, 돌풍 구간을 음영으로 표시.
    """
    import matplotlib
    matplotlib.use('Agg')  # 디스플레이 없이 파일 저장
    import matplotlib.pyplot as plt

    ctrl_names = list(results.keys())
    colors = {'LQR': '#2196F3', 'Hybrid': '#FF5722', 'NMPC': '#4CAF50'}
    phase_bounds = profile.get_phase_boundaries()

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('Mission Profile Simulation', fontsize=14, fontweight='bold')

    for cn in ctrl_names:
        res = results[cn]
        ts = res['ts']
        xs = res['xs']
        vr = res['v_refs']
        zr = res['z_refs']
        us = res['us']
        c = colors.get(cn, 'gray')

        # 1) v_x 추종
        axes[0].plot(ts, xs[:, 3], color=c, linewidth=0.8, label=cn)

        # 2) z 추종
        axes[1].plot(ts, xs[:, 2], color=c, linewidth=0.8, label=cn)

        # 3) Δz
        axes[2].plot(ts, xs[:, 2] - zr, color=c, linewidth=0.8, label=cn)

        # 4) 모터 평균
        n_avg = np.mean(us, axis=1)
        axes[3].plot(ts[:-1], n_avg, color=c, linewidth=0.6, label=cn)

    # 기준 궤적 (점선, 첫 제어기 데이터 사용)
    first = results[ctrl_names[0]]
    axes[0].plot(first['ts'], first['v_refs'][:, 0],
                 'k--', linewidth=1.0, alpha=0.5, label='Ref')
    axes[1].plot(first['ts'], first['z_refs'],
                 'k--', linewidth=1.0, alpha=0.5, label='Ref')

    # 축 라벨
    axes[0].set_ylabel('v_x [m/s]')
    axes[0].set_title('Forward Velocity Tracking')
    axes[1].set_ylabel('z [m]')
    axes[1].set_title('Altitude Tracking')
    axes[2].set_ylabel('Δz [m]')
    axes[2].set_title('Altitude Error')
    axes[2].axhline(0, color='k', linewidth=0.5)
    axes[3].set_ylabel('Motor avg [rad/s]')
    axes[3].set_title('Motor Speed (average)')
    axes[3].set_xlabel('Time [s]')

    # 구간 경계 + 돌풍 음영
    for ax in axes:
        for name, t0, t1 in phase_bounds:
            ax.axvline(t0, color='gray', linewidth=0.5, alpha=0.3)
        ax.axvspan(35, 36, color='red', alpha=0.15)
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, alpha=0.2)

    # 구간 이름 (영어)
    phase_en = {'이륙': 'Takeoff', '안정화': 'Stab', '가속': 'Accel',
                '순항': 'Cruise', '감속': 'Decel', '호버링': 'Hover'}
    for name, t0, t1 in phase_bounds:
        mid = (t0 + t1) / 2
        axes[0].text(mid, axes[0].get_ylim()[1] * 0.95,
                     phase_en.get(name, name),
                     ha='center', fontsize=8, alpha=0.6)

    # 돌풍 라벨
    axes[2].annotate('Gust', xy=(35.5, 0),
                     xytext=(37.5, axes[2].get_ylim()[1]*0.7),
                     fontsize=9, color='red', fontweight='bold',
                     arrowprops=dict(arrowstyle='->', color='red', lw=1.5),
                     ha='center')

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n  [저장] {save_path}")

    # macOS에서 자동 열기
    import subprocess
    try:
        subprocess.Popen(['open', save_path])
    except Exception:
        pass


# ════════════════════════════════════════════════════
# 7. 출력 헬퍼
# ════════════════════════════════════════════════════

def _print_metric_table(title, key, ctrl_names, all_pm, phase_bounds, results):
    """구간별 지표 테이블 출력."""
    print(f"\n  {title}")
    print(f"  {'구간':>6s}", end="")
    for cn in ctrl_names:
        print(f"  {cn:>10s}", end="")
    print()
    print(f"  {'─'*(8 + 12*len(ctrl_names))}")

    for i, (pname, _, _) in enumerate(phase_bounds):
        print(f"  {pname:>6s}", end="")
        for cn in ctrl_names:
            pm = all_pm[cn]
            if i < len(pm) and not pm[i]['diverged']:
                print(f"  {pm[i][key]:>10.4f}", end="")
            else:
                print(f"  {'발산':>10s}", end="")
        print()

    print(f"  {'전체':>6s}", end="")
    for cn in ctrl_names:
        ov = compute_overall(results[cn])
        if ov['diverged'] or key not in ov:
            print(f"  {'발산':>10s}", end="")
        else:
            print(f"  {ov[key]:>10.4f}", end="")
    print()


# ════════════════════════════════════════════════════
# 6. 메인
# ════════════════════════════════════════════════════

def main():
    plant = AxialDronePlant(P, dt=0.001)
    dt = plant.dt

    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)

    print("\n" + "=" * 70)
    print("  전 비행구간 통합 시뮬레이션 (돌풍 포함)")
    print("  이륙 -> 가속 -> 순항(+돌풍) -> 감속 -> 호버링")
    print("=" * 70)

    profile.print_profile()

    # 초기 상태
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 2.0

    # 돌풍: 순항 중간(t=35s)에 수직 10 m/s, 1초 지속
    gust_fn = make_gust_fn('vertical', 10.0, 35.0, 1.0)
    print(f"\n  돌풍: 수직 10 m/s, t=35~36s (순항 중)")

    trim_0 = find_trim(P, 0.0)

    # ── 제어기 생성 ──
    print("\n[준비] 제어기 생성...")

    controllers = {}

    lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=2.0)
    controllers['LQR'] = MissionController(lqr, profile)

    vnmpc = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                         N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    hybrid = ProperHybrid(vnmpc, P, dt=dt)
    controllers['Hybrid'] = MissionController(hybrid, profile)

    nmpc = NMPCController(P, v_ref=[0, 0, 0], z_ref=2.0,
                           u_ref=trim_0['control'],
                           N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    controllers['NMPC'] = MissionController(nmpc, profile)

    # ════════════════════════════════════════════════
    # 단일 시뮬레이션 (3 제어기)
    # ════════════════════════════════════════════════
    print(f"\n{'='*70}")
    print("  [1] 단일 미션 시뮬레이션 (돌풍 포함)")
    print(f"{'='*70}")

    results = {}
    ctrl_names = list(controllers.keys())

    for name, ctrl in controllers.items():
        ctrl.reset()
        print(f"\n  {name} 시뮬 시작...", end=" ", flush=True)
        t0 = timer.time()
        result = run_mission(plant, ctrl, x0.copy(), profile, wind_fn=gust_fn)
        elapsed = timer.time() - t0

        if np.any(np.isnan(result['xs'])):
            nan_idx = np.where(np.any(np.isnan(result['xs']), axis=1))[0]
            t_div = result['ts'][nan_idx[0]]
            _, _, phase = profile.get_ref(t_div)
            print(f"발산! t={t_div:.1f}s ({phase}) [{elapsed:.1f}s]")
        else:
            ov = compute_overall(result)
            print(f"완료 [{elapsed:.1f}s]  "
                  f"RMSE: vx={ov['rmse_vx']:.3f}, z={ov['rmse_z']:.3f}")

        results[name] = result

    # ── 시각화 ──
    plot_mission(results, profile, save_path='results/mission_plot.png')

    # ── 구간별 비교 ──
    all_pm = {cn: compute_phase_metrics(results[cn], profile)
              for cn in ctrl_names}
    phase_bounds = profile.get_phase_boundaries()

    _print_metric_table('RMSE v_x [m/s]', 'rmse_vx',
                        ctrl_names, all_pm, phase_bounds, results)
    _print_metric_table('RMSE z [m]', 'rmse_z',
                        ctrl_names, all_pm, phase_bounds, results)
    _print_metric_table('최대 Dz [m]', 'max_z_err',
                        ctrl_names, all_pm, phase_bounds, results)

    # ── 구간 전환 + 돌풍 응답 ──
    print(f"\n{'='*70}")
    print("  구간 전환 + 돌풍 응답")
    print(f"{'='*70}")

    transitions = [
        ('이륙 상승',     0.0,  11.0),
        ('가속 시작',    13.0,  18.0),
        ('순항 진입',    26.0,  30.0),
        ('돌풍 응답',    35.0,  39.0),
        ('감속 시작',    43.0,  48.0),
        ('호버 진입',    56.0,  61.0),
    ]

    for tr_name, t_a, t_b in transitions:
        print(f"\n  {tr_name} (t={t_a:.0f}~{t_b:.0f}s)")
        print(f"  {'제어기':>8s}  {'최대Dvx':>8s}  {'최대Dz':>8s}  "
              f"{'최종vx':>8s}  {'최종z':>8s}")
        print(f"  {'─'*44}")

        for cn in ctrl_names:
            ts = results[cn]['ts']
            xs = results[cn]['xs']
            vr = results[cn]['v_refs']
            zr = results[cn]['z_refs']

            mask = (ts >= t_a) & (ts <= t_b)
            if not np.any(mask) or np.any(np.isnan(xs[mask])):
                print(f"  {cn:>8s}  {'N/A':>8s}  {'N/A':>8s}  "
                      f"{'N/A':>8s}  {'N/A':>8s}")
                continue

            max_dvx = np.max(np.abs(xs[mask, 3] - vr[mask, 0]))
            max_dz = np.max(np.abs(xs[mask, 2] - zr[mask]))
            idx_end = np.where(mask)[0][-1]

            print(f"  {cn:>8s}  {max_dvx:>8.3f}  {max_dz:>8.3f}  "
                  f"{xs[idx_end, 3]:>8.2f}  {xs[idx_end, 2]:>8.2f}")

    # ── 최종 상태 ──
    print(f"\n{'='*70}")
    print(f"  최종 상태 (t={profile.T_total:.0f}초) -- 기준: v=0, z=50m")
    print(f"{'='*70}")

    n_hov = np.sqrt(P['mass'] * P['g'] / (4 * P['k_T']))
    print(f"\n  {'제어기':>8s}  {'v_x':>8s}  {'v_z':>8s}  "
          f"{'z':>8s}  {'|v|':>8s}")
    print(f"  {'─'*38}")

    for cn in ctrl_names:
        xf = results[cn]['xs'][-1]
        if np.any(np.isnan(xf)):
            print(f"  {cn:>8s}  발산")
            continue
        v = xf[3:6]
        print(f"  {cn:>8s}  {v[0]:>8.3f}  {v[2]:>8.3f}  "
              f"{xf[2]:>8.3f}  {np.linalg.norm(v):>8.3f}")

    # ── 종합 순위 ──
    print(f"\n  전체 RMSE z 순위:")
    ranked = sorted(ctrl_names,
                    key=lambda cn: compute_overall(results[cn]).get('rmse_z', np.inf))
    for i, cn in enumerate(ranked):
        ov = compute_overall(results[cn])
        z = ov['rmse_z'] if not ov['diverged'] else np.inf
        marker = " *" if i == 0 else ""
        print(f"    {i+1}. {cn}: {z:.4f} m{marker}")

    # ════════════════════════════════════════════════
    # 몬테카를로 (LQR + Hybrid)
    # ════════════════════════════════════════════════
    N_MC = 10

    print(f"\n{'='*70}")
    print(f"  [2] 몬테카를로 ({N_MC}회)")
    print(f"  랜덤: 초기조건(z,vx,vz) + 돌풍(강도 5~15 m/s, 시각 랜덤)")
    print(f"  LQR ~5초/회, Hybrid ~50초/회")
    print(f"{'='*70}")

    mc_targets = {
        'LQR': controllers['LQR'],
        'Hybrid': controllers['Hybrid'],
    }

    mc_results = {}

    for name, ctrl in mc_targets.items():
        print(f"\n  {name} 몬테카를로 ({N_MC}회)...")
        t0 = timer.time()
        mc = run_monte_carlo(plant, ctrl, profile, n_trials=N_MC, seed=42)
        elapsed = timer.time() - t0
        print(f"  완료 [{elapsed:.1f}s]")
        mc_results[name] = mc

    # ── MC 결과 요약 ──
    print(f"\n{'='*70}")
    print(f"  몬테카를로 결과 (N={N_MC})")
    print(f"{'='*70}\n")

    for name in mc_targets:
        print_mc_summary(name, mc_results[name])
        print()

    # ── MC 비교 ──
    lqr_valid = [r['rmse_z'] for r in mc_results['LQR'] if not r['diverged']]
    hyb_valid = [r['rmse_z'] for r in mc_results['Hybrid'] if not r['diverged']]

    if lqr_valid and hyb_valid:
        lqr_mean = np.mean(lqr_valid)
        hyb_mean = np.mean(hyb_valid)
        improvement = (lqr_mean - hyb_mean) / lqr_mean * 100

        print(f"  Hybrid vs LQR:")
        print(f"    평균 RMSE z: Hybrid {hyb_mean:.3f} vs LQR {lqr_mean:.3f}"
              f" ({improvement:+.1f}%)")
        print(f"    Hybrid 최악: {np.max(hyb_valid):.3f}  "
              f"vs  LQR 최선: {np.min(lqr_valid):.3f}")

        if np.max(hyb_valid) < np.min(lqr_valid):
            print(f"    => Hybrid 최악 < LQR 최선: 통계적 우위 확실")
        else:
            print(f"    => 구간이 겹침: 조건에 따라 역전 가능")

    # ════════════════════════════════════════════════
    # RTK EKF 미션 (LQR + Hybrid)
    # ════════════════════════════════════════════════
    print(f"\n{'='*70}")
    print("  [3] RTK EKF 미션 (RTK GPS sigma=0.02m)")
    print("  센서노이즈 하에서 제어기 성능 재검증")
    print(f"{'='*70}")

    from ekf_comparison import create_sensors
    sensors_rtk = create_sensors(dt, noise_level=1.0, gps_noise_pos=0.02)

    ekf_targets = {'LQR': controllers['LQR'], 'Hybrid': controllers['Hybrid']}
    results_ekf = {}

    for name, ctrl in ekf_targets.items():
        print(f"\n  {name}+RTK EKF...", end=" ", flush=True)
        t0 = timer.time()
        res_ekf = run_mission_ekf(plant, ctrl, x0.copy(), profile,
                                   sensors_rtk, wind_fn=gust_fn, seed=42)
        elapsed = timer.time() - t0

        if np.any(np.isnan(res_ekf['xs'])):
            print(f"발산! [{elapsed:.1f}s]")
        else:
            ov = compute_overall(res_ekf)
            print(f"완료 [{elapsed:.1f}s]  "
                  f"RMSE: vx={ov['rmse_vx']:.3f}, z={ov['rmse_z']:.3f}")
        results_ekf[name] = res_ekf

    # ── 참값 vs RTK EKF 비교 ──
    print(f"\n  참값 vs RTK EKF 비교 (RMSE z)")
    print(f"  {'제어기':>8s}  {'참값':>8s}  {'RTK EKF':>8s}  {'저하율':>8s}")
    print(f"  {'─'*36}")

    for name in ekf_targets:
        ov_p = compute_overall(results[name])
        ov_e = compute_overall(results_ekf[name])
        if ov_p['diverged'] or ov_e['diverged']:
            print(f"  {name:>8s}  발산")
            continue
        deg = ((ov_e['rmse_z'] - ov_p['rmse_z']) / max(ov_p['rmse_z'], 1e-6)) * 100
        print(f"  {name:>8s}  {ov_p['rmse_z']:>8.3f}  {ov_e['rmse_z']:>8.3f}  {deg:>+7.1f}%")

    # 구간별 EKF 비교
    ekf_names = list(ekf_targets.keys())
    all_pm_ekf = {cn: compute_phase_metrics(results_ekf[cn], profile)
                  for cn in ekf_names}
    _print_metric_table('RTK EKF — RMSE z [m]', 'rmse_z',
                        ekf_names, all_pm_ekf, phase_bounds, results_ekf)

    # EKF 결과 플롯
    plot_mission(results_ekf, profile, save_path='results/mission_ekf_plot.png')

    print(f"\n{'='*70}")
    print("  완료!")
    print(f"{'='*70}")


if __name__ == '__main__':
    main()
