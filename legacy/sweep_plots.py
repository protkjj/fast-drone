"""
시나리오 A (정적 순항 속도 스윕) — 성능 저하 곡선 + 고정 vs 스케줄링 비교 플롯
==============================================================================

legacy/sweep.py, legacy/sweep_scheduled.py 는 텍스트 출력만 하고 플롯을 저장하지
않았다. 이 스크립트는 두 실험의 시나리오 A(순항 교란 회복)를 재실행해서
발표용 PNG와 표 작성용 txt를 results/ 에 저장한다. legacy 원본은 수정하지 않고,
제어기/플랜트는 루트 모듈에서 직접 import 한다.

실험 조건 (legacy 두 스크립트와 동일):
  - 속도 그리드 30~85 m/s, 5 m/s 간격 (트림은 85까지 수렴 확인됨)
  - 각 속도의 트림 상태에서 시작, 교란 Δvx=+1, Δvz=+2 m/s 부여
  - T_sim=10 s (sweep.py 기준), dt=1 ms, z_ref=50 m
  - 지표: RMSE vx, RMSE z, 로터 포화율(n≥0.95·n_max 시간 비율), 발산 여부

제어기 5종 (NMPC 제외 — 발표 범위 밖):
  1. PID 고정        : CascadedPID. 게인 하드코딩(저속 튜닝), v_ref만 변경
  2. PID 스케줄      : ScheduledPID. 매 스텝 v_x 기반 게인/틸트제한 보간
  3. LQR 고정        : 게인(K_r)과 트림 피드포워드(자세·로터속도) 모두
                       30 m/s 설계값 고정. 속도 목표만 x_trim[3]=V로 갱신.
                       "저속 튜닝 제어기를 그대로 고속에 사용"하는 순진한 배치.
  4. LQR 게인만 고정 : K_r만 30 m/s 고정, 트림은 각 속도 정확값 사용
                       (sweep_scheduled.py의 _FixedLQR와 동일). 게인 불일치의
                       영향만 분리해서 보는 진단용.
  5. LQR 스케줄      : ScheduledLQR. 속도별 K_r/트림을 선형 보간.
                       V_table에 85 추가 — 기본 테이블(0~80)은 85 m/s에서
                       80 m/s 트림을 목표로 삼아 인공 오차가 생기기 때문.

── 2026-07-29 실행에서 확인된 사실 (플롯 해석 시 주의) ─────────────────────
  - 이 시나리오(정적 순항, 소교란 회복)에서 RMSE_z 는 LQR 계열이 전부 평평하다.
    속도가 높을수록 공력 감쇠(dD/dv ∝ V)가 커져 회복이 오히려 쉬워지기 때문
    (오픈루프 RMSE_vx가 30→85 m/s에서 2.54→0.69로 감소하는 것으로 확인).
  - K_r은 30↔85 m/s 간 52% 다르지만(자세 블록 60%) 성능 차이는 미미
    → "게인만 고정"(4번)은 저하가 거의 없다. 고정 게인의 실제 저하는
    트림 피드포워드 불일치에서 오고(3번), RMSE_z가 아닌 RMSE_vx 에 나타난다
    (0.86→4.93 m/s, 5.7배 선형 증가).
  - 따라서 "고정↑ vs 스케줄 평평" 대비는 RMSE_vx + 엄격 고정(3번) 조합에서만
    보인다. z축만 그린 버전과 vx+z 2패널 버전을 모두 저장하니 발표에 맞는
    쪽을 고를 것.
  - legacy/sweep.py의 고정 LQR은 현재 controller.py(K_r 리팩터링 이후)와
    인터페이스가 어긋나 재실행 시 전 속도 DIV로 나온다 → 재사용하지 않았다.

실행: python3 sweep_plots.py   (약 1분)
산출물:
  results/sweep_knee.png / .txt                  — 플롯 1: 성능 저하 곡선 (RMSE z)
  results/sweep_knee_vxz.png                     — 플롯 1 확장: vx+z 2패널
  results/sweep_fixed_vs_scheduled.png / .txt    — 플롯 2: 고정 vs 스케줄 (RMSE z)
  results/sweep_fixed_vs_scheduled_vxz.png       — 플롯 2 확장: vx+z 2패널
"""

from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')          # 화면 없이 파일로만 저장
import matplotlib.pyplot as plt

from vehicle_params import vehicle_params as P
from dynamics import AxialDronePlant
from trim import find_trim
from controller import CascadedPID, ScheduledPID, LQRController, ScheduledLQR

# ══════════════════════════════════════════════════
# 실험 조건
# ══════════════════════════════════════════════════

V_RANGE = np.arange(30, 90, 5, dtype=float)   # 30~85 m/s
T_SIM = 10.0
Z_REF = 50.0
DT = 0.001
SAT_THRESHOLD_PCT = 5.0     # 이 비율 넘으면 "포화 시작"으로 표기
KNEE_FACTOR = 3.0           # 30 m/s 대비 RMSE 3배 → 무릎점

RESULTS_DIR = Path(__file__).parent / 'results'

# 색: dataviz 팔레트 검증 통과 조합 (CVD ΔE 74.6)
# 계열(hue) = 제어기 종류, 선스타일 = 고정/스케줄 — 색맹·흑백 인쇄에서도 구분됨
C_PID = '#2a78d6'   # blue
C_LQR = '#e34948'   # red


# ══════════════════════════════════════════════════
# LQR 고정 변형 2종
# ══════════════════════════════════════════════════

class StrictFixedLQR:
    """게인+트림 모두 30 m/s 설계값 고정. 속도 목표만 x_trim[3]=V로 갱신.

    저속에서 튜닝한 제어기를 수정 없이 고속에 투입하는 상황을 모사한다.
    자세·로터속도 피드포워드가 30 m/s 것이므로, 부족분은 전부 피드백이
    메워야 하고 적분기가 없어 속도 정상상태 오차가 속도에 비례해 커진다.
    """

    def __init__(self, params, K_r, x_trim_30, u_trim_30, V, z_ref):
        self.p = params
        self.K_r = K_r
        self.x_trim = x_trim_30.copy()
        self.u_trim = u_trim_30.copy()
        self.x_trim[3] = V
        self.x_trim[2] = z_ref

    def __call__(self, t, x):
        x_t = self.x_trim.copy()
        x_t[0:2] = x[0:2]       # 수평 위치는 동역학과 무관 → 오차 0 처리
        dx_r = ScheduledLQR._compute_error_state(x, x_t)
        u = self.u_trim - self.K_r @ dx_r
        return np.clip(u, self.p['n_min'], self.p['n_max'])


class GainsOnlyFixedLQR:
    """K_r만 30 m/s 고정, 트림 오프셋은 각 속도 정확값 (진단용).

    sweep_scheduled.py의 _FixedLQR와 동일한 구조 (legacy 무수정 재구현).
    게인 불일치의 영향만 분리해서 본다.
    """

    def __init__(self, params, K_r, x_trim, u_trim, z_ref):
        self.p = params
        self.K_r = K_r
        self.x_trim = x_trim.copy()
        self.u_trim = u_trim.copy()
        self.x_trim[2] = z_ref

    def __call__(self, t, x):
        x_t = self.x_trim.copy()
        x_t[0:2] = x[0:2]
        dx_r = ScheduledLQR._compute_error_state(x, x_t)
        u = self.u_trim - self.K_r @ dx_r
        return np.clip(u, self.p['n_min'], self.p['n_max'])


# ══════════════════════════════════════════════════
# 시뮬 + 측정 (legacy/sweep.py의 _simulate_and_measure와 동일 로직)
# ══════════════════════════════════════════════════

def simulate_and_measure(plant, x0, ctrl, V_ref, z_ref, T_sim):
    try:
        ts, xs, us = plant.simulate(x0, ctrl, T_sim)
    except Exception:
        return {'rmse_vx': np.inf, 'rmse_z': np.inf,
                'sat_pct': 100.0, 'diverged': True}

    z_vals = xs[:, 2]
    diverged = (np.any(np.isnan(xs)) or
                np.any(np.abs(z_vals - z_ref) > 200) or
                np.any(np.abs(xs[:, 5]) > 100))

    if diverged:
        # 발산 이후 구간은 RMSE를 오염시키므로 발산 시점까지만 사용
        bad = np.where(np.abs(z_vals - z_ref) > 200)[0]
        end = bad[0] if len(bad) > 0 else len(xs)
        end = max(end, 10)
    else:
        end = len(xs)

    n_all = us[:min(end, len(us)), :]
    sat_pct = 100.0 * np.mean(n_all >= 0.95 * plant.params['n_max']) \
        if len(n_all) > 0 else 0.0

    return {
        'rmse_vx': float(np.sqrt(np.mean((xs[:end, 3] - V_ref) ** 2))),
        'rmse_z':  float(np.sqrt(np.mean((xs[:end, 2] - z_ref) ** 2))),
        'sat_pct': float(sat_pct),
        'diverged': bool(diverged),
    }


# ══════════════════════════════════════════════════
# 스윕 실행
# ══════════════════════════════════════════════════

KEYS = ['pid_fixed', 'pid_sched', 'lqr_fixed', 'lqr_gains_only', 'lqr_sched']
LABELS = {
    'pid_fixed':      'PID fixed',
    'pid_sched':      'PID scheduled',
    'lqr_fixed':      'LQR fixed',
    'lqr_gains_only': 'LQR fixed (gains only)',
    'lqr_sched':      'LQR scheduled',
}


def run_sweep():
    plant = AxialDronePlant(P, dt=DT)

    # ── 30 m/s 고정 게인/트림 (LQR) ──
    trim_30 = find_trim(P, 30.0)
    x_t30, u_t30 = trim_30['state'].copy(), trim_30['control'].copy()
    lqr_30 = LQRController(P, x_t30, u_t30)
    assert lqr_30.valid, "30 m/s LQR 설계 실패"
    K_r_30 = lqr_30.K_r.copy()

    # ── 스케줄 LQR (1회 빌드) — 테이블에 85 추가 (모듈 docstring 참고) ──
    v_table = np.append(np.arange(0, 90, 10), 85.0)
    sched_lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=Z_REF, V_table=v_table)

    rows = []
    print(f"\n{'V':>4s}  " + "  ".join(f"{LABELS[k]:>22s}" for k in KEYS))
    print('─' * 130)

    for V in V_RANGE:
        trim_v = find_trim(P, V)
        if trim_v['residual'] > 1e-3:
            print(f"{V:4.0f}  트림 실패 → 건너뜀")
            continue

        x_t, u_t = trim_v['state'].copy(), trim_v['control'].copy()
        x0 = x_t.copy()
        x0[2] = Z_REF
        x0[3] += 1.0    # 속도 교란
        x0[5] += 2.0    # 수직 교란

        row = {'V': float(V)}

        pid = CascadedPID(P, v_ref=[V, 0, 0], z_ref=Z_REF, dt=DT)
        pid.reset()
        row['pid_fixed'] = simulate_and_measure(plant, x0.copy(), pid, V, Z_REF, T_SIM)

        spid = ScheduledPID(P, v_ref=[V, 0, 0], z_ref=Z_REF, dt=DT)
        spid.reset()
        row['pid_sched'] = simulate_and_measure(plant, x0.copy(), spid, V, Z_REF, T_SIM)

        sf = StrictFixedLQR(P, K_r_30, x_t30, u_t30, V, Z_REF)
        row['lqr_fixed'] = simulate_and_measure(plant, x0.copy(), sf, V, Z_REF, T_SIM)

        go = GainsOnlyFixedLQR(P, K_r_30, x_t, u_t, Z_REF)
        row['lqr_gains_only'] = simulate_and_measure(plant, x0.copy(), go, V, Z_REF, T_SIM)

        sched_lqr.v_ref = np.array([V, 0.0, 0.0])
        sched_lqr.z_ref = Z_REF
        row['lqr_sched'] = simulate_and_measure(plant, x0.copy(), sched_lqr, V, Z_REF, T_SIM)

        rows.append(row)
        print(f"{V:4.0f}  " + "  ".join(_cell(row[k]) for k in KEYS), flush=True)

    return rows


def _cell(r):
    if r['diverged']:
        return f"{'DIV':^22s}"
    return f"{'vx=' + format(r['rmse_vx'], '.2f'):>10s}/z={r['rmse_z']:<7.2f}"


# ══════════════════════════════════════════════════
# 무릎점/포화/발산 특정
# ══════════════════════════════════════════════════

def find_knee(rows, key, metric='rmse_z'):
    """30 m/s 대비 RMSE가 KNEE_FACTOR배를 넘거나 발산하는 첫 속도."""
    base = rows[0][key][metric]
    for r in rows:
        if r[key]['diverged'] or r[key][metric] > base * KNEE_FACTOR:
            return r['V']
    return None


def find_first(rows, key, pred):
    return next((r['V'] for r in rows if pred(r[key])), None)


# ══════════════════════════════════════════════════
# 플롯 공통 요소
# ══════════════════════════════════════════════════

METRIC_LABEL = {'rmse_z': 'RMSE z [m]', 'rmse_vx': 'RMSE v$_x$ [m/s]'}

# (색, 선스타일, 마커) — 고정=점선+빈 마커, 스케줄=실선+찬 마커
STYLE = {
    'pid_fixed':      (C_PID, '--', 'o'),
    'pid_sched':      (C_PID, '-',  'o'),
    'lqr_fixed':      (C_LQR, '--', 's'),
    'lqr_gains_only': (C_LQR, ':',  'D'),
    'lqr_sched':      (C_LQR, '-',  's'),
}


def _setup_axes(ax, metric):
    ax.set_xlabel('Cruise speed V [m/s]')
    ax.set_ylabel(METRIC_LABEL[metric])
    ax.grid(True, alpha=0.25, linewidth=0.5)
    ax.set_axisbelow(True)
    # 위쪽 보조 x축: km/h (같은 데이터의 단위 변환 — 300 km/h 목표 대비용)
    top = ax.secondary_xaxis('top', functions=(lambda v: v * 3.6,
                                               lambda k: k / 3.6))
    top.set_xlabel('[km/h]', fontsize=9)
    top.tick_params(labelsize=8)


def _plot_series(ax, rows, key, metric, solid_style=False):
    color, ls, marker = STYLE[key]
    if solid_style:              # 플롯 1(고정만 등장)에서는 실선이 더 깔끔
        ls = '-'
    V = np.array([r['V'] for r in rows])
    y = np.array([r[key][metric] for r in rows])
    div = np.array([r[key]['diverged'] for r in rows])

    ax.plot(V, y, color=color, ls=ls, marker=marker, ms=6, lw=2,
            label=LABELS[key],
            markerfacecolor=('white' if '--' in ls or ':' in ls else color),
            markeredgecolor=color)
    # 발산 지점: 검은 × 오버레이 (RMSE는 발산 직전까지 값)
    if div.any():
        ax.scatter(V[div], y[div], marker='x', s=90, color='black',
                   zorder=5, linewidths=2)
    return V, y


def _annotate_knee(ax, rows, key, metric, slot=0):
    """slot: 같은 축에 무릎점이 여러 개일 때 텍스트 높이를 계단식으로 낮춰
    겹침을 방지 (0=최상단, 1=한 칸 아래, ...)."""
    knee = find_knee(rows, key, metric)
    if knee is not None:
        color = STYLE[key][0]
        ax.axvline(knee, color=color, ls=':', lw=1.2, alpha=0.7)
        y0, y1 = ax.get_ylim()
        ax.text(knee + 0.5, y1 - (0.05 + 0.07 * slot) * (y1 - y0),
                f'knee {knee:.0f} m/s', color=color, fontsize=8, va='top')
    return knee


def _annotate_events(ax, rows, key, metric):
    """포화 시작(>5%)과 발산 시작 지점 텍스트 주석 (해당 시)."""
    color = STYLE[key][0]
    V = [r['V'] for r in rows]
    sat_V = find_first(rows, key, lambda r: r['sat_pct'] > SAT_THRESHOLD_PCT)
    if sat_V is not None:
        i = V.index(sat_V)
        ax.annotate(f'sat>{SAT_THRESHOLD_PCT:.0f}%',
                    (sat_V, rows[i][key][metric]),
                    textcoords='offset points', xytext=(6, -14),
                    fontsize=8, color=color)
    div_V = find_first(rows, key, lambda r: r['diverged'])
    if div_V is not None:
        i = V.index(div_V)
        ax.annotate('DIV', (div_V, rows[i][key][metric]),
                    textcoords='offset points', xytext=(6, 6),
                    fontsize=8, fontweight='bold', color='black')


# ══════════════════════════════════════════════════
# 플롯 1 — 성능 저하 곡선 (고정 게인)
# ══════════════════════════════════════════════════

KNEE_KEYS = ['pid_fixed', 'lqr_fixed']

def plot_knee(rows):
    """요청 사양: RMSE z 단일 축."""
    fig, ax = plt.subplots(figsize=(8, 5))
    for key in KNEE_KEYS:
        _plot_series(ax, rows, key, 'rmse_z', solid_style=True)
        _annotate_events(ax, rows, key, 'rmse_z')
    for i, key in enumerate(KNEE_KEYS):
        _annotate_knee(ax, rows, key, 'rmse_z', slot=i)
    _setup_axes(ax, 'rmse_z')
    ax.set_title('Fixed low-speed (30 m/s) gains vs cruise speed', fontsize=11)
    ax.legend(loc='upper left', fontsize=9, framealpha=0.9)
    fig.tight_layout()
    out = RESULTS_DIR / 'sweep_knee.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"저장: {out}")


def plot_knee_vxz(rows):
    """확장판: vx+z 2패널 + '게인만 고정' 진단 곡선.
    고정 게인 저하가 vx 축(트림 피드포워드 불일치)에 나타남을 보여준다."""
    keys = ['pid_fixed', 'lqr_fixed', 'lqr_gains_only']
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, metric in zip(axes, ['rmse_vx', 'rmse_z']):
        for key in keys:
            _plot_series(ax, rows, key, metric)
            _annotate_events(ax, rows, key, metric)
        for i, key in enumerate(keys):
            _annotate_knee(ax, rows, key, metric, slot=i)
        _setup_axes(ax, metric)
    axes[0].legend(loc='upper left', fontsize=8, framealpha=0.9)
    fig.suptitle('Fixed low-speed (30 m/s) tuning vs cruise speed — '
                 'degradation appears in velocity tracking', fontsize=11)
    fig.tight_layout()
    out = RESULTS_DIR / 'sweep_knee_vxz.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"저장: {out}")


# ══════════════════════════════════════════════════
# 플롯 2 — 고정 vs 스케줄링
# ══════════════════════════════════════════════════

CMP_KEYS = ['pid_fixed', 'pid_sched', 'lqr_fixed', 'lqr_sched']

def plot_fixed_vs_scheduled(rows):
    """요청 사양: RMSE z 단일 축."""
    fig, ax = plt.subplots(figsize=(8, 5))
    for key in CMP_KEYS:
        _plot_series(ax, rows, key, 'rmse_z')
    _setup_axes(ax, 'rmse_z')
    ax.set_title('Fixed gains vs gain scheduling — cruise disturbance recovery',
                 fontsize=11)
    ax.legend(loc='upper left', fontsize=9, framealpha=0.9)
    fig.tight_layout()
    out = RESULTS_DIR / 'sweep_fixed_vs_scheduled.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"저장: {out}")


def plot_fixed_vs_scheduled_vxz(rows):
    """확장판: vx+z 2패널. 고정↑ vs 스케줄 평평 대비는 vx 패널에서 보인다."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, metric in zip(axes, ['rmse_vx', 'rmse_z']):
        for key in CMP_KEYS:
            _plot_series(ax, rows, key, metric)
        _setup_axes(ax, metric)
    axes[0].legend(loc='upper left', fontsize=8, framealpha=0.9)
    fig.suptitle('Fixed gains vs gain scheduling — cruise disturbance recovery',
                 fontsize=11)
    fig.tight_layout()
    out = RESULTS_DIR / 'sweep_fixed_vs_scheduled_vxz.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"저장: {out}")


# ══════════════════════════════════════════════════
# txt 저장 (발표 표 작성용)
# ══════════════════════════════════════════════════

def _table(rows, keys):
    lines = []
    head = f"{'V[m/s]':>7s}"
    for k in keys:
        head += f" | {LABELS[k]:^28s}"
    lines.append(head)
    sub = f"{'':>7s}"
    for _ in keys:
        sub += f" | {'RMSE_vx':>8s} {'RMSE_z':>8s} {'SAT%':>5s} {'DIV':>3s}"
    lines.append(sub)
    lines.append('─' * len(sub))
    for r in rows:
        line = f"{r['V']:7.0f}"
        for k in keys:
            m = r[k]
            line += (f" | {m['rmse_vx']:8.2f} {m['rmse_z']:8.2f}"
                     f" {m['sat_pct']:5.1f} {'YES' if m['diverged'] else ' no':>3s}")
        lines.append(line)
    return lines


def _conditions():
    return [
        "조건: 각 속도 트림에서 시작, 교란 Δvx=+1, Δvz=+2 m/s, "
        f"T_sim={T_SIM:.0f}s, dt={DT*1000:.0f}ms, z_ref={Z_REF:.0f}m",
        "LQR fixed = 게인(K_r)+트림 피드포워드 모두 30 m/s 고정, 속도 목표만 갱신",
        "LQR fixed (gains only) = K_r만 고정, 트림은 각 속도 정확값 (진단용)",
        "LQR scheduled = V_table 0~80(+85) 선형 보간 "
        "(85 추가: 기본 테이블은 80에서 잘려 85 m/s에 인공 오차 발생)",
        "SAT% = 로터 명령이 0.95·n_max 이상인 시간 비율, "
        "DIV = |z-z_ref|>200m 또는 |vz|>100 또는 NaN",
    ]


def _knee_summary(rows, keys):
    lines = ["무릎점/포화/발산 요약 (무릎점 = 30 m/s 대비 RMSE 3배 또는 발산):"]
    for key in keys:
        lines.append(
            f"  {LABELS[key]:>22s}: "
            f"무릎점(z)={_fmt(find_knee(rows, key, 'rmse_z'))}, "
            f"무릎점(vx)={_fmt(find_knee(rows, key, 'rmse_vx'))}, "
            f"포화시작={_fmt(find_first(rows, key, lambda r: r['sat_pct'] > SAT_THRESHOLD_PCT))}, "
            f"발산={_fmt(find_first(rows, key, lambda r: r['diverged']))}")
    return lines


def write_knee_txt(rows):
    lines = ["성능 저하 곡선 — 저속(30 m/s) 튜닝 고정, 순항 속도 스윕 (시나리오 A)",
             "=" * 100]
    lines += _conditions()
    lines.append("")
    lines += _table(rows, ['pid_fixed', 'lqr_fixed', 'lqr_gains_only'])
    lines.append("")
    lines += _knee_summary(rows, ['pid_fixed', 'lqr_fixed', 'lqr_gains_only'])
    lines.append("")
    lines.append("해석 주의: 이 시나리오에서 LQR 고정의 저하는 RMSE_z가 아니라 RMSE_vx에 나타남")
    lines.append("  (트림 피드포워드 불일치 → 적분기 없는 LQR의 속도 정상상태 오차가 속도에 비례).")
    lines.append("  게인만 고정(gains only)은 저하가 거의 없음 → 게인 불일치 자체는 소교란 회복에 무해.")
    lines.append("  공력 감쇠가 속도에 비례해 커져서 z 회복은 고속에서 오히려 쉬워짐 (오픈루프 검증 완료).")
    out = RESULTS_DIR / 'sweep_knee.txt'
    out.write_text("\n".join(lines) + "\n", encoding='utf-8')
    print(f"저장: {out}")


def write_comparison_txt(rows):
    lines = ["고정 게인 vs 게인 스케줄링 — 시나리오 A (순항 교란 회복, NMPC 제외)",
             "=" * 130]
    lines += _conditions()
    lines.append("")
    lines += _table(rows, CMP_KEYS)
    lines.append("")
    lines += _knee_summary(rows, CMP_KEYS)
    lines.append("")
    lines.append("고속(85 m/s) 고정/스케줄 비율:")
    r85 = next((r for r in rows if r['V'] == 85), None)
    if r85 is not None:
        for fixed, sched, name in [('pid_fixed', 'pid_sched', 'PID'),
                                   ('lqr_fixed', 'lqr_sched', 'LQR')]:
            f, s = r85[fixed], r85[sched]
            if f['diverged'] or s['diverged']:
                lines.append(f"  {name}: 고정 발산={f['diverged']}, "
                             f"스케줄 발산={s['diverged']}")
            else:
                lines.append(
                    f"  {name}: RMSE_vx {f['rmse_vx']:.2f}/{s['rmse_vx']:.2f}"
                    f" = {f['rmse_vx']/s['rmse_vx']:.1f}배,  "
                    f"RMSE_z {f['rmse_z']:.2f}/{s['rmse_z']:.2f}"
                    f" = {f['rmse_z']/s['rmse_z']:.1f}배")
    out = RESULTS_DIR / 'sweep_fixed_vs_scheduled.txt'
    out.write_text("\n".join(lines) + "\n", encoding='utf-8')
    print(f"저장: {out}")


def _fmt(v):
    return f"{v:.0f} m/s" if v is not None else "없음"


# ══════════════════════════════════════════════════

if __name__ == '__main__':
    print("시나리오 A 속도 스윕 — 고정 vs 스케줄링 (NMPC 제외)")
    rows = run_sweep()
    write_knee_txt(rows)
    write_comparison_txt(rows)
    plot_knee(rows)
    plot_knee_vxz(rows)
    plot_fixed_vs_scheduled(rows)
    plot_fixed_vs_scheduled_vxz(rows)
    print("\n완료.")
