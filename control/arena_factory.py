"""경기장 제어기 팩토리 — 민석 스위트에 우리 **현재** 제어기 5종을 등록한다.

`control/validation_suite.py::run_trial`은 원래 팀원의 옛 `Factory`
(models/team_light/control, 2026-09-09 스냅샷 제어기)만 알았다. 이 파일의
`ArenaFactory`가 같은 자리에 들어가 V13·M17·F13·GSLQR·CPID를 만든다.
옛 제어기는 결과에 섞지 않는다(작업지시서 작업 B).

공정성 장치는 전부 여기 한 곳에 모았다 — 제어기마다 따로 두면 하나만
어긋나도 알아채기 어렵기 때문이다.

  ReferenceWindow   모든 제어기가 **같은 객체**로 참조를 받는다. 미리보기
                    길이 H를 넘는 요청은 예외(PreviewViolation). (불변식 I-2)
  SolverMonitor     NMPC 세 종의 수렴 추적(consec_fail·solve_log)을 **한
                    구현**으로 똑같이 건다. M17은 원래 이 추적이 없었다. (I-3)
  trim_warm_start   세 NMPC의 첫 웜스타트를 같은 규칙으로 채운다:
                    X = 실측 초기상태, U = 명목 제어기 모델의 트림 입력.
                    같은 트림 정보를 CPID는 적분기 출발값으로 받는다(적분기로만
                    트림을 유지하는 구조라, 0에서 출발하면 CPID만 트림 정보 없이
                    뛰게 된다). GSLQR은 트림 피드포워드가 그 역할을 한다.
  IntegratorLog     GSLQR·CPID 적분기의 한계 도달·적분 정지 스텝을 같은 형식으로 센다.
  ControllerModel   명목 제어기 모델(집중정수 적합) 하나를 5종이 공유한다.
                    제어기 파라미터가 기체의 호버 자세 규약('hover_quat')을 들고
                    있어, 트림이 처음부터 플랜트 규약으로 나온다 — 우리 기본값과
                    추력축 둘레 180° 달라서, 맞추지 않으면 GSLQR·CPID가 가짜 180°
                    롤 오차를 본다. (NMPC 계열은 쿼터니언을 비용에 안 써서 무관)
"""
from copy import deepcopy
import json
from pathlib import Path
import weakref

import numpy as np
from scipy.spatial.transform import Rotation

from control.arena import ROOT

ARENA_LABELS = ('V13', 'M17', 'F13', 'GSLQR', 'CPID')
NMPC_LABELS = ('V13', 'M17', 'F13')


class PreviewViolation(AssertionError):
    """제어기가 허용된 미리보기 길이 H보다 먼 미래 참조를 요청했다."""


class ReferenceWindow:
    """절대시각 참조 r(τ) = (vx, vy, vz, z)를 τ ∈ [t_now, t_now + H]에서만 준다.

    NMPC는 예측 노드마다(τ = t + k·Δt, k=0..N) 부르고, GSLQR·CPID는 현재
    시각(τ = t)만 부른다. 같은 창을 받았는데 쓰는 폭이 다른 것은 **구조**의
    차이이고, 창 자체가 다른 것은 **정보**의 차이다 — 후자는 막는다.
    """

    def __init__(self, profile, horizon_s):
        self._profile = profile
        self.horizon_s = float(horizon_s)
        self.t_now = 0.0
        self.max_lookahead = 0.0      # 실제로 가장 멀리 본 시간(검사용)

    def set_time(self, t):
        self.t_now = float(t)

    def __call__(self, tau):
        ahead = float(tau) - self.t_now
        if ahead > self.horizon_s + 1e-9:
            raise PreviewViolation(f'requested {ahead:.6f} s ahead > H={self.horizon_s} s')
        self.max_lookahead = max(self.max_lookahead, ahead)
        v, z, _ = self._profile.get_ref(float(tau))
        return np.array([v[0], v[1], v[2], z], dtype=float)


class SolverMonitor:
    """NMPC `_solve`를 감싸 수렴 이력을 남긴다 — 세 종에 같은 규칙.

    consec_fail 규칙은 V13·F13 클래스 안의 규칙을 그대로 옮겼다: 한 번이라도
    수렴한 뒤의 연속 미수렴만 센다(첫 수렴 전 실패는 초기화 문제라 따로 본다).
    스위트는 consec_fail ≥ 5에서 시행을 멈춘다. solve_log에는 벽시계 시간을
    넣지 않는다 — 결과 해시가 컴퓨터 속도에 따라 달라지면 안 되기 때문이다.
    """

    ACCEPTED = ('Solve_Succeeded', 'Solved_To_Acceptable_Level')

    def __init__(self, nmpc):
        # NMPC는 **약한 참조**로 잡는다. 감싼 _solve를 NMPC 인스턴스에 달기 때문에, 강한 참조로
        # 잡으면 NMPC → 감싼 함수 → 모니터 → NMPC 순환이 생긴다. 그러면 NLP(M17은 1 GB 넘음)가
        # 참조 카운트로 풀리지 않고 순환 GC를 기다리며 쌓인다. 실측: 튜닝 프로세스가 평가 1~2회 만에
        # 6~10 GB까지 불었다(2026-09-26). 계산 경로는 그대로라 결과는 비트 단위로 같다.
        self._nmpc = weakref.ref(nmpc)
        self.solve_log = []
        self.consec_fail = 0
        self._ever_converged = False
        solve = type(nmpc)._solve          # 묶인 메서드(= NMPC 강한 참조)가 아니라 함수로 잡는다
        target = self._nmpc

        def monitored(x):
            nmpc = target()
            u = solve(nmpc, x)
            stats = nmpc.solver.stats()
            status = stats.get('return_status', 'unknown')
            accepted = status in self.ACCEPTED
            if accepted:
                self.consec_fail = 0
                self._ever_converged = True
            elif self._ever_converged:
                self.consec_fail += 1
            self.solve_log.append(dict(t=float(nmpc._t_now), status=status, accepted=accepted,
                                       iter_count=stats.get('iter_count'),
                                       finite=bool(np.all(np.isfinite(u)))))
            return u

        nmpc._solve = monitored

    @property
    def nmpc(self):
        return self._nmpc()


def trim_warm_start(nmpc, x_meas, u_trim):
    """논문모드 결정변수 배치 [X_0, U_0, X_1, U_1, …, U_{N-1}, X_N]에 웜스타트.

    세 NMPC(V13·M17·F13)의 논문모드 배치가 모두 이 모양이라(nx·nu만 다름)
    함수 하나로 똑같이 채울 수 있다. 첫 솔브 전에 한 번만 부르고, 클래스의
    `_cold_start` 보정(실측으로 X만 덮어쓰기)은 끈다 — 두 규칙이 겹치지 않게.
    """
    x_meas = np.asarray(x_meas, dtype=float)
    u_trim = np.asarray(u_trim, dtype=float)
    nx, nu, N = x_meas.size, u_trim.size, nmpc.N
    if nmpc.w0.size != (N + 1)*nx + N*nu:
        raise ValueError(f'unexpected decision layout: {nmpc.w0.size} != (N+1)*{nx}+N*{nu}')
    stride = nx + nu
    for k in range(N + 1):
        nmpc.w0[k*stride:k*stride + nx] = x_meas
    for k in range(N):
        nmpc.w0[k*stride + nx:k*stride + nx + nu] = u_trim
    nmpc._cold_start = False


class IntegratorLog:
    """적분기 '한계 도달'·'적분 정지' 스텝을 센다. GSLQR과 CPID에 같은 형식을 쓴다
    (kj 결정 2026-09-26).

    한 스텝에 어느 채널이든 한계에 닿아 있으면 한계 도달 스텝으로 세고, 조건부
    적분이 멈췄으면 적분 정지 스텝으로 센다. 한계 도달이 시행 시간의 1%를 넘는
    제어기가 튜닝이나 스모크에 하나라도 있으면 보고서 '결정 필요'에 올린다
    (설정 reporting.integrator_at_limit_fraction). 제어기 상태를 읽기만 하므로
    결과에 영향이 없다.
    """
    TOL = 1e-9

    def __init__(self):
        self.steps = 0
        self.at_limit_steps = 0
        self.frozen_steps = 0
        self.channels = {}

    def observe(self, status):
        self.steps += 1
        any_limit = False
        for name, (value, limit) in status['channels'].items():
            ch = self.channels.setdefault(name, dict(limit=limit, at_limit_steps=0,
                                                     peak_fraction=0.0))
            fraction = value/limit if limit > 0 else 0.0
            ch['peak_fraction'] = max(ch['peak_fraction'], fraction)
            if fraction >= 1.0 - self.TOL:
                ch['at_limit_steps'] += 1
                any_limit = True
        self.at_limit_steps += any_limit
        self.frozen_steps += bool(status['frozen'])

    def report(self):
        n = max(self.steps, 1)
        return dict(steps=self.steps, at_limit_steps=self.at_limit_steps,
                    at_limit_fraction=self.at_limit_steps/n, frozen_steps=self.frozen_steps,
                    frozen_fraction=self.frozen_steps/n, channels=deepcopy(self.channels))


class ArenaController:
    """스위트가 부르는 겉포장. 5종 모두 같은 모양으로 불린다: ctrl(t, x17)."""

    OBSERVATION_KEYS = ('t', 'x17', 'reference_window')

    def __init__(self, label, inner, window, nmpc=None, warm_start=None, settings=None,
                 feedforward_step=None):
        self.label = label
        self.inner = inner
        self.window = window
        self.nmpc = nmpc
        self.monitor = SolverMonitor(nmpc) if nmpc is not None else None
        self.settings = settings or {}
        self._warm_start = warm_start
        self._started = False
        # 참조 가속도 피드포워드를 쓰는 제어기(GSLQR·CPID)만 값이 있다: 창에서 몇 초 앞을 읽어 차분하는가
        self.feedforward_step = feedforward_step
        # 적분기가 있는 제어기(GSLQR·CPID)만 계측한다. NMPC 계열은 적분기가 없다.
        self.integrators = IntegratorLog() if hasattr(inner, 'integrator_status') else None

    def observation_spec(self):
        return dict(keys=self.OBSERVATION_KEYS, preview_horizon_s=self.window.horizon_s,
                    window_type=type(self.window).__name__)

    def __call__(self, t, x):
        if not self._started:
            if self._warm_start is not None:
                self._warm_start(np.asarray(x, dtype=float))
            self._started = True
        u = self.inner(t, x)
        if self.integrators is not None:
            self.integrators.observe(self.inner.integrator_status())
        return u

    def integrator_report(self):
        """적분기 계측 요약(없으면 None) — 스위트가 시행 행에 'integrators'로 넣는다."""
        return None if self.integrators is None else self.integrators.report()


# ══════════════════════════════════════════════════════════════════
# 명목 제어기 모델과 자세 규약
# ══════════════════════════════════════════════════════════════════

def convention_map(cp, plant_hover_quat):
    """(교차검증용) 우리 기본 규약 트림 → 플랜트 규약. (동체 상대회전 R_rel, 로터 치환 perm)

    경기장은 이제 제어기 파라미터의 'hover_quat'으로 트림을 **직접** 플랜트 규약에서
    구한다(control/trim.py). 이 함수는 그 결과를 독립적으로 확인하는 데만 쓴다 —
    cp는 'hover_quat'이 **없는** 기본 규약 파라미터여야 한다.

    두 호버 자세는 모두 추력축(동체 +x)을 세계 +z로 향하지만, 동체 y·z축이
    가리키는 방향이 반대다(실측: 모든 속도에서 추력축 둘레 정확히 180°).
    물리적으로 같은 트림을 플랜트 규약으로 옮기려면 자세에 R_rel을 곱하고,
    그 회전으로 **자리가 바뀐 로터끼리** 회전수를 맞바꿔야 한다:
      플랜트 규약의 로터 i 위치 = R_A·R_rel·r_i = R_A·r_j  →  n_B[i] = n_A[j]
    회전방향(반토크 부호)까지 같은 로터끼리만 대응돼야 물리가 같다 — 아니면 예외.
    """
    from control.dynamics import AxialDronePlant
    R_ours = Rotation.from_quat(AxialDronePlant.hover_state(cp)[6:10])
    R_rel = R_ours.inv() * Rotation.from_quat(plant_hover_quat)
    M = R_rel.as_matrix()
    pos = np.asarray(cp['rotor_positions'], dtype=float)
    dirs = np.asarray(cp['rotor_directions'])
    perm = []
    for i in range(len(pos)):
        target = M @ pos[i]
        j = int(np.argmin(np.linalg.norm(pos - target, axis=1)))
        if np.linalg.norm(pos[j] - target) > 1e-9 or dirs[j] != dirs[i]:
            raise ValueError('hover conventions are not related by a rotor-preserving symmetry')
        perm.append(j)
    return R_rel, np.array(perm)


class ControllerModel:
    """5종이 공유하는 명목 제어기 모델(CP)과 트림표(플랜트 규약).

    CP = control.kh_adapter.build_controller_params(팀 native): 기하·질량·
    추진은 팀 값, 공력만 집중정수로 적합. 여기에 새 정보를 더하지 않는다 —
    플랜트의 실제 섭동값이나 난수는 들어오지 않는다(논문 §3.5).
    """

    def __init__(self, native):
        from control.kh_adapter import build_controller_params, fit_cp_schedule
        from models.team_light.control.baseline_v2 import parameter_hash
        from models.team_light.control.geometry import hover_quaternion
        # 얕은 복사면 제어기가 중첩 배열을 고칠 때 플랜트 원본까지 바뀐다 —
        # 스위트의 '명목 파라미터 변조' 검사가 잡겠지만 애초에 막는다.
        self.cp = build_controller_params(deepcopy(native))
        self.sha256 = parameter_hash(self.cp)
        self.plant_hover_quat = np.asarray(hover_quaternion(), dtype=float)
        # 제어기 모델의 호버 규약이 플랜트와 같아야 GSLQR·CPID의 자세 기준이 맞는다.
        if not np.allclose(self.cp['hover_quat'], self.plant_hover_quat, atol=1e-12):
            raise ValueError('controller-model hover convention differs from the plant')
        aero = {k: self.cp[k] for k in ('C_Na', 'C_dc', 'C_A0', 'C_Aa2', 'x_cp', 'C_lp', 'C_mq')}
        _, self.cp_schedule_report = fit_cp_schedule(native, aero)
        self._trims = {}          # 1 m/s 격자 → 명목 모델 트림(플랜트 규약, 연속법)

    def trim(self, V):
        """명목 제어기 모델의 트림(플랜트 호버 규약). 0에서 1 m/s 연속법(캐시).

        냉시동 fsolve는 이 기체에서 25 m/s 위로 못 간다(실측) — 연속법으로는 0~85가
        전부 풀린다. 반환: find_trim dict + speed.
        """
        from control.trim import find_trim
        key = round(float(V), 6)
        if key not in self._trims:
            grid = [v for v in np.arange(0.0, key, 1.0)] + [key]
            guess = None
            for v in grid:
                k = round(float(v), 6)
                if k not in self._trims:
                    tr = find_trim(self.cp, float(v), guess=guess, quiet=True)
                    if not tr['converged']:
                        raise ValueError(f'controller-model trim continuation broke at {v} m/s: '
                                         f'{tr["why"]}')
                    self._trims[k] = dict(tr, speed=float(v))
                guess = self._trims[k]['guess']
        return self._trims[key]

    def trim_acceleration(self, V):
        """명목 모델 트림에서 로터 추력 벡터가 내는 가속도 [m/s², 세계 좌표]. 중력은 뺀다.

        CPID의 외부 루프는 F_des = m(a_des + g·e_z)를 추력 벡터로 삼는다. 그래서
        트림을 유지하려면 a_des가 a = F_rotor/m − g·e_z여야 한다. 공력이 받치는 몫
        (수평은 항력, 수직은 양력)이 여기에 들어 있다. CPID 적분기의 출발값으로 쓴다
        (NMPC 웜스타트가 쓰는 것과 같은 명목 트림).
        """
        tr = self.trim(V)
        x, u = tr['state'], tr['control']
        thrust_dir = (np.array([1.0, 0.0, 0.0]) if self.cp.get('thrust_axis', 'z') == 'x'
                      else np.array([0.0, 0.0, -1.0]))          # 동체 좌표 추력 방향
        R = Rotation.from_quat(x[6:10]).as_matrix()
        F = R @ thrust_dir * float(np.sum(self.rotor_thrusts(u, x)))
        return F/self.cp['mass'] - np.array([0.0, 0.0, self.cp['g']])

    def rotor_thrusts(self, n, x):
        """명목 모델의 로터별 추력 [N] — control.dynamics와 같은 분기를 따른다.

        control/dynamics.py가 팀 곡선을 쓰는 조건(propulsion_model 일치)을
        그대로 따라 한다. 그래야 웜스타트의 트림 입력이 NMPC 예측모델·INDI와
        같은 추진 모델에서 나온다(kj 조건: 전부 같은 모델).
        """
        import control.dynamics as dyn
        n = np.asarray(n, dtype=float)
        R = Rotation.from_quat(x[6:10]).as_matrix()
        v_axial = max(float((R.T @ x[3:6])[0]), 0.0)
        curve = getattr(dyn, 'PROP_CURVE_MODEL', None)
        if curve is not None and self.cp.get('propulsion_model') == curve:
            from models.team_light.control.propeller_curve import values_and_derivatives
            return np.asarray(values_and_derivatives(self.cp, n, v_axial)[0], dtype=float)
        n_rps = n/(2*np.pi)
        J = v_axial/(n_rps*self.cp['D_prop'] + dyn.EPS)
        return self.cp['k_T']*n**2*np.maximum(1.0 - J/self.cp['J_max'], 0.0)


def cpid_heading_for_plant(cp, hover_quat=None):
    """CPID의 목표 방위 heading을 기체의 호버 규약(cp['hover_quat'])에 맞춘다.

    x축 기체에서 CascadedPID는 동체 y축을 b2 = normalize(b1 × c1),
    c1 = [cos h, sin h, 0]으로 만든다. 호버(b1 = 세계 +z)에서 b2 = [-sin h, cos h, 0]
    이므로, 호버의 동체 y축(세계 좌표)과 같아지는 h를 고른다. 팀 규약에서는
    동체 y가 세계 -y라 h = π다 — 기본값 0을 그대로 쓰면 첫 스텝부터 180° 자세
    오차의 특이점(오차 벡터가 0이 되는 불안정 평형) 위에서 출발한다.
    hover_quat을 주면 그것을, 아니면 cp의 규약(없으면 기본 호버 자세)을 쓴다.
    """
    from control.dynamics import AxialDronePlant
    q = hover_quat if hover_quat is not None else AxialDronePlant.hover_state(cp)[6:10]
    y_world = Rotation.from_quat(np.asarray(q, dtype=float)).as_matrix()[:, 1]
    return float(np.arctan2(-y_world[0], y_world[1]))


# ══════════════════════════════════════════════════════════════════
# 팩토리
# ══════════════════════════════════════════════════════════════════

def _load_gains(path):
    data = json.loads((ROOT/path).read_text(encoding='utf-8'))
    return data


class ArenaFactory:
    """`validation_suite.run_trial`이 부르는 팩토리 규약을 구현한다.

    factory.p        플랜트 명목 파라미터(팀 native). 스위트가 이것만 섭동해
                     플랜트를 만든다 — 제어기는 절대 이 dict를 받지 않는다.
    make_for_profile 시행마다 새 제어기 상태(NLP도 새로 짓는다: 같은 설정 두
                     번 돌리면 비트 단위로 같아야 하므로 캐시된 솔버 상태를
                     나눠 쓰지 않는다 — I-6).
    update_at        매 스텝 참조 창의 시각과 현재 참조를 모든 제어기에 같게 넣는다.
    solver_of        NMPC는 SolverMonitor, 나머지는 None.
    """

    def __init__(self, config, native=None, overrides=None, model=None):
        """overrides: {label: {...}} 튜닝 후보 — NMPC는 cost_weights·Q_z, GSLQR·CPID는
        게인 파일의 키를 덮어쓴다. model: 이미 만든 ControllerModel(튜닝이 평가마다
        제어기 모델을 다시 적합하지 않게). 둘 다 없으면 설정 파일 그대로다."""
        from models.team_light.control.baseline_v2 import baseline_params
        self.config = config
        self.p = native if native is not None else baseline_params()
        self.model = model if model is not None else ControllerModel(self.p)
        self.cp = self.model.cp
        self.controller_model_sha256 = self.model.sha256
        self.dt = float(config['plant']['dt_s'])
        self.horizon = float(config['preview_horizon_s'])
        self.overrides = overrides or {}
        self.gains = {label: _load_gains(spec['gains'])
                      for label, spec in config['controllers'].items() if 'gains' in spec}
        for label, extra in self.overrides.items():
            if label in self.gains:
                self.gains[label] = {**self.gains[label], **extra}
        self.settings = {}
        self._gslqr_proto = None

    # ── 스위트 규약 ────────────────────────────────────────────
    def make(self, label, speed, z):
        raise NotImplementedError('arena controllers need the full profile: use make_for_profile')

    def make_for_profile(self, label, profile, case=None):
        if label not in self.config['controllers']:
            raise ValueError(f'controller {label!r} is not in the arena config')
        window = ReferenceWindow(profile, self.horizon)
        v0, z0, _ = profile.get_ref(0.0)
        builder = getattr(self, f'_build_{label.lower()}')
        ctrl = builder(window, np.asarray(v0, dtype=float), float(z0))
        self.settings[label] = ctrl.settings
        return ctrl

    def update_at(self, ctrl, t, v, z):
        ctrl.window.set_time(t)
        r = ctrl.window(t)           # 모든 제어기가 창을 통해 현재 참조를 읽는다
        target = ctrl.nmpc if ctrl.nmpc is not None else ctrl.inner
        target.v_ref = r[0:3].copy()
        target.z_ref = float(r[3])
        step = getattr(ctrl, 'feedforward_step', None)     # 대체 빌더(테스트)에는 없을 수 있다
        if step:
            # 참조 가속도 피드포워드(GSLQR·CPID, kj 결정 2026-09-26 오후). **같은 창**에서 NMPC 예측
            # 격자의 첫 간격(Δt_pred) 앞 참조를 읽어 차분한다 — NMPC가 첫 간격에서 보는 가속도와 같은
            # 정보이고, 창이 H를 넘는 요청은 막는다. 속도 성분만 차분한다(고도 z를 차분하면 고도
            # 계단이 가속도 펄스가 된다). 참조가 일정하면 두 값이 같아 정확히 0이다.
            ahead = ctrl.window(t + step)
            target.a_ref = (ahead[0:3] - r[0:3])/step

    def update(self, ctrl, v, z):
        raise NotImplementedError('arena controllers need the time: use update_at')

    def solver_of(self, ctrl):
        return ctrl.monitor

    # ── 제어기별 생성 ──────────────────────────────────────────
    def _nmpc_kwargs(self, label, window):
        """세 NMPC에 **같은 dict**로 넘기는 설정. 가중치 덮어쓰기(튜닝)는
        overrides[label]['cost_weights']로만 들어온다 — 한 NMPC만 다른 값을
        받는 것은 튜닝 결과를 쓸 때뿐이고, 그때도 manifest에 그대로 남는다."""
        nm = self.config['nmpc_common']
        if nm['soft_constraints']:
            raise ValueError('soft constraints are not implemented in the paper-mode NLPs')
        override = self.overrides.get(label, {})
        return dict(N=int(nm['N']), dt_nmpc=float(nm['dt_pred_s']), dt_ctrl=float(nm['dt_ctrl_s']),
                    max_iter=int(nm['max_iter']), tol=float(nm['tol']),
                    cost_spec=nm['cost_spec'], ref_fn=window,
                    Q_z=float(override.get('Q_z', nm['Q_z'])),
                    cost_weights=override.get('cost_weights'))

    def _nmpc_settings(self, label, nmpc, **extra):
        # 설정 파일 값이 아니라 **만들어진 객체에 실제로 들어간 값**을 읽는다 —
        # 설정과 실제가 어긋나면 I-3가 그 차이를 본다.
        return dict(kind=label, warm_start=self.config['nmpc_common']['warm_start'],
                    **nmpc.solver_settings(), **extra)

    def _trim_input(self, label, v0):
        tr = self.model.trim(float(v0[0]))
        f = self.model.rotor_thrusts(tr['control'], tr['state'])
        if label == 'V13':
            return np.array([float(np.sum(f)), 0.0, 0.0, 0.0])
        if label == 'F13':
            return f
        return tr['control']

    def _build_v13(self, window, v0, z0):
        from control.hybrid_comparison import VirtualNMPC, ProperHybrid
        spec = self.config['controllers']['V13']
        nmpc = VirtualNMPC(self.cp, v_ref=v0, z_ref=z0,
                           alloc_feedback=bool(spec['alloc_feedback']),
                           **self._nmpc_kwargs('V13', window))
        inner = ProperHybrid(nmpc, self.cp, dt=self.dt, alloc_mode=spec['alloc_mode'],
                             time_align=spec['time_align'])
        u_trim = self._trim_input('V13', v0)
        settings = self._nmpc_settings('V13', nmpc, alloc_mode=inner.alloc_mode,
                                       time_align=inner.time_align,
                                       alloc_feedback=nmpc.alloc_feedback)
        return ArenaController('V13', inner, window, nmpc=nmpc, settings=settings,
                               warm_start=lambda x: trim_warm_start(nmpc, x[0:13], u_trim))

    def _build_f13(self, window, v0, z0):
        from control.nmpc_f13 import build_f13_controller
        spec = self.config['controllers']['F13']
        inner = build_f13_controller(self.cp, v_ref=v0, z_ref=z0, dt=self.dt,
                                     **self._nmpc_kwargs('F13', window))
        if inner.alloc_mode != spec['alloc_mode'] or inner.time_align != spec['time_align']:
            raise ValueError('F13 INDI settings differ from the arena config')
        nmpc = inner.nmpc.inner
        u_trim = self._trim_input('F13', v0)
        settings = self._nmpc_settings('F13', nmpc, alloc_mode=inner.alloc_mode,
                                       time_align=inner.time_align, alloc_feedback=False)
        return ArenaController('F13', inner, window, nmpc=nmpc, settings=settings,
                               warm_start=lambda x: trim_warm_start(nmpc, x[0:13], u_trim))

    def _build_m17(self, window, v0, z0):
        from control.nmpc import NMPCController
        # 양추력 하한(kj 결정 2026-09-26 오후)은 M17에만 준다. 세 NMPC 공통 dict(_nmpc_kwargs)에
        # 넣지 않는 이유: V13·F13은 NLP 입력이 추력이라 '추력 ≥ 0'이 이미 상자 제약이다. 이것은 같은
        # 제약을 회전수 좌표로 옮긴 것이지 M17에만 주는 새 이점이 아니다.
        nmpc = NMPCController(self.cp, v_ref=v0, z_ref=z0, **self._nmpc_kwargs('M17', window),
                              rotor_floor=self.config['controllers']['M17'].get('rotor_floor'))
        u_trim = self._trim_input('M17', v0)
        settings = self._nmpc_settings('M17', nmpc, rotor_floor=nmpc.rotor_floor)
        return ArenaController('M17', nmpc, window, nmpc=nmpc, settings=settings,
                               warm_start=lambda x: trim_warm_start(nmpc, x, u_trim))

    def _gslqr_prototype(self):
        """GSLQR는 설계가 결정적이라 한 번 짓고 시행마다 깊은 복사한다."""
        if self._gslqr_proto is None:
            from control.controller import ScheduledLQR
            spec = self.config['controllers']['GSLQR']
            g = self.gains['GSLQR']
            V_table = [float(v) for v in spec['V_table_m_s']]
            trims = [self.model.trim(v) for v in V_table]
            self._gslqr_proto = ScheduledLQR(
                self.cp, v_ref=[0.0, 0.0, 0.0], z_ref=0.0, V_table=V_table,
                Q=np.diag(np.asarray(g['Q_diag'], dtype=float)),
                R=np.eye(4)*float(g['R_scale']),
                integral_states=tuple(g['integral_states']),
                Q_integral=g.get('Q_integral'), dt=self.dt,
                integral_limit=float(g['integral_limit']), trims=trims,
                acceleration_feedforward=self._feedforward_step('GSLQR') is not None)
        return self._gslqr_proto

    def _build_gslqr(self, window, v0, z0):
        ctrl = deepcopy(self._gslqr_prototype())
        ctrl.reset()
        ctrl.v_ref = v0.copy()
        ctrl.z_ref = z0
        g = self.gains['GSLQR']
        step = self._feedforward_step('GSLQR')
        settings = dict(kind='GSLQR', V_table_m_s=ctrl.V_table.tolist(),
                        dropped=list(ctrl.dropped), Q_diag=list(g['Q_diag']),
                        R_scale=g['R_scale'], integral_states=list(g['integral_states']),
                        Q_integral=g.get('Q_integral'), integral_limit=g['integral_limit'],
                        trims='controller model, plant hover convention',
                        reference_feedforward=self.config['controllers']['GSLQR'].get('reference_feedforward'),
                        lookahead_s=step or 0.0, feedforward_limits=ctrl.feedforward_limits())
        return ArenaController('GSLQR', ctrl, window, settings=settings, feedforward_step=step)

    def _build_cpid(self, window, v0, z0):
        from control.controller import CascadedPID
        g = self.gains['CPID']
        spec = self.config['controllers']['CPID']
        heading = cpid_heading_for_plant(self.cp)      # 제어기 모델의 호버 규약(= 플랜트)
        ctrl = CascadedPID(self.cp, v_ref=v0, z_ref=z0, heading=heading, dt=self.dt)
        for key in ('Kp_vel', 'Ki_vel', 'a_int_max', 'Kp_z', 'Kd_z', 'Ki_z', 'int_z_max'):
            setattr(ctrl, key, float(g[key]))
        # 고도 적분 경기장 방식(가속도 권한 + 조건부). 게인 파일에 없으면 기존 방식.
        ctrl.a_int_z_max = None if g.get('a_int_z_max') is None else float(g['a_int_z_max'])
        ctrl.Kp_att = np.asarray(g['Kp_att'], dtype=float)
        ctrl.Kd_att = np.asarray(g['Kd_att'], dtype=float)
        ctrl.max_tilt = np.radians(float(g['max_tilt_deg']))
        ctrl.reset()
        # 출발 규칙: NMPC 웜스타트와 같은 정보(시작 참조속도의 명목 모델 트림)로 적분기를
        # 채운다. 설정이 켤 때만(kj 결정 2026-09-26: 경기장 설정에서만 켠다).
        preload = spec.get('integrator_preload')
        if preload not in (None, 'controller_model_trim'):
            raise ValueError(f'unknown CPID integrator_preload {preload!r}')
        a_ff = self.model.trim_acceleration(float(v0[0])) if preload else None
        step = self._feedforward_step('CPID')
        ctrl.acc_feedforward = step is not None
        settings = dict(kind='CPID', heading_rad=heading,
                        **{k: g[k] for k in ('Kp_vel', 'Ki_vel', 'a_int_max', 'Kp_z', 'Kd_z', 'Ki_z', 'int_z_max',
                                              'Kp_att', 'Kd_att', 'max_tilt_deg')},
                        a_int_z_max=ctrl.a_int_z_max, integrator_preload=preload,
                        preload_acceleration=None if a_ff is None else a_ff.tolist(),
                        reference_feedforward=spec.get('reference_feedforward'),
                        lookahead_s=step or 0.0)
        return ArenaController('CPID', ctrl, window, settings=settings, feedforward_step=step,
                               warm_start=None if a_ff is None
                               else (lambda x: ctrl.preload_integrators(a_ff)))

    def _feedforward_step(self, label):
        """참조 가속도 피드포워드를 켜면 창에서 몇 초 앞을 읽어 차분하는가 — NMPC 예측 격자의 첫 간격
        (nmpc_common.dt_pred_s)이다. 끄면 None(창에서 현재 참조만 읽는다)."""
        mode = self.config['controllers'][label].get('reference_feedforward')
        if mode not in (None, 'window_acceleration'):
            raise ValueError(f'unknown {label} reference_feedforward {mode!r}')
        return None if mode is None else float(self.config['nmpc_common']['dt_pred_s'])
