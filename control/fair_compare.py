"""공정 비교 하네스 — 논문 §5.3·§5.5의 공정성 계약을 코드로 강제한다.

논문은 공정성 규칙을 이미 적어 두었다.

  §5.3 "원인 분석 모드에서는 플랜트, 상태 정보, 참조, 예측 구간, 명목 공력,
        입력 갱신률과 솔버 설정을 일치시키고 한 요소씩 바꾼다."
  §5.5 "미래 참조를 사용하는 모든 제어기에는 같은 미리보기 정보를 제공한다."

문제는 그게 **산문일 뿐 코드가 강제하지 않는다**는 것이었다. 실제로 이 파일을
쓰게 된 계기가, 임시 스크립트에서 참조를 `ProperHybrid`에 넣었는데 정작 참조를
읽는 객체는 내부 `VirtualNMPC`였던 사고다. LQR 만 램프를 받고 V13 은 호버
명령만 받은 채 "V13 이 70 m/s 를 못 따라간다"는 숫자가 나왔다. 조용히 기울어진
운동장이었고, 그때는 하이브리드 쪽이 불리했다.

그래서 이 모듈의 원칙은 하나다 — **틀리면 조용히 나쁜 숫자가 나오는 대신
즉시 예외를 던진다.**

기울어짐은 양방향 모두 실격이다. 하이브리드에 유리한 설정도 똑같이 막는다.

사용:

    spec = ComparisonSpec(params, reference=ramp(0., 70., 1., 15.), duration=25.)
    rows = run_comparison(spec, ['GSLQR', 'V13', 'V13-nopreview'])
"""
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from control.dynamics import AxialDronePlant
from control.trim import find_trim

# 논문 식(32) 의 부드러운 보간 — 양 끝에서 1~4차 도함수가 0이라 속도·고도
# 참조를 이어 붙여도 미분 불연속이 생기지 않는다.
def smoothstep(u):
    u = min(max(float(u), 0.0), 1.0)
    return 126*u**5 - 420*u**6 + 540*u**7 - 315*u**8 + 70*u**9


def speed_ramp(v0, v1, t0, T_ramp):
    """논문 §5.5 의 v_ref(t) = v0 + (v1-v0)·S((t-t0)/T_r)."""
    def ref(t):
        return v0 + (v1 - v0)*smoothstep((t - t0)/T_ramp)
    return ref


@dataclass
class ComparisonSpec:
    """모든 비교군이 **똑같이** 받아야 하는 것들.

    한 곳에 모아 두는 이유는, 비교군마다 다른 값을 쓰는 일이 생기지 않게
    하기 위해서다. 한 요소씩 바꾸는 실험(§5.3)은 이 객체를 복사해 필드
    하나만 바꾸는 식으로 한다.
    """
    params: dict
    reference: Callable[[float], float]      # t → v_ref,x [m/s]
    duration: float
    z_ref: float = 50.0
    dt_plant: float = 0.001
    dt_ctrl: float = 0.02                    # 상위 제어 주기 (논문 Δt_c)
    N: int = 20                              # 예측 구간
    dt_pred: float = 0.05                    # 예측 격자 h
    max_iter: int = 30                       # 솔버 반복 상한
    V_table: Optional[list] = None           # GSLQR 격자 (트림 구간에 맞출 것)
    wind_fn: Optional[Callable] = None
    settle: float = 0.0                      # 평가 제외 구간 [s]
    initial_offset: dict = field(default_factory=dict)   # z, vx 섭동


class ReferenceNotDelivered(AssertionError):
    """참조가 제어기 내부까지 도달하지 않았다."""


# ══════════════════════════════════════════════════════════════════
# 비교군 생성 — 참조를 '읽는 객체'가 무엇인지 여기서 한 번만 정한다
# ══════════════════════════════════════════════════════════════════

def _build(name, spec, v0):
    """(controller, ref_target) 반환.

    ref_target 은 v_ref 를 실제로 읽는 객체다. ProperHybrid 는 참조를 쓰지
    않고 내부 상위 제어기가 쓴다 — 이 구분을 호출부에 맡기면 사고가 난다.
    """
    from control.controller import ScheduledLQR
    from control.hybrid_comparison import ProperHybrid, VirtualNMPC

    P, z = spec.params, spec.z_ref
    preview = (lambda t: (spec.reference(t), 0.0, 0.0, z))

    if name == 'GSLQR':
        if spec.V_table is None:
            raise ValueError(
                "GSLQR 에는 V_table 을 명시할 것. 기본 격자(0~80)는 기체에 따라 "
                "트림이 없는 점을 포함해 유효 구간의 게인까지 오염시킨다.")
        c = ScheduledLQR(P, v_ref=[v0, 0, 0], z_ref=z, V_table=spec.V_table)
        return c, c

    if name in ('V13', 'V13-nopreview'):
        nmpc = VirtualNMPC(
            P, v_ref=[v0, 0, 0], z_ref=z, N=spec.N, dt_nmpc=spec.dt_pred,
            dt_ctrl=spec.dt_ctrl, max_iter=spec.max_iter,
            ref_fn=preview if name == 'V13' else None)
        return ProperHybrid(nmpc, P, dt=spec.dt_plant), nmpc

    if name == 'GINDI':
        from control.gindi import GeometricGuidance
        g = GeometricGuidance(P, v_ref=[v0, 0, 0], z_ref=z, dt_ctrl=spec.dt_ctrl)
        return ProperHybrid(g, P, dt=spec.dt_plant), g

    raise ValueError(f"알 수 없는 비교군: {name}")


def audit_matched(spec, names):
    """비교 전에 조건이 실제로 일치하는지 확인하고, 안 맞으면 알린다.

    반환: 경고 문자열 목록(비면 통과). 예외로 막지 않는 이유는 '계산 비교
    모드'(§5.3)처럼 일부러 다르게 두는 경우가 있기 때문이다 — 다만 그때도
    차이를 **공개**해야 하므로 목록으로 돌려준다.
    """
    warnings = []
    v0 = spec.reference(0.0)
    specs = {}
    for name in names:
        c, tgt = _build(name, spec, v0)
        inner = getattr(c, 'nmpc', c)
        specs[name] = {
            'cost_spec': getattr(inner, 'cost_spec', None),
            'dt_ctrl': getattr(inner, 'dt_ctrl', None),
            'N': getattr(inner, 'N', None),
            'max_iter': getattr(inner, '_max_iter', None),
            'preview': getattr(inner, 'ref_fn', None) is not None,
            'alloc_mode': getattr(c, 'alloc_mode', None),
            'time_align': getattr(c, 'time_align', None),
        }

    def differing(key, ignore_none=True):
        vals = {n: s[key] for n, s in specs.items()
                if not (ignore_none and s[key] is None)}
        return vals if len(set(map(str, vals.values()))) > 1 else None

    for key, why in (('cost_spec', '비용함수가 다르면 구조 차이로 오인된다(§5.3)'),
                     ('dt_ctrl', '입력 갱신률 일치(§5.3)'),
                     ('N', '예측 구간 일치(§5.3)'),
                     ('max_iter', '솔버 설정 일치(§5.3)'),
                     ('alloc_mode', '하위 루프 일치 — 표6 사다리 단을 섞지 말 것'),
                     ('time_align', '하위 루프 일치 — 표6 사다리 단을 섞지 말 것')):
        bad = differing(key)
        if bad:
            warnings.append(f"[{key}] 비교군마다 다름 {bad} — {why}")

    prev = {n: s['preview'] for n, s in specs.items()}
    if len(set(prev.values())) > 1:
        warnings.append(
            f"[preview] 미리보기 보유가 갈린다 {prev}. §5.5 는 '미래 참조를 "
            f"사용하는 모든 제어기에 같은 미리보기'를 요구한다. 구조적으로 "
            f"미래 참조를 못 쓰는 제어기(LQR 등)와 섞어 비교할 때는, 우위 중 "
            f"얼마가 미리보기 덕인지 분리해 보고할 것 "
            f"(V13 과 V13-nopreview 를 함께 돌리면 된다).")
    return warnings


# ══════════════════════════════════════════════════════════════════
# 실행
# ══════════════════════════════════════════════════════════════════

def run_one(name, spec):
    """한 비교군을 돌리고 지표를 낸다. 참조 미전달이면 즉시 예외."""
    P = spec.params
    v0 = spec.reference(0.0)
    trim = find_trim(P, v0, quiet=True)
    if not trim['converged']:
        raise ValueError(f"초기 속도 {v0} m/s 에서 트림이 없다 — 참조를 낮출 것")

    x0 = np.zeros(17)
    x0[2] = spec.z_ref + spec.initial_offset.get('z', 0.0)
    x0[3:6] = [v0 + spec.initial_offset.get('vx', 0.0), 0.0, 0.0]
    x0[6:10] = trim['state'][6:10]
    x0[13:17] = trim['state'][13:17]

    controller, ref_target = _build(name, spec, v0)

    # 참조를 '설정하는 객체'와 '읽는 객체'가 같은지 구조적으로 확인한다.
    # ProperHybrid 는 참조를 안 쓰고 내부 상위 제어기가 쓴다 — 바깥에 v_ref 를
    # 꽂으면 새 속성이 조용히 생기고 제어기는 초기 참조에 머문다. 그 사고가
    # 이 모듈을 만든 이유다(모듈 독스트링 참조).
    consumer = getattr(controller, 'nmpc', controller)
    if ref_target is not consumer:
        raise ReferenceNotDelivered(
            f"{name}: 참조를 넣을 객체({type(ref_target).__name__})가 실제로 "
            f"참조를 읽는 객체({type(consumer).__name__})와 다르다")

    plant = AxialDronePlant(P, dt=spec.dt_plant)
    steps = int(round(spec.duration/spec.dt_plant))
    X = np.zeros((steps + 1, 17))
    X[0] = x0
    v_ref = np.zeros(steps + 1)

    for k in range(steps):
        t = k*spec.dt_plant
        v_ref[k] = spec.reference(t)
        # 참조는 '읽는 객체'에 직접 넣는다.
        ref_target.v_ref = np.array([v_ref[k], 0.0, 0.0])
        if hasattr(ref_target, '_t_now'):
            ref_target._t_now = t          # 미리보기 평가 시각
        w = spec.wind_fn(t) if spec.wind_fn else None
        X[k + 1] = plant.step(X[k], controller(t, X[k]), w)
    v_ref[steps] = spec.reference(steps*spec.dt_plant)

    delivered = float(np.asarray(ref_target.v_ref).ravel()[0])
    if abs(delivered - v_ref[steps - 1]) > 1e-9:
        raise ReferenceNotDelivered(
            f"{name}: 참조가 제어기에 도달하지 않았다 "
            f"(전달 {delivered}, 기대 {v_ref[steps-1]})")

    k0 = int(round(spec.settle/spec.dt_plant))
    omega = np.linalg.norm(X[:, 10:13], axis=1)
    # 실기 실패 판정(MEMORY.md): |ω|>35 자이로 포화, 또는 |ω|>25 가 200 ms 이상
    over = omega > 25.0
    run_len = 0
    sustained = 0
    for flag in over:
        run_len = run_len + 1 if flag else 0
        sustained = max(sustained, run_len)
    failed = (not np.all(np.isfinite(X))) or omega.max() > 35.0 \
        or sustained*spec.dt_plant >= 0.2 or abs(X[-1, 2] - spec.z_ref) > 50.0

    return {
        'name': name,
        'RMSE_z': float(np.sqrt(np.mean((X[k0:, 2] - spec.z_ref)**2))),
        'RMSE_vx': float(np.sqrt(np.mean((X[k0:, 3] - v_ref[k0:])**2))),
        'omega_max': float(omega.max()),
        'z_end': float(X[-1, 2]),
        'vx_end': float(X[-1, 3]),
        'failed': bool(failed),
    }


def run_comparison(spec, names, verbose=True):
    """조건 일치를 먼저 감사하고, 모든 비교군을 같은 조건으로 돌린다."""
    warnings = audit_matched(spec, names)
    if verbose:
        for w in warnings:
            print(f"⚠ 공정성 경고: {w}")
        if not warnings:
            print("✓ 조건 일치 감사 통과")
    rows = [run_one(n, spec) for n in names]
    if verbose:
        print(f"{'비교군':>16} {'RMSE_z':>9} {'RMSE_vx':>9} {'|ω|max':>8} {'판정':>6}")
        for r in rows:
            print(f"{r['name']:>16} {r['RMSE_z']:9.4f} {r['RMSE_vx']:9.4f} "
                  f"{r['omega_max']:8.3f} {'실패' if r['failed'] else '정상':>6}")
    return rows, warnings
