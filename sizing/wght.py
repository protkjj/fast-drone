"""
WGHT — 중량 수렴 모듈
=====================

근거: C-1-ii-01-ICD-001 / WGHT 시트 rev.4 + 검토 지적 2건 반영

[이 모듈이 푸는 문제]
기체 총중량 MTOW 를 구한다. 어려운 점은 구조 중량이 MTOW 에 다시 의존한다는 것이다
(무거운 기체일수록 튼튼해야 하고, 튼튼하면 더 무겁다). 그래서 고정점 반복을 돈다:

    MTOW  ←  W_fixed + W_str(MTOW)

W_fixed(페이로드·배터리·아비오닉스·추진·배선)는 MTOW 와 무관하므로 루프 밖에서 1회만 낸다.
루프 안에서 MTOW 에 따라 변하는 항은 W_str 하나뿐이다 (§2.8).

[순수 함수다 — §0]
상태를 보관하지 않는다. C-2 에서 수천 후보를 배치로 돌리므로 상태를 들고 있으면
설계점끼리 결과가 오염된다. 같은 팀이 이미 겪은 사고다
(PROJECT_REPORT 버그 1-3: NMPC warm start 리셋 누락 → MC 독립성 훼손).

  주의: 이 무상태성은 주입된 strc_fn 이 무상태일 때만 성립한다.
        strc_fn 이 warm start 나 캐시를 들고 있으면 버그 1-3 이 그대로 재현된다.
        options['check_strc_purity']=True 로 켜면 같은 MTOW 2회 호출을 대조해 잡는다.

[relax 격리 원칙 — §0.1]
relax 가 등장하는 곳은 두 줄뿐이다.

    MTOW_{k+1} = MTOW_k + relax * resid_k      ← 갱신
    S_hat      = 1 - (1 - r_hat) / relax       ← 환산

그 외 어떤 판정식·가드·이력에도 relax 에 의존하는 양을 두지 않는다.
같은 버그가 3회 재발한 뒤 세운 원칙이며, 원인은 매번 '완화된 양(Δ)을 판정에 쓴 것'이었다.
그래서 Δ 라는 양 자체를 두지 않는다. resid 만 쓴다.
"""
from __future__ import annotations

import math

# ─────────────────────────────────────────────────────────────────────────────
# datum 규약 (§5-D)
#
# 위치값의 기준점은 GEOM 이 정의하고 전 모듈이 공유해야 한다.
# J_yy 는 기준점이 바뀌면 평행축 항 m*d^2 만큼 통째로 달라지는데, 숫자가 그럴듯하게
# 나오고 부호도 안 뒤집히는 '조용한 계통오차'다. 그래서 규약 문자열을 결과에 실어
# 보낸다 — 숫자와 가정이 같이 다녀야 나중에 검증할 수 있다.
# ─────────────────────────────────────────────────────────────────────────────
DATUM_CONVENTION = (
    "x: 기수(nose) 기준, 후방 양(+). 각 모듈은 자기 것의 cg 기준 관성을 제출하고, "
    "기체 cg 로의 이관은 WGHT 가 평행축 정리로 수행한다. "
    "[미확정 — GEOM 회신(§5-G) 전까지 가정]"
)

# J_yy 를 x 오프셋만으로 계산하는 근거.
# J_yy = Σ m((x-x_cg)^2 + (z-z_cg)^2) 인데, 로터가 동체 z=0 평면에 놓인 배치
# (프로젝트 dynamics.py 의 × 배치도 rotor_positions 의 z 성분이 전부 0)라면
# z 항이 사라져 x 오프셋만 남는다.
JYY_ASSUMPTION = "로터·부품이 동체 z=0 평면에 있다고 가정 → J_yy 는 x 오프셋만으로 결정"


DEFAULT_OPTIONS = {
    'eps_conv':    1e-3,    # 수렴 허용오차 (MTOW 상대). §2.4 — '오차' 기준이지 스텝 기준이 아니다
    'iter_max':    50,      # §1.5 확정
    'relax':       1.0,     # 출하 기본값. 상향은 게이트 2 통과 후 조건부 (§5-12)
    'resid_floor': 1e-6,    # MTOW_k 대비 '상대' 하한. 게이트 1 에서 실측 대체 (§2.6)
    'delta_r':     0.05,    # 발산/리밋사이클 구분 여유 (§3.2)
    'n_confirm':   3,       # 발산·사이클 연속 확인 횟수. 오탐 1건이 설계점 탈락으로 직결되므로 보수적
    'n_min_iter':  3,       # r_hat 추정 전 판정 금지 (§2.4)
    'check_strc_purity': False,   # 켜면 strc_fn 무상태성을 1회 대조 (§0)
}


class WghtError(Exception):
    """WGHT 공통 예외."""


class WghtInputError(WghtError):
    """입력이 비물리 — 설계점이 아니라 호출부가 틀린 것 (§3.4)."""


class WghtModelError(WghtError):
    """strc_fn 이 비물리 값을 반환 — 모델/코드가 틀린 것 (§3.4)."""


# ─────────────────────────────────────────────────────────────────────────────
# 보조 계산
# ─────────────────────────────────────────────────────────────────────────────
def wire_mass(I_max, L_wire, k_wire):
    """배선 중량 W_wire = f(I_max, L_wire).

    f 의 확정 형태는 §5-9 미결(AWG 별 단위중량 표)이다.
    지금은 '전류가 굵기를, 굵기×길이가 무게를 정한다'는 1차 근사만 둔다:

        단위길이중량 [kg/m] ≈ k_wire * I_max
        W_wire              = 단위길이중량 * L_wire

    k_wire 를 MTOW 비례로 정의하면 루프 안 MTOW 의존 항이 W_str 말고 하나 더 생겨
    ICD 의 '반복이 구조·무게 둘뿐' 전제가 깨진다. 그래서 I_max 기반으로만 둔다 (§2.8).
    """
    return k_wire * I_max * L_wire


def mass_properties(point_items, W_str, x_cg_str, J_yy_str):
    """질량 특성(x_cg, J_yy)을 낸다.

    point_items : [(질량[kg], 위치 x[m]), ...] — 점질량으로 다루는 항목
    W_str, x_cg_str, J_yy_str : 구조. 분포 질량이므로 자기 cg 기준 관성을 함께 받는다.

    구조를 점질량으로 놓으면 J_yy 가 계통적으로 과소평가되고, J_yy 가 작으면
    alpha_max = M/J_yy 가 크게 나와 EC 5위 점수가 부풀려지는 방향이다 — 안전측 오차가 아니다.
    그래서 J_yy_str 을 받아 평행축 정리로 이관한다 (§2.7, §5-F).

    반환: (x_cg, J_yy).  필요한 위치값이 하나라도 없으면 (None, None).
    """
    if x_cg_str is None or J_yy_str is None:
        return None, None
    if any(x is None for _, x in point_items):
        return None, None

    items = list(point_items) + [(W_str, x_cg_str)]
    m_tot = sum(m for m, _ in items)
    if m_tot <= 0:
        return None, None

    x_cg = sum(m * x for m, x in items) / m_tot

    # 평행축 정리: J = Σ (J_자기cg + m*d^2).  점질량은 J_자기cg = 0.
    J_yy = J_yy_str + sum(m * (x - x_cg) ** 2 for m, x in items)
    return x_cg, J_yy


# ─────────────────────────────────────────────────────────────────────────────
# 본체
# ─────────────────────────────────────────────────────────────────────────────
def wght(inputs, constants, upstream, strc_fn, options=None):
    """중량 수렴 루프. 순수 함수 — 호출할 때마다 처음부터 계산한다.

    inputs    : {'E_batt'}                      — WGHT 가 직접 쓰는 설계변수 (§1.1)
    constants : {'W_pl','W_avio','e_spec','k_wire','k_pack'}      (§1.2)
    upstream  : {'W_mot','W_bladeset','W_mount','x_mount','I_max',
                 'x_rotor_i','L_wire','x_i'}                       (§1.3)
    strc_fn   : MTOW -> {'W_str', 'x_cg_str', 'J_yy_str',
                         'm_print', 'margin_str', 'print_setting'} (§1.4)
    options   : DEFAULT_OPTIONS 참조                                (§1.5)
    """
    opt = dict(DEFAULT_OPTIONS)
    opt.update(options or {})

    # ── §3.4 사전 검사 — 여기 걸리는 건 '있을 수 있는 설계점'이 아니라 코드가 틀린 것 ──
    if not callable(strc_fn):
        raise WghtInputError("strc_fn 이 연결되지 않았습니다 (§3.4).")
    e_spec = constants['e_spec']
    if e_spec <= 0:
        raise WghtInputError(f"e_spec 은 양수여야 합니다: {e_spec} (나눗셈이 먼저 터짐)")
    if inputs['E_batt'] < 0:
        raise WghtInputError(f"E_batt 가 음수입니다: {inputs['E_batt']}")

    # eps_conv 가 resid_floor 이하면 수렴 판정이 영영 도달하지 못한다.
    # 판정은 |resid| < eps*(1-S_hat)*MTOW 에서, 가드는 |resid| < resid_floor*MTOW 에서
    # 걸리므로 eps*(1-S_hat) < resid_floor 이면 항상 가드가 먼저다. S_hat 은 사전에
    # 모르지만 (1-S_hat) < 1 이므로 eps <= resid_floor 는 무조건 그 경우다.
    # (실측: resid_floor=1e-6, S=0.21 에서 eps<=1.27e-6 부터 가드가 선점 — 예측과 일치)
    if opt['eps_conv'] <= opt['resid_floor']:
        raise WghtInputError(
            f"eps_conv({opt['eps_conv']:.3e}) 가 resid_floor({opt['resid_floor']:.3e}) 이하입니다. "
            "이러면 하한 가드가 항상 먼저 발동해 eps_conv 가 아무 역할도 못 합니다 "
            "(status 는 converged 로 나오지만 S_hat=None 이 되어 오차 보고가 불가능)."
        )

    # ── §2.1 루프 밖 (1회) ──
    W_batt = inputs['E_batt'] / e_spec * (1.0 + constants['k_pack'])
    W_wire = wire_mass(upstream['I_max'], upstream['L_wire'], constants['k_wire'])

    # n_rotor 는 로터 허브 위치 배열의 길이에서 얻는다 (rev.4 #2).
    # 별도 심볼로 받지 않는 이유: 같은 정보를 두 형태로 받으면 불일치 시 어느 쪽이
    # 참인지 WGHT 가 판단해야 한다.
    x_rotor_i = list(upstream['x_rotor_i'])
    n_rotor = len(x_rotor_i)
    if n_rotor <= 0:
        raise WghtInputError("x_rotor_i 가 비어 있습니다 — n_rotor 를 정할 수 없습니다.")

    W_mot_tot = n_rotor * upstream['W_mot']
    W_blade_tot = n_rotor * upstream['W_bladeset']
    W_mount = upstream['W_mount']
    W_prop = W_mot_tot + W_blade_tot + W_mount

    W_fixed = W_prop + constants['W_pl'] + constants['W_avio'] + W_batt + W_wire
    if W_fixed <= 0:
        raise WghtInputError(f"W_fixed 가 0 이하입니다: {W_fixed}")

    # MTOW_0 = W_fixed/(1-phi_guess).  1.3 같은 '배수'가 아니라 구조 비중 추정값으로 쓰면
    # 게이트 3 조정이 '배수 튜닝'이 아니라 '구조 비중 갱신'이 되어 S 와 같은 언어를 쓴다 (§2.1).
    phi_guess = opt.get('phi_guess', 0.23)
    MTOW = W_fixed / (1.0 - phi_guess)

    # ── strc_fn 무상태성 대조 (§0) ──
    if opt['check_strc_purity']:
        a = strc_fn(MTOW)['W_str']
        b = strc_fn(MTOW)['W_str']
        if a != b:
            raise WghtModelError(
                f"strc_fn 이 순수하지 않습니다: 같은 MTOW={MTOW!r} 에 W_str 이 {a} → {b}. "
                "warm start/캐시를 들고 있으면 배치 실행에서 설계점 간 오염이 생깁니다 (§0)."
            )

    # ── §2.2 루프 ──
    history = []
    resid_prev = None
    prev_snap = None            # 직전 반복의 (MTOW, raw, resid, strc) — 리밋사이클 분기 선택용
    n_div_struct = n_div_osc = n_cycle = 0

    for k in range(opt['iter_max']):
        strc = strc_fn(MTOW)
        W_str = strc['W_str']
        if not math.isfinite(W_str) or W_str < 0:
            raise WghtModelError(
                f"strc_fn 이 비물리 W_str={W_str} 를 반환했습니다 (MTOW={MTOW}). "
                "이건 '있을 수 있는 설계점'이 아니라 모델이 틀린 것입니다 (§3.4)."
            )

        raw = W_fixed + W_str          # = 반환 대상. §2.5 의 Σbreakdown 과 항등
        resid = raw - MTOW             # 부호 유지, 완화 전 (§2.2)
        history.append((MTOW, raw, resid))

        snap = (MTOW, raw, resid, strc)
        ctx = dict(W_fixed=W_fixed, W_batt=W_batt, W_wire=W_wire, W_mot_tot=W_mot_tot,
                   W_blade_tot=W_blade_tot, W_mount=W_mount, constants=constants,
                   upstream=upstream, x_rotor_i=x_rotor_i, history=history, n_iter=k + 1)

        # ── 1. resid 하한 가드 (§2.6) ──
        # resid 가 수치 노이즈 수준으로 떨어지면 r_hat 은 노이즈만 잰다. 노이즈는 방향이
        # 없어 S_hat 이 1 근처에도 1 이상에도 찍힌다. '발산 오판'과 '수렴 불능'은 같은
        # 현상의 양면이므로 가드를 한 곳에 두고, 하한 미달은 '이미 수렴'으로 보낸다.
        floor_k = opt['resid_floor'] * MTOW
        if abs(resid) < floor_k:
            return _result(snap, 'converged', S_hat=None, err=0.0, ctx=ctx,
                           note='resid 하한 미달 — 이미 수렴 (§2.6)')

        S_hat = None
        if resid_prev is not None:
            # r_hat 을 Δ 비가 아니라 resid 비로 정의한다. 값은 같고(relax 가 약분되고),
            # relax 격리 원칙이 정의 단계에서 보장된다 (§2.3).
            r_hat = resid / resid_prev
            S_hat = 1.0 - (1.0 - r_hat) / opt['relax']

            # 카운터는 r_hat 이 생긴 시점부터 세되, 판정은 n_min_iter 이후에만 한다 (§2.4).
            n_div_struct = n_div_struct + 1 if r_hat >= 1.0 else 0
            n_div_osc = n_div_osc + 1 if r_hat <= -(1.0 + opt['delta_r']) else 0

            in_band = -(1.0 + opt['delta_r']) <= r_hat <= -(1.0 - opt['delta_r'])
            not_growing = abs(resid) <= abs(resid_prev)
            n_cycle = n_cycle + 1 if (in_band and not_growing) else 0

            if k + 1 >= opt['n_min_iter']:
                # ── 4. 구조 발산 ──
                # r_hat >= 1  <=>  S_hat >= 1  (relax>0 에서 항상). deadband 를 두지 않는다:
                # 두면 1 < S_hat < 1+delta_r 이 어느 status 에도 안 걸리는 구멍이 생기고,
                # 그 구간에서 err 분모 (1-S_hat) 가 음수가 되어 판정식이 무조건 통과한다
                # (= 발산을 'converged' 로 뒤집는다).
                if n_div_struct >= opt['n_confirm']:
                    return _result(snap, 'diverged', S_hat=S_hat, err=None, ctx=ctx,
                                   div_mode='structural',
                                   note='S_hat>=1 — 고정점이 존재하지 않음. STRC 모델 책임 (§3.3)')

                # ── 5. 진동 발산 (수치) ──
                if n_div_osc >= opt['n_confirm']:
                    return _result(snap, 'diverged', S_hat=S_hat, err=None, ctx=ctx,
                                   div_mode='numerical',
                                   note='relax 과대 — 우리의 relax 선택 책임 (§3.3)')

                # ── 6. 리밋 사이클 ──
                # 슬라이서 출력이 계단형(레이어 수·둘레 수가 정수)이면 MTOW 가 두 값 사이를
                # 무한 진동하고 |resid| 는 양자 크기에서 멈춘다. 발산이 아니라 이산화 한계다.
                if n_cycle >= opt['n_confirm'] and prev_snap is not None:
                    return _limit_cycle_result(snap, prev_snap, S_hat, ctx)

                # ── 7. 수렴 판정 ──
                # S_hat < 1 을 전제로 명시한다. err_iterate 유도가 그 범위에서만 유효하고,
                # 없으면 분모 음수로 판정식이 무조건 통과한다.
                if S_hat < 1.0:
                    err_iterate = abs(resid) / (1.0 - S_hat)
                    if err_iterate < opt['eps_conv'] * MTOW:
                        err = S_hat * abs(resid) / (1.0 - S_hat)   # 반환값 오차 (§2.5)
                        return _result(snap, 'converged', S_hat=S_hat, err=abs(err), ctx=ctx)

        resid_prev = resid
        prev_snap = snap
        MTOW = MTOW + opt['relax'] * resid          # §2.2 갱신 — relax 는 여기서만 쓴다

    # ── 8. 반복 소진 ──
    # 이진 판단을 Integration 에 떠넘기지 않는다. err 을 같이 줘서 임계로 처리하게 한다 (§3.5).
    err = None
    if S_hat is not None and S_hat < 1.0:
        err = abs(S_hat) * abs(resid) / (1.0 - S_hat)
    return _result(snap, 'max_iter', S_hat=S_hat, err=err, ctx=ctx)


# ─────────────────────────────────────────────────────────────────────────────
# 결과 조립
# ─────────────────────────────────────────────────────────────────────────────
def _limit_cycle_result(snap, prev_snap, S_hat, ctx):
    """리밋 사이클: 진동 두 값 중 |resid| 작은 쪽 '분기를 통째로' 채택한다.

    중점을 반환하면 안 된다. 중점은 어떤 W_fixed + W_str(.) 과도 같지 않아
    §2.5 의 항등식(MTOW == Σbreakdown)이 깨진다. breakdown 을 같이 평균내는 것도 안 되는데,
    print_setting 의 레이어 수·둘레 수는 정수라 두 설정의 중간값이 물리적으로 존재하지
    않기 때문이다. print_setting 은 진단값이 아니라 제작으로 나가는 산출물이다 (§4).

    '참값이 두 값 사이에 있다'는 의도는 값이 아니라 err 로 표현한다 —
    판정용/보고용 오차를 나눈 §2.5 와 같은 처리다.
    """
    chosen = snap if abs(snap[2]) <= abs(prev_snap[2]) else prev_snap
    quantum = abs(snap[1] - prev_snap[1])      # 두 분기의 반환값 차 = 양자 크기
    return _result(chosen, 'limit_cycle', S_hat=S_hat, err=quantum / 2.0, ctx=ctx,
                   note='이산화 한계까지 수렴. 탈락시키지 말 것 (§3.2)')


def _result(snap, status, S_hat, err, ctx, div_mode=None, note=None):
    MTOW_iter, raw, resid, strc = snap
    c, u = ctx['constants'], ctx['upstream']

    # §2.5 — 완화된 반복값이 아니라 마지막 STRC 호출의 합을 그대로 보고한다.
    # 아래 breakdown 의 합은 정의상 raw 와 정확히 같다 (검산이 필요 없는 구조).
    breakdown = {
        'W_pl':       c['W_pl'],
        'W_avio':     c['W_avio'],
        'W_batt':     ctx['W_batt'],
        'W_wire':     ctx['W_wire'],
        'W_mot':      ctx['W_mot_tot'],
        'W_bladeset': ctx['W_blade_tot'],
        'W_mount':    ctx['W_mount'],
        'W_str':      strc['W_str'],
    }

    x_i = u.get('x_i') or {}
    point_items = [
        (c['W_pl'],   x_i.get('W_pl')),
        (c['W_avio'], x_i.get('W_avio')),
        (ctx['W_batt'], x_i.get('W_batt')),
        (ctx['W_wire'], x_i.get('W_wire')),      # 경로 중점 가정 (§5-D)
        (ctx['W_mount'], u.get('x_mount')),
    ]
    # 로터는 허브 위치에 놓는다. 암 부착점에 놓으면 J_yy 기여가 (0.25/0.05)^2 = 25배
    # 과소평가된다 — rev.4 #1 이 x_rotor_i 를 신설한 이유다.
    per_rotor = u['W_mot'] + u['W_bladeset']
    point_items += [(per_rotor, x) for x in ctx['x_rotor_i']]

    x_cg, J_yy = mass_properties(point_items, strc['W_str'],
                                 strc.get('x_cg_str'), strc.get('J_yy_str'))

    return {
        'MTOW':      raw,                 # §2.5 — Σbreakdown 과 항등
        'breakdown': breakdown,
        'x_cg':      x_cg,
        'J_yy':      J_yy,
        'n_iter':    ctx['n_iter'],
        'status':    status,
        'div_mode':  div_mode,            # 'diverged' 일 때만 채워짐 (§3.3)
        'history':   list(ctx['history']),  # (MTOW_k, MTOW_raw, resid_k), resid 부호 유지 (§4.1)
        'err':       err,                 # 반환 MTOW 의 오차. S_hat>=1 이면 None
        'S_hat':     S_hat,               # 추정 실패 시 None — 사후 필터가 가능하도록 (§4)
        'm_print':       strc.get('m_print'),
        'margin_str':    strc.get('margin_str'),
        'print_setting': strc.get('print_setting'),
        'MTOW_iterate':  MTOW_iter,       # 진단: 완화된 반복값 (보고용 아님)
        'datum':         DATUM_CONVENTION,
        'jyy_assumption': JYY_ASSUMPTION,
        'note':          note,
    }
