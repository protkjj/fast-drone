"""
WGHT 검증 — 게이트 1 (루프가 돈다)
==================================

실행: python3 -m sizing.test_wght   (저장소 루트에서)

[검증 전략]
이 모듈은 C-2 에서 수천 후보에 반복 사용되는 프로덕션 코드이므로 유닛테스트를 붙인다.
핵심은 '알려진 정답과 대조'다 — 선형 스텁 W_str = S*MTOW 는 고정점이 해석적으로
MTOW* = W_fixed/(1-S) 로 나오고, 발산 경계가 정확히 S=1 이며, S_hat 이 relax 와 무관하게
S 를 복원해야 한다. 이 셋이 §7.1 이 말한 '이론 재현' 기준이다.

숫자가 이론과 다르면 "경계가 그 값이다"가 아니라 "우리 코드에 버그가 있다"가 정답이다.
"""
import math
import sys

from sizing.wght import (wght, mass_properties, WghtInputError, WghtModelError,
                         DEFAULT_OPTIONS)
from sizing.strc_stub import (make_linear_strc, make_power_strc, make_quantized_strc,
                              make_impure_strc)

# ── 공통 입력 ────────────────────────────────────────────────────────────────
BASE = dict(
    inputs={'E_batt': 500.0},
    constants={'W_pl': 1.0, 'W_avio': 0.5, 'e_spec': 180.0, 'k_wire': 2e-4, 'k_pack': 0.15},
    upstream={'W_mot': 0.12, 'W_bladeset': 0.03, 'W_mount': 0.10, 'x_mount': 0.45,
              'I_max': 60.0, 'L_wire': 2.5,
              'x_rotor_i': [0.30, 0.30, 0.70, 0.70],
              'x_i': {'W_pl': 0.20, 'W_avio': 0.35, 'W_batt': 0.50, 'W_wire': 0.50}},
)


def run(strc_fn, **opts):
    return wght(strc_fn=strc_fn, options=opts or None, **BASE)


def W_fixed_of(res):
    """반환된 breakdown 에서 W_str 을 뺀 나머지 = W_fixed."""
    return sum(v for k, v in res['breakdown'].items() if k != 'W_str')


def check(cond, msg):
    if not cond:
        print(f"  FAIL: {msg}")
        raise AssertionError(msg)


def banner(n, title):
    print(f"\n{'='*70}\nTEST {n}: {title}")


# ─────────────────────────────────────────────────────────────────────────────
def test_1_analytic():
    banner(1, "선형 스텁 — 해석해 대조 (고정점·S_hat)")
    S = 0.21
    r = run(make_linear_strc(S))
    exact = W_fixed_of(r) / (1 - S)
    rel = abs(r['MTOW'] - exact) / exact

    print(f"  status={r['status']}  n_iter={r['n_iter']}  S_hat={r['S_hat']:.9f}")
    print(f"  MTOW  = {r['MTOW']:.9f}")
    print(f"  해석해 = {exact:.9f}   상대오차 = {rel:.3e}")

    check(r['status'] == 'converged', "수렴해야 한다")
    check(abs(r['S_hat'] - S) < 1e-9, f"S_hat 이 S 를 복원해야 한다: {r['S_hat']} vs {S}")
    # §5-7: 실제 정확도는 eps 가 아니라 eps*S_hat 이 상한이다 (판정은 err_iterate, 납품은 err)
    check(rel <= DEFAULT_OPTIONS['eps_conv'] * S,
          f"실제 오차가 상한 eps*S 이내여야 한다: {rel:.3e} > {DEFAULT_OPTIONS['eps_conv']*S:.3e}")
    print("  PASS")


def test_2_relax_invariance():
    banner(2, "relax 불변성 — §0.1 격리 원칙")
    S = 0.21
    rows = []
    for relax in [0.3, 0.5, 1.0, 1.5, 2.0]:
        r = run(make_linear_strc(S), relax=relax, eps_conv=1e-5)
        rows.append((relax, r['MTOW'], r['S_hat'], r['n_iter']))
        print(f"  relax={relax:4.1f}  MTOW={r['MTOW']:.9f}  S_hat={r['S_hat']:.9f}  n={r['n_iter']}")

    # S_hat 은 relax 와 무관해야 한다 — 이게 relax-불변 지표라고 부른 근거다
    for relax, _, sh, _ in rows:
        check(abs(sh - S) < 1e-9, f"relax={relax} 에서 S_hat 이 어긋남: {sh}")
    # MTOW 도 relax 와 무관 (수렴 허용오차 범위 내)
    spread = max(m for _, m, _, _ in rows) - min(m for _, m, _, _ in rows)
    print(f"  MTOW 최대편차 = {spread:.3e}")
    check(spread / rows[0][1] < 1e-5, f"MTOW 가 relax 에 끌려다님: 편차 {spread:.3e}")
    print("  PASS")


def test_3_sum_identity():
    banner(3, "§2.5 항등식 — MTOW == Σbreakdown")
    for name, fn, opts in [
        ("선형",     make_linear_strc(0.21),          {}),
        ("멱함수",   make_power_strc(0.35, 0.92),     {}),
        ("양자화",   make_quantized_strc(0.21, 0.02), {'relax': 2.0}),
    ]:
        r = run(fn, **opts)
        total = sum(r['breakdown'].values())
        d = abs(total - r['MTOW'])
        print(f"  {name:6s} status={r['status']:12s} |Σbreakdown - MTOW| = {d:.3e}")
        # 부동소수 합산 순서 차이만 허용. 항등식이므로 이 이상 벌어지면 안 된다.
        check(d <= 1e-12 * max(1.0, r['MTOW']), f"{name}: 항등식이 깨짐 ({d:.3e})")
    print("  PASS")


def test_4_structural_divergence():
    banner(4, "구조 발산 — 경계가 정확히 S=1 (§7.2)")
    print(f"  {'S':>6} {'status':>12} {'div_mode':>11} {'err':>10}")
    for S, want in [(0.99, 'max_iter'), (1.00, 'diverged'), (1.10, 'diverged'), (2.00, 'diverged')]:
        r = run(make_linear_strc(S))
        er = 'None' if r['err'] is None else f"{r['err']:.2e}"
        print(f"  {S:>6.2f} {r['status']:>12} {str(r['div_mode']):>11} {er:>10}")
        check(r['status'] == want, f"S={S}: {want} 여야 하는데 {r['status']}")
        if want == 'diverged':
            check(r['div_mode'] == 'structural', f"S={S}: div_mode 가 structural 이어야 함")
            check(r['err'] is None, f"S={S}: S_hat>=1 이면 err 은 None 이어야 함")
    print("  PASS")


def test_5_hole_regression():
    banner(5, "[회귀] Ŝ ∈ (1, 1+delta_r) 구멍 — 발산을 수렴으로 뒤집던 자리")
    # 수정 전 동작: |r̂| >= 1+delta_r 만 diverged 로 잡아 S=1.02 가 어디에도 안 걸리고,
    # err_iterate = |resid|/(1-1.02) 가 음수라 판정식 (< eps*MTOW) 을 무조건 통과했다.
    # → 발산 중인 설계점이 'converged' 로 보고되고 err 도 음수로 나갔다.
    for S in [1.001, 1.02, 1.049]:
        r = run(make_linear_strc(S))
        print(f"  S={S:<6}  status={r['status']:10s}  div_mode={str(r['div_mode']):11s}  err={r['err']}")
        check(r['status'] == 'diverged', f"S={S} 는 diverged 여야 한다 (구멍 재발)")
        check(r['div_mode'] == 'structural', f"S={S}: structural 이어야 한다")
        check(r['err'] is None, f"S={S}: err 이 None 이어야 한다 (음수 유출 금지)")
    # 음수 err 이 어떤 경로로도 새어나가지 않는지 전 구간 확인
    for S in [0.2, 0.5, 0.9, 0.99, 1.0, 1.02, 1.5, 2.0]:
        r = run(make_linear_strc(S))
        check(r['err'] is None or r['err'] >= 0, f"S={S}: 음수 err 유출 ({r['err']})")
    print("  음수 err 유출 없음 (S=0.2~2.0 전 구간)")
    print("  PASS")


def test_6_numerical_divergence():
    banner(6, "수치 발산 — relax > 2/(1-S) 에서만 (§3.3)")
    S = 0.21
    limit = 2 / (1 - S)
    print(f"  이론 안정한계 relax < 2/(1-S) = {limit:.4f}")
    print(f"  {'relax':>6} {'h_theory':>9} {'status':>12} {'div_mode':>11}")
    for relax in [1.0, 2.0, 2.4, 2.6, 3.0]:
        h = 1 + relax * (S - 1)
        r = run(make_linear_strc(S), relax=relax)
        print(f"  {relax:>6.2f} {h:>9.4f} {r['status']:>12} {str(r['div_mode']):>11}")
        if abs(h) < 0.99:                       # 여유를 두고 안정 구간만 단정
            check(r['status'] == 'converged', f"relax={relax}: 안정 구간인데 {r['status']}")
        if abs(h) > 1.01:                       # 확실한 불안정 구간
            check(r['status'] == 'diverged', f"relax={relax}: 불안정인데 {r['status']}")
            check(r['div_mode'] == 'numerical',
                  f"relax={relax}: S<1 이므로 numerical 이어야 한다 (구조 발산과 같은 색이면 결론이 흐려짐)")
    print("  PASS")


def test_7_limit_cycle():
    banner(7, "리밋 사이클 — 중점이 아니라 분기 채택 (검토 지적 1)")
    # 두 경우를 다 본다.
    #  (a) 두 분기의 반환값(raw)이 같은 양자 구간 → raw 고정 → err = 0 (정확)
    #  (b) 두 분기가 다른 구간   → raw 가 두 값 → err = 두 값 차의 절반
    for label, q, relax in [("(a) raw 고정", 0.02, 2.0), ("(b) raw 진동", 0.02, 2.5)]:
        r = run(make_quantized_strc(0.21, q), relax=relax)
        print(f"  {label}: status={r['status']}  n_iter={r['n_iter']}  err={r['err']:.4e}")
        check(r['status'] == 'limit_cycle', f"{label}: limit_cycle 이어야 하는데 {r['status']}")

        # 핵심: 중점을 반환하면 이 항등식이 깨진다. 분기를 통째로 채택했는지 이걸로 본다.
        total = sum(r['breakdown'].values())
        check(abs(total - r['MTOW']) <= 1e-12 * r['MTOW'],
              f"{label}: 중점 반환 흔적 — 항등식이 깨짐")

        # 반환값은 실제로 이력에 있던 반복의 raw 중 하나여야 한다 (합성값이면 안 된다)
        raws = [h[1] for h in r['history']]
        check(any(abs(r['MTOW'] - x) <= 1e-12 * r['MTOW'] for x in raws),
              f"{label}: 반환 MTOW 가 어떤 반복의 raw 와도 일치하지 않는다 (합성값)")

        # err 은 두 분기 반환값 차의 절반이어야 한다
        gap = abs(raws[-1] - raws[-2])
        print(f"       두 분기 raw 차={gap:.4e}  err={r['err']:.4e}  (기대 {gap/2:.4e})")
        check(r['err'] is not None and r['err'] >= 0, f"{label}: err 이 음수/None")
        check(abs(r['err'] - gap / 2) <= 1e-12 * max(1.0, gap), f"{label}: err != 양자/2")

        # print_setting 은 제작으로 나가는 산출물이다. 레이어 수는 정수여야 하고,
        # 두 설정의 '평균'은 존재하지 않는 기계 설정이다.
        ps = r['print_setting']
        check(isinstance(ps['n_layer'], int), f"{label}: n_layer 가 정수가 아님: {ps['n_layer']!r}")
    print("  → err=0 은 버그가 아니다: 두 분기가 같은 raw 를 주면 그 값이 정확한 고정점이다")
    print("  PASS")


def test_8_quantization_at_ship_setting():
    banner(8, "출하 설정(relax=1)에서는 양자화가 사이클을 만들지 않는다")
    # relax=1 이면 반복식이 M_{k+1} = g(M_k) 이고 g 가 단조증가면 2-주기 궤도가 없다.
    # (a<b, f(a)=b>a, f(b)=a<b 이면 f(a)>f(b) 로 단조성 모순)
    print(f"  {'quantum':>9} {'status':>12} {'n_iter':>7}")
    for q in [0.005, 0.02, 0.1, 0.5, 2.0]:
        r = run(make_quantized_strc(0.21, q), relax=1.0)
        print(f"  {q:>9.3f} {r['status']:>12} {r['n_iter']:>7}")
        check(r['status'] == 'converged',
              f"q={q}: relax=1 에서는 수렴해야 한다 (사이클은 relax>1 에서만)")
    print("  → limit_cycle 기계는 §7.4 스윕 격자(relax 0.3~3.0)용이지 출하 설정용이 아니다")
    print("  PASS")


def test_9_min_iter_rule():
    banner(9, "r̂ 추정 전 수렴 선언 금지 (§2.4)")
    for S in [0.05, 0.21, 0.4, 0.6]:
        r = run(make_linear_strc(S))
        floor_path = r['note'] is not None      # 하한 가드 경로는 예외
        print(f"  S={S:<5} status={r['status']:10s} n_iter={r['n_iter']}  "
              f"{'(하한 가드)' if floor_path else ''}")
        if r['status'] == 'converged' and not floor_path:
            check(r['n_iter'] >= DEFAULT_OPTIONS['n_min_iter'],
                  f"S={S}: {r['n_iter']}회 만에 수렴 선언 — r̂ 없이 판정했다")
    print("  PASS")


def test_10_max_iter_reports_err():
    banner(10, "max_iter 는 err 을 동반한다 (§3.5 — 이진 판단을 넘기지 않는다)")
    r = run(make_linear_strc(0.95))
    print(f"  S=0.95  status={r['status']}  n_iter={r['n_iter']}  "
          f"S_hat={r['S_hat']:.4f}  err={r['err']:.4e}")
    check(r['status'] == 'max_iter', f"max_iter 여야 하는데 {r['status']}")
    check(r['err'] is not None and r['err'] > 0, "err 을 같이 줘야 Integration 이 임계로 처리한다")
    print("  PASS")


def test_11_purity_guard():
    banner(11, "strc_fn 무상태성 대조 (§0 — 버그 1-3 재발 방지)")
    try:
        run(make_impure_strc(0.21), check_strc_purity=True)
        check(False, "상태를 든 strc_fn 을 통과시켰다")
    except WghtModelError as e:
        print(f"  잡힘: {str(e)[:72]}...")
    # 기본값(꺼짐)에서는 통과해야 한다 — 배치 성능 때문에 상시 켜지 않는다
    r = run(make_impure_strc(0.21))
    print(f"  기본값(꺼짐)에서는 실행됨: status={r['status']}")
    check(DEFAULT_OPTIONS['check_strc_purity'] is False, "기본값은 꺼져 있어야 한다")
    print("  PASS")


def test_12_exceptions():
    banner(12, "비물리 입력·모델은 예외 (§3.4)")
    # (1) e_spec <= 0
    bad = {**BASE, 'constants': {**BASE['constants'], 'e_spec': 0.0}}
    try:
        wght(strc_fn=make_linear_strc(0.21), **bad); check(False, "e_spec=0 통과")
    except WghtInputError:
        print("  e_spec<=0        → WghtInputError ✓")
    # (2) strc_fn 미연결
    try:
        wght(strc_fn=None, **BASE); check(False, "strc_fn=None 통과")
    except WghtInputError:
        print("  strc_fn 미연결    → WghtInputError ✓")
    # (3) W_str < 0
    try:
        wght(strc_fn=make_linear_strc(-0.5), **BASE); check(False, "W_str<0 통과")
    except WghtModelError:
        print("  W_str<0          → WghtModelError ✓")
    # (4) eps_conv <= resid_floor (설정 함정)
    try:
        run(make_linear_strc(0.21), eps_conv=1e-8); check(False, "eps<=floor 통과")
    except WghtInputError:
        print("  eps<=resid_floor → WghtInputError ✓")
    print("  PASS")


def test_13_mass_properties():
    banner(13, "질량 특성 — 평행축 정리·병진 불변성")
    # (a) 전 부품을 한 점에 모으면 평행축 항이 전부 사라져 J_yy == J_yy_str
    same = [(1.0, 0.5), (2.0, 0.5)]
    x_cg, J = mass_properties(same, W_str=3.0, x_cg_str=0.5, J_yy_str=0.7)
    print(f"  (a) 한 점 집중: x_cg={x_cg:.6f}  J_yy={J:.6f}  (J_yy_str=0.7)")
    check(abs(x_cg - 0.5) < 1e-12 and abs(J - 0.7) < 1e-12, "평행축 항이 남았다")

    # (b) 전 위치를 +d 평행이동 → x_cg 는 +d, J_yy 는 불변
    d = 1.234
    items = [(1.0, 0.2), (2.0, 0.8)]
    x0, J0 = mass_properties(items, 3.0, 0.5, 0.7)
    x1, J1 = mass_properties([(m, x + d) for m, x in items], 3.0, 0.5 + d, 0.7)
    print(f"  (b) 평행이동: Δx_cg={x1-x0:.9f} (기대 {d})  ΔJ_yy={J1-J0:.3e} (기대 0)")
    check(abs((x1 - x0) - d) < 1e-9 and abs(J1 - J0) < 1e-9, "병진 불변성 위반")

    # (c) 로터를 허브(암 끝)가 아니라 부착점에 놓으면 J_yy 가 달라져야 한다.
    #     달라지지 않으면 x_rotor_i 를 안 쓰고 있다는 뜻이다 (rev.4 #1 의 25배 과소평가).
    hub = wght(strc_fn=make_linear_strc(0.21), **BASE)
    near = {**BASE, 'upstream': {**BASE['upstream'], 'x_rotor_i': [0.5, 0.5, 0.5, 0.5]}}
    att = wght(strc_fn=make_linear_strc(0.21), **near)
    print(f"  (c) 허브 배치 J_yy={hub['J_yy']:.6f} / 부착점 배치 J_yy={att['J_yy']:.6f}")
    check(hub['J_yy'] is not None and att['J_yy'] is not None, "J_yy 가 계산되지 않았다")
    check(abs(hub['J_yy'] - att['J_yy']) > 1e-6, "x_rotor_i 가 J_yy 에 반영되지 않는다")

    # (d) 상류 위치가 없으면 조용히 0 을 만들지 말고 None
    nopos = {**BASE, 'upstream': {**BASE['upstream'], 'x_i': {}}}
    r = wght(strc_fn=make_linear_strc(0.21), **nopos)
    print(f"  (d) 위치 결측 시: x_cg={r['x_cg']}  J_yy={r['J_yy']}")
    check(r['x_cg'] is None and r['J_yy'] is None, "위치가 없는데 숫자를 만들어냈다")
    print("  PASS")


def test_14_history_and_shape():
    banner(14, "출력 규격 — history 3튜플·resid 부호·n_rotor")
    r = run(make_linear_strc(0.21))
    h = r['history']
    print(f"  history 길이={len(h)}  첫 항목={tuple(round(v, 6) for v in h[0])}")
    check(all(len(t) == 3 for t in h), "history 는 (MTOW_k, MTOW_raw, resid_k) 3튜플이어야 한다")
    # 부호가 없으면 과완화 구간(h'<0) 사후 분석이 불가능하다 (§4.1)
    r_over = run(make_linear_strc(0.21), relax=2.0, eps_conv=1e-5)
    signs = {math.copysign(1, t[2]) for t in r_over['history']}
    print(f"  relax=2.0 의 resid 부호 집합 = {signs}")
    check(len(signs) > 1, "과완화인데 resid 부호가 한 종류 — 절댓값으로 저장하고 있다")

    # n_rotor 는 x_rotor_i 길이에서 온다 (rev.4 #2)
    six = {**BASE, 'upstream': {**BASE['upstream'], 'x_rotor_i': [0.3] * 3 + [0.7] * 3}}
    r6 = wght(strc_fn=make_linear_strc(0.21), **six)
    per = BASE['upstream']['W_mot'] + BASE['upstream']['W_bladeset']
    got = r6['breakdown']['W_mot'] + r6['breakdown']['W_bladeset']
    print(f"  로터 6개: W_mot+W_bladeset = {got:.6f}  (기대 {6*per:.6f})")
    check(abs(got - 6 * per) < 1e-12, "n_rotor 가 x_rotor_i 길이를 따르지 않는다")

    for k in ['MTOW', 'breakdown', 'x_cg', 'J_yy', 'n_iter', 'status', 'div_mode',
              'history', 'err', 'S_hat', 'm_print', 'margin_str', 'print_setting']:
        check(k in r, f"출력 키 누락: {k}")
    print("  출력 키 13종 전부 존재")
    print("  PASS")


def test_15_purity_of_wght():
    banner(15, "wght 자신의 무상태성 — 같은 입력이면 같은 출력")
    fn = make_linear_strc(0.21)
    a = run(fn)
    b = run(fn)
    c = run(make_linear_strc(1.5))      # 사이에 발산 설계점을 끼워넣는다
    d = run(fn)
    print(f"  1회차 MTOW={a['MTOW']:.12f}")
    print(f"  2회차 MTOW={b['MTOW']:.12f}")
    print(f"  발산 설계점(S=1.5) 통과 후 MTOW={d['MTOW']:.12f}  (status={c['status']})")
    check(a['MTOW'] == b['MTOW'] == d['MTOW'], "호출 순서가 결과를 바꾼다 — 상태가 남아 있다")
    print("  PASS")


ALL = [test_1_analytic, test_2_relax_invariance, test_3_sum_identity,
       test_4_structural_divergence, test_5_hole_regression, test_6_numerical_divergence,
       test_7_limit_cycle, test_8_quantization_at_ship_setting, test_9_min_iter_rule,
       test_10_max_iter_reports_err, test_11_purity_guard, test_12_exceptions,
       test_13_mass_properties, test_14_history_and_shape, test_15_purity_of_wght]

if __name__ == '__main__':
    failed = []
    for t in ALL:
        try:
            t()
        except AssertionError as e:
            failed.append((t.__name__, str(e)))
    print(f"\n{'='*70}")
    if failed:
        print(f"FAILED {len(failed)}/{len(ALL)}")
        for n, m in failed:
            print(f"  - {n}: {m}")
        sys.exit(1)
    print("ALL TESTS PASSED")
    print('='*70)
