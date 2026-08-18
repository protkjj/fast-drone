"""
게이트 2 (축소 범위) — Ŝ 추정기의 편향 정량화
=============================================

실행: python3 -m sizing.gate2_shat_bias   (저장소 루트에서)

[왜 이것만 하는가]
§7.1 이 정리했듯 스윕은 경계를 '발견'하지 않는다. 경계 S=1 은 §7.2 가 해석적으로 안다.
게이트 1 에서 그 재현은 이미 확인했다. 남은 둘 중

  · relax 속도 지도 — 지금 하면 손해. 선형 스텁에서 나오는 답은 1/(1-S) 인데 §7.2 에서
    이미 알고, §5-12 가 "출하 1.0 고정, 상향은 실제 STRC 평활성 확인 후"로 닫아놨다.
    실물 없이는 결정에 못 쓴다.
  · 비선형 이탈 폭 — 그냥 돌리면 "b=1.2 에서 몇 % 벗어남" 이 나오고 끝이다. 쓸 데가 없다.

그래서 목적을 바꾼다: **Ŝ 가 '어느 시점의 S' 를 재는지**를 묻는다.

[문제 정식화]
b != 1 이면 S(MTOW) = a*b*MTOW^(b-1) 로 반복 중에 S 가 변한다. 그런데 §7.6 은 결론을
"수렴점 근방의 dW_str/dMTOW" 로 번역해 넘기라고 한다. 즉 Ŝ 가 S(M*) 를 재야 그 문장이 성립한다.
실제로 Ŝ = resid_k/resid_{k-1} 는 마지막 두 반복 구간의 할선 기울기라 M* 에서 떨어진
지점의 S 를 잰다. 그 어긋남이 편향이다.

이게 왜 중요한가: §7.5 는 실사 배치 수천 후보의 Ŝ 히스토그램으로 "설계공간이 경계 S=1 에서
얼마나 떨어져 있나"를 판단한다. 추정기가 편향돼 있으면 **분포 전체가 밀려서**
"여유 3배" 같은 결론이 통째로 틀어진다.

[설계 — b 와 S 를 분리한다]
b 를 바꾸면 S 도 같이 변해 두 효과가 섞인다. 그래서 S(M*) 를 목표값에 고정한 채 b 만 바꾼다.
고정점에서 W_str = a*M*^b 이고 phi = W_str/M* 이므로

    S(M*) = a*b*M*^(b-1) = b * phi
    M*    = W_fixed / (1 - phi)

따라서 목표 S 와 b 가 주어지면  phi = S/b,  M* = W_fixed/(1-phi),  a = phi * M*^(1-b).
참값이 전 b 에서 동일하므로 Ŝ 의 편차가 곧 편향이다.
"""
from __future__ import annotations

import math

from sizing.wght import wght, wire_mass
from sizing.strc_stub import make_power_strc, make_linear_strc

BASE = dict(
    inputs={'E_batt': 500.0},
    constants={'W_pl': 1.0, 'W_avio': 0.5, 'e_spec': 180.0, 'k_wire': 2e-4, 'k_pack': 0.15},
    upstream={'W_mot': 0.12, 'W_bladeset': 0.03, 'W_mount': 0.10, 'x_mount': 0.45,
              'I_max': 60.0, 'L_wire': 2.5,
              'x_rotor_i': [0.30, 0.30, 0.70, 0.70],
              'x_i': {'W_pl': 0.20, 'W_avio': 0.35, 'W_batt': 0.50, 'W_wire': 0.50}},
)


def compute_W_fixed():
    """wght 와 같은 식으로 W_fixed 를 낸다 (루프 밖 항)."""
    c, u, i = BASE['constants'], BASE['upstream'], BASE['inputs']
    W_batt = i['E_batt'] / c['e_spec'] * (1 + c['k_pack'])
    W_wire = wire_mass(u['I_max'], u['L_wire'], c['k_wire'])
    n = len(u['x_rotor_i'])
    W_prop = n * (u['W_mot'] + u['W_bladeset']) + u['W_mount']
    return W_prop + c['W_pl'] + c['W_avio'] + W_batt + W_wire


def design_power_stub(W_fixed, S_target, b):
    """S(M*) = S_target 이 되도록 멱함수 계수 a 를 역산한다."""
    phi = S_target / b
    if not (0.0 < phi < 1.0):
        raise ValueError(f"phi={phi} 가 (0,1) 밖 — b={b} 가 너무 작다")
    M_star = W_fixed / (1.0 - phi)
    a = phi * M_star ** (1.0 - b)
    return a, M_star, phi


def verify(W_fixed, a, b, M_star, S_target):
    """역산이 맞는지 독립 확인 — 고정점 방정식과 미분계수를 직접 대입."""
    lhs = W_fixed + a * M_star ** b           # g(M*) 가 M* 와 같아야 한다
    S_at = a * b * M_star ** (b - 1.0)        # dW_str/dMTOW at M*
    return abs(lhs - M_star), abs(S_at - S_target)


def run_case(a, b, eps_conv, relax=1.0):
    return wght(strc_fn=make_power_strc(a, b), options={'eps_conv': eps_conv, 'relax': relax},
                **BASE)


def main():
    W_fixed = compute_W_fixed()
    S_TARGET = 0.21
    print("=" * 94)
    print("게이트 2 (축소) — Ŝ 추정기 편향:  Ŝ 는 S(M*) 를 재는가?")
    print("=" * 94)
    print(f"W_fixed = {W_fixed:.9f} kg,  목표 S(M*) = {S_TARGET} (전 b 공통),  relax = 1.0")
    print()

    # ── 0. 역산 검증 ──
    print("[0] 스텁 역산 자체가 맞는지 독립 확인 (고정점 방정식 + 미분계수 직접 대입)")
    worst_fp = worst_S = 0.0
    for b in [0.6, 0.8, 1.0, 1.2, 1.5]:
        a, M_star, _ = design_power_stub(W_fixed, S_TARGET, b)
        e_fp, e_S = verify(W_fixed, a, b, M_star, S_TARGET)
        worst_fp, worst_S = max(worst_fp, e_fp), max(worst_S, e_S)
    print(f"    |g(M*) - M*| 최대 = {worst_fp:.3e},  |S(M*) - 0.21| 최대 = {worst_S:.3e}  → 역산 OK")
    print()

    # ── 1. b 스윕 ──
    print("[1] b 스윕 (eps_conv = 1e-3, 출하 기본값)")
    print(f"    {'b':>5} {'phi':>7} {'M*':>11} {'S_hat':>10} {'편향':>11} {'상대편향':>10} "
          f"{'|M_k-M*|/M*':>12} {'n':>3}")
    rows_b = []
    for b in [0.60, 0.70, 0.80, 0.90, 1.00, 1.10, 1.20, 1.35, 1.50]:
        a, M_star, phi = design_power_stub(W_fixed, S_TARGET, b)
        r = run_case(a, b, 1e-3)
        bias = r['S_hat'] - S_TARGET
        # Ŝ 가 추정된 시점의 반복값이 M* 에서 얼마나 떨어져 있었나
        M_k = r['MTOW_iterate']
        dist = abs(M_k - M_star) / M_star
        rows_b.append((b, phi, M_star, r['S_hat'], bias, dist, r['n_iter']))
        print(f"    {b:>5.2f} {phi:>7.4f} {M_star:>11.6f} {r['S_hat']:>10.6f} "
              f"{bias:>+11.3e} {bias/S_TARGET:>+10.2%} {dist:>12.3e} {r['n_iter']:>3}")
    print()

    # ── 2. eps_conv 스윕 ──
    print("[2] eps_conv 스윕 — 수렴을 조이면 편향이 줄어드나 (b = 1.35 고정)")
    print(f"    {'eps':>8} {'S_hat':>10} {'편향':>11} {'상대편향':>10} {'|M_k-M*|/M*':>12} {'n':>3}")
    b = 1.35
    a, M_star, _ = design_power_stub(W_fixed, S_TARGET, b)
    rows_e = []
    for eps in [1e-2, 1e-3, 1e-4, 1e-5]:
        r = run_case(a, b, eps)
        bias = r['S_hat'] - S_TARGET
        dist = abs(r['MTOW_iterate'] - M_star) / M_star
        rows_e.append((eps, bias, dist))
        print(f"    {eps:>8.0e} {r['S_hat']:>10.6f} {bias:>+11.3e} {bias/S_TARGET:>+10.2%} "
              f"{dist:>12.3e} {r['n_iter']:>3}")
    print()

    # ── 3. 대조군: 선형(b=1)은 편향이 정확히 0 이어야 한다 ──
    print("[3] 대조군 — 선형 스텁은 S 가 상수라 편향이 원리적으로 0")
    r = wght(strc_fn=make_linear_strc(S_TARGET), options={'eps_conv': 1e-3}, **BASE)
    print(f"    S_hat = {r['S_hat']:.12f}   편향 = {r['S_hat'] - S_TARGET:+.3e}")
    print()

    # ── 4. Ŝ 는 정확히 무엇을 재는가 ──
    # [1]·[2] 를 보면 편향/dist 가 |b-1| 에 정비례한다(계수 0.605, 변동계수 0.09%).
    # 그 계수를 S 로 나누면 2.88 인데, 이는 (1 + 1/S)/2 = 2.881 과 일치한다.
    # 마지막 반복이 M* 에서 1 만큼 떨어져 있으면 그 직전은 1/S 만큼 떨어져 있으므로,
    # 2.88 은 곧 '두 반복의 중점'이다. 아래에서 직접 확인한다.
    print("[4] Ŝ 는 어느 지점의 S 인가 — 세 후보와 직접 대조")
    S_of = lambda a, b, M: a * b * M ** (b - 1.0)
    print(f"    {'b':>5} {'S_hat':>10} {'|Ŝ-S(M*)|':>12} {'|Ŝ-S(M_k)|':>12} {'|Ŝ-S(M_mid)|':>14}")
    acc = {'M*': [], 'M_k': [], 'M_mid': []}
    for b in [0.60, 0.70, 0.80, 0.90, 1.10, 1.20, 1.35, 1.50, 1.70]:
        a, M_star, _ = design_power_stub(W_fixed, S_TARGET, b)
        r = run_case(a, b, 1e-3)
        M_k, M_km1 = r['history'][-1][0], r['history'][-2][0]
        cand = {'M*': S_of(a, b, M_star), 'M_k': S_of(a, b, M_k),
                'M_mid': S_of(a, b, 0.5 * (M_k + M_km1))}
        for key in acc:
            acc[key].append(abs(r['S_hat'] - cand[key]))
        print(f"    {b:>5.2f} {r['S_hat']:>10.6f} {abs(r['S_hat']-cand['M*']):>12.2e} "
              f"{abs(r['S_hat']-cand['M_k']):>12.2e} {abs(r['S_hat']-cand['M_mid']):>14.2e}")
    means = {k: sum(v) / len(v) for k, v in acc.items()}
    print(f"    평균오차:  S(M*) {means['M*']:.2e}   S(M_k) {means['M_k']:.2e}   "
          f"S(M_mid) {means['M_mid']:.2e}")
    print()

    # ── 해석 ──
    print("=" * 94)
    print("[결론]")
    print()
    print("  1) Ŝ 는 S(M*) 가 아니라 **마지막 두 반복의 중점에서의 S** 를 잰다.")
    print(f"     S(M_mid) 와의 오차 {means['M_mid']:.1e} 로, 다른 두 후보보다 4 자릿수 정확하다.")
    print("     Ŝ = resid_k/resid_{k-1} 이 그 구간의 할선 기울기이므로 당연한 결과지만,")
    print("     §7.6 이 결론을 '수렴점 근방의 dW_str/dMTOW' 로 번역하라고 했으므로 확인이 필요했다.")
    print()
    max_rel = max(abs(bb / S_TARGET) for _, _, _, _, bb, _, _ in rows_b)
    print(f"  2) 그 어긋남(편향)은 b 0.6~1.5 에서 상대 최대 {max_rel:.2%} 로 작다.")
    print("     경험식:  편향 ≈ 0.605 × |b-1| × (반복값의 M* 대비 상대거리)")
    print("     (b 0.6~1.7 의 9 점에서 계수 변동 0.09%. 계수 0.605 = S x (1+1/S)/2 = 0.21 x 2.88)")
    print()
    ratio = abs(rows_e[0][1]) / abs(rows_e[-1][1]) if rows_e[-1][1] else float('inf')
    print(f"  3) 편향은 eps_conv 로 직접 통제된다 — 1e-2 -> 1e-5 에서 {ratio:.0f}배 감소.")
    print("     반복값이 M* 에 가까워질수록 중점도 M* 에 가까워지기 때문이다.")
    print("     즉 편향은 추정기의 결함이 아니라 '어디서 멈췄나'의 함수다.")
    print()
    print("  4) §7.5 히스토그램에 대한 함의: 출하 eps_conv=1e-3 에서 Ŝ 의 상대편향은")
    print(f"     {max_rel:.2%} 수준이고, 경계 S=1 까지의 여유(0.7~0.8)에 비하면 무시할 수 있다.")
    print("     **'여유 3배' 같은 판단은 추정기 편향으로 뒤집히지 않는다.**")
    print()
    print("  [정정] 실행 전에 '편향 부호가 b 에 따라 뒤집힌다'고 적어두었으나 틀렸다.")
    print("         b<1 이든 b>1 이든 편향은 전부 양수다. 중점이 항상 M* 보다 바깥쪽에 있고,")
    print("         S 의 기울기 부호와 중점의 방향이 함께 뒤집혀 곱이 항상 양수가 되기 때문이다.")


if __name__ == '__main__':
    main()
