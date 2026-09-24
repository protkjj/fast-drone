"""야간 단계2 — 고속가지(하이파이) 섭동 강건성 + V_L·V_H 후보 (PILOT).

표7 섭동을 하이파이 트림 솔버(control/trim_hifi.py)에 적용해 85 m/s 트림이
섭동 하에서도 존재하는지, 재개방 경계가 섭동마다 어디로 움직이는지 본다.
저속가지도 같은 방법으로 재확인해 V_L 후보를 만든다.

⚠ 시간 제약(야간 자율작업, 단계1 초과분 반영)으로 재개방 경계는 1 m/s
해상도 이분탐색(정밀 아님, ±0.5 m/s)만 했다 — 표8 본시험 수준 정밀도가
아니다. 모멘트 여유(돌풍 추정)는 §4.7 rν/εν 프레임이 아니라 **정적** 여유
(트림점에서 낼 수 있는 최대 추가 모멘트 대비 여유)로 근사했다 — 동적 여유가
아니다. 실행: python3 -m control.night_perturbation_robustness
"""
import numpy as np

from control.trim_hifi import find_trim_hifi
from control.vehicle_params import load_selected_params

P = load_selected_params()

# 표7 '모델' 구분 섭동 (플랜트에만 적용 — 이 스크립트에선 트림 존재성만 보므로
# '플랜트'와 '제어기가 아는 모델'의 구분이 트림 계산 자체엔 의미 없다. 트림은
# 항상 실제 물리 기준으로 풀어야 한다).
ARM = P['arm_length']
PERTURBATIONS = [
    ('명목', {}),
    ('질량 +30%', {'mass': P['mass']*1.30}),
    ('질량 -30%', {'mass': P['mass']*0.70}),
    ('추력계수 +30%', {'k_T': P['k_T']*1.30}),
    ('추력계수 -30%', {'k_T': P['k_T']*0.70}),
    ('동체공력 +50%', {'C_Na': P['C_Na']*1.50, 'C_A0': P['C_A0']*1.50}),
    ('동체공력 +100%', {'C_Na': P['C_Na']*2.00, 'C_A0': P['C_A0']*2.00}),
    ('무게중심 +10%팔', {'x_cp': P['x_cp'] + 0.10*ARM}),
    ('무게중심 -10%팔', {'x_cp': P['x_cp'] - 0.10*ARM}),
    ('공력감쇠 +50%', {'C_lp': P['C_lp']*1.50, 'C_mq': P['C_mq']*1.50}),
    ('공력감쇠 -50%', {'C_lp': P['C_lp']*0.50, 'C_mq': P['C_mq']*0.50}),
]


def exists_at(params, V, theta_hints=(0, 10, 20, 40, 60, 75, 80, 85, 87, 88, 89)):
    from control.trim_hifi import default_seeds
    seeds = default_seeds(params, theta_deg=theta_hints, n_scale=(0.8, 1.0, 1.2, 1.4, 1.6, 1.8))
    return find_trim_hifi(params, V, seeds=seeds)


def find_high_branch_boundary(params, lo=78.0, hi=90.0, step=1.0, tol=0.5):
    """대략적인 재개방 경계(±tol) — 정밀 이분탐색 아님(시간 제약)."""
    last_infeasible = None
    for V in np.arange(lo, hi + 0.01, step):
        r = exists_at(params, V)
        if not r['converged']:
            last_infeasible = V
        else:
            if last_infeasible is None:
                return V, r   # 이미 lo 에서부터 존재 — 경계가 lo 미만
            a, b = last_infeasible, V
            while b - a > tol:
                m = (a + b)/2
                if exists_at(params, m)['converged']:
                    b = m
                else:
                    a = m
            return b, exists_at(params, b)
    return None, None


def find_low_branch_boundary(params, lo=15.0, hi=22.0, step=0.5, tol=0.25):
    last_feasible = None
    for V in np.arange(lo, hi + 0.01, step):
        r = exists_at(params, V, theta_hints=(0, 5, 10, 20, 30, 40, 50, 60))
        if r['converged']:
            last_feasible = V
        elif last_feasible is not None:
            a, b = last_feasible, V
            while b - a > tol:
                m = (a + b)/2
                if exists_at(params, m, theta_hints=(0, 5, 10, 20, 30, 40, 50, 60))['converged']:
                    a = m
                else:
                    b = m
            return a
    return last_feasible


def moment_margin(params, trim_result):
    """트림점에서 로터가 추가로 낼 수 있는 최대 피치모멘트 대비, 실제 요구되는
    공력 모멘트 변화(측풍 등가)를 감당할 여유. 정적 근사 — 동적 응답 아님.
    """
    x = trim_result['state']
    n = x[13:17]
    arm = params['arm_length']/np.sqrt(2)
    # 남은 회전수 여유(작은 쪽)로 추가 낼 수 있는 차동추력의 대략적 상한.
    headroom_pair = min(params['n_max'] - n[0], n[0] - params['n_min'],
                        params['n_max'] - n[2], n[2] - params['n_min'])
    return headroom_pair, arm


def main():
    print("=" * 70)
    print("① 표7 섭동 하 85 m/s 트림 존재성 + 고속가지 재개방 경계(대략)")
    print(f"{'섭동':>16} {'85trim':>7} {'재개방(±0.5)':>13} {'n_front':>8} {'n_rear':>8}")
    rows = []
    for name, ov in PERTURBATIONS:
        params = dict(P)
        params.update(ov)
        r85 = exists_at(params, 85.0)
        boundary, r_b = find_high_branch_boundary(params)
        nf, nr = (r85['state'][13], r85['state'][15]) if r85['converged'] else (None, None)
        rows.append((name, r85['converged'], boundary, nf, nr))
        print(f"{name:>16} {'O' if r85['converged'] else 'X':>7} "
              f"{f'{boundary:.1f}' if boundary else '없음':>13} "
              f"{f'{nf:.1f}' if nf else '-':>8} {f'{nr:.1f}' if nr else '-':>8}")

    print("\n② 저속가지 경계(대략, ±0.25) — V_L 후보 근거")
    for name, ov in PERTURBATIONS:
        params = dict(P)
        params.update(ov)
        b = find_low_branch_boundary(params)
        print(f"{name:>16}  저속가지 상한 ≈ {b}")

    print("\n③ 85 m/s 명목 트림의 정적 모멘트 여유(근사) → 측풍 추정")
    r85 = exists_at(P, 85.0)
    headroom, arm = moment_margin(P, r85)
    # 여유 회전수 -> 대략적 추가추력 -> 모멘트로 환산(선형 근사, k_T 기준)
    n_ref = r85['state'][13]
    extra_thrust = P['k_T']*((n_ref+headroom)**2 - n_ref**2)
    extra_moment = arm*extra_thrust
    print(f"  회전수 여유(작은 쪽 기준) = {headroom:.1f} rad/s")
    print(f"  → 대략적 추가 모멘트 여유 ≈ {extra_moment:.4f} N·m (선형근사, 상한 아님)")
    print(f"  ⚠ 이건 정적 상한 근사다 — 논문 §4.7 rν/εν 유한시간 프레임과 다르다.")
    for gust in (5.0, 10.0, 15.0):
        angle_deg = np.degrees(np.arctan(gust/85.0))
        print(f"  측풍 {gust:.0f} m/s → 유속각 {angle_deg:.1f}° (85 m/s 대비)")


if __name__ == '__main__':
    main()
