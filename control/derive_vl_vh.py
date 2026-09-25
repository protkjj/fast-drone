"""VL·VH 도출 — 논문 v5.3 §5.8의 "사전 검증"을 코드로 수행한다.

논문 §5.5는 순항 속도 후보를 VL=20 m/s, VH=40 m/s로 적어 두고, §5.8은
이렇게 단서를 붙인다.

    "확인 단계의 기준 저속 VL과 고속 VH는 모델 맵과 공통 트림의 사전 검증으로
     고정한다. 시작 후보는 각각 20 m/s와 40 m/s이며, 해당 모델에서 유효한
     트림이 존재하지 않으면 제어기별 결과를 보기 전에 낮춘다. 이때 축소
     이유와 최종값을 기록한다."

이 모듈이 그 "사전 검증"이다. 제어기 결과를 보기 전에 돌려야 하며(그래야
값을 결과에 맞춰 고르는 일이 없다), 출력은 results/VL_VH_DERIVATION.md 에
남긴다.

판정 기준 — '유효한 트림'은 두 조건을 모두 만족해야 한다.
  1. find_trim 이 수렴한다.
  2. 그 해의 로터 회전수가 [n_min, n_max] 안에 있다.
두 번째를 빼면 수렴했지만 물리적으로 불가능한 점이 격자에 들어간다
([[lqr-fallback-thrust-axis-gap]] 의 ScheduledLQR 오염 버그가 그 사례다).

VH 는 명목값만으로 정하면 안 된다. 표7의 섭동은 플랜트에만 적용되므로
(제어기 명목값은 고정) 섭동된 기체가 그 속도에서 수평 비행을 유지할 수
있어야 한다. 따라서 **모든 섭동에서 트림이 존재하는 속도**를 상한으로 본다.

실행:  python3 -m control.derive_vl_vh
"""
import numpy as np

from control.trim import find_trim
from control.vehicle_params import load_selected_params

# 논문 §5.5의 시작 후보.
VL_CANDIDATE = 20.0
VH_CANDIDATE = 40.0

# 연속법 전진 폭과 이분탐색 횟수. 0.25 m/s 는 경계를 놓치지 않으면서
# 60 m/s 까지 훑는 데 충분하고, 18회 이분탐색이면 0.25/2^18 ≈ 1e-6 m/s.
STEP = 0.25
BISECT = 18


def trim_is_valid(params, trim):
    """수렴했고 회전수가 실제 구동 범위 안인지."""
    if not trim['converged']:
        return False
    n = trim['state'][13:17]
    return (n.min() >= params['n_min'] - 1e-9
            and n.max() <= params['n_max'] + 1e-9)


def level_trim_ceiling(params, search_to=60.0):
    """수평 트림이 존재하는 최고 속도.

    냉시동 fsolve 는 고속에서 엉뚱한 가지로 빠지므로(trim.py 가 이미 권고)
    호버부터 연속법으로 올라간다. 직전 해를 초기추측으로 넘기면 같은 가지를
    따라가므로, 경계가 '해가 없어서'인지 '초기추측이 나빠서'인지 헷갈리지 않는다.
    """
    guess, ceiling, V = None, 0.0, 0.0
    while V < search_to:
        nxt = V + STEP
        trim = find_trim(params, nxt, guess=guess, quiet=True)
        if not trim_is_valid(params, trim):
            break
        guess, ceiling, V = trim['guess'], nxt, nxt

    lo, hi = ceiling, ceiling + STEP
    for _ in range(BISECT):
        mid = 0.5*(lo + hi)
        trim = find_trim(params, mid, guess=guess, quiet=True)
        if trim_is_valid(params, trim):
            lo, guess = mid, trim['guess']
        else:
            hi = mid
    return lo


def perturbations(base):
    """표7의 '모델' 구분 섭동. 외란·센서는 트림 존재성과 무관하다.

    표7은 질량 ±30%, 추력계수 ±30%, 동체 공력 계수 +50%/+100%,
    무게중심 편차 팔 길이의 10% 를 '한 번에 하나씩 플랜트에만' 적용한다.
    동체 공력은 축방향·법선방향 계수를 함께 올린다(식 8 의 두 항).
    """
    arm = base['arm_length']
    return [
        ('명목', {}),
        ('질량 +30%', {'mass': base['mass']*1.30}),
        ('질량 -30%', {'mass': base['mass']*0.70}),
        ('추력계수 +30%', {'k_T': base['k_T']*1.30}),
        ('추력계수 -30%', {'k_T': base['k_T']*0.70}),
        ('동체공력 +50%', {'C_Na': base['C_Na']*1.50, 'C_A0': base['C_A0']*1.50}),
        ('동체공력 +100%', {'C_Na': base['C_Na']*2.00, 'C_A0': base['C_A0']*2.00}),
        ('무게중심 +10% 팔', {'x_cp': base['x_cp'] + 0.10*arm}),
        ('무게중심 -10% 팔', {'x_cp': base['x_cp'] - 0.10*arm}),
    ]


def main():
    base = load_selected_params()
    if base is None:
        raise SystemExit('선정 프로파일을 읽을 수 없다 — research/profiles/selected.json 확인')

    print(f"선정 프로파일 {base['source_id']}  mass={base['mass']:.6f} kg  "
          f"n_max={base['n_max']:.1f} rad/s  추력축={base['thrust_axis']}")
    print(f"논문 §5.5 시작 후보: VL={VL_CANDIDATE:.0f}, VH={VH_CANDIDATE:.0f} m/s\n")

    print(f"{'섭동':>18} {'트림 상한[m/s]':>14} {'VL후보':>7} {'VH후보':>7}")
    ceilings = {}
    for name, override in perturbations(base):
        params = dict(base)
        params.update(override)
        ceilings[name] = level_trim_ceiling(params)
        print(f"{name:>18} {ceilings[name]:14.3f} "
              f"{'O' if ceilings[name] >= VL_CANDIDATE else 'X':>7} "
              f"{'O' if ceilings[name] >= VH_CANDIDATE else 'X':>7}")

    nominal = ceilings['명목']
    worst_name = min(ceilings, key=ceilings.get)
    worst = ceilings[worst_name]
    print(f"\n명목 상한        {nominal:.3f} m/s")
    print(f"최악 섭동        {worst_name} → {worst:.3f} m/s")
    print(f"시작 후보 판정   VL={VL_CANDIDATE:.0f}: "
          f"{'가능' if nominal >= VL_CANDIDATE else '불가 — 명목에서도 트림 없음'}")
    print(f"                 VH={VH_CANDIDATE:.0f}: "
          f"{'가능' if nominal >= VH_CANDIDATE else '불가 — 명목에서도 트림 없음'}")

    print(f"\n{'후보':>6} {'모든 섭동 통과':>14} {'최악 대비 여유':>14}")
    for cand in (8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0):
        print(f"{cand:6.1f} {'O' if worst >= cand else 'X':>14} "
              f"{100*(worst - cand)/cand:+13.1f}%")

    # k_T 불변성은 한계의 정체를 알려 준다 — 아래 문서에서 해석한다.
    print(f"\n추력계수 ±30% 로 상한이 변하는가: "
          f"{abs(ceilings['추력계수 +30%'] - ceilings['추력계수 -30%']) > 1e-3}")


if __name__ == '__main__':
    main()
