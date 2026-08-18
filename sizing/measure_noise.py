"""
게이트 1 실측 도구 — resid_floor 와 양자화 크기
===============================================

실행: python3 -m sizing.measure_noise   (저장소 루트에서)

[왜 실측인가 — §2.6]
rev.3 의 resid_floor = 1e-6 x MTOW 는 근거 없는 수다. 배정도 반올림은 상대 1e-16 수준이라
10 자릿수 위이고, 정작 잡아야 할 'strc_fn 자체의 수치 노이즈'보다 큰지 작은지 모른다.

[측정 원리]
resid = W_fixed + W_str(MTOW) - MTOW 에서 W_fixed 와 MTOW 는 정확한 값이다.
따라서 resid 의 불확실성은 곧 W_str 의 불확실성이다. MTOW 를 아주 좁게 훑으면서
W_str 의 응답을 보면 두 가지가 한 번에 나온다:

  · 노이즈  — 국소 선형 추세를 뺀 잔차의 산포. resid_floor 는 이보다 커야 한다.
  · 양자화  — 고유값이 이산 격자를 이루면 그 최소 간격. limit_cycle 판정용 (§3.2).

[해석 주의]
매끄러운 모델이면 노이즈≈0, 양자화 없음으로 나온다. 그건 '측정 실패'가 아니라
'그 모델은 하한 가드가 거의 필요 없다'는 결과다. 실제 STRC(슬라이서 회귀)는
계단형일 수 있으므로 실물이 오면 반드시 다시 잰다.
"""
from __future__ import annotations

import numpy as np

from sizing.strc_stub import (make_linear_strc, make_quantized_strc,
                              make_power_strc, make_noisy_strc)


def probe(strc_fn, MTOW_center, rel_window=1e-3, n=401):
    """MTOW_center 주변 +-rel_window 를 n 점으로 훑어 W_str 응답을 얻는다.

    창을 좁게(기본 0.1%) 잡는 이유: 넓으면 모델의 '진짜 기울기'가 섞여 들어와
    노이즈와 구분되지 않는다. 수렴 근방에서 반복이 실제로 움직이는 폭이 이 정도다.
    """
    xs = MTOW_center * (1.0 + rel_window * np.linspace(-1.0, 1.0, n))
    ys = np.array([float(strc_fn(float(x))['W_str']) for x in xs])
    return xs, ys


def measure_quantum(ys, rel_tol=1e-12):
    """고유값이 이산 격자면 최소 간격을 돌려준다. 연속이면 None.

    판정: 표본 수 대비 고유값이 충분히 적으면 '계단형'으로 본다.
          매끄러운 함수는 표본마다 값이 달라 고유값 수 ≈ 표본 수가 된다.
    """
    scale = max(1.0, float(np.max(np.abs(ys))))
    uniq = np.unique(np.round(ys / (scale * rel_tol)))* scale * rel_tol
    if len(uniq) >= len(ys) * 0.5:          # 절반 이상이 서로 다르면 연속으로 본다
        return None, len(uniq)
    if len(uniq) < 2:
        return 0.0, len(uniq)               # 창 안에서 완전히 평평 (한 계단 안)
    return float(np.min(np.diff(uniq))), len(uniq)


def measure_noise(xs, ys):
    """국소 선형 추세를 뺀 잔차의 표준편차 = 수치 노이즈.

    1차 다항 적합을 쓰는 이유: 좁은 창 안에서 매끄러운 모델은 거의 직선이므로,
    직선을 빼고 남는 것이 곧 '모델이 만들어내는 흔들림'이다.
    """
    slope, intercept = np.polyfit(xs, ys, 1)
    resid = ys - (slope * xs + intercept)
    return float(np.std(resid)), float(slope)


def measure(strc_fn, MTOW_center, name='', rel_window=1e-3, n=401, k_sigma=3.0,
            max_widen=8):
    """한 모델에 대한 전체 측정. resid_floor 권고값까지 낸다.

    창이 양자보다 좁으면 표본이 전부 한 계단 안에 들어가 고유값이 1개가 되고,
    그러면 '양자 크기 0' 이라는 무의미한 답이 나온다. 그 상태의 권고값을 그대로 쓰면
    하한 가드가 통째로 꺼진다. 그래서 고유값이 2개 이상 보일 때까지 창을 배로 넓힌다.
    """
    widened = 0
    while True:
        xs, ys = probe(strc_fn, MTOW_center, rel_window, n)
        quantum, n_uniq = measure_quantum(ys)
        if n_uniq >= 2 or widened >= max_widen:
            break
        rel_window *= 2.0          # 한 계단 안에 갇혔다 — 창을 넓힌다
        widened += 1

    noise, slope = measure_noise(xs, ys)
    staircase = quantum is not None and quantum > 0.0

    # resid_floor 는 '의미 없는 차이를 신호로 착각하지 않을' 크기여야 한다.
    #  · 계단형이면 양자의 절반. 그보다 작은 resid 차이는 물리적으로 존재하지 않는다.
    #    이때 선형적합 잔차는 '노이즈'가 아니라 계단 자체이므로 floor 산정에 쓰지 않는다.
    #  · 매끄러우면 노이즈의 k_sigma 배.
    if staircase:
        floor_abs = quantum * 0.5
    else:
        floor_abs = k_sigma * noise
    floor_rel = floor_abs / MTOW_center

    return {
        'name': name, 'MTOW_center': MTOW_center,
        'slope_local': slope,          # = 국소 dW_str/dMTOW = S. §7.6 이 넘기라는 그 양
        'noise_abs': noise, 'staircase': staircase,
        'quantum': quantum, 'n_unique': n_uniq,
        'rel_window': rel_window, 'widened': widened,
        'floor_abs': floor_abs, 'floor_rel': floor_rel,
    }


def _report(rows):
    print(f"{'모델':<22} {'국소 S':>8} {'산포':>10} {'양자':>10} {'고유':>5} "
          f"{'창':>9} {'권고 floor_rel':>15}")
    print('-' * 88)
    for r in rows:
        q = '연속' if not r['staircase'] else f"{r['quantum']:.3e}"
        w = f"{r['rel_window']:.1e}" + ('*' if r['widened'] else ' ')
        print(f"{r['name']:<22} {r['slope_local']:>8.4f} {r['noise_abs']:>10.3e} "
              f"{q:>10} {r['n_unique']:>5} {w:>9} {r['floor_rel']:>15.3e}")
    if any(r['widened'] for r in rows):
        print("  * 창이 양자보다 좁아 자동으로 넓힌 행 (한 계단 안에 갇히면 양자를 못 잰다)")


if __name__ == '__main__':
    # 실제 STRC 가 없으므로 스텁으로 방법을 시연한다.
    # 실물이 오면 make_* 자리에 실제 strc_fn 을 넣고 그대로 돌리면 된다.
    MTOW0 = 6.87        # 선형 스텁 S=0.21 에서의 수렴점 근방

    print("=" * 82)
    print("게이트 1 실측 — resid_floor / 양자화 크기  (스텁 시연)")
    print("=" * 82)
    print(f"측정 중심 MTOW = {MTOW0} kg,  창 = +-0.1%,  표본 401점,  권고 = 3-sigma")
    print()

    rows = [
        measure(make_linear_strc(0.21),            MTOW0, '선형 S=0.21'),
        measure(make_power_strc(0.35, 0.92),       MTOW0, '멱함수 a=.35 b=.92'),
        measure(make_quantized_strc(0.21, 0.02),   MTOW0, '양자화 q=0.02'),
        measure(make_quantized_strc(0.21, 0.002),  MTOW0, '양자화 q=0.002'),
        measure(make_noisy_strc(make_linear_strc(0.21), 1e-9),  MTOW0, '선형+노이즈 1e-9'),
        measure(make_noisy_strc(make_linear_strc(0.21), 1e-6),  MTOW0, '선형+노이즈 1e-6'),
    ]
    _report(rows)

    print()
    print("[읽는 법]")
    print("  · 매끄러운 모델(선형·멱함수)은 노이즈≈0, 양자 None → 하한 가드가 사실상 불필요.")
    print("    기본값 1e-6 은 이 경우 '넉넉한 안전값'이지 측정값이 아니다.")
    print("  · 양자화 모델은 노이즈가 아니라 양자가 floor 를 지배한다.")
    print("    양자 크기 미만의 resid 차이는 물리적 의미가 없으므로 그 절반을 하한으로 권고한다.")
    print("  · 노이즈가 있는 모델은 3-sigma 가 곧 floor 다. 이보다 작게 잡으면 r_hat 이")
    print("    노이즈만 재고, S_hat 이 1 근처에도 1 이상에도 찍혀 발산 오판이 난다 (§2.6).")
    print()
    print("[실물 STRC 가 오면]")
    print("  measure(실제_strc_fn, 예상_MTOW, '실제 STRC') 를 돌리고,")
    print("  나온 floor_rel 을 options['resid_floor'] 로 넣는다. 그게 §5-14 의 확정 절차다.")
