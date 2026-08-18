"""
STRC 스텁 — WGHT 수렴 루프 검증용 가짜 구조 모듈
================================================

실제 STRC 는 게이트 0(인터페이스 확정) 대기 중이다. 그동안 WGHT 의 수렴 상태머신을
검증하려면 '정답을 아는' 구조 모델이 필요하다. 그게 이 파일이다.

[왜 선형 스텁이 기본인가 — §7.4]
멱함수 W_str = a*MTOW^b 는 b != 1 일 때 구조 비중 phi 가 반복 중에 변한다.
그러면 S 가 상수가 아니라서 '경계가 S=1 이다'라는 이론값과 대조할 수 없다.
선형 W_str = S*MTOW 는 S 가 기울기 그 자체이고 고정점이 해석적으로 나온다:

    MTOW* = W_fixed / (1 - S)

비선형성의 영향은 멱함수 스텁 몇 점으로 따로 본다.

[모든 스텁은 순수하다]
파라미터를 클로저로 잡을 뿐 상태를 갱신하지 않는다. WGHT 의 무상태성은
주입 함수가 무상태일 때만 성립하기 때문이다 (wght.py §0 주석 참조).
그 원칙을 어기면 어떻게 되는지 보이려고 make_impure_strc 만 예외로 둔다 — 테스트 전용이다.
"""
from __future__ import annotations

import math


def _mass_props(W_str, l_body, x_nose_to_mid):
    """스텁의 구조 질량 특성.

    구조를 길이 l_body 인 균일 봉으로 근사한다. 자기 cg 기준 J = m*L^2/12 이다.
    실제 STRC 가 오면 통째로 대체된다 — 여기서 중요한 건 값의 정확도가 아니라
    'x_cg_str·J_yy_str 이 자기 cg 기준으로 온다'는 §5-D 규약을 흉내내는 것이다.
    """
    return x_nose_to_mid, W_str * l_body ** 2 / 12.0


def _print_setting(W_str, quantum=None):
    """슬라이서 설정. 레이어 수·둘레 수는 정수다 — 평균이 불가능한 양이라는 점이 핵심."""
    n_layer = int(round(W_str * 40.0))
    return {'n_layer': max(1, n_layer), 'n_perimeter': 3, 'infill': 0.25}


def make_linear_strc(S, l_body=1.0, x_cg_str=0.5, margin=1.5):
    """W_str = S * MTOW.  고정점 MTOW* = W_fixed/(1-S) 가 해석적으로 나온다.

    S >= 1 이면 고정점이 음수 = 애초에 존재하지 않는다. 그래서 발산은 수치 문제가
    아니라 구조 문제이고, 경계는 relax 와 무관하게 정확히 S=1 이다 (§7.2).
    """
    def strc_fn(MTOW):
        W_str = S * MTOW
        _, J = _mass_props(W_str, l_body, x_cg_str)
        return {'W_str': W_str, 'x_cg_str': x_cg_str, 'J_yy_str': J,
                'm_print': W_str * 1.05, 'margin_str': margin,
                'print_setting': _print_setting(W_str)}
    return strc_fn


def make_power_strc(a, b, l_body=1.0, x_cg_str=0.5, margin=1.5):
    """W_str = a * MTOW^b.  비선형성 확인용 (§7.4 '별도 확인').

    국소 민감도는 S(MTOW) = dW_str/dMTOW = a*b*MTOW^(b-1) 로 MTOW 에 의존한다.
    즉 반복 중에 S 가 변하므로, 수렴 후 S_hat 은 '수렴점 근방의 국소 미분계수'다.
    §7.6 이 결론을 국소 미분계수로 번역해 넘기라고 한 이유가 이것이다.
    """
    def strc_fn(MTOW):
        W_str = a * MTOW ** b
        _, J = _mass_props(W_str, l_body, x_cg_str)
        return {'W_str': W_str, 'x_cg_str': x_cg_str, 'J_yy_str': J,
                'm_print': W_str * 1.05, 'margin_str': margin,
                'print_setting': _print_setting(W_str)}
    return strc_fn


def make_quantized_strc(S, quantum, l_body=1.0, x_cg_str=0.5, margin=1.5):
    """W_str = quantum * round(S*MTOW / quantum).  계단형 출력.

    실제 STRC 는 슬라이서 회귀라 레이어 수·인필·둘레 수가 정수다. 그래서 출력이
    계단형일 수 있고, 그러면 MTOW 가 두 값 사이를 무한 진동한다:

        r_hat -> -1,  |resid| 는 양자 크기에서 멈춤,  무한 지속

    이것을 'diverged' 로 분류하면 특정 형상대의 설계점이 통째로, 조용히 OEC 후보에서
    빠진다. limit_cycle 을 별도 status 로 둔 이유다 (§3.2).
    """
    if quantum <= 0:
        raise ValueError("quantum 은 양수여야 합니다.")

    def strc_fn(MTOW):
        W_str = quantum * round(S * MTOW / quantum)
        _, J = _mass_props(W_str, l_body, x_cg_str)
        return {'W_str': W_str, 'x_cg_str': x_cg_str, 'J_yy_str': J,
                'm_print': W_str * 1.05, 'margin_str': margin,
                'print_setting': _print_setting(W_str)}
    return strc_fn


def make_noisy_strc(base_fn, rel_noise, seed=0):
    """base_fn 출력에 재현 가능한 '수치 노이즈'를 얹는다 — resid_floor 실측 연습용.

    난수를 쓰지 않는다. MTOW 값 자체를 해시해 결정론적으로 흔든다.
    난수를 쓰면 같은 MTOW 에 다른 값이 나와 '순수 함수' 규약을 스텁이 스스로 어기게 되고,
    그러면 재현이 안 돼 실측 도구로 쓸 수 없다.
    """
    def strc_fn(MTOW):
        out = dict(base_fn(MTOW))
        h = math.sin((MTOW * 1e6 + seed) % 6.283185307179586)   # 결정론적 의사 노이즈
        out['W_str'] = out['W_str'] * (1.0 + rel_noise * h)
        return out
    return strc_fn


def make_impure_strc(S):
    """상태를 들고 있는 '나쁜' 스텁 — options['check_strc_purity'] 가 이걸 잡아야 한다.

    실제로 이런 코드가 생기는 경로는 warm start 캐시다. 같은 팀이 이미 겪었다
    (PROJECT_REPORT 버그 1-3). 테스트 전용이며 다른 곳에서 쓰지 말 것.
    """
    state = {'n': 0}

    def strc_fn(MTOW):
        state['n'] += 1
        W_str = S * MTOW * (1.0 + 1e-3 * state['n'])   # 호출 횟수에 따라 답이 변한다
        return {'W_str': W_str, 'x_cg_str': 0.5, 'J_yy_str': W_str / 12.0,
                'm_print': W_str, 'margin_str': 1.5,
                'print_setting': {'n_layer': 10, 'n_perimeter': 3, 'infill': 0.25}}
    return strc_fn
