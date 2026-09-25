"""판별 실험 회귀 테스트 — kj 지시(2026-09-25 저녁, "가설을 낮춘다").

짧은 시험(정지 트림+미소섭동)이 80/85 m/s에서 실패하고 65초 미션(연속
궤적)은 통과하는 이유로 두 가설이 경합했다:
  H-시나리오: 시나리오 종류(정지 대 지나가기) 자체가 변수
  H-웜스타트이력: 짧은시험은 고속트림 콜드스타트, 미션은 호버부터
                 웜스타트가 이어짐 — 그 차이가 진짜 변수

kj 설계대로 (1) 짧은 시험의 웜스타트를 트림 상태·트림 입력으로 채우고
(2) 쿼터니언 정규화 나눗셈 특이점을 eps로 고친(V13/M17/F13 전체 적용,
`control/hybrid_comparison.py`·`nmpc.py`·`nmpc_f13.py`) 뒤 재실행했다.
**결과(당시): 실패 양상이 바뀌지 않았다** — 두 웜스타트 조건이 z_RMSE·
v_RMSE·포화율·정지사유까지 사실상 동일했다. kj 판정 규칙("1)+2) 후에도
실패하면 시나리오 종류로 판정")에 따라 **H-시나리오가 지지, H-웜스타트
이력(이 단순한 형태)은 기각**이었다.

**🔴 보류(2026-09-25 밤)**: 이 판정 이후 프로펠러 모델 자체가 선형
fac 모델(APC 실제 대비 트림에서 43~69%만 추력을 줌)에서 팀원 APC 곡선
기반으로 바뀌었다(`control/dynamics.py`·`hybrid_comparison.py`,
results/SOLVER_FAILURE_REPORT_2026-09-25.md 참고). 그 결과 80m/s 짧은
시험이 **완주하는 쪽으로 바뀌어** 아래 두 테스트가 깨졌다 — 이 판정
자체가 옛(틀린) 프로펠러 모델 기준이었다는 뜻이다. kj 지시: 이번
세션은 프로펠러 모델 교체까지만 하고 판정·시험 재실행·결론은 다음
세션(arena-completion)이 맡는다 — 그래서 여기서 새 판정을 내리지
않고 **재평가 보류로 스킵**만 해 둔다. 85m/s는 여전히 기존 결과대로
실패해 아직 스킵하지 않았다(그 자체가 재확인 대상).
"""
import numpy as np
import pytest

from control.kh_adapter import kh_native_params, build_controller_params
from control.kh_repro import run_case, make_our_split, make_our_split_trim_warmstart

NATIVE = kh_native_params()
CP = build_controller_params(NATIVE)


def test_trim_warmstart_injection_actually_changes_w0():
    """실험 자체가 유효한지 확인 — 트림 웜스타트가 정말로 기본값과
    다른 w0를 만드는지(안 그러면 '차이 없음' 결과가 실험 무효 때문일
    수 있다)."""
    _, nmpc_default = make_our_split(CP, 80.0, 20.0)
    _, nmpc_trim = make_our_split_trim_warmstart(CP, NATIVE, 80.0, 20.0)
    assert nmpc_default._cold_start is True
    assert nmpc_trim._cold_start is False
    # X_0 초기추측(첫 13개)이 서로 달라야 한다 — 기본은 호버형(v=0),
    # 트림판은 실제 순항 속도(80m/s)를 담고 있어야 한다.
    assert not np.allclose(nmpc_default.w0[0:13], nmpc_trim.w0[0:13])
    assert nmpc_trim.w0[3] == pytest.approx(80.0)   # vx = 순항속도


@pytest.mark.parametrize('speed', [
    pytest.param(80.0, marks=pytest.mark.skip(
        reason="프로펠러 곡선 수정(2026-09-25 밤) 이후 80m/s가 완주하는 "
               "쪽으로 바뀜 — 이 판정은 옛 프로펠러 모델 기준. 재평가는 "
               "arena-completion 세션 담당, 여기서 새로 판정하지 않음.")),
    85.0,
])
def test_trim_warmstart_does_not_rescue_short_test(speed):
    """핵심 판별 결과 — 트림 웜스타트(+eps 정칙화는 코드에 항상 적용됨)를
    줘도 80/85m/s 짧은 시험(nominal)은 여전히 실패한다."""
    def factory(spd, z):
        return make_our_split_trim_warmstart(CP, NATIVE, spd, z)

    result = run_case(NATIVE, factory, 'V13(트림웜스타트)', speed, 'nominal')
    assert result['completed'] is False
    assert result['saturation_fraction'] > 0.5   # 여전히 심하게 포화


@pytest.mark.skip(
    reason="프로펠러 곡선 수정(2026-09-25 밤) 이후 80m/s가 완주하는 쪽으로 "
           "바뀜 — 이 판정은 옛 프로펠러 모델 기준. 재평가는 arena-completion "
           "세션 담당, 여기서 새로 판정하지 않음.")
def test_trim_warmstart_and_default_fail_the_same_way():
    """대조군(기본 웜스타트)과 실험군(트림 웜스타트)의 실패 양상이
    사실상 같다는 것 자체를 고정한다 — "웜스타트를 고치면 나아진다"는
    관찰이 나오면 이 회귀가 깨져야 한다."""
    def default_factory(spd, z):
        return make_our_split(CP, spd, z)

    def trim_factory(spd, z):
        return make_our_split_trim_warmstart(CP, NATIVE, spd, z)

    r_default = run_case(NATIVE, default_factory, 'V13(기본)', 80.0, 'nominal')
    r_trim = run_case(NATIVE, trim_factory, 'V13(트림)', 80.0, 'nominal')
    assert r_default['completed'] == r_trim['completed'] == False
    assert r_default['stop_reason'] == r_trim['stop_reason']
    np.testing.assert_allclose(r_default['saturation_fraction'],
                               r_trim['saturation_fraction'], atol=0.05)
