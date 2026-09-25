"""kj 판별 실험(2026-09-25 저녁, "가설을 낮춘다" 지시) — 짧은 시험이
80/85 m/s에서 실패하고 65초 미션은 통과하는 이유가:

  H-시나리오: "정지해서 버티기"(짧은시험) vs "지나가기"(미션)가 근본적으로
              다른 문제라서
  H-웜스타트이력: 짧은시험은 고속 트림에서 콜드스타트, 미션은 호버부터
                 웜스타트가 이어져서

인지 가르는 판별 실험. kj가 설계한 3단계:
  1) 짧은 시험의 NMPC 초기 웜스타트를 트림 상태·트림 입력으로 채운다
     (`kh_repro.make_our_split_trim_warmstart`).
  2) 쿼터니언 정규화 나눗셈 특이점을 eps로 고친다 — 이미 V13/M17/F13
     전체에 적용됨(`control/hybrid_comparison.py` 등, 코드 자체에 반영
     되어 있어 이 스크립트가 따로 할 일은 없다 — 아래 결과는 자동으로
     그 수정이 적용된 상태다).
  3) 1)+2) 후에도 80/85 m/s 짧은 시험이 실패하면 H-시나리오,
     통과하면(또는 최소한 실패 양상이 바뀌면) H-웜스타트이력으로 판정.

실행: python3 -m control.kh_discriminate_scenario_vs_warmstart
"""
from models.team_light.control.run_baseline_comparison import Factory as KHFactory

from control.kh_adapter import kh_native_params, build_controller_params
from control.kh_repro import run_case, make_our_split, make_our_split_trim_warmstart


def main():
    native = kh_native_params()
    ctrl_params = build_controller_params(native)

    def our_v13_default(speed, z):
        return make_our_split(ctrl_params, speed, z)

    def our_v13_trim_warmstart(speed, z):
        return make_our_split_trim_warmstart(ctrl_params, native, speed, z)

    scenarios = [(80.0, 'nominal'), (85.0, 'nominal')]

    print("=" * 78)
    print("대조군 — 기본(실측 콜드스타트) 웜스타트, eps 정칙화는 이미 적용된 코드")
    for V, case in scenarios:
        run_case(native, our_v13_default, 'V13(기본웜스타트)', V, case)

    print()
    print("실험군 — 트림 웜스타트(kj 판별실험 1) + eps 정칙화(2)")
    for V, case in scenarios:
        run_case(native, our_v13_trim_warmstart, 'V13(트림웜스타트)', V, case)


if __name__ == '__main__':
    main()
