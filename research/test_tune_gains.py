"""튜닝 프로토콜은 주석이 아니라 검사로 지켜져야 한다.

논문 §5.3 의 세 요구(독립 시나리오 · 같은 예산 · 본시험 비참조) 중 앞의 둘은
기계로 검사할 수 있다. 특히 '독립 시나리오'는 사람이 눈으로 맞춰 두면 나중에
표8 조건이 바뀔 때 조용히 깨진다.
"""
import json
import re
import subprocess
from pathlib import Path

import numpy as np
import pytest

from research.tune_gains import (CPID_BASE, GINDI_BASE, GSLQR_BASE, RUN_SETTINGS,
                                 SCHEDULE_SPEEDS, TUNING_SCENARIOS, expand_pid,
                                 gslqr_weights)

RESEARCH = Path(__file__).parent


def _literal(name):
    """runtime.js 의 게인 상수 리터럴을 읽어 dict 로 만든다."""
    source = (RESEARCH/"runtime.js").read_text()
    body = re.search(rf"const {name} = \{{(.*?)\}};", source, re.S).group(1)
    body = re.sub(r"(\w+):", r'"\1":', body)          # 키에 따옴표
    body = re.sub(r"(?<![\d.])\.(\d)", r"0.\1", body)  # .15 -> 0.15
    return json.loads("{"+body+"}")


def _test_conditions():
    out = subprocess.run(
        ["node", "-e", "console.log(JSON.stringify(require('./sweep.cjs').CONDITIONS))"],
        cwd=RESEARCH, capture_output=True, text=True)
    if out.returncode != 0:
        pytest.skip(f"node 로 sweep.cjs 를 못 읽음: {out.stderr[:200]}")
    return json.loads(out.stdout)


def test_tuning_scenarios_share_no_operating_point_with_the_test_set():
    """§5.3 '독립적인 튜닝 시나리오' — 속도·바람 조합이 표8과 하나도 겹치면 안 된다."""
    def signature(speed, opts):
        return (float(speed), float(opts.get("wind_speed", 0)),
                float(opts.get("wind_angle", 0)), float(opts.get("wind_vertical_mps", 0)))

    test_points = {signature(c["speed"], c["opts"]) for c in _test_conditions()}
    tuning_points = {signature(s["speed"], s["opts"]) for s in TUNING_SCENARIOS}
    assert not (test_points & tuning_points), f"겹치는 작동점: {test_points & tuning_points}"


def test_tuning_speeds_are_distinct_from_test_speeds():
    """속도만 따로 봐도 겹치지 않아야 한다(바람이 달라도 같은 속도면 독립성이 약하다)."""
    test_speeds = {float(c["speed"]) for c in _test_conditions()}
    assert not ({float(s["speed"]) for s in TUNING_SCENARIOS} & test_speeds)


def test_tuning_seeds_are_disjoint_from_test_seeds():
    """§5.2 '학습·튜닝·본시험 집합 분리' — 난수까지 갈라야 한다.

    sweep.cjs 는 seed = 1000+s 를 쓴다(최대 수백 개). 튜닝은 2000번대를 쓴다.
    """
    assert all(s["seed"] >= 2000 for s in TUNING_SCENARIOS)
    assert min(s["seed"] for s in TUNING_SCENARIOS) > 1000 + 500


def test_tuning_speeds_stay_inside_the_trimmable_and_authority_limited_band():
    """트림이 없거나 권한을 넘는 속도에서 튜닝하면 게인이 아니라 기체 한계를 맞추게 된다."""
    from research.trim_envelope import LevelTrimAudit
    from research.trim_levers import combined_authority
    audit = LevelTrimAudit()
    for scenario in TUNING_SCENARIOS:
        crosswind = scenario["opts"].get("wind_speed", 0) if \
            scenario["opts"].get("wind_angle") == 90 else 0
        used = combined_authority(audit, scenario["speed"], crosswind)["authority_used"]
        assert used < 1, f"{scenario['id']}: 권한 소모 {used:.1%}"


def _tuning_record():
    path = RESEARCH.parent/"results"/"tuned_gains.json"
    if not path.exists():
        pytest.skip("튜닝 기록 없음 — python3 -m research.tune_gains 먼저 실행")
    return json.loads(path.read_text())


def test_runtime_gains_are_exactly_the_recorded_tuning_outcome():
    """§5.3 '본시험 결과를 보고 가중치를 다시 선택하지 않는다'.

    손으로 고친 게인은 프로토콜 위반이고, 기록과 배포가 갈라지면 논문의 수치를
    재현할 수 없다. 배포된 상수가 탐색이 내놓은 값 그대로인지 검사한다.
    """
    chosen = _tuning_record().get("selection", {}).get("chosen")
    if not chosen:
        pytest.skip("선택 기록(selection) 없음")
    for name, controller in [("CPID_GAINS", "cpid"), ("GINDI_GAINS", "gindi")]:
        if controller not in chosen:
            pytest.skip(f"{controller} 선택 기록 없음")
        expected = expand_pid(chosen[controller]["params"])
        for key, value in _literal(name).items():
            assert np.allclose(expected[key], value), \
                f"{name}.{key}: 배포 {value} != 튜닝 기록 {expected[key]}"


def test_shipped_gslqr_weights_are_exactly_the_recorded_tuning_outcome():
    from research.gain_schedule import TUNED_Q_DIAG, TUNED_R_SCALE
    chosen = _tuning_record().get("selection", {}).get("chosen", {})
    if "gslqr" not in chosen:
        pytest.skip("gslqr 선택 기록 없음")
    Q, R = gslqr_weights(chosen["gslqr"]["params"])
    np.testing.assert_allclose(np.diag(Q), TUNED_Q_DIAG)
    np.testing.assert_allclose(np.diag(R), [TUNED_R_SCALE]*4)


def test_recorded_baselines_are_the_pre_tuning_placeholders():
    """탐색의 출발점이 기록과 같아야 '개선폭 몇 배' 보고가 성립한다."""
    record = _tuning_record()["results"]
    for controller, base in [("cpid", CPID_BASE), ("gindi", GINDI_BASE),
                             ("gslqr", GSLQR_BASE)]:
        if controller in record:
            assert record[controller]["baseline"]["params"] == base


def test_expand_pid_ties_the_two_horizontal_axes():
    """수평 2축을 한 파라미터로 묶는 것이 탐색 차원을 줄이는 근거다."""
    gains = expand_pid(CPID_BASE)
    for key in ("kpV", "kiV", "kdV"):
        assert gains[key][0] == gains[key][1]
        assert len(gains[key]) == 3


def test_gslqr_weight_matrices_have_the_reduced_error_state_shape():
    Q, R = gslqr_weights(GSLQR_BASE)
    assert Q.shape == (14, 14) and R.shape == (4, 4)
    assert np.all(np.diag(Q) > 0) and np.all(np.diag(R) > 0)
    assert np.count_nonzero(Q - np.diag(np.diag(Q))) == 0


def test_schedule_factory_matches_the_real_lqr_design_path():
    """튜너가 쓰는 ScheduleFactory 가 gain_schedule.design 과 같은 K 를 내야 한다.

    (튜닝 전에는 'GSLQR_BASE == design() 기본값'으로 검사했지만, 기본값이 이제
     튜닝 결과라 그 등식은 성립하지 않는다. 검사할 것은 두 경로의 일치다.)
    """
    from research.gain_schedule import design
    from research.tune_gains import ScheduleFactory
    factory = ScheduleFactory(speeds=[0, 8])
    schedule = factory.build(GSLQR_BASE)
    assert schedule is not None
    entry = factory.cache[0]
    Q, R = gslqr_weights(GSLQR_BASE)
    expected, _ = design(entry["A"], entry["B"], Q, R)
    np.testing.assert_allclose(schedule["K_r"][0], expected, rtol=1e-9)


def test_shipped_design_defaults_are_the_tuned_weights_not_the_placeholder():
    """gain_schedule.design() 의 기본값이 배포 기준이다 — 여기에 튜닝이 반영돼야
    번들 재생성이 튜닝 결과를 싣는다."""
    from research.gain_schedule import TUNED_Q_DIAG, design
    from research.tune_gains import ScheduleFactory
    placeholder_Q, placeholder_R = gslqr_weights(GSLQR_BASE)
    assert list(np.diag(placeholder_Q)) != list(TUNED_Q_DIAG), "튜닝이 반영되지 않았다"
    factory = ScheduleFactory(speeds=[0])
    entry = factory.cache[0]
    default_K, _ = design(entry["A"], entry["B"])
    placeholder_K, _ = design(entry["A"], entry["B"], placeholder_Q, placeholder_R)
    assert not np.allclose(default_K, placeholder_K)


def test_schedule_grid_matches_the_bundle_builder():
    """튜닝이 배포와 다른 격자를 쓰면 튜닝 결과를 그대로 반영할 수 없다."""
    source = (RESEARCH/"build_bundle.py").read_text()
    grid = re.search(r"build_schedule\(p, speeds=\[([^\]]+)\]", source).group(1)
    assert [float(x) for x in grid.split(",")] == [float(x) for x in SCHEDULE_SPEEDS]


def test_run_settings_match_the_test_harness_timing():
    """램프 타이밍이 다르면 평가창(evaluateRun)이 달라져 점수가 비교 불가가 된다."""
    source = (RESEARCH/"sweep.cjs").read_text()
    t0 = float(re.search(r"RAMP_T0 = ([\d.]+)", source).group(1))
    duration = float(re.search(r"RAMP_DURATION_S = ([\d.]+)", source).group(1))
    assert RUN_SETTINGS["ramp_t0"] == t0
    assert RUN_SETTINGS["ramp_duration_s"] == duration
    assert RUN_SETTINGS["seconds"] > t0 + duration, "정착 구간 표본이 없으면 평가가 실패 처리된다"
