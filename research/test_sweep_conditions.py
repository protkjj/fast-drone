"""표8 조건은 '제어기를 구분하는 조건'이어야 한다.

이 파일이 생긴 이유는 실제로 두 번 깨졌기 때문이다.
  · V_H=18 + 측풍 5: 기체의 피치·요 권한을 넘어, 어떤 제어기도 자세를 못 잡는
    조건이었다 (12개 중 8개). -> TRIM_CAUSE_AND_LEVERS.md §6.5
  · Q10 initial_soc=0.2: profile.battery.minimum_soc 와 같아서 runtime.js:959 의
    컷오프에 첫 스텝부터 걸렸다. 세 제어기 전부 같은 사유로 실패했다.

둘 다 "제어기 성능"이 아니라 "조건 자체의 불가능"을 재고 있었고, 눈으로는
안 보였다. 조건을 손댈 때마다 여기서 걸리게 한다.
"""
import json
import subprocess
from pathlib import Path

import pytest

from research.model import profile
from research.trim_envelope import LevelTrimAudit
from research.trim_levers import combined_authority

RESEARCH = Path(__file__).parent
# 8초 런의 실측 SOC 소비(gindi, V=14, 측풍5). 여유는 이 값 기준으로 본다.
MEASURED_SOC_DRAW_8S = 0.0169


@pytest.fixture(scope="module")
def conditions():
    out = subprocess.run(
        ["node", "-e", "console.log(JSON.stringify(require('./sweep.cjs').CONDITIONS))"],
        cwd=RESEARCH, capture_output=True, text=True)
    if out.returncode != 0:
        pytest.skip(f"node 로 sweep.cjs 를 못 읽음: {out.stderr[:200]}")
    return json.loads(out.stdout)


@pytest.fixture(scope="module")
def audit():
    return LevelTrimAudit()


def test_every_condition_speed_has_a_level_trim(audit, conditions):
    """트림이 없는 속도를 목표로 주면 추종 오차가 아니라 기체 한계를 재게 된다."""
    for cond in conditions:
        result = audit.evaluate(cond["speed"], 1)
        assert result["model_feasible"], \
            f"{cond['id']}: V={cond['speed']} 트림 불가 {result['rejection_reasons']}"


def test_no_condition_exceeds_pitch_and_yaw_authority(audit, conditions):
    """피치·요 요구는 더해진다(T >= (|My|+|Mz|)/a). 측풍은 트림 위에 얹힌다."""
    for cond in conditions:
        opts = cond["opts"]
        crosswind = opts.get("wind_speed", 0) if opts.get("wind_angle") == 90 else 0
        used = combined_authority(audit, cond["speed"], crosswind)["authority_used"]
        assert used <= 1, f"{cond['id']}: 권한 소모 {used:.1%} — 기체 한계를 재는 조건"


def test_vertical_wind_conditions_stay_inside_authority(audit, conditions):
    """수직풍은 받음각을 직접 바꾼다(상승풍이 나쁜 쪽). 측풍과 같은 식으로 잰다."""
    import numpy as np
    p = audit.p
    for cond in conditions:
        vertical = cond["opts"].get("wind_vertical_mps")
        if not vertical:
            continue
        theta = np.radians(audit.force_balance(cond["speed"])["theta_deg"])
        force = np.asarray(audit.f["aero"](audit.state(cond["speed"], theta, 1)[:13],
                                           np.array([0., 0., float(vertical)]))).ravel()
        offset = p["cp_from_cg_m"]
        needed = (abs(offset*float(force[2])) + abs(offset*float(force[1])))/audit.pitch_arm
        thrust = audit.weight*np.sin(theta) - float(force[0])
        assert needed/thrust <= 1, f"{cond['id']}: 권한 소모 {needed/thrust:.1%}"


def test_initial_soc_leaves_room_above_the_cutoff(conditions):
    """runtime.js:959 는 SOC < minimum_soc 면 즉시 실패시킨다.

    바닥에서 시작하면 첫 스텝에 걸려 모든 제어기가 같은 사유로 죽는다.
    소비량만큼의 여유로는 부족하므로 2배를 요구한다.
    """
    floor = profile("selected")["battery"]["minimum_soc"]
    for cond in conditions:
        soc = cond["opts"].get("initial_soc")
        if soc is None:
            continue
        assert soc > floor, f"{cond['id']}: initial_soc {soc} <= minimum_soc {floor}"
        assert soc - floor >= 2*MEASURED_SOC_DRAW_8S, \
            f"{cond['id']}: 여유 {soc-floor:.4f} < 소비 {MEASURED_SOC_DRAW_8S}의 2배"


def test_conditions_still_span_low_and_high_speed(conditions):
    """권한 때문에 V_H 를 낮추더라도, 두 비행영역 대비는 남아 있어야 한다."""
    speeds = sorted({c["speed"] for c in conditions})
    assert len(speeds) >= 2
    assert max(speeds)/min(speeds) >= 1.5, f"속도 대비가 너무 좁다: {speeds}"


def test_disturbance_conditions_actually_perturb(conditions):
    """무풍과 구분되지 않는 '외란' 조건이 섞여 있으면 표가 정보를 잃는다."""
    perturbed = [c for c in conditions
                 if any(k in c["opts"] for k in
                        ("wind_speed", "wind_vertical_mps", "scales", "initial_soc",
                         "gps_position_std_m", "indi_rpm_desync_ms"))]
    assert len(perturbed) >= len(conditions)//2
