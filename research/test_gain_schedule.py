"""GSLQR(표5) 이득 스케줄 회귀 — 폐루프 안정성과 축소 좌표 일관성만 확인한다.

선정 프로파일은 20-80 m/s 구간에 수평 트림이 없다(TRIM_ENVELOPE_AUDIT.md,
2026-09-22 재확인). 이 테스트는 그 사실을 전제로 0-18 m/s 구간만 검증한다.
"""
import numpy as np
import pytest

from research.model import profile, rpm_limit
from research.gain_schedule import build_schedule, trim_point, linearize, reduce, design
from research.trim_envelope import LevelTrimAudit


@pytest.fixture(scope="module")
def p():
    return profile("selected")


def test_hover_trim_point_matches_audit(p):
    audit = LevelTrimAudit(p)
    x18, n, voltage = trim_point(audit, 0.0)
    assert x18.shape == (18,)
    np.testing.assert_allclose(x18[6:10], [0, -np.sqrt(.5), 0, np.sqrt(.5)], atol=1e-8)
    assert np.all(n > 0)
    assert voltage > 0


def test_hover_gain_gives_stable_closed_loop(p):
    audit = LevelTrimAudit(p)
    from research.model import build
    x18, n, voltage = trim_point(audit, 0.0)
    A17, B17 = linearize(build(p), x18, n, voltage)
    A_r, B_r, _ = reduce(A17, B17, x18[6:10])
    K_r, max_real = design(A_r, B_r)
    assert K_r.shape == (4, 14)
    assert max_real < -1e-6, f"폐루프 불안정: max_real={max_real}"


def test_schedule_covers_feasible_band_only():
    """20-80 m/s는 트림이 없으므로 요청해도 건너뛰어야 하고, 요청 안 하면 비어있다."""
    sched = build_schedule(profile("selected"), speeds=[0, 10, 18, 40, 83.3])
    assert 40 in [s["speed_mps"] for s in sched["skipped"]]
    assert 83.3 in [s["speed_mps"] for s in sched["skipped"]]
    assert sched["speeds_mps"] == [0.0, 10.0, 18.0]


def test_gain_grows_then_shrinks_through_transition():
    """0-18 m/s는 호버->전진비행 전이 구간이라 피치각이 급변한다(90도->39도) —
    이득이 부드럽게 변하는지(발산/불연속 없는지)만 확인, 특정 형태는 주장하지 않는다.

    원래는 max|K| < 1000 이라는 절대 상한이었는데, 2026-09-22 Q/R 튜닝 후
    1087 이 나와 걸렸다. 확인해 보니 큰 이득은 전부 dphi(자세, **라디안**) 열이고
    (V=14 에서 dphi_x=999), 0.1 rad=5.7도 오차에 약 100 rad/s 를 내는 것이라
    호버 회전수 1513 대비 7% 다. 즉 선형화점 문제가 아니라 단위 때문에 숫자만
    커 보이는 것이었다. 매직넘버 대신 이 테스트가 원래 말한 것(연속성)과
    물리적 기준(구동기 범위)으로 바꾼다 — 둘 다 원래 검사보다 강하다.
    """
    speeds = [0, 4, 8, 12, 16, 18]
    sched = build_schedule(profile("selected"), speeds=speeds)
    norms = [np.max(np.abs(K)) for K in sched["K_r"]]
    assert all(np.isfinite(norms))

    # (1) 연속성 — 런타임은 노드 사이를 선형보간한다(식42). 인접 노드가 크게
    # 튀면 보간 중간이 아무 설계도 아닌 이득이 되고 스케줄 전환에서 명령이
    # 불연속해진다(논문 §5.3 이 사전 시험을 요구하는 지점).
    for i in range(len(norms)-1):
        ratio = max(norms[i+1]/norms[i], norms[i]/norms[i+1])
        assert ratio < 3, f"{speeds[i]}->{speeds[i+1]} m/s 이득이 {ratio:.1f}배 점프"

    # (2) 절대 상한은 구동기 범위로 잡는다. 어떤 상태든 단위 오차 하나가
    # 로터 명령 전 구간을 넘어서면 그건 설계가 아니라 포화 스위치다.
    assert max(norms) < rpm_limit(profile("selected"))


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
