"""작업자 수 권장식 — (가용 × 0.7) / 작업자당, CPU − 1, 시나리오 수 18 중 가장 작은 값, 최소 1."""
import pytest

from control.arena_suggest_workers import recommended_workers


@pytest.mark.parametrize('available, per_worker, cpus, expected', [
    (16.0, 2.0, 16, 5),     # 메모리가 묶는다: 11.2 // 2 = 5
    (64.0, 2.0, 8, 7),      # CPU − 1이 묶는다
    (512.0, 0.6, 64, 18),   # 시나리오 수가 묶는다
    (2.0, 2.0, 8, 1),       # 1.4 // 2 = 0이어도 최소 1
    (1.0, 2.0, 1, 1),       # CPU 1개여도 최소 1
    (8.571, 2.0, 16, 2),    # 5.9997 // 2 = 2 (경계 바로 아래)
    (8.572, 2.0, 16, 3),    # 6.0004 // 2 = 3
])
def test_recommended_workers(available, per_worker, cpus, expected):
    assert recommended_workers(available, per_worker, cpus) == expected
