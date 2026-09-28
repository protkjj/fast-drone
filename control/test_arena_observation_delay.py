"""관측 지연 버퍼 시험. kj 시험 항목: 지연 0=비트 동일, 지연>0=정확히 k 스텝 전 값."""
import numpy as np
import pytest

from control.arena_observation_delay import DelayedObservation

NX = 17
DT = 0.002


def make_x(k):
    x = np.zeros(NX)
    x[:] = float(k)          # 매 스텝을 스텝번호로 채운 구분되는 상태
    return x


def test_zero_delay_returns_the_same_state():
    obs = DelayedObservation(DT, state_delay_s=0.0, rotor_delay_s=0.0)
    obs.reset()
    for k in range(1, 5):
        x = make_x(k)
        out = obs.observe(x)
        np.testing.assert_array_equal(out, x)


def test_state_delay_gives_exactly_k_steps_ago():
    """기록이 아직 k_delay스텝 안 쌓였으면(예열 구간) 가장 오래된(첫) 실측값을 그대로 낸다 —
    "이 전에는 무엇이었나" 정의가 없어서다. make_x(k)=k라서 기대값은 max(1, k_now-k_delay)."""
    k_delay = 5
    obs = DelayedObservation(DT, state_delay_s=k_delay*DT, rotor_delay_s=0.0)
    obs.reset()
    outs = [obs.observe(make_x(k)) for k in range(1, 12)]
    for i, out in enumerate(outs):
        k_now = i + 1
        expected = max(1, k_now - k_delay)
        np.testing.assert_array_equal(out[0:13], expected)


def test_rotor_delay_is_independent_of_state_delay():
    obs = DelayedObservation(DT, state_delay_s=3*DT, rotor_delay_s=1*DT)
    obs.reset()
    outs = [obs.observe(make_x(k)) for k in range(1, 8)]
    for i, out in enumerate(outs):
        k_now = i + 1
        np.testing.assert_array_equal(out[0:13], max(1, k_now - 3))
        np.testing.assert_array_equal(out[13:17], max(1, k_now - 1))


def test_5ms_rounds_to_the_nearest_integer_step():
    """dt=0.002s에서 5ms=2.5스텝 — round()로 2스텝(4ms)이 된다. 근사임을 여기서 확인한다."""
    obs = DelayedObservation(DT, rotor_delay_s=0.005)
    assert obs.k_rotor == 2


def test_history_buffer_does_not_grow_unbounded():
    obs = DelayedObservation(DT, state_delay_s=3*DT)
    obs.reset()
    for k in range(1, 500):
        obs.observe(make_x(k))
    assert len(obs._history) <= 5
