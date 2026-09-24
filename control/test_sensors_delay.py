"""
GPS 지연·누락·런타임 갱신률 변경 검증 — 논문 §5.7/표7 대응.
"""

import numpy as np

from control.sensors import GPSSensor


def _true_state(t):
    x = np.zeros(17)
    x[0:3] = [t, 0.0, 0.0]     # x=t로 두면 "몇 초 전 위치"인지 바로 확인 가능
    x[3:6] = [1.0, 0.0, 0.0]
    return x


def test_no_delay_matches_old_behavior():
    """delay_s=0, dropout_prob=0(기본값)이면 기존과 동일하게 매 주기 즉시 출력."""
    dt = 0.001
    gps = GPSSensor(dt_plant=dt, gps_rate=10.0, noise_pos=0.0, noise_vel=0.0, seed=1)
    hits = 0
    for k in range(1, 201):  # 0.2s = 2주기(10Hz→0.1s마다)
        r = gps.measure(_true_state(k * dt))
        if r is not None:
            hits += 1
            pos_meas, _ = r
            assert abs(pos_meas[0] - k * dt) < 1e-9, "지연 없어야 하는데 지연됨"
    assert hits == 2, f"10Hz·0.2s면 2회 나와야 하는데 {hits}"
    print(f"  hits={hits} (기대 2), 지연 없음 확인. PASS")


def test_delay_holds_back_samples():
    """delay_s=0.05면 표본이 취득 후 0.05s 뒤에야 나와야 한다."""
    dt = 0.001
    gps = GPSSensor(dt_plant=dt, gps_rate=10.0, noise_pos=0.0, noise_vel=0.0,
                    seed=1, delay_s=0.05)
    first_hit_t = None
    for k in range(1, 301):
        t = k * dt
        r = gps.measure(_true_state(t))
        if r is not None and first_hit_t is None:
            first_hit_t = t
            pos_meas, _ = r
            age = t - pos_meas[0]
            print(f"  첫 출력 t={t:.3f}s, 표본 시각={pos_meas[0]:.3f}s, age={age:.3f}s")
            # 이산시간(dt=1ms)이라 delay_s 경계가 정확히 스텝에 안 맞으면
            # 한 스텝(dt) 안에서 반올림된다 — 그 오차까지만 허용한다.
            assert abs(age - 0.05) < dt + 1e-9, f"지연이 0.05s±1스텝이어야 하는데 {age}"
    assert first_hit_t is not None
    # 원래(무지연) 첫 출력은 t=0.1s였을 것 — 지연으로 ~0.15s로 밀려야 함
    assert abs(first_hit_t - 0.15) < dt + 1e-9
    print("  PASS")


def test_bernoulli_dropout_drops_samples():
    """dropout_prob=1.0이면 표본이 하나도 안 나와야 한다(결정론적 극단 케이스)."""
    dt = 0.001
    gps = GPSSensor(dt_plant=dt, gps_rate=10.0, noise_pos=0.0, noise_vel=0.0,
                    seed=1, dropout_prob=1.0)
    hits = sum(1 for k in range(1, 501) if gps.measure(_true_state(k*dt)) is not None)
    assert hits == 0, f"dropout_prob=1.0인데 {hits}회 나옴"
    print("  dropout_prob=1.0 → 0회 출력 확인. PASS")


def test_scheduled_outage_blocks_window():
    """schedule_outage(1.0, 1.0)이면 [1.0,2.0)s 동안 아무 것도 안 나와야 한다."""
    dt = 0.001
    gps = GPSSensor(dt_plant=dt, gps_rate=10.0, noise_pos=0.0, noise_vel=0.0, seed=1)
    gps.schedule_outage(1.0, 1.0)
    hits_in_window = 0
    hits_outside = 0
    for k in range(1, 2501):  # 0~2.5s
        t = k * dt
        r = gps.measure(_true_state(t))
        if r is not None:
            if 1.0 <= t < 2.0:
                hits_in_window += 1
            else:
                hits_outside += 1
    print(f"  구간 내 출력={hits_in_window}(기대 0), 구간 외 출력={hits_outside}(기대 >0)")
    assert hits_in_window == 0
    assert hits_outside > 0
    print("  PASS")


def test_set_rate_changes_period():
    """set_rate(20)이면 주기가 절반(10Hz 대비)으로 줄어야 한다."""
    dt = 0.001
    gps = GPSSensor(dt_plant=dt, gps_rate=10.0, noise_pos=0.0, noise_vel=0.0, seed=1)
    before = gps.period_steps
    gps.set_rate(20.0)
    after = gps.period_steps
    print(f"  period_steps: {before} → {after}")
    assert after == before // 2
    print("  PASS")


if __name__ == '__main__':
    tests = [
        test_no_delay_matches_old_behavior,
        test_delay_holds_back_samples,
        test_bernoulli_dropout_drops_samples,
        test_scheduled_outage_blocks_window,
        test_set_rate_changes_period,
    ]
    for t in tests:
        print(f"\n{'='*55}\n{t.__name__}")
        t()
    print(f"\n{'='*55}\nALL SENSOR DELAY/DROPOUT TESTS PASSED\n{'='*55}")
