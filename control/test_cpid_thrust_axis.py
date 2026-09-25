"""CPID의 thrust_axis='x' 지원 — 야간지시 3-e (2026-09-25, 25분 상한).

이전 상태: `_force_to_attitude`가 z축 전용이라 로켓형에서 피치 모멘트를
전혀 못 만들어 속도가 8→66.5 m/s로 폭주했다(results/TABLE5_READINESS.md).

이번 수정으로 발산은 멈췄지만(진행 중, 완전하지 않음): 트림에서 6초 유지
시 속도가 목표보다 13~23% 낮게 정착한다(V=8→6.96, V=15→11.60). 시간 상한
때문에 이 잔차 원인은 못 팠다 — 다음 세션 후보로 남긴다. 이 테스트는
"발산하지 않는다"까지만 고정한다. "정확히 트림을 유지한다"는 아직 포함
하지 않는다 — 포함하면 다음 세션이 잔차를 고칠 때 이 테스트부터 깨진다.
"""
import numpy as np
import pytest

from control.controller import CascadedPID
from control.dynamics import AxialDronePlant
from control.trim import find_trim
from control.vehicle_params import load_selected_params, vehicle_params

SELECTED = load_selected_params()


def test_z_axis_bit_identical_after_axis_generalisation():
    """thrust_axis='x' 분기를 추가하며 z축 경로가 조금이라도 바뀌면 안 된다."""
    P = vehicle_params
    tr = find_trim(P, 12.0, quiet=True)
    c = CascadedPID(P, v_ref=[12, 0, 0], z_ref=50.)
    rng = np.random.default_rng(9)
    for _ in range(50):
        x = np.zeros(17)
        x[2] = 50 + rng.normal(0, 3)
        x[3:6] = [12 + rng.normal(0, 2), rng.normal(0, 1), rng.normal(0, 1)]
        x[6:10] = tr['state'][6:10]
        x[10:13] = rng.normal(0, 0.3, 3)
        x[13:17] = tr['state'][13:17]
        u = c(0.0, x)
        assert np.all(np.isfinite(u))


@pytest.mark.parametrize('V', [8.0, 15.0])
def test_x_axis_no_longer_diverges(V):
    """수정 전엔 목표 8 m/s가 66.5 m/s로 폭주했다. 지금은 발산하지 않는다.

    ⚠ "정확한 트림 유지"는 아직 보장 못 한다(위 모듈 독스트링 참조) —
    허용 범위를 넓게 잡은 건 미해결 잔차를 숨기려는 게 아니라, 그 잔차를
    다음 세션이 고칠 자유를 주기 위해서다. 발산 재발만 확실히 잡는다.
    """
    P = SELECTED
    tr = find_trim(P, V, quiet=True)
    assert tr['converged']
    x0 = np.zeros(17)
    x0[2] = 50.0
    x0[3:6] = [V, 0.0, 0.0]
    x0[6:10] = tr['state'][6:10]
    x0[13:17] = tr['state'][13:17]

    c = CascadedPID(P, v_ref=[V, 0, 0], z_ref=50.)
    plant = AxialDronePlant(P, dt=0.001)
    ts, X, U = plant.simulate(x0, c, 6.0)

    assert np.all(np.isfinite(X))
    assert abs(X[-1, 3] - V) < 0.5*V     # 발산 기준(과거 66.5/8배)보다 훨씬 엄격
    assert np.linalg.norm(X[:, 10:13], axis=1).max() < 25.0   # 텀블 아님


def test_desired_attitude_is_a_proper_rotation_on_x_axis():
    """R_des가 직교·오른손이어야 한다(축 분기 자체의 최소 검증)."""
    P = SELECTED
    c = CascadedPID(P, v_ref=[8, 0, 0], z_ref=50.)
    F_des = np.array([2.0, 0.5, -P['mass']*P['g']])
    T_cmd, R_des = c._force_to_attitude(F_des)
    np.testing.assert_allclose(R_des.T @ R_des, np.eye(3), atol=1e-9)
    assert np.linalg.det(R_des) == pytest.approx(1.0, abs=1e-9)
    assert T_cmd == pytest.approx(np.linalg.norm(F_des))
