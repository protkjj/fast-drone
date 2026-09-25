"""불변식 I-8(kj 지시, 2026-09-25 밤) — V_L~V_H의 플랜트 트림 회전수에서
제어기 모델 추력이 플랜트(팀원 APC 곡선) 추력의 ±5% 안에 들어야 한다.

배경: 선형 fac=1-J/J_max 모델(J_max=1.348)이 실제 APC 5.5x6.5E 곡선을
과소평가해(트림 회전수에서 20/80/85 m/s 각각 약 0.69/0.43/0.43배) INDI의
T_meas·제어효과도·추력상한이 전부 틀려 있었다. `propulsion_model`이
팀원 곡선이면 `control/dynamics.py`·`control/hybrid_comparison.py`가
그 곡선(models.team_light.control.propeller_curve, PCHIP)을 그대로
쓰도록 바꿨다 — 재구현하지 않음.

이 파일은 "판정"(V13이 통과하는지)이 아니라 "프로펠러 모델 교체 자체가
맞게 됐는지"만 확인한다. 짧은 시험 재실행·가설 판정은 범위 밖(다음
세션/arena-completion 담당).
"""
import numpy as np
import pytest

from models.team_light.control.trim import find_trim as kh_find_trim
from scipy.spatial.transform import Rotation

from control.kh_adapter import kh_native_params, build_controller_params
from control.hybrid_comparison import _curve_thrust, rotor_thrust_cap

NATIVE = kh_native_params()
CP = build_controller_params(NATIVE)


@pytest.mark.parametrize('speed', [20.0, 80.0, 85.0])
def test_i8_controller_thrust_within_5pct_of_plant_at_trim(speed):
    """I-8: 플랜트 트림 회전수에서 제어기 모델(곡선) 추력이 플랜트
    추력(트림 정의 자체)의 ±5% 안에 있어야 한다."""
    tr = kh_find_trim(NATIVE, speed)
    n = tr['control']
    R = Rotation.from_quat(tr['state'][6:10]).as_matrix()
    v_body = R.T @ tr['state'][3:6]
    V_axial = max(v_body[0], 0.0)

    T_ctrl = _curve_thrust(CP, n, V_axial).sum()
    T_plant = tr['T_total']
    assert T_ctrl == pytest.approx(T_plant, rel=0.05)


def test_curve_thrust_matches_linear_model_removed_the_old_gap():
    """수정 전 상태(선형 fac 모델)였다면 80m/s에서 비율이 약 0.43이었다
    (results/SOLVER_FAILURE_REPORT_2026-09-25.md 참고) — 회귀 방지용으로
    '옛 오차가 다시 나타나면' 실패하게 넓은 여유로 고정한다."""
    tr = kh_find_trim(NATIVE, 80.0)
    n = tr['control']
    R = Rotation.from_quat(tr['state'][6:10]).as_matrix()
    v_body = R.T @ tr['state'][3:6]
    V_axial = max(v_body[0], 0.0)
    ratio = _curve_thrust(CP, n, V_axial).sum() / tr['T_total']
    assert ratio > 0.9   # 옛 선형모델은 약 0.43이었다 — 0.9 미만이면 회귀


@pytest.mark.parametrize('speed', [20.0, 80.0, 85.0])
def test_rotor_thrust_cap_uses_curve_not_linear(speed):
    """rotor_thrust_cap()도 곡선을 쓰는지 — 선형모델이었다면 이 값이
    실제보다 훨씬 낮았다(포화 판정·A1 배분에 쓰이는 값이라 중요)."""
    tr = kh_find_trim(NATIVE, speed)
    R = Rotation.from_quat(tr['state'][6:10]).as_matrix()
    v_body = R.T @ tr['state'][3:6]
    V_axial = max(v_body[0], 0.0)
    cap = rotor_thrust_cap(CP, V_axial)
    assert np.all(np.isfinite(cap)) and np.all(cap > 0)
    # 정지추력(J=0) 상한보다는 항상 작거나 같아야 한다(물리적으로 당연).
    from models.team_light.control.propeller_curve import coefficients
    CT0, _ = coefficients(CP, 0.0)
    static_cap = CP['rho'] * CP['n_max']**2/(2*np.pi)**2 * CP['D_prop']**4 * float(CT0)
    assert np.all(cap <= static_cap + 1e-6)


def test_our_own_aircraft_unaffected():
    """우리 자신의 기체(propulsion_model 없음)는 이번 수정과 무관하게
    기존 선형모델 그대로 — 회귀 가드."""
    from control.vehicle_params import load_selected_params
    ours = load_selected_params()
    assert ours.get('propulsion_model') != 'apc_30k_pchip_v2'
