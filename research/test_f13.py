"""F13(표5) 예측모델 회귀 — 로터추력 직접입력 13상태, 실제 강체 회전방정식.

hybrid(V13)의 이상화(omega_dot=nu)와 달리 실제 J*omega_dot=M-omega x J*omega를
쓴다는 게 F13과 V13의 예측모델 차이(논문 "가장 가까운 인터페이스 비교").
"""
import numpy as np
import pytest

from research.model import build, initial_state, profile


@pytest.fixture(scope="module")
def functions():
    return build(profile("selected"))


def test_hover_thrust_split_is_equilibrium(functions):
    p = profile("selected")
    x0 = initial_state(p)[:13]
    f_hover = np.full(4, p["mass_kg"]*p["g"]/4)
    dx = np.asarray(functions["f13"](x0, f_hover, [0, 0, 0])).ravel()
    assert np.max(np.abs(dx)) < 1e-10


def test_uses_real_rigid_body_rotation_not_idealized(functions):
    """omega_dot=nu(가상모델)와 달리 F13은 실제 J*omega_dot=M-omega x J*omega를
    쓰므로, 0이 아닌 각속도에서 자이로 결합항(omega x J*omega)이 실제로 보여야 한다."""
    p = profile("selected")
    x0 = initial_state(p)[:13]
    x0[10:13] = [.3, -.5, .2]  # 0이 아닌 각속도(자이로 결합항이 있어야 보임)
    f_hover = np.full(4, p["mass_kg"]*p["g"]/4)
    dx_spin = np.asarray(functions["f13"](x0, f_hover, [0, 0, 0])).ravel()
    x0[10:13] = 0.0
    dx_still = np.asarray(functions["f13"](x0, f_hover, [0, 0, 0])).ravel()
    # 각가속도(9:13 -> 10:13 index in 13-state dx는 10:13) 성분이 회전 시 달라져야
    # (자이로/공력감쇠 결합이 반영된다는 뜻 — 이상화 모델(virtual)이면 nu=0으로
    # 고정돼 있어 이 항이 아예 없다).
    assert not np.allclose(dx_spin[10:13], dx_still[10:13], atol=1e-6)


def test_reaction_torque_ratio_is_positive_and_finite(functions):
    """반작용 토크/추력 비율(호버점 고정, model.py의 근사)이 물리적으로 말이 되는지."""
    p = profile("selected")
    n_hover = initial_state(p)[13:17]
    _, _, dT, dQ = functions["rotors"](n_hover, 0)
    dT, dQ = np.asarray(dT).ravel(), np.asarray(dQ).ravel()
    ratio = dQ/dT
    assert np.all(np.isfinite(ratio))
    assert np.all(ratio > 0), "반작용 토크는 추력과 같은 부호(둘 다 회전수의 함수)여야 함"


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
