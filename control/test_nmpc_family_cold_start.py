"""NMPC 계열(V13·M17·F13) 콜드스타트 웜스타트 회귀 테스트 — kj 지시
(2026-09-25 저녁, 4번): "정확한 트림에서 풀면 트림 입력이 나오고 수렴하며
NaN이 없다"를 세 구현 공통 단위 테스트로 만든다.

배경: kj가 직접 재확인해 지적한 대로, `_build_nlp_paper`류의 콜드스타트
웜스타트가 상태 부분을 전부 "호버형" 고정값([0,0,0,...,호버쿼터니언,
...,0,0,0])으로 채우고 있었다. 순항 트림이 호버와 먼 조건(80/85 m/s,
피치 -8~13°)에서는 이 추측값이 실측과 크게 어긋나 다중사격 등식제약이
전체 호라이즌에서 동시에 크게 위반된 채 IPOPT가 시작하게 되고, 그 결과
(격리 실험으로 확인, results/SOLVER_FAILURE_LOG_2026-09-25.md 참고):
  - V13(80m/s, 정확한 트림, 섭동 없음): 입력이 [81.1,-95.4,62.0,-100.0]
    으로 박스에 붙고 Maximum_Iterations_Exceeded, nlp_g에서 NaN 경고
  - eps로 쿼터니언 정규화 나눗셈만 정칙화해서는 결과가 전혀 안 바뀜
    (원인이 나눗셈 특이점이 아니라 웜스타트임을 확인)
  - X_k(상태) 웜스타트만 실측/트림으로 바꾸면(U_k는 이미 합리적이라
    그대로 둠) 위 실패가 사라지고 정확히 트림으로 수렴

수정: `_solve()`가 매 인스턴스의 **첫 호출에서만**(`self._cold_start`
플래그) `self.w0`의 X_k 블록을 그 호출의 실측 상태로 덮어쓴다. 세 구현
전부 동일 패턴(kj 규칙: V13만 고치지 않는다).

주의: 이게 80/85 m/s 짧은 시험(±0.2m/±0.5m/s 섭동)의 "첫 스텝 열화"
전체를 없애지는 않는다 — 정확한 트림에서는 완전히 고치지만, 작은 섭동이
있으면 여전히 첫 스텝이 낮게 나오는 현상이 남는다(별개의, 더 작은
미해결 문제 — control/test_v13_degenerate_first_step.py 참고).
"""
import numpy as np
import pytest

from models.team_light.control.trim import find_trim as kh_find_trim

from control.kh_adapter import kh_native_params, build_controller_params
from control.hybrid_comparison import VirtualNMPC
from control.nmpc import NMPCController
from control.nmpc_f13 import RotorThrustNMPC13

NATIVE = kh_native_params()
CP = build_controller_params(NATIVE)
Z = 20.0


@pytest.mark.parametrize('speed', [80.0, 85.0])
def test_v13_exact_trim_converges_to_trim_thrust(speed):
    tr = kh_find_trim(NATIVE, speed)
    x13 = np.concatenate([tr['state'][0:10], tr['state'][10:13]])
    x13[2] = Z
    nmpc = VirtualNMPC(CP, v_ref=[speed, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper')
    vc = nmpc._solve(x13)
    assert nmpc.last_status == 'Solve_Succeeded'
    assert np.all(np.isfinite(vc))
    assert vc[0] == pytest.approx(tr['T_total'], rel=0.05)
    np.testing.assert_allclose(vc[1:4], 0.0, atol=1.5)


@pytest.mark.parametrize('speed', [80.0, 85.0])
def test_m17_exact_trim_converges_to_trim_rotor_speeds(speed):
    tr = kh_find_trim(NATIVE, speed)
    x17 = tr['state'].copy()
    x17[2] = Z
    m17 = NMPCController(CP, v_ref=[speed, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper')
    u = m17._solve(x17)
    assert m17._solve_log[-1] == 'Solve_Succeeded'
    assert np.all(np.isfinite(u))
    # M17은 17상태 전체(로터 동역학 포함)를 예측해 V13/F13만큼 정확히
    # 트림에 붙진 않지만(자체 예측모델의 잔차), 콜드스타트 실패의 징후인
    # "완전붕괴/박스포화"는 없어야 한다 — 트림 대비 25% 이내.
    np.testing.assert_allclose(u, tr['control'], rtol=0.25)


@pytest.mark.parametrize('speed', [80.0, 85.0])
def test_f13_exact_trim_converges_to_trim_thrust(speed):
    tr = kh_find_trim(NATIVE, speed)
    x13 = np.concatenate([tr['state'][0:10], tr['state'][10:13]])
    x13[2] = Z
    f13 = RotorThrustNMPC13(CP, v_ref=[speed, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper')
    u = f13._solve(x13)
    assert f13.last_status == 'Solve_Succeeded'
    assert np.all(np.isfinite(u))
    assert float(np.sum(u)) == pytest.approx(tr['T_total'], rel=0.05)


def _trim_x13(speed):
    tr = kh_find_trim(NATIVE, speed)
    x13 = np.concatenate([tr['state'][0:10], tr['state'][10:13]])
    x13[2] = Z
    return x13


def _trim_x17(speed):
    x17 = kh_find_trim(NATIVE, speed)['state'].copy()
    x17[2] = Z
    return x17


@pytest.mark.parametrize('make_nmpc,make_x', [
    (lambda: VirtualNMPC(CP, v_ref=[80.0, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper'), _trim_x13),
    (lambda: NMPCController(CP, v_ref=[80.0, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper'), _trim_x17),
    (lambda: RotorThrustNMPC13(CP, v_ref=[80.0, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper'), _trim_x13),
])
def test_cold_start_flag_consumed_exactly_once(make_nmpc, make_x):
    """콜드스타트 보정은 인스턴스당 딱 한 번만(첫 _solve) 일어나야 한다 —
    이후 호출까지 매번 실측값으로 덮어쓰면 정상적인 웜스타트 시프트(이전
    솔브 결과 재사용)를 망가뜨린다."""
    nmpc = make_nmpc()
    assert nmpc._cold_start is True
    nmpc._solve(make_x(80.0))
    assert nmpc._cold_start is False
    nmpc.reset()
    assert nmpc._cold_start is True
