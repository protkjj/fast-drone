"""표6 사다리의 S1(시간 정렬)과 C2(첫 입력 배분값 근처 제한) 검증.

논문 표6의 제거실험 사다리는 세 축으로 이루어진다.
  V13-0 → V13-1 : A0 → A1  (총추력 우선 배분)        → test_allocation_a1.py
  V13-1 → V13-2 : C0 → C1  (배분 결과 피드백)         → test_allocation_a1.py
  V13-2 → V13   : S0 → S1  (시간 정렬, 식22-23)       → 이 파일
  V13   → V13-C2: 연결 강도 (5.4절 변형)              → 이 파일

두 플래그 모두 기본값이 '기존 동작'이어야 한다. 기존 실험 결과(results/)가
플래그 추가만으로 바뀌면 사다리의 어느 칸이 원인인지 못 가리기 때문이다.
"""
import numpy as np
import pytest

from control.vehicle_params import vehicle_params as P
from control.hybrid_comparison import ProperHybrid, VirtualNMPC


class _StubNMPC:
    """결정적 가상명령. IPOPT를 빼고 INDI 측정 경로만 시험한다."""

    def __init__(self, params):
        self.mg = params['mass'] * params['g']

    def __call__(self, t, x):
        return np.array([self.mg, 0.0, 0.0, 0.0])

    def set_prev_input(self, v):
        pass


def _rotor_thrust(params, n, V_axial):
    """ProperHybrid 안의 전진비 보정 추력식을 독립적으로 재계산한다.

    같은 함수를 부르면 서로를 검증하지 못하므로 손으로 다시 쓴다.
    """
    n = np.asarray(n, dtype=float)
    n_rps = n / (2 * np.pi)
    J = V_axial / (n_rps * params['D_prop'] + 1e-8)
    fac = np.maximum(1.0 - J / params['J_max'], 0.0)
    return params['k_T'] * n**2 * fac


# z축 기체의 호버 자세. scalar-last 규약이라 [1,0,0,0]은 단위 쿼터니언이
# 아니라 x축 180° 회전이다 — 동체 z-down을 세계 z-up으로 뒤집어, 동체 -z
# 추력이 세계 +z(위)를 향하게 한다. find_trim(V=0)이 정확히 이 값을 낸다.
# 여기에 단위 쿼터니언 [0,0,0,1]을 넣으면 기체가 거꾸로라 NMPC가 T=0을
# 고르고 뒤집으려 α를 포화시킨다(실제로 한 번 당했다).
Q_HOVER_Z = [1.0, 0.0, 0.0, 0.0]


def _state(n, v=(0.0, 0.0, 0.0), omega=(0.0, 0.0, 0.0), z=50.0):
    x = np.zeros(17)
    x[2] = z
    x[3:6] = v
    x[6:10] = Q_HOVER_Z
    x[10:13] = omega
    x[13:17] = n
    return x


# ── S0/S1 ────────────────────────────────────────────────────────────

def test_s0_is_the_default():
    """플래그를 안 주면 기존 동작이어야 한다."""
    h = ProperHybrid(_StubNMPC(P), P)
    assert h.time_align == 'S0'
    assert h._f_prev is None and h._f_filt is None   # S1 상태는 만들지도 않음


def test_bad_time_align_is_rejected():
    with pytest.raises(ValueError, match='time_align'):
        ProperHybrid(_StubNMPC(P), P, time_align='S2')


def test_s1_lpf_coefficient_follows_eq23():
    """식(23) λ = 1 − exp(−2π f_c Δt). S0의 후향차분 근사와 구별된다."""
    dt, f_cut = 1e-3, 50.0
    s1 = ProperHybrid(_StubNMPC(P), P, dt=dt, f_cut=f_cut, time_align='S1')
    s0 = ProperHybrid(_StubNMPC(P), P, dt=dt, f_cut=f_cut, time_align='S0')

    assert s1._lpf_coeff(dt) == pytest.approx(1.0 - np.exp(-2*np.pi*f_cut*dt))
    assert s0._lpf_coeff(dt) == pytest.approx(dt / (dt + 1.0/(2*np.pi*f_cut)))
    # 같은 필터의 다른 이산화 — 가깝지만 같지 않다(1 ms/50 Hz에서 약 12%).
    assert s0._lpf_coeff(dt) < s1._lpf_coeff(dt)
    assert s1._lpf_coeff(dt) / s0._lpf_coeff(dt) == pytest.approx(1.126, abs=2e-3)


def test_s1_applies_mid_sample_alignment_and_the_same_filter():
    """식(22) f_mid=(f_k+f_{k-1})/2 와 식(23) 필터를 손계산과 대조한다."""
    dt = 1e-3
    h = ProperHybrid(_StubNMPC(P), P, dt=dt, time_align='S1')
    n_eq = np.sqrt(P['mass']*P['g'] / (4*P['k_T']))
    seq = [n_eq*np.ones(4), n_eq*np.array([1.05, 1.0, .95, 1.0]),
           n_eq*np.array([1.10, 1.0, .90, 1.0])]

    # 1번째 호출은 초기화(_fallback)라 S1 상태를 건드리지 않는다.
    h(0.0, _state(seq[0]))
    assert h._f_prev is None

    # 2번째: f_{k-1}이 없으니 f_mid=f_k, 필터도 그 값으로 초기화된다.
    h(dt, _state(seq[1]))
    f2 = _rotor_thrust(P, seq[1], 0.0)
    np.testing.assert_allclose(h._f_prev, f2, rtol=1e-12)
    np.testing.assert_allclose(h._f_filt, f2, rtol=1e-12)

    # 3번째: 중간값 정렬 + 각가속도와 같은 계수의 LPF.
    h(2*dt, _state(seq[2]))
    f3 = _rotor_thrust(P, seq[2], 0.0)
    alpha = 1.0 - np.exp(-2*np.pi*50.0*dt)
    expected = alpha*0.5*(f3 + f2) + (1 - alpha)*f2
    np.testing.assert_allclose(h._f_filt, expected, rtol=1e-12)


def test_s1_thrust_filter_starts_at_hover_not_zero():
    """LPF를 0에서 시작하면 INDI가 '없는 추력 부족'을 크게 본다."""
    h = ProperHybrid(_StubNMPC(P), P, time_align='S1')
    n_eq = np.sqrt(P['mass']*P['g'] / (4*P['k_T']))
    h(0.0, _state(n_eq*np.ones(4)))
    h(1e-3, _state(n_eq*np.ones(4)))
    # 첫 유효 표본에서 이미 호버 추력 근처여야 한다(0에서 올라오지 않는다).
    assert h._f_filt.sum() == pytest.approx(P['mass']*P['g'], rel=1e-6)


def test_s1_matches_s0_on_smooth_input_and_diverges_on_noise():
    """S1의 효과는 측정 잡음에 비례한다 — 논문 4.5절의 '가상 외란' 주장."""
    n_eq = np.sqrt(P['mass']*P['g'] / (4*P['k_T']))

    def run(align, jitter, seed=3):
        h = ProperHybrid(_StubNMPC(P), P, dt=1e-3, alloc_mode='A1',
                         time_align=align)
        rng = np.random.default_rng(seed)
        out = []
        for k in range(600):
            t = k*1e-3
            n = n_eq*(1 + .05*np.sin(2*np.pi*2*t + np.arange(4)))
            if jitter:
                n = n*(1 + jitter*rng.standard_normal(4))
            out.append(h(t, _state(n, v=(12., 0., 0.),
                                   omega=(.2*np.sin(4*t), 0., 0.))))
        return np.array(out)

    smooth = np.abs(run('S0', 0.0) - run('S1', 0.0)).max()
    noisy = np.abs(run('S0', 0.02) - run('S1', 0.02)).max()
    # 매끄러운 신호에서는 위상 보정 수준(회전수 상한의 1% 미만).
    assert smooth < 0.01*P['n_max']
    # 잡음이 있으면 뚜렷하게 갈린다(필터가 지터를 걸러내므로).
    assert noisy > 5*smooth


def test_reset_clears_s1_state():
    """MC 시행 간 독립성 — 남은 필터 상태가 다음 시행에 새면 안 된다."""
    h = ProperHybrid(_StubNMPC(P), P, time_align='S1')
    n_eq = np.sqrt(P['mass']*P['g'] / (4*P['k_T']))
    h(0.0, _state(n_eq*np.ones(4)))
    h(1e-3, _state(n_eq*np.ones(4)))
    assert h._f_filt is not None
    h.reset()
    assert h._f_prev is None and h._f_filt is None


# ── C0/C1/C2 ─────────────────────────────────────────────────────────

def test_c2_is_off_by_default():
    """기본값에서는 제약이 추가되지 않아야 한다(g 차원 불변)."""
    base = VirtualNMPC(P, v_ref=[15, 0, 0], z_ref=50.)
    assert base.c2_limit is None
    with_c1 = VirtualNMPC(P, v_ref=[15, 0, 0], z_ref=50., alloc_feedback=True)
    assert with_c1.lbg.size == base.lbg.size


def test_c2_requires_c1_and_a_positive_limit():
    """C1 없이 C2를 켜면 '트림 주변 제한'이 되어 다른 실험이 된다."""
    with pytest.raises(ValueError, match='alloc_feedback'):
        VirtualNMPC(P, c2_limit=0.5)
    with pytest.raises(ValueError, match='positive'):
        VirtualNMPC(P, c2_limit=0.0, alloc_feedback=True)


@pytest.mark.parametrize('limit', [0.30, 0.05])
def test_c2_constrains_the_first_input_near_the_allocation(limit):
    """‖Dν⁻¹(ν_0 − ν_alloc)‖ ≤ limit 이 실제로 구속되는지.

    냉시동은 기본 max_iter=30으로 수렴하지 못해 근사 실행가능해에 머문다.
    여기서는 제약식 자체를 보려는 것이므로 반복 상한을 넉넉히 준다.
    """
    mg = P['mass']*P['g']
    D_nu = np.array([mg, 100., 100., 100.])
    nu_alloc = np.array([0.70*mg, 0., 0., 0.])   # NMPC가 원하는 값에서 멀리
    x = _state(np.sqrt(mg/(4*P['k_T']))*np.ones(4), v=(12., 0., 0.))

    free = VirtualNMPC(P, v_ref=[15, 0, 0], z_ref=50.,
                       alloc_feedback=True, max_iter=300)
    free.set_prev_input(nu_alloc)
    d_free = np.linalg.norm((free(0., x) - nu_alloc) / D_nu)

    c2 = VirtualNMPC(P, v_ref=[15, 0, 0], z_ref=50., alloc_feedback=True,
                     c2_limit=limit, max_iter=300)
    c2.set_prev_input(nu_alloc)
    d_c2 = np.linalg.norm((c2(0., x) - nu_alloc) / D_nu)

    assert c2.lbg.size == free.lbg.size + 1      # 제약 한 줄 추가
    assert d_free > limit                        # 제한이 의미 있는 상황인지 먼저
    assert d_c2 <= limit + 1e-6                  # 만족
    assert d_c2 == pytest.approx(limit, rel=1e-3)  # 활성 제약이라 경계에 붙는다


# ── cost_spec='paper' (식13-18·31) ────────────────────────────────────

def test_paper_is_the_default_cost_spec():
    """2026-09-24부터 기본값이 논문식이다. 기존 결과 재현용 legacy는 남아 있다."""
    m = VirtualNMPC(P, v_ref=[12, 0, 0], z_ref=50.)
    assert m.cost_spec == 'paper'
    assert m.w0.size == 21*13 + 20*4          # 식(31) n_var,13 = 353
    old = VirtualNMPC(P, v_ref=[12, 0, 0], z_ref=50., cost_spec='legacy')
    assert old.w0.size == 20*13 + 20*4        # x₀가 파라미터인 옛 배치


def test_paper_mode_problem_size_matches_eq31():
    """식(31) n_var,13 = (N+1)·13 + N·4 = 353. x₀를 결정변수에 넣어야 나온다."""
    m = VirtualNMPC(P, v_ref=[12, 0, 0], z_ref=50., cost_spec='paper')
    assert m.w0.size == (m.N + 1)*13 + m.N*4 == 353
    assert m.lbg.size == (m.N + 1)*13         # 초기상태 등식 + N개 전이


def test_paper_mode_input_bounds_match_eq18():
    """식(18) 0 ≤ T ≤ T_max,0 및 |α_i| ≤ 100 rad/s²."""
    m = VirtualNMPC(P, v_ref=[12, 0, 0], z_ref=50., cost_spec='paper')
    nx = 13
    assert m.lbw[nx] == 0.0
    assert m.ubw[nx] == pytest.approx(4*P['k_T']*P['n_max']**2)
    np.testing.assert_allclose(m.ubw[nx+1:nx+4], 100.0)
    np.testing.assert_allclose(m.lbw[nx+1:nx+4], -100.0)


def test_paper_mode_cost_is_exactly_eq14_to_eq18():
    """비용함수를 논문식 손계산과 대조한다 — 전사가 맞는지의 직접 증거."""
    m = VirtualNMPC(P, v_ref=[12, 0, 0], z_ref=50., cost_spec='paper')
    N, nx, nu = m.N, 13, 4
    J_fn = m.solver.get_function('nlp_f')

    rng = np.random.default_rng(11)
    w = m.w0 + rng.standard_normal(m.w0.size)
    refs = rng.standard_normal((4, N+1))*3 + np.array([[12.], [0.], [0.], [50.]])
    nu_prev = np.array([0.9*P['mass']*P['g'], 1., -2., .5])
    x_meas = np.concatenate([_state(np.zeros(4))[0:10],
                             _state(np.zeros(4))[10:13]])
    p_val = np.concatenate([x_meas, refs.ravel(order='F'), nu_prev])

    # 손계산: 배치는 X_0,U_0,X_1,U_1,…,X_{N-1},U_{N-1},X_N
    mg = P['mass']*P['g']
    D_nu = np.array([mg, 100., 100., 100.])      # 식(15)
    nu_h = np.array([mg, 0., 0., 0.])
    Xs, Us, i = [], [], 0
    for _ in range(N):
        Xs.append(w[i:i+nx]); i += nx
        Us.append(w[i:i+nu]); i += nu
    Xs.append(w[i:i+nx]); i += nx
    assert i == w.size

    def stage(X, k):                             # 식(14)
        return (5.0*np.sum((X[3:6] - refs[0:3, k])**2)
                + 20.0*(X[2] - refs[3, k])**2
                + np.sum(X[10:13]**2))

    expected, u_prev = 0.0, nu_prev
    for k in range(N):
        expected += stage(Xs[k], k)
        expected += 0.02*np.sum(((Us[k] - nu_h)/D_nu)**2)    # 식(17) 1항
        expected += 0.10*np.sum(((Us[k] - u_prev)/D_nu)**2)  # 식(17) 2항
        u_prev = Us[k]
    expected += 10.0*stage(Xs[N], N)             # 식(16) 종말 10배 (ω 포함)

    assert float(J_fn(w, p_val)[0]) == pytest.approx(expected, rel=1e-12)


def test_paper_mode_hovers_at_mg():
    """호버 트림에서 T=mg, α=0 — 부호·축이 뒤집혔으면 여기서 깨진다."""
    from control.trim import find_trim
    tr = find_trim(P, 0.0, quiet=True)
    assert tr['converged']
    x = np.zeros(17)
    x[2] = 50.
    x[6:10] = tr['state'][6:10]
    x[13:17] = tr['state'][13:17]

    m = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=50., cost_spec='paper',
                    max_iter=300)
    u = m(0., x)
    assert m.last_status == 'Solve_Succeeded'
    assert u[0] == pytest.approx(P['mass']*P['g'], rel=1e-4)
    np.testing.assert_allclose(u[1:4], 0.0, atol=1e-5)


def test_ref_fn_fills_the_prediction_horizon():
    """식(14)의 r_{j|k} — 노드마다 참조가 달라야 램프를 추종할 수 있다."""
    m = VirtualNMPC(P, cost_spec='paper',
                    ref_fn=lambda t: (5. + 2.*t, 0., 0., 50.))
    m._t_now = 1.0
    R = m._reference_horizon()
    assert R.shape == (4, m.N + 1)
    # t=1.0부터 dt_nmpc=0.05 간격
    np.testing.assert_allclose(R[0], 5. + 2.*(1.0 + 0.05*np.arange(m.N+1)))
    np.testing.assert_allclose(R[3], 50.)

    # ref_fn이 없으면 상수 참조로 모든 노드를 채운다
    c = VirtualNMPC(P, v_ref=[9, 1, 0], z_ref=30., cost_spec='paper')
    Rc = c._reference_horizon()
    np.testing.assert_allclose(Rc, np.tile([[9.], [1.], [0.], [30.]], (1, c.N+1)))


def test_ref_fn_is_rejected_in_legacy_mode():
    """legacy 경로는 참조가 상수 하나뿐이라 노드별 참조를 받을 수 없다."""
    with pytest.raises(ValueError, match='ref_fn'):
        VirtualNMPC(P, cost_spec='legacy', ref_fn=lambda t: (0., 0., 0., 0.))


def test_bad_cost_spec_is_rejected():
    with pytest.raises(ValueError, match='cost_spec'):
        VirtualNMPC(P, cost_spec='v53')


# ── 비교군 간 비용함수 일치 ───────────────────────────────────────────

def test_every_comparison_controller_declares_its_cost_spec():
    """표5·표6 비교군이 같은 비용을 쓰는지 프로그램적으로 확인할 수 있어야 한다.

    논문 §5.3: "원인 분석 모드에서는 플랜트, 상태 정보, 참조, 예측 구간, 명목
    공력, 입력 갱신률과 솔버 설정을 일치시키고 한 요소씩 바꾼다." 비용함수도
    그 '일치시킬 것'에 들어간다 — 한쪽만 논문식이면 그 차이가 구조 차이로
    오인된다.

    2026-09-25 밤(야간지시 3-a): M17·F13에도 cost_spec='paper'를 추가하고
    기본값을 뒤집어 V13과 맞췄다 — 이제 비교군 전체가 일치를 **요구**한다.
    (acados 경로 AcadosVirtualNMPC는 여전히 legacy다 — 별도 클래스라 이
    전환을 따라오지 못했다. control/vnmpc_acados.py의 경고 주석 참고.)
    """
    from control.nmpc import NMPCController
    from control.nmpc_f13 import RotorThrustNMPC13

    declared = {
        'V13': VirtualNMPC(P).cost_spec,
        'M17': NMPCController(P).cost_spec,
        'F13': RotorThrustNMPC13(P).cost_spec,
    }
    assert declared == {'V13': 'paper', 'M17': 'paper', 'F13': 'paper'}, (
        f"비교군 비용함수가 다시 갈라졌다: {declared}. 표5·표6 비교 전에는 "
        "반드시 전부 일치해야 한다.")
