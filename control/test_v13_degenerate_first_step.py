"""V13(VirtualNMPC) 80/85 m/s 실패 원인 진단 — kj 작업지시서(2026-09-25 저녁)
3단계(실패 분류) 관련 테스트. **1차 진단은 kj가 직접 재확인해 정정했다 —
이 파일은 그 정정을 반영한 버전이다.**

증상 1(주요 원인, 수정됨): 정확한 트림 상태(섭동 없음)에서도 콜드스타트
웜스타트가 나빠(호버형 고정 추측값) 입력이 박스에 붙고
Maximum_Iterations_Exceeded + NaN 경고가 났다. kj 지적대로 이건 "정확한
해의 성질"이 아니라 순수 수치 실패였다 — `control/hybrid_comparison.py`
등의 `_solve()`에 콜드스타트 보정을 넣어 고쳤다(V13/M17/F13 전체,
`control/test_nmpc_family_cold_start.py` 참고).

증상 2(더 작은 잔여 문제, 미해결): 콜드스타트 수정 후에도 **작은 섭동**
(+0.2m/+0.5m/s)이 있으면 여전히 U_0(적용되는 첫 입력)만 트림값보다
크게 낮게 나온다(예: 80m/s에서 ~0.38N vs 트림 11.3N). 이 파일은 그
원인 후보를 하나씩 제거해 가며 확인한다 — 웜스타트(트림으로 줘도 안
없어짐), 수렴 허용오차(강화해도 안 없어짐)는 기각했고, 논문 식(17)
입력비용 가중치가 U_0에 대한 비용 민감도를 너무 낮춘다는 게 지지되는
가설이다. 결론이 아니다 — NMPC 계열 전체에 적용할 추가 수정 여부는
kj 결정 사항.
"""
import numpy as np
import casadi as ca
import pytest

from models.team_light.control.trim import find_trim as kh_find_trim

from control.kh_adapter import kh_native_params, build_controller_params
from control.hybrid_comparison import VirtualNMPC, NX_V

NATIVE = kh_native_params()
CP = build_controller_params(NATIVE)
SPEED, Z = 80.0, 20.0


def _perturbed_trim():
    tr = kh_find_trim(NATIVE, SPEED)
    x13 = np.concatenate([tr['state'][0:10], tr['state'][10:13]])
    x13[2] = Z
    x0 = x13.copy()
    x0[2] += 0.2
    x0[3] += 0.5
    return tr, x13, x0


def test_trim_is_preserved_by_the_virtual_integrator():
    """웜스타트/모델 불일치 가설을 기각하는 근거 1 — F(x_trim,u_trim)이 트림을
    실제로 보존하는지(diag7 재현). 이게 깨지면 애초에 '트림'이라 부를 수 없다."""
    tr, x_trim13, _ = _perturbed_trim()
    nmpc = VirtualNMPC(CP, v_ref=[SPEED, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper')
    u_trim = np.array([tr['T_total'], 0.0, 0.0, 0.0])
    x_next = np.array(nmpc.F(x_trim13, u_trim)).flatten()
    np.testing.assert_allclose(x_next[3:6], x_trim13[3:6], atol=0.02)
    np.testing.assert_allclose(np.linalg.norm(x_next[6:10]), 1.0, atol=1e-9)


def test_perturbed_case_low_u0_survives_proper_warm_start():
    """증상2 가설 기각 근거 1 — **정확한 트림(섭동 없음)에서는 웜스타트
    수정 하나로 완전히 고쳐졌다**(control/test_nmpc_family_cold_start.py).
    하지만 여기서는 아주 작은 섭동(+0.2m/+0.5m/s)이 있는 상태에서, 콜드
    스타트가 아니라 트림값으로 완벽하게 웜스타트를 줘도(아래처럼 수동
    구성) 증상이 그대로면 — "웜스타트 품질"은 증상2의 원인이 아니다.

    참고: 프로덕션 `_solve()`도 콜드스타트에서 X_k를 실측값(x0, 트림이
    아니라 **섭동이 포함된 값**)으로 덮어쓰므로 이 수동 구성과 결과가
    같다 — 아래 assert는 그 사실 자체도 같이 확인한다.
    """
    tr, x_trim13, x0 = _perturbed_trim()
    nmpc = VirtualNMPC(CP, v_ref=[SPEED, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper')
    N = nmpc.N
    w0 = [x0.tolist()]
    for _ in range(N):
        w0.append([tr['T_total'], 0.0, 0.0, 0.0])
        w0.append(x_trim13.tolist())
    nmpc.w0 = np.array([v for row in w0 for v in row])
    nmpc._cold_start = False   # 수동 웜스타트를 프로덕션 콜드스타트 보정이 덮어쓰지 않게
    vc = nmpc._solve(x0)
    assert nmpc.last_status == 'Solve_Succeeded'
    # 증상 재현: 완벽한(트림) 웜스타트에도 T_cmd가 트림(~11.3N)과 거리가 멀다.
    assert vc[0] < 3.0, (
        f"T_cmd={vc[0]:.3f}N — 트림 웜스타트로도 여전히 낮으면(증상 재현) "
        "'웜스타트 품질'은 증상2의 원인이 아니라는 가설이 유지된다")


def test_tighter_tolerance_does_not_fix_it():
    """증상2 가설 기각 근거 2 — (섭동 있는 경우, 트림 웜스타트를 이미 준
    상태에서) ipopt.tol을 1e-4에서 1e-7로, max_iter를 30에서 300으로
    강화해도 U_0가 그대로면 '느슨한 수렴'은 증상2의 원인이 아니다.

    _build_nlp_paper의 그래프를 그대로 다시 짜되(원본은 안 건드림) tol만
    바꾼다 — 원본 함수가 tol을 인자로 받지 않아서다. 웜스타트는 처음부터
    트림으로 줘서(x_trim13) 증상1(콜드스타트 웜스타트 버그, 이미 수정됨)이
    안 섞이게 한다.
    """
    tr, x_trim13, x0 = _perturbed_trim()
    base = VirtualNMPC(CP, v_ref=[SPEED, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper')
    N, nx, nu = base.N, NX_V, 4
    T_ref, F = base.T_ref, base.F

    D_nu = ca.DM([T_ref, 100.0, 100.0, 100.0])
    nu_h = ca.DM([T_ref, 0.0, 0.0, 0.0])
    T_max = 4 * CP['k_T'] * CP['n_max']**2
    a_max = 100.0
    p = ca.SX.sym('p', nx + 4*(N+1) + nu)
    x_meas = p[0:nx]
    refs = ca.reshape(p[nx:nx+4*(N+1)], 4, N+1)
    nu_prev = p[nx+4*(N+1):]
    w, w0, lbw, ubw = [], [], [], []
    g, lbg, ubg = [], [], []
    J = 0.0

    def new_state(sym_name, guess_state):
        X = ca.SX.sym(sym_name, nx)
        w.append(X)
        lbw.extend([-1e6]*nx); ubw.extend([1e6]*nx)
        w0.extend(guess_state.tolist())
        return X

    X_k = new_state('X0', x_trim13)   # 트림 웜스타트(가설1은 이미 별도로 기각 확인함)
    g.append(X_k - x_meas); lbg.extend([0.]*nx); ubg.extend([0.]*nx)
    U_prev = nu_prev
    for k in range(N):
        e_v = X_k[3:6] - refs[0:3, k]
        e_z = X_k[2] - refs[3, k]
        J += 5.0*ca.sumsqr(e_v) + 20.0*e_z**2 + ca.sumsqr(X_k[10:13])
        U_k = ca.SX.sym(f'U{k}', nu)
        w.append(U_k)
        lbw.extend([0., -a_max, -a_max, -a_max]); ubw.extend([T_max, a_max, a_max, a_max])
        w0.extend([T_ref, 0., 0., 0.])
        J += 0.02*ca.sumsqr((U_k-nu_h)/D_nu) + 0.10*ca.sumsqr((U_k-U_prev)/D_nu)
        X_next = new_state(f'X{k+1}', x_trim13)
        g.append(X_next - F(X_k, U_k)); lbg.extend([0.]*nx); ubg.extend([0.]*nx)
        X_k, U_prev = X_next, U_k
    e_v = X_k[3:6] - refs[0:3, N]; e_z = X_k[2] - refs[3, N]
    J += 10.0*(5.0*ca.sumsqr(e_v) + 20.0*e_z**2 + ca.sumsqr(X_k[10:13]))

    nlp = {'f': J, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': p}
    solver = ca.nlpsol('probe_tight_tol', 'ipopt', nlp, {
        'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
        'ipopt.max_iter': 300, 'ipopt.tol': 1e-7, 'ipopt.acceptable_tol': 1e-6})
    ref_one = np.array([SPEED, 0.0, 0.0, Z])
    p_val = np.concatenate([x0, np.tile(ref_one[:, None], (1, N+1)).ravel(order='F'),
                            [T_ref, 0, 0, 0]])
    sol = solver(x0=np.array(w0), lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=p_val)
    U0 = np.array(sol['x']).flatten()[nx:nx+nu]
    status = solver.stats().get('return_status')
    assert status == 'Solve_Succeeded'
    assert U0[0] < 3.0, (
        f"tol=1e-7/max_iter=300 으로 강화해도 T_cmd={U0[0]:.3f}N 로 낮으면"
        "(증상 재현) '느슨한 허용오차'는 원인이 아니라는 가설이 유지된다")


def _solve_full_T_sequence(speed):
    """주의: `nmpc.solver(...)`를 직접 불러 `_solve()`를 우회한다 — 즉
    콜드스타트 수정(증상1) 이전의 원시 웜스타트(호버형 고정값)로 푼
    해다. 증상1은 이미 고쳤지만, "고치기 전엔 전체 호라이즌이 어떤
    모양이었는지"는 진단 기록으로 남긴다(아래 두 테스트)."""
    tr = kh_find_trim(NATIVE, speed)
    x13 = np.concatenate([tr['state'][0:10], tr['state'][10:13]])
    x13[2] = Z
    x0 = x13.copy(); x0[2] += 0.2; x0[3] += 0.5

    nmpc = VirtualNMPC(CP, v_ref=[speed, 0, 0], z_ref=Z, dt_ctrl=0.02, cost_spec='paper')
    p_val = np.concatenate([x0, nmpc._reference_horizon().ravel(order='F'), nmpc._prev_input])
    sol = nmpc.solver(x0=nmpc.w0, lbx=nmpc.lbw, ubx=nmpc.ubw,
                      lbg=nmpc.lbg, ubg=nmpc.ubg, p=p_val)
    w_opt = np.array(sol['x']).flatten()
    nx, nu = NX_V, 4
    idx = nx  # X_0 다음이 U_0
    Ts = []
    for k in range(nmpc.N):
        Ts.append(w_opt[idx])
        idx += nu + nx
    status = nmpc.solver.stats().get('return_status')
    return np.array(Ts), tr['T_total'], status


def test_80mps_full_horizon_settles_to_trim_after_first_two_steps():
    """핵심 관찰(diag8, 80 m/s) — 전체 20스텝 해 중 k=0,1만 비정상이고 k≥2는
    거의 정확히 트림에 붙는다는 것 자체를 회귀로 고정한다. 이게 '1스텝
    열화' 가설의 직접 증거다. 85 m/s는 양상이 달라(아래 테스트) 따로 둔다."""
    Ts, T_trim, status = _solve_full_T_sequence(80.0)
    assert status == 'Solve_Succeeded'
    np.testing.assert_allclose(Ts[2:], T_trim, rtol=0.2)


def test_85mps_full_horizon_does_not_settle_unlike_80mps():
    """85 m/s는 80 m/s와 다른(더 심한) 양상 — k=2 이후에도 트림에 붙지
    않고 램프업(~40N까지) 후 마지막 몇 스텝에서 거의 0으로 붕괴한다.
    "80/85가 같은 종류의 실패"라고 단정하면 안 된다는 근거로 고정해 둔다
    (kj 3단계: 실패를 시나리오별로 분류할 것 — 85는 별도 조사 필요,
    지지/기각 판정 유보).
    """
    Ts, T_trim, status = _solve_full_T_sequence(85.0)
    assert status in ('Solve_Succeeded', 'Maximum_Iterations_Exceeded')
    # 80 m/s처럼 깔끔히 트림에 붙지 '않는다'는 것 자체가 관찰 대상이다.
    settles_like_80 = np.allclose(Ts[2:], T_trim, rtol=0.2)
    assert not settles_like_80, (
        "85 m/s가 80 m/s와 같은 패턴으로 바뀌었다면 이 회귀가 실패하는 게 "
        "맞다 — 그 경우 로그의 '85m/s는 다른 양상' 기록을 갱신할 것")
