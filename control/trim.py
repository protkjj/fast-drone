"""
트림 조건 탐색 — 축대칭 미사일 + 쿼드콥터
==========================================

주어진 순항 속도 V에서 정상 수평비행(v̇=0, ω̇=0)을 만족하는
(θ_pitch, n_eq, Δn) 를 찾는다.

θ_pitch: 호버로부터의 피치각 (양수=기수 상승, 음수=기수 하강=전방비행)
n_eq:    4로터 평균 속도
Δn:      전방-후방 차동 (n_front = n_eq + Δn, n_rear = n_eq - Δn)
         → 피치 모멘트 균형용 (동체 정적 모멘트 상쇄)
"""

import numpy as np
from scipy.optimize import fsolve
from scipy.spatial.transform import Rotation

from control.vehicle_params import vehicle_params
from control.dynamics import AxialDronePlant, NX


def find_trim(params, V_cruise, guess=None, quiet=False):
    """
    수평 순항 트림 조건 탐색.

    Parameters
    ----------
    params   : dict   기체 파라미터
    V_cruise : float  순항 속도 [m/s]

    Returns
    -------
    trim : dict
        'state':   np.ndarray(17)  트림 상태
        'control': np.ndarray(4)   트림 제어입력
        'theta':   float           피치각 [rad] (음수=전방비행)
        'n_eq':    float           평균 로터속도
        'dn':      float           전방-후방 차동
        'alpha':   float           받음각 [rad]
        'info':    dict            추가 정보
    """
    plant = AxialDronePlant(params, dt=0.001)
    m, g = params['mass'], params['g']
    n_hov = np.sqrt(m * g / (4 * params['k_T']))

    def build_trim_state(theta, n_eq, dn):
        """트림 파라미터 → 17D 상태 벡터 구성."""
        # 자세: 호버(180°about x) + 피치(θ about y)
        R_hover = Rotation.from_quat([1, 0, 0, 0])
        R_pitch = Rotation.from_euler('y', theta)
        R_total = R_hover * R_pitch
        q = R_total.as_quat()

        # 로터: 전방(r1,r2) = n_eq + dn, 후방(r3,r4) = n_eq - dn
        n_front = np.clip(n_eq + dn, params['n_min'], params['n_max'])
        n_rear  = np.clip(n_eq - dn, params['n_min'], params['n_max'])

        x = np.zeros(NX)
        x[3]     = V_cruise        # v_x (관성, 전방)
        x[6:10]  = q
        x[13:17] = [n_front, n_front, n_rear, n_rear]
        return x

    def residual(opt_vars):
        """[v̇_x, v̇_z, ω̇_y] = 0 잔차."""
        theta, n_eq, dn = opt_vars
        x = build_trim_state(theta, n_eq, dn)
        u = x[13:17].copy()        # 정상상태: n_cmd = n
        xdot = plant.evaluate_xdot(x, u)
        return [xdot[3],            # v̇_x = 0 (수평 힘 균형)
                xdot[5],            # v̇_z = 0 (수직 힘 균형)
                xdot[11]]           # ω̇_y = 0 (피치 모멘트 균형)

    # 초기 추정: 약간 기수 하강, 호버 RPM 근처.
    # guess 를 주면 그걸 쓴다 — 연속법(trim_continuation)이 직전 해를 넘긴다.
    # 냉시동으로는 고속에서 fsolve 가 엉뚱한 가지로 빠져 발산한다 (실측:
    # 로켓 배치 83 m/s 에서 잔차 7.6). 브라우저 쪽은 이미 연속법으로 푼다.
    if guess is None:
        x0_guess = [-0.05, n_hov * 1.01, 0.0]
    else:
        x0_guess = list(guess)

    sol, info, ier, msg = fsolve(residual, x0_guess, full_output=True)

    if ier != 1 and not quiet:
        print(f"  [경고] 트림 수렴 실패: {msg}")

    theta_sol, n_eq_sol, dn_sol = sol

    # 트림 상태/제어 구성
    x_trim = build_trim_state(theta_sol, n_eq_sol, dn_sol)
    u_trim = x_trim[13:17].copy()

    # 받음각 계산
    R_trim = Rotation.from_quat(x_trim[6:10]).as_matrix()
    v_body = R_trim.T @ x_trim[3:6]
    alpha = np.arctan2(v_body[2], v_body[0])

    # 잔차 확인
    xdot_check = plant.evaluate_xdot(x_trim, u_trim)
    res_norm = np.linalg.norm([xdot_check[3], xdot_check[5], xdot_check[11]])

    # ★ 왜 안 됐는지를 남긴다. 상한은 회전수 천장이 아니라 **후방 로터가
    #   음추력을 요구**하는 데서 온다 (로켓 배치 52.09 m/s 에서 T_rear=0,
    #   60 m/s 에서 -4.45 N). 그런데 trim_speed_sweep 은 천장만 보므로
    #   "✗ 수렴 실패" 로만 찍히고 진짜 이유가 감춰졌다.
    n_front_s = n_eq_sol + dn_sol
    n_rear_s = n_eq_sol - dn_sol
    why = ""
    if n_rear_s < params['n_min'] - 1e-9:
        why = f"후방 로터가 하한 미만 ({n_rear_s:.1f} < {params['n_min']:.1f}) — 음추력 요구"
    elif n_front_s > params['n_max'] + 1e-9:
        why = f"전방 로터가 상한 초과 ({n_front_s:.1f} > {params['n_max']:.1f})"
    elif res_norm > 1e-6:
        why = "뉴턴 미수렴"

    return {
        'converged': bool(ier == 1 and res_norm <= 1e-6 and not why),
        'why':     why,
        'guess':   [theta_sol, n_eq_sol, dn_sol],
        'state':   x_trim,
        'control': u_trim,
        'theta':   theta_sol,
        'n_eq':    n_eq_sol,
        'dn':      dn_sol,
        'alpha':   alpha,
        'v_body':  v_body,
        'residual': res_norm,
        'xdot':    xdot_check,
    }


def trim_continuation(params, V_cruise, step=5.0, quiet=True):
    """0 에서 V_cruise 까지 걸어 올라가며 직전 해를 다음 초기값으로 쓴다.

    트림 곡선을 따라가는 표준 방법이다. 냉시동 fsolve 는 고속에서 엉뚱한
    가지로 빠진다 — 로켓 배치 83 m/s 를 바로 주면 잔차 7.6 으로 발산하고,
    기수가 뒤를 보는(u_b<0) 가짜 해로 '수렴' 하기도 한다. 그러면 항력이
    추진력으로 둔갑한다.

    Returns
    -------
    (trim, reached) : 마지막으로 성공한 트림과 그때의 속도.
        reached < V_cruise 면 그 위에는 정상비행점이 없다는 뜻이다.
    """
    guess, last, reached = None, None, 0.0
    v = 0.0
    while True:
        v = min(V_cruise, v + step) if v > 0 or step <= V_cruise else V_cruise
        if v <= 0:
            v = min(step, V_cruise)
        tr = find_trim(params, v, guess=guess, quiet=quiet)
        if not tr['converged']:
            break
        last, reached, guess = tr, v, tr['guess']
        if v >= V_cruise - 1e-9:
            break
    if last is None:
        last = find_trim(params, 0.0, quiet=quiet)
        reached = 0.0
    return last, reached


def print_trim(trim, V_cruise, params):
    """트림 결과 출력."""
    m, g = params['mass'], params['g']
    n_hov = np.sqrt(m * g / (4 * params['k_T']))

    T_per = params['k_T'] * trim['n_eq']**2
    T_total = 4 * T_per

    print(f"  순항 속도      = {V_cruise:.1f} m/s ({V_cruise*3.6:.0f} km/h)")
    print(f"  피치각 θ       = {np.degrees(trim['theta']):.3f}°")
    print(f"  받음각 α       = {np.degrees(trim['alpha']):.3f}°")
    print(f"  평균 RPM n_eq  = {trim['n_eq']:.1f} rad/s  (호버: {n_hov:.1f})")
    print(f"  차동 Δn        = {trim['dn']:.2f} rad/s")
    print(f"  전방 로터      = {trim['n_eq']+trim['dn']:.1f} rad/s")
    print(f"  후방 로터      = {trim['n_eq']-trim['dn']:.1f} rad/s")
    print(f"  총 추력        = {T_total:.1f} N  (무게: {m*g:.1f} N)")
    print(f"  v_body         = [{trim['v_body'][0]:.2f}, {trim['v_body'][1]:.2f}, {trim['v_body'][2]:.2f}]")
    print(f"  잔차 ‖res‖     = {trim['residual']:.2e}")


def trim_speed_sweep(params, V_range=None, verbose=True):
    """
    트림 속도 스윕: 이 기체의 물리적 최고 트림속도를 특정한다.

    반환: 각 속도별 트림 결과 리스트 + 최대 트림속도
    """
    if V_range is None:
        V_range = np.arange(0, 90, 5)

    results = []
    prev_trim = None

    for V in V_range:
        trim = find_trim(params, V)
        converged = trim['residual'] < 1e-4

        # 로터 포화 확인: 후방 로터 = n_eq - dn (dn이 음수이므로 후방이 큼)
        n_rear = trim['n_eq'] - trim['dn']
        n_front = trim['n_eq'] + trim['dn']
        n_max_actual = max(abs(n_rear), abs(n_front))
        saturated = n_max_actual > params['n_max'] * 0.95

        results.append({
            'V': V,
            'trim': trim,
            'converged': converged,
            'n_max_actual': n_max_actual,
            'saturated': saturated,
        })

        if verbose:
            flag = "✓" if converged and not saturated else ("⚠SAT" if saturated else "✗")
            print(f"  {V:5.0f} m/s | θ={np.degrees(trim['theta']):+7.2f}° "
                  f"| Δn={trim['dn']:+8.1f} | n_max={n_max_actual:7.1f} "
                  f"| res={trim['residual']:.1e} | {flag}")

    # 최대 트림속도 찾기
    valid = [r for r in results if r['converged'] and not r['saturated']]
    V_max_trim = valid[-1]['V'] if valid else 0.0

    return results, V_max_trim


# ══════════════════════════════════════════════════
if __name__ == '__main__':
    print("\n트림 속도 스윕 — 물리적 최고 트림속도 특정\n" + "=" * 60)
    print(f"  {'속도':>5s} | {'피치':>8s} | {'Δn':>8s} | {'n_max':>7s} | {'잔차':>9s} |")
    print(f"  {'─'*56}")

    results, V_max = trim_speed_sweep(vehicle_params,
                                       V_range=np.arange(0, 90, 5))

    print(f"\n{'='*60}")
    print(f"  이 기체의 물리적 최고 트림속도: {V_max:.0f} m/s ({V_max*3.6:.0f} km/h)")
    print(f"  n_max = {vehicle_params['n_max']:.0f} rad/s")

    if V_max < 83:
        print(f"\n  ⚠ 300 km/h(83 m/s) 트림 불가!")
        print(f"    원인: 로터 포화 (n_rear → n_max)")
        print(f"    의미: 이 플레이스홀더 파라미터로는 300 km/h 정상비행점 자체가 없음")
        print(f"    → NMPC를 붙여도 물리적으로 불가능한 속도")
        print(f"    → 파라미터 조정 필요 (n_max↑, k_T↑, C_A0↓, 또는 공력팀 실제 값)")
    else:
        print(f"\n  ✓ 300 km/h 트림 가능 → NMPC로 제어 가능성 있음")
    print(f"{'='*60}")
