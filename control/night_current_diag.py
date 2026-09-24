"""야간 작업 단계1 — 고속 트림가지 모터 전류 진단 (PILOT, 진단 전용).

전류를 제약으로 넣지 않는다 — research 의 완전 분석(모멘트+전류)이 83.3 m/s
근방을 전류로 막는다고 본 것과, control 이 찾은 고속가지(67.99~90 m/s)가
전류를 검사하지 않는다는 사실 사이의 간극을 **정량화**하는 것이 목적이다.
결정(A: 전류 제외 유지 / B: 재도입)은 여기서 내리지 않는다 — 근거만 만든다.

전류 계산은 control/battery.py 의 고정점 반복(식11-14)과 동일한 식을 쓰되,
로터별 반작용 토크는 그 파일의 상수 k_Q·(1-J/J_max) 근사 대신 **표
기반(CP(J))** 값을 쓴다 — research 의 45.61A 도 표 기반이므로 같은 기준으로
비교해야 원인 규명이 의미 있다.

실행: python3 -m control.night_current_diag
"""
import numpy as np

from control.trim import find_trim
from control.vehicle_params import load_selected_params

P = load_selected_params()
n_hov = np.sqrt(P['mass']*P['g']/(4*P['k_T']))


def table_torque(n, V_axial, D, rho, J_grid, CP_grid, CT_grid):
    """CP(J) 표로 로터 반작용 토크. control/dynamics_hifi.py 와 같은 규약
    (모터링 사분면만, ct 하한 0, cp 하한 J*ct+.005)."""
    rev = max(n, 0.0)/(2*np.pi)
    J = V_axial/(rev*D + 1e-8)
    ct = max(float(np.interp(J, J_grid, CT_grid)), 0.0)
    cp = max(float(np.interp(J, J_grid, CP_grid)), J*ct + .005)
    return cp*rho*rev**2*D**5/(2*np.pi), J


def motor_currents(n_vec, Q_vec, p):
    """control/battery.py::BatteryModel.step() 의 정상상태(n_cmd=n, dn/dt=0)
    고정점 반복을 그대로 재현한다 — 재사용하지 않고 다시 쓰는 이유는 그 함수가
    Q 를 내부에서 k_Q*n**2 로 직접 계산해 표 기반 Q 를 주입할 수 없어서다.
    """
    k_t, k_e, R_m = p['k_t'], p['k_e'], p['R_m']
    I_lim, V_oc, R_b = p['I_lim'], p['V_oc'], p['R_b']
    eta_esc = p['eta_esc']
    I_req = Q_vec/k_t
    I_cap = np.clip(I_req, 0.0, I_lim)
    V_b = V_oc
    I_i = np.zeros(4)
    for _ in range(6):   # 고정점 6회 — 3회(battery.py 기본)보다 넉넉히
        I_V = np.maximum((V_b - k_e*n_vec)/R_m, 0.0)
        I_i = np.minimum(I_cap, I_V)
        P_e = float(np.sum((k_e*n_vec + R_m*I_i)*I_i))/eta_esc
        I_b = P_e/max(V_b, 1e-3)
        V_b = V_oc - R_b*I_b
    return I_i, V_b


def diagnose(V, guess):
    tr = find_trim(P, V, guess=guess, quiet=True)
    if not tr['converged']:
        return None
    x = tr['state']
    n = x[13:17]
    theta_deg = np.degrees(tr['theta'])
    alpha_deg = np.degrees(tr['alpha']) if tr['alpha'] is not None else float('nan')

    from scipy.spatial.transform import Rotation
    R = Rotation.from_quat(x[6:10]).as_matrix()
    v_body = R.T @ x[3:6]
    V_axial = max(v_body[0], 0.0)

    J_grid, CT_grid, CP_grid = (P['prop_table']['J'], P['prop_table']['CT'],
                                P['prop_table']['CP'])
    Q_vec = np.zeros(4)
    J_each = np.zeros(4)
    for i in range(4):
        Q_vec[i], J_each[i] = table_torque(n[i], V_axial, P['D_prop'], P['rho'],
                                           J_grid, CP_grid, CT_grid)
    I_i, V_b = motor_currents(n, Q_vec, P)

    return {
        'V': V, 'converged': True, 'theta_deg': theta_deg, 'alpha_deg': alpha_deg,
        'n': n.copy(), 'J_each': J_each, 'Q_each': Q_vec, 'I_each': I_i,
        'I_over_limit_pct': 100*I_i/P['I_lim'], 'V_bus': V_b,
        'front_rear_thrust_ratio': None,   # 아래서 채움
    }


def main():
    speeds = np.arange(68.0, 90.001, 2.0)
    print(f"고속 트림가지 전류 진단 (I_lim={P['I_lim']} A — research 4절: 제품 정격 아님, "
          f"웹연구모델 가정)")
    print(f"{'V':>5} {'θ':>7} {'α':>6} {'n_front':>8} {'n_rear':>8} "
          f"{'I_front':>8} {'I_rear':>8} {'I/한계%(최대)':>13} {'V_bus':>7}")
    rows = []
    guess = [np.radians(88.0), n_hov*1.6, 0.0]
    for V in speeds:
        r = diagnose(V, guess)
        if r is None:
            # 재시도: 여러 시드
            for th in (70, 75, 80, 85, 87, 89):
                r = diagnose(V, [np.radians(th), n_hov*1.5, 0.0])
                if r is not None:
                    break
        if r is None:
            print(f"{V:5.1f}  트림 실패")
            continue
        guess = [np.radians(r['theta_deg']), np.mean(r['n']), (r['n'][0]-r['n'][2])/1]
        rows.append(r)
        i_pct_max = r['I_over_limit_pct'].max()
        print(f"{V:5.1f} {r['theta_deg']:7.3f} {r['alpha_deg']:6.3f} "
              f"{r['n'][0]:8.1f} {r['n'][2]:8.1f} "
              f"{r['I_each'][0]:8.3f} {r['I_each'][2]:8.3f} {i_pct_max:12.1f}% {r['V_bus']:7.3f}")

    print(f"\n85 m/s 상세:")
    r85 = next((r for r in rows if abs(r['V']-85.0) < 1e-6), None) or diagnose(85.0, guess)
    if r85:
        print(f"  로터 회전수 n = {np.round(r85['n'],2)} rad/s (n_max={P['n_max']:.1f})")
        print(f"  전진비 J = {np.round(r85['J_each'],4)}")
        print(f"  반작용 토크 Q(표 기반) = {np.round(r85['Q_each'],4)} N·m")
        print(f"  로터별 전류 = {np.round(r85['I_each'],3)} A  (한계 {P['I_lim']} A)")
        thrust_ratio = (r85['n'][0]/r85['n'][2])**2
        print(f"  전방:후방 추력비(근사, n² 비례) = {thrust_ratio:.4f} : 1"
              f"  (research 83.3m/s 보고값은 0.142:6.373 ≈ 1:45, 훨씬 더 쏠림)")

    return rows


if __name__ == '__main__':
    main()
