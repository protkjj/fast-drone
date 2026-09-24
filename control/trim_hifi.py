"""표 기반(고충실도) 물리로 직접 트림을 푼다 — `control/trim.py`의 `find_trim`과
같은 3식(v̇x=0, v̇z=0, ω̇y=0)이지만 플랜트를 `AxialDronePlantHighFidelity`로
바꾼다. `control/trim.py`는 건드리지 않는다 — 그쪽은 모든 제어기가 공유하는
단순화 모델 기준 트림이고, 이 파일은 진단 전용(플랜트가 실제로 이 상태를
유지하는지 확인)이다.

2026-09-25 밤 작업에서 이 모듈이 필요해진 이유: 단순화 모델(fac=max(1-J/J_max,0))
은 전진비가 아무리 커져도 추력이 **절대 음수가 되지 않는다.** 실제 프로펠러는
J≈1.544 를 넘으면 추력이 실제로 반대 방향이 된다(표 영점). 그 결과 단순화
모델로 찾은 "고속 트림가지"의 일부는 J가 표 유효범위(2.4)를 한참 넘는
지점이었고 — 하이파이 플랜트로 잔차를 평가하면 v̇x 가 -2.6~-7.9 m/s² 로
크게 어긋났다(전혀 트림이 아니었다). 이 모듈로 다시 푼 결과, 진짜 고속가지는
**81.14 m/s 재개방**(단순화 모델 기준 67.99 가 아니라)이었고, 저속가지 상한은
19.626(하이파이) — research 자체 보고값 19.63 과 0.005 m/s 이내로 일치해
이 솔버의 신뢰성을 뒷받침한다.
"""
import numpy as np
from scipy.optimize import fsolve
from scipy.spatial.transform import Rotation

from control.dynamics_hifi import AxialDronePlantHighFidelity

# 기본 시드 격자 — theta(x축 관례: 0=호버/수직, 90=완전수평) × n_eq 배율.
# 연속법이 아니라 매 속도 독립적으로 이 격자 전체를 시도해, 저속가지에
# 갇혀 고속가지를 못 보는 사고(그리고 그 반대)를 피한다.
DEFAULT_THETA_DEG = (0, 5, 10, 20, 30, 40, 50, 60, 65, 70, 75, 78, 80, 82, 84, 86, 87, 88, 89)
DEFAULT_N_SCALE = (0.7, 0.9, 1.0, 1.1, 1.3, 1.5, 1.7, 1.9)


def _build_state(V, theta, n_eq, dn, params):
    R_hover = Rotation.from_quat([0.0, -np.sqrt(0.5), 0.0, np.sqrt(0.5)])
    q = (R_hover * Rotation.from_euler('y', theta)).as_quat()
    n_pos = np.clip(n_eq + dn, params['n_min'], params['n_max'])
    n_neg = np.clip(n_eq - dn, params['n_min'], params['n_max'])
    x = np.zeros(17)
    x[3] = V
    x[6:10] = q
    x[13:17] = [n_pos, n_pos, n_neg, n_neg]
    return x


def default_seeds(params, theta_deg=DEFAULT_THETA_DEG, n_scale=DEFAULT_N_SCALE):
    n_hov = np.sqrt(params['mass']*params['g']/(4*params['k_T']))
    return [(np.radians(th), n_hov*ns, 0.0) for th in theta_deg for ns in n_scale]


def find_trim_hifi(params, V, seeds=None, xtol=1e-12, residual_tol=1e-4):
    """표 기반 플랜트에서 트림을 직접 찾는다(연속법 아님 — 매 속도 독립 탐색).

    반환: {'converged','theta','state','control','residual_norm'} 또는
    수렴 실패면 {'converged': False}. `find_trim`과 달리 여러 시드 중 잔차가
    가장 작은 해를 고른다 — 방정식이 여러 고립근을 가질 수 있어서다(실제로
    저속가지·고속가지가 같은 V 근방에서 둘 다 근이 되는 경우는 없었지만,
    단순화 모델에서는 있었다 — 그게 이 파일이 필요해진 이유).
    """
    if seeds is None:
        seeds = default_seeds(params)
    plant = AxialDronePlantHighFidelity(params, dt=0.001)

    def residual(v):
        theta, n_eq, dn = v
        x = _build_state(V, theta, n_eq, dn, params)
        xdot = plant.evaluate_xdot(x, x[13:17])
        return [xdot[3], xdot[5], xdot[11]]

    best = None
    for guess in seeds:
        sol, info, ier, msg = fsolve(residual, guess, full_output=True, xtol=xtol)
        theta, n_eq, dn = sol
        x = _build_state(V, theta, n_eq, dn, params)
        xdot = plant.evaluate_xdot(x, x[13:17])
        res_norm = float(np.linalg.norm(xdot[[3, 5, 11]]))
        n = x[13:17]
        valid = (ier == 1 and res_norm < residual_tol
                and n.min() >= params['n_min'] - 1e-6
                and n.max() <= params['n_max'] + 1e-6)
        if valid and (best is None or res_norm < best['residual_norm']):
            best = {'converged': True, 'theta': float(theta), 'state': x,
                    'control': x[13:17].copy(), 'residual_norm': res_norm}
    return best or {'converged': False}


def required_motor_current(params, n_vec, V_axial):
    """이 회전수·유입속도를 **유지하는 데 필요한 전류**(I_lim 클립 없음).

    control/battery.py::BatteryModel.step() 은 내부에서 반작용 토크를
    Q=k_Q*n**2*fac(단순화)로 계산해 표 기반 값을 못 넣는다. 이 함수는 같은
    고정점 반복(식11-14)을 쓰되 반작용 토크를 CP(J) 표에서 직접 구한다 —
    research 의 45.61A 도 표 기반이므로 같은 기준으로 비교하려면 이래야 한다.
    클립을 안 거는 이유: "40A로 자르면 얼마나 부족한지"가 아니라 "실제로
    몇 A가 필요한지"가 A/B 결정에 필요한 숫자이기 때문이다.
    """
    pt = params['prop_table']
    D, rho = params['D_prop'], params['rho']
    n_rps = n_vec/(2*np.pi)
    J = V_axial/(n_rps*D + 1e-8)
    ct = np.maximum(np.interp(J, pt['J'], pt['CT']), 0.0)
    cp = np.maximum(np.interp(J, pt['J'], pt['CP']), J*ct + .005)
    Q = cp*rho*n_rps**2*D**5/(2*np.pi)
    I_req = Q/params['k_t']

    k_e, R_m, I_lim = params['k_e'], params['R_m'], params['I_lim']
    V_oc, R_b, eta_esc = params['V_oc'], params['R_b'], params['eta_esc']
    I_cap = np.clip(I_req, 0.0, I_lim)
    V_b = V_oc
    I_i = np.zeros(4)
    for _ in range(8):
        I_v = np.maximum((V_b - k_e*n_vec)/R_m, 0.0)
        I_i = np.minimum(I_cap, I_v)
        P_e = float(np.sum((k_e*n_vec + R_m*I_i)*I_i))/eta_esc
        I_b = P_e/max(V_b, 1e-3)
        V_b = V_oc - R_b*I_b
    return {'I_capped': I_i.copy(), 'I_uncapped': I_req,
            'J': J, 'V_bus': V_b, 'Q': Q}
