"""주장 검증: dynamics.py `_body_aerodynamics` 와 팀 aero.py 가 같은 수식인가?

가설
  dynamics.py 의 C_Na  == 팀 CN_alpha
  dynamics.py 의 C_dc  == eta_cf * Cd_c * A_plan / S_ref
  dynamics.py 의 C_A0  == 팀 CD0(V)      (C_Aa2 == 0)
이면 두 모델의 (축력, 수직력) 이 같은 (V, alpha) 에서 일치해야 한다.
"""
import math, os, sys
import numpy as np
import casadi as ca

sys.path.insert(0, os.environ.get("ROCKET_DRONE_PATH",
                                  os.path.expanduser("~/Desktop/dynamic/rocket-drone")))
import constants as k
from interfaces import DesignVars
from modules import atm, geom, aero

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
from control.dynamics import _body_aerodynamics

DV = DesignVars(d_body=0.09, lambda_body=8.0, S_fin=0.022, x_fin=0.60,
                AR_fin=2.2, f_mount=1.0, n_design=4.0, d_prop=0.13,
                pd_prop=1.50, n_ser=6, k_E=1.0, k_mot=1.0)
air = atm.run(k.h_miss); hl = geom.hull(DV); aer = aero.run(DV, hl, air)
POD = (0.02981657140810981, 0.10435799992838433)

A_nose = k.k_side * DV.d_body * hl.l_nose
A_cyl  = DV.d_body * hl.l_cyl
A_fin  = k.k_finproj * DV.S_fin
A_plan = A_nose + A_cyl + A_fin
C_dc_equiv = k.eta_cf * k.Cd_c * A_plan / hl.S_ref
print(f"A_plan={A_plan:.6f} m^2   A_plan/S_ref={A_plan/hl.S_ref:.4f}")
print(f"등가 C_dc = eta_cf*Cd_c*A_plan/S_ref = {C_dc_equiv:.6f}")
print(f"등가 C_Na = CN_alpha              = {aer.CN_alpha:.6f}")
print()

# dynamics.py 를 심볼릭이 아니라 수치로 쓰기 위한 래퍼
def dyn_FN_FA(V, alpha_rad, params):
    u = V*math.cos(alpha_rad); w = V*math.sin(alpha_rad); v = 0.0
    vb = ca.DM([u, v, w]); om = ca.DM([0,0,0])
    F, M = _body_aerodynamics(vb, om, params)
    F = np.array(ca.DM(ca.substitute(F, ca.SX.sym('dummy'), 0)).elements()) \
        if isinstance(F, ca.SX) else np.array(ca.DM(F).elements())
    return F

print(f"{'alpha[deg]':>10} | {'팀 A[N]':>10} {'dyn A[N]':>10} | {'팀 N[N]':>10} {'dyn N[N]':>10} | {'rel err':>10}")
print("-"*72)
worst = 0.0
for V in (20.0, 60.0, 83.3, 120.0):
    CD0_V = aer.F_drag(V, 0.0, POD) / (0.5*air.rho*V*V*hl.S_ref)
    params = dict(rho=air.rho, S_ref=hl.S_ref, d_ref=DV.d_body,
                  C_Na=aer.CN_alpha, C_dc=C_dc_equiv, C_A0=CD0_V, C_Aa2=0.0,
                  x_cp=0.0, C_lp=0.0, C_mq=0.0)
    for a_deg in (0.0, 2.0, 5.0, 10.0, 20.0, 30.0, 45.0, 60.0):
        a = math.radians(a_deg)
        q = 0.5*air.rho*V*V
        # 팀: N = q*S*C_N,  A = q*S*CD0
        CN_team = (aer.CL(V, a, POD)*math.cos(a)
                   + (aer.F_drag(V, a, POD)/(q*hl.S_ref))*math.sin(a))
        N_team = q*hl.S_ref*CN_team
        A_team = q*hl.S_ref*CD0_V
        F = dyn_FN_FA(V, a, params)
        A_dyn = -F[0]                       # 축력 크기 (dyn 은 -x 방향)
        N_dyn = math.hypot(F[1], F[2])      # 수직력 크기
        den = max(abs(N_team), 1e-9)
        err = abs(N_dyn - N_team)/den
        worst = max(worst, err, abs(A_dyn-A_team)/max(abs(A_team),1e-9))
        if V == 83.3:
            print(f"{a_deg:10.1f} | {A_team:10.4f} {A_dyn:10.4f} | "
                  f"{N_team:10.4f} {N_dyn:10.4f} | {err:10.2e}")
print("-"*72)
print(f"전 속도·전 받음각 최대 상대오차 = {worst:.3e}")
print("판정:", "동일 수식 확인" if worst < 1e-9 else "불일치 — 가설 기각")
