"""
배터리·전기 레이어 (논문 식11-14) — 부착형(bolt-on) 모델
==========================================================

핵심 17상태 강체+로터 ODE(dynamics.py)에는 손대지 않는다. 논문 §3.1이
SOC를 "상위 축약모델과 무관한 플랜트 전용 18번째 성분"으로 명시하므로
(x=[p,v,q,ω]인 13상태 축약모델엔 SOC가 아예 없음), 이 모델은 플랜트
시뮬레이션 루프에서 17상태 ODE와 나란히, 하지만 그 밖에서 매 스텝 별도로
적분한다.

M17(NMPCController, electrical_constraints=True)의 예측에는 SOC를 예측
구간 내내 동적으로 적분하지 않고 "현재 측정된 V_b"를 상수로 고정해서
쓴다 — 이건 이 파일이 아니라 nmpc.py의 몫이고, 이 파일은 실제 플랜트가
쓰는 V_b·I_b·s의 실제(참) 값을 계산한다.
"""

import numpy as np


class BatteryModel:
    """식(11)-(14). 매 스텝 로터 전류·전압·SOC를 계산한다.

    Q_f,i(마찰 토크)는 현재 dynamics.py에 별도 마찰 모델이 없어 0으로 둔다
    (식(11)의 항 자체는 남겨뒀으니 나중에 마찰 모델이 생기면 바로 반영 가능).
    """

    def __init__(self, params, s0=1.0):
        self.p = params
        self.s = float(s0)   # SOC, 0~1
        self._I_b_prev = 0.0

    def reset(self, s0=1.0):
        self.s = float(s0)
        self._I_b_prev = 0.0

    def step(self, n_vec, n_cmd_vec, dt):
        """
        Parameters
        ----------
        n_vec : array(4)      실제 로터 각속도 [rad/s] (플랜트 상태 x[13:17]).
        n_cmd_vec : array(4)  로터 속도 명령 [rad/s] (요구 전류 계산, 식11).
        dt : float

        Returns
        -------
        V_b, I_b, s, I_i(array(4)) : 이번 스텝의 버스 전압·전류·SOC·모터별 전류.
        """
        p = self.p
        k_t, k_e, R_m = p['k_t'], p['k_e'], p['R_m']
        J_r, tau_m = p['I_rotor'], p['tau_m']
        k_Q = p['k_Q']
        I_lim = p['I_lim']
        V_oc, R_b, C_b = p['V_oc'], p['R_b'], p['C_b']
        eta_esc = p.get('eta_esc', 0.95)

        n_vec = np.asarray(n_vec, dtype=float)
        n_cmd_vec = np.asarray(n_cmd_vec, dtype=float)

        Q = k_Q * n_vec**2                       # 공력 반작용 토크(식5-10과 동일 모델)
        Q_f = 0.0                                 # 마찰 토크 — 현재 모델 없음(위 docstring 참조)
        I_req = (Q + Q_f) / k_t + J_r * (n_cmd_vec - n_vec) / (k_t * tau_m)
        I_cap = np.clip(I_req, 0.0, I_lim)

        # V_b·I_b·I_i가 서로 얽힌 대수방정식(식13) — 논문은 "각 적분 시점에서
        # 양의 물리 해를 구한다"고만 명시, 정확한 해법은 안 밝혔으므로 여기선
        # 고정점 반복 3회로 근사한다(R_b가 작아 통상 1~2회 안에 수렴).
        V_b = V_oc
        I_i = np.zeros(4)
        I_b = 0.0
        for _ in range(3):
            I_V = np.maximum((V_b - k_e * n_vec) / R_m, 0.0)
            I_i = np.minimum(I_cap, I_V)
            P_e = float(np.sum((k_e * n_vec + R_m * I_i) * I_i)) / eta_esc
            I_b = P_e / max(V_b, 1e-3)
            V_b = V_oc - R_b * I_b

        self.s = float(np.clip(self.s - I_b * dt / (3600.0 * C_b), 0.0, 1.0))
        self._I_b_prev = I_b
        return V_b, I_b, self.s, I_i

    @staticmethod
    def electrical_energy(V_b, I_b, dt):
        """식(47) E_el 적분의 피적분항 (순간 전력 × dt)."""
        return V_b * I_b * dt
