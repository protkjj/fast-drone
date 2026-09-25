"""GINDI (논문 표5 확장 비교군) — 기하학적 외부 루프 + 공통 INDI.

표5의 정의는 "DFBC 또는 기하학적 외부 제어와 공통 INDI", 비교 목적은
"낮은 계산 비용의 기준선"이다. 즉 NMPC를 기하학적(비최적화) 외부 루프로
바꾸되, **하위 루프는 V13/F13과 똑같은 INDI를 쓴다**. 최적화를 뺀 대가가
얼마인지를 재는 비교군이므로, INDI가 달라지면 비교가 성립하지 않는다.

구조:

    속도 PID → 원하는 가속도 a_d
             → 원하는 힘 F_d = m(a_d + g·ê_z) − (명목 공력)
             → 총추력 T_d = ‖F_d‖, 원하는 자세 R_d (추력축을 F_d에 정렬)
             → 자세 오차 → 원하는 각속도 → 각속도 PI → 모멘트 M_d
             → **α_d = I⁻¹ M_d** 를 가상입력으로 ProperHybrid(INDI)에 넘김

마지막 한 줄이 CPID와 갈리는 지점이다. CPID는 같은 (T_d, M_d)를 정적 배분
행렬로 모터 명령으로 바꾸고, GINDI는 α_d로 바꿔 측정 기반 INDI에 넘긴다.

인터페이스가 VirtualNMPC와 같으므로(``__call__(t, x) -> [T, α]``) 쓰는 법도
같다.

    gindi = ProperHybrid(GeometricGuidance(params, v_ref=[12, 0, 0], z_ref=50.),
                         params, dt=plant.dt)

설계는 research/runtime.js의 ``geometricAttitudeCommand`` / ``class GINDI``를
참조했다. 물리 규약(추력축, 쿼터니언 scalar-last)은 control 쪽을 따른다.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from control.dynamics import _body_aerodynamics
from control.hybrid_comparison import rotor_thrust_cap

# research/tune_gains.py 가 논문 §5.3 프로토콜(독립 튜닝 시나리오, 동일 예산,
# 본시험 비참조)로 2026-09-22에 튜닝한 값. 출발점으로 가져왔다.
#
# ⚠ 이 숫자는 research 엔진·선정 프로파일에서 튜닝된 것이다. 논문 실험
#   브랜치가 control로 확정됐으므로, 본시험 전에 control 자체의 튜닝
#   시나리오에서 같은 예산으로 다시 튜닝해야 §5.3을 만족한다. 지금 값은
#   "합리적인 출발점"이지 "§5.3을 만족하는 튜닝 결과"가 아니다.
#
# 각속도 게인이 CPID와 다른 이유 — INDI는 명령된 [T,α]를 CPID의 정적 배분보다
# 훨씬 촘촘히 추종하므로, 같은 숫자를 넣으면 실효 대역폭이 달라진다.
GINDI_GAINS = {
    'kpV': (5.6, 5.6, 8.0),     # 속도 오차 → 가속도 (P)
    'kiV': (0.15, 0.15, 1.2),   # (I)
    'kdV': (0.1, 0.1, 0.2),     # (D, 측정 미분 + 10 Hz LPF)
    'kz': 1.8,                  # 고도 오차 → 수직 속도 참조
    'kR': 3.0,                  # 자세 오차 → 각속도 참조
    'kpW': 3.75,                # 각속도 오차 → 모멘트 (P)
    'kiW': 2.0,                 # (I)
}

# 미분항 저역통과 차단주파수. INDI의 50 Hz(식23)와 다른 값이고 다른 신호에
# 걸린다 — 이건 외부 루프의 속도 오차 미분이다.
DERIV_CUTOFF_HZ = 10.0

# 식(18)의 각가속도 한계. NMPC 계열(V13/M17/F13)은 이 상한을 제약으로 갖는데
# 기하 외부 루프에는 최적화가 없으니 직접 걸어야 한다. 같은 입력 상자를 쓰지
# 않으면 표5 비교가 성립하지 않는다.
ALPHA_LIMIT = 100.0


class GeometricGuidance:
    """기하학적 외부 루프. 가상입력 [T, α_x, α_y, α_z]를 낸다.

    VirtualNMPC와 같은 인터페이스라 ProperHybrid가 그대로 받는다.
    """

    def __init__(self, params, v_ref=None, z_ref=0.0, dt_ctrl=0.02, gains=None):
        """
        dt_ctrl : float
            외부 루프 갱신 간격. 상위 50 Hz / 하위 1 kHz 구조를 유지하기 위해
            VirtualNMPC와 같은 값을 기본으로 둔다. 갱신 사이에는 직전 명령을
            유지한다 — INDI가 매 1 ms마다 그 명령을 실현하려 애쓰는 것이
            이 아키텍처의 핵심이다.
        gains : dict | None
            GINDI_GAINS를 부분 덮어쓴다. 튜닝 드라이버가 이 파일을 고치지 않고
            후보를 평가할 수 있게 하기 위한 것이다(research 쪽과 같은 이유).
        """
        self.p = params
        self.v_ref = np.array(v_ref, dtype=float) if v_ref is not None else np.zeros(3)
        self.z_ref = float(z_ref)
        self.dt_ctrl = dt_ctrl
        self.g = {**GINDI_GAINS, **(gains or {})}

        self.axis = params.get('thrust_axis', 'z')
        self.inertia = np.array([params['Ixx'], params['Iyy'], params['Izz']])
        self.reset()

    def reset(self):
        """MC 시행 간 독립성 — 적분기·필터·유지명령을 모두 비운다."""
        self._int_v = np.zeros(3)
        self._int_w = np.zeros(3)
        self._deriv_v = np.zeros(3)
        self._last_ev = None
        self._last_axes = None             # 힘이 0에 가까울 때 쓸 직전 목표자세
        self._last_t = -np.inf
        self._prev_t = None
        self._u_current = np.array([self.p['mass']*self.p['g'], 0.0, 0.0, 0.0])

    def set_prev_input(self, v):
        """배분 결과 피드백(식27-28)은 비용함수를 가진 제어기만 쓴다.

        GINDI에는 최소화할 비용이 없으므로 받기만 하고 무시한다. ProperHybrid가
        A1 모드에서 항상 호출하므로 인터페이스만 맞춰 둔다(nmpc_f13.py와 동일).
        """

    # ── 내부 ────────────────────────────────────────────────────────

    def _desired_attitude(self, F_hat):
        """추력축을 F_hat에 정렬하는 목표 자세 R_d (열이 동체축).

        추력축이 다르면 정렬할 열도 다르다.
          x축(로켓형): 동체 추력이 +x → 1열을 +F_hat에 맞춘다.
          z축(멀티로터): 동체 추력이 −z → 3열을 −F_hat에 맞춘다.
        남은 두 축은 기준 방향을 추력축에 수직으로 투영해 만든다(헤딩 유지).
        """
        if self.axis == 'x':
            primary, heading = F_hat, np.array([0.0, 1.0, 0.0])
            second = heading - np.dot(heading, primary)*primary
            if np.linalg.norm(second) < 1e-6:
                second = np.cross(np.array([0.0, 0.0, 1.0]), primary)
            second /= max(np.linalg.norm(second), 1e-9)
            return np.column_stack([primary, second, np.cross(primary, second)])

        third, heading = -F_hat, np.array([1.0, 0.0, 0.0])
        first = heading - np.dot(heading, third)*third
        if np.linalg.norm(first) < 1e-6:
            first = np.cross(np.array([0.0, 1.0, 0.0]), third)
        first /= max(np.linalg.norm(first), 1e-9)
        return np.column_stack([first, np.cross(third, first), third])

    def _guidance(self, t, x):
        # 실제 경과시간 사용 — 루프율이 가변이면 고정 dt가 미분·적분을 왜곡한다.
        dt = self.dt_ctrl if self._prev_t is None else t - self._prev_t
        dt = min(max(dt, 1e-4), 0.2)
        self._prev_t = t

        v, q, omega = x[3:6], x[6:10], x[10:13]
        R = Rotation.from_quat(q).as_matrix()

        # ── 1. 속도 루프: 고도 오차를 수직 속도 참조로 ──
        v_des = np.array([self.v_ref[0], self.v_ref[1],
                          self.g['kz']*(self.z_ref - x[2])])
        e_v = v_des - v
        if self._last_ev is None:
            self._last_ev = e_v.copy()
        lam = 1.0 - np.exp(-2*np.pi*DERIV_CUTOFF_HZ*dt)
        self._deriv_v += lam*((e_v - self._last_ev)/dt - self._deriv_v)
        self._last_ev = e_v.copy()

        a_des = (np.array(self.g['kpV'])*e_v
                 + np.array(self.g['kiV'])*self._int_v
                 + np.array(self.g['kdV'])*self._deriv_v)

        # ── 2. 원하는 힘: 중력 보상 + 명목 공력 상쇄 ──
        # 공력을 앞먹임으로 빼 주는 게 이 루프가 '기하학적'인 이유다. 고속에서
        # 동체 항력이 지배적이라 이게 없으면 적분기가 전부 떠안아야 한다.
        F_aero_body, _ = _body_aerodynamics(R.T @ v, omega, self.p)
        F_aero_world = R @ np.asarray(F_aero_body, dtype=float).ravel()
        F_des = self.p['mass']*(a_des + np.array([0.0, 0.0, self.p['g']])) - F_aero_world

        F_norm = float(np.linalg.norm(F_des))
        if F_norm < 1e-6 and self._last_axes is not None:
            R_des = self._last_axes            # 방향이 정의되지 않으면 직전 유지
        else:
            F_hat = (F_des/F_norm if F_norm >= 1e-6
                     else (R[:, 0].copy() if self.axis == 'x' else -R[:, 2].copy()))
            R_des = self._desired_attitude(F_hat)
        self._last_axes = R_des
        T_des = F_norm

        # ── 3. 자세 루프: 동체 좌표 회전오차 → 각속도 참조 ──
        # log(R_curᵀ R_des) 는 '지금 자세에서 목표까지' 회전벡터를 동체 좌표로
        # 준다. ω도 동체 좌표라 바로 비교할 수 있다.
        e_R = Rotation.from_matrix(R.T @ R_des).as_rotvec()
        e_omega = np.asarray(self.g['kR'], dtype=float)*e_R - omega
        M_des = (np.asarray(self.g['kpW'], dtype=float)*e_omega
                 + np.asarray(self.g['kiW'], dtype=float)*self._int_w)

        # ── 4. 조건부 적분(안티와인드업) ──
        # 요구 추력이 물리적 상한을 넘은 동안은 두 적분기를 모두 세운다.
        # 낼 수 없는 추력을 계속 적분하면 포화가 풀린 뒤 크게 튄다.
        v_axial = max(float((R.T @ v)[0]) if self.axis == 'x'
                      else float(-(R.T @ v)[2]), 0.0)
        capacity = float(rotor_thrust_cap(self.p, v_axial).sum())
        if T_des <= capacity + 1e-9:
            self._int_v += e_v*dt
            self._int_w += e_omega*dt

        # ── 5. 모멘트를 각가속도로: 여기서 CPID와 갈린다 ──
        # kpW·kiW를 3-벡터로 줄 수 있게 한 이유: 스칼라 하나를 축마다 다른
        # 관성으로 나누면 실효 각가속도 게인이 축마다 갈린다. 선정 프로파일은
        # Iyy/Ixx = 4.6 이라 kpW=3.75 에서 롤 490 대 피치 107 이 되고, 롤
        # 루프가 모터 지연이 감당할 수 있는 대역을 넘어 불안정해진다
        # (실측 성장률 +33.5/s, results/GINDI_STATUS.md).
        alpha_des = M_des / self.inertia

        # 식(18)의 입력 상자. NMPC 계열은 제약으로 갖는 것을 여기선 직접 건다.
        alpha_des = np.clip(alpha_des, -ALPHA_LIMIT, ALPHA_LIMIT)
        T_des = float(np.clip(T_des, 0.0, capacity))
        return np.concatenate([[T_des], alpha_des])

    def __call__(self, t, x_full):
        """17D 플랜트 상태 → 가상입력 [T, α]. 갱신 사이엔 직전 명령 유지."""
        if t - self._last_t >= self.dt_ctrl - 1e-8:
            self._u_current = self._guidance(t, x_full)
            self._last_t = t
        return self._u_current


def make_gindi(params, v_ref=None, z_ref=0.0, dt=0.001, gains=None, **hybrid_kw):
    """GINDI = 기하 외부 루프 + 공통 INDI.

    V13과 같은 ProperHybrid를 쓰므로 alloc_mode·time_align 같은 표6 플래그가
    그대로 적용된다 — 비교군 사이에서 하위 루프를 동일하게 두려면 이 값들도
    같게 맞춰야 한다.
    """
    from control.hybrid_comparison import ProperHybrid
    guidance = GeometricGuidance(params, v_ref=v_ref, z_ref=z_ref, gains=gains)
    return ProperHybrid(guidance, params, dt=dt, **hybrid_kw)
