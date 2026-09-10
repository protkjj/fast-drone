"""공력 계수 소스 — CSV 생성기와 기준값 생성기가 공유하는 단일 출처.

두 소스가 있다:
  (a) placeholder : control/dynamics.py 의 계수 모델 (control/vehicle_params.py 값)
  (b) sized       : 팀 사이징 rocket-drone/modules/aero.py, 확정 설계점

DESIGN.md §3 에서 **두 소스가 같은 수식**임을 수치로 확인했다 (최대 상대오차 2.7e-10).
그래서 아래 공통 인터페이스 하나로 둘 다 담긴다:

    src.coeffs(V, alpha) -> {C_A, C_N, x_cp, C_lp, C_mq}     ← CSV 한 행
    src.dyn_params(V)    -> dict                             ← _body_aerodynamics 인자

`dyn_params` 가 따로 있는 이유: 기준값(reference)은 표를 거치지 않고 프로젝트가
이미 신뢰하는 `_body_aerodynamics()` 를 **직접** 불러서 만들어야 순환논증이 안 된다.
표는 그 함수를 격자로 샘플링한 것이고, 격자점 위에서는 둘이 정확히 같아야 한다.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

# 팀 사이징 저장소 — fast_drone 저장소 밖이라 경로를 주입받는다
ROCKET_DRONE = os.environ.get(
    "ROCKET_DRONE_PATH", os.path.expanduser("~/Desktop/dynamic/rocket-drone"))

# 팀 확정 설계점 (rocket-drone/main.py 예시점 + HANDOFF 통과안 S_fin/x_fin)
SIZED_DESIGN_POINT = dict(
    d_body=0.09, lambda_body=8.0, S_fin=0.022, x_fin=0.60, AR_fin=2.2,
    f_mount=1.0, n_design=4.0, d_prop=0.13, pd_prop=1.50, n_ser=6,
    k_E=1.0, k_mot=1.0,
)

# 감쇠 미분계수 — 팀 aero.py 에는 없다. dynamics.py 값을 잠정 사용.
# DESIGN.md §7 미결 항목: d_ref 가 0.15 -> 0.09 로 바뀐 영향 미확인.
C_LP_DEFAULT = -5.0
C_MQ_DEFAULT = -10.0


@dataclass
class AeroSource:
    """공력 계수 소스의 공통 얼굴."""
    name: str
    S_ref: float          # [m^2] 기준면적 = 동체 최대단면적
    d_ref: float          # [m]   기준길이 = 동체 지름
    rho: float            # [kg/m^3]
    x_cp_body: float      # [m]   CG 기준 body x. 음수 = CG 뒤 = 정적 안정
    C_lp: float = C_LP_DEFAULT
    C_mq: float = C_MQ_DEFAULT
    meta: dict = field(default_factory=dict)

    # 아래 둘은 서브클래스가 채운다
    def _C_A(self, V: float, alpha: float) -> float:
        raise NotImplementedError

    def _C_N(self, V: float, alpha: float) -> float:
        raise NotImplementedError

    def coeffs(self, V: float, alpha: float) -> dict:
        """CSV 한 행. alpha 는 전 받음각 [rad], 0 <= alpha <= pi."""
        return dict(C_A=self._C_A(V, alpha), C_N=self._C_N(V, alpha),
                    x_cp=self.x_cp_body, C_lp=self.C_lp, C_mq=self.C_mq)

    def dyn_params(self, V: float) -> dict:
        """`control.dynamics._body_aerodynamics` 가 받는 params dict.

        속도 의존 계수(팀 CD0(V))는 이 V 에 고정해 넣는다. 그래서 이 dict 는
        **단일 속도에서만** 유효하다.
        """
        raise NotImplementedError


class PlaceholderSource(AeroSource):
    """(a) control/vehicle_params.py 의 계수 모델.

    C_A = C_A0 + C_Aa2 * sin^2(alpha)        <- (v^2+w^2)/V^2 = sin^2(alpha)
    C_N = C_Na * sin(a)cos(a) + C_dc * sin^2(a)
    """

    def __init__(self, vp: dict):
        super().__init__(
            name="placeholder",
            S_ref=float(vp["S_ref"]), d_ref=float(vp["d_ref"]),
            rho=float(vp["rho"]), x_cp_body=float(vp["x_cp"]),
            C_lp=float(vp["C_lp"]), C_mq=float(vp["C_mq"]),
            meta=dict(source="control/vehicle_params.py",
                      mass=vp["mass"], note="형상팀 확정 전 임시값"))
        self._vp = dict(vp)

    def _C_A(self, V, alpha):
        s = math.sin(alpha)
        return self._vp["C_A0"] + self._vp["C_Aa2"] * s * s

    def _C_N(self, V, alpha):
        s, c = math.sin(alpha), math.cos(alpha)
        return self._vp["C_Na"] * s * c + self._vp["C_dc"] * s * s

    def dyn_params(self, V):
        return dict(rho=self.rho, S_ref=self.S_ref, d_ref=self.d_ref,
                    C_Na=self._vp["C_Na"], C_dc=self._vp["C_dc"],
                    C_A0=self._vp["C_A0"], C_Aa2=self._vp["C_Aa2"],
                    x_cp=self.x_cp_body, C_lp=self.C_lp, C_mq=self.C_mq)


class SizedSource(AeroSource):
    """(b) 팀 rocket-drone/modules/aero.py, 확정 설계점.

    C_A = CD0(V)                      <- 축력이 alpha 무관 (DESIGN.md §3)
    C_N = CN_alpha*sin(a)cos(a) + eta_cf*Cd_c*(A_plan/S_ref)*sin^2(a)
    """

    def __init__(self, design_point: dict | None = None, pod: tuple | None = None):
        if ROCKET_DRONE not in sys.path:
            sys.path.insert(0, ROCKET_DRONE)
        try:
            import constants as k                     # noqa: N813
            from interfaces import DesignVars
            from modules import atm, geom, aero, stab
            import main as launcher
        except ImportError as e:
            raise SystemExit(
                f"팀 사이징 저장소를 못 읽었습니다: {ROCKET_DRONE}\n"
                f"  ({e})\n"
                f"  ROCKET_DRONE_PATH 환경변수로 경로를 지정하세요.") from e

        dp = dict(SIZED_DESIGN_POINT if design_point is None else design_point)
        dv = DesignVars(**dp)

        air = atm.run(k.h_miss)
        hl = geom.hull(dv)
        aer = aero.run(dv, hl, air)

        # 질량특성(x_cg)과 포드 치수는 사이징 루프 ②를 돌려야 나온다.
        # stab.run 을 잠깐 가로채 MassProps 를 받아온다 (이 프로세스 한정).
        captured = {}
        _orig = stab.run

        def _spy(dv_, hl_, aer_, mp_, lay_, dT_):
            captured["mp"] = mp_
            captured["lay"] = lay_
            return _orig(dv_, hl_, aer_, mp_, lay_, dT_)

        stab.run = _spy
        try:
            result = launcher.evaluate(dv)
        finally:
            stab.run = _orig

        if "mp" not in captured:
            raise SystemExit(
                f"사이징이 ② 이전에 탈락했습니다 (fail_code={result.fail_code}). "
                f"질량특성을 못 얻습니다.")

        mp = captured["mp"]
        if pod is None:
            pod = (result.diag["d_pod"], result.diag["l_pod"])

        # 평면형 투영 면적 — aero.py 의 크로스플로 항과 같은 구성
        A_plan = (k.k_side * dv.d_body * hl.l_nose
                  + dv.d_body * hl.l_cyl
                  + k.k_finproj * dv.S_fin)

        super().__init__(
            name="sized",
            S_ref=hl.S_ref, d_ref=dv.d_body, rho=air.rho,
            # 팀 x_cp 는 기수 기준 양수. 동체좌표(CG 기준, 뒤가 음수)로 변환.
            x_cp_body=-(aer.x_cp - mp.x_cg),
            meta=dict(
                source="rocket-drone/modules/aero.py",
                design_point=dp,
                MTOW=result.ec["C1_MTOW[kg]"], x_cg=mp.x_cg,
                J_xx=mp.J_xx, J_yy=mp.J_yy, J_zz=mp.J_zz,
                l_body=hl.l_body, l_nose=hl.l_nose, l_cyl=hl.l_cyl,
                arm_rotor=captured["lay"].arm_rotor,
                d_pod=pod[0], l_pod=pod[1],
                CN_alpha=aer.CN_alpha, x_cp_nose=aer.x_cp,
                SM_cal=(aer.x_cp - mp.x_cg) / dv.d_body,
                feasible=result.feasible),
        )
        self._aer, self._hl, self._air, self._pod = aer, hl, air, pod
        self._CN_alpha = aer.CN_alpha
        self._K_cross = k.eta_cf * k.Cd_c * A_plan / hl.S_ref

    def _CD0(self, V: float) -> float:
        """팀 F_drag(V, 0) 를 되돌려 CD0(V) 를 얻는다 (CD0 가 비공개 클로저라서)."""
        V = max(V, 1.0)      # aero.py 내부와 같은 하한 (Re -> 0 에서 상관식이 무너짐)
        q = 0.5 * self._air.rho * V * V * self._hl.S_ref
        return self._aer.F_drag(V, 0.0, self._pod) / q

    def _C_A(self, V, alpha):
        return self._CD0(V)

    def _C_N(self, V, alpha):
        s, c = math.sin(alpha), math.cos(alpha)
        return self._CN_alpha * s * c + self._K_cross * s * s

    def dyn_params(self, V):
        return dict(rho=self.rho, S_ref=self.S_ref, d_ref=self.d_ref,
                    C_Na=self._CN_alpha, C_dc=self._K_cross,
                    C_A0=self._CD0(V), C_Aa2=0.0,
                    x_cp=self.x_cp_body, C_lp=self.C_lp, C_mq=self.C_mq)


def get_source(name: str) -> AeroSource:
    if name == "placeholder":
        from control.vehicle_params import vehicle_params
        return PlaceholderSource(vehicle_params)
    if name == "sized":
        return SizedSource()
    raise SystemExit(f"알 수 없는 소스: {name!r} (placeholder | sized)")
