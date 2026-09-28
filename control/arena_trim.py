"""경기장 6자유도 트림 — 무게중심 편차 시나리오의 플랜트 출발 트림과 '트림 존재' 판정용.

왜 따로 만드나(2026-09-28 격리 실험, 보고서 20절): 팀원 `find_trim`(벤더, 무수정)은 평면 탐색기다.
미지수가 [θ, n_+z, n_-z] 셋이고(로터를 +z쌍·−z쌍으로 묶어 좌우 대칭 가정), 잔차도 v̇x·v̇z·ω̇y
셋뿐이다. 호버(V=0)는 탐색 없이 대칭 해석해(dn=0)를 돌려준다. 그래서 무게중심을
  - 동체 y로 옮기면 좌우 비대칭 추력이 필요해 표현 자체가 불가능하고(0·20·85 m/s 모두 실패),
  - 동체 z로 옮기면 호버에서 피치 차동이 필요한데 해석해 분기가 0으로 고정한다(0 m/s 실패).
이 경우들도 로터 4개를 따로 풀면 트림이 있다(잔차 1e-13 수준). 벤더 실패를 '기체 한계'로
분류하면 틀린다 — 벤더 문서도 "An unsuccessful search is NOT proof ..."라고 적는다.

미지수 7개(θ, 회전 2개, 로터 4개)에 잔차 6개라 그대로 풀면 해가 한 줄로 이어진 가족이다(실측:
85 m/s CG y에서 롤 0°·−10°·−28.6° 모두 트림). 한 개를 고정해 해를 하나로 정한다(kj 결정 2026-09-28):
  조건 1 'roll0'      비행 방향(세계 x) 둘레 회전 = 0, 세계 z 둘레 회전(요) 자유 → 옆미끄럼이 생길 수 있다
  조건 2 'sideslip0'  요 = 0, 비행 방향 둘레 회전 자유 → 속도 벡터 둘레로 도므로 동체 속도가 안 바뀌어
                      옆미끄럼은 평면 트림처럼 0이다
조건 1로 없으면 조건 2, 둘 다 없으면 '기체 한계'(ValueError). 어느 조건으로 잡았는지 돌려준다.

호버(V=0)의 조건 1은 요도 0으로 고정한다(미지수 5, 잔차 6): 호버에서는 추력축이 세계 z라
세계 z 둘레 회전이 곧 추력축 둘레 회전 — 평형을 안 바꾸는 대칭이라 요가 정해지지 않는다(처음엔
풀어 뒀다가 명목 호버에서 요가 −15°로 흐르고 잔차가 8e-8에 멈췄다). 호버에서 요는 뜻이 없으니
고정해도 잃는 것이 없다.

자세 = Rot_world([φ, 0, ψ]) · pitch_quaternion(θ). 명목 기체(좌우 대칭)에서는 φ=ψ=0이고
θ·회전수가 벤더 find_trim과 같아야 한다 — `control/test_arena_trim.py`가 시험한다.

이 모듈은 **플랜트** 출발 상태에만 쓴다. 제어기는 지금처럼 명목 모델 트림만 받는다.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from models.team_light.control.dynamics import AxialDronePlant, NX
from models.team_light.control.geometry import pitch_quaternion, rotor_thrusts
from models.team_light.control.trim import find_trim as planar_trim

CONDITIONS = ('roll0', 'sideslip0')
RESIDUAL_TOL = 1e-6              # 벤더 find_trim의 valid 기준과 같다
ANGLE_BOUND = np.pi/4            # 고정 안 한 회전의 탐색 범위(격자 없이 한 번에 푼다)


def _attitude(theta, free_angle, condition):
    rotvec = [0.0, 0.0, free_angle] if condition == 'roll0' else [free_angle, 0.0, 0.0]
    return (Rotation.from_rotvec(rotvec)*Rotation.from_quat(pitch_quaternion(theta))).as_quat()


def _solve(params, V, condition, guess):
    """한 조건에서 [θ, 자유 회전, n1..n4] 최소제곱. 호버의 roll0은 자유 회전 없이
    [θ, n1..n4]. 결과 dict(유효 여부 포함)."""
    plant = AxialDronePlant(params)
    free = not (condition == 'roll0' and V == 0.0)

    def unpack(z):
        return (z[0], z[1], z[2:]) if free else (z[0], 0.0, z[1:])

    def state(z):
        theta, angle, n = unpack(z)
        x = np.zeros(NX)
        x[3] = V
        x[6:10] = _attitude(theta, angle, condition)
        x[13:17] = n
        return x

    def residual(z):
        x = state(z)
        xd = plant.evaluate_xdot(x, x[13:17])
        return np.r_[xd[3:6], xd[10:13]]

    angle_slot = [0] if free else []
    lower = [-np.pi/2] + [-ANGLE_BOUND]*len(angle_slot) + [params['n_min']]*4
    upper = [np.pi/2] + [ANGLE_BOUND]*len(angle_slot) + [params['n_max']]*4
    start = np.r_[guess[0], guess[1:2] if free else [], guess[2:]]
    n_scale = max(float(np.mean(guess[2:])), 1.0)
    sol = least_squares(residual, np.clip(start, lower, upper), bounds=(lower, upper),
                        x_scale=[1.0]*(1 + len(angle_slot)) + [n_scale]*4,
                        ftol=1e-14, xtol=1e-14, gtol=1e-14, max_nfev=5000)
    x = state(sol.x)
    theta, angle, _ = unpack(sol.x)
    u = x[13:17].copy()
    res_norm = float(np.linalg.norm(residual(sol.x)))
    valid = bool(np.isfinite(res_norm) and res_norm < RESIDUAL_TOL
                 and np.all(u >= params['n_min']) and np.all(u <= params['n_max']))
    vb = Rotation.from_quat(x[6:10]).as_matrix().T @ x[3:6]
    return dict(state=x, control=u, theta=float(theta), free_angle=float(angle),
                condition=condition, residual=res_norm, valid=valid, converged=valid,
                v_body=vb, alpha=float(np.arctan2(vb[2], vb[0])),
                sideslip=float(np.arcsin(np.clip(vb[1]/max(np.linalg.norm(vb), 1e-12), -1, 1))),
                T_total=float(np.sum(rotor_thrusts(params, u, vb))))


def find_trim_6dof(params, V_cruise, *, strict=True):
    """조건 1(roll0) → 조건 2(sideslip0) 순서로 트림을 찾는다. 반환 dict는 벤더 find_trim처럼
    'state'·'control'·'valid'를 갖고, 'condition'에 잡은 조건(둘 다 실패면 None)을 적는다.

    초기값은 같은 params의 벤더 평면 탐색 결과(strict=False — 무효여도 θ·회전수 크기는 쓸 만하다).
    """
    if not np.isfinite(V_cruise) or V_cruise < 0:
        raise ValueError('V_cruise must be a finite nonnegative speed.')
    planar = planar_trim(params, float(V_cruise), strict=False)
    guess = np.r_[min(planar['theta'], np.pi/2 - 1e-6), 0.0, planar['control']]
    attempts = []
    for condition in CONDITIONS:
        result = _solve(params, float(V_cruise), condition, guess)
        attempts.append(dict(condition=condition, residual=result['residual']))
        if result['valid']:
            result['attempts'] = attempts
            return result
    result['condition'] = None
    result['attempts'] = attempts
    if strict:
        raise ValueError(f'vehicle limit: no 6-DOF trim at {V_cruise:g} m/s under '
                         f'{CONDITIONS} (residuals {[a["residual"] for a in attempts]})')
    return result
