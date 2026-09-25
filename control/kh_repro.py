"""kj 지시(권고순서 2·3, 2026-09-25 저녁 갱신) — 팀원 분리형과 우리 V13을
같은 플랜트·시나리오로.

시나리오 정의(초기오차·지속시간·돌풍·정지조건·성공판정)는 팀원의
`models.team_light.control.run_baseline_comparison.run_case`를 **그대로
옮겨 적었다**(그 함수 자체를 재사용하지 못하는 이유는 그 함수가 팀원 제어기
클래스를 직접 import해 하드코딩하고 있어서다 — 시나리오 로직은 같게,
제어기만 바꿔 끼운다). 플랜트는 항상 팀원 것
(`models.team_light.control.dynamics.AxialDronePlant`)이고, 우리 쪽 제어기
파라미터는 `control.kh_adapter.build_controller_params()`가 만든다(로터
기하는 팀원 값 그대로, 공력만 최소제곱 적합).

**주의**: "팀원 분리형" 비교 대상은 `models/team_light/control/`(우리
control/의 2026-09-09 옛 스냅샷, cost_spec/S1 등 없음)의 ProperHybrid가
아니라 `Factory.make('Split',...)`가 실제로 쓰는 `ComparisonNMPC`+
`ProperHybrid` 조합이다 — 규현의 원 비교실험(docs/BASELINE_V2_RESULTS.md)이
그 조합으로 나온 결과이므로 여기서도 그대로 쓴다(그래야 46.4s 등 문서
수치가 재현된다). 우리 V13 쪽은 **현재**(병합 이후 최신) `control/
hybrid_comparison.py`를 쓴다 — kj 확인사항: 옛 스냅샷 기반 비교는
'분리형' 자체가 아니라 구버전 구현에 대한 비교라 무효.

실행: python3 -m control.kh_repro
"""
from copy import deepcopy
from time import perf_counter

import numpy as np
from scipy.spatial.transform import Rotation

from models.team_light.control.dynamics import AxialDronePlant as KHPlant
from models.team_light.control.trim import find_trim as kh_find_trim
from models.team_light.control.propeller_curve import domain_status as kh_domain_status
from models.team_light.control.run_baseline_comparison import Factory as KHFactory, gust

from control.kh_adapter import kh_native_params, build_controller_params
from control.hybrid_comparison import ProperHybrid, VirtualNMPC

DT = 0.002    # 팀원 시험과 동일 플랜트 스텝(run_baseline_comparison.DT)


def _install_solve_log_spy(nmpc):
    """`VirtualNMPC`에는 팀원 `ComparisonNMPC`같은 solve_log가 없다 — 실패를
    "솔버 실패"와 "제어 실패"로 분류하려면 있어야 한다(kj 지시 2번).
    `control/hybrid_comparison.py`는 안 건드리고 인스턴스 메서드만 감싼다
    (이 저장소의 `bench_acados.py` 등이 이미 쓰는 스파이 패턴과 동일).
    """
    nmpc.solve_log = []
    original_solve = nmpc._solve

    def spied(x13):
        t0 = perf_counter()
        u = original_solve(x13)
        status = nmpc.last_status
        accepted = status in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
        nmpc.solve_log.append(dict(
            t=None, status=status, accepted=bool(accepted),
            seconds=perf_counter() - t0,
            iter_count=nmpc.solver.stats().get('iter_count'),
            u=u.copy() if u is not None else None))
        return u

    nmpc._solve = spied
    return nmpc


def make_our_split(ctrl_params, speed, z, dt_ctrl=0.02):
    """우리 V13(S1 시간정렬 + 논문 비용함수) — kj 지시: 'S1·논문 비용함수 적용판'.

    콜드스타트 웜스타트는 `VirtualNMPC._solve()`의 `_cold_start` 보정이
    첫 호출에서 **실측 상태**(여기서는 트림+섭동)로 채운다."""
    nmpc = VirtualNMPC(ctrl_params, v_ref=[speed, 0, 0], z_ref=z,
                       dt_ctrl=dt_ctrl, cost_spec='paper')
    _install_solve_log_spy(nmpc)
    hyb = ProperHybrid(nmpc, ctrl_params, dt=DT, time_align='S1')
    return hyb, nmpc


def make_our_split_trim_warmstart(ctrl_params, native_params, speed, z, dt_ctrl=0.02):
    """kj 판별실험 1) — 콜드스타트 웜스타트를 실측(트림+섭동) 대신
    **정확한 트림 상태·트림 입력**으로 채운 판. H-웜스타트이력 가설
    검증용 — 미션이 "쉬운 지점(호버)에서 시작해 웜스타트가 이어지는"
    것과 비슷하게, 짧은 시험도 "이미 좋은 웜스타트를 들고 시작"하면
    통과하는지 본다. `make_our_split`과 차이는 이 웜스타트 하나뿐이다.
    """
    nmpc = VirtualNMPC(ctrl_params, v_ref=[speed, 0, 0], z_ref=z,
                       dt_ctrl=dt_ctrl, cost_spec='paper')
    tr = kh_find_trim(native_params, speed)
    x_trim13 = np.concatenate([tr['state'][0:10], tr['state'][10:13]])
    x_trim13[2] = z
    u_trim = np.array([tr['T_total'], 0.0, 0.0, 0.0])
    stride = 4 + 13   # NU_V + NX_V
    for k in range(nmpc.N + 1):
        off = k * stride
        nmpc.w0[off:off + 13] = x_trim13
    for k in range(nmpc.N):
        off = k * stride + 13
        nmpc.w0[off:off + 4] = u_trim
    nmpc._cold_start = False   # _solve()의 실측 기반 보정이 위 트림 웜스타트를 덮어쓰지 않게
    _install_solve_log_spy(nmpc)
    hyb = ProperHybrid(nmpc, ctrl_params, dt=DT, time_align='S1')
    return hyb, nmpc


def run_case(plant_params, ctrl_factory, label, speed, case, *, duration=4.0):
    """팀원 `run_baseline_comparison.run_case`의 시나리오 로직을 그대로 옮김
    (초기오차 +0.2 m / +0.5 m/s, z목표 20 m, 정지조건·성공판정 동일).
    제어기만 `ctrl_factory(speed, z) -> (controller, solver_or_None)`로 교체.
    """
    z = 20.0
    tr = kh_find_trim(plant_params, speed)
    x = tr['state'].copy()
    x[2] = z
    x[2] += 0.2
    x[3] += 0.5

    ctrl, solver = ctrl_factory(speed, z)
    plant = KHPlant(plant_params, dt=DT)

    times = [0.0]
    states = [x.copy()]
    commands = []
    reason = None
    outside = 0
    start = perf_counter()
    n_steps = round(duration/DT)
    for k in range(n_steps):
        t = k*DT
        v_ref = np.array([speed, 0.0, 0.0])
        w = (gust(t, 2, 2.0) if case == 'vertical' else
             gust(t, 1, 2.0) if case == 'lateral' else np.zeros(3))
        try:
            u = np.asarray(ctrl(t, x))
            if not np.all(np.isfinite(u)):
                reason = 'nonfinite command'
                break
            if np.any(u < plant_params['n_min'] - 1e-6) or np.any(u > plant_params['n_max'] + 1e-6):
                reason = 'command outside physical bounds'
                break
            if solver is not None and getattr(solver, 'consec_fail', 0) >= 5:
                reason = '5 consecutive optimizer failures'
                break
            xn = plant.step(x, u, w)
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            reason = f"{type(exc).__name__}: {str(exc)[:160]}"
            break
        if not np.all(np.isfinite(xn)):
            reason = 'nonfinite state'
            break
        vb = Rotation.from_quat(xn[6:10]).as_matrix().T @ (xn[3:6] - w)
        outside += not kh_domain_status(plant_params, xn[13:17], vb)['inside_assumed_domain']
        commands.append(u.copy())
        x = xn
        times.append((k + 1)*DT)
        states.append(x.copy())
        if abs(x[2] - z) > 5.0:
            reason = 'altitude deviation limit'
        elif np.linalg.norm(x[3:6] - v_ref) > 20.0:
            reason = 'velocity error limit'
        elif np.linalg.norm(x[10:13]) > 30.0:
            reason = 'body rate limit'
        if reason:
            break

    times = np.asarray(times)
    states = np.asarray(states)
    commands = np.asarray(commands).reshape(-1, 4)
    ez = states[:, 2] - z
    ev = states[:, 3:6] - np.array([speed, 0.0, 0.0])
    saturation = float(np.mean(np.any(
        (commands <= plant_params['n_min'] + 1e-6) |
        (commands >= plant_params['n_max'] - 1e-6), axis=1))) if len(commands) else 0.0
    solve_log = deepcopy(getattr(solver, 'solve_log', [])) if solver is not None else []
    n_fail = sum(1 for e in solve_log if not e.get('accepted', True))
    result = dict(
        controller=label, speed_m_s=speed, case=case, completed=reason is None,
        stop_reason=reason, elapsed_sim_s=float(times[-1]),
        wall_seconds=perf_counter() - start,
        z_rmse_m=float(np.sqrt(np.mean(ez**2))), z_final_m=float(ez[-1]),
        vel_rmse_m_s=float(np.sqrt(np.mean(np.sum(ev**2, axis=1)))),
        vel_final_m_s=float(np.linalg.norm(ev[-1])),
        saturation_fraction=saturation,
        prop_domain_outside_fraction=outside/max(len(commands), 1),
        optimizer_failures=n_fail, optimizer_calls=len(solve_log),
        last_solve_log_entries=solve_log[-3:] if solve_log else [],
    )
    result['tracking_pass'] = bool(
        reason is None and abs(ez[-1]) < 0.5 and np.linalg.norm(ev[-1]) < 0.5
        and saturation < 0.01)
    print(f"  {label:14s} {speed:5.1f}m/s {case:9s}  완주={result['completed']!s:5s}  "
          f"z_RMSE={result['z_rmse_m']:.4f}  v_RMSE={result['vel_rmse_m_s']:.3f}  "
          f"sat={100*saturation:.1f}%  솔버실패={n_fail}/{len(solve_log)}  "
          f"정지사유={reason}")
    return result


def main():
    native = kh_native_params()
    ctrl_params = build_controller_params(native)

    factory = KHFactory(native)

    def their_split(speed, z):
        ctrl = factory.make('Split', speed, z)
        solver = ctrl.nmpc if hasattr(ctrl, 'nmpc') else ctrl
        return ctrl, solver

    def our_v13(speed, z):
        hyb, nmpc = make_our_split(ctrl_params, speed, z)
        return hyb, nmpc

    scenarios = [(80.0, 'nominal'), (80.0, 'vertical'),
                (85.0, 'nominal'), (85.0, 'vertical')]

    print("=" * 70)
    print("팀원 분리형(Split) — 팀원 플랜트·팀원 제어기, 그대로")
    results_theirs = [run_case(native, their_split, 'Split(팀원)', V, case)
                      for V, case in scenarios]

    print()
    print("우리 V13(S1·논문비용) — 팀원 플랜트, 우리 제어기(공력만 적합)")
    results_ours = [run_case(native, our_v13, 'V13(우리,S1+paper)', V, case)
                    for V, case in scenarios]

    return results_theirs, results_ours


if __name__ == '__main__':
    main()
