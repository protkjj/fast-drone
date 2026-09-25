"""kj 작업지시서(2026-09-25 저녁) 2단계 — 65초 통합 미션 재현.

`docs/BASELINE_V2_RESULTS.md`(팀원 규현, `external/fastdrone_kh/docs/`)의
"기존 70 m/s 통합 미션"(이륙10s+안정화3s+가속15s+순항15s+감속15s+호버링7s
=65s, 순항 중 35s에 10m/s 수직돌풍)을 그대로 재현한다. 먼저 팀원 자신의
Split/직접NMPC/나이브/GS-LQR/INDI(그들 코드 그대로, `run_baseline_comparison.
run_case(mission=True)`)로 문서의 숫자(분리형 46.400s, 직접NMPC·나이브
46.120s에 감속 중 연속 최적화 실패, GS-LQR·INDI 65.000s 완주)가 재현되는지
확인한 다음, 같은 플랜트·같은 미션에 우리 V13을 추가로 돌린다.

우리 V13은 매 스텝 참조가 바뀌므로(가속→순항→감속) `VirtualNMPC`의
`ref_fn`(cost_spec='paper' 전용, 노드별 참조)으로 `MissionProfile.get_ref`를
그대로 연결한다.

실행: python3 -m control.kh_mission_repro
"""
import sys
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.spatial.transform import Rotation

_KH_ROOT = Path(__file__).resolve().parent.parent / 'external' / 'fastdrone_kh'
if str(_KH_ROOT) not in sys.path:
    sys.path.insert(0, str(_KH_ROOT))

from kh_control.dynamics import AxialDronePlant as KHPlant  # noqa: E402
from kh_control.trim import find_trim as kh_find_trim  # noqa: E402
from kh_control.propeller_curve import domain_status as kh_domain_status  # noqa: E402
from kh_control.mission_sim import MissionProfile as KHMissionProfile  # noqa: E402
from kh_control.run_baseline_comparison import Factory as KHFactory, gust, solver_of  # noqa: E402

from control.kh_adapter import kh_native_params, build_controller_params  # noqa: E402
from control.hybrid_comparison import ProperHybrid, VirtualNMPC  # noqa: E402
from control.kh_repro import _install_solve_log_spy  # noqa: E402

DT = 0.002


def run_mission(plant_params, ctrl_factory, label, *, profile=None):
    """`run_baseline_comparison.run_case(mission=True)`와 동일 로직(팀원
    시나리오 그대로) — 제어기만 팩토리로 교체 가능하게 뺐다."""
    profile = profile or KHMissionProfile(70.0, 50.0)
    tr = kh_find_trim(plant_params, 0.0)
    x = tr['state'].copy()
    x[2] = 2.0

    ctrl, solver = ctrl_factory(profile)
    plant = KHPlant(plant_params, dt=DT)

    times = [0.0]
    states = [x.copy()]
    commands = []
    reason = None
    outside = 0
    start = perf_counter()
    n_steps = round(profile.T_total / DT)
    for k in range(n_steps):
        t = k * DT
        v_ref, z_ref, _ = profile.get_ref(t)
        if hasattr(ctrl_factory, 'update'):
            ctrl_factory.update(ctrl, v_ref, z_ref)
        w = gust(t, 2, 10.0, 35.0)
        try:
            u = np.asarray(ctrl(t, x))
            if not np.all(np.isfinite(u)):
                reason = 'nonfinite command'; break
            if np.any(u < plant_params['n_min'] - 1e-6) or np.any(u > plant_params['n_max'] + 1e-6):
                reason = 'command outside physical bounds'; break
            if solver is not None and getattr(solver, 'consec_fail', 0) >= 5:
                reason = '5 consecutive optimizer failures'; break
            xn = plant.step(x, u, w)
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            reason = f"{type(exc).__name__}: {str(exc)[:160]}"; break
        if not np.all(np.isfinite(xn)):
            reason = 'nonfinite state'; break
        vb = Rotation.from_quat(xn[6:10]).as_matrix().T @ (xn[3:6] - w)
        outside += not kh_domain_status(plant_params, xn[13:17], vb)['inside_assumed_domain']
        commands.append(u.copy())
        x = xn
        times.append((k + 1) * DT)
        states.append(x.copy())
        if abs(x[2] - z_ref) > 15.0:
            reason = 'altitude deviation limit'
        elif np.linalg.norm(x[3:6] - v_ref) > 35.0:
            reason = 'velocity error limit'
        elif np.linalg.norm(x[10:13]) > 30.0:
            reason = 'body rate limit'
        if reason:
            break

    times = np.asarray(times)
    states = np.asarray(states)
    commands = np.asarray(commands).reshape(-1, 4)
    v_refs, z_refs = profile.compute_refs(times)
    ez = states[:, 2] - z_refs
    ev = states[:, 3:6] - v_refs
    saturation = float(np.mean(np.any(
        (commands <= plant_params['n_min'] + 1e-6) |
        (commands >= plant_params['n_max'] - 1e-6), axis=1))) if len(commands) else 0.0
    solve_log = getattr(solver, 'solve_log', []) if solver is not None else []
    n_fail = sum(1 for e in solve_log if not e.get('accepted', True))
    result = dict(
        controller=label, completed=reason is None, stop_reason=reason,
        elapsed_sim_s=float(times[-1]), wall_seconds=perf_counter() - start,
        z_rmse_m=float(np.sqrt(np.mean(ez**2))),
        vel_rmse_m_s=float(np.sqrt(np.mean(np.sum(ev**2, axis=1)))),
        saturation_fraction=saturation,
        prop_domain_outside_fraction=outside / max(len(commands), 1),
        optimizer_failures=n_fail, optimizer_calls=len(solve_log),
        max_omega=float(np.max(np.linalg.norm(states[:, 10:13], axis=1))))
    print(f"  {label:16s}  종료={result['elapsed_sim_s']:6.2f}s/{profile.T_total:.0f}s  "
          f"완주={result['completed']!s:5s}  z_RMSE={result['z_rmse_m']:.3f}  "
          f"v_RMSE={result['vel_rmse_m_s']:.3f}  |w|max={result['max_omega']:.2f}  "
          f"sat={100*saturation:.1f}%  솔버실패={n_fail}/{len(solve_log)}  정지={reason}")
    return result


class _TheirFactoryAdapter:
    """KHFactory(native)를 run_mission의 ctrl_factory(profile) 시그니처에 맞춘다."""

    def __init__(self, factory, label):
        self.factory = factory
        self.label = label

    def __call__(self, profile):
        ctrl = self.factory.make(self.label, 0.0, 2.0)
        solver = solver_of(ctrl)
        return ctrl, solver

    def update(self, ctrl, v, z):
        self.factory.update(ctrl, v, z)


def make_our_v13_mission(ctrl_params, profile):
    def ref_fn(t):
        v, z, _ = profile.get_ref(t)
        return np.array([v[0], v[1], v[2], z])

    nmpc = VirtualNMPC(ctrl_params, v_ref=[0, 0, 0], z_ref=2.0, dt_ctrl=0.02,
                       cost_spec='paper', ref_fn=ref_fn)
    _install_solve_log_spy(nmpc)
    hyb = ProperHybrid(nmpc, ctrl_params, dt=DT, time_align='S1')
    return hyb, nmpc


def main():
    native = kh_native_params()
    cp = build_controller_params(native)
    profile = KHMissionProfile(70.0, 50.0)
    print(f"미션 총 {profile.T_total:.0f}s, 순항 {profile.cruise_start:.0f}-"
          f"{profile.cruise_end:.0f}s, 돌풍 35-36s(수직 10m/s)")

    print("\n=== 팀원 자신의 구현(그대로) — docs/BASELINE_V2_RESULTS.md §4 재현 확인 ===")
    factory = KHFactory(native)
    for label in ('GS-LQR', 'INDI', 'NMPC', 'Naive', 'Split'):
        run_mission(native, _TheirFactoryAdapter(factory, label), f'{label}(팀원)', profile=profile)

    print("\n=== 우리 V13(S1·논문비용, ref_fn으로 미션 참조 연결) ===")
    def our_factory(profile):
        return make_our_v13_mission(cp, profile)
    run_mission(native, our_factory, 'V13(우리,S1+paper)', profile=profile)


if __name__ == '__main__':
    main()
