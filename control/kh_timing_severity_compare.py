"""kj 작업지시서(2026-09-25 저녁) 2단계 — 가감속 설정 두 가지 비교:
"팀원이 조정한 완화 설정과 완화 전 원래 설정".

민석의 `docs/TIMING_OPTIMIZATION.md`가 고른 "완화" 구간표(가속8s/
크루즈3s/감속25s)를 그대로 가져다, **규현 기체 + 현재(병합 이후 최신)
우리 V13** 조합에 처음으로 적용한다 — 민석 자신의 연구는 옛 8kg
플레이스홀더 기체 기준이었음을 확인했다(`control/kh_adapter.py`,
`results/SOLVER_FAILURE_LOG_2026-09-25.md` 참고). "원래" 구간표
(가속/순항/감속 15/15/15s)는 규현의 65초 미션(`kh_mission_repro.py`)
및 민석의 "이전" 값과 일치한다(둘 다 근거가 됨).

돌풍은 끈다(`gust_fn=lambda t: np.zeros(3)`) — 규현의 65초 미션은
가감속 구조가 다른 프로파일에 하드코딩된 t=35s 돌풍을 그대로 쓰면
프로파일마다 다른 위상(가속중/감속중/호버중)에 걸려 가감속 난이도
비교 자체가 오염된다.

**주의(2026-09-25 세션에서 시간상 미완료)**: 6개 미션(3속도×2설정)을
새로 NLP를 지어 도는 게 무거워 한 세션 안에 못 끝냈다. 다음 세션에서
이 파일을 그대로 실행해 이어서 확인할 것.

실행: python3 -m control.kh_timing_severity_compare
"""
import numpy as np

from control.kh_adapter import kh_native_params, build_controller_params
from control.kh_mission_repro import run_mission, make_our_v13_mission


class SimpleMissionProfile:
    """`kh_control.mission_sim.MissionProfile`과 같은 인터페이스
    (get_ref/compute_refs/T_total), 구간표만 바꿀 수 있게 만든 최소 버전."""

    def __init__(self, phases, cruise_speed):
        # phases: [(name, t_start, dur, vx0, vx1, z0, z1), ...]
        self.phases = phases
        self.T_total = sum(p[2] for p in phases)
        self.cruise_speed = cruise_speed

    def get_ref(self, t):
        for name, t_start, dur, vx0, vx1, z0, z1 in self.phases:
            t_end = t_start + dur
            if t < t_end or name == self.phases[-1][0]:
                tau = np.clip((t - t_start) / dur, 0.0, 1.0)
                s = 0.5 * (1.0 - np.cos(np.pi * tau))
                return (np.array([vx0 + (vx1 - vx0) * s, 0.0, 0.0]),
                        z0 + (z1 - z0) * s, name)
        last = self.phases[-1]
        return np.array([last[4], 0.0, 0.0]), last[6], last[0]

    def compute_refs(self, ts):
        v_refs = np.zeros((len(ts), 3))
        z_refs = np.zeros(len(ts))
        for i, t in enumerate(ts):
            v, z, _ = self.get_ref(t)
            v_refs[i] = v
            z_refs[i] = z
        return v_refs, z_refs


def original_profile(V, Z=50.0):
    """규현/민석 "이전"(15/15/15) 구조 — 이륙(고도 상승) 단계는 생략하고
    이미 순항고도에서 시작(민석 자신의 완화 연구도 같은 축약을 썼다)."""
    return SimpleMissionProfile([
        ('초기호버', 0.0, 3.0, 0.0, 0.0, Z, Z),
        ('가속', 3.0, 15.0, 0.0, V, Z, Z),
        ('순항', 18.0, 15.0, V, V, Z, Z),
        ('감속', 33.0, 15.0, V, 0.0, Z, Z),
        ('최종호버', 48.0, 7.0, 0.0, 0.0, Z, Z),
    ], V)


def relaxed_profile(V, Z=50.0):
    """민석 docs/TIMING_OPTIMIZATION.md "새 지속 시간"(가속8/크루즈3/감속25)."""
    return SimpleMissionProfile([
        ('초기호버', 0.0, 1.0, 0.0, 0.0, Z, Z),
        ('가속', 1.0, 8.0, 0.0, V, Z, Z),
        ('순항', 9.0, 3.0, V, V, Z, Z),
        ('감속', 12.0, 25.0, V, 0.0, Z, Z),
        ('최종호버', 37.0, 3.0, 0.0, 0.0, Z, Z),
    ], V)


def main():
    native = kh_native_params()
    cp = build_controller_params(native)

    for V in (70.0, 80.0, 85.0):
        for label, builder in (('원래(15/15/15)', original_profile),
                               ('완화(8/3/25)', relaxed_profile)):
            profile = builder(V)

            def our_factory(profile):
                return make_our_v13_mission(cp, profile)

            run_mission(native, our_factory, f'V13 {V:g}m/s {label}',
                       profile=profile, gust_fn=lambda t: np.zeros(3))


if __name__ == '__main__':
    main()
