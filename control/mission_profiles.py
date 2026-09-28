"""Independent nominal-performance and cruise/gust reference trajectories."""
import numpy as np

# Empirical 70 m/s timing study: docs/TIMING_OPTIMIZATION.md.
# These are numerical performance-test settings, not a model-validity claim.
DEFAULT_MISSION_DURATIONS = (1.0, 8.0, 3.0, 25.0, 3.0)
DEFAULT_GUST_TIMES = (1.0, 1.0, 2.5)  # lead, pulse, recovery; 70 m/s, peak 2 m/s


class MissionProfile:
    """Hover -> accelerate -> cruise -> decelerate -> hover, with no takeoff.

    Times are seconds and altitude is inertial z-up in metres. Both ramps use
    the existing half-cosine interpolation; endpoint acceleration is zero.
    """

    def __init__(self, cruise_speed=70.0, cruise_alt=50.0,
                 durations=DEFAULT_MISSION_DURATIONS):
        if not np.isfinite(cruise_speed) or cruise_speed < 0:
            raise ValueError('cruise_speed must be finite and nonnegative')
        if not np.isfinite(cruise_alt) or cruise_alt <= 0:
            raise ValueError('cruise_alt must be finite and positive')
        if len(durations) != 5 or not np.all(np.isfinite(durations)) or min(durations) <= 0:
            raise ValueError('five finite positive phase durations are required')
        self.cruise_speed, self.cruise_alt = cruise_speed, cruise_alt
        V, Z = cruise_speed, cruise_alt
        self.phases = []
        t = 0.0
        for name, duration, v0, v1 in zip(
                ('초기호버', '가속', '순항', '감속', '호버링'), durations,
                (0, 0, V, V, 0), (0, V, V, 0, 0)):
            self.phases.append((name, t, float(duration), v0, v1, Z, Z))
            t += duration
        self.T_total = float(t)
        self.cruise_start, self.cruise_end = self.phase_interval('순항')
        self.decel_start, self.decel_end = self.phase_interval('감속')
        self.gust_interval = None

    def phase_interval(self, name):
        for phase, t, duration, *_ in self.phases:
            if phase == name:
                return t, t + duration
        raise ValueError(f'unknown phase: {name}')

    def get_ref(self, t):
        for i, (name, start, duration, v0, v1, z0, z1) in enumerate(self.phases):
            if t < start + duration or i == len(self.phases) - 1:
                u = np.clip((t - start) / duration, 0.0, 1.0)
                s = 0.5 * (1.0 - np.cos(np.pi * u))
                return np.array([v0 + (v1-v0)*s, 0.0, 0.0]), z0+(z1-z0)*s, name

    def compute_refs(self, ts):
        ts = np.asarray(ts)
        velocities = np.zeros((len(ts), 3))
        altitudes = np.empty(len(ts))
        for i, (_, start, duration, v0, v1, z0, z1) in enumerate(self.phases):
            mask = (ts >= start) & (ts < start+duration)
            if i == 0:
                mask |= ts < start
            if i == len(self.phases)-1:
                mask |= ts >= start+duration
            u = np.clip((ts[mask]-start)/duration, 0.0, 1.0)
            s = 0.5*(1.0-np.cos(np.pi*u))
            velocities[mask, 0] = v0+(v1-v0)*s
            altitudes[mask] = z0+(z1-z0)*s
        return velocities, altitudes

    def get_phase_boundaries(self):
        return [(name, t, t+duration) for name, t, duration, *_ in self.phases]

    def print_profile(self):
        for name, start, end in self.get_phase_boundaries():
            print(f'  {name}: {start:g}--{end:g} s')
        print(f'  Total: {self.T_total:g} s')


class SmoothstepProfile(MissionProfile):
    """Hold -> paper eq.(32) speed change -> hold (reference-profile tests).

    The ramp uses control.fair_compare.smoothstep, whose first four
    derivatives vanish at both ends. The mission above keeps its half-cosine
    ramps; the two shapes are deliberately not mixed.
    """

    def __init__(self, v0, v1, cruise_alt, lead, ramp, tail):
        if not np.all(np.isfinite([v0, v1, cruise_alt, lead, ramp, tail])):
            raise ValueError('smoothstep profile values must be finite')
        if min(lead, ramp, tail) <= 0 or cruise_alt <= 0 or min(v0, v1) < 0:
            raise ValueError('positive durations/altitude and nonnegative speeds are required')
        self.cruise_speed, self.cruise_alt = max(v0, v1), cruise_alt
        Z = cruise_alt
        self.phases = [('유지', 0.0, float(lead), v0, v0, Z, Z),
                       ('기동', float(lead), float(ramp), v0, v1, Z, Z),
                       ('정착', float(lead+ramp), float(tail), v1, v1, Z, Z)]
        self.T_total = float(lead+ramp+tail)
        self.ramp_start, self.ramp_end = float(lead), float(lead+ramp)
        self.cruise_start, self.cruise_end = self.phase_interval('유지')
        self.decel_start = self.decel_end = None
        self.gust_interval = None

    @staticmethod
    def shape(u):
        from control.fair_compare import smoothstep
        return np.vectorize(smoothstep, otypes=[float])(u)

    def get_ref(self, t):
        for i, (name, start, duration, v0, v1, z0, z1) in enumerate(self.phases):
            if t < start + duration or i == len(self.phases) - 1:
                s = float(self.shape((t - start) / duration))
                return np.array([v0 + (v1-v0)*s, 0.0, 0.0]), z0+(z1-z0)*s, name

    def compute_refs(self, ts):
        ts = np.asarray(ts)
        velocities = np.zeros((len(ts), 3))
        altitudes = np.empty(len(ts))
        for i, (_, start, duration, v0, v1, z0, z1) in enumerate(self.phases):
            mask = (ts >= start) & (ts < start+duration)
            if i == 0:
                mask |= ts < start
            if i == len(self.phases)-1:
                mask |= ts >= start+duration
            s = self.shape((ts[mask]-start)/duration)
            velocities[mask, 0] = v0+(v1-v0)*s
            altitudes[mask] = z0+(z1-z0)*s
        return velocities, altitudes


class StepProfile:
    """트림 속도·고도에서 t=STEP_TIME부터 계단 하나. 스위트 profile 규약을 따른다.

    ramp_s > 0이면 식(32) smoothstep으로 ramp_s초에 걸쳐 바뀌는 매끄러운 계단, 0이면 옛 원계단이다.
    `control/arena_design_check.py`(설계점검)와 `control/arena.py`의 'step' 튜닝 시나리오가 같이 쓴다
    — 두 모듈이 서로를 참조하면 순환 임포트가 나서(arena.py → arena_design_check.py → arena.py)
    여기(둘 다 임포트하는 하위 모듈)로 옮겼다. `control/arena_design_check.py`의 `STEP_TIME`(같은 값
    0.5)은 `step_metrics`가 따로 쓰므로 중복해 둔다.
    """

    STEP_TIME = 0.5

    def __init__(self, V, z, dv, dz, duration=15.0, ramp_s=1.0):
        self.V, self.z, self.dv, self.dz = float(V), float(z), float(dv), float(dz)
        self.ramp_s = float(ramp_s)
        self.T_total = float(duration)
        t0 = self.STEP_TIME
        self.phases = [('트림', 0.0, t0, V, V, z, z),
                       ('계단', t0, duration - t0, V + dv, V + dv, z + dz, z + dz)]
        self.gust_interval = None
        self.cruise_start, self.cruise_end = 0.0, self.T_total
        self.decel_start = self.decel_end = None

    def _progress(self, ts):
        """계단 진행도 s ∈ [0, 1] — 원계단이면 0/1, 매끄러운 계단이면 smoothstep((t−0.5)/전이 시간)."""
        ts = np.asarray(ts, dtype=float)
        return SmoothstepProfile.shape(np.clip((ts - self.STEP_TIME)/self.ramp_s, 0.0, 1.0))

    def get_ref(self, t):
        if not self.ramp_s:                         # 옛 원계단 — 기록을 비트 단위로 재현하려고 그대로 둔다
            after = t >= self.STEP_TIME
            return (np.array([self.V + self.dv*after, 0.0, 0.0]), self.z + self.dz*after,
                    '계단' if after else '트림')
        s = float(self._progress(t))
        return (np.array([self.V + self.dv*s, 0.0, 0.0]), self.z + self.dz*s,
                '계단' if t >= self.STEP_TIME else '트림')

    def compute_refs(self, ts):
        if not self.ramp_s:
            after = np.asarray(ts) >= self.STEP_TIME
            v = np.zeros((len(after), 3))
            v[:, 0] = self.V + self.dv*after
            return v, self.z + self.dz*after
        s = self._progress(ts)
        v = np.zeros((len(s), 3))
        v[:, 0] = self.V + self.dv*s
        return v, self.z + self.dz*s

    def get_phase_boundaries(self):
        return [(name, t, t + d) for name, t, d, *_ in self.phases]


class GustProfile(MissionProfile):
    """Start at cruise trim, establish flight, apply a gust, then recover."""

    def __init__(self, cruise_speed=70.0, cruise_alt=50.0,
                 settle=DEFAULT_GUST_TIMES[0], gust_duration=DEFAULT_GUST_TIMES[1],
                 recovery=DEFAULT_GUST_TIMES[2]):
        super().__init__(cruise_speed, cruise_alt)
        if not np.all(np.isfinite([settle, gust_duration, recovery])) or min(
                settle, gust_duration, recovery) <= 0:
            raise ValueError('settle, gust_duration and recovery must be positive')
        V, Z = cruise_speed, cruise_alt
        self.phases = [
            ('순항', 0.0, settle, V, V, Z, Z),
            ('돌풍', settle, gust_duration, V, V, Z, Z),
            ('회복', settle+gust_duration, recovery, V, V, Z, Z),
        ]
        self.T_total = settle+gust_duration+recovery
        self.cruise_start, self.cruise_end = 0.0, self.T_total
        self.decel_start = self.decel_end = None
        self.gust_interval = (settle, settle+gust_duration)
