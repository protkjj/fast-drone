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
