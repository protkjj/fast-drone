"""Existing 65-second mission reference only; no old-model run entry point.

Run python -m models.team_light.control.run_baseline_comparison --stage all to simulate.
"""
import numpy as np


class MissionProfile:
    """
    비행 구간별 기준 궤적 생성.

    각 구간 내에서 1-cos 보간: 시작/끝 가속도 = 0 (매끄러운 전환).
    """

    def __init__(self, cruise_speed=70.0, cruise_alt=50.0):
        V = cruise_speed
        Z = cruise_alt

        # (이름, t_start, duration, vx_start, vx_end, z_start, z_end)
        self.phases = [
            ('이륙',    0.0,  10.0,   0.0,  0.0,   2.0,    Z),
            ('안정화',  10.0,   3.0,   0.0,  0.0,     Z,    Z),
            ('가속',   13.0,  15.0,   0.0,    V,     Z,    Z),
            ('순항',   28.0,  15.0,     V,    V,     Z,    Z),
            ('감속',   43.0,  15.0,     V,  0.0,     Z,    Z),
            ('호버링',  58.0,   7.0,   0.0,  0.0,     Z,    Z),
        ]
        self.T_total = sum(p[2] for p in self.phases)  # 65초
        self.cruise_speed = cruise_speed
        self.cruise_alt = cruise_alt

        # 순항 구간 시간 범위 (돌풍 랜덤화에 사용)
        cruise = self.phases[3]
        self.cruise_start = cruise[1]
        self.cruise_end = cruise[1] + cruise[2]

    def get_ref(self, t):
        """시각 t에서의 기준값. Returns: (v_ref[3], z_ref, phase_name)"""
        for name, t_start, dur, vx0, vx1, z0, z1 in self.phases:
            t_end = t_start + dur
            if t < t_end or name == self.phases[-1][0]:
                tau = np.clip((t - t_start) / dur, 0.0, 1.0)
                s = 0.5 * (1.0 - np.cos(np.pi * tau))
                vx = vx0 + (vx1 - vx0) * s
                z = z0 + (z1 - z0) * s
                return np.array([vx, 0.0, 0.0]), z, name

        last = self.phases[-1]
        return np.array([last[4], 0.0, 0.0]), last[6], last[0]

    def compute_refs(self, ts):
        """시간 배열에서 기준 궤적 벡터화 계산."""
        v_refs = np.zeros((len(ts), 3))
        z_refs = np.zeros(len(ts))

        for name, t_start, dur, vx0, vx1, z0, z1 in self.phases:
            t_end = t_start + dur
            mask = (ts >= t_start) & (ts < t_end)
            if not np.any(mask):
                continue
            tau = np.clip((ts[mask] - t_start) / dur, 0.0, 1.0)
            s = 0.5 * (1.0 - np.cos(np.pi * tau))
            v_refs[mask, 0] = vx0 + (vx1 - vx0) * s
            z_refs[mask] = z0 + (z1 - z0) * s

        last = self.phases[-1]
        mask = ts >= (last[1] + last[2])
        v_refs[mask, 0] = last[4]
        z_refs[mask] = last[6]

        return v_refs, z_refs

    def get_phase_boundaries(self):
        return [(name, t0, t0 + dur)
                for name, t0, dur, *_ in self.phases]

    def print_profile(self):
        print(f"\n  {'구간':>6s}  {'시간':>10s}  "
              f"{'v_x [m/s]':>12s}  {'z [m]':>10s}  {'최대가속':>10s}")
        print(f"  {'─'*56}")
        for name, t0, dur, vx0, vx1, z0, z1 in self.phases:
            t1 = t0 + dur
            v_str = f"{vx0:.0f} -> {vx1:.0f}" if vx0 != vx1 else f"{vx0:.0f}"
            z_str = f"{z0:.0f} -> {z1:.0f}" if z0 != z1 else f"{z0:.0f}"
            if dur > 0 and vx0 != vx1:
                a_str = f"{np.pi/2*abs(vx1-vx0)/dur:.1f} m/s2"
            elif dur > 0 and z0 != z1:
                a_str = f"{np.pi/2*abs(z1-z0)/dur:.1f} m/s2(z)"
            else:
                a_str = "-"
            print(f"  {name:>6s}  {t0:>4.0f}~{t1:>4.0f}s  "
                  f"{v_str:>12s}  {z_str:>10s}  {a_str:>10s}")
        print(f"\n  총 시뮬 시간: {self.T_total:.0f}초")
