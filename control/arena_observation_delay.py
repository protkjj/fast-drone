"""관측 지연 버퍼 — 상태 공급 지점(경기장이 매 스텝 컨트롤러에 주는 x)에 건다.

kj 결정: 센서(측정값 공급) 수준 구현. 회전수를 읽는 모든 컨트롤러가 같은 지연값을 받는다.
컨트롤러별로 다르게 주지 않는다.

두 슬라이스를 독립 지연:
  state  x[0:13](위치·속도·쿼터니언·각속도) — 10·30 ms
  rotor  x[13:17](회전수) — 5 ms

dt=0.002s라 5ms는 정확한 정수 스텝이 아니다(2.5). round()로 가장 가까운 정수 스텝(2, 4ms)을
쓴다 — 근사임을 여기 기록한다.

지연 0(또는 미설정)이면 원본 x를 그대로 돌려준다(비트 동일).
"""
import numpy as np


class DelayedObservation:
    def __init__(self, dt, state_delay_s=0.0, rotor_delay_s=0.0):
        self.dt = float(dt)
        self.k_state = int(round(float(state_delay_s)/self.dt))
        self.k_rotor = int(round(float(rotor_delay_s)/self.dt))
        self._history = []

    def reset(self):
        """새 시행 시작 — 이전 시행의 기록을 비운다. observe()의 첫 호출이 기록을 새로
        시작한다(그때는 지연이 걸릴 과거가 없어 지금 값을 그대로 낸다)."""
        self._history = []

    def observe(self, x_true):
        self._history.append(np.asarray(x_true, dtype=float).copy())
        k_max = max(self.k_state, self.k_rotor)
        if len(self._history) > k_max + 1:
            del self._history[:-(k_max + 1)]
        i_state = max(0, len(self._history) - 1 - self.k_state)
        i_rotor = max(0, len(self._history) - 1 - self.k_rotor)
        x_obs = self._history[i_state].copy()
        x_obs[13:17] = self._history[i_rotor][13:17]
        return x_obs
