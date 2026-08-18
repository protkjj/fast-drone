"""
control — 고속 미사일형 쿼드 드론 제어/시뮬레이션 패키지

이 패키지 안의 모듈은 서로를 절대 import 로 참조한다:
    from control.dynamics import build_dynamics

따라서 실행은 반드시 저장소 루트에서 모듈 형태로 한다:
    python3 -m control.mission_sim          (O)
    python3 control/mission_sim.py          (X — sys.path 에 루트가 없어 실패)

[왜 -m 인가]
파이썬은 `python3 control/mission_sim.py` 로 실행하면 sys.path[0] 에
스크립트가 있는 `control/` 을 넣는다. 저장소 루트가 경로에 없으므로
`import control` 자체가 실패한다. `-m` 은 cwd(=루트)를 sys.path 에 넣어
`control` 을 패키지로 인식시킨다.
"""
