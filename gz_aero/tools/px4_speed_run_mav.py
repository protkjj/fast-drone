#!/usr/bin/env python3
"""PX4 를 MAVLink 로 직선 가속시켜 300 km/h 까지 올린다.

    python3 gz_aero/tools/px4_speed_run_mav.py --speed 83.3

왜 MAVLink 인가:
  px4_speed_run.py 는 px4_msgs(ROS2)를 쓰는데 저장소에 벤더링돼 있지 않아
  colcon 빌드가 필요하다. pymavlink 는 PX4 설치에 딸려 와서 바로 쓸 수 있다.
  둘은 같은 일을 한다 — PX4 기본 속도제어에 세트포인트를 흘릴 뿐이다.

왜 offboard_node.py 를 안 쓰나:
  그건 직접 액추에이터 제어라 우리 제어기(NMPC+INDI)를 통째로 태운다.
  지금 보려는 것은 제어기 성능이 아니라 **공력이 고속에서 나오는 크기**다.

바람은 0 으로 두고 돌린다. 바람과 기체속도를 섞으면 원인을 못 가른다.
    gz topic -t /fast_drone/aero/wind -m gz.msgs.Vector3d -p "x: 0, y: 0, z: 0"
"""
import argparse
import math
import time

from pymavlink import mavutil

# SET_POSITION_TARGET_LOCAL_NED 의 type_mask 는 **무시할 항목**에 1 을 세운다.
#   bit0 x, bit1 y, bit2 z, bit3 vx, bit4 vy, bit5 vz,
#   bit6~8 가속도, bit9 force, bit10 yaw, bit11 yaw_rate
# 고도는 위치로(z 사용), 수평은 속도로(vx, vy 사용), 기수는 yaw 로 잡는다.
MASK = (1 << 0) | (1 << 1) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 11)

PARAMS = [
    ("MPC_XY_VEL_MAX", 95.0),     # 수평 속도 상한. 기본 12 라 이걸 안 풀면 못 간다
    ("MPC_XY_CRUISE", 90.0),      # 구형 이름. 없으면 에러 없이 무시된다
    ("MPC_XY_VEL_ALL", 90.0),     # 신형 이름
    ("MPC_TILTMAX_AIR", 45.0),    # 항력을 이기려면 기울여야 한다
    ("MPC_ACC_HOR_MAX", 25.0),
    ("MPC_JERK_MAX", 80.0),
    ("MPC_Z_VEL_MAX_UP", 8.0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=83.3,
                    help="목표 속도 [m/s]  (83.3 = 300 km/h)")
    ap.add_argument("--alt", type=float, default=60.0, help="고도 [m]")
    ap.add_argument("--climb", type=float, default=15.0, help="상승 시간 [s]")
    ap.add_argument("--ramp", type=float, default=30.0, help="가감속 시간 [s]")
    ap.add_argument("--hold", type=float, default=20.0, help="유지 시간 [s]")
    ap.add_argument("--heading", type=float, default=0.0, help="기수 방위 [deg]")
    ap.add_argument("--url", default="udpin:0.0.0.0:14540",
                    help="PX4 SITL 의 onboard MAVLink")
    ap.add_argument("--no-params", action="store_true",
                    help="속도 제한 파라미터를 건드리지 않는다")
    a = ap.parse_args()

    print(f"목표 {a.speed:.1f} m/s ({a.speed * 3.6:.0f} km/h), "
          f"고도 {a.alt:.0f} m, 방위 {a.heading:.0f} deg")
    print(f"연결 {a.url} ...", flush=True)
    m = mavutil.mavlink_connection(a.url)
    m.wait_heartbeat()
    sys_id, comp_id = m.target_system, m.target_component
    print(f"연결됨  system {sys_id}  component {comp_id}\n")

    if not a.no_params:
        for name, val in PARAMS:
            m.mav.param_set_send(sys_id, comp_id, name.encode(), float(val),
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.05)
        print("속도 제한 파라미터 전송 완료 (없는 이름은 PX4 가 무시한다)\n")
        time.sleep(0.5)

    t_climb = a.climb
    t_ramp = t_climb + a.ramp
    t_hold = t_ramp + a.hold
    t_end = t_hold + a.ramp

    psi = math.radians(a.heading)
    t0 = time.time()
    boot = t0
    said = -1
    v_peak = 0.0
    v_meas = 0.0
    alt = 0.0
    armed = False
    mode = ""
    sent_mode = sent_arm = 0.0

    while True:
        t = time.time() - t0

        if t < t_climb:
            v = 0.0
        elif t < t_ramp:
            v = a.speed * (t - t_climb) / max(a.ramp, 1e-6)
        elif t < t_hold:
            v = a.speed
        elif t < t_end:
            v = a.speed * (1.0 - (t - t_hold) / max(a.ramp, 1e-6))
        else:
            v = 0.0

        m.mav.set_position_target_local_ned_send(
            int((time.time() - boot) * 1000), sys_id, comp_id,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK,
            0.0, 0.0, -float(a.alt),                       # 위치 (z 만 유효)
            v * math.cos(psi), v * math.sin(psi), 0.0,     # 속도
            0.0, 0.0, 0.0,                                 # 가속도 (무시)
            psi, 0.0)

        # PX4 규칙: 세트포인트가 먼저 흐르고 있어야 offboard 로 넘어갈 수 있다
        if t > 1.5 and "OFFBOARD" not in mode and time.time() - sent_mode > 1.0:
            sent_mode = time.time()
            m.mav.command_long_send(
                sys_id, comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0)          # PX4 main mode 6 = Offboard
        if t > 2.5 and not armed and time.time() - sent_arm > 1.0:
            sent_arm = time.time()
            m.mav.command_long_send(
                sys_id, comp_id,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)

        msg = m.recv_match(blocking=False)
        while msg is not None:
            typ = msg.get_type()
            if typ == "LOCAL_POSITION_NED":
                v_meas = math.hypot(msg.vx, msg.vy)
                alt = -msg.z
            elif typ == "HEARTBEAT":
                armed = bool(msg.base_mode &
                             mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode = mavutil.mode_string_v10(msg) or ""
            msg = m.recv_match(blocking=False)

        v_peak = max(v_peak, v_meas)
        if int(t) != said:
            said = int(t)
            phase = ("상승" if t < t_climb else "가속" if t < t_ramp else
                     "유지" if t < t_hold else "감속" if t < t_end else "종료")
            print(f"  t {t:5.1f}  {phase}  명령 {v:6.2f}  실측 {v_meas:6.2f} m/s"
                  f" ({v_meas * 3.6:6.1f} km/h)  고도 {alt:5.1f} m"
                  f"  {'ARMED' if armed else '-----'}  {mode}", flush=True)

        if t > t_end + 5.0:
            break
        time.sleep(0.02)          # 50 Hz

    print(f"\n최고 실측 속도 {v_peak:.2f} m/s = {v_peak * 3.6:.1f} km/h "
          f"(명령 {a.speed:.1f} m/s = {a.speed * 3.6:.0f} km/h)")
    print("공력 판정:  python3 gz_aero/tools/check_real_vehicle.py")
    print("그림    :  python3 gz_aero/tools/plot_real_vehicle.py")


if __name__ == "__main__":
    main()
