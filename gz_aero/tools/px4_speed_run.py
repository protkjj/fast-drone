#!/usr/bin/env python3
"""PX4 자체 속도제어로 직선 가속 시험. 공력이 고속에서 어떻게 나오는지 본다.

    source ~/ros2_ws/install/setup.bash
    python3 gz_aero/tools/px4_speed_run.py --speed 83.3

왜 offboard_node.py 를 안 쓰나:
  그건 **직접 액추에이터 제어**라 우리 제어기(NMPC+INDI)를 통째로 태운다.
  지금 보려는 것은 제어기 성능이 아니라 **공력이 고속에서 나오는 크기**라,
  PX4 기본 속도제어로 도는 게 변수가 적다.

바람은 0 으로 두고 돌린다. 바람과 기체속도를 섞으면 무엇이 원인인지 못 가른다.
    gz topic -t /fast_drone/aero/wind -m gz.msgs.Vector3d -p "x: 0, y: 0, z: 0"
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
HZ = 50.0
NAN = float("nan")


class SpeedRun(Node):
    def __init__(self, a):
        super().__init__("fast_drone_speed_run")
        self.a = a
        self.pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", QOS)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", QOS)
        self.create_subscription(VehicleLocalPosition,
                                 "/fmu/out/vehicle_local_position",
                                 self._on_pos, QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status",
                                 self._on_status, QOS)

        self.v_meas = 0.0
        self.alt = 0.0
        self.armed = False
        self.offboard = False
        self.k = 0
        self.v_peak = 0.0
        self.said = -1
        self.create_timer(1.0 / HZ, self._tick)

        # 단계 경계 [s].  올라가기 -> 가속 -> 유지 -> 감속
        self.t_climb = a.climb
        self.t_ramp = self.t_climb + a.ramp
        self.t_hold = self.t_ramp + a.hold
        self.t_end = self.t_hold + a.ramp

    def _on_pos(self, m):
        self.v_meas = math.hypot(m.vx, m.vy)
        self.alt = -m.z

    def _on_status(self, m):
        self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        m.param1, m.param2 = p1, p2
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def _tick(self):
        t = self.k / HZ
        self.k += 1

        ocm = OffboardControlMode()
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ocm.position = True      # 고도는 위치로 잡고
        ocm.velocity = True      # 수평은 속도로 준다
        self.pub_ocm.publish(ocm)

        # 세트포인트를 충분히 흘린 뒤에야 offboard 로 넘어갈 수 있다 (PX4 규칙)
        if t > 1.0 and not self.offboard:
            self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if t > 1.5 and not self.armed:
            self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if t < self.t_climb:
            v = 0.0
        elif t < self.t_ramp:
            v = self.a.speed * (t - self.t_climb) / max(self.a.ramp, 1e-6)
        elif t < self.t_hold:
            v = self.a.speed
        elif t < self.t_end:
            v = self.a.speed * (1.0 - (t - self.t_hold) / max(self.a.ramp, 1e-6))
        else:
            v = 0.0

        psi = math.radians(self.a.heading)
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # NaN = "이 축은 제어하지 마라". 고도만 위치로 잡는다.
        sp.position = [NAN, NAN, -float(self.a.alt)]
        sp.velocity = [v * math.cos(psi), v * math.sin(psi), NAN]
        sp.acceleration = [NAN, NAN, NAN]
        sp.jerk = [NAN, NAN, NAN]
        sp.yaw = psi
        sp.yawspeed = NAN
        self.pub_sp.publish(sp)

        self.v_peak = max(self.v_peak, self.v_meas)
        if int(t) != self.said:
            self.said = int(t)
            phase = ("상승" if t < self.t_climb else
                     "가속" if t < self.t_ramp else
                     "유지" if t < self.t_hold else
                     "감속" if t < self.t_end else "종료")
            print(f"  t {t:5.1f}  {phase}  명령 {v:6.2f}  실측 {self.v_meas:6.2f} m/s"
                  f"  고도 {self.alt:5.1f} m"
                  f"  {'ARMED' if self.armed else '-----'}"
                  f" {'OFFBOARD' if self.offboard else '--------'}", flush=True)

        if t > self.t_end + 3.0:
            print(f"\n최고 실측 속도 {self.v_peak:.2f} m/s "
                  f"(명령 {self.a.speed:.1f})")
            print("공력 로그 판정:  python3 gz_aero/tools/check_real_vehicle.py")
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=83.3, help="목표 속도 [m/s]")
    ap.add_argument("--alt", type=float, default=40.0, help="고도 [m]")
    ap.add_argument("--climb", type=float, default=10.0, help="상승 시간 [s]")
    ap.add_argument("--ramp", type=float, default=25.0, help="가감속 시간 [s]")
    ap.add_argument("--hold", type=float, default=15.0, help="유지 시간 [s]")
    ap.add_argument("--heading", type=float, default=0.0, help="기수 방위 [deg]")
    a = ap.parse_args()

    print(f"목표 {a.speed} m/s, 고도 {a.alt} m, 방위 {a.heading} deg")
    print("바람이 0 인지 확인하세요. 안 그러면 원인을 못 가릅니다.\n")
    rclpy.init()
    node = SpeedRun(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
