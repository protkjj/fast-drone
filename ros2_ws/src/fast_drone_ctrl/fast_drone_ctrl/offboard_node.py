"""
PX4 Offboard 제어 노드 — 직접 액추에이터(모터) 제어
====================================================

PX4 SITL ↔ ROS2 통신 흐름:

  PX4 SITL
    │ (micro-XRCE-DDS)
    ├── 발행 → VehicleOdometry     → 이 노드가 구독 (상태 수신)
    ├── 발행 → VehicleStatus       → 이 노드가 구독 (arming 상태)
    ├── 구독 ← OffboardControlMode ← 이 노드가 발행 (제어 모드)
    ├── 구독 ← VehicleCommand      ← 이 노드가 발행 (ARM/OFFBOARD)
    └── 구독 ← ActuatorMotors      ← 이 노드가 발행 (모터 명령)

제어 루프:
  1. VehicleOdometry → NED→NWU 변환 → 상태 벡터 x(17) 조립
  2. 제어기 호출: u = controller(t, x) → 모터 속도 [rad/s]
  3. 정규화 [0,1] → ActuatorMotors 발행

Offboard 진입 절차 (PX4 요구사항):
  1. OffboardControlMode를 최소 10회 이상 발행 (ARM 전)
  2. Offboard 모드 전환 명령
  3. ARM 명령
  4. 이후 ActuatorMotors 연속 발행
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

# px4_msgs 타입들
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
    ActuatorMotors,
    VehicleAngularVelocity,
)

from .frame_utils import (
    pos_ned_to_nwu,
    vel_ned_to_nwu,
    quat_scalar_first_to_last,
    quat_ned_to_nwu,
    motor_speed_to_normalized,
)
from .safety import SafetyGuard


# ════════════════════════════════════════════════════
# QoS 프로파일 (PX4 micro-XRCE-DDS 호환)
# ════════════════════════════════════════════════════

# PX4 토픽은 "best effort" + "transient local" 조합 사용
_px4_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OffboardController(Node):
    """
    PX4 Offboard 모터 직접 제어 노드.

    사용법 (launch 파일 또는 터미널):
        ros2 run fast_drone_ctrl offboard_node
            --ros-args
            -p controller_type:=pid
            -p v_ref_x:=30.0
            -p z_ref:=50.0
    """

    def __init__(self):
        super().__init__('offboard_controller')

        # ── 파라미터 선언 ──
        self.declare_parameter('controller_type', 'pid')
        self.declare_parameter('v_ref_x', 0.0)
        self.declare_parameter('v_ref_y', 0.0)
        self.declare_parameter('v_ref_z', 0.0)
        self.declare_parameter('z_ref', 10.0)
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('n_max', 1800.0)
        self.declare_parameter('arm_delay_sec', 2.0)

        # 파라미터 읽기
        ctrl_type = self.get_parameter('controller_type').value
        vx = self.get_parameter('v_ref_x').value
        vy = self.get_parameter('v_ref_y').value
        vz = self.get_parameter('v_ref_z').value
        self.z_ref = self.get_parameter('z_ref').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.n_max = self.get_parameter('n_max').value
        arm_delay = self.get_parameter('arm_delay_sec').value

        self.v_ref = np.array([vx, vy, vz])
        self.dt = 1.0 / self.control_rate

        # ── 안전장치 ──
        self.safety = SafetyGuard(
            n_max=self.n_max,
            dt=self.dt,
        )

        # ── 제어기 생성 ──
        self.controller = self._create_controller(ctrl_type)
        self.get_logger().info(
            f'제어기: {ctrl_type}, v_ref={self.v_ref}, z_ref={self.z_ref}, '
            f'rate={self.control_rate}Hz'
        )

        # ── 상태 변수 ──
        self._state_valid = False           # 첫 상태 수신 여부
        self._pos_ned = np.zeros(3)
        self._vel_ned = np.zeros(3)
        self._q_ned_sf = np.array([1.0, 0.0, 0.0, 0.0])  # scalar-first
        self._omega_body = np.zeros(3)
        self._motor_speeds = np.zeros(4)    # 우리가 보낸 명령 추적
        self._vehicle_armed = False
        self._vehicle_status = None
        self._offboard_setpoint_count = 0
        self._t_start = None                # 제어 시작 시각
        self._arm_requested = False

        # ARM 전 offboard 메시지 발행 횟수 (PX4 요구: 최소 10)
        self._pre_arm_count = int(arm_delay * self.control_rate)

        # ── 퍼블리셔 ──
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _px4_qos)
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _px4_qos)
        self.pub_actuator = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', _px4_qos)

        # ── 서브스크라이버 ──
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._odom_callback, _px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_callback, _px4_qos)
        self.create_subscription(
            VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity',
            self._gyro_callback, _px4_qos)

        # ── 제어 타이머 ──
        self._timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info('노드 초기화 완료. PX4 연결 대기 중...')

    # ════════════════════════════════════════════════
    # 콜백: PX4 → ROS2
    # ════════════════════════════════════════════════

    def _odom_callback(self, msg: VehicleOdometry):
        """VehicleOdometry 수신 → 내부 상태 업데이트."""
        self._pos_ned = np.array(msg.position, dtype=float)
        self._vel_ned = np.array(msg.velocity, dtype=float)
        self._q_ned_sf = np.array(msg.q, dtype=float)

        if not self._state_valid:
            self._state_valid = True
            self.get_logger().info(
                f'첫 상태 수신! pos_ned={self._pos_ned}'
            )

    def _gyro_callback(self, msg: VehicleAngularVelocity):
        """VehicleAngularVelocity 수신 → 동체 각속도."""
        self._omega_body = np.array(msg.xyz, dtype=float)

    def _status_callback(self, msg: VehicleStatus):
        """VehicleStatus 수신 → arming 상태 추적."""
        self._vehicle_status = msg
        was_armed = self._vehicle_armed
        self._vehicle_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

        if self._vehicle_armed and not was_armed:
            self.get_logger().info('기체 ARMED')
            self._t_start = self.get_clock().now()
        elif not self._vehicle_armed and was_armed:
            self.get_logger().warn('기체 DISARMED')

    # ════════════════════════════════════════════════
    # 메인 제어 루프
    # ════════════════════════════════════════════════

    def _control_loop(self):
        """
        타이머 콜백: control_rate_hz 주기로 실행.

        Phase 1: Offboard 모드 진입 전 — 제어 모드 메시지만 발행
        Phase 2: ARM 후 — 제어기 실행 + 모터 명령 발행
        """
        # 항상 OffboardControlMode 발행 (끊기면 failsafe 발동)
        self._publish_offboard_mode()
        self._offboard_setpoint_count += 1

        # Phase 1: ARM 전 준비
        if not self._arm_requested:
            if self._offboard_setpoint_count >= self._pre_arm_count:
                if self._state_valid:
                    self.get_logger().info(
                        f'Offboard 진입 요청 '
                        f'({self._offboard_setpoint_count}회 발행 완료)'
                    )
                    self._send_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                        param1=1.0,   # base mode
                        param2=6.0,   # PX4 custom mode: Offboard
                    )
                    self._send_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        param1=1.0,   # 1 = ARM
                    )
                    self._arm_requested = True
                else:
                    # 상태 미수신 → 대기
                    if self._offboard_setpoint_count % 100 == 0:
                        self.get_logger().warn('상태 데이터 대기 중...')

            # ARM 전에도 제로 모터 명령 발행 (PX4가 기대)
            self._publish_zero_motors()
            return

        # Phase 2: 제어기 실행
        if not self._state_valid:
            self._publish_zero_motors()
            return

        # 상태 벡터 조립 (NED → NWU 변환)
        x = self._assemble_state()

        # 시간 계산
        if self._t_start is None:
            t = 0.0
        else:
            dt_ros = self.get_clock().now() - self._t_start
            t = dt_ros.nanoseconds * 1e-9

        # 제어기 호출: u = controller(t, x) → 모터 속도 [rad/s]
        try:
            u_raw = self.controller(t, x)
        except Exception as e:
            self.get_logger().error(f'제어기 예외: {e}')
            u_raw = np.full(4, float('nan'))  # 안전장치가 폴백 처리

        # 안전장치 통과 (NaN 감지, 변화율 제한, 자세/고도 체크)
        u = self.safety.check(u_raw, state=x)

        if self.safety.triggered:
            self.get_logger().warn(
                f'[안전] {self.safety.level.name}: {self.safety.trigger_reason}'
            )

        # 모터 명령 발행
        self._publish_motors(u)

        # 모터 속도 추적 (1차 지연 모델로 내부 추정)
        tau_m = 0.02  # 모터 시상수
        alpha = self.dt / (self.dt + tau_m)
        self._motor_speeds = alpha * u + (1 - alpha) * self._motor_speeds

        # 주기적 상태 로그
        if self._offboard_setpoint_count % int(self.control_rate * 2) == 0:
            pos_nwu = pos_ned_to_nwu(self._pos_ned)
            vel_nwu = vel_ned_to_nwu(self._vel_ned)
            self.get_logger().info(
                f't={t:.1f}s | '
                f'pos=[{pos_nwu[0]:.1f}, {pos_nwu[1]:.1f}, {pos_nwu[2]:.1f}] | '
                f'vel=[{vel_nwu[0]:.1f}, {vel_nwu[1]:.1f}, {vel_nwu[2]:.1f}] | '
                f'u_avg={np.mean(u):.0f} rad/s'
            )

    # ════════════════════════════════════════════════
    # 상태 벡터 조립
    # ════════════════════════════════════════════════

    def _assemble_state(self):
        """
        PX4 메시지 데이터 → 우리 상태 벡터 x(17).

        x = [pos(3), vel(3), quat(4), omega(3), motors(4)]
        """
        pos_nwu = pos_ned_to_nwu(self._pos_ned)
        vel_nwu = vel_ned_to_nwu(self._vel_ned)

        # 쿼터니언: PX4 scalar-first → scalar-last → NED→NWU
        q_ned_sl = quat_scalar_first_to_last(self._q_ned_sf)
        q_nwu = quat_ned_to_nwu(q_ned_sl)

        # 부호 정규화 (연속성)
        if q_nwu[3] < 0:
            q_nwu = -q_nwu

        # 동체 각속도: FRD 프레임 공통이므로 변환 불필요
        omega = self._omega_body

        # 로터 속도: 우리가 보낸 명령으로 추정 (모터 동역학 1차 지연 적용됨)
        motors = self._motor_speeds

        return np.concatenate([pos_nwu, vel_nwu, q_nwu, omega, motors])

    # ════════════════════════════════════════════════
    # PX4 메시지 발행
    # ════════════════════════════════════════════════

    def _publish_offboard_mode(self):
        """OffboardControlMode 발행 — 직접 액추에이터 제어."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # 모든 상위 레벨 비활성, 직접 액추에이터만 활성
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = True
        self.pub_offboard_mode.publish(msg)

    def _publish_motors(self, motor_speeds):
        """ActuatorMotors 발행 — 모터 속도를 정규화하여 전송."""
        msg = ActuatorMotors()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        normalized = motor_speed_to_normalized(motor_speeds, self.n_max)

        # 모터 매핑 (우리 r1~r4 → PX4 motor 1~4)
        # 커스텀 에어프레임에서 매핑을 일치시키므로 순서 동일
        for i in range(4):
            msg.control[i] = float(normalized[i])
        # 나머지 채널 NaN (미사용)
        for i in range(4, 16):
            msg.control[i] = float('nan')

        self.pub_actuator.publish(msg)

    def _publish_zero_motors(self):
        """모터 정지 명령."""
        self._publish_motors(np.zeros(4))

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0,
                               param3=0.0, param4=0.0,
                               param5=0.0, param6=0.0, param7=0.0):
        """VehicleCommand 발행 (ARM, 모드 전환 등)."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_vehicle_cmd.publish(msg)

    # ════════════════════════════════════════════════
    # 제어기 생성
    # ════════════════════════════════════════════════

    def _create_controller(self, ctrl_type):
        """
        파라미터로 지정된 제어기 생성.

        이 함수는 우리 기존 제어기를 그대로 사용.
        controllers/ 디렉토리의 모듈을 임포트.

        Returns
        -------
        callable : (t, x) → u[4]  모터 속도 [rad/s]
        """
        # 지연 임포트 (ROS2 빌드 시 casadi 없어도 패키지 빌드 가능)
        from .controllers.vehicle_params import vehicle_params as P
        from .controllers.controller import (
            CascadedPID, ScheduledPID, LQRController, ScheduledLQR,
            INDIController,
        )

        dt = self.dt
        v_ref = self.v_ref.tolist()
        z_ref = self.z_ref

        if ctrl_type == 'pid':
            self.get_logger().info('CascadedPID 생성')
            return CascadedPID(P, v_ref=v_ref, z_ref=z_ref, dt=dt)

        elif ctrl_type == 'scheduled_pid':
            self.get_logger().info('ScheduledPID 생성')
            return ScheduledPID(P, v_ref=v_ref, z_ref=z_ref, dt=dt)

        elif ctrl_type == 'lqr':
            from .controllers.trim import find_trim
            V = self.v_ref[0]
            trim = find_trim(P, float(V))
            self.get_logger().info(f'LQR 생성 (V_trim={V} m/s)')
            ctrl = LQRController(P, trim['state'], trim['control'])
            ctrl.set_position_ref(np.array([0.0, 0.0, z_ref]))
            return ctrl

        elif ctrl_type == 'scheduled_lqr':
            self.get_logger().info('ScheduledLQR 생성')
            return ScheduledLQR(P, v_ref=v_ref, z_ref=z_ref)

        elif ctrl_type == 'indi':
            self.get_logger().info('INDI 생성')
            return INDIController(P, v_ref=v_ref, z_ref=z_ref, dt=dt)

        elif ctrl_type == 'nmpc':
            from .controllers.nmpc import NMPCController
            self.get_logger().info('NMPC 생성 (CasADi+IPOPT)')
            return NMPCController(P, v_ref=v_ref, z_ref=z_ref)

        else:
            raise ValueError(
                f"알 수 없는 제어기: '{ctrl_type}'. "
                f"가능한 값: pid, scheduled_pid, lqr, scheduled_lqr, indi, nmpc"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 요청 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
