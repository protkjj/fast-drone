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

# PX4 uXRCE-DDS out 토픽은 best_effort + volatile.
# (구 TRANSIENT_LOCAL 구독은 volatile 발행과 QoS 불일치로 데이터 미수신됨)
_px4_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
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

    # 우리 로터 순서(r0=FR,r1=FL,r2=RL,r3=RR) → gz motorNumber 매핑.
    # gz motor1↔2가 우리와 뒤바뀌어 있어 1,2 스왑. MOTOR_MAP[gz]=our_r.
    MOTOR_MAP = (0, 2, 1, 3)

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
        # ── 플랜트 특성화용 오픈루프 테스트 ──
        # test_motor >= 0 이면 컨트롤러/안전장치 우회, 우리 로터 순서(r0..r3)로
        # 전부 base 속도 + test_motor 하나만 bump. gz 기울기로 부호 실측.
        self.declare_parameter('test_motor', -1)      # -1=정상제어, 0..3=우리 로터 인덱스
        self.declare_parameter('test_base', 572.0)    # rad/s (호버 근처)
        self.declare_parameter('test_bump', 150.0)    # rad/s (bump 크기)
        # 자세 게인 스케일: 파이썬 sim 게인(Kp=[200,500,500],Kd=[20,50,50])은
        # 무노이즈 가정이라 너무 높음 → SITL EKF omega 노이즈를 증폭해 진동 발산.
        # ×0.3~0.5로 낮추면 안정(오프라인 노이즈 테스트 확인). 튜닝용 파라미터.
        self.declare_parameter('att_gain_scale', 0.35)

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
        # 자세/각속도 게인 스케일 적용 (SITL EKF omega 노이즈 강건성)
        # 컨트롤러마다 게인 표현이 달라 타입별로 적용 (_apply_gain_scale 참고)
        _gs = float(self.get_parameter('att_gain_scale').value)
        self._apply_gain_scale(self.controller, ctrl_type, _gs)
        self.get_logger().info(
            f'제어기: {ctrl_type}, v_ref={self.v_ref}, z_ref={self.z_ref}, '
            f'rate={self.control_rate}Hz'
        )

        # ── 상태 변수 ──
        self._state_valid = False           # 첫 상태 수신 여부
        self._heading_latched = False       # heading을 초기 기수로 래치했는지
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
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self._status_callback, _px4_qos)
        # 각속도는 VehicleOdometry.angular_velocity(body FRD)에서 취득
        # (v1.18 SITL DDS 토픽에 vehicle_angular_velocity 미포함)

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
        # 동체 각속도(FRD) — 별도 VehicleAngularVelocity 대신 odom에서 취득
        self._omega_body = np.array(msg.angular_velocity, dtype=float)

        if not self._state_valid:
            self._state_valid = True
            self.get_logger().info(
                f'첫 상태 수신! pos_ned={self._pos_ned}'
            )

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

        # ── 오픈루프 플랜트 테스트 (컨트롤러/안전장치 우회) ──
        test_motor = self.get_parameter('test_motor').value
        if test_motor is not None and test_motor >= 0:
            base = float(self.get_parameter('test_base').value)
            bump = float(self.get_parameter('test_bump').value)
            # 피드백 로깅: 물리적으로 알려진 회전 중 노드가 보고하는 omega/yaw
            if self._offboard_setpoint_count % 15 == 0:
                _xs = self._assemble_state()
                from scipy.spatial.transform import Rotation as _R
                _Rm = _R.from_quat(_xs[6:10]).as_matrix()
                _yaw = float(np.degrees(np.arctan2(_Rm[1, 0], _Rm[0, 0])))
                _om = _xs[10:13]
                self.get_logger().warn(
                    f'[FB] yaw={_yaw:+.1f} omega=({_om[0]:+.3f},{_om[1]:+.3f},{_om[2]:+.3f})'
                )
            u_test = np.full(4, base)
            tm = int(test_motor)
            # 우리 로터: r0=FR, r1=FL, r2=RL, r3=RR
            if tm == 10:      # 순수 롤(+): 우측 페어(r0,r3) bump → 우측 상승
                u_test[0] += bump; u_test[3] += bump
            elif tm == 11:    # 순수 피치: 전방 페어(r0,r1) bump → 기수 상승
                u_test[0] += bump; u_test[1] += bump
            elif tm == 12:    # 순수 요: 대각 페어(r0,r2)=CW군 bump
                u_test[0] += bump; u_test[2] += bump
            else:
                u_test[tm] += bump
            self._publish_motors(u_test)
            if self._offboard_setpoint_count % 25 == 0:
                self.get_logger().warn(
                    f'[TEST] 우리로터 r{test_motor} bump | u={u_test.astype(int).tolist()} rad/s'
                )
            return

        # 상태 벡터 조립 (NED → NWU 변환)
        x = self._assemble_state()

        # 시간 계산
        if self._t_start is None:
            t = 0.0
        else:
            dt_ros = self.get_clock().now() - self._t_start
            t = dt_ros.nanoseconds * 1e-9

        # ── heading 래치: 첫 유효 상태에서 목표 기수를 현재(스폰) 기수로 정렬 ──
        # (기본 목표 기수 0°가 스폰 기수 -96°와 싸워 요 폭주/커플. 파이썬 sim은 항상
        #  0°에서 시작해 이 경로가 미검증이었음. 컨트롤러별로 현재 기수에 맞춤.)
        if not self._heading_latched:
            if hasattr(self.controller, 'heading'):
                # PID류: 목표 기수 필드만 현재 기수로
                from scipy.spatial.transform import Rotation as _R
                _R0 = _R.from_quat(x[6:10]).as_matrix()
                self.controller.heading = float(np.arctan2(_R0[1, 0], _R0[0, 0]))
                self.get_logger().warn(
                    f'[heading 래치] 목표 기수 = {np.degrees(self.controller.heading):.1f}°'
                )
            elif hasattr(self.controller, 'x_trim'):
                # LQR: yaw=0 트림 기준 선형화라 실제 기수에서 월드좌표 오차 피드백이
                # 잘못된 동체축으로 매핑됨 → 롤 커플/발산. 트림을 현재 기수로
                # 재선형화해 피드백 축을 실제 동체축과 정렬.
                self._relatch_lqr_heading(x)
            self._heading_latched = True

        # ── 진단 계측: 피드백 자세 lean(방향) + omega(각속도) + 요각 ──
        if self._offboard_setpoint_count % 10 == 0:
            from scipy.spatial.transform import Rotation as _R
            _Rm = _R.from_quat(x[6:10]).as_matrix()
            _bz = _Rm[:, 2]                 # 바디 z축(월드 NWU). 수평시 [0,0,-1]
            _lean = (float(_bz[0]), float(_bz[1]))   # 기울기 방향(월드 수평성분)
            _yaw = float(np.degrees(np.arctan2(_Rm[1, 0], _Rm[0, 0])))
            _om = x[10:13]
            self.get_logger().warn(
                f'[DIAG] t={t:.2f} lean=({_lean[0]:+.3f},{_lean[1]:+.3f}) '
                f'yaw={_yaw:+.1f} omega=({_om[0]:+.3f},{_om[1]:+.3f},{_om[2]:+.3f})'
            )

        # 제어기 호출: u = controller(t, x) → 모터 속도 [rad/s]
        try:
            u_raw = self.controller(t, x)
        except Exception as e:
            self.get_logger().error(f'제어기 예외: {e}')
            u_raw = np.full(4, float('nan'))  # 안전장치가 폴백 처리

        # 안전장치 통과 (NaN 감지, 변화율 제한, 자세/고도 체크)
        u = self.safety.check(u_raw, state=x)

        if self.safety.triggered and self._offboard_setpoint_count % 50 == 0:
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
        # (PX4 v1.18: 필드명 actuator → direct_actuator, thrust_and_torque 추가)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = True
        self.pub_offboard_mode.publish(msg)

    def _publish_motors(self, motor_speeds):
        """ActuatorMotors 발행 — 모터 속도를 정규화하여 전송."""
        msg = ActuatorMotors()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        normalized = motor_speed_to_normalized(motor_speeds, self.n_max)

        # 모터 매핑: 우리 dynamics.py 로터 순서(r0=FR,r1=FL,r2=RL,r3=RR)를
        # gz 모델 motorNumber(0=FR,1=RL,2=FL,3=RR)에 맞춤.
        # → gz motor1↔2가 우리 r1↔2와 뒤바뀌어 있어 스왑 필요.
        #   MOTOR_MAP[gz] = our_r : gz모터 gz에 우리 r번 명령을 넣음
        #   (스핀도 x500 위상과 동일해 이 스왑으로 위치·요 모두 정합)
        #   ※ 만약 요(yaw)만 반대로 돌면 gz 모델 turningDirection 4개 반전.
        MOTOR_MAP = self.MOTOR_MAP
        n_ch = len(msg.control)  # PX4 버전별 채널 수 (v1.15=16, v1.18=12)
        for gz in range(4):
            msg.control[gz] = float(normalized[MOTOR_MAP[gz]])
        # 나머지 채널 NaN (미사용)
        for i in range(4, n_ch):
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

    def shutdown_disarm(self):
        """
        노드 종료 시 기체를 안전하게 disarm.

        Ctrl+C로 노드만 죽으면 PX4는 armed 상태를 유지해 모터가 계속 돈다.
        종료 직전 모터 정지 + disarm 명령을 보내고 DDS로 전송되도록 잠깐 spin.
        (force 미사용: 비행 중이면 PX4가 거부 → offboard-loss failsafe에 위임)
        """
        try:
            self._publish_zero_motors()
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0,   # 0 = DISARM
            )
            # 명령이 실제 송신되도록 잠깐 스핀 (약 0.2초)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.02)
            self.get_logger().info('종료 시 disarm 명령 전송 완료')
        except Exception as e:
            self.get_logger().warn(f'종료 disarm 실패: {e}')

    # ════════════════════════════════════════════════
    # 제어기 생성
    # ════════════════════════════════════════════════

    def _relatch_lqr_heading(self, x):
        """
        LQR 트림을 현재(스폰) 기수로 재선형화 — PID heading 래치의 LQR 등가물.

        LQR은 yaw=0 트림 기준 error-state 선형화라, 실제 기수(예: -96°)에서는
        월드좌표 속도/위치 오차(δv, δz)가 잘못된 동체축으로 매핑됨 → 피치 보정이
        롤로 새는 축 커플링 → 롤 발산. 트림 자세를 현재 기수로 회전시켜 다시
        선형화하면 A_r/B_r/K_r이 실제 동체축과 정렬되어 커플링이 사라진다.

        (한 번만: 첫 유효 상태에서 ARE를 재풀이 ~수십 ms, 이후 정상.)
        """
        from scipy.spatial.transform import Rotation as _R
        from .controllers.controller import LQRController
        from .controllers.vehicle_params import vehicle_params as P
        c = self.controller
        R_now = _R.from_quat(x[6:10]).as_matrix()
        yaw_now = float(np.arctan2(R_now[1, 0], R_now[0, 0]))
        R_trim = _R.from_quat(c.x_trim[6:10]).as_matrix()
        yaw_trim = float(np.arctan2(R_trim[1, 0], R_trim[0, 0]))
        Rz = _R.from_euler('z', yaw_now - yaw_trim)   # 트림→현재 기수 회전

        x_trim_new = c.x_trim.copy()
        x_trim_new[6:10] = (Rz * _R.from_quat(c.x_trim[6:10])).as_quat()
        x_trim_new[3:6] = Rz.apply(c.x_trim[3:6])     # 트림 속도도 새 기수로 정렬(v!=0)

        new_c = LQRController(P, x_trim_new, c.u_trim)
        new_c.set_position_ref(np.array([0.0, 0.0, self.z_ref]))
        _gs = float(self.get_parameter('att_gain_scale').value)
        self._apply_gain_scale(new_c, 'lqr', _gs)      # 게인 스케일 재적용
        self.controller = new_c
        self.get_logger().warn(
            f'[LQR heading 래치] 트림 재선형화 @ 기수 {np.degrees(yaw_now):+.1f}° '
            f'(valid={getattr(new_c, "valid", "?")}, max_real={getattr(new_c, "max_real", float("nan")):.3f})'
        )

    def _apply_gain_scale(self, controller, ctrl_type, scale):
        """
        자세/각속도 피드백 게인을 scale배로 축소 (SITL EKF omega 노이즈 강건성).

        파이썬 sim 게인은 무노이즈 가정이라 SITL EKF omega 노이즈를 증폭 →
        진동/전복 발산 (PID 전복과 동일 메커니즘). 컨트롤러마다 게인 표현이
        달라 타입별로 적용:
          - PID류(Kp_att/Kd_att): 자세 PD 게인
          - LQR(K_r):            오차상태 δφ(자세, 4:7)·δω(각속도, 7:10) 열만
                                 (δz/δv/δn = 위치·속도·고도·로터 추종은 유지)
          - INDI(Kp_indi/Kd_indi): 내측 각가속도 PD 게인
          - NMPC:                최적화 기반이라 해당 게인 없음 (미적용)

        기본값 0.35는 PID SITL 호버에서 검증된 값(안전 기본값). ROS 파라미터
        att_gain_scale로 런타임 튜닝 가능(-p att_gain_scale:=0.5 등).
        """
        if scale == 1.0:
            self.get_logger().info('자세 게인 스케일 ×1.0 (미적용)')
            return
        if hasattr(controller, 'Kp_att'):
            controller.Kp_att = np.asarray(controller.Kp_att, float) * scale
            controller.Kd_att = np.asarray(controller.Kd_att, float) * scale
            self.get_logger().info(f'자세 게인(PID Kp_att/Kd_att) ×{scale} 적용')
            if ctrl_type == 'scheduled_pid':
                self.get_logger().warn(
                    'scheduled_pid는 매 호출 게인을 재계산 → 스케일이 유지되지 '
                    '않음. 고속 튜닝 시 controller.py _schedule 수정 필요.'
                )
        elif hasattr(controller, 'K_r'):
            # 오차상태 순서: [δz, δv(3), δφ(3), δω(3), δn(4)]
            #   → 자세 δφ = 열4:7, 각속도 δω = 열7:10
            controller.K_r = controller.K_r.copy()
            controller.K_r[:, 4:10] *= scale
            if hasattr(controller, 'T_pinv'):
                controller.K = controller.K_r @ controller.T_pinv  # 풀상태 K 재계산
            self.get_logger().info(
                f'자세/각속도 게인(LQR K_r δφ·δω 열) ×{scale} 적용')
        elif hasattr(controller, 'Kp_indi'):
            controller.Kp_indi = np.asarray(controller.Kp_indi, float) * scale
            controller.Kd_indi = np.asarray(controller.Kd_indi, float) * scale
            self.get_logger().info(f'각가속도 게인(INDI Kp_indi/Kd_indi) ×{scale} 적용')
        else:
            self.get_logger().info(
                f'게인 스케일 미적용 (해당 게인 속성 없음: {ctrl_type})')

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

        elif ctrl_type == 'hybrid':
            # 확정 제어기: ProperHybrid = VirtualNMPC(가상명령 [T,ω̇]) + INDI(모터).
            # VirtualNMPC 비용엔 yaw/heading 항 없음 → heading 래치 불필요.
            # 자세 게인 없음(비용 내부) → att_gain_scale 미적용(else 분기).
            from .controllers.hybrid_comparison import VirtualNMPC, ProperHybrid
            self.get_logger().info(
                'ProperHybrid 생성 (VirtualNMPC dt_ctrl=0.1s/10Hz, iter=5 + INDI 100Hz)')
            # SITL 실시간: NMPC 솔브 블로킹 완화 (dt_ctrl 0.02→0.1, iter 30→5)
            # INDI는 실제 Δt로 ω̇ 측정(하드코딩 dt 제거)해 가변 루프율에 강건.
            vnmpc = VirtualNMPC(P, v_ref=v_ref, z_ref=z_ref,
                                dt_ctrl=0.10, max_iter=5)
            return ProperHybrid(vnmpc, P, dt=dt)

        else:
            raise ValueError(
                f"알 수 없는 제어기: '{ctrl_type}'. "
                f"가능한 값: pid, scheduled_pid, lqr, scheduled_lqr, indi, nmpc, hybrid"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 요청 (Ctrl+C)')
    finally:
        # 종료 시 기체 disarm (모터 계속 도는 것 방지)
        node.shutdown_disarm()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
