"""
PX4 SITL Offboard 제어 Launch 파일.

사용법:
    # 기본 (PID, 호버)
    ros2 launch fast_drone_ctrl sitl_offboard.launch.py

    # NMPC, 30 m/s 순항
    ros2 launch fast_drone_ctrl sitl_offboard.launch.py \
        controller_type:=nmpc v_ref_x:=30.0 z_ref:=50.0

    # LQR, 호버
    ros2 launch fast_drone_ctrl sitl_offboard.launch.py \
        controller_type:=lqr z_ref:=10.0

참고:
    이 launch 파일은 offboard 노드만 실행합니다.
    PX4 SITL + Gazebo + micro-XRCE-DDS agent는
    별도 터미널에서 먼저 실행해야 합니다.
    (scripts/setup_ubuntu.sh 참고)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # ── Launch 인자 ──
        DeclareLaunchArgument(
            'controller_type', default_value='pid',
            description='제어기 종류: pid, scheduled_pid, lqr, scheduled_lqr, indi, nmpc',
        ),
        DeclareLaunchArgument(
            'v_ref_x', default_value='0.0',
            description='목표 전진 속도 [m/s]',
        ),
        DeclareLaunchArgument(
            'v_ref_y', default_value='0.0',
            description='목표 횡방향 속도 [m/s]',
        ),
        DeclareLaunchArgument(
            'v_ref_z', default_value='0.0',
            description='목표 수직 속도 [m/s]',
        ),
        DeclareLaunchArgument(
            'z_ref', default_value='10.0',
            description='목표 고도 [m] (NWU z-up)',
        ),
        DeclareLaunchArgument(
            'control_rate_hz', default_value='100.0',
            description='제어 루프 주파수 [Hz]',
        ),

        # ── 노드 ──
        Node(
            package='fast_drone_ctrl',
            executable='offboard_node',
            name='offboard_controller',
            output='screen',
            parameters=[{
                # 노드가 double로 선언한 파라미터는 ParameterValue(value_type=float)로
                # 감싸야 launch의 문자열 substitution이 타입 불일치 예외 없이 전달됨.
                'controller_type': LaunchConfiguration('controller_type'),
                'v_ref_x': ParameterValue(LaunchConfiguration('v_ref_x'), value_type=float),
                'v_ref_y': ParameterValue(LaunchConfiguration('v_ref_y'), value_type=float),
                'v_ref_z': ParameterValue(LaunchConfiguration('v_ref_z'), value_type=float),
                'z_ref': ParameterValue(LaunchConfiguration('z_ref'), value_type=float),
                'control_rate_hz': ParameterValue(
                    LaunchConfiguration('control_rate_hz'), value_type=float),
                'n_max': 1800.0,
                'arm_delay_sec': 2.0,
            }],
        ),
    ])
