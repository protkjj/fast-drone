#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# PX4 SITL 실행 도우미
# ═══════════════════════════════════════════════════════════════
#
# 사용법:
#   ./scripts/run_sitl.sh px4         # 터미널 1: PX4 SITL + Gazebo
#   ./scripts/run_sitl.sh agent       # 터미널 2: DDS Agent
#   ./scripts/run_sitl.sh offboard    # 터미널 3: Offboard 제어 노드
#   ./scripts/run_sitl.sh topics      # 터미널 4: 토픽 모니터링
#
# ═══════════════════════════════════════════════════════════════

set -euo pipefail

WORK_DIR="${HOME}/px4_drone"
PX4_DIR="${WORK_DIR}/PX4-Autopilot"
ROS2_WS="${WORK_DIR}/ros2_ws"

case "${1:-help}" in
    px4)
        echo ">>> PX4 SITL + Gazebo Harmonic 시작"
        echo "    (첫 실행 시 모델 다운로드로 시간 소요)"
        cd "${PX4_DIR}"
        make px4_sitl gz_x500
        ;;

    agent)
        echo ">>> micro-XRCE-DDS Agent 시작 (UDP 포트 8888)"
        MicroXRCEAgent udp4 -p 8888
        ;;

    offboard)
        echo ">>> Offboard 제어 노드 시작"
        source "${ROS2_WS}/install/setup.bash"

        # 기본값 또는 인자로 제어기/파라미터 지정
        CTRL="${2:-pid}"
        VX="${3:-0.0}"
        ZREF="${4:-10.0}"

        echo "    제어기: ${CTRL}, v_ref_x: ${VX}, z_ref: ${ZREF}"

        ros2 launch fast_drone_ctrl sitl_offboard.launch.py \
            controller_type:="${CTRL}" \
            v_ref_x:="${VX}" \
            z_ref:="${ZREF}"
        ;;

    topics)
        echo ">>> ROS2 토픽 모니터링"
        source "${ROS2_WS}/install/setup.bash"
        echo ""
        echo "사용 가능한 토픽:"
        ros2 topic list
        echo ""
        echo "오도메트리 모니터링:"
        ros2 topic echo /fmu/out/vehicle_odometry --once
        ;;

    help|*)
        echo "사용법: $0 {px4|agent|offboard|topics}"
        echo ""
        echo "  px4       PX4 SITL + Gazebo 시작"
        echo "  agent     micro-XRCE-DDS Agent 시작"
        echo "  offboard  Offboard 제어 노드 시작"
        echo "             옵션: $0 offboard [controller] [v_ref_x] [z_ref]"
        echo "             예시: $0 offboard pid 0.0 10.0"
        echo "             예시: $0 offboard nmpc 30.0 50.0"
        echo "  topics    ROS2 토픽 확인"
        ;;
esac
