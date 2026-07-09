#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# PX4 SITL + Gazebo Harmonic + ROS2 Jazzy 환경 세팅
# ═══════════════════════════════════════════════════════════════
#
# 사용법:
#   chmod +x scripts/setup_ubuntu.sh
#   ./scripts/setup_ubuntu.sh
#
# 이 스크립트가 하는 것:
#   1. PX4-Autopilot 클론 + 의존성 + 빌드
#   2. Gazebo Harmonic 설치 (PX4와 호환)
#   3. micro-XRCE-DDS Agent 빌드 (PX4 ↔ ROS2 브릿지)
#   4. px4_msgs 클론 (ROS2 메시지 타입)
#   5. Python 의존성 (casadi, scipy 등)
#   6. 우리 ROS2 패키지 빌드
#
# 전제:
#   - Ubuntu 24.04 (Jazzy) 또는 22.04 (Humble → Jazzy 별도 설치)
#   - ROS2 Jazzy 이미 설치됨
#   - sudo 권한 있음
#
# 참고:
#   PX4 공식 문서: https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
#   Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu
# ═══════════════════════════════════════════════════════════════

set -euo pipefail

# 색상 출력
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ── 경로 설정 ──
WORK_DIR="${HOME}/px4_drone"
PX4_DIR="${WORK_DIR}/PX4-Autopilot"
AGENT_DIR="${WORK_DIR}/micro-xrce-dds-agent"
ROS2_WS="${WORK_DIR}/ros2_ws"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

info "작업 디렉토리: ${WORK_DIR}"
info "프로젝트 소스: ${PROJECT_DIR}"
mkdir -p "${WORK_DIR}"


# ═══════════════════════════════════════════════════
# 1. PX4-Autopilot
# ═══════════════════════════════════════════════════
info "=== Step 1/6: PX4-Autopilot ==="

if [ -d "${PX4_DIR}" ]; then
    warn "PX4-Autopilot 이미 존재: ${PX4_DIR}"
    warn "업데이트하려면 수동으로 git pull 하세요."
else
    info "PX4-Autopilot 클론 중..."
    git clone https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}" --recursive

    info "PX4 의존성 설치 중... (sudo 필요)"
    cd "${PX4_DIR}"
    # PX4 공식 설치 스크립트: 시스템 의존성 + 크로스컴파일 도구
    bash ./Tools/setup/ubuntu.sh --no-sim-tools

    info "PX4 SITL 빌드 중... (첫 빌드는 10~20분 소요)"
    cd "${PX4_DIR}"
    make px4_sitl
fi


# ═══════════════════════════════════════════════════
# 2. Gazebo Harmonic
# ═══════════════════════════════════════════════════
info "=== Step 2/6: Gazebo Harmonic ==="

if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>/dev/null || echo "unknown")
    warn "Gazebo 이미 설치됨: ${GZ_VERSION}"
else
    info "Gazebo Harmonic 설치 중..."

    # Gazebo 저장소 키 + 소스 추가
    sudo apt-get update
    sudo apt-get install -y lsb-release wget gnupg

    sudo wget https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    sudo apt-get update
    sudo apt-get install -y gz-harmonic

    info "Gazebo Harmonic 설치 완료"
fi


# ═══════════════════════════════════════════════════
# 3. micro-XRCE-DDS Agent
# ═══════════════════════════════════════════════════
info "=== Step 3/6: micro-XRCE-DDS Agent ==="

if [ -d "${AGENT_DIR}" ]; then
    warn "micro-XRCE-DDS Agent 이미 존재: ${AGENT_DIR}"
else
    info "micro-XRCE-DDS Agent 빌드 중..."
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${AGENT_DIR}"
    cd "${AGENT_DIR}"
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig /usr/local/lib/

    info "micro-XRCE-DDS Agent 설치 완료"
fi


# ═══════════════════════════════════════════════════
# 4. ROS2 워크스페이스 + px4_msgs
# ═══════════════════════════════════════════════════
info "=== Step 4/6: ROS2 워크스페이스 ==="

mkdir -p "${ROS2_WS}/src"

# px4_msgs 클론
if [ -d "${ROS2_WS}/src/px4_msgs" ]; then
    warn "px4_msgs 이미 존재"
else
    info "px4_msgs 클론 중..."
    git clone https://github.com/PX4/px4_msgs.git "${ROS2_WS}/src/px4_msgs"
fi

# px4_ros_com (예제 + 유틸)
if [ -d "${ROS2_WS}/src/px4_ros_com" ]; then
    warn "px4_ros_com 이미 존재"
else
    info "px4_ros_com 클론 중..."
    git clone https://github.com/PX4/px4_ros_com.git "${ROS2_WS}/src/px4_ros_com"
fi


# ═══════════════════════════════════════════════════
# 5. 우리 패키지 복사 + Python 의존성
# ═══════════════════════════════════════════════════
info "=== Step 5/6: fast_drone_ctrl 패키지 ==="

# 패키지 복사 (또는 심볼릭 링크)
CTRL_PKG="${ROS2_WS}/src/fast_drone_ctrl"
if [ -d "${CTRL_PKG}" ]; then
    warn "fast_drone_ctrl 이미 존재. 업데이트 중..."
    rm -rf "${CTRL_PKG}"
fi

cp -r "${PROJECT_DIR}/ros2_ws/src/fast_drone_ctrl" "${CTRL_PKG}"

# 기존 제어기 파일들을 controllers/ 디렉토리에 복사
CTRL_SRC="${CTRL_PKG}/fast_drone_ctrl/controllers"
mkdir -p "${CTRL_SRC}"

# 필요한 파일만 복사
for f in dynamics.py controller.py nmpc.py vehicle_params.py trim.py; do
    if [ -f "${PROJECT_DIR}/${f}" ]; then
        cp "${PROJECT_DIR}/${f}" "${CTRL_SRC}/"
        info "  복사: ${f}"
    else
        warn "  파일 없음: ${f}"
    fi
done

# __init__.py 확인
touch "${CTRL_SRC}/__init__.py"

# Python 의존성
info "Python 의존성 설치 중..."
pip3 install --user numpy scipy casadi


# ═══════════════════════════════════════════════════
# 6. ROS2 워크스페이스 빌드
# ═══════════════════════════════════════════════════
info "=== Step 6/6: colcon build ==="

cd "${ROS2_WS}"

# ROS2 환경 소싱
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    warn "Jazzy 없음, Humble 사용"
    source /opt/ros/humble/setup.bash
else
    error "ROS2 설치를 찾을 수 없습니다!"
fi

colcon build --symlink-install

info "ROS2 워크스페이스 빌드 완료!"


# ═══════════════════════════════════════════════════
# 완료 + 사용법 안내
# ═══════════════════════════════════════════════════

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  설치 완료!"
echo "═══════════════════════════════════════════════════════"
echo ""
echo "  실행 방법 (터미널 4개 필요):"
echo ""
echo "  [터미널 1] PX4 SITL + Gazebo:"
echo "    cd ${PX4_DIR}"
echo "    make px4_sitl gz_x500"
echo ""
echo "  [터미널 2] micro-XRCE-DDS Agent:"
echo "    MicroXRCEAgent udp4 -p 8888"
echo ""
echo "  [터미널 3] ROS2 토픽 확인:"
echo "    source ${ROS2_WS}/install/setup.bash"
echo "    ros2 topic list"
echo ""
echo "  [터미널 4] Offboard 제어:"
echo "    source ${ROS2_WS}/install/setup.bash"
echo "    ros2 launch fast_drone_ctrl sitl_offboard.launch.py"
echo ""
echo "  bashrc에 추가 (선택):"
echo "    echo 'source ${ROS2_WS}/install/setup.bash' >> ~/.bashrc"
echo "    echo 'export GZ_SIM_RESOURCE_PATH=${PX4_DIR}/Tools/simulation/gz/models' >> ~/.bashrc"
echo ""
echo "═══════════════════════════════════════════════════════"
