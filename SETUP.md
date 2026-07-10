# SETUP — 고속 ISR 드론 시뮬 환경 구축 (팀원용)

> 목표: **아무것도 없는 Ubuntu PC → Python 시뮬 → PX4 SITL + Gazebo** 까지 그대로 재현.
> 이 문서 하나만 위에서 아래로 따라 하면 됩니다. Docker 불필요(네이티브 설치).

---

## 0. 확정 버전 (★ 이 조합만 검증됨 — 섞지 말 것)

`TODO.md` 경고: ROS2 ↔ Ubuntu ↔ Gazebo 버전 짝이 어긋나면 micro-XRCE-DDS부터 말썽납니다.

| 구성요소 | 버전 | 확인 명령 |
|---|---|---|
| Ubuntu | **24.04 LTS** (noble) | `lsb_release -a` |
| Python | **3.12.x** | `python3 --version` |
| ROS2 | **Jazzy** | `ls /opt/ros` |
| Gazebo | **Harmonic** (gz-sim 8.x) | `gz sim --version` |
| PX4-Autopilot | main (SITL) | `git -C ~/px4_drone/PX4-Autopilot describe` |
| micro-XRCE-DDS Agent | 최신 | `MicroXRCEAgent --version` |

전체 소요: 네트워크 포함 대략 **40~60분** (PX4 첫 빌드가 10~20분).
디스크 여유 **최소 30GB**, RAM 8GB+ 권장.

---

## Part A — Python 시뮬 (제일 먼저, 제일 쉬움)

ROS/PX4 없이도 전체 제어 연구 시뮬(`mission_sim.py`)이 여기서 다 돌아갑니다.

```bash
# 1) 저장소 클론
git clone https://github.com/protkjj/fast-drone.git
cd fast-drone

# 2) (권장) 가상환경 — 시스템 python 오염 방지
python3 -m venv .venv
source .venv/bin/activate        # 끌 때: deactivate

# 3) 의존성 설치
pip install -r requirements.txt

# 4) 동작 확인 — 플랜트 테스트
python3 test_plant.py

# 5) 메인 진입점 — 통합 미션 시뮬 (이륙→가속→순항+돌풍→감속→호버, 65초)
python3 mission_sim.py
```

정상이면 콘솔에 제어기별 RMSE 표가 뜨고 `results/`에 플롯 PNG가 생깁니다.
결과 해석은 `results/PROJECT_REPORT.md`, `results/MISSION_ANALYSIS.md` 참고.

> **여기까지가 파이썬 연구 파트 전부.** ROS/Gazebo가 필요 없는 팀원은 Part A까지만 하면 됩니다.

---

## Part B — Ubuntu 기본 (24.04 확인)

```bash
lsb_release -a           # Ubuntu 24.04.x 여야 함
sudo apt update && sudo apt upgrade -y
sudo apt install -y git cmake build-essential python3-pip
```

Ubuntu가 24.04가 아니면 **여기서 멈추세요** — 다른 버전은 아래 조합이 안 맞습니다.

---

## Part C — ROS2 Jazzy 설치

> `scripts/setup_ubuntu.sh`는 ROS2가 **이미 깔려 있다고 가정**합니다. 그래서 ROS2만 여기서 수동 설치.

```bash
# 1) 로케일
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2) 저장소 등록
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3) 설치 (desktop = RViz/데모 포함)
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions

# 4) 자동 소싱 (새 터미널마다 ROS2 사용 가능)
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc

# 5) 확인
ros2 --help
```

---

## Part D — Gazebo Harmonic 설치

```bash
sudo apt install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic

gz sim --version          # Gazebo Sim 8.x = Harmonic
```

> 참고: ROS2 Jazzy를 desktop으로 깔면 ROS 벤더 버전 gz도 딸려옵니다. 위 `gz-harmonic`
> 스탠드얼론 패키지도 같이 있어야 PX4 `make ... gz_x500`이 확실히 붙습니다.

---

## Part E — micro-XRCE-DDS Agent (PX4 ↔ ROS2 브릿지)

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/

MicroXRCEAgent --version   # 확인
```

---

## Part F — PX4-Autopilot (클론 + SITL 빌드) ★ 제일 오래 걸림

```bash
mkdir -p ~/px4_drone && cd ~/px4_drone
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 시스템 의존성 (sudo, 패키지 다수 설치)
bash ./Tools/setup/ubuntu.sh --no-sim-tools

# SITL 빌드 (첫 빌드 10~20분)
make px4_sitl
```

---

## Part G — ROS2 워크스페이스 빌드

> 여기부터는 저장소의 `scripts/setup_ubuntu.sh`가 자동으로 해줍니다(Part F 포함).
> 수동으로 이해하려면 아래, 자동으로 하려면 → **한 방 설치** 절 참고.

```bash
mkdir -p ~/px4_drone/ros2_ws/src && cd ~/px4_drone/ros2_ws/src

# PX4 메시지 타입 + 예제
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

# 우리 제어 패키지 복사 (fast-drone 저장소 경로에 맞춰 수정)
cp -r ~/fast-drone/ros2_ws/src/fast_drone_ctrl .

# 제어기 소스도 controllers/ 로 복사
mkdir -p fast_drone_ctrl/fast_drone_ctrl/controllers
for f in dynamics.py controller.py nmpc.py vehicle_params.py trim.py; do
    cp ~/fast-drone/$f fast_drone_ctrl/fast_drone_ctrl/controllers/
done
touch fast_drone_ctrl/fast_drone_ctrl/controllers/__init__.py

# 빌드
cd ~/px4_drone/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
echo 'source ~/px4_drone/ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

## ⚡ 한 방 설치 (Part E~G 자동)

ROS2 Jazzy(Part C)까지 끝냈다면, 나머지는 저장소 스크립트가 처리합니다:

```bash
cd ~/fast-drone
chmod +x scripts/setup_ubuntu.sh
./scripts/setup_ubuntu.sh
```

스크립트가 하는 일: PX4 클론+빌드 → Gazebo 확인 → DDS Agent → px4_msgs/px4_ros_com →
우리 패키지 복사 → colcon build. (이미 설치된 건 건너뜁니다.)

---

## Part H — PX4 SITL 실행 (터미널 4개)

`scripts/run_sitl.sh`로 감싸놨습니다:

```bash
# 터미널 1 — PX4 SITL + Gazebo (x500 기본 쿼드로 먼저 통신 확인!)
cd ~/px4_drone/PX4-Autopilot && make px4_sitl gz_x500
#   또는:  ~/fast-drone/scripts/run_sitl.sh px4

# 터미널 2 — DDS Agent
MicroXRCEAgent udp4 -p 8888
#   또는:  ~/fast-drone/scripts/run_sitl.sh agent

# 터미널 3 — 토픽 확인 (ROS2가 PX4 데이터 받는지)
source ~/px4_drone/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_odometry --once
#   또는:  ~/fast-drone/scripts/run_sitl.sh topics

# 터미널 4 — Offboard 제어 노드
~/fast-drone/scripts/run_sitl.sh offboard pid 0.0 10.0
```

`/fmu/out/...` 토픽이 보이고 값이 흐르면 **SITL 파이프라인 성공**입니다.

---

## 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| `ros2 topic list`에 `/fmu/...` 없음 | DDS Agent(터미널 2) 안 떴거나 포트 불일치. Agent가 `8888`인지 확인. 그래도 안 되면 **버전 짝(0번 표)부터 의심**. |
| `make px4_sitl gz_x500`에서 gz 못 찾음 | `gz sim --version` 확인. Part D의 `gz-harmonic` 스탠드얼론 재설치. |
| PX4 빌드 중 의존성 에러 | `bash ./Tools/setup/ubuntu.sh --no-sim-tools` 다시. 이후 `make clean && make px4_sitl`. |
| `colcon build` px4_msgs 에러 | ROS2 소싱 먼저: `source /opt/ros/jazzy/setup.bash`. `ros-dev-tools` 설치 확인. |
| Gazebo GUI 안 뜸 (원격/헤드리스) | `HEADLESS=1 make px4_sitl gz_x500` 로 GUI 없이 실행. |
| `casadi` import 에러 | `pip install -r requirements.txt` (가상환경 활성화 상태에서). |

---

## acados (선택 — 실시간 NMPC, 급하지 않음)

`nmpc.py`가 실시간(50Hz, 21ms) 구동 시 acados를 씁니다. pip 설치 불가, 소스 빌드 필요.
파이썬 시뮬/SITL 통신 검증에는 **불필요** — 실기체 실시간 단계에서만 설치.
→ https://docs.acados.org/installation

---

## 다음 단계 (환경 구축 후)

1. **Phase 1**: 기본 x500 쿼드로 SITL 통신부터 확인 (커스텀 기체 전에!)
2. **Phase 2**: Gazebo 커스텀 공력 플러그인(C++) — 고속 동체 공력 반영
3. **Phase 3**: 실기체 (RTK GPS + 컴패니언 컴퓨터 + acados RTI)

자세한 로드맵: `HANDOFF.md`, `TODO.md`
