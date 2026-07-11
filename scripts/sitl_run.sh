#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════
# sitl_run.sh — 고속 드론 SITL "원커맨드" 런처
# ═══════════════════════════════════════════════════════════════════════
# 한 명령으로:  XRCE-DDS agent(8888) → PX4 SITL(gz_fast_missile) → offboard_node
# 를 순서대로(각 단계 실제 준비 완료를 폴링) 기동하고, 로그를 타임스탬프 폴더에
# 모으고, 종료 시 자식 프로세스를 안전한 순서로 정리한다. 재실행 안전(idempotent).
#
# 사용법:
#   scripts/sitl_run.sh                          # 기본: pid, z=5, 30초, GUI
#   scripts/sitl_run.sh -c lqr -z 10 -d 20       # lqr, z=10m, 20초
#   scripts/sitl_run.sh -c hybrid --headless     # 헤드리스(GUI 없이)
#   CONTROLLER=pid Z_REF=5 DURATION=30 scripts/sitl_run.sh   # env로도 가능
#
# 플래그(우선) / env(기본):
#   -c CONTROLLER   (pid)     제어기: pid|lqr|indi|nmpc|hybrid|scheduled_pid|scheduled_lqr
#   -z Z_REF        (5.0)     목표 고도 [m]
#   -x V_REF_X      (0.0)     목표 전진속도 [m/s]
#   -g ATT_GAIN_SCALE (0.35)  자세게인 스케일
#   -d DURATION     (30)      실행 시간 [s]
#      --headless             gz GUI 없이 (HEADLESS=1)
#      --build                실행 전 Desktop 소스→px4_drone 배포 + colcon build
#      --no-tmux              tmux 뷰 비활성(백그라운드+로그만)
#
# 로그:  results/sitl_runs/<timestamp>/{agent,px4,gz,node}.log  (node엔 [DIAG]/[안전] 포함)
# 종료:  Ctrl+C 또는 시간초과 → node(→disarm) → px4 → gz → agent 순 정리
# ═══════════════════════════════════════════════════════════════════════
# nounset(-u)은 쓰지 않음: ROS setup.bash가 미설정 변수를 다수 참조해 set -u면 소싱이 깨짐
set -o pipefail

# ── 경로 (검증된 환경) ──
REPO="/home/kj/Desktop/fast_drone"
PX4_DIR="/home/kj/px4_drone/PX4-Autopilot"
ROS_WS="/home/kj/px4_drone/ros2_ws"                 # 빌드된 워크스페이스(px4_msgs 포함)
DEV_PKG="${REPO}/ros2_ws/src/fast_drone_ctrl"       # 개발 소스
ROS_DISTRO_SETUP="/opt/ros/jazzy/setup.bash"
PORT=8888
GZ_WORLD="default"

# ── 파라미터 기본값 (env 우선, 플래그가 덮어씀) ──
CONTROLLER="${CONTROLLER:-pid}"
Z_REF="${Z_REF:-5.0}"
V_REF_X="${V_REF_X:-0.0}"
ATT_GAIN_SCALE="${ATT_GAIN_SCALE:-0.35}"
DURATION="${DURATION:-30}"
HEADLESS="${HEADLESS:-0}"
DO_BUILD=0
USE_TMUX=1

# ── 플래그 파싱 ──
while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--controller) CONTROLLER="$2"; shift 2;;
    -z|--z-ref)      Z_REF="$2"; shift 2;;
    -x|--v-ref-x)    V_REF_X="$2"; shift 2;;
    -g|--att-gain)   ATT_GAIN_SCALE="$2"; shift 2;;
    -d|--duration)   DURATION="$2"; shift 2;;
    --headless)      HEADLESS=1; shift;;
    --build)         DO_BUILD=1; shift;;
    --no-tmux)       USE_TMUX=0; shift;;
    -h|--help)       sed -n '2,40p' "$0"; exit 0;;
    *) echo "알 수 없는 인자: $1"; exit 2;;
  esac
done

command -v MicroXRCEAgent >/dev/null || { echo "❌ MicroXRCEAgent 없음"; exit 1; }
AGENT_BIN="$(command -v MicroXRCEAgent)"
have_tmux=0; command -v tmux >/dev/null && have_tmux=1
SESSION="sitl_view"

# ── 로그 디렉토리 ──
TS="$(date +%Y%m%d_%H%M%S)"
LOGDIR="${REPO}/results/sitl_runs/${TS}"
mkdir -p "${LOGDIR}"
AGENT_LOG="${LOGDIR}/agent.log"; PX4_LOG="${LOGDIR}/px4.log"; PX4_RAW="${LOGDIR}/px4_raw.log"
GZ_LOG="${LOGDIR}/gz.log";       NODE_LOG="${LOGDIR}/node.log"
: > "$AGENT_LOG"; : > "$PX4_LOG"; : > "$GZ_LOG"; : > "$NODE_LOG"

# PX4 raw 로그를 ANSI/pxh 스팸 제거해 px4.log로 후처리 후 raw 삭제.
# raw는 pxh 콘솔 스팸으로 수백MB까지 커질 수 있고 개행이 거의 없어 전체 sed 처리 시 느림/메모리↑ →
# 유용정보(기동·토픽생성·에러)가 있는 앞 8MB만 필터링 저장(빠름·안전), 나머지 스팸구간은 버림.
filter_px4_log() {
  [[ -s "$PX4_RAW" ]] || return 0
  local sz; sz=$(du -h "$PX4_RAW" 2>/dev/null | cut -f1)
  head -c 8000000 "$PX4_RAW" 2>/dev/null \
    | sed 's/\x1b\[[0-9;]*[A-Za-z]//g; s/pxh> //g' 2>/dev/null \
    | grep -v '^[[:space:]]*$' > "$PX4_LOG" 2>/dev/null
  printf '\n[sitl_run] (px4 raw %s → 앞 8MB만 필터 저장, 이후 pxh 스팸 구간 생략)\n' "${sz:-?}" >> "$PX4_LOG"
  rm -f "$PX4_RAW"
}

log() { echo -e "\033[1;36m[sitl_run]\033[0m $*"; }
warn() { echo -e "\033[1;33m[sitl_run]\033[0m $*"; }

# ── 정리(공용): node(→disarm) → px4 → gz → agent 순 ──
kill_pattern() { pkill -"${2:-TERM}" -f "$1" 2>/dev/null; }
teardown() {
  trap - INT TERM EXIT
  echo; log "정리 중 (node→px4→gz→agent 순)…"
  # 1) 노드 먼저 SIGINT → shutdown_disarm 유도
  kill_pattern 'lib/fast_drone_ctrl/offboard_node' INT
  kill_pattern 'offboard_node --ros-args' INT
  sleep 2                       # disarm 명령 DDS 송신 대기(고정 sleep은 이 정리 단계만)
  # 2) PX4 (make/cmake/ninja/px4 바이너리 트리 전부)
  kill_pattern 'make px4_sitl' TERM; kill_pattern 'gz_fast_missile' TERM
  kill_pattern 'px4_sitl_default' TERM; kill_pattern 'bin/px4' INT
  # 3) Gazebo (server -s + gui -g)
  kill_pattern 'gz sim' TERM
  # 4) agent
  kill_pattern 'MicroXRCEAgent' TERM
  [[ $have_tmux -eq 1 ]] && tmux kill-session -t "$SESSION" 2>/dev/null
  sleep 2
  # 잔여 강제 종료
  kill_pattern 'px4_sitl_default' KILL; kill_pattern 'gz sim' KILL
  kill_pattern 'gz_fast_missile' KILL; kill_pattern 'MicroXRCEAgent' KILL
  filter_px4_log                # raw→px4.log(ANSI/pxh 제거) 후 raw 삭제
  log "정리 완료. 로그: ${LOGDIR}"
}
trap teardown INT TERM EXIT

# ── 폴링 헬퍼: 조건이 참이 될 때까지 (고정 sleep 아님, 실제 상태 확인) ──
wait_until() {  # $1=설명 $2=타임아웃s $3...=판정명령(0이면 준비됨)
  local desc="$1" to="$2"; shift 2
  local start=$SECONDS last=0 el
  until "$@" >/dev/null 2>&1; do
    el=$(( SECONDS - start ))
    if (( el >= to )); then warn "⏱  '${desc}' 준비 타임아웃(${to}s)"; return 1; fi
    if (( el - last >= 10 )); then log "…'${desc}' 대기 중 (${el}s)"; last=$el; fi
    sleep 0.5
  done
  return 0
}
agent_up() { ss -uln 2>/dev/null | grep -q ":${PORT} "; }
px4_topics_up() {
  source "$ROS_DISTRO_SETUP" 2>/dev/null; source "${ROS_WS}/install/setup.bash" 2>/dev/null
  timeout 4 ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'
}

# ── 0) stale 정리 (idempotent 시작) ── 이전 gz가 덜 빠지면 새 gz가 'Waiting for world'로 멈춤
log "이전 stale 프로세스/포트 정리…"
kill_pattern 'lib/fast_drone_ctrl/offboard_node' INT
kill_pattern 'make px4_sitl' TERM; kill_pattern 'gz_fast_missile' TERM
kill_pattern 'px4_sitl_default' TERM; kill_pattern 'bin/px4' TERM
kill_pattern 'gz sim' TERM; kill_pattern 'MicroXRCEAgent' TERM
sleep 2
kill_pattern 'px4_sitl_default' KILL; kill_pattern 'gz sim' KILL
kill_pattern 'gz_fast_missile' KILL; kill_pattern 'MicroXRCEAgent' KILL
wait_until "이전 gz/px4 완전 종료" 15 bash -c "! pgrep -f '[g]z sim|[p]x4_sitl_default' >/dev/null" \
  || warn "이전 gz/px4 잔존(계속 진행)"
wait_until "포트 ${PORT} 해제" 10 bash -c "! ss -uln 2>/dev/null | grep -q ':${PORT} '" \
  || warn "포트 ${PORT} 여전히 점유(계속 진행)"

# ── (옵션) 빌드 ──
if (( DO_BUILD )); then
  log "빌드: Desktop 소스 → px4_drone 배포 + colcon"
  rsync -a --delete "${DEV_PKG}/" "${ROS_WS}/src/fast_drone_ctrl/" 2>/dev/null \
    || cp -rf "${DEV_PKG}/." "${ROS_WS}/src/fast_drone_ctrl/"
  ( source "$ROS_DISTRO_SETUP"; cd "$ROS_WS" && colcon build --packages-select fast_drone_ctrl ) \
    >"${LOGDIR}/build.log" 2>&1 || { warn "colcon build 실패 → ${LOGDIR}/build.log"; exit 1; }
  log "빌드 완료"
fi

log "설정: controller=${CONTROLLER} z_ref=${Z_REF} v_ref_x=${V_REF_X} att_gain_scale=${ATT_GAIN_SCALE} dur=${DURATION}s headless=${HEADLESS}"

# ── (a) XRCE-DDS agent ──
log "(a) XRCE-DDS agent 기동 (UDP ${PORT})"
setsid bash -c "exec '${AGENT_BIN}' udp4 -p ${PORT}" >"$AGENT_LOG" 2>&1 &
wait_until "agent 포트 ${PORT}" 15 agent_up || { warn "agent 기동 실패"; exit 1; }
log "    ✓ agent 준비"

# ── (b) PX4 SITL (gz_fast_missile) ──
log "(b) PX4 SITL + Gazebo(gz_fast_missile) 기동"
# ⚠️ PX4 콘솔(pxh>)이 비TTY에서 ANSI [2K로 무한 재출력(~2MB/s). 이걸 라이브 파이프(sed)로
#    거르면 backpressure로 PX4 stdout이 막혀 gz가 'Waiting for world'로 정지함(실측).
#    → raw 파일로 직접 리다이렉트(파이프 없음=backpressure 없음). 종료 시 sed로 후처리 후 raw 삭제.
setsid bash -c "cd '${PX4_DIR}' && HEADLESS=${HEADLESS} exec make px4_sitl gz_fast_missile" \
  </dev/null >"$PX4_RAW" 2>&1 &
ln -sf "$PX4_LOG" "$GZ_LOG" 2>/dev/null   # gz 로그는 px4.log에 섞임
log "    PX4↔agent↔DDS 연결 대기(첫 기동은 오래 걸릴 수 있음)…"
wait_until "PX4 DDS 토픽(/fmu/out/vehicle_odometry)" 180 px4_topics_up \
  || { warn "PX4 토픽 미수신 → ${PX4_LOG} 확인"; exit 1; }
log "    ✓ PX4 준비 (DDS 토픽 수신)"
# headless면 gz GUI(-g)만 종료(서버 -s 유지) — PX4는 HEADLESS env를 안 보므로 여기서 처리
if (( HEADLESS )); then kill_pattern 'gz sim -g' TERM; log "    (headless) gz GUI 종료"; fi

# ── (c) offboard_node ──
log "(c) offboard_node 실행 (${DURATION}s, 자동 offboard+arm)"
NODE_CMD="source '${ROS_DISTRO_SETUP}'; source '${ROS_WS}/install/setup.bash'; \
exec timeout --signal=INT --kill-after=5 ${DURATION} \
ros2 run fast_drone_ctrl offboard_node --ros-args \
 -p controller_type:=${CONTROLLER} -p z_ref:=${Z_REF} \
 -p v_ref_x:=${V_REF_X} -p att_gain_scale:=${ATT_GAIN_SCALE}"
setsid bash -c "${NODE_CMD}" >"$NODE_LOG" 2>&1 &
NODE_BG=$!

# ── tmux 3-pane 라이브 뷰 (있고 TTY이고 --no-tmux 아니면) ──
if (( have_tmux && USE_TMUX )) && [[ -t 1 ]]; then
  tmux kill-session -t "$SESSION" 2>/dev/null
  tmux new-session -d -s "$SESSION" -x 210 -y 50 "tail -f '${NODE_LOG}'"
  tmux split-window -v -t "$SESSION" "tail -f '${PX4_LOG}'"
  tmux split-window -h -t "$SESSION" "tail -f '${AGENT_LOG}'"
  tmux select-layout -t "$SESSION" tiled >/dev/null
  ( sleep "$DURATION"; tmux kill-session -t "$SESSION" 2>/dev/null ) &
  log "tmux 뷰 attach (detach=Ctrl-b d, 종료=Ctrl-C)"
  tmux attach -t "$SESSION"
else
  log "백그라운드 실행. node 로그 실시간:"
  tail -f "$NODE_LOG" &
  TAIL_PID=$!
  wait "$NODE_BG" 2>/dev/null
  kill "$TAIL_PID" 2>/dev/null
fi

log "실행 종료 → 정리 시작"
# teardown 은 EXIT trap 이 수행
