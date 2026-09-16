#!/usr/bin/env bash
# ROS 2 <-> PX4/Gazebo unified launcher for SimForDrone.
#
# Typical flow:
#   ./start_ros2_px4.sh build
#   ./start_ros2_px4.sh start          # headless (default)
#   ./start_ros2_px4.sh test
#   ./start_ros2_px4.sh attach px4     # interactive PX4 shell
#   ./start_ros2_px4.sh stop
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${PROJECT_DIR}/PX4-Autopilot}"
ROS2_WS="${ROS2_WS:-${PROJECT_DIR}/../IsaacDrone/ros2_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
ROS_WS_SETUP="${ROS2_WS}/install/setup.bash"
LOG_DIR="${PROJECT_DIR}/logs/ros2_px4"
TMUX_SOCKET="simfordrone"
AGENT_SESSION="xrce_agent"
PX4_SESSION="px4_gazebo"

usage() {
  cat <<'EOF'
Usage: ./start_ros2_px4.sh <command> [options]

Commands:
  build             Build px4_msgs and Micro XRCE-DDS Agent
  start [--gui]     Start Agent first, then PX4+Gazebo (headless by default)
  status            Show tmux sessions, ports, processes, and ROS topic count
  test              Verify live PX4 status, odometry, and PX4 input subscribers
  attach agent      Attach to the Micro XRCE-DDS Agent terminal
  attach px4        Attach to the interactive PX4 (pxh>) terminal
  logs [agent|px4]  Tail saved logs
  stop              Gracefully stop PX4/Gazebo and the Agent

Environment overrides:
  PX4_DIR=/path/to/PX4-Autopilot
  ROS2_WS=/path/to/ros2_ws
  ROS_DISTRO=jazzy

Startup/test flow:
  1. build  -> message definitions and Agent exist
  2. start  -> UDP 8888 Agent starts before PX4 uXRCE-DDS client
  3. status -> processes, UDP port, and /fmu topics are visible
  4. test   -> ROS 2 decodes PX4 status and odometry; input readers exist
  5. attach px4 -> optional manual PX4 shell work
  6. stop   -> stop the complete stack
EOF
}

require_file() {
  if [[ ! -e "$1" ]]; then
    echo "ERROR: missing $1" >&2
    exit 1
  fi
}

source_ros() {
  require_file "${ROS_SETUP}"
  # ROS Jazzy setup reads optional AMENT_* variables directly.
  # Temporarily relax nounset while loading ROS environments.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
  require_file "${ROS_WS_SETUP}"
  set +u
  # shellcheck disable=SC1090
  source "${ROS_WS_SETUP}"
  set -u
}

tmux_cmd() {
  tmux -L "${TMUX_SOCKET}" "$@"
}

session_exists() {
  tmux_cmd has-session -t "$1" 2>/dev/null
}

cmd_build() {
  require_file "${ROS_SETUP}"
  require_file "${ROS2_WS}/src/px4_msgs/package.xml"
  require_file "${ROS2_WS}/src/Micro-XRCE-DDS-Agent/CMakeLists.txt"

  mkdir -p "${LOG_DIR}"
  echo "Building ROS 2 workspace: ${ROS2_WS}"
  (
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER PYTHONPATH
    unset CMAKE_PREFIX_PATH LD_LIBRARY_PATH
    export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    # ROS Jazzy setup is not nounset-safe.
    set +u
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
    set -u
    cd "${ROS2_WS}"
    colcon build --event-handlers console_cohesion+
  ) 2>&1 | tee "${LOG_DIR}/build.log"
}

cmd_start() {
  local gui=0
  if [[ "${1:-}" == "--gui" ]]; then
    gui=1
  elif [[ -n "${1:-}" ]]; then
    echo "ERROR: unknown start option: $1" >&2
    usage
    exit 2
  fi

  require_file "${ROS_WS_SETUP}"
  require_file "${ROS2_WS}/install/microxrcedds_agent/bin/MicroXRCEAgent"
  require_file "${PX4_DIR}/Makefile"
  require_file "${PX4_DIR}/.venv/bin/python"
  mkdir -p "${LOG_DIR}"

  if session_exists "${AGENT_SESSION}" || session_exists "${PX4_SESSION}"; then
    echo "ERROR: SimForDrone sessions already exist. Run '$0 status' or '$0 stop'." >&2
    exit 1
  fi

  if ss -lunH | awk '{print $4}' | grep -qE '(^|:)8888$'; then
    echo "ERROR: UDP 8888 is already occupied. Stop the old XRCE Agent first." >&2
    exit 1
  fi

  echo "[1/2] Starting Micro XRCE-DDS Agent on UDP 8888"
  tmux_cmd new-session -d -s "${AGENT_SESSION}" -c "${ROS2_WS}" \
    "bash --noprofile --norc -c 'source ${ROS_SETUP} && source ${ROS_WS_SETUP} && exec MicroXRCEAgent udp4 -p 8888 -v 4 2>&1 | tee ${LOG_DIR}/agent.log'"

  for _ in {1..20}; do
    if ss -lunH | awk '{print $4}' | grep -qE '(^|:)8888$'; then
      break
    fi
    sleep 0.25
  done
  if ! ss -lunH | awk '{print $4}' | grep -qE '(^|:)8888$'; then
    echo "ERROR: Agent did not open UDP 8888. See: $0 logs agent" >&2
    tmux_cmd kill-session -t "${AGENT_SESSION}" 2>/dev/null || true
    exit 1
  fi

  echo "[2/2] Starting PX4 SITL + Gazebo"
  local headless="HEADLESS=1"
  local graphics_env=""
  if (( gui )); then
    headless=""
    graphics_env="DISPLAY=${DISPLAY:-:0} __GLX_VENDOR_LIBRARY_NAME=nvidia __NV_PRIME_RENDER_OFFLOAD=1"
  fi

  tmux_cmd new-session -d -s "${PX4_SESSION}" -c "${PX4_DIR}" \
    "bash --noprofile --norc -c 'unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER PYTHONPATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH; export PATH=${PX4_DIR}/.venv/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin; ${graphics_env} ${headless} make px4_sitl gz_x500 2>&1 | tee ${LOG_DIR}/px4.log'"

  echo "Started. Use '$0 status', then '$0 test'."
  echo "Interactive PX4 shell: $0 attach px4"
}

cmd_status() {
  echo "== tmux sessions (${TMUX_SOCKET}) =="
  tmux_cmd list-sessions 2>/dev/null || echo "No SimForDrone sessions"
  echo
  echo "== UDP ports =="
  ss -lunp 2>/dev/null | grep -E ':(8888|18570|14580)\b' || true
  echo
  echo "== processes =="
  pgrep -af 'MicroXRCEAgent|PX4-Autopilot/.*/bin/px4|gz sim' || true
  echo
  echo "== ROS 2 PX4 topics =="
  if [[ -f "${ROS_WS_SETUP}" ]]; then
    source_ros
    local count
    count="$(ros2 topic list 2>/dev/null | grep -c '^/fmu/' || true)"
    echo "/fmu topic count: ${count}"
    ros2 topic list 2>/dev/null | grep -E '^/fmu/out/(vehicle_status|vehicle_odometry|vehicle_local_position)' || true
  else
    echo "ROS workspace has not been built"
  fi
}

find_topic() {
  local pattern="$1"
  ros2 topic list | grep -E "${pattern}" | head -n 1 || true
}

cmd_test() {
  source_ros
  local status_topic position_topic
  status_topic="$(find_topic '^/fmu/out/vehicle_status(_v[0-9]+)?$')"
  position_topic="$(find_topic '^/fmu/out/vehicle_local_position(_v[0-9]+)?$')"

  if [[ -z "${status_topic}" || -z "${position_topic}" ]]; then
    echo "ERROR: PX4 output topics not found. Run '$0 status' and inspect both logs." >&2
    exit 1
  fi

  echo "PASS: status topic = ${status_topic}"
  ros2 topic echo --once "${status_topic}"
  echo "PASS: position topic = ${position_topic}"
  ros2 topic echo --once "${position_topic}"

  if ros2 topic info /fmu/in/offboard_control_mode --verbose | grep -q 'Subscription count: 1'; then
    echo "PASS: PX4 subscribes to /fmu/in/offboard_control_mode"
  else
    echo "FAIL: PX4 input subscription was not detected" >&2
    exit 1
  fi

  echo "ROS 2 <-> PX4 read path and PX4 input reader discovery passed."
}

cmd_attach() {
  case "${1:-}" in
    agent) tmux_cmd attach-session -t "${AGENT_SESSION}" ;;
    px4)   tmux_cmd attach-session -t "${PX4_SESSION}" ;;
    *) echo "Usage: $0 attach {agent|px4}" >&2; exit 2 ;;
  esac
}

cmd_logs() {
  case "${1:-px4}" in
    agent) tail -n 100 -f "${LOG_DIR}/agent.log" ;;
    px4)   tail -n 100 -f "${LOG_DIR}/px4.log" ;;
    *) echo "Usage: $0 logs {agent|px4}" >&2; exit 2 ;;
  esac
}

cmd_stop() {
  echo "Stopping PX4/Gazebo..."
  if session_exists "${PX4_SESSION}"; then
    tmux_cmd send-keys -t "${PX4_SESSION}" C-c
    sleep 2
    tmux_cmd kill-session -t "${PX4_SESSION}" 2>/dev/null || true
  fi

  echo "Stopping Micro XRCE-DDS Agent..."
  if session_exists "${AGENT_SESSION}"; then
    tmux_cmd send-keys -t "${AGENT_SESSION}" C-c
    sleep 1
    tmux_cmd kill-session -t "${AGENT_SESSION}" 2>/dev/null || true
  fi
  echo "Stopped."
}

case "${1:-}" in
  build)  shift; cmd_build "$@" ;;
  start)  shift; cmd_start "$@" ;;
  status) shift; cmd_status "$@" ;;
  test)   shift; cmd_test "$@" ;;
  attach) shift; cmd_attach "$@" ;;
  logs)   shift; cmd_logs "$@" ;;
  stop)   shift; cmd_stop "$@" ;;
  -h|--help|help|"") usage ;;
  *) echo "ERROR: unknown command: $1" >&2; usage; exit 2 ;;
esac
