#!/usr/bin/env bash
# P2.0 的独立位姿验收终端：仅订阅 ROS，不向 PX4 发送命令。

set -Eeuo pipefail

SIMFORDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SIMFORDRONE_ROOT}/scripts/env/activate_system_ros2_jazzy.sh"

cd "${SIMFORDRONE_ROOT}"
exec /usr/bin/python3 "${SIMFORDRONE_ROOT}/utils/record_tracker_takeoff_pose.py" "$@"
