#!/usr/bin/env bash
# 在独立系统 ROS Jazzy 进程中采样一次 RGB-D；不会控制 PX4。

set -Eeuo pipefail

SIMFORDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SIMFORDRONE_ROOT}/env/activate_system_ros2_jazzy.sh"

cd "${SIMFORDRONE_ROOT}"
exec /usr/bin/python3 "${SIMFORDRONE_ROOT}/utils/capture_rgbd_sample.py" "$@"
