#!/usr/bin/env bash
# 仅控制 tracker：订阅 target 状态并执行 V0 相对位置跟踪。
#
# 职责边界：本脚本独占 MAVLink 14541，只控制 tracker；target 状态来自
# 另一个进程（scripts/run_target_waypoints.sh）经由 UDP 127.0.0.1:14600 发布。
# --duration 0 表示一直跟踪到 Ctrl-C 或目标状态超时。
#
# 常用示例：
#   ./scripts/run_tracker.sh --execute                  # 锁存当前相对位置持续跟踪
#   ./scripts/run_tracker.sh --offset-east -3 --execute # 指定站位：目标东側 3 m

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMFORDRONE_ROOT="${ROOT}"
source "${ROOT}/scripts/env/activate_px4_mavlink_control.sh"

cd "${ROOT}"
export PYTHONPATH="${ROOT}/tracking:${ROOT}/px4ctrl"
exec "${SIMFORDRONE_PX4_PYTHON}" -m tracking.run_tracker "$@"
