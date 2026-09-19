#!/usr/bin/env bash
# 仅控制 target：执行航点并发布共享 ENU 状态给 tracker。
#
# 职责边界：本脚本独占 MAVLink 14540，只控制 target，永不下发 tracker 命令；
# 与 tracker 进程通过 UDP 127.0.0.1:14600 交换目标状态。
# 默认不带 --execute 时为 dry-run，不会解锁或起飞。
#
# 常用示例：
#   ./scripts/run_target_waypoints.sh --interactive --execute  # 终端手动输入航点
#   ./scripts/run_target_waypoints.sh --point 3 0 2 --execute  # 脚本预设航点

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMFORDRONE_ROOT="${ROOT}"
source "${ROOT}/scripts/env/activate_px4_mavlink_control.sh"

cd "${ROOT}"
export PYTHONPATH="${ROOT}/tracking:${ROOT}/px4ctrl"
exec "${SIMFORDRONE_PX4_PYTHON}" -m tracking.target_waypoints "$@"
