#!/usr/bin/env bash
# px4ctrl 的唯一宿主侧入口：加载 MAVLink 控制环境后转交 Python 包。
#
# 用法示例:
#   ./scripts/run_px4ctrl.sh probe --role target
#   ./scripts/run_px4ctrl.sh measure-hover --role target --execute
#   ./scripts/run_px4ctrl.sh takeoff-hover-land --role target --execute --hold-seconds 30

set -Eeuo pipefail

SIMFORDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMFORDRONE_ROOT
source "${SIMFORDRONE_ROOT}/scripts/env/activate_px4_mavlink_control.sh"

cd "${SIMFORDRONE_ROOT}/px4ctrl"
# 显式给出绝对日志根目录：否则相对路径会落在包目录里，且随启动方式而变化。
exec "${SIMFORDRONE_PX4_PYTHON}" -m px4ctrl.cli \
    --output-root "${SIMFORDRONE_ROOT}/logs/px4ctrl" "$@"
