#!/usr/bin/env bash
# V0 双机 GT 位置跟踪入口。默认 dry-run，必须显式 --execute 才会飞行。
#
# 注意：本入口在单一进程内同时控制两台 PX4（早期实现）。若需要分终端独立
# 启动、或需要手动输入航点，请改用 test/run_target_waypoints.sh 与
# test/run_tracker.sh 两个进程。

set -Eeuo pipefail

SIMFORDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMFORDRONE_ROOT
source "${SIMFORDRONE_ROOT}/env/activate_px4_mavlink_control.sh"

cd "${SIMFORDRONE_ROOT}"
export PYTHONPATH="${SIMFORDRONE_ROOT}/tracking:${SIMFORDRONE_ROOT}/px4ctrl"
exec "${SIMFORDRONE_PX4_PYTHON}" -m tracking.run_static "$@"
