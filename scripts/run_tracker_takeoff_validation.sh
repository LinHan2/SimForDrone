#!/usr/bin/env bash
# P2.0 跟随机起飞验证入口。默认 dry-run；必须传 --execute 才会控制 PX4。

set -Eeuo pipefail

SIMFORDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMFORDRONE_ROOT
source "${SIMFORDRONE_ROOT}/env/activate_px4_mavlink_control.sh"

cd "${SIMFORDRONE_ROOT}"
exec "${SIMFORDRONE_PX4_PYTHON}" \
    "${SIMFORDRONE_ROOT}/src/simfordrone/tracker_takeoff_validation.py" "$@"

