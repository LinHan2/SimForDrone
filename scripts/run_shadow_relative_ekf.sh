#!/usr/bin/env bash
# 只读 RGB-D 相对 EKF 验证：不打开 MAVLink，不发送飞控指令。

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/scripts/env/activate_system_ros2_jazzy.sh"

cd "${ROOT}"
export PYTHONPATH="${ROOT}/tracking:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
exec /usr/bin/python3 "${ROOT}/utils/shadow_relative_ekf.py" "$@"
