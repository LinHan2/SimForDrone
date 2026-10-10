#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/scripts/env/activate_system_ros2_jazzy.sh"
cd "${ROOT}"
export PYTHONPATH="${ROOT}/tracking:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
exec /usr/bin/python3 -u "${ROOT}/utils/bridge_target_pixel.py" "$@"