#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMFORDRONE_ROOT="${ROOT}"
source "${ROOT}/scripts/env/activate_px4_mavlink_control.sh"
exec "${SIMFORDRONE_PX4_PYTHON}" -u "${ROOT}/scripts/tracking_experiment.py" "$@"