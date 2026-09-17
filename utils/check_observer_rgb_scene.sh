#!/usr/bin/env bash
# 采集 observer 前视 RGB-D，并对 RGB 做基础可视性检查；不控制 PX4。

set -Eeuo pipefail

SIMFORDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SIMFORDRONE_ROOT}/env/activate_system_ros2_jazzy.sh"

capture_log="$(mktemp)"
trap 'rm -f "${capture_log}"' EXIT
cd "${SIMFORDRONE_ROOT}"
/usr/bin/python3 "${SIMFORDRONE_ROOT}/utils/capture_rgbd_sample.py" \
  --output-root "${SIMFORDRONE_ROOT}/logs/scene_rgb_checks" "$@" | tee "${capture_log}"

sample_dir="$(sed -n 's/^SAVED: //p' "${capture_log}" | tail -n 1)"
if [[ -z "${sample_dir}" || ! -f "${sample_dir}/rgb.png" ]]; then
  echo "FAIL: 未能从采样器输出定位 rgb.png" >&2
  exit 1
fi

exec /usr/bin/python3 "${SIMFORDRONE_ROOT}/utils/evaluate_observer_rgb.py" "${sample_dir}/rgb.png"
