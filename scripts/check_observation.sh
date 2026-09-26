#!/usr/bin/env bash
# 只读 ROS 2 观测验收：默认检查话题，--rgbd 额外采样并检查前视 RGB-D。

set -Eeuo pipefail

SIMFORDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SIMFORDRONE_ROOT}/scripts/env/activate_system_ros2_jazzy.sh"

check_rgbd=0
case "${1:-}" in
    "") ;;
    --rgbd) check_rgbd=1 ;;
    -h|--help)
        echo "Usage: ./scripts/check_observation.sh [--rgbd]"
        exit 0
        ;;
    *)
        echo "ERROR: unknown option: $1" >&2
        exit 2
        ;;
esac

required_topics=(
    /target_uav_0/state/pose
    /tracker_uav_1/state/pose
    /tracker_uav_1/front_camera/color/image_raw
    /tracker_uav_1/front_camera/color/camera_info
    /tracker_uav_1/front_camera/depth
)

topic_list="$(ros2 topic list)"
missing=0
for topic in "${required_topics[@]}"; do
    if grep -Fxq "${topic}" <<<"${topic_list}"; then
        printf 'OK      %s\n' "${topic}"
    else
        printf 'MISSING %s\n' "${topic}" >&2
        missing=1
    fi
done

(( missing == 0 )) || exit 1

echo
echo "Sampling one pose and camera calibration message (15 s timeout each):"
timeout 15s ros2 topic echo --once /tracker_uav_1/state/pose
timeout 15s ros2 topic echo --once /tracker_uav_1/front_camera/color/camera_info

if (( check_rgbd )); then
    capture_log="$(mktemp)"
    trap 'rm -f "${capture_log}"' EXIT
    cd "${SIMFORDRONE_ROOT}"
    /usr/bin/python3 "${SIMFORDRONE_ROOT}/utils/capture_rgbd_sample.py" \
        --output-root "${SIMFORDRONE_ROOT}/logs/scene_rgb_checks" | tee "${capture_log}"
    sample_dir="$(sed -n 's/^SAVED: //p' "${capture_log}" | tail -n 1)"
    if [[ -z "${sample_dir}" || ! -f "${sample_dir}/rgb.png" ]]; then
        echo "FAIL: 未能从采样器输出定位 rgb.png" >&2
        exit 1
    fi
    /usr/bin/python3 "${SIMFORDRONE_ROOT}/utils/evaluate_observer_rgb.py" "${sample_dir}/rgb.png"
fi

echo "PASS: dual-UAV ROS 2 observation interface is publishing."