#!/usr/bin/env bash
# 双机观测阶段的只读验收：不启动、不停止，也不向飞控发送控制指令。

set -Ee -o pipefail

# 验收端使用系统 ROS Jazzy，与 Isaac Sim 内部 Python 3.11 进程严格分离。
SIMFORDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SIMFORDRONE_ROOT}/scripts/env/activate_system_ros2_jazzy.sh"

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
echo "PASS: dual-UAV ROS 2 observation interface is publishing."
