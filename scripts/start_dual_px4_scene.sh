#!/usr/bin/env bash
# 通过 Isaac Sim 内置 ROS 2 Jazzy bridge 与 WebRTC 启动 Pegasus 双 PX4 场景。
# 启动前必须停止其他 Isaac Sim 实例，否则会争用 GPU、端口和 DDS 资源。

set -Eeuo pipefail

SIMFORDRONE_ROOT="/data/disk2/home/hl/research/SimForDrone"
ISAACSIM_ROOT="/data/disk2/home/hl/isaacsim-5.1.0"
SCENE_SCRIPT="${SIMFORDRONE_ROOT}/scripts/01_dual_px4_scene.py"
ISAAC_ENV_SCRIPT="${SIMFORDRONE_ROOT}/env/activate_isaacsim_internal_ros.sh"
LOG_DIR="${SIMFORDRONE_ROOT}/logs/isaac_px4"

# 仅在校园网可达地址变更时覆盖；默认值与现有端口映射保持一致。
PUBLIC_ENDPOINT="${ISAAC_PUBLIC_ENDPOINT:-10.134.88.113}"
STREAM_PORT="${ISAAC_STREAM_PORT:-49100}"
PX4_INPUT_SCALING="${SIMFORDRONE_PX4_INPUT_SCALING:-2000}"

die() {
    echo "ERROR: $*" >&2
    exit 1
}

[[ -x "${ISAACSIM_ROOT}/python.sh" ]] || die "Isaac Sim Python not found: ${ISAACSIM_ROOT}/python.sh"
[[ -f "${SCENE_SCRIPT}" ]] || die "Dual-scene script not found: ${SCENE_SCRIPT}"
[[ -f "${ISAAC_ENV_SCRIPT}" ]] || die "Isaac runtime profile not found: ${ISAAC_ENV_SCRIPT}"
[[ -d "${SIMFORDRONE_ROOT}/PegasusSimulator/extensions/pegasus.simulator" ]] \
    || die "Pegasus source extension not found"
[[ -x "${SIMFORDRONE_ROOT}/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]] \
    || die "PX4 SITL binary is missing; build PX4 before starting this scene"

mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/dual_px4_$(date +%Y%m%d-%H%M%S).log"

cat <<EOF
Starting Pegasus dual-PX4 scene
  Isaac Sim: ${ISAACSIM_ROOT}
  PX4:       ${SIMFORDRONE_ROOT}/PX4-Autopilot
  ROS 2:     Jazzy internal rclpy + Fast DDS
  PX4 gain:  ${PX4_INPUT_SCALING}
  WebRTC:    ${PUBLIC_ENDPOINT}:${STREAM_PORT}
  Log:       ${LOG_FILE}

Stop any separately launched isaac-sim.streaming.sh instance before continuing.
Press Ctrl-C here to stop this scene and both PX4 child processes.
EOF

# 在子 shell 中加载 Isaac 专用环境，避免它反向污染用户当前 SSH 终端。
(
    export SIMFORDRONE_ROOT ISAACSIM_ROOT SIMFORDRONE_PX4_INPUT_SCALING="${PX4_INPUT_SCALING}"
    source "${ISAAC_ENV_SCRIPT}"
    exec "${ISAACSIM_ROOT}/python.sh" \
        "${SCENE_SCRIPT}" \
        --/app/livestream/publicEndpointAddress="${PUBLIC_ENDPOINT}" \
        --/app/livestream/port="${STREAM_PORT}"
) 2>&1 | tee "${LOG_FILE}"
