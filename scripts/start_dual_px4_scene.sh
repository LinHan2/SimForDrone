#!/usr/bin/env bash
# 通过 Isaac Sim 内置 ROS 2 Jazzy bridge 与 WebRTC 启动 Pegasus 双 PX4 场景。
# 启动前必须停止其他 Isaac Sim 实例，否则会争用 GPU、端口和 DDS 资源。

set -Eeuo pipefail

SIMFORDRONE_ROOT="/data/disk2/home/hl/research/SimForDrone"
ISAACSIM_ROOT="/data/disk2/home/hl/isaacsim-5.1.0"
SCENE_SCRIPT="${SIMFORDRONE_ROOT}/scripts/01_dual_px4_scene.py"
ISAAC_ENV_SCRIPT="${SIMFORDRONE_ROOT}/env/activate_isaacsim_internal_ros.sh"
LOG_DIR="${SIMFORDRONE_ROOT}/logs/isaac_px4"
# NvStreamer 会把 .etli 跟踪日志写到进程的当前工作目录。此前工作目录是仓库根，
# 导致每轮启动都留下数十 MB 且持续累积；这里固定到 logs/ 下并限制保留份数。
ETLI_DIR="${SIMFORDRONE_ROOT}/logs/nvstreamer"
ETLI_KEEP="${ISAAC_ETLI_KEEP:-2}"
# 单个 .etli 的上限与检查周期；超过上限即截断，避免长时间运行把磁盘吃满。
ETLI_MAX_MB="${ISAAC_ETLI_MAX_MB:-64}"
ETLI_GUARD_INTERVAL="${ISAAC_ETLI_GUARD_INTERVAL:-30}"

# 仅在校园网可达地址变更时覆盖；默认值与现有端口映射保持一致。
PUBLIC_ENDPOINT="${ISAAC_PUBLIC_ENDPOINT:-10.134.88.113}"
STREAM_PORT="${ISAAC_STREAM_PORT:-49100}"
PX4_INPUT_SCALING="${SIMFORDRONE_PX4_INPUT_SCALING:-2000}"
ISAAC_GUI="${SIMFORDRONE_ISAAC_GUI:-0}"
# carb 设置 /app/livestream/webrtcEtli 控制 WebRTC 跟踪记录；置 0 可关闭。
WEBRTC_ETLI="${ISAAC_WEBRTC_ETLI:-0}"

die() {
    echo "ERROR: $*" >&2
    exit 1
}

usage() {
    cat <<'EOF'
Usage: ./scripts/start_dual_px4_scene.sh [--gui]

  --gui  Start the local Isaac Sim desktop interface. Requires an available
         graphical display; it exposes the Stage, Property, Timeline, and
         status-bar panels for inspecting scene objects.

Environment overrides:
  SIMFORDRONE_PX4_INPUT_SCALING  PX4 output-to-thrust gain (default 2000)
  ISAAC_WEBRTC_ETLI              WebRTC trace arg, informational only (default 0)
  ISAAC_ETLI_KEEP                How many .etli traces to retain (default 2)
  ISAAC_ETLI_MAX_MB              Truncate an .etli once it exceeds this (default 64)
  SIMFORDRONE_ISAAC_GUI          Same as --gui when set to 1

Note: the WebRTC .etli trace cannot be disabled from the command line; it is bounded
by ISAAC_ETLI_MAX_MB instead.
EOF
}

case "${1:-}" in
    "") ;;
    --gui) ISAAC_GUI=1 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "Unknown argument: $1" ;;
esac

[[ -x "${ISAACSIM_ROOT}/python.sh" ]] || die "Isaac Sim Python not found: ${ISAACSIM_ROOT}/python.sh"
[[ -f "${SCENE_SCRIPT}" ]] || die "Dual-scene script not found: ${SCENE_SCRIPT}"
[[ -f "${ISAAC_ENV_SCRIPT}" ]] || die "Isaac runtime profile not found: ${ISAAC_ENV_SCRIPT}"
[[ -d "${SIMFORDRONE_ROOT}/PegasusSimulator/extensions/pegasus.simulator" ]] \
    || die "Pegasus source extension not found"
[[ -x "${SIMFORDRONE_ROOT}/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]] \
    || die "PX4 SITL binary is missing; build PX4 before starting this scene"

mkdir -p "${LOG_DIR}" "${ETLI_DIR}"
LOG_FILE="${LOG_DIR}/dual_px4_$(date +%Y%m%d-%H%M%S).log"

# 只保留最近 ${ETLI_KEEP} 份跟踪日志；同时清理历史遗留在仓库根的同类文件。
# 注意：必须避免把 `ls` 的"无匹配"失败经由管道传播——配合 `set -e` 会让脚本静默退出。
prune_etli() {
    local dir="$1" keep="$2"
    [[ -d "${dir}" ]] || return 0
    local -a traces=()
    while IFS= read -r line; do
        [[ -n "${line}" ]] && traces+=("${line}")
    done < <(ls -1t "${dir}"/*.etli 2>/dev/null || true)

    local index
    for ((index = keep; index < ${#traces[@]}; index++)); do
        rm -f "${traces[index]}"
    done
    return 0
}
prune_etli "${ETLI_DIR}" "${ETLI_KEEP}"
prune_etli "${SIMFORDRONE_ROOT}" 0

cat <<EOF
Starting Pegasus dual-PX4 scene
  Isaac Sim: ${ISAACSIM_ROOT}
  PX4:       ${SIMFORDRONE_ROOT}/PX4-Autopilot
  ROS 2:     Jazzy internal rclpy + Fast DDS
  PX4 gain:  ${PX4_INPUT_SCALING}
  Isaac UI:  $([[ "${ISAAC_GUI}" == "1" ]] && echo enabled || echo disabled)
  WebRTC:    ${PUBLIC_ENDPOINT}:${STREAM_PORT}
  WebRTC trace (etli): ${WEBRTC_ETLI} -> ${ETLI_DIR}（单文件上限 ${ETLI_MAX_MB} MB）
  Log:       ${LOG_FILE}

Stop any separately launched isaac-sim.streaming.sh instance before continuing.
Press Ctrl-C here to stop this scene and both PX4 child processes.
EOF

# NvStreamServer 的 .etli 跟踪日志无法通过命令行关闭（`/app/livestream/webrtcEtli`
# 实测无效，开关在编译库内）。既然它写在我们的工作目录里，就在此处限制其体积：
# 超过上限即截断。写入方持有同一个 inode，会继续写下去，磁盘占用因此被钉住。
guard_etli_size() {
    local cap_mb="$1" dir="$2" file size_mb
    while true; do
        sleep "${ETLI_GUARD_INTERVAL}"
        for file in "${dir}"/*.etli; do
            [[ -f "${file}" ]] || continue
            size_mb=$(du -m "${file}" 2>/dev/null | cut -f1)
            if [[ -n "${size_mb}" && "${size_mb}" -gt "${cap_mb}" ]]; then
                if truncate -s 0 "${file}" 2>/dev/null; then
                    echo "[etli-guard] ${file} 超过 ${cap_mb} MB，已截断" >&2
                fi
            fi
        done
    done
}

guard_etli_size "${ETLI_MAX_MB}" "${ETLI_DIR}" &
ETLI_GUARD_PID=$!
# 场景结束后必须回收守护进程，否则每轮启动都会泄漏一个后台任务。
# shellcheck disable=SC2317  # 由 trap 调用
stop_etli_guard() { kill "${ETLI_GUARD_PID}" 2>/dev/null || true; }
trap stop_etli_guard EXIT INT TERM

# 在子 shell 中加载 Isaac 专用环境，避免它反向污染用户当前 SSH 终端。
(
    export SIMFORDRONE_ROOT ISAACSIM_ROOT SIMFORDRONE_PX4_INPUT_SCALING="${PX4_INPUT_SCALING}" SIMFORDRONE_ISAAC_GUI="${ISAAC_GUI}"
    source "${ISAAC_ENV_SCRIPT}"
    # 切换到 logs/ 下的专用目录，使 .etli 不再落到仓库根。
    cd "${ETLI_DIR}"
    exec "${ISAACSIM_ROOT}/python.sh" \
        "${SCENE_SCRIPT}" \
        --/app/livestream/publicEndpointAddress="${PUBLIC_ENDPOINT}" \
        --/app/livestream/port="${STREAM_PORT}" \
        --/app/livestream/webrtcEtli="${WEBRTC_ETLI}"
) 2>&1 | tee "${LOG_FILE}"
stop_etli_guard
