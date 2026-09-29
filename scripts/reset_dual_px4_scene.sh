#!/usr/bin/env bash
# 完整重置 Isaac/Pegasus/双 PX4 SITL，清除飞行后残留的 PX4 EKF 与 health 状态。
# 不发送 arm、offboard 或其他飞控命令。

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENE_SCRIPT="${ROOT}/scripts/01_dual_px4_scene.py"
START_SCRIPT="${ROOT}/scripts/start_dual_px4_scene.sh"
WAIT_SECONDS="${SIMFORDRONE_RESET_WAIT_SECONDS:-30}"
force=0
stop_only=0

usage() {
    cat <<'EOF'
Usage: ./scripts/reset_dual_px4_scene.sh [--stop-only] [--force]

Stops the SimForDrone Isaac/Pegasus dual-PX4 scene, waits for it to exit, then
starts a fresh scene in this terminal. This clears PX4 SITL estimator/health state.

  --stop-only  Stop the current scene but do not restart it.
  --force      Send SIGTERM only when the scene ignores SIGINT past the timeout.

The default operation never arms a vehicle or sends flight setpoints.
EOF
}

case "${1:-}" in
    "") ;;
    --stop-only) stop_only=1 ;;
    --force) force=1 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

mapfile -t scene_pids < <(pgrep -f -- "${SCENE_SCRIPT}" || true)
if (( ${#scene_pids[@]} > 0 )); then
    echo "Reset: requesting clean shutdown of scene PID(s): ${scene_pids[*]}"
    kill -INT "${scene_pids[@]}"
    deadline=$((SECONDS + WAIT_SECONDS))
    while (( SECONDS < deadline )); do
        if ! pgrep -f -- "${SCENE_SCRIPT}" >/dev/null; then
            break
        fi
        sleep 1
    done
fi

if pgrep -f -- "${SCENE_SCRIPT}" >/dev/null; then
    if (( ! force )); then
        echo "ERROR: scene did not stop within ${WAIT_SECONDS}s; inspect its terminal or rerun with --force." >&2
        exit 1
    fi
    mapfile -t scene_pids < <(pgrep -f -- "${SCENE_SCRIPT}")
    echo "Reset: clean shutdown timed out; sending SIGTERM to PID(s): ${scene_pids[*]}" >&2
    kill -TERM "${scene_pids[@]}"
    sleep 2
    if pgrep -f -- "${SCENE_SCRIPT}" >/dev/null; then
        echo "ERROR: scene remains active after SIGTERM; refusing SIGKILL." >&2
        exit 1
    fi
fi

echo "Reset: old Isaac/Pegasus/PX4 scene has stopped."
if (( stop_only )); then
    exit 0
fi

exec "${START_SCRIPT}"
