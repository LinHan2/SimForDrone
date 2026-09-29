#!/usr/bin/env bash
# 编排 target、truth tracker 与只读 shadow；仅在 FOV 门通过后放行 target 航迹。

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SHADOW_DURATION=60
FOV_TIMEOUT=30
FOV_FRAMES=15
READY_TIMEOUT=90

usage() {
    cat <<'EOF'
用法：./scripts/run_target_shadow_gated.sh [编排选项] --execute [-- <target 航迹选项>]

示例：
  ./scripts/run_target_shadow_gated.sh --execute -- \
        --trajectory circle --trajectory-radius 3 --trajectory-cycles 2 \
        --max-speed 2.2 --max-accel 2.5

要求先单独启动双 PX4 场景；此命令会解锁并控制两架飞机。
target 先悬停，truth tracker 启动并进入跟踪，然后检查 Oracle FOV + RGB-D；
通过后启动只读 shadow 并放行 target 轨迹。门未通过则安全收尾。
可选：--ready-timeout 90 --fov-timeout 30 --fov-frames 15 --shadow-duration 60
FOV 门不是图像目标检测，也不保证轨迹全程可见；shadow 不驱动 tracker。
EOF
}

target_args=()
execute=0
while (($#)); do
    case "$1" in
        --shadow-duration|--fov-timeout|--fov-frames|--ready-timeout)
            if (($# < 2)); then echo "缺少 $1 的值" >&2; exit 2; fi
            case "$1" in
                --shadow-duration) SHADOW_DURATION="$2" ;;
                --fov-timeout) FOV_TIMEOUT="$2" ;;
                --fov-frames) FOV_FRAMES="$2" ;;
                --ready-timeout) READY_TIMEOUT="$2" ;;
            esac
            shift 2 ;;
        --execute) execute=1; shift ;;
        --help|-h) usage; exit 0 ;;
        --) shift; target_args=("$@"); break ;;
        *) echo "未知编排选项：$1" >&2; usage >&2; exit 2 ;;
    esac
done

if (( ! execute )); then
    echo "安全门：必须显式传入 --execute 才能启动两架飞机。" >&2
    exit 2
fi
if ! [[ "$SHADOW_DURATION" =~ ^[0-9]+([.][0-9]+)?$ && "$SHADOW_DURATION" != 0 ]] ||
   ! [[ "$FOV_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ && "$FOV_TIMEOUT" != 0 ]] ||
   ! [[ "$READY_TIMEOUT" =~ ^[0-9]+$ && "$READY_TIMEOUT" != 0 ]] ||
   ! [[ "$FOV_FRAMES" =~ ^[1-9][0-9]*$ ]]; then
    echo "超时与时长必须为正数，帧数必须为正整数。" >&2
    exit 2
fi
for arg in "${target_args[@]}"; do
    case "$arg" in
        --probe|--interactive|--start-delay|--start-gate-file|--start-gate-timeout|--state-port|--shadow-state-port|--state-host|--execute)
            echo "编排脚本不接受 target 参数 $arg；请使用默认航迹或在 -- 后传入轨迹参数。" >&2; exit 2 ;;
    esac
done
if pgrep -f -- "${ROOT}/scripts/01_dual_px4_scene.py" >/dev/null; then
    :
else
    echo "安全门：未检测到双 PX4 场景；请先运行 ./scripts/start_dual_px4_scene.sh" >&2
    exit 2
fi
if ss -H -ulpn 'sport = :14601' | grep -q .; then
    echo "安全门：UDP 14601 已被占用；不会终止未知进程。请先执行：ss -ulpn | grep ':14601'" >&2
    exit 2
fi

mkdir -p "${ROOT}/logs/tracking"
run_dir="$(mktemp -d "${ROOT}/logs/tracking/gated-XXXXXX")"
gate_file="${run_dir}/release"
target_log="${run_dir}/target.log"
tracker_log="${run_dir}/tracker.log"
shadow_log="${run_dir}/shadow.log"
target_pid='' tracker_pid='' shadow_pid=''
cleanup() {
    trap - EXIT INT TERM
    for pid in "$target_pid" "$tracker_pid"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then kill -INT "$pid" 2>/dev/null || true; fi
    done
    if [[ -n "$shadow_pid" ]]; then kill -TERM "$shadow_pid" 2>/dev/null || true; fi
    for pid in "$target_pid" "$tracker_pid" "$shadow_pid"; do
        if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
    done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[gated] 日志：${run_dir}"
echo "[gated] 启动 target 并保持悬停等待放行"
gate_timeout="$(awk -v ready="$READY_TIMEOUT" -v fov="$FOV_TIMEOUT" 'BEGIN { printf "%.0f", 2 * ready + fov + 60 }')"
"${ROOT}/scripts/run_target_waypoints.sh" --execute --start-delay 0 \
    --start-gate-file "$gate_file" --start-gate-timeout "$gate_timeout" \
    "${target_args[@]}" >"$target_log" 2>&1 &
target_pid=$!

echo "[gated] 等待 target 真正进入悬停"
deadline=$((SECONDS + READY_TIMEOUT))
until grep -q 'target 已悬停，等待 tracker' "$target_log"; do
    if ! kill -0 "$target_pid" 2>/dev/null || (( SECONDS >= deadline )); then
        echo "安全门：target 未进入悬停；查看 $target_log" >&2
        tail -n 20 "$target_log" >&2
        if grep -q 'arm_command 被 PX4 拒绝' "$target_log"; then
            scene_log="$(ls -t "${ROOT}"/logs/isaac_px4/dual_px4_*.log 2>/dev/null | head -n 1 || true)"
            if [[ -n "$scene_log" ]]; then
                echo "场景日志最近的 PX4 解锁告警（$scene_log）：" >&2
                tail -n 30 "$scene_log" | grep -E 'Arming denied|Preflight Fail|health' >&2 || true
            fi
            echo "请勿反复强制解锁。确认两机落地上锁后，运行 ./scripts/restart_px4_only.sh --execute；若当前场景尚未加载该功能或健康告警仍在，再手动重建场景。" >&2
        fi
        exit 1
    fi
    sleep 1
done

echo "[gated] 启动 tracker（truth 基线）"
"${ROOT}/scripts/run_tracker.sh" --execute --state-source truth --duration 0 --status-period 1 \
    --follow-distance 5 >"$tracker_log" 2>&1 &
tracker_pid=$!
deadline=$((SECONDS + READY_TIMEOUT))
until grep -q 'TRACK t=' "$tracker_log"; do
    if ! kill -0 "$target_pid" 2>/dev/null || ! kill -0 "$tracker_pid" 2>/dev/null || (( SECONDS >= deadline )); then
        echo "安全门：tracker 未能进入跟踪；查看 $target_log 和 $tracker_log" >&2
        tail -n 12 "$target_log" "$tracker_log" >&2
        exit 1
    fi
    sleep 1
done

echo "[gated] 等待 tracker 水平间距进入 5 ± 0.5 m"
deadline=$((SECONDS + READY_TIMEOUT))
until grep -q 'FOLLOW DISTANCE READY' "$tracker_log"; do
    if ! kill -0 "$target_pid" 2>/dev/null || ! kill -0 "$tracker_pid" 2>/dev/null || (( SECONDS >= deadline )); then
        echo "安全门：tracker 未能接近 5m；查看 $tracker_log" >&2
        tail -n 12 "$tracker_log" >&2
        exit 1
    fi
    sleep 1
done

echo "[gated] tracker 已进入跟踪；等待连续 ${FOV_FRAMES} 帧 Oracle FOV + 深度"
(
    source "${ROOT}/scripts/env/activate_system_ros2_jazzy.sh"
    cd "${ROOT}"
    export PYTHONPATH="${ROOT}/tracking:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
    /usr/bin/python3 "${ROOT}/utils/wait_tracker_fov.py" --timeout "$FOV_TIMEOUT" --required-frames "$FOV_FRAMES"
) || { echo "安全门：FOV 未通过；两机将安全收尾。" >&2; exit 1; }
if ! kill -0 "$target_pid" 2>/dev/null || ! kill -0 "$tracker_pid" 2>/dev/null; then
    echo "安全门：控制器已退出，不放行 target 航迹。" >&2
    exit 1
fi

"${ROOT}/scripts/run_shadow_relative_ekf.sh" --duration "$SHADOW_DURATION" >"$shadow_log" 2>&1 &
shadow_pid=$!
shadow_deadline=$((SECONDS + 15))
until grep -q 'SHADOW READY:' "$shadow_log"; do
    if ! kill -0 "$target_pid" 2>/dev/null || ! kill -0 "$tracker_pid" 2>/dev/null ||
       grep -q 'SHADOW STARTUP FAILED:' "$shadow_log" || (( SECONDS >= shadow_deadline )); then
        echo "安全门：shadow 未就绪；不放行 target。查看 $shadow_log" >&2
        tail -n 12 "$shadow_log" >&2
        exit 1
    fi
    sleep 1
done
touch "$gate_file"
echo "[gated] shadow 已启动；target 航迹放行。日志：$run_dir"
while kill -0 "$target_pid" 2>/dev/null; do
    if ! kill -0 "$tracker_pid" 2>/dev/null; then
        echo "tracker 提前退出；终止 target 航迹。查看 $tracker_log" >&2
        exit 1
    fi
    sleep 1
done
wait "$target_pid"
target_pid=''
echo "[gated] target 已结束；等待 tracker 安全收尾"
wait "$tracker_pid"
tracker_pid=''
wait "$shadow_pid"
shadow_pid=''