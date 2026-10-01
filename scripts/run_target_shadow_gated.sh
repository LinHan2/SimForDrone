#!/usr/bin/env bash
# 编排 target、truth tracker 与只读 shadow；仅在 FOV 门通过后放行 target 航迹。

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SHADOW_DURATION=60
FOV_TIMEOUT=30
FOV_FRAMES=15
READY_TIMEOUT=90
PBVS_SHADOW=0
VISIBILITY_PREDICTION=0
ORACLE_ROI_EVERY_FRAME=0
TARGET_HORIZONTAL_KP=0.20
TRUTH_BASELINE=0
FEEDFORWARD=acceleration
PREDICTION_HORIZON=0

usage() {
    cat <<'EOF'
用法：./scripts/run_target_shadow_gated.sh [编排选项] --execute [-- <target 航迹选项>]

示例：
  ./scripts/run_target_shadow_gated.sh --execute -- \
        --trajectory circle --trajectory-radius 1.5 --trajectory-cycles 2 \
        --max-speed 0.20 --max-accel 0.15

当前入口只支持第一层验证：GT 目标状态 → tracker → 控制器。
必须同时传入 --truth-baseline 显式确认这不是视觉伺服，才会解锁两架飞机。
须先单独启动双 PX4 场景。
target 先悬停，truth tracker 启动并进入跟踪，然后检查 Oracle FOV + RGB-D；
通过后启动只读 shadow 并放行 target 轨迹。门未通过则安全收尾。
可选：--ready-timeout 90 --fov-timeout 30 --fov-frames 15 --shadow-duration 60
    --target-horizontal-kp 0.20（仅门控 target 水平位置增益；真机配置不变）
    --pbvs-shadow（记录 Airsim PBVS 候选速度；tracker 仍下发 V0 指令）
    --visibility-prediction（仅记录双姿态未来视野预测；不参与控制）
    --oracle-roi-every-frame（只读实验：每帧真值定位 ROI，位置量测仍取深度）
FOV 门不是图像目标检测，也不保证轨迹全程可见；shadow 不驱动 tracker。
EOF
}

target_args=()
execute=0
while (($#)); do
    case "$1" in
        --shadow-duration|--fov-timeout|--fov-frames|--ready-timeout|--target-horizontal-kp|--feedforward|--prediction-horizon)
            if (($# < 2)); then echo "缺少 $1 的值" >&2; exit 2; fi
            case "$1" in
                --shadow-duration) SHADOW_DURATION="$2" ;;
                --fov-timeout) FOV_TIMEOUT="$2" ;;
                --fov-frames) FOV_FRAMES="$2" ;;
                --ready-timeout) READY_TIMEOUT="$2" ;;
                --target-horizontal-kp) TARGET_HORIZONTAL_KP="$2" ;;
                --feedforward) FEEDFORWARD="$2" ;;
                --prediction-horizon) PREDICTION_HORIZON="$2" ;;
            esac
            shift 2 ;;
        --execute) execute=1; shift ;;
        --truth-baseline) TRUTH_BASELINE=1; shift ;;
        --pbvs-shadow) PBVS_SHADOW=1; shift ;;
        --visibility-prediction) VISIBILITY_PREDICTION=1; shift ;;
        --oracle-roi-every-frame) ORACLE_ROI_EVERY_FRAME=1; shift ;;
        --help|-h) usage; exit 0 ;;
        --) shift; target_args=("$@"); break ;;
        *) echo "未知编排选项：$1" >&2; usage >&2; exit 2 ;;
    esac
done

if (( ! execute )); then
    echo "安全门：必须显式传入 --execute 才能启动两架飞机。" >&2
    exit 2
fi
# tracker 下发的是 GT 目标状态导出的 V0 位置指令，Oracle FOV 只验证起点；
# 要求额外确认一次，避免这条命令被当成已验证的视觉伺服闭环。
if (( ! TRUTH_BASELINE )); then
    echo "安全门：本入口是第一层 GT 控制链验证，tracker 用真值目标状态下发 V0 位置指令，" >&2
    echo "未接入目标像素与深度闭环，Oracle FOV 只检查起点，不保证全程可见。" >&2
    echo "确认要跑这一层请加 --truth-baseline；不要据此声称视觉伺服可用。" >&2
    exit 2
fi
if ! [[ "$SHADOW_DURATION" =~ ^[0-9]+([.][0-9]+)?$ && "$SHADOW_DURATION" != 0 ]] ||
   ! [[ "$FOV_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ && "$FOV_TIMEOUT" != 0 ]] ||
   ! [[ "$READY_TIMEOUT" =~ ^[0-9]+$ && "$READY_TIMEOUT" != 0 ]] ||
   ! [[ "$FOV_FRAMES" =~ ^[1-9][0-9]*$ ]]; then
    echo "超时与时长必须为正数，帧数必须为正整数。" >&2
    exit 2
fi
if ! [[ "$TARGET_HORIZONTAL_KP" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   (( $(awk -v v="$TARGET_HORIZONTAL_KP" 'BEGIN { print (v <= 0) }') )); then
    echo "target-horizontal-kp 必须是正数" >&2
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
"${ROOT}/scripts/run_target_waypoints.sh" --execute --start-delay 0 --horizontal-kp "$TARGET_HORIZONTAL_KP" \
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
tracker_shadow_args=()
if (( PBVS_SHADOW )); then
    tracker_shadow_args+=(--pbvs-shadow)
fi
if (( VISIBILITY_PREDICTION )); then
    tracker_shadow_args+=(--visibility-prediction)
fi
"${ROOT}/scripts/run_tracker.sh" --execute --state-source truth --duration 0 --status-period 1 \
    --follow-band-min 5 --follow-band-max 10 \
    --feedforward "$FEEDFORWARD" --prediction-horizon "$PREDICTION_HORIZON" \
    "${tracker_shadow_args[@]}" >"$tracker_log" 2>&1 &
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

echo "[gated] 等待 tracker 水平间距进入 5~10 m 软约束带"
deadline=$((SECONDS + READY_TIMEOUT))
until grep -q 'FOLLOW BAND READY' "$tracker_log"; do
    if ! kill -0 "$target_pid" 2>/dev/null || ! kill -0 "$tracker_pid" 2>/dev/null || (( SECONDS >= deadline )); then
        echo "安全门：tracker 未能进入 5~10 m 软约束带；查看 $tracker_log" >&2
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

shadow_args=()
if (( ORACLE_ROI_EVERY_FRAME )); then
    shadow_args+=(--oracle-roi-every-frame)
fi
"${ROOT}/scripts/run_shadow_relative_ekf.sh" --duration "$SHADOW_DURATION" "${shadow_args[@]}" >"$shadow_log" 2>&1 &
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
        if grep -q '^PASS: tracker-v0' "$tracker_log"; then
            echo "[gated] tracker 已安全收尾；等待 target 完成落地上锁"
            break
        fi
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