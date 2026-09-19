#!/usr/bin/env bash
# 使用 JSON 解析器汇总最新一次独立 tracker 运行结果。
#
# 为什么用 /usr/bin/python3 而不是 conda/PX4 venv：本脚本只读 JSON，
# 不需要 pymavlink 或 rclpy，使用系统 Python 可以避免控制环境与观察环境混用。

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
shopt -s nullglob
runs=("${ROOT}"/logs/tracking/tracker-v0-*/run.json)
if (( ${#runs[@]} == 0 )); then
    echo "ERROR: 尚无 tracker-v0 运行日志" >&2
    exit 1
fi
latest="${runs[${#runs[@]}-1]}"

/usr/bin/python3 - "${latest}" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, encoding="utf-8") as stream:
    result = json.load(stream)

print(f"日志: {path}")
print(f"结果: {result.get('result', 'unknown')}")
print(f"状态源: {result.get('state_source', 'unknown')}")
print(f"相对 offset: {result.get('relative_offset', '未记录')}")
mean_error = result.get("error_mean_m")
max_error = result.get("error_max_m")
print(f"平均误差: {mean_error:.3f} m" if mean_error is not None else "平均误差: 未记录（可能是 dry-run）")
print(f"最大误差: {max_error:.3f} m" if max_error is not None else "最大误差: 未记录（可能是 dry-run）")
print(f"落地: {result.get('on_ground', '未记录')}")
print(f"上锁: {result.get('disarmed', '未记录')}")
print(f"通过: {result.get('passed', '未记录')}")
PY
