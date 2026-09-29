#!/usr/bin/env bash
# Request Pegasus to recycle both PX4 SITL backends without resetting the Isaac scene.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMFORDRONE_ROOT="$ROOT"
if [[ "${1:-}" == "--help" ]]; then
    echo "Usage: ./scripts/restart_px4_only.sh --execute (both UAVs must be landed and disarmed)"
    exit 0
fi
if [[ "${1:-}" != "--execute" || $# -ne 1 ]]; then
    echo "安全门：仅允许显式 --execute；不会重置或绕过 PX4 健康检查。" >&2
    exit 2
fi
source "${ROOT}/scripts/env/activate_px4_mavlink_control.sh"
export PYTHONPATH="${ROOT}/px4ctrl${PYTHONPATH:+:${PYTHONPATH}}"
"${SIMFORDRONE_PX4_PYTHON}" - "$ROOT" <<'PY'
import json
import os
from pathlib import Path
import sys
import time
import uuid

from px4ctrl.link import MavlinkLink
from px4ctrl.params import load_params
from px4ctrl.vehicle import resolve_role

root = Path(sys.argv[1])
scene_script = str(root / "scripts/01_dual_px4_scene.py")
scenes = []
for proc in Path("/proc").iterdir():
    if not proc.name.isdecimal():
        continue
    try:
        args = (proc / "cmdline").read_bytes().split(b"\0")
        executable = (proc / "exe").resolve().name
        if "python" in executable and any(arg.decode(errors="replace") == scene_script for arg in args):
            scenes.append(int(proc.name))
    except (OSError, PermissionError):
        pass
if len(scenes) != 1:
    sys.exit(f"需要恰好一个运行中的 Isaac 场景，实际找到 {len(scenes)} 个；不会重启任何 PX4")
request_dir = root / "logs/px4_restart" / str(scenes[0])
if not request_dir.is_dir():
    sys.exit("当前场景不支持 PX4 独立重启；请先手动重启场景一次以加载新代码")
if (request_dir / "request.json").exists():
    sys.exit("已有未处理的重启请求；拒绝覆盖")
for proc in Path("/proc").iterdir():
    if not proc.name.isdecimal():
        continue
    try:
        args = (proc / "cmdline").read_bytes().split(b"\0")
        if any(arg in (b"tracking.target_waypoints", b"tracking.run_tracker") for arg in args):
            sys.exit("检测到飞行控制器进程；请先安全结束实验，再重启 PX4")
    except OSError:
        pass

params = load_params(root / "px4ctrl/config/sim.yaml")
for name in ("target", "tracker"):
    link = MavlinkLink(params.link, resolve_role(name), shared_frame=params.shared_frame)
    try:
        link.open(heartbeat_timeout=5.0)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and link.extended_state.recv_time == 0.0:
            link.pump()
            time.sleep(0.05)
        if link.state.armed or not link.is_landed() or link.extended_state.recv_time == 0.0:
            sys.exit(f"{name} 未确认落地且上锁；拒绝重启")
        print(f"{name}: 已落地、上锁", flush=True)
    finally:
        link.close()

nonce = uuid.uuid4().hex
response = request_dir / "response.json"
response.unlink(missing_ok=True)
request = request_dir / "request.json"
temporary = request_dir / f"request-{nonce}.tmp"
temporary.write_text(json.dumps({"nonce": nonce}), encoding="utf-8")
os.replace(temporary, request)
print("已请求场景重启两个 PX4 backend；未发送 ARM/OFFBOARD 命令", flush=True)
deadline = time.monotonic() + 30.0
while time.monotonic() < deadline:
    if response.exists():
        result = json.loads(response.read_text(encoding="utf-8"))
        if result.get("nonce") == nonce:
            response.unlink(missing_ok=True)
            if not result.get("ok"):
                sys.exit(f"场景拒绝重启：{result.get('message')}")
            print(result["message"], flush=True)
            break
    time.sleep(0.2)
else:
    sys.exit("等待场景响应超时；请查看场景终端，勿立即重复重启")

for name in ("target", "tracker"):
    link = MavlinkLink(params.link, resolve_role(name), shared_frame=params.shared_frame)
    try:
        link.open(heartbeat_timeout=30.0)
        print(f"{name}: 重启后心跳已恢复（不代表健康检查通过）", flush=True)
    finally:
        link.close()
PY