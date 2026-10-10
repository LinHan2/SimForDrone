import argparse
import math
import os
from pathlib import Path
import queue
import shlex
import signal
import subprocess
import sys
import threading
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]


def validate_commands(commands):
    sys.path[:0] = [str(ROOT / "tracking"), str(ROOT / "px4ctrl")]
    from tracking.target_waypoints import parse_args as target_args
    from tracking.run_tracker import parse_args as tracker_args

    original = sys.argv
    try:
        for role, parser in (("target", target_args), ("tracker", tracker_args)):
            sys.argv = commands[role]
            parser()
    finally:
        sys.argv = original


def command_arguments(settings):
    if not isinstance(settings, dict):
        raise ValueError("参数段必须是 YAML mapping")
    arguments = []
    for name, value in settings.items():
        if not isinstance(name, str) or not name.replace("_", "").isalnum():
            raise ValueError("参数名只能包含字母、数字和下划线")
        if value is None or value is False:
            continue
        arguments.append("--" + name.replace("_", "-"))
        if value is not True:
            if not isinstance(value, (str, int, float)):
                raise ValueError(f"不支持的参数值: {name}")
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"参数必须为有限值: {name}")
            arguments.append(str(value))
    return arguments


def load_commands(path, gate):
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or set(config) not in ({"startup", "target", "tracker"}, {"startup", "scene", "target", "tracker"}):
        raise ValueError("配置必须包含 startup、target、tracker 三段；旧 scene 段忽略")
    startup = config["startup"]
    if not isinstance(startup, dict) or set(startup) != {"ready_timeout_s", "landing_timeout_s"}:
        raise ValueError("startup 需要 ready_timeout_s 和 landing_timeout_s")
    for value in startup.values():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("启动和降落等待时间必须为有限正数")
    target = dict(config["target"])
    tracker = dict(config["tracker"])
    if tracker.get("duration") != 0 or tracker.get("status_period", 0) <= 0:
        raise ValueError("顺序编排要求 tracker.duration=0 且 status_period>0")
    if {"execute", "probe", "start_gate_file", "start_gate_timeout", "output_root"} & target.keys():
        raise ValueError("target 执行模式、放行文件和输出目录由编排器管理")
    if {"execute", "probe", "output_root"} & tracker.keys():
        raise ValueError("tracker 执行模式和输出目录由编排器管理")
    target.update(start_gate_file=str(gate), start_gate_timeout=startup["ready_timeout_s"])
    output = str(gate.parent / "tracking")
    commands = {
        "target": [str(ROOT / "scripts/run_target_waypoints.sh"), "--execute",
                   "--output-root", output] + command_arguments(target),
        "tracker": [str(ROOT / "scripts/run_tracker.sh"), "--execute",
                    "--output-root", output] + command_arguments(tracker),
    }
    if tracker.get("pixel_velocity_mode") == "predictive" or (
            tracker.get("pixel_velocity_mode") is None and tracker.get("pixel_velocity_enabled")):
        commands["pixel_bridge"] = [str(ROOT / "scripts/run_image_pixel_bridge.sh")]
    return config, commands


def main():
    parser = argparse.ArgumentParser(description="按 YAML 顺序启动 target 和 tracker；场景须单独启动")
    parser.add_argument("--config", type=Path, default=ROOT / "tracking/config/experiment.yaml")
    parser.add_argument("--dry-run", action="store_true", help="只打印配置与命令，不启动任何进程")
    parser.add_argument("--shadow-ekf", action="store_true", help="并行运行只读 RGB-D 二维切空间观测器")
    args = parser.parse_args()
    run_dir = ROOT / "logs/experiments" / (time.strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}")
    gate = run_dir / "trajectory.release"
    config, commands = load_commands(args.config, gate)
    if args.shadow_ekf:
        commands["shadow"] = [str(ROOT / "scripts/run_shadow_relative_ekf.sh"),
                              "--duration", "3600", "--max-range-m", "15",
                              "--output-root", str(run_dir / "shadow")]
    validate_commands(commands)
    for role, command in commands.items():
        print(f"{role}: {shlex.join(command)}", flush=True)
    if args.dry_run:
        return 0
    existing = subprocess.run(["pgrep", "-af", "[t]racking.run_tracker|[t]racking.target_waypoints"], capture_output=True, text=True)
    if existing.returncode == 0:
        raise RuntimeError("已有 target/tracker 控制进程，请先自行停止旧回合：\n" + existing.stdout)
    if existing.returncode != 1:
        raise RuntimeError("无法检查已有进程，取消启动")
    run_dir.mkdir(parents=True)
    (run_dir / "config.yaml").write_text(args.config.read_text(encoding="utf-8"), encoding="utf-8")
    print(f"EXPERIMENT: {run_dir}", flush=True)
    events = queue.Queue()
    processes = {}
    readers = []

    def start(role):
        process = subprocess.Popen(commands[role], cwd=ROOT, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1,
                                   start_new_session=True, env={**os.environ, "PYTHONUNBUFFERED": "1"})
        processes[role] = process

        def relay():
            with (run_dir / f"{role}.log").open("w", encoding="utf-8") as stream:
                for line in process.stdout:
                    stream.write(line)
                    stream.flush()
                    print(f"[{role}] {line.rstrip()}", flush=True)
                    events.put((role, line))
            process.stdout.close()

        reader = threading.Thread(target=relay, daemon=True)
        reader.start()
        readers.append(reader)

    def wait_for(role, text, count=1):
        deadline = time.monotonic() + config["startup"]["ready_timeout_s"]
        observed = 0
        while observed < count:
            if any(process.poll() is not None for process in processes.values()):
                raise RuntimeError("启动过程中有进程退出，请检查本轮日志")
            if time.monotonic() >= deadline:
                raise RuntimeError(f"等待 {role}: {text} 超时")
            try:
                source, line = events.get(timeout=0.2)
            except queue.Empty:
                continue
            if source == role and text in line:
                observed += 1

    result = 1
    try:
        if "pixel_bridge" in commands:
            start("pixel_bridge")
        if "shadow" in commands:
            start("shadow")
            wait_for("shadow", "SHADOW READY")
        start("target")
        wait_for("target", "target 已悬停")
        start("tracker")
        wait_for("tracker", "TRACK t=")
        gate.touch()
        print("目标航迹已放行；等待两机完成并输出同轮图表", flush=True)
        while processes["target"].poll() is None or processes["tracker"].poll() is None:
            if "pixel_bridge" in processes and processes["pixel_bridge"].poll() is not None:
                raise RuntimeError("RGB-D 像素桥提前退出，停止本轮控制")
            for role in ("target", "tracker"):
                code = processes[role].poll()
                if code is not None and code != 0:
                    raise RuntimeError(f"{role} 失败，退出码 {code}")
            if processes["tracker"].poll() == 0 and processes["target"].poll() is None:
                raise RuntimeError("tracker 先于 target 结束，请使用 duration: 0")
            threading.Event().wait(0.2)
        if any(processes[role].poll() != 0 for role in ("target", "tracker")):
            raise RuntimeError("控制进程未正常结束，请检查本轮日志")
        result = 0
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"停止本轮: {error}；先等待控制进程安全收尾", flush=True)
    finally:
        for role in ("tracker", "target", "pixel_bridge", "shadow"):
            process = processes.get(role)
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for role in ("tracker", "target", "pixel_bridge", "shadow"):
            process = processes.get(role)
            if process is not None:
                try:
                    process.wait(timeout=config["startup"]["landing_timeout_s"])
                except subprocess.TimeoutExpired:
                    print(f"{role} 尚未退出，保留场景和进程，请人工检查。日志: {run_dir}", flush=True)
                    return 1
        if any(processes[role].returncode != 0 for role in ("target", "tracker") if role in processes):
            print(f"控制进程异常退出，无法确认安全降落；保留场景，请人工检查。日志: {run_dir}", flush=True)
            return 1
        for reader in readers:
            reader.join(timeout=2)
        print(f"本轮日志与图表: {run_dir}", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())