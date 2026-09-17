#!/usr/bin/env python3
"""P2.0 跟随机 PX4 MAVLink 起飞、悬停、降落验证。

默认仅接收 heartbeat，不发送任何控制命令。只有明确给出 ``--execute`` 才会
向 PX4 实例 1 发送 TAKEOFF、ARM 和 LAND 命令。
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil


# Pegasus/PX4 多机端口规则：实例 i 的 Offboard 远端端口为 14540+i。
TRACKER_VEHICLE_ID = 1
TRACKER_MAVLINK_PORT = 14541
TRACKER_SYSTEM_ID = 2


class ValidationError(RuntimeError):
    """MAVLink 连接或命令确认失败时抛出，保证调用者得到非零退出码。"""


def write_log(output_dir: Path, payload: dict[str, Any]) -> None:
    """将每一步命令和 ACK 写为 JSON，便于复现实验与失败诊断。"""

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "mavlink_commands.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def wait_for_ack(
    connection: mavutil.mavfile,
    command: int,
    timeout: float,
) -> dict[str, int]:
    """等待特定 MAVLink command 的 ACK，只接受 PX4 明确的 ACCEPTED。"""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.25)
        if message is None or message.command != command:
            continue
        result = int(message.result)
        ack = {
            "command": int(message.command),
            "result": result,
            "progress": int(getattr(message, "progress", 0)),
            "result_param2": int(getattr(message, "result_param2", 0)),
        }
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise ValidationError(f"命令 {command} 被 PX4 拒绝: {ack}")
        return ack
    raise ValidationError(f"等待命令 {command} 的 ACK 超时 ({timeout:.1f}s)")


def send_command(
    connection: mavutil.mavfile,
    command: int,
    timeout: float,
    *params: float,
) -> dict[str, int]:
    """发送 COMMAND_LONG 并等待 ACK；所有命令只发给 heartbeat 对应的 PX4。"""

    if len(params) != 7:
        raise ValueError("MAVLink COMMAND_LONG 必须包含 7 个参数")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        command,
        0,
        *params,
    )
    return wait_for_ack(connection, command, timeout)


def land_safely(connection: mavutil.mavfile, timeout: float) -> dict[str, int]:
    """请求自动降落；即使用户 Ctrl-C，也优先发送此安全退出命令。"""

    nan = math.nan
    return send_command(
        connection,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        timeout,
        0.0,
        0.0,
        0.0,
        nan,
        nan,
        nan,
        nan,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="显式授权发送 TAKEOFF/ARM/LAND；未提供时只探测 heartbeat。",
    )
    parser.add_argument("--port", type=int, default=TRACKER_MAVLINK_PORT)
    parser.add_argument("--expected-system-id", type=int, default=TRACKER_SYSTEM_ID)
    parser.add_argument("--hold-seconds", type=float, default=8.0)
    parser.add_argument("--ack-timeout", type=float, default=5.0)
    parser.add_argument("--heartbeat-timeout", type=float, default=10.0)
    parser.add_argument(
        "--output-root", type=Path, default=Path("logs/takeoff_validation")
    )
    args = parser.parse_args()
    if args.hold_seconds < 0.0:
        parser.error("--hold-seconds 必须非负")
    return args


def main() -> int:
    args = parse_args()
    output_dir = args.output_root / time.strftime("%Y%m%d-%H%M%S")
    record: dict[str, Any] = {
        "vehicle_role": "tracker_uav_1",
        "vehicle_id": TRACKER_VEHICLE_ID,
        "connection": f"udpin:0.0.0.0:{args.port}",
        "execute": args.execute,
        "events": [],
    }

    connection = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{args.port}", source_system=245, source_component=190
    )
    armed_or_takeoff_requested = False
    try:
        heartbeat = connection.wait_heartbeat(timeout=args.heartbeat_timeout)
        actual_system = int(connection.target_system)
        record["heartbeat"] = {
            "system_id": actual_system,
            "component_id": int(connection.target_component),
            "base_mode": int(heartbeat.base_mode),
            "custom_mode": int(heartbeat.custom_mode),
        }
        print(f"HEARTBEAT: system={actual_system}, component={connection.target_component}")
        if actual_system != args.expected_system_id:
            raise ValidationError(
                f"端口 {args.port} 收到系统 ID {actual_system}，期望 {args.expected_system_id}；拒绝控制错误无人机"
            )

        if not args.execute:
            record["result"] = "dry_run_heartbeat_ok"
            write_log(output_dir, record)
            print("DRY RUN PASS: 未发送 ARM/TAKEOFF/LAND。使用 --execute 才会飞行。")
            return 0

        # 复刻 PX4 commander takeoff 的安全顺序：先接受起飞导航命令，再解锁。
        nan = math.nan
        takeoff_ack = send_command(
            connection,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            args.ack_timeout,
            nan,
            nan,
            nan,
            nan,
            nan,
            nan,
            nan,
        )
        record["events"].append({"name": "takeoff_command", "ack": takeoff_ack})
        armed_or_takeoff_requested = True
        print(f"TAKEOFF accepted: {takeoff_ack}")

        arm_ack = send_command(
            connection,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            args.ack_timeout,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        record["events"].append({"name": "arm_command", "ack": arm_ack})
        print(f"ARM accepted: {arm_ack}; hold {args.hold_seconds:.1f}s")
        time.sleep(args.hold_seconds)

        land_ack = land_safely(connection, args.ack_timeout)
        record["events"].append({"name": "land_command", "ack": land_ack})
        record["result"] = "commands_accepted"
        print(f"LAND accepted: {land_ack}")
        return 0
    except KeyboardInterrupt:
        record["result"] = "interrupted"
        print("INTERRUPT: 正在请求自动降落...", file=sys.stderr)
        if armed_or_takeoff_requested:
            try:
                land_ack = land_safely(connection, args.ack_timeout)
                record["events"].append({"name": "land_after_interrupt", "ack": land_ack})
                print(f"LAND accepted: {land_ack}", file=sys.stderr)
            except Exception as error:
                record["land_error"] = str(error)
                print(f"WARN: 自动降落命令失败: {error}", file=sys.stderr)
        return 130
    except Exception as error:
        record["result"] = "failed"
        record["error"] = str(error)
        print(f"FAIL: {error}", file=sys.stderr)
        if armed_or_takeoff_requested:
            try:
                land_ack = land_safely(connection, args.ack_timeout)
                record["events"].append({"name": "land_after_failure", "ack": land_ack})
                print(f"FAILSAFE LAND accepted: {land_ack}", file=sys.stderr)
            except Exception as land_error:
                record["land_error"] = str(land_error)
                print(f"WARN: 自动降落命令失败: {land_error}", file=sys.stderr)
        return 1
    finally:
        write_log(output_dir, record)
        connection.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    raise SystemExit(main())
