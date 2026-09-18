"""Fly the target through local-ENU waypoints and publish its shared state."""

from __future__ import annotations

import argparse
import json
import math
import select
import sys
import time
from pathlib import Path

from px4ctrl.cli import apply_task_defaults, enter_offboard, finish, wait_ready
from px4ctrl.controller import LinearControl
from px4ctrl.fsm import PX4CtrlFSM, State
from px4ctrl.inputs import CommandData
from px4ctrl.link import MavlinkLink
from px4ctrl.params import load_params
from px4ctrl.vehicle import resolve_role
from tracking.guidance import TargetState
from tracking.state_io import DEFAULT_STATE_HOST, DEFAULT_STATE_PORT, TargetStatePublisher

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "px4ctrl" / "config" / "sim.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="真正控制 target；默认仅探测")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--interactive", action="store_true", help="起飞后从终端非阻塞读取手动航点")
    parser.add_argument(
        "--point",
        action="append",
        nargs=3,
        type=float,
        metavar=("EAST", "NORTH", "UP"),
        help="相对 target 起飞点的 ENU 航点（m），可重复；默认直线往返",
    )
    parser.add_argument("--start-delay", type=float, default=15.0, help="起飞稳定后等待 tracker 的时间（s）")
    parser.add_argument("--dwell", type=float, default=5.0, help="到达每个航点后的停留时间（s）")
    parser.add_argument("--final-hold", type=float, default=15.0, help="末航点完成后继续悬停和发布状态（s）")
    parser.add_argument("--tolerance", type=float, default=0.25, help="航点到达半径（m）")
    parser.add_argument("--state-host", default=DEFAULT_STATE_HOST)
    parser.add_argument("--state-port", type=int, default=DEFAULT_STATE_PORT)
    parser.add_argument("--output-root", type=Path, default=ROOT / "logs" / "tracking")
    return parser.parse_args()


def task_defaults(params) -> argparse.Namespace:
    args = argparse.Namespace(
        rate_hz=None, altitude=None, offset_north=None, offset_east=None,
        hold_seconds=None, ready_timeout=None, landing_timeout=None, disarm_timeout=None,
    )
    apply_task_defaults(args, params)
    return args


def publish_state(link: MavlinkLink, publisher: TargetStatePublisher, now: float) -> None:
    """把 target 当前共享 ENU 位置/速度发给 tracker 进程。

    共享位置尚未就绪时静默跳过：`shared_position` 依赖 GLOBAL_POSITION_INT，
    启动阶段会短暂为 None，此时发 0 值会让 tracker 误以为目标在原点。
    """

    if link.shared_position is not None:
        publisher.publish(TargetState(p=link.shared_position, v=link.odom.v), timestamp=now)


def parse_interactive_command(line: str) -> tuple[str, tuple[float, float, float] | None]:
    """Parse ``E N U``, ``status`` or ``land`` from the operator terminal.

    纯字符串解析，不涉及飞控，因此可以独立单测：输入错误的处理必须在真正
    发送设定点之前完成，避免把无效坐标送给控制器。
    """

    command = line.strip().lower()
    # land/quit/exit 都归为降落；land 失败时上层有兜底 LAND，因此可安全退出循环。
    if command in ("land", "quit", "exit"):
        return "land", None
    if command in ("status", "s"):
        return "status", None
    parts = command.split()
    if len(parts) != 3:
        raise ValueError("请输入三个数字 'EAST NORTH UP'，或 status / land")
    try:
        point = tuple(float(value) for value in parts)
    except ValueError as error:
        raise ValueError("航点必须是三个数字 'EAST NORTH UP'") from error
    return "waypoint", point


def run_interactive(
    link: MavlinkLink,
    fsm: PX4CtrlFSM,
    publisher: TargetStatePublisher,
    home: tuple[float, float, float],
    rate: float,
    tolerance: float,
) -> list[dict[str, float]]:
    """Keep control streaming while accepting manual waypoints from stdin.

    必须与 `input()` 区分：`input()` 会阻塞线程，而 Offboard 要求设定点流持续
    高于 2 Hz，一旦阻塞超过 PX4 超时就会触发 failsafe。因此这里用 `select`
    以零超时轮询标准输入，并在每次循环照常 `fsm.process()` 持续下发设定点。
    """

    print("\n手动航点模式：输入 EAST NORTH UP（相对起飞点，m）")
    print("其他命令：status 查看状态，land 安全降落")
    print("waypoint> ", end="", flush=True)
    samples: list[dict[str, float]] = []
    # 初始目标为当前悬停点，保证在用户还没输入前飞机保持静止而非飞向原点。
    waypoint = fsm.hover_pose
    waypoint_index = 0
    while True:
        now = time.monotonic()
        fsm.process(now)
        publish_state(link, publisher, now)
        error = math.dist(link.odom.p, waypoint)
        samples.append({"t": now, "waypoint": float(waypoint_index), "error": error})

        # 零超时轮询：有输入才读，没有输入不等待，控制循环节奏不受键盘影响。
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if readable:
            line = sys.stdin.readline()
            # 读到 EOF（例如输入被重定向结束）时按安全降落处理，而不是空转。
            if not line:
                print("标准输入已关闭，执行安全降落")
                break
            try:
                command, point = parse_interactive_command(line)
            except ValueError as error_message:
                # 非法输入只提示，不改变当前航点，保证飞机继续稳定悬停。
                print(f"输入错误：{error_message}")
            else:
                if command == "land":
                    break
                if command == "status":
                    print(
                        f"target local={tuple(round(value, 3) for value in link.odom.p)} "
                        f"shared={tuple(round(value, 3) for value in link.shared_position)} "
                        f"waypoint={tuple(round(value, 3) for value in waypoint)} "
                        f"error={error:.3f}m"
                    )
                elif point is not None:
                    waypoint_index += 1
                    # 用户输入的是相对起飞点的偏移，必须叠加 home 才能得到 PX4
                    # local ENU 期望点（控制器在该系下解释位置设定点）。
                    waypoint = tuple(home[index] + point[index] for index in range(3))
                    fsm.set_command(CommandData(p=waypoint, yaw=0.0))
                    fsm.request_command_control()
                    print(f"接受航点 {waypoint_index}：relative ENU={point}, local ENU={waypoint}")
            print("waypoint> ", end="", flush=True)
        time.sleep(rate)
    return samples


def wait_shared_position(link: MavlinkLink, fsm: PX4CtrlFSM, timeout: float) -> None:
    """等待共享 ENU 就绪。

    GLOBAL_POSITION_INT 晚于 LOCAL_POSITION_NED 到达，若不等它就直接发布状态，
    tracker 会在最初若干秒收到 0 值目标而飞向错误方向。
    """

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        # 用 tick 而非 process：此时尚未解锁，只需要维持接收与心跳。
        fsm.tick()
        if link.shared_position is not None:
            return
        time.sleep(0.02)
    raise RuntimeError("target 共享 ENU 等待超时")


def main() -> int:
    args = parse_args()
    if args.start_delay < 0.0 or args.dwell < 0.0 or args.final_hold < 0.0 or args.tolerance <= 0.0:
        raise ValueError("start-delay/dwell/final-hold 必须非负，tolerance 必须为正")
    points = args.point or [(0.0, 0.0, 2.0), (2.0, 0.0, 2.0), (0.0, 0.0, 2.0)]
    params = load_params(args.config)
    defaults = task_defaults(params)
    link = MavlinkLink(params.link, resolve_role("target"), shared_frame=params.shared_frame)
    fsm = PX4CtrlFSM(params, LinearControl(params), link, log=lambda message: print(f"[target] {message}"))
    publisher = TargetStatePublisher(args.state_host, args.state_port)
    output_dir = args.output_root / f"target-waypoints-{time.strftime('%Y%m%d-%H%M%S')}"
    record: dict[str, object] = {"task": "target-waypoints", "points": points, "execute": args.execute}

    try:
        link.open()
        wait_ready(link, fsm, defaults.ready_timeout, print)
        wait_shared_position(link, fsm, defaults.ready_timeout)
        if not args.execute:
            print("target dry-run：连续发布共享状态 5s，未发送控制命令")
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                link.pump()
                publish_state(link, publisher, now)
                time.sleep(1.0 / defaults.rate_hz)
            print("DRY RUN PASS: target 遥测、共享 ENU 和状态发布均正常。")
            record["result"] = "dry_run_probe_ok"
            return 0

        enter_offboard(link, fsm, print)
        home = link.odom.p
        fsm.request_takeoff(time.monotonic())
        rate = 1.0 / defaults.rate_hz
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            fsm.process(now)
            publish_state(link, publisher, now)
            if fsm.state == State.AUTO_HOVER:
                break
            time.sleep(rate)
        else:
            raise RuntimeError("target 起飞后 45s 内未进入 AUTO_HOVER")

        print(f"target 已悬停，等待 tracker {args.start_delay:.1f}s")
        wait_until = time.monotonic() + args.start_delay
        while time.monotonic() < wait_until:
            now = time.monotonic()
            fsm.process(now)
            publish_state(link, publisher, now)
            time.sleep(rate)

        if args.interactive:
            # 交互模式：不做固定航点序列，由操作员随时输入并决定何时 land。
            # 这样 tracker 的跟踪时长不必与 target 脚本时长对齐，避免两进程错位。
            samples = run_interactive(link, fsm, publisher, home, rate, args.tolerance)
        else:
            samples = []
            for index, point in enumerate(points, start=1):
                waypoint = (home[0] + point[0], home[1] + point[1], home[2] + point[2])
                fsm.set_command(CommandData(p=waypoint, yaw=0.0))
                fsm.request_command_control()
                print(f"航点 {index}/{len(points)}：local ENU={waypoint}")
                reached_at: float | None = None
                deadline = time.monotonic() + 60.0
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    fsm.process(now)
                    publish_state(link, publisher, now)
                    error = math.dist(link.odom.p, waypoint)
                    samples.append({"t": now, "waypoint": float(index), "error": error})
                    if error <= args.tolerance:
                        reached_at = now if reached_at is None else reached_at
                        if now - reached_at >= args.dwell:
                            break
                    else:
                        reached_at = None
                    time.sleep(rate)
                else:
                    raise RuntimeError(f"target 未在 60s 内到达航点 {index}，最终误差 {error:.3f}m")

            print(f"末航点保持 {args.final_hold:.1f}s，继续发布 target 状态")
            deadline = time.monotonic() + args.final_hold
            while time.monotonic() < deadline:
                now = time.monotonic()
                fsm.process(now)
                publish_state(link, publisher, now)
                time.sleep(rate)

        record["samples"] = samples[:: max(1, len(samples) // 300)]
        record.update(finish(link, fsm, defaults.landing_timeout, print))
        passed = bool(record.get("on_ground")) and bool(record.get("disarmed"))
        record["passed"] = passed
        record["result"] = "completed"
        print(f"{'PASS' if passed else 'FAIL'}: target-waypoints")
        return 0 if passed else 1
    except KeyboardInterrupt:
        record["result"] = "interrupted"
        print("INTERRUPT: target 正在安全降落", file=sys.stderr)
        if fsm.stream_enabled:
            record.update(finish(link, fsm, defaults.landing_timeout, print))
        return 130
    except Exception as error:
        record["result"] = "failed"
        record["error"] = str(error)
        print(f"FAIL: {error}", file=sys.stderr)
        if fsm.stream_enabled:
            record.update(finish(link, fsm, defaults.landing_timeout, print))
        return 1
    finally:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "run.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        publisher.close()
        link.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    raise SystemExit(main())