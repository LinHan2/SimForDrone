"""只控制 target：执行 local ENU 航点并发布自己的共享系状态。"""

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
from px4ctrl.plotting import PICTURE_DIR, write_response_plot
from px4ctrl.vehicle import resolve_role
from tracking.guidance import TargetState, Vector3
from tracking.state_io import DEFAULT_STATE_HOST, DEFAULT_STATE_PORT, TargetStatePublisher
from tracking.trajectory import TrajectoryCycle, WaypointSegment

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "px4ctrl" / "config" / "sim.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--probe", action="store_true", help="仅检查 target 遥测与共享状态发布，不解锁")
    mode.add_argument("--execute", action="store_false", dest="probe", help="兼容旧命令；现在默认直接执行")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--interactive", action="store_true", help="起飞后从终端非阻塞读取手动航点")
    parser.add_argument(
        "--trajectory",
        choices=("circle", "figure8", "helix"),
        help="执行闭合动态轨迹；不能与 --interactive/--point 同时使用",
    )
    parser.add_argument("--trajectory-cycles", type=int, default=1, help="闭合轨迹执行周期数")
    parser.add_argument("--trajectory-radius", type=float, default=1.0, help="闭合轨迹水平半径（m）")
    parser.add_argument(
        "--trajectory-vertical-amplitude",
        type=float,
        default=0.0,
        help="螺旋轨迹的垂向振幅（m）；circle/figure8 忽略该值",
    )
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
    parser.add_argument(
        "--probe-seconds",
        type=float,
        default=15.0,
        help="dry-run 持续发布 target 状态的时间（s）",
    )
    # 倾角预算必须覆盖完整控制量 Kp·位置误差 + Kv·速度误差 + 加速度前馈，不能只看
    # 轨迹的加速度上限。25° 约为 4.6 m/s²；保守的 0.25/0.25 为反馈项留出余量。
    parser.add_argument("--max-speed", type=float, default=0.25, help="航段最大速度（m/s）")
    parser.add_argument("--max-accel", type=float, default=0.25, help="航段最大加速度（m/s²）")
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
    """解析操作员终端的 ``E N U``、``status`` 或 ``land``。

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


def reference_from_segment(
    segment: WaypointSegment | None,
    segment_started: float,
    now: float,
    fallback: Vector3,
) -> tuple[Vector3, Vector3, Vector3]:
    """返回本周期应下发的 ``(位置, 速度, 加速度)``。

    - 无活动航段时保持 ``fallback``，速度/加速度为零（即稳定悬停）；
    - 有航段时按五次多项式给出参考与前馈，段末自动保持终点
      （`WaypointSegment.sample` 会把超时部分夹到终点并清零前馈）。
    """

    if segment is None:
        return fallback, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return segment.sample(now - segment_started)


def reference_from_trajectory(
    trajectory: TrajectoryCycle, origin: Vector3, elapsed: float
) -> tuple[Vector3, Vector3, Vector3]:
    """将相对闭合轨迹转换为 target 本机 local ENU 的控制参考。"""

    offset, velocity, acceleration = trajectory.sample(elapsed)
    return (
        tuple(origin[index] + offset[index] for index in range(3)),
        velocity,
        acceleration,
    )


def run_interactive(
    link: MavlinkLink,
    fsm: PX4CtrlFSM,
    publisher: TargetStatePublisher,
    home: tuple[float, float, float],
    rate: float,
    tolerance: float,
    max_speed: float,
    max_accel: float,
) -> list[dict[str, float]]:
    """在持续下发设定点的同时接受终端手动航点。

    必须与 `input()` 区分：`input()` 会阻塞线程，而 Offboard 要求设定点流持续
    高于 2 Hz，一旦阻塞超过 PX4 超时就会触发 failsafe。因此这里用 `select`
    以零超时轮询标准输入，并在每次循环照常 `fsm.process()` 持续下发设定点。

    另外：航点不能当阶跃下发。实测把 3 m 阶跃直接交给位置环，会因倾角饱和产生
    ±2 m 的持续振荡（见 READMELIST/progress.md）。因此每次接受航点都用五次多项式
    规划一段平滑轨迹，并**每周期**下发其位置/速度/加速度前馈。
    """

    print("\n手动航点模式：输入 EAST NORTH UP（相对起飞点，m）")
    print("其他命令：status 查看状态，land 安全降落")
    print(f"航段限值：最大速度 {max_speed:.2f} m/s，最大加速度 {max_accel:.2f} m/s²")
    print("waypoint> ", end="", flush=True)
    samples: list[dict[str, float]] = []
    # 初始目标为当前悬停点，保证在用户还没输入前飞机保持静止而非飞向原点。
    waypoint = fsm.hover_pose
    waypoint_index = 0
    # 活动航段与其起始时刻；None 表示已到位、保持悬停。
    segment: WaypointSegment | None = None
    segment_started = 0.0
    # 只在首次进入指令控制时切换状态，避免每输入一个航点都重复打日志。
    command_mode_requested = False

    while True:
        now = time.monotonic()
        # 每周期都下发参考（而不是只在输入航点时下发一次）。
        reference_p, reference_v, reference_a = reference_from_segment(
            segment, segment_started, now, waypoint
        )
        fsm.set_command(
            CommandData(p=reference_p, v=reference_v, a=reference_a, yaw=0.0)
        )
        fsm.process(now)
        publish_state(link, publisher, now)
        error = math.dist(link.odom.p, waypoint)
        samples.append(
            {
                "t": now,
                "waypoint": float(waypoint_index),
                "error": error,
                "actual_e": link.odom.p[0],
                "actual_n": link.odom.p[1],
                "actual_u": link.odom.p[2],
                "reference_e": reference_p[0],
                "reference_n": reference_p[1],
                "reference_u": reference_p[2],
            }
        )

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
                    # 起点取**实测**位置：用旧航点作起点会在未到位时产生轨迹跳变。
                    segment = WaypointSegment(link.odom.p, waypoint, max_speed, max_accel)
                    segment_started = now
                    if not command_mode_requested:
                        fsm.request_command_control()
                        command_mode_requested = True
                    print(
                        f"接受航点 {waypoint_index}：relative ENU={point}, local ENU={waypoint}, "
                        f"航段时长 {segment.duration:.2f}s"
                    )
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
    if (
        args.start_delay < 0.0
        or args.dwell < 0.0
        or args.final_hold < 0.0
        or args.probe_seconds <= 0.0
        or args.tolerance <= 0.0
    ):
        raise ValueError("start-delay/dwell/final-hold 必须非负，probe-seconds/tolerance 必须为正")
    if args.max_speed <= 0.0 or args.max_accel <= 0.0:
        raise ValueError("max-speed 与 max-accel 必须为正")
    if args.trajectory is not None and (args.interactive or args.point):
        raise ValueError("--trajectory 不能与 --interactive 或 --point 同时使用")
    if args.trajectory_cycles <= 0 or args.trajectory_radius <= 0.0:
        raise ValueError("trajectory-cycles 与 trajectory-radius 必须为正")
    if args.trajectory_vertical_amplitude < 0.0:
        raise ValueError("trajectory-vertical-amplitude 不得为负")
    points = args.point or [(0.0, 0.0, 2.0), (2.0, 0.0, 2.0), (0.0, 0.0, 2.0)]
    params = load_params(args.config)
    defaults = task_defaults(params)
    link = MavlinkLink(params.link, resolve_role("target"), shared_frame=params.shared_frame)
    fsm = PX4CtrlFSM(params, LinearControl(params), link, log=lambda message: print(f"[target] {message}"))
    publisher = TargetStatePublisher(args.state_host, args.state_port)
    task_name = "target-trajectory" if args.trajectory is not None else "target-waypoints"
    output_dir = args.output_root / f"{task_name}-{time.strftime('%Y%m%d-%H%M%S')}"
    record: dict[str, object] = {"task": task_name, "points": points, "execute": not args.probe}
    samples: list[dict[str, float]] = []

    try:
        link.open()
        wait_ready(link, fsm, defaults.ready_timeout, print)
        wait_shared_position(link, fsm, defaults.ready_timeout)
        if args.probe:
            print(f"target dry-run：连续发布共享状态 {args.probe_seconds:.1f}s，未发送控制命令")
            deadline = time.monotonic() + args.probe_seconds
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
            samples = run_interactive(
                link, fsm, publisher, home, rate, args.tolerance, args.max_speed, args.max_accel
            )
        elif args.trajectory is not None:
            trajectory = TrajectoryCycle(
                pattern=args.trajectory,
                radius=args.trajectory_radius,
                vertical_amplitude=args.trajectory_vertical_amplitude,
                max_speed=args.max_speed,
                max_accel=args.max_accel,
            )
            record["trajectory"] = {
                "pattern": args.trajectory,
                "cycles": args.trajectory_cycles,
                "radius_m": args.trajectory_radius,
                "vertical_amplitude_m": args.trajectory_vertical_amplitude,
                "cycle_duration_s": trajectory.duration,
            }
            # 起点取实测悬停位置。每周期的首末参考及其一、二阶导数均为零，故周期
            # 拼接不会引入位置、速度或加速度跳变。
            origin = link.odom.p
            fsm.request_command_control()
            started = time.monotonic()
            total_duration = trajectory.duration * args.trajectory_cycles
            next_status = started
            print(
                f"target 动态轨迹：{args.trajectory}，{args.trajectory_cycles} 周期，"
                f"单周期 {trajectory.duration:.2f}s，v≤{args.max_speed:.2f} m/s，"
                f"a≤{args.max_accel:.2f} m/s²"
            )
            while time.monotonic() - started < total_duration:
                now = time.monotonic()
                elapsed = now - started
                cycle_elapsed = elapsed % trajectory.duration
                reference_p, reference_v, reference_a = reference_from_trajectory(
                    trajectory, origin, cycle_elapsed
                )
                fsm.set_command(CommandData(p=reference_p, v=reference_v, a=reference_a, yaw=0.0))
                fsm.process(now)
                publish_state(link, publisher, now)
                error = math.dist(link.odom.p, reference_p)
                samples.append(
                    {
                        "t": now,
                        "waypoint": 0.0,
                        "error": error,
                        "actual_e": link.odom.p[0],
                        "actual_n": link.odom.p[1],
                        "actual_u": link.odom.p[2],
                        "reference_e": reference_p[0],
                        "reference_n": reference_p[1],
                        "reference_u": reference_p[2],
                        "tilt_saturated": float(fsm.controller.debug.tilt_saturated),
                    }
                )
                if now >= next_status:
                    print(
                        f"[target-trajectory] t={elapsed:5.1f}s "
                        f"shared=({link.shared_position[0]:+.2f},{link.shared_position[1]:+.2f},"
                        f"{link.shared_position[2]:+.2f}) ref_local=({reference_p[0]:+.2f},"
                        f"{reference_p[1]:+.2f},{reference_p[2]:+.2f}) err={error:.3f}m",
                        flush=True,
                    )
                    next_status = now + 1.0
                time.sleep(rate)
        else:
            # 只需在首个航段前切一次指令控制状态。
            command_mode_requested = False
            # 下一个状态打印时刻（1 Hz）。
            next_status = time.monotonic()
            for index, point in enumerate(points, start=1):
                waypoint = (home[0] + point[0], home[1] + point[1], home[2] + point[2])
                # 从**实测**当前位置规划一次五次多项式航段，而不是把航点当阶跃：
                # 阶跃会让位置环请求远超倾角上限的加速度，实测会产生 ±2 m 持续振荡。
                segment = WaypointSegment(link.odom.p, waypoint, args.max_speed, args.max_accel)
                segment_started = time.monotonic()
                print(
                    f"航点 {index}/{len(points)}：local ENU={waypoint}，航段时长 "
                    f"{segment.duration:.2f}s（v≤{args.max_speed:.2f} m/s，"
                    f"a≤{args.max_accel:.2f} m/s²）"
                )
                reached_at: float | None = None
                # 截止时间 = 轨迹本身时长 + 充裕裕量，不再用固定 60 s 碰运气。
                deadline = segment_started + segment.duration + 60.0
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    reference_p, reference_v, reference_a = segment.sample(now - segment_started)
                    fsm.set_command(
                        CommandData(p=reference_p, v=reference_v, a=reference_a, yaw=0.0)
                    )
                    if not command_mode_requested:
                        fsm.request_command_control()
                        command_mode_requested = True
                    fsm.process(now)
                    publish_state(link, publisher, now)
                    error = math.dist(link.odom.p, waypoint)
                    samples.append(
                        {
                            "t": now,
                            "waypoint": float(index),
                            "error": error,
                            "actual_e": link.odom.p[0],
                            "actual_n": link.odom.p[1],
                            "actual_u": link.odom.p[2],
                            "reference_e": reference_p[0],
                            "reference_n": reference_p[1],
                            "reference_u": reference_p[2],
                        }
                    )
                    # 同时打印 local 与 shared 两套坐标：local 是控制器与收敛判据所用，
                    # shared 是 tracker 看到的。两者保持固定偏移 ⇒ 真实运动；
                    # 两者发散 ⇒ 问题在估计/坐标系，而非控制。
                    if now >= next_status:
                        local = link.odom.p
                        shared = link.shared_position
                        reference_p_local, _, _ = segment.sample(now - segment_started)
                        print(
                            f"[target-state] local=({local[0]:+.2f},{local[1]:+.2f},{local[2]:+.2f}) "
                            f"shared=({shared[0]:+.2f},{shared[1]:+.2f},{shared[2]:+.2f}) "
                            f"ref_local=({reference_p_local[0]:+.2f},{reference_p_local[1]:+.2f},"
                            f"{reference_p_local[2]:+.2f}) err={error:.3f}m",
                            flush=True,
                        )
                        next_status = now + 1.0
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
        if samples:
            record["reference_error_mean_m"] = sum(sample["error"] for sample in samples) / len(samples)
            record["reference_error_max_m"] = max(sample["error"] for sample in samples)
            record["tilt_saturated_samples"] = sum(
                int(sample.get("tilt_saturated", 0.0)) for sample in samples
            )
        record.update(finish(link, fsm, defaults.landing_timeout, print, defaults.disarm_timeout))
        passed = bool(record.get("on_ground")) and bool(record.get("disarmed"))
        record["passed"] = passed
        record["result"] = "completed"
        print(f"{'PASS' if passed else 'FAIL'}: target-waypoints")
        return 0 if passed else 1
    except KeyboardInterrupt:
        record["result"] = "interrupted"
        print("INTERRUPT: target 正在安全降落", file=sys.stderr)
        if fsm.stream_enabled:
            record.update(finish(link, fsm, defaults.landing_timeout, print, defaults.disarm_timeout))
        return 130
    except Exception as error:
        record["result"] = "failed"
        record["error"] = str(error)
        print(f"FAIL: {error}", file=sys.stderr)
        if fsm.stream_enabled:
            record.update(finish(link, fsm, defaults.landing_timeout, print, defaults.disarm_timeout))
        return 1
    finally:
        if samples and "samples" not in record:
            record["samples"] = samples[:: max(1, len(samples) // 300)]
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "run.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        plot_path = write_response_plot(output_dir, record, picture_dir=PICTURE_DIR)
        if plot_path is not None:
            record["response_plot"] = plot_path.name
            record["picture_plot"] = str(PICTURE_DIR / f"{output_dir.name}.png")
            (output_dir / "run.json").write_text(
                json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            print(f"PLOT: {plot_path}；归档: {record['picture_plot']}")
        publisher.close()
        link.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    raise SystemExit(main())