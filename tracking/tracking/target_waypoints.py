"""只控制 target：执行 local ENU 航点并发布自己的共享系状态。"""

from __future__ import annotations

import argparse
import json
import math
import queue
import select
import sys
import threading
import time
from pathlib import Path

from px4ctrl.cli import apply_task_defaults, enter_offboard, finish, wait_ready
from px4ctrl.controller import LinearControl
from px4ctrl.fsm import PX4CtrlFSM, State
from px4ctrl.inputs import CommandData
from px4ctrl.link import MavlinkLink
from px4ctrl.params import load_params
from px4ctrl.vehicle import resolve_role
from tracking.guidance import TargetState, Vector3
from tracking.state_io import DEFAULT_STATE_HOST, DEFAULT_STATE_PORT, TargetStatePublisher
from tracking.trajectory import WaypointSegment

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "px4ctrl" / "config" / "sim.yaml"


def put_latest(messages: queue.Queue[str], message: str) -> None:
    """非阻塞写入最新诊断；输出端拥塞时丢弃旧诊断而不拖慢控制循环。"""

    try:
        messages.put_nowait(message)
        return
    except queue.Full:
        pass
    try:
        messages.get_nowait()
    except queue.Empty:
        pass
    try:
        messages.put_nowait(message)
    except queue.Full:
        # 消费线程恰好抢先取走旧项时，另一个生产者也可能已填满；诊断可丢，设定点不可等。
        pass


class DeferredDiagnostics:
    """把慢终端输出移出控制线程的有界诊断通道。"""

    def __init__(self) -> None:
        self._messages: queue.Queue[str] = queue.Queue(maxsize=1)
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._drain, name="target-diagnostics", daemon=True)
        self._thread.start()

    def publish(self, message: str) -> None:
        put_latest(self._messages, message)

    def close(self) -> None:
        self._closed.set()
        self._thread.join(timeout=0.1)

    def _drain(self) -> None:
        while not self._closed.is_set():
            try:
                message = self._messages.get(timeout=0.1)
            except queue.Empty:
                continue
            print(message, flush=True)


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
    # 航段限值要显著低于倾角饱和对应的水平加速度（约 g·tan(25°) ≈ 4.6 m/s²），
    # 否则位置环仍会饱和并振荡；0.8/1.0 已在实飞中验证不会引起振荡。
    parser.add_argument("--max-speed", type=float, default=0.8, help="航段最大速度（m/s）")
    parser.add_argument("--max-accel", type=float, default=1.0, help="航段最大加速度（m/s²）")
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
    if args.start_delay < 0.0 or args.dwell < 0.0 or args.final_hold < 0.0 or args.tolerance <= 0.0:
        raise ValueError("start-delay/dwell/final-hold 必须非负，tolerance 必须为正")
    if args.max_speed <= 0.0 or args.max_accel <= 0.0:
        raise ValueError("max-speed 与 max-accel 必须为正")
    points = args.point or [(0.0, 0.0, 2.0), (2.0, 0.0, 2.0), (0.0, 0.0, 2.0)]
    params = load_params(args.config)
    defaults = task_defaults(params)
    link = MavlinkLink(params.link, resolve_role("target"), shared_frame=params.shared_frame)
    fsm = PX4CtrlFSM(params, LinearControl(params), link, log=lambda message: print(f"[target] {message}"))
    publisher = TargetStatePublisher(args.state_host, args.state_port)
    diagnostics = DeferredDiagnostics()
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
            samples = run_interactive(
                link, fsm, publisher, home, rate, args.tolerance, args.max_speed, args.max_accel
            )
        else:
            samples = []
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
                    samples.append({"t": now, "waypoint": float(index), "error": error})
                    # 同时打印 local 与 shared 两套坐标：local 是控制器与收敛判据所用，
                    # shared 是 tracker 看到的。两者保持固定偏移 ⇒ 真实运动；
                    # 两者发散 ⇒ 问题在估计/坐标系，而非控制。
                    if now >= next_status:
                        local = link.odom.p
                        shared = link.shared_position
                        reference_p_local, _, _ = segment.sample(now - segment_started)
                        diagnostics.publish(
                            f"[target-state] local=({local[0]:+.2f},{local[1]:+.2f},{local[2]:+.2f}) "
                            f"shared=({shared[0]:+.2f},{shared[1]:+.2f},{shared[2]:+.2f}) "
                            f"ref_local=({reference_p_local[0]:+.2f},{reference_p_local[1]:+.2f},"
                            f"{reference_p_local[2]:+.2f}) err={error:.3f}m"
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
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "run.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        diagnostics.close()
        publisher.close()
        link.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    raise SystemExit(main())