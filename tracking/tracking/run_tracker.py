"""只控制 tracker：消费另一进程发布的目标状态并执行 V0 相对位置跟踪。"""

from __future__ import annotations

import argparse
import json
import math
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
from tracking.estimation import NoisyTargetSensor, PassthroughEstimator, select_target_state
from tracking.guidance import ObserverState, PositionTrackerV0
from tracking.state_io import DEFAULT_STATE_HOST, DEFAULT_STATE_PORT, TargetStateSubscriber

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "px4ctrl" / "config" / "sim.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--probe", action="store_true", help="仅检查 tracker 遥测与 target 状态流，不解锁")
    mode.add_argument("--execute", action="store_false", dest="probe", help="兼容旧命令；现在默认直接执行")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=30.0, help="跟踪时长（s）；0 表示一直跟踪到中断或目标状态超时")
    parser.add_argument("--offset-east", type=float, default=None)
    parser.add_argument("--offset-north", type=float, default=None)
    parser.add_argument("--offset-up", type=float, default=None)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--state-source", choices=("truth", "estimator"), default="estimator")
    parser.add_argument("--position-noise-std", type=float, default=0.05)
    parser.add_argument("--velocity-noise-std", type=float, default=0.02)
    parser.add_argument("--noise-seed", type=int, default=0)
    parser.add_argument("--state-timeout", type=float, default=0.5)
    parser.add_argument(
        "--settling-seconds",
        type=float,
        default=5.0,
        help="统计动态跟踪误差时排除起飞/初始站位调整的时长（s）",
    )
    parser.add_argument("--status-period", type=float, default=1.0, help="实时状态输出周期（s），0 表示关闭")
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


def wait_tracker_shared(link: MavlinkLink, fsm: PX4CtrlFSM, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        fsm.tick()
        if link.shared_position is not None:
            return
        time.sleep(0.02)
    raise RuntimeError("tracker 共享 ENU 等待超时")


def wait_target_state(subscriber: TargetStateSubscriber, timeout: float, freshness: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            return subscriber.require_fresh(time.monotonic(), freshness)
        except RuntimeError:
            time.sleep(0.02)
    raise RuntimeError("未收到新鲜 target 状态；请先启动 scripts/run_target_waypoints.sh")


def command_from_reference(reference) -> CommandData:
    return CommandData(
        recv_time=time.monotonic(),
        p=reference.p,
        v=reference.v,
        a=reference.a,
        j=reference.j,
        yaw=reference.yaw,
        yaw_rate=reference.yaw_rate,
    )


def main() -> int:
    args = parse_args()
    if (
        args.duration < 0.0
        or args.state_timeout <= 0.0
        or args.status_period < 0.0
        or args.settling_seconds < 0.0
    ):
        raise ValueError("duration/status-period/settling-seconds 不得为负，state-timeout 必须为正")
    params = load_params(args.config)
    defaults = task_defaults(params)
    link = MavlinkLink(params.link, resolve_role("tracker"), shared_frame=params.shared_frame)
    fsm = PX4CtrlFSM(params, LinearControl(params), link, log=lambda message: print(f"[tracker] {message}"))
    subscriber = TargetStateSubscriber(args.state_host, args.state_port)
    output_dir = args.output_root / f"tracker-v0-{time.strftime('%Y%m%d-%H%M%S')}"
    record: dict[str, object] = {"task": "tracker-v0", "execute": not args.probe, "state_source": args.state_source}
    samples: list[dict[str, float]] = []

    try:
        link.open()
        wait_ready(link, fsm, defaults.ready_timeout, print)
        wait_tracker_shared(link, fsm, defaults.ready_timeout)
        target_message = wait_target_state(subscriber, defaults.ready_timeout, args.state_timeout)
        if args.probe:
            print(f"DRY RUN PASS: tracker 与 target 状态流已就绪，target={target_message.p}")
            record["result"] = "dry_run_probe_ok"
            return 0

        enter_offboard(link, fsm, print)
        fsm.request_takeoff(time.monotonic())
        rate = 1.0 / defaults.rate_hz
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            subscriber.require_fresh(now, args.state_timeout)
            fsm.process(now)
            if fsm.state == State.AUTO_HOVER:
                break
            time.sleep(rate)
        else:
            raise RuntimeError("tracker 起飞后 45s 内未进入 AUTO_HOVER")

        target_message = subscriber.require_fresh(time.monotonic(), args.state_timeout)
        # 未指定 offset 时锁存当前真实相对位置：不同轮次的落点会漂移，
        # 写死数值会让第一轮就出现大幅横移（实测曾有 ~4.5 m 与 6 m 两种间距）。
        if args.offset_east is None and args.offset_north is None and args.offset_up is None:
            relative_offset = tuple(
                link.shared_position[index] - target_message.p[index]
                for index in range(3)
            )
            print(f"锁存 tracker 当前相对 offset={relative_offset}")
        else:
            relative_offset = (
                0.0 if args.offset_east is None else args.offset_east,
                0.0 if args.offset_north is None else args.offset_north,
                0.0 if args.offset_up is None else args.offset_up,
            )

        guidance = PositionTrackerV0(relative_offset=relative_offset, yaw=args.yaw)
        sensor = NoisyTargetSensor(args.position_noise_std, args.velocity_noise_std, args.noise_seed)
        estimator = PassthroughEstimator()
        fsm.request_command_control()
        record["relative_offset"] = relative_offset
        started = time.monotonic()
        next_status = started
        # duration == 0 表示持续跟踪直到 Ctrl-C 或 target 状态超时；
        # 手动航点实验用它可以避免 tracker 先于 target 结束而导致误差统计失真。
        while args.duration == 0.0 or time.monotonic() - started < args.duration:
            now = time.monotonic()
            target_message = subscriber.require_fresh(now, args.state_timeout)
            truth = target_message.as_target_state()
            target_state, measurement = select_target_state(args.state_source, truth, now, sensor, estimator)
            reference = guidance.generate(
                target_state,
                ObserverState(shared_p=link.shared_position, local_p=link.odom.p),
            )
            fsm.set_command(command_from_reference(reference))
            output = fsm.process(now)
            desired_shared = tuple(truth.p[index] + relative_offset[index] for index in range(3))
            error = math.dist(link.shared_position, desired_shared)
            samples.append(
                {
                    "t": now - started,
                    "error": error,
                    "measurement_error": math.dist(measurement.p, truth.p) if measurement else 0.0,
                    "thrust": output.thrust,
                    "target_e": truth.p[0],
                    "target_n": truth.p[1],
                    "target_u": truth.p[2],
                    "tracker_e": link.shared_position[0],
                    "tracker_n": link.shared_position[1],
                    "tracker_u": link.shared_position[2],
                    "desired_e": desired_shared[0],
                    "desired_n": desired_shared[1],
                    "desired_u": desired_shared[2],
                    "tilt_saturated": float(fsm.controller.debug.tilt_saturated),
                    "fsm_cmd_ctrl": float(fsm.state == State.CMD_CTRL),
                }
            )
            if args.status_period > 0.0 and now >= next_status:
                print(
                    f"TRACK t={now - started:5.1f}s source={args.state_source} "
                    f"target=({truth.p[0]:+.2f},{truth.p[1]:+.2f},{truth.p[2]:+.2f}) "
                    f"tracker=({link.shared_position[0]:+.2f},{link.shared_position[1]:+.2f},"
                    f"{link.shared_position[2]:+.2f}) desired=({desired_shared[0]:+.2f},"
                    f"{desired_shared[1]:+.2f},{desired_shared[2]:+.2f}) error={error:.3f}m"
                )
                next_status = now + args.status_period
            time.sleep(rate)

        record["samples"] = samples[:: max(1, len(samples) // 300)]
        evaluation_samples = [sample for sample in samples if sample["t"] >= args.settling_seconds]
        if not evaluation_samples:
            evaluation_samples = samples
        record["settling_seconds"] = args.settling_seconds
        record["error_mean_m"] = sum(sample["error"] for sample in evaluation_samples) / len(evaluation_samples)
        record["error_max_m"] = max(sample["error"] for sample in evaluation_samples)
        record["tilt_saturated_samples"] = sum(
            int(sample["tilt_saturated"]) for sample in samples
        )
        record["cmd_ctrl_fraction"] = sum(sample["fsm_cmd_ctrl"] for sample in samples) / len(samples)
        record.update(finish(link, fsm, defaults.landing_timeout, print, defaults.disarm_timeout))
        passed = bool(record.get("on_ground")) and bool(record.get("disarmed"))
        record["passed"] = passed
        record["result"] = "completed"
        print(f"{'PASS' if passed else 'FAIL'}: tracker-v0, mean={record['error_mean_m']:.3f}m, max={record['error_max_m']:.3f}m")
        return 0 if passed else 1
    except KeyboardInterrupt:
        record["result"] = "interrupted"
        print("INTERRUPT: tracker 正在安全降落", file=sys.stderr)
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
        subscriber.close()
        link.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    raise SystemExit(main())