"""Run the V0 static-target tracking experiment with two PX4 vehicles."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from px4ctrl.cli import apply_task_defaults, wait_ready
from px4ctrl.controller import LinearControl
from px4ctrl.fsm import PX4CtrlFSM, State
from px4ctrl.inputs import CommandData
from px4ctrl.link import MavlinkLink
from px4ctrl.params import load_params
from px4ctrl.vehicle import resolve_role
from tracking.estimation import NoisyTargetSensor, PassthroughEstimator, select_target_state
from tracking.guidance import ObserverState, PositionTrackerV0, TargetState

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "px4ctrl" / "config" / "sim.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="真正控制两台飞行器；默认仅探测")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=20.0, help="进入跟踪后的持续时间（s）")
    parser.add_argument("--offset-east", type=float, default=None)
    parser.add_argument("--offset-north", type=float, default=None)
    parser.add_argument("--offset-up", type=float, default=None)
    parser.add_argument("--yaw", type=float, default=0.0, help="tracker 固定期望偏航（rad）")
    parser.add_argument(
        "--state-source",
        choices=("truth", "estimator"),
        default="estimator",
        help="制导使用无噪真值，或加噪量测经过估计器接口（默认）",
    )
    parser.add_argument("--position-noise-std", type=float, default=0.05, help="位置量测噪声标准差（m）")
    parser.add_argument("--velocity-noise-std", type=float, default=0.02, help="速度量测噪声标准差（m/s）")
    parser.add_argument("--noise-seed", type=int, default=0, help="噪声随机种子")
    parser.add_argument("--output-root", type=Path, default=ROOT / "logs" / "tracking")
    return parser.parse_args()


def command_from_reference(reference) -> CommandData:
    """Convert the algorithm-layer contract at the px4ctrl boundary."""

    return CommandData(
        recv_time=time.monotonic(),
        p=reference.p,
        v=reference.v,
        a=reference.a,
        j=reference.j,
        yaw=reference.yaw,
        yaw_rate=reference.yaw_rate,
    )


def pump_both(target_fsm: PX4CtrlFSM, tracker_fsm: PX4CtrlFSM) -> None:
    target_fsm.tick()
    tracker_fsm.tick()


def wait_shared_positions(
    target_link: MavlinkLink,
    tracker_link: MavlinkLink,
    timeout: float,
) -> None:
    """Wait for both GLOBAL_POSITION_INT streams to populate shared ENU."""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        target_link.pump()
        tracker_link.pump()
        if target_link.shared_position is not None and tracker_link.shared_position is not None:
            print(
                f"共享 ENU 已就绪：target={target_link.shared_position}, "
                f"tracker={tracker_link.shared_position}"
            )
            return
        time.sleep(0.02)
    raise RuntimeError(
        "共享 ENU 等待超时："
        f"target={'已就绪' if target_link.shared_position is not None else '缺失'}, "
        f"tracker={'已就绪' if tracker_link.shared_position is not None else '缺失'}；"
        "检查 shared_frame 与 GLOBAL_POSITION_INT"
    )


def enter_offboard_both(
    target_link: MavlinkLink,
    target_fsm: PX4CtrlFSM,
    tracker_link: MavlinkLink,
    tracker_fsm: PX4CtrlFSM,
) -> None:
    """Prestream, arm, and enter Offboard without interrupting either stream."""

    target_fsm.request_hover(time.monotonic())
    tracker_fsm.request_hover(time.monotonic())
    target_fsm.enable()
    tracker_fsm.enable()
    print("预热双机设定点流 2.0s")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        target_fsm.process(now)
        tracker_fsm.process(now)
        time.sleep(0.01)

    tick = lambda: pump_both(target_fsm, tracker_fsm)
    print(f"target ARM: {target_link.arm(tick=tick)}")
    print(f"tracker ARM: {tracker_link.arm(tick=tick)}")
    print(f"target OFFBOARD: {target_link.set_offboard(tick=tick)}")
    print(f"tracker OFFBOARD: {tracker_link.set_offboard(tick=tick)}")

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        pump_both(target_fsm, tracker_fsm)
        if target_link.is_offboard() and tracker_link.is_offboard():
            print("双机 OFFBOARD 已确认")
            return
        time.sleep(0.01)
    raise RuntimeError("未能确认双机均进入 Offboard")


def land_both(
    target_link: MavlinkLink,
    target_fsm: PX4CtrlFSM,
    tracker_link: MavlinkLink,
    tracker_fsm: PX4CtrlFSM,
    timeout: float,
) -> dict[str, bool]:
    """Command both vehicles to land, then confirm ground and disarm."""

    result = {"target_on_ground": False, "tracker_on_ground": False, "target_disarmed": False, "tracker_disarmed": False}
    tick = lambda: pump_both(target_fsm, tracker_fsm)
    for name, link in (("target", target_link), ("tracker", tracker_link)):
        try:
            print(f"{name} LAND: {link.land(tick=tick)}")
        except Exception as error:
            print(f"WARN: {name} LAND 失败: {error}", file=sys.stderr)

    target_fsm.disable()
    tracker_fsm.disable()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        target_link.pump()
        tracker_link.pump()
        result["target_on_ground"] = target_link.is_landed()
        result["tracker_on_ground"] = tracker_link.is_landed()
        if result["target_on_ground"] and result["tracker_on_ground"]:
            break
        time.sleep(0.02)

    for name, link in (("target", target_link), ("tracker", tracker_link)):
        try:
            link.disarm()
        except Exception as error:
            print(f"WARN: {name} DISARM 失败: {error}", file=sys.stderr)
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        target_link.pump()
        tracker_link.pump()
        result["target_disarmed"] = not target_link.state.armed
        result["tracker_disarmed"] = not tracker_link.state.armed
        if result["target_disarmed"] and result["tracker_disarmed"]:
            break
        time.sleep(0.02)
    return result


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        raise ValueError("duration 必须为正数")

    params = load_params(args.config)
    task_args = argparse.Namespace(
        rate_hz=None, altitude=None, offset_north=None, offset_east=None,
        hold_seconds=None, ready_timeout=None, landing_timeout=None, disarm_timeout=None,
    )
    apply_task_defaults(task_args, params)
    target_link = MavlinkLink(params.link, resolve_role("target"), shared_frame=params.shared_frame)
    tracker_link = MavlinkLink(params.link, resolve_role("tracker"), shared_frame=params.shared_frame)
    target_fsm = PX4CtrlFSM(params, LinearControl(params), target_link, log=lambda message: print(f"[target] {message}"))
    tracker_fsm = PX4CtrlFSM(params, LinearControl(params), tracker_link, log=lambda message: print(f"[tracker] {message}"))
    output_dir = args.output_root / f"static-v0-{time.strftime('%Y%m%d-%H%M%S')}"
    record: dict[str, object] = {"task": "static-v0", "execute": args.execute, "args": vars(args) | {"config": str(args.config), "output_root": str(args.output_root)}}

    try:
        target_link.open()
        tracker_link.open()
        wait_ready(target_link, target_fsm, task_args.ready_timeout, lambda message: print(f"[target] {message}"))
        wait_ready(tracker_link, tracker_fsm, task_args.ready_timeout, lambda message: print(f"[tracker] {message}"))
        wait_shared_positions(target_link, tracker_link, task_args.ready_timeout)
        if not args.execute:
            print("DRY RUN PASS: 双机遥测与共享 ENU 已就绪，未发送控制命令。")
            record["result"] = "dry_run_probe_ok"
            return 0

        enter_offboard_both(target_link, target_fsm, tracker_link, tracker_fsm)
        now = time.monotonic()
        target_fsm.request_takeoff(now)
        tracker_fsm.request_takeoff(now)
        rate = 1.0 / task_args.rate_hz
        deadline = now + 45.0
        while time.monotonic() < deadline:
            cycle = time.monotonic()
            target_fsm.process(cycle)
            tracker_fsm.process(cycle)
            if target_fsm.state == State.AUTO_HOVER and tracker_fsm.state == State.AUTO_HOVER:
                break
            time.sleep(rate)
        else:
            raise RuntimeError("双机起飞后 45s 内未同时进入 AUTO_HOVER")

        if args.offset_east is None and args.offset_north is None and args.offset_up is None:
            relative_offset = tuple(
                tracker_link.shared_position[index] - target_link.shared_position[index]
                for index in range(3)
            )
            print(f"锁存起飞后的初始相对 offset={relative_offset}")
        else:
            relative_offset = (
                0.0 if args.offset_east is None else args.offset_east,
                0.0 if args.offset_north is None else args.offset_north,
                0.0 if args.offset_up is None else args.offset_up,
            )
        guidance = PositionTrackerV0(relative_offset=relative_offset, yaw=args.yaw)
        record["relative_offset"] = relative_offset
        sensor = NoisyTargetSensor(
            position_std=args.position_noise_std,
            velocity_std=args.velocity_noise_std,
            seed=args.noise_seed,
        )
        estimator = PassthroughEstimator()
        print(
            f"目标状态源={args.state_source}; 位置噪声 σ={args.position_noise_std:.3f} m, "
            f"速度噪声 σ={args.velocity_noise_std:.3f} m/s, seed={args.noise_seed}"
        )
        tracker_fsm.request_command_control()
        samples: list[dict[str, float]] = []
        started = time.monotonic()
        while time.monotonic() - started < args.duration:
            cycle = time.monotonic()
            target_fsm.process(cycle)
            if target_link.shared_position is None or tracker_link.shared_position is None:
                raise RuntimeError("跟踪过程中共享 ENU 丢失")
            truth = TargetState(p=target_link.shared_position, v=target_link.odom.v)
            target_state, measurement = select_target_state(
                args.state_source, truth, cycle, sensor, estimator
            )
            reference = guidance.generate(
                target_state,
                ObserverState(shared_p=tracker_link.shared_position, local_p=tracker_link.odom.p),
            )
            tracker_fsm.set_command(command_from_reference(reference))
            output = tracker_fsm.process(cycle)
            desired_shared = tuple(target_link.shared_position[index] + guidance.relative_offset[index] for index in range(3))
            error = math.dist(tracker_link.shared_position, desired_shared)
            measurement_error = math.dist(measurement.p, truth.p) if measurement is not None else 0.0
            samples.append(
                {
                    "t": cycle - started,
                    "error": error,
                    "measurement_error": measurement_error,
                    "thrust": output.thrust,
                }
            )
            time.sleep(rate)

        record["samples"] = samples[:: max(1, len(samples) // 300)]
        record["error_mean_m"] = sum(sample["error"] for sample in samples) / len(samples)
        record["error_max_m"] = max(sample["error"] for sample in samples)
        record.update(land_both(target_link, target_fsm, tracker_link, tracker_fsm, task_args.landing_timeout))
        passed = all(bool(record[key]) for key in ("target_on_ground", "tracker_on_ground", "target_disarmed", "tracker_disarmed"))
        record["passed"] = passed
        record["result"] = "completed"
        print(f"{'PASS' if passed else 'FAIL'}: static-v0, mean error={record['error_mean_m']:.3f} m, max={record['error_max_m']:.3f} m")
        return 0 if passed else 1
    except KeyboardInterrupt:
        record["result"] = "interrupted"
        print("INTERRUPT: 正在同时降落两机", file=sys.stderr)
        if target_fsm.stream_enabled or tracker_fsm.stream_enabled:
            record.update(land_both(target_link, target_fsm, tracker_link, tracker_fsm, task_args.landing_timeout))
        return 130
    except Exception as error:
        record["result"] = "failed"
        record["error"] = str(error)
        print(f"FAIL: {error}", file=sys.stderr)
        if target_fsm.stream_enabled or tracker_fsm.stream_enabled:
            record.update(land_both(target_link, target_fsm, tracker_link, tracker_fsm, task_args.landing_timeout))
        return 1
    finally:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "run.json").write_text(json.dumps(record, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8")
        target_link.close()
        tracker_link.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    raise SystemExit(main())
