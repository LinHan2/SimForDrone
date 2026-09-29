"""只控制 tracker：消费另一进程发布的目标状态并执行 V0 相对位置跟踪。"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

from px4ctrl.cli import apply_task_defaults, enter_offboard, finish, wait_ready
from px4ctrl.controller import LinearControl
from px4ctrl.fsm import PX4CtrlFSM, State
from px4ctrl.inputs import CommandData, yaw_from_quaternion
from px4ctrl.link import MavlinkLink
from px4ctrl.params import load_params
from px4ctrl.plotting import PICTURE_DIR, write_response_plot, write_tracker_estimation_plots
from px4ctrl.vehicle import resolve_role
from tracking.estimation import (
    FreshTargetTimestampGate,
    NoisyTargetSensor,
    ObserverKinematics,
    PassthroughEstimator,
    RelativeEkfTargetEstimator,
    TargetMeasurement,
    observer_world_acceleration,
    select_target_state,
)
from tracking.guidance import ObserverState, PositionTrackerV0, follow_offset, slew_yaw
from tracking.relative_ekf import RelativeTargetEKF
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
    parser.add_argument("--follow-distance", type=float, default=None, help="沿初始视线缓慢接近的水平跟随距离（m）")
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--max-tilt-deg", type=float, default=20.0, help="tracker 组合倾角上限（度）")
    parser.add_argument("--max-yaw-rate", type=float, default=0.35, help="偏航参考最大变化速度（rad/s）")
    parser.add_argument(
        "--state-source",
        choices=("truth", "estimator", "relative-ekf"),
        default="estimator",
        help="truth 为真值基线；estimator 为带噪透传；relative-ekf 用位置量测和姿态约束估计",
    )
    parser.add_argument("--position-noise-std", type=float, default=0.05)
    parser.add_argument("--velocity-noise-std", type=float, default=0.02)
    parser.add_argument("--noise-seed", type=int, default=0)
    parser.add_argument(
        "--relative-measurement-position-std",
        type=float,
        default=0.0,
        help="relative-ekf 的合成 RGB-D 位置量测标准差（m）；0 保持共享状态基线",
    )
    parser.add_argument(
        "--relative-measurement-dropout-probability",
        type=float,
        default=0.0,
        help="relative-ekf 合成 RGB-D 量测丢失概率",
    )
    parser.add_argument(
        "--relative-ekf-position-std",
        type=float,
        default=0.102,
        help="RGB-D 相对位置量测标准差（m）；默认取 Airsim2box R_bearingbox",
    )
    parser.add_argument(
        "--relative-ekf-attitude-constraint-std",
        type=float,
        default=0.03,
        help="目标姿态-加速度约束标准差（m/s²）；默认取 Airsim2box R_attitude",
    )
    parser.add_argument(
        "--relative-ekf-position-process-std",
        type=float,
        default=0.002,
        help="位置块离散过程噪声标准差（m）；默认 sqrt(Q_dp)=sqrt(4e-6)",
    )
    parser.add_argument(
        "--relative-ekf-velocity-process-std",
        type=float,
        default=0.1095445115,
        help="论文十维模型速度块离散过程噪声标准差（m/s）",
    )
    parser.add_argument(
        "--relative-ekf-acceleration-process-std",
        type=float,
        default=0.2449489743,
        help="论文十维模型加速度块离散过程噪声标准差（m/s²）",
    )
    parser.add_argument(
        "--relative-ekf-scale-process-std",
        type=float,
        default=0.0948683298,
        help="论文十维模型目标尺寸尺度 alpha 的离散过程噪声标准差（m）",
    )
    parser.add_argument(
        "--relative-ekf-fading",
        type=float,
        default=1.001,
        help="协方差遗忘因子；默认取 Airsim2box 值",
    )
    parser.add_argument(
        "--relative-ekf-warmup-seconds",
        type=float,
        default=8.0,
        help="relative-ekf 进入 CMD_CTRL 前在 AUTO_HOVER 中消费新量测的预热时长（s）",
    )
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


def relative_ekf_target_state(
    target_message,
    link: MavlinkLink,
    estimator: RelativeEkfTargetEstimator,
    measurement: TargetMeasurement | None = None,
):
    """由目标量测/姿态与观测机 IMU 构造 EKF 目标状态，不让真值速度进入制导。"""

    observer = ObserverKinematics(
        p=link.shared_position,
        v=link.odom.v,
        a=observer_world_acceleration(link.imu.q, link.imu.acc),
    )
    return estimator.update(
        measurement or TargetMeasurement(target_message.timestamp, target_message.p, target_message.v),
        target_message.q,
        observer,
    )


def make_relative_ekf(args: argparse.Namespace) -> RelativeEkfTargetEstimator:
    """从显式 CLI 参数创建滤波器，避免 Q/R 隐藏在代码默认值中。"""

    return RelativeEkfTargetEstimator(
        RelativeTargetEKF(
            position_std=args.relative_ekf_position_std,
            attitude_constraint_std=args.relative_ekf_attitude_constraint_std,
            position_process_std=args.relative_ekf_position_process_std,
            velocity_process_std=args.relative_ekf_velocity_process_std,
            acceleration_process_std=args.relative_ekf_acceleration_process_std,
            scale_process_std=args.relative_ekf_scale_process_std,
            fading=args.relative_ekf_fading,
        )
    )


def terminal_error(result: str) -> RuntimeError:
    return RuntimeError(f"target 任务以 {result} 终止")


def main() -> int:
    args = parse_args()
    if (
        args.duration < 0.0
        or args.state_timeout <= 0.0
        or args.status_period < 0.0
        or args.settling_seconds < 0.0
        or args.relative_measurement_position_std < 0.0
        or args.relative_ekf_position_std <= 0.0
        or args.relative_ekf_attitude_constraint_std <= 0.0
        or args.relative_ekf_position_process_std <= 0.0
        or args.relative_ekf_velocity_process_std <= 0.0
        or args.relative_ekf_acceleration_process_std <= 0.0
        or args.relative_ekf_scale_process_std <= 0.0
        or args.relative_ekf_fading < 1.0
        or args.relative_ekf_warmup_seconds < 0.0
        or not 0.0 < args.max_tilt_deg < 45.0
        or args.max_yaw_rate <= 0.0
        or (args.follow_distance is not None and (not math.isfinite(args.follow_distance) or args.follow_distance <= 0.0))
        or (args.follow_distance is not None and any(value is not None for value in (args.offset_east, args.offset_north, args.offset_up)))
        or not 0.0 <= args.relative_measurement_dropout_probability <= 1.0
    ):
        raise ValueError(
            "时长/噪声标准差/预热时长不得为负，EKF 量测与过程噪声标准差必须为正，"
            "遗忘因子不得小于 1，丢帧概率必须在 [0, 1]"
        )
    params = replace(load_params(args.config), max_angle=args.max_tilt_deg)
    defaults = task_defaults(params)
    link = MavlinkLink(params.link, resolve_role("tracker"), shared_frame=params.shared_frame)
    fsm = PX4CtrlFSM(params, LinearControl(params), link, log=lambda message: print(f"[tracker] {message}"))
    subscriber = TargetStateSubscriber(args.state_host, args.state_port)
    output_dir = args.output_root / f"tracker-v0-{time.strftime('%Y%m%d-%H%M%S')}"
    record: dict[str, object] = {"task": "tracker-v0", "execute": not args.probe, "state_source": args.state_source, "max_tilt_deg": args.max_tilt_deg, "max_yaw_rate": args.max_yaw_rate}
    samples: list[dict[str, float]] = []
    # 控制周期可重复使用 UDP 缓存帧；真值曲线和有限差分必须只使用这条唯一报文流。
    target_samples: list[dict[str, float]] = []
    relative_estimator = make_relative_ekf(args)
    relative_sensor = NoisyTargetSensor(
        position_std=args.relative_measurement_position_std,
        velocity_std=0.0,
        seed=args.noise_seed,
        dropout_probability=args.relative_measurement_dropout_probability,
    )
    record["relative_ekf_tuning"] = relative_estimator.tuning
    record["relative_measurement_model"] = {
        "kind": "synthetic_rgbd_position_proxy",
        "position_std_m": args.relative_measurement_position_std,
        "dropout_probability": args.relative_measurement_dropout_probability,
        "seed": args.noise_seed,
    }
    record["relative_ekf_warmup_seconds"] = args.relative_ekf_warmup_seconds

    try:
        link.open()
        wait_ready(link, fsm, defaults.ready_timeout, print)
        wait_tracker_shared(link, fsm, defaults.ready_timeout)
        target_message = wait_target_state(subscriber, defaults.ready_timeout, args.state_timeout)
        if args.probe:
            if args.state_source == "relative-ekf":
                estimate = relative_ekf_target_state(
                    target_message,
                    link,
                    relative_estimator,
                )
                print(
                    "relative-ekf probe: "
                    f"p={estimate.p}, v={estimate.v}, a={estimate.a}"
                )
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
        # 状态流的发布频率可能低于控制频率。同一时间戳只允许推进 EKF 一次：
        # 重复时间戳会令 dt=0，破坏离散模型假设并触发异常。未推进时保持上一帧估计。
        target_gate = FreshTargetTimestampGate()
        target_state = None
        measurement: TargetMeasurement | None = None
        if args.state_source == "relative-ekf":
            warmup_deadline = time.monotonic() + args.relative_ekf_warmup_seconds
            print(f"relative-ekf 在 AUTO_HOVER 预热 {args.relative_ekf_warmup_seconds:.1f}s")
            while time.monotonic() < warmup_deadline:
                now = time.monotonic()
                fsm.process(now)
                target_message = subscriber.require_fresh(now, args.state_timeout)
                if target_gate.accept(target_message.timestamp):
                    truth = target_message.as_target_state()
                    measurement = relative_sensor.measure(truth, target_message.timestamp)
                    if measurement is not None:
                        target_state = relative_ekf_target_state(
                            target_message, link, relative_estimator, measurement
                        )
                time.sleep(rate)
            if target_state is None:
                raise RuntimeError("relative-ekf 预热期间未收到有效位置量测")
            print(
                f"relative-ekf 预热完成：p={target_state.p}, v={target_state.v}, a={target_state.a}"
            )

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
        if args.follow_distance is not None:
            follow_offset(relative_offset, args.follow_distance, 0.0)
        relative_offset_start = relative_offset
        sensor = NoisyTargetSensor(args.position_noise_std, args.velocity_noise_std, args.noise_seed)
        estimator = PassthroughEstimator()
        fsm.request_command_control()
        record["relative_offset"] = relative_offset
        started = time.monotonic()
        last_yaw_at = started
        commanded_yaw = yaw_from_quaternion(link.odom.q)
        next_status = started
        follow_ready = False
        terminal_result: str | None = None
        previous_truth_timestamp: float | None = None
        # duration == 0 表示持续跟踪直到 Ctrl-C 或 target 状态超时；
        # 手动航点实验用它可以避免 tracker 先于 target 结束而导致误差统计失真。
        while args.duration == 0.0 or time.monotonic() - started < args.duration:
            now = time.monotonic()
            subscriber.poll()
            if subscriber.terminal is not None:
                terminal_result = subscriber.terminal.result
                if terminal_result == "completed":
                    print("target 已正常完成，tracker 开始安全收尾")
                    break
                raise terminal_error(terminal_result)
            target_message = subscriber.require_fresh(now, args.state_timeout)
            truth = target_message.as_target_state()
            target_is_new = (
                previous_truth_timestamp is None
                or target_message.timestamp > previous_truth_timestamp
            )
            if args.state_source == "relative-ekf":
                # 只有新时间戳才生成一次量测并推进 EKF；旧时间戳是缓存重放，
                # 不得重复量测更新，否则 dt=0 破坏时间假设。
                if target_gate.accept(target_message.timestamp):
                    measurement = relative_sensor.measure(truth, target_message.timestamp)
                    if measurement is not None:
                        target_state = relative_ekf_target_state(
                            target_message,
                            link,
                            relative_estimator,
                            measurement,
                        )
                if target_state is None:
                    raise RuntimeError("relative-ekf 尚无有效目标估计")
            else:
                target_state, measurement = select_target_state(
                    args.state_source, truth, now, sensor, estimator
                )
            if target_is_new:
                previous_truth_timestamp = target_message.timestamp
            if args.follow_distance is not None:
                relative_offset = follow_offset(relative_offset_start, args.follow_distance, now - started)
                guidance = PositionTrackerV0(relative_offset=relative_offset, yaw=args.yaw)
            reference = guidance.generate(
                target_state,
                ObserverState(shared_p=link.shared_position, local_p=link.odom.p),
            )
            commanded_yaw = slew_yaw(commanded_yaw, reference.yaw, args.max_yaw_rate * (now - last_yaw_at))
            last_yaw_at = now
            reference = replace(reference, yaw=commanded_yaw)
            fsm.set_command(command_from_reference(reference))
            output = fsm.process(now)
            # 绘图与误差都使用实际送入控制器的估计目标，不用真值掩盖估计误差。
            desired_shared = tuple(target_state.p[index] + relative_offset[index] for index in range(3))
            error = math.dist(link.shared_position, desired_shared)
            horizontal_distance = math.hypot(truth.p[0] - link.shared_position[0], truth.p[1] - link.shared_position[1])
            if (args.follow_distance is not None and not follow_ready
                    and abs(horizontal_distance - args.follow_distance) <= 0.5):
                print("FOLLOW DISTANCE READY", flush=True)
                follow_ready = True
            if target_is_new:
                truth_acceleration = target_message.reference_a or (float("nan"),) * 3
                # 时间轴来自发布端而非 tracker 控制节拍，因此相邻点必对应不同目标报文。
                target_samples.append(
                    {
                        "t": target_message.timestamp - started,
                        "target_sequence": float(target_message.sequence),
                        "target_e": truth.p[0],
                        "target_n": truth.p[1],
                        "target_u": truth.p[2],
                        "target_ve": truth.v[0],
                        "target_vn": truth.v[1],
                        "target_vu": truth.v[2],
                        "target_ae": truth_acceleration[0],
                        "target_an": truth_acceleration[1],
                        "target_au": truth_acceleration[2],
                        "measurement_e": measurement.p[0] if measurement else float("nan"),
                        "measurement_n": measurement.p[1] if measurement else float("nan"),
                        "measurement_u": measurement.p[2] if measurement else float("nan"),
                        "estimate_e": target_state.p[0],
                        "estimate_n": target_state.p[1],
                        "estimate_u": target_state.p[2],
                        "estimate_ve": target_state.v[0],
                        "estimate_vn": target_state.v[1],
                        "estimate_vu": target_state.v[2],
                        "estimate_ae": target_state.a[0],
                        "estimate_an": target_state.a[1],
                        "estimate_au": target_state.a[2],
                    }
                )
            samples.append(
                {
                    "t": now - started,
                    "target_timestamp_s": target_message.timestamp - started,
                    "target_age_s": now - target_message.timestamp,
                    "target_sequence": float(target_message.sequence),
                    "target_is_new": float(target_is_new),
                    "error": error,
                    "measurement_error": math.dist(measurement.p, truth.p) if measurement else 0.0,
                    "estimate_error": math.dist(target_state.p, truth.p),
                    "thrust": output.thrust,
                    "target_e": truth.p[0],
                    "target_n": truth.p[1],
                    "target_u": truth.p[2],
                    "target_ve": truth.v[0],
                    "target_vn": truth.v[1],
                    "target_vu": truth.v[2],
                    "target_ae": truth_acceleration[0],
                    "target_an": truth_acceleration[1],
                    "target_au": truth_acceleration[2],
                    "measurement_e": measurement.p[0] if measurement else float("nan"),
                    "measurement_n": measurement.p[1] if measurement else float("nan"),
                    "measurement_u": measurement.p[2] if measurement else float("nan"),
                    "estimate_e": target_state.p[0],
                    "estimate_n": target_state.p[1],
                    "estimate_u": target_state.p[2],
                    "estimate_ve": target_state.v[0],
                    "estimate_vn": target_state.v[1],
                    "estimate_vu": target_state.v[2],
                    "estimate_ae": target_state.a[0],
                    "estimate_an": target_state.a[1],
                    "estimate_au": target_state.a[2],
                    "tracker_e": link.shared_position[0],
                    "tracker_n": link.shared_position[1],
                    "tracker_u": link.shared_position[2],
                    "desired_e": desired_shared[0],
                    "desired_n": desired_shared[1],
                    "desired_u": desired_shared[2],
                    "horizontal_distance": horizontal_distance,
                    "desired_distance": math.hypot(relative_offset[0], relative_offset[1]),
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
                    f"{desired_shared[1]:+.2f},{desired_shared[2]:+.2f}) error={error:.3f}m "
                    f"distance={horizontal_distance:.2f}m desired={math.hypot(*relative_offset[:2]):.2f}m",
                    flush=True,
                )
                next_status = now + args.status_period
            time.sleep(rate)

        record["samples"] = samples[:: max(1, len(samples) // 300)]
        record["target_samples"] = target_samples
        record["target_sample_count"] = len(target_samples)
        record["target_hold_samples"] = len(samples) - len(target_samples)
        record["target_stamp_duplicates"] = target_gate.duplicate_count
        record["target_stamp_regressions"] = target_gate.regression_count
        record["target_acceleration_truth"] = (
            "trajectory_reference"
            if any(math.isfinite(sample["target_ae"]) for sample in target_samples)
            else "unavailable"
        )
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
        if terminal_result is not None:
            record["target_terminal_result"] = terminal_result
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
        if target_samples and "target_samples" not in record:
            record["target_samples"] = target_samples
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "run.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        estimation_plots = write_tracker_estimation_plots(output_dir, record)
        if estimation_plots:
            record["estimation_plots"] = [
                path.relative_to(output_dir).as_posix() for path in estimation_plots
            ]
        plot_path = write_response_plot(output_dir, record, picture_dir=PICTURE_DIR)
        if plot_path is not None:
            record["response_plot"] = plot_path.name
            record["picture_plot"] = str(PICTURE_DIR / f"{output_dir.name}.png")
        if estimation_plots:
            (output_dir / "run.json").write_text(
                json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            print(f"FIGURES: {len(estimation_plots)} 张估计/跟踪器图 → {output_dir / 'figures'}")
        if plot_path is not None:
            (output_dir / "run.json").write_text(
                json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            print(f"PLOT: {plot_path}；归档: {record['picture_plot']}")
        elif not record.get("samples"):
            print("PLOT SKIPPED: tracker 未进入跟踪采样阶段，没有可绘制的响应数据；请检查 FAIL/INTERRUPT。", file=sys.stderr)
        subscriber.close()
        link.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    import signal

    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    raise SystemExit(main())