"""只控制 tracker：消费另一进程发布的目标状态并执行 V0 相对位置跟踪。"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

import yaml

from px4ctrl.cli import apply_task_defaults, enter_offboard, finish, wait_ready
from px4ctrl.controller import _quat_from_zyx
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
from tracking.evaluate_airsim_pbvs import load_controller, plot_log_diagnostics, shadow_velocity
from tracking.guidance import (
    ObserverState,
    PositionTrackerV0,
    TargetState,
    VisibilitySafetyFilter,
    apply_feedforward_level,
    band_offset,
    pitch_compensated_offset,
    predicted_target,
    slew_yaw,
)
from tracking.relative_ekf import MetricRelativeTargetEKF
from tracking.state_io import DEFAULT_STATE_HOST, DEFAULT_STATE_PORT, TargetStateSubscriber
from tracking.tracker_control import TrackerLinearControl, TrackingLimits
from tracking.visibility_diagnostics import summarize_tracking

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "px4ctrl" / "config" / "sim.yaml"
DEFAULT_TRACKING_CONFIG = ROOT / "tracking" / "config" / "tracker.yaml"


PBVS_KEYS = {"kp", "kd", "ki_rad", "i_rad_limit", "follow_dist", "min_follow_dist",
             "max_follow_dist", "kv_tan", "kv_ff_abs", "max_speed", "max_accel", "lpf_alpha"}


def load_tracking_config(path: Path, actions: dict) -> dict:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or not isinstance(config.get("tracker"), dict):
        raise ValueError("tracking config requires a tracker mapping")
    shadow = config.get("pbvs_shadow", {})
    if not isinstance(shadow, dict):
        raise ValueError("pbvs_shadow must be a mapping")
    if not isinstance(config.get("visibility_prediction", {}), dict):
        raise ValueError("visibility_prediction must be a mapping")
    unknown = set(config) - {"tracker", "pbvs_shadow", "visibility_prediction"}
    unknown |= set(config["tracker"]) - set(actions)
    unknown |= set(shadow) - PBVS_KEYS
    if unknown:
        raise ValueError(f"unknown tracking config keys: {sorted(unknown)}")
    for key, value in config["tracker"].items():
        action = actions[key]
        if value is None:
            continue
        if isinstance(action, argparse._StoreTrueAction) or isinstance(action, argparse._StoreFalseAction):
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be boolean")
        elif action.type is not None:
            if isinstance(value, bool):
                raise ValueError(f"{key} must not be boolean")
            try:
                value = action.type(value)
            except (TypeError, ValueError) as error:
                raise ValueError(f"invalid tracking config {key}: {value!r}") from error
            config["tracker"][key] = value
        if action.choices is not None and value not in action.choices:
            raise ValueError(f"invalid tracking config {key}: {value!r}")
    for key, value in shadow.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"invalid pbvs_shadow {key}: {value!r}")
        if key != "lpf_alpha" and value < 0:
            raise ValueError(f"pbvs_shadow {key} must be nonnegative")
    if (shadow.get("follow_dist", 5.0) <= 0 or shadow.get("max_speed", 5.0) <= 0
            or shadow.get("max_accel", 3.2) <= 0 or shadow.get("min_follow_dist", 4.0) <= 0
            or not 0 < shadow.get("lpf_alpha", 0.76) <= 1
            or shadow.get("min_follow_dist", 4.0) > shadow.get("follow_dist", 5.0)
            or shadow.get("follow_dist", 5.0) > shadow.get("max_follow_dist", 6.2)):
        raise ValueError("PBVS distance bounds, speed, acceleration or filter invalid")
    return config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracking-config", type=Path, default=DEFAULT_TRACKING_CONFIG,
                        help="tracking 参数 YAML；显式命令行参数优先")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--probe", action="store_true", help="仅检查 tracker 遥测与 target 状态流，不解锁")
    mode.add_argument("--execute", action="store_false", dest="probe", help="兼容旧命令；现在默认直接执行")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=30.0, help="跟踪时长（s）；0 表示一直跟踪到中断或目标状态超时")
    parser.add_argument("--offset-east", type=float, default=None)
    parser.add_argument("--offset-north", type=float, default=None)
    parser.add_argument("--offset-up", type=float, default=None)
    parser.add_argument("--follow-band-min", type=float, default=None,
                        help="软约束带下界（m）；带内不做径向牵引，越界才拉回边界")
    parser.add_argument("--follow-band-max", type=float, default=None, help="软约束带上界（m）")
    parser.add_argument("--approach-speed", type=float, default=0.5, help="跟随距离收敛速度（m/s）")
    parser.add_argument("--follow-height-offset", type=float, default=None,
                        help="跟随模式目标相对高度（m）；未设置时保留起飞时高度差")
    parser.add_argument("--follow-height-speed", type=float, default=0.05,
                        help="相对高度参考最大变化速度（m/s）")
    parser.add_argument("--max-position-error", type=float, default=1.0, help="5 m 跟随模式位置误差限幅（m）")
    parser.add_argument("--max-correction-speed", type=float, default=0.45,
                        help="水平位置误差产生的最大修正速度（m/s），不限制目标速度前馈")
    parser.add_argument("--horizontal-kp", type=float, default=2.0,
                        help="仅 tracker CMD_CTRL 的东西/南北水平位置增益")
    safety = parser.add_mutually_exclusive_group()
    safety.add_argument("--tracking-safety", action="store_true", default=False,
                        help="仅 CMD_CTRL 限制完整加速度和 jerk，允许暂时跟随滞后")
    safety.add_argument("--no-tracking-safety", action="store_false", dest="tracking_safety")
    parser.add_argument("--tracking-horizontal-accel", type=float, default=1.5)
    parser.add_argument("--tracking-vertical-accel-min", type=float, default=-0.8)
    parser.add_argument("--tracking-vertical-accel-max", type=float, default=0.8)
    parser.add_argument("--tracking-jerk", type=float, default=3.0)
    parser.add_argument("--tracking-max-dt", type=float, default=0.1)
    parser.add_argument("--acceleration-feedforward-gain", type=float, default=0.0,
                        help="目标加速度直接前馈权重 [0,1]；不改变预测所用目标状态")
    parser.add_argument("--fov-central-margin", type=float, default=0.7)
    parser.add_argument("--fov-min-coverage", type=float, default=0.95)
    parser.add_argument("--fov-min-central-fraction", type=float, default=0.9)
    parser.add_argument("--fov-max-loss-fraction", type=float, default=0.0)
    parser.add_argument("--fov-max-gap", type=float, default=0.2)
    parser.add_argument("--pbvs-shadow", action="store_true", help="仅记录 Airsim PBVS 候选速度；实际下发仍为 V0")
    visibility = parser.add_mutually_exclusive_group()
    visibility.add_argument("--visibility-prediction", action="store_true", default=False,
                            help="仅预测并记录几何可见性；不改变控制指令")
    visibility.add_argument("--no-visibility-prediction", action="store_false", dest="visibility_prediction",
                            help="完全关闭可见性预测旁路")
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument(
        "--feedforward",
        choices=("position", "velocity", "acceleration"),
        default="acceleration",
        help="前馈层级：position 只用位置(A)；velocity 加目标速度(B)；acceleration 再加目标加速度(C)",
    )
    parser.add_argument(
        "--prediction-horizon",
        type=float,
        default=0.0,
        help="用 p+v*tau+0.5*a*tau^2 预测目标未来位置的 tau（s）；0 关闭。补偿控制响应滞后",
    )
    parser.add_argument("--max-tilt-deg", type=float, default=20.0, help="tracker 组合倾角上限（度）")
    parser.add_argument(
        "--pitch-fov-compensation",
        type=float,
        default=0.0,
        help="按实测俯仰抬高期望高度差的上限（m）；0 关闭。用于高速时保持目标在画面中部",
    )
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
        help="米制三维相对位置量测标准差（m）；需按 RGB-D 误差标定",
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
        help="九维模型速度块离散过程噪声标准差（m/s）",
    )
    parser.add_argument(
        "--relative-ekf-acceleration-process-std",
        type=float,
        default=0.2449489743,
        help="九维模型加速度块离散过程噪声标准差（m/s²）",
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
    config_path = parser.parse_known_args()[0].tracking_config
    actions = {action.dest: action for action in parser._actions if action.dest not in ("help", "tracking_config")}
    config = load_tracking_config(config_path, actions)
    for key in ("config", "output_root"):
        value = config["tracker"].get(key)
        if value is not None and not value.is_absolute():
            config["tracker"][key] = ROOT / value
    parser.set_defaults(**config["tracker"])
    args = parser.parse_args()
    args.pbvs_shadow_params = config.get("pbvs_shadow", {})
    args.visibility_prediction_params = config.get("visibility_prediction", {})
    return args


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


def observer_pitch(attitude: tuple[float, float, float, float]) -> float:
    """从 ENU/FLU 四元数 ``(x, y, z, w)`` 取俯仰角；正值对应低头加速。"""

    x, y, z, w = attitude
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def observer_roll(attitude):
    """记录实际横滚，供稳定性独立评估；不作为像素控制反馈。"""
    quat_x, quat_y, quat_z, quat_w = attitude
    return math.atan2(2.0 * (quat_w * quat_x + quat_y * quat_z),
                      1.0 - 2.0 * (quat_x**2 + quat_y**2))


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
        MetricRelativeTargetEKF(
            position_std=args.relative_ekf_position_std,
            attitude_constraint_std=args.relative_ekf_attitude_constraint_std,
            position_process_std=args.relative_ekf_position_process_std,
            velocity_process_std=args.relative_ekf_velocity_process_std,
            acceleration_process_std=args.relative_ekf_acceleration_process_std,
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
        or args.relative_ekf_fading < 1.0
        or args.relative_ekf_warmup_seconds < 0.0
        or not 0.0 < args.max_tilt_deg < 45.0
        or args.max_yaw_rate <= 0.0
        or (args.follow_band_min is None) != (args.follow_band_max is None)
        or (args.follow_band_min is not None
            and not 0.0 < args.follow_band_min <= args.follow_band_max)
        or args.prediction_horizon < 0.0
        or not math.isfinite(args.prediction_horizon)
        or not math.isfinite(args.approach_speed) or args.approach_speed <= 0.0
        or (args.follow_height_offset is not None and not math.isfinite(args.follow_height_offset))
        or not math.isfinite(args.follow_height_speed) or args.follow_height_speed <= 0.0
        or not math.isfinite(args.max_position_error) or args.max_position_error <= 0.0
        or not math.isfinite(args.max_correction_speed) or args.max_correction_speed <= 0.0
        or not math.isfinite(args.horizontal_kp) or args.horizontal_kp <= 0.0
        or not 0.0 <= args.acceleration_feedforward_gain <= 1.0
        or (args.follow_band_min is not None and any(value is not None for value in (args.offset_east, args.offset_north, args.offset_up)))
        or not 0.0 <= args.relative_measurement_dropout_probability <= 1.0
    ):
        raise ValueError(
            "时长/噪声标准差/预热时长不得为负，EKF 量测与过程噪声标准差必须为正，"
            "遗忘因子不得小于 1，丢帧概率必须在 [0, 1]"
        )
    limits = TrackingLimits(args.tracking_horizontal_accel, args.tracking_vertical_accel_min,
                            args.tracking_vertical_accel_max, args.tracking_jerk, args.tracking_max_dt)
    assessment_options = dict(central_margin=args.fov_central_margin, min_coverage=args.fov_min_coverage,
                              min_central_fraction=args.fov_min_central_fraction,
                              max_loss_fraction=args.fov_max_loss_fraction, max_gap=args.fov_max_gap)
    summarize_tracking({}, **assessment_options)
    base_params = load_params(args.config)
    params = replace(base_params, max_angle=args.max_tilt_deg,
                     gain=replace(base_params.gain, kp0=args.horizontal_kp, kp1=args.horizontal_kp))
    position_to_speed_gain = max(params.gain.kp0 / params.gain.kv0,
                                 params.gain.kp1 / params.gain.kv1)
    defaults = task_defaults(params)
    link = MavlinkLink(params.link, resolve_role("tracker"), shared_frame=params.shared_frame)
    controller = TrackerLinearControl(params, limits)
    fsm = PX4CtrlFSM(params, controller, link, log=lambda message: print(f"[tracker] {message}"))
    controller.active = lambda: args.tracking_safety and fsm.state == State.CMD_CTRL
    subscriber = TargetStateSubscriber(args.state_host, args.state_port)
    output_dir = args.output_root / f"tracker-v0-{time.strftime('%Y%m%d-%H%M%S')}"
    record: dict[str, object] = {"task": "tracker-v0", "execute": not args.probe, "state_source": args.state_source, "max_tilt_deg": args.max_tilt_deg, "max_yaw_rate": args.max_yaw_rate, "max_correction_speed": args.max_correction_speed, "horizontal_kp": args.horizontal_kp, "follow_height_offset": args.follow_height_offset, "follow_height_speed": args.follow_height_speed, "feedforward": args.feedforward, "prediction_horizon": args.prediction_horizon}
    samples: list[dict[str, float]] = []
    control_samples: list[dict[str, object]] = []
    record["tracking_safety"] = {"enabled": args.tracking_safety, **asdict(limits)}
    record["acceleration_feedforward_gain"] = args.acceleration_feedforward_gain
    record["passed_scope"] = "safe_landing_and_disarm_only; see tracking_assessment"
    # 任务优先级必须严格遵守：保持目标可见 > 保持安全距离 > 相对位置/速度误差收敛 > 尽量复制目标机动。
    # 也就是说，正常情况下 a_safe = a_nom；当目标快出视野时，安全过滤器必须偏离 nominal controller，
    # 先保护视野，再恢复中间层的距离与误差控制，最后才考虑模仿目标机动。
    record["control_priority"] = [
        "keep_target_visible",
        "keep_safe_distance",
        "reduce_relative_position_and_velocity_error",
        "imitate_target_motion",
    ]
    record["tracking_objective"] = (
        "nominal_tracking_controller_plus_fov_safety_filter; "
        "keep_target_visible_first_then_safe_distance_then_tracking_error_then_target_motion"
    )
    record["position_source"] = "PX4_shared_ENU_proxy_not_Isaac_GT"
    visibility_samples: list[dict[str, object]] = []
    visibility_diagnostics = None
    if args.visibility_prediction:
        from tracking.visibility_diagnostics import VisibilityDiagnostics

        visibility_diagnostics = VisibilityDiagnostics(args.visibility_prediction_params, ROOT)
        record["visibility_prediction"] = visibility_diagnostics.metadata
        if "initialization_error" in visibility_diagnostics.metadata:
            print(f"VISIBILITY DIAGNOSTICS UNAVAILABLE: {visibility_diagnostics.metadata['initialization_error']}", file=sys.stderr)
    else:
        record["visibility_prediction"] = {"enabled": False}
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
    record["pbvs_shadow"] = args.pbvs_shadow

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
        visibility_filter = None
        if args.follow_band_min is not None and args.follow_band_max is not None:
            visibility_filter = VisibilitySafetyFilter(
                min_distance=args.follow_band_min,
                max_distance=args.follow_band_max,
            )
        pbvs = None
        if args.pbvs_shadow:
            pbvs = load_controller()(**args.pbvs_shadow_params, dt=rate)
        sensor = NoisyTargetSensor(args.position_noise_std, args.velocity_noise_std, args.noise_seed)
        estimator = PassthroughEstimator()
        fsm.request_command_control()
        record["relative_offset"] = relative_offset
        started = time.monotonic()
        last_yaw_at = started
        last_filter_at: float | None = None
        commanded_yaw = yaw_from_quaternion(link.odom.q)
        commanded_pitch = observer_pitch(link.odom.q)
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
            if visibility_diagnostics is not None:
                visibility_target_state = (relative_estimator.last_estimate
                                           if args.state_source == "relative-ekf" else target_state)
            # 先裁剪前馈层级再外推，保证 A/B/C 基线只差前馈项本身。
            target_state = apply_feedforward_level(target_state, args.feedforward)
            target_state = predicted_target(target_state, args.prediction_horizon)
            if args.follow_band_min is not None and args.follow_band_max is not None:
                # 以当前实测相对位置为基准：带内不牵引，只在越界时缩放回边界。
                current_relative = tuple(
                    link.shared_position[index] - target_state.p[index] for index in range(3)
                )
                relative_offset = band_offset(
                    current_relative, args.follow_band_min, args.follow_band_max,
                    args.follow_height_offset,
                )
                if args.pitch_fov_compensation > 0.0:
                    # 用上一周期的指令俯仰而非实测俯仰：实测滞后指令最大约 7°，
                    # 按实测补偿会在高速段留下与出画同量级的残差。
                    relative_offset = pitch_compensated_offset(
                        relative_offset, commanded_pitch, args.pitch_fov_compensation
                    )
                guidance = PositionTrackerV0(relative_offset=relative_offset, yaw=args.yaw,
                                             max_position_error=args.max_position_error,
                                             max_correction_speed=args.max_correction_speed,
                                             position_to_speed_gain=position_to_speed_gain)
            # 实际指令: (p_ref^L, v_ref, a_ref, psi_ref) 由 V0 生成；此处无像素误差反馈。
            reference = guidance.generate(
                target_state,
                ObserverState(shared_p=link.shared_position, local_p=link.odom.p),
            )
            reference = replace(reference, a=tuple(args.acceleration_feedforward_gain * value
                                                   for value in reference.a))
            if visibility_filter is not None:
                predicted_fov_exit = False
                # 预测器可能因配置/标定错误而不可用，此时退化为纯 nominal 跟踪而不是中断飞行。
                if visibility_diagnostics is not None and visibility_diagnostics.predictor is not None:
                    tracker_state = TargetState(link.shared_position, link.odom.v,
                                                observer_world_acceleration(link.imu.q, link.imu.acc))
                    desired_world_attitude = _quat_from_zyx(reference.yaw, observer_pitch(link.odom.q),
                                                            observer_roll(link.odom.q))
                    prediction = visibility_diagnostics.predictor.predict(
                        target_state, tracker_state, link.odom.q, desired_world_attitude,
                        tracker_acceleration_reliable=all(math.isfinite(value) for value in tracker_state.a),
                    )
                    predicted_fov_exit = bool(prediction.valid and prediction.predicted_fov_exit)
                filter_dt = 0.0 if last_filter_at is None else now - last_filter_at
                last_filter_at = now
                reference = visibility_filter.apply(
                    reference, target_state,
                    ObserverState(shared_p=link.shared_position, local_p=link.odom.p),
                    predicted_exit=predicted_fov_exit, dt=filter_dt,
                )
            # psi_cmd = psi_prev + clip(wrap(psi_ref - psi_prev), +/- max_yaw_rate * dt)。
            commanded_yaw = slew_yaw(commanded_yaw, reference.yaw, args.max_yaw_rate * (now - last_yaw_at))
            last_yaw_at = now
            reference = replace(reference, yaw=commanded_yaw)
            shadow_command = None
            if pbvs is not None:
                # PBVS 仅记录候选速度，不参与下方实际下发的 V0 CommandData。
                shadow_command = shadow_velocity(pbvs, target_state.p, target_state.v,
                                                 link.shared_position, link.odom.v)
            fsm.set_command(command_from_reference(reference))
            was_cmd_ctrl = fsm.state == State.CMD_CTRL
            output = fsm.process(now)
            commanded_pitch = observer_pitch(output.q)
            if visibility_diagnostics is not None:
                if was_cmd_ctrl and fsm.state == State.CMD_CTRL:
                    visibility_samples.append(visibility_diagnostics.sample(
                        now=time.monotonic(), started=started, target_state=visibility_target_state,
                        target_message=target_message, link=link, reference=reference, output=output,
                        debug=fsm.controller.debug, state_source=args.state_source,
                    ))
                else:
                    visibility_samples.append({"t": now - started, "monotonic_s": now,
                                               "valid": False, "reason": "fsm_not_CMD_CTRL"})
            if was_cmd_ctrl and fsm.state == State.CMD_CTRL:
                relative_position = tuple(truth.p[axis] - link.shared_position[axis] for axis in range(3))
                relative_velocity = tuple(truth.v[axis] - link.odom.v[axis] for axis in range(3))
                distance = math.hypot(*relative_position[:2])
                band_violation = (max(args.follow_band_min - distance, 0.0, distance - args.follow_band_max)
                                  if args.follow_band_min is not None and args.follow_band_max is not None else None)
                control_samples.append({
                    "t": now - started,
                    "monotonic_s": now,
                    "odom_recv_t": link.odom.recv_time - started,
                    "attitude_recv_monotonic_s": link.odom.attitude_recv_time,
                    "reference_p": reference.p,
                    "reference_v": reference.v,
                    "reference_a": reference.a,
                    "target_p": truth.p, "target_v": truth.v,
                    "target_a": target_message.reference_a,
                    "target_acceleration_source": "trajectory_reference_not_measured",
                    "observer_shared_p": link.shared_position,
                    "relative_position": relative_position,
                    "relative_velocity": relative_velocity,
                    "predicted_target_p": target_state.p,
                    "prediction_horizon": args.prediction_horizon,
                    "horizontal_distance": distance,
                    "distance_band_violation": band_violation,
                    "desired_roll": controller.debug.roll,
                    "desired_pitch": controller.debug.pitch,
                    "desired_yaw": reference.yaw,
                    "actual_roll": observer_roll(link.odom.q),
                    "actual_pitch": observer_pitch(link.odom.q),
                    "actual_yaw": yaw_from_quaternion(link.odom.q),
                    **controller.tracking_debug,
                    "odom_p": link.odom.p,
                    "odom_v": link.odom.v,
                    "odom_q": link.odom.q,
                    "output_q": output.q,
                    "output_thrust": output.thrust,
                    "tilt_saturated": fsm.controller.debug.tilt_saturated,
                    "relative_offset": relative_offset,
                    "odom_pitch": observer_pitch(link.odom.q),
                    "output_pitch": observer_pitch(output.q),
                    "pbvs_shadow_v": shadow_command,
                })
            # 绘图与误差都使用实际送入控制器的估计目标，不用真值掩盖估计误差。
            desired_shared = tuple(target_state.p[index] + relative_offset[index] for index in range(3))
            error = math.dist(link.shared_position, desired_shared)
            horizontal_distance = math.hypot(truth.p[0] - link.shared_position[0], truth.p[1] - link.shared_position[1])
            if args.follow_band_min is not None and args.follow_band_max is not None:
                if not follow_ready and args.follow_band_min <= horizontal_distance <= args.follow_band_max:
                    print("FOLLOW BAND READY", flush=True)
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
        print(f"{'PASS' if passed else 'FAIL'}: tracker-v0, safe landing/disarm only; tracking assessment is separate")
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
        record["control_samples"] = control_samples
        if visibility_diagnostics is not None:
            record["visibility_samples"] = visibility_samples
        if samples and "samples" not in record:
            record["samples"] = samples[:: max(1, len(samples) // 300)]
        if target_samples and "target_samples" not in record:
            record["target_samples"] = target_samples
        record["tracking_assessment"] = summarize_tracking(record, **assessment_options)
        print("TRACKING ASSESSMENT: " + json.dumps(record["tracking_assessment"], allow_nan=False))
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
        if args.pbvs_shadow and control_samples:
            try:
                shadow_plot = output_dir / "pbvs_shadow_response.png"
                plot_log_diagnostics(output_dir / "run.json", shadow_plot)
                print(f"PBVS SHADOW PLOT: {shadow_plot}")
            except Exception as error:
                print(f"PBVS SHADOW PLOT FAILED: {error}", file=sys.stderr)
        if visibility_samples:
            try:
                from tracking.visibility_diagnostics import export_visibility

                paths = export_visibility(record, output_dir)
                print(f"VISIBILITY DIAGNOSTICS: {', '.join(str(path) for path in paths)}")
            except Exception as error:
                print(f"VISIBILITY EXPORT FAILED (raw samples retained): {error}", file=sys.stderr)
        subscriber.close()
        link.close()
        print(f"LOG: {output_dir}")


if __name__ == "__main__":
    import signal

    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    raise SystemExit(main())