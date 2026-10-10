#!/usr/bin/env python3
"""只读验证：target PX4-EKF 姿态 + tracker 深度目标追踪驱动米制相对 EKF。

不连接 MAVLink、不创建 px4ctrl 状态机、不发送任何飞控指令。仿真 target/tracker
ROS 目标位姿用于搜索提示和离线评分；位置量测来自 RGB-D，仍属于 GT 辅助关联。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu

from tracking.estimation import observer_world_acceleration
from tracking.relative_ekf import MetricPoseMeasurement, MetricRelativeTargetEKF
from tracking.fov_readiness import evaluate_oracle_fov_depth
from tracking.shadow_observation import (
    observation_time_skew,
    depth_roi_median,
    project_oracle_pixel,
    world_to_camera,
    pixel_ray_body,
    range_from_optical_depth,
    relative_position_from_depth,
    rotate_vector,
    track_depth_roi,
)
from tracking.state_io import TargetStateSubscriber
from tracking.shadow_plotting import plot_observation
from utils.capture_rgbd_sample import image_to_array, stamp_seconds
from utils.rgbd_target_detection import detect_target_pixel

RGB_DEPTH_TOPIC = "/tracker_uav_1/front_camera/depth"
RGB_TOPIC = "/tracker_uav_1/front_camera/color/image_raw"
IMU_TOPIC = "/tracker_uav_1/sensors/imu"
CAMERA_INFO_TOPIC = "/tracker_uav_1/front_camera/color/camera_info"
TRACKER_POSE_TOPIC = "/tracker_uav_1/state/pose"
TARGET_POSE_TOPIC = "/target_uav_0/state/pose"
TRACKER_VELOCITY_TOPIC = "/tracker_uav_1/state/twist_inertial"
TARGET_VELOCITY_TOPIC = "/target_uav_0/state/twist_inertial"
TARGET_ACCELERATION_TOPIC = "/target_uav_0/state/accel"


def pose_vector(message: PoseStamped) -> tuple[float, float, float]:
    position = message.pose.position
    return float(position.x), float(position.y), float(position.z)


def pose_quaternion(message: PoseStamped) -> tuple[float, float, float, float]:
    orientation = message.pose.orientation
    return float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)


class ShadowRelativeEkf(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("simfordrone_shadow_relative_ekf")
        self.args = args
        self.subscriber = TargetStateSubscriber(args.state_host, args.state_port)
        self.filter = MetricRelativeTargetEKF(position_std=args.range_std)
        self.camera_matrix: np.ndarray | None = None
        self.tracker_pose: PoseStamped | None = None
        self.target_pose: PoseStamped | None = None
        self.rgb: Image | None = None
        self.imu: Imu | None = None
        self.imu_received_at: float | None = None
        self.last_image_stamp: float | None = None
        self.tracker_pose_received_at: float | None = None
        self.target_pose_received_at: float | None = None
        self.samples: list[dict[str, object]] = []
        self.fov_samples: list[dict[str, object]] = []
        self.last_depth_observation: tuple[float, float, float] | None = None
        self.rejections: dict[str, int] = {}
        self.gt_messages: dict[str, tuple[np.ndarray, float, str, float]] = {}
        self.started = time.monotonic()
        self.last_status = self.started
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, RGB_DEPTH_TOPIC, self.on_depth, image_qos)
        self.create_subscription(Image, RGB_TOPIC, self.on_rgb, image_qos)
        self.create_subscription(Imu, IMU_TOPIC, self.on_imu, pose_qos)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.on_camera_info, image_qos)
        self.create_subscription(PoseStamped, TRACKER_POSE_TOPIC, self.on_tracker_pose, pose_qos)
        self.create_subscription(PoseStamped, TARGET_POSE_TOPIC, self.on_target_pose, pose_qos)
        self.create_subscription(TwistStamped, TRACKER_VELOCITY_TOPIC,
                     lambda message: self.on_gt("observer_velocity", message), pose_qos)
        self.create_subscription(TwistStamped, TARGET_VELOCITY_TOPIC,
                     lambda message: self.on_gt("target_velocity", message), pose_qos)
        self.create_subscription(AccelStamped, TARGET_ACCELERATION_TOPIC,
                     lambda message: self.on_gt("target_acceleration", message), pose_qos)

    def on_camera_info(self, message: CameraInfo) -> None:
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    def on_rgb(self, message: Image) -> None:
        self.rgb = message

    def on_imu(self, message: Imu) -> None:
        self.imu = message
        self.imu_received_at = time.monotonic()

    def on_tracker_pose(self, message: PoseStamped) -> None:
        self.tracker_pose = message
        self.tracker_pose_received_at = time.monotonic()

    def on_target_pose(self, message: PoseStamped) -> None:
        self.target_pose = message
        self.target_pose_received_at = time.monotonic()

    def reject(self, reason: str) -> None:
        self.rejections[reason] = self.rejections.get(reason, 0) + 1

    def on_gt(self, quantity: str, message) -> None:
        linear = (message.accel.linear if quantity == "target_acceleration"
                  else message.twist.linear)
        self.gt_messages[quantity] = (
            np.asarray((linear.x, linear.y, linear.z), dtype=float),
            stamp_seconds(message), message.header.frame_id, time.monotonic())

    def truth_kinematics(self, now: float) -> tuple[np.ndarray | None, np.ndarray | None, dict]:
        """读取仿真世界系状态，仅用于评分；缺失或过期时不差分补位。"""

        vectors = {}
        diagnostics = {}
        for quantity, pose in (("observer_velocity", self.tracker_pose),
                               ("target_velocity", self.target_pose),
                               ("target_acceleration", self.target_pose)):
            cached = self.gt_messages.get(quantity)
            if cached is None or pose is None:
                diagnostics[quantity] = {"status": "missing"}
                continue
            vector, stamp, frame, received = cached
            age = now - received
            skew = abs(stamp - stamp_seconds(pose))
            valid = (frame == "map" and stamp > 0
                     and 0 <= age <= self.args.max_pose_age_s
                     and skew <= self.args.max_observation_skew_s)
            valid = valid and bool(np.all(np.isfinite(vector)))
            diagnostics[quantity] = {"status": "valid" if valid else "invalid_or_stale",
                                     "stamp": stamp, "age_s": age,
                                     "pose_skew_s": skew, "frame_id": frame}
            if valid:
                vectors[quantity] = vector
        relative_velocity = None
        if "target_velocity" in vectors and "observer_velocity" in vectors:
            velocity_skew = abs(diagnostics["target_velocity"]["stamp"]
                                - diagnostics["observer_velocity"]["stamp"])
            diagnostics["relative_velocity_skew_s"] = velocity_skew
            if velocity_skew <= self.args.max_observation_skew_s:
                relative_velocity = vectors["target_velocity"] - vectors["observer_velocity"]
        return relative_velocity, vectors.get("target_acceleration"), diagnostics

    def on_depth(self, message: Image) -> None:
        self.subscriber.poll()
        target_state = self.subscriber.latest
        if (
            self.camera_matrix is None
            or self.tracker_pose is None
            or self.target_pose is None
            or self.tracker_pose_received_at is None
            or self.target_pose_received_at is None
            or target_state is None
            or self.imu is None or self.imu_received_at is None
        ):
            self.reject("waiting_for_inputs")
            return
        now = time.monotonic()
        if now - target_state.timestamp > self.args.target_state_timeout:
            self.reject("stale_target_px4_attitude")
            return
        attitude_timestamp = target_state.attitude_timestamp
        if attitude_timestamp is None or attitude_timestamp <= 0.0:
            self.reject("missing_target_attitude_timestamp")
            return
        time_skew = observation_time_skew(
            now, self.tracker_pose_received_at, self.target_pose_received_at, attitude_timestamp
        )
        if time_skew > self.args.max_observation_skew_s:
            self.reject("unsynchronized_observation")
            return
        image_stamp = stamp_seconds(message)
        if image_stamp <= 0 or (self.last_image_stamp is not None and image_stamp <= self.last_image_stamp):
            self.reject("nonincreasing_image_stamp")
            return
        if (now - self.imu_received_at > self.args.max_observation_skew_s
            or abs(stamp_seconds(self.tracker_pose) - stamp_seconds(self.imu))
            > self.args.max_observation_skew_s):
            self.reject("stale_or_unsynchronized_imu")
            return
        pose_header_skew = max(
            abs(stamp_seconds(self.tracker_pose) - stamp_seconds(self.target_pose)),
            abs(stamp_seconds(self.tracker_pose) - stamp_seconds(self.imu)),
        )
        pose_age = max(now - self.tracker_pose_received_at, now - self.target_pose_received_at)
        if pose_header_skew > self.args.max_observation_skew_s:
            self.reject("unsynchronized_pose_headers")
            return
        if pose_age > self.args.max_pose_age_s:
            self.reject("stale_ros_pose")
            return
        depth = image_to_array(message)
        if message.encoding == "16UC1":
            depth = depth.astype(np.float32) * 0.001
        fov = evaluate_oracle_fov_depth(
            target_world=pose_vector(self.target_pose),
            tracker_world=pose_vector(self.tracker_pose),
            tracker_attitude=pose_quaternion(self.tracker_pose),
            camera_offset_body=tuple(self.args.camera_offset_flu),
            camera_matrix=self.camera_matrix,
            depth=depth,
            roi_radius_px=self.args.roi_radius_px,
            max_range_m=self.args.max_range_m,
            max_oracle_range_error_m=self.args.max_oracle_range_error_m,
            camera_forward_sign=self.args.camera_forward_sign,
        )
        self.fov_samples.append({
            "monotonic_s": now,
            "reason": fov.reason,
            "pixel_u": fov.pixel_u,
            "pixel_v": fov.pixel_v,
            "image_width": int(depth.shape[1]),
            "image_height": int(depth.shape[0]),
            "tracker_q": pose_quaternion(self.tracker_pose),
            "tracker_u": pose_vector(self.tracker_pose)[2],
            "target_u": pose_vector(self.target_pose)[2],
        })
        if not self.args.legacy_depth_tracking:
            if self.rgb is None or abs(image_stamp - stamp_seconds(self.rgb)) > 0.05:
                self.reject("missing_or_unsynchronized_rgb")
                return
            projection = project_oracle_pixel(
                pose_vector(self.target_pose), pose_vector(self.tracker_pose),
                pose_quaternion(self.tracker_pose), tuple(self.args.camera_offset_flu),
                self.camera_matrix, self.args.camera_forward_sign,
            )
            if projection is None or self.rgb.encoding not in ("rgb8", "bgr8"):
                self.reject("invalid_rgb_search_hint")
                return
            colors = image_to_array(self.rgb)
            if self.rgb.encoding == "bgr8":
                colors = colors[..., ::-1].copy()
            expected_optical = world_to_camera(
                pose_vector(self.target_pose), pose_vector(self.tracker_pose),
                pose_quaternion(self.tracker_pose), tuple(self.args.camera_offset_flu),
                self.args.camera_forward_sign,
            )
            detected = detect_target_pixel(colors, depth, projection[:2], float(expected_optical[2]))
            if detected is None:
                self.reject("rgbd_target_not_detected")
                return
            pixel_u, pixel_v = detected
            optical_depth = depth_roi_median(depth, pixel_u, pixel_v, 3)
            if optical_depth is None:
                self.reject("invalid_detected_depth")
                return
        elif self.args.oracle_roi_every_frame or self.last_depth_observation is None:
            if not fov.ready or fov.pixel_u is None or fov.pixel_v is None or fov.range_m is None:
                self.reject(fov.reason)
                return
            pixel_u, pixel_v = fov.pixel_u, fov.pixel_v
            initial_ray = pixel_ray_body(pixel_u, pixel_v, self.camera_matrix, self.args.camera_forward_sign)
            optical_depth = fov.range_m * abs(float(initial_ray[0]))
        else:
            tracked = track_depth_roi(depth, *self.last_depth_observation, self.args.track_radius_px, self.args.track_depth_tolerance_m)
            if tracked is None:
                self.reject("depth_target_lost")
                return
            pixel_u, pixel_v, optical_depth = tracked
        ray_body = pixel_ray_body(
            pixel_u,
            pixel_v,
            self.camera_matrix,
            self.args.camera_forward_sign,
        )
        range_m = range_from_optical_depth(optical_depth, ray_body)
        if range_m is None or range_m > self.args.max_range_m:
            self.reject("range_out_of_bounds")
            return
        relative_position = relative_position_from_depth(
            ray_body, range_m, tuple(self.args.camera_offset_flu), pose_quaternion(self.tracker_pose)
        )
        target_thrust = rotate_vector(target_state.q, (0.0, 0.0, 1.0))
        specific_force = self.imu.linear_acceleration
        observer_acceleration = observer_world_acceleration(
            pose_quaternion(self.tracker_pose),
            (specific_force.x, -specific_force.y, -specific_force.z),
        )
        timestamp = image_stamp
        try:
            estimate = self.filter.update(
                MetricPoseMeasurement(
                    timestamp=timestamp,
                    relative_position=tuple(relative_position),
                    target_thrust_direction=tuple(target_thrust),
                ),
                observer_acceleration=observer_acceleration,
            )
        except ValueError as error:
            self.reject(f"ekf_{error}")
            return
        self.last_depth_observation = (pixel_u, pixel_v, optical_depth)
        self.last_image_stamp = image_stamp
        truth_relative_pos = np.asarray(pose_vector(self.target_pose)) - np.asarray(pose_vector(self.tracker_pose))
        truth_relative_vel, truth_target_acc, gt_diagnostics = self.truth_kinematics(now)
        est_pos = np.asarray(estimate.relative_position)
        est_vel = np.asarray(estimate.relative_velocity)
        est_acc = np.asarray(estimate.target_acceleration)
        P_diag = self.filter.get_covariance_diagonal()
        self.samples.append(
            {
                "t": now - self.started,
                "image_stamp": image_stamp,
                "gt_diagnostics": gt_diagnostics,
                "observer_acceleration": observer_acceleration,
                "target_thrust_direction": list(target_thrust),
                "attitude_constraint_mode": "tangent_2d",
                "optical_depth_m": float(optical_depth),
                "depth_range_m": float(range_m),
                "oracle_range_m": fov.oracle_range_m,
                "pixel_u": pixel_u,
                "pixel_v": pixel_v,
                "position_error_m": float(np.linalg.norm(est_pos - truth_relative_pos)),
                "range_error_m": float(abs(np.linalg.norm(est_pos) - np.linalg.norm(truth_relative_pos))),
                "measurement_relative_position": relative_position.tolist(),
                "estimate": {
                    "relative_position": est_pos.tolist(),
                    "relative_velocity": est_vel.tolist(),
                    "target_acceleration": est_acc.tolist(),
                },
                "truth": {
                    "relative_position": truth_relative_pos.tolist(),
                    "relative_velocity": truth_relative_vel.tolist() if truth_relative_vel is not None else None,
                    "target_acceleration": truth_target_acc.tolist() if truth_target_acc is not None else None,
                },
                "velocity_error_mps": (
                    float(np.linalg.norm(est_vel - truth_relative_vel))
                    if truth_relative_vel is not None
                    else None
                ),
                "acceleration_error_mps2": (
                    float(np.linalg.norm(est_acc - truth_target_acc))
                    if truth_target_acc is not None
                    else None
                ),
                "covariance_diag": {
                    "position": P_diag[:3].tolist(),
                    "velocity": P_diag[3:6].tolist(),
                    "acceleration": P_diag[6:9].tolist(),
                },
                "target_px4_attitude_age_s": time.monotonic() - attitude_timestamp,
                "target_px4_attitude_q": list(target_state.q),
                "observation_arrival_skew_s": time_skew,
                "ros_pose_age_s": pose_age,
                "ros_pose_header_skew_s": pose_header_skew,
                "cross_clock_alignment": "arrival gate only; image simulation / sensor wall clock",
            }
        )
        if now - self.last_status >= 2.0:
            print(f"SHADOW tangent_2d samples={len(self.samples)} "
                  f"position_error={np.linalg.norm(est_pos - truth_relative_pos):.3f}m "
                  f"rejections={self.rejections}", flush=True)
            self.last_status = now

    def numeric_sample_values(self, key: str) -> list[float]:
        """JSON样本允许嵌套字段，统计前只保留已评分的数值。"""

        values: list[float] = []
        for sample in self.samples:
            value = sample[key]
            if isinstance(value, (float, int)):
                values.append(float(value))
        return values

    def write_result(self) -> Path:
        self.args.output_root.mkdir(parents=True, exist_ok=True)
        output = self.args.output_root / f"shadow-relative-ekf-{time.strftime('%Y%m%d-%H%M%S')}.json"
        position_errors = self.numeric_sample_values("position_error_m")
        velocity_errors = self.numeric_sample_values("velocity_error_mps")
        acceleration_errors = self.numeric_sample_values("acceleration_error_mps2")
        result = {
            "mode": "shadow_only_no_mavlink_control",
            "target_attitude": "target PX4 EKF ATTITUDE_QUATERNION relayed over UDP",
            "depth_measurement": (
                "RGB-D connected component; GT-assisted search; surface Z depth"
                if not self.args.legacy_depth_tracking else
                "Oracle ROI every frame; depth is target surface, not center"
                if self.args.oracle_roi_every_frame else
                "Oracle seed once; local depth tracking thereafter; depth is target surface, not center"
            ),
            "filter": "nine-state metric position/velocity/target acceleration",
            "observer_acceleration": "ROS IMU FRD specific force -> FLU -> ENU plus gravity",
            "attitude_constraint_mode": "tangent_2d",
            "filter_timebase": "ROS image simulation timestamp",
            "cross_clock_alignment": "arrival gate only; no acquisition-time interpolation",
            "truth_kinematics": "simulator_state_enu",
            "truth_sources": {"position": [TARGET_POSE_TOPIC, TRACKER_POSE_TOPIC],
                              "relative_velocity": [TARGET_VELOCITY_TOPIC, TRACKER_VELOCITY_TOPIC],
                              "target_acceleration": TARGET_ACCELERATION_TOPIC},
            "gt_alignment": "fresh state messages gated against pose headers; cross-clock arrival alignment only",
            "gt_acceleration_definition": "simulator velocity difference over physics dt; includes gravity, not specific force",
            "sample_count": len(self.samples),
            "position_rmse_m": math.sqrt(sum(error * error for error in position_errors) / len(position_errors)) if position_errors else None,
            "position_max_error_m": max(position_errors) if position_errors else None,
            "velocity_score_sample_count": len(velocity_errors),
            "velocity_rmse_mps": math.sqrt(sum(error * error for error in velocity_errors) / len(velocity_errors)) if velocity_errors else None,
            "velocity_max_error_mps": max(velocity_errors) if velocity_errors else None,
            "acceleration_score_sample_count": len(acceleration_errors),
            "acceleration_rmse_mps2": math.sqrt(sum(error * error for error in acceleration_errors) / len(acceleration_errors)) if acceleration_errors else None,
            "acceleration_max_error_mps2": max(acceleration_errors) if acceleration_errors else None,
            "rejections": self.rejections,
            "fov_samples": self.fov_samples,
            "samples": self.samples,
        }
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if self.samples:
            plot_observation(result, output.with_name(output.stem + "-observation.png"))
        return output

    def close(self) -> None:
        self.subscriber.close()
        self.destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plot-log", type=Path, help="仅为已有 shadow JSON 绘制观测曲线，不连接飞控或 ROS")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--state-host", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=14601)
    parser.add_argument("--target-state-timeout", type=float, default=0.5)
    parser.add_argument("--max-pose-age-s", type=float, default=0.75)
    parser.add_argument("--max-observation-skew-s", type=float, default=0.15)
    parser.add_argument("--roi-radius-px", type=int, default=12)
    parser.add_argument("--oracle-roi-every-frame", action="store_true")
    parser.add_argument("--legacy-depth-tracking", action="store_true")
    parser.add_argument("--track-radius-px", type=int, default=18)
    parser.add_argument("--track-depth-tolerance-m", type=float, default=0.3)
    parser.add_argument("--range-std", type=float, default=0.05)
    parser.add_argument("--max-range-m", type=float, default=10.0)
    parser.add_argument("--max-oracle-range-error-m", type=float, default=0.5)
    parser.add_argument("--camera-offset-flu", nargs=3, type=float, default=(0.30, 0.0, 0.0))
    parser.add_argument("--camera-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--output-root", type=Path, default=Path("logs/tracking"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.plot_log is not None:
        record = json.loads(args.plot_log.read_text(encoding="utf-8"))
        output = args.plot_log.with_name(args.plot_log.stem + "-observation.png")
        plot_observation(record, output)
        print(f"OBSERVATION PLOT: {output}")
        print(f"KINEMATICS PLOT: {output.with_name(output.stem.replace('-observation', '-kinematics') + '.png')}")
        return 0
    if (args.duration <= 0.0 or args.range_std <= 0.0 or args.max_range_m <= 0.0
            or args.max_observation_skew_s <= 0.0 or args.track_radius_px < 1 or args.track_depth_tolerance_m <= 0.0):
        raise ValueError("duration、range_std、max_range_m 和量测门限必须为正")
    rclpy.init()
    node: ShadowRelativeEkf | None = None
    try:
        try:
            node = ShadowRelativeEkf(args)
        except (RuntimeError, OSError) as error:
            # 启动期冲突（典型为状态端口被串行前一级的 tracker 占用）：打印可操作
            # 的提示后干净退出，不抛裸 traceback，避免掩盖真正的操作原因。
            print(f"SHADOW STARTUP FAILED: {error}", file=sys.stderr)
            return 2
        print("SHADOW READY: target state socket bound", flush=True)
        deadline = time.monotonic() + args.duration
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.25)
        except KeyboardInterrupt:
            pass
        output = node.write_result()
        print(f"SHADOW LOG: {output}")
        if node.samples:
            print(f"OBSERVATION PLOT: {output.with_name(output.stem + '-observation.png')}")
            print(f"KINEMATICS PLOT: {output.with_name(output.stem + '-kinematics.png')}")
        return 0 if node.samples else 1
    finally:
        if node is not None:
            node.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
