#!/usr/bin/env python3
"""只读验证：target PX4-EKF 姿态 + tracker RGB-D 深度驱动相对 EKF。

不连接 MAVLink、不创建 px4ctrl 状态机、不发送任何飞控指令。仿真 target/tracker
ROS 位姿只用于 Oracle ROI 投影和离线评分，绝不作为 EKF 位置量测。
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
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from tracking.relative_ekf import RelativePoseMeasurement, RelativeTargetEKF
from tracking.fov_readiness import evaluate_oracle_fov_depth
from tracking.shadow_observation import (
    pixel_ray_body,
    rotate_vector,
)
from tracking.state_io import TargetStateSubscriber
from utils.capture_rgbd_sample import image_to_array, stamp_seconds

RGB_DEPTH_TOPIC = "/tracker_uav_1/front_camera/depth"
CAMERA_INFO_TOPIC = "/tracker_uav_1/front_camera/color/camera_info"
TRACKER_POSE_TOPIC = "/tracker_uav_1/state/pose"
TARGET_POSE_TOPIC = "/target_uav_0/state/pose"


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
        self.filter = RelativeTargetEKF(stereo_range_std=args.range_std)
        self.camera_matrix: np.ndarray | None = None
        self.tracker_pose: PoseStamped | None = None
        self.target_pose: PoseStamped | None = None
        self.tracker_pose_received_at: float | None = None
        self.target_pose_received_at: float | None = None
        self.samples: list[dict[str, object]] = []
        self.rejections: dict[str, int] = {}
        self._previous_truth_position: np.ndarray | None = None
        self._previous_truth_velocity: np.ndarray | None = None
        self._previous_truth_timestamp: float | None = None
        self.started = time.monotonic()
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
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.on_camera_info, image_qos)
        self.create_subscription(PoseStamped, TRACKER_POSE_TOPIC, self.on_tracker_pose, pose_qos)
        self.create_subscription(PoseStamped, TARGET_POSE_TOPIC, self.on_target_pose, pose_qos)

    def on_camera_info(self, message: CameraInfo) -> None:
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    def on_tracker_pose(self, message: PoseStamped) -> None:
        self.tracker_pose = message
        self.tracker_pose_received_at = time.monotonic()

    def on_target_pose(self, message: PoseStamped) -> None:
        self.target_pose = message
        self.target_pose_received_at = time.monotonic()

    def reject(self, reason: str) -> None:
        self.rejections[reason] = self.rejections.get(reason, 0) + 1

    def truth_kinematics(
        self,
        relative_position: np.ndarray,
        timestamp: float,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        """以有效深度帧的本地时钟差分生成评分真值，不参与滤波更新。"""

        relative_velocity: np.ndarray | None = None
        target_acceleration: np.ndarray | None = None
        if self._previous_truth_position is not None and self._previous_truth_timestamp is not None:
            dt = timestamp - self._previous_truth_timestamp
            if dt > 1e-4:
                relative_velocity = (relative_position - self._previous_truth_position) / dt
                if self._previous_truth_velocity is not None:
                    target_acceleration = (relative_velocity - self._previous_truth_velocity) / dt
        self._previous_truth_position = relative_position.copy()
        self._previous_truth_velocity = relative_velocity
        self._previous_truth_timestamp = timestamp
        return relative_velocity, target_acceleration

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
        ):
            self.reject("waiting_for_inputs")
            return
        now = time.monotonic()
        if now - target_state.timestamp > self.args.target_state_timeout:
            self.reject("stale_target_px4_attitude")
            return
        image_stamp = stamp_seconds(message)
        pose_header_skew = max(
            abs(image_stamp - stamp_seconds(self.tracker_pose)),
            abs(image_stamp - stamp_seconds(self.target_pose)),
        )
        pose_age = max(now - self.tracker_pose_received_at, now - self.target_pose_received_at)
        if pose_age > self.args.max_pose_age_s:
            self.reject("stale_ros_pose")
            return
        depth = image_to_array(message)
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
        if not fov.ready:
            self.reject(fov.reason)
            return
        pixel_u, pixel_v, range_m, oracle_range = fov.pixel_u, fov.pixel_v, fov.range_m, fov.oracle_range_m
        if pixel_u is None or pixel_v is None or range_m is None or oracle_range is None:
            self.reject("invalid_fov_readiness")
            return
        ray_body = pixel_ray_body(
            pixel_u,
            pixel_v,
            self.camera_matrix,
            self.args.camera_forward_sign,
        )
        world_ray = rotate_vector(pose_quaternion(self.tracker_pose), tuple(ray_body))
        target_thrust = rotate_vector(target_state.q, (0.0, 0.0, 1.0))
        timestamp = now
        try:
            estimate = self.filter.update(
                RelativePoseMeasurement(
                    timestamp=timestamp,
                    relative_position=tuple(world_ray),
                    target_thrust_direction=tuple(target_thrust),
                    stereo_range_m=float(range_m),
                ),
                observer_acceleration=(0.0, 0.0, 0.0),
            )
        except ValueError as error:
            self.reject(f"ekf_{error}")
            return
        truth_relative_pos = np.asarray(pose_vector(self.target_pose)) - np.asarray(pose_vector(self.tracker_pose))
        truth_relative_vel, truth_target_acc = self.truth_kinematics(truth_relative_pos, timestamp)
        est_pos = np.asarray(estimate.relative_position)
        est_vel = np.asarray(estimate.relative_velocity)
        est_acc = np.asarray(estimate.target_acceleration)
        P_diag = self.filter.get_covariance_diagonal()
        self.samples.append(
            {
                "t": timestamp - self.started,
                "depth_range_m": float(range_m),
                "oracle_range_m": float(oracle_range),
                "position_error_m": float(np.linalg.norm(est_pos - truth_relative_pos)),
                "range_error_m": float(abs(np.linalg.norm(est_pos) - np.linalg.norm(truth_relative_pos))),
                "scale": estimate.scale,
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
                    "scale": float(P_diag[9]),
                },
                "target_px4_attitude_age_s": time.monotonic() - target_state.timestamp,
                "ros_pose_age_s": pose_age,
                "ros_pose_header_skew_s": pose_header_skew,
            }
        )

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
            "depth_measurement": "Oracle ROI median; truth used only for ROI and scoring",
            "observer_acceleration": "zero; static/hover validation only",
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
            "samples": self.samples,
        }
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return output

    def close(self) -> None:
        self.subscriber.close()
        self.destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--state-host", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=14601)
    parser.add_argument("--target-state-timeout", type=float, default=0.5)
    parser.add_argument("--max-pose-age-s", type=float, default=0.75)
    parser.add_argument("--roi-radius-px", type=int, default=12)
    parser.add_argument("--range-std", type=float, default=0.05)
    parser.add_argument("--max-range-m", type=float, default=10.0)
    parser.add_argument("--max-oracle-range-error-m", type=float, default=0.5)
    parser.add_argument("--camera-offset-flu", nargs=3, type=float, default=(0.30, 0.0, 0.0))
    parser.add_argument("--camera-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--output-root", type=Path, default=Path("logs/tracking"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0 or args.range_std <= 0.0 or args.max_range_m <= 0.0:
        raise ValueError("duration、range_std 和 max_range_m 必须为正")
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
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        output = node.write_result()
        print(f"SHADOW LOG: {output}")
        return 0 if node.samples else 1
    finally:
        if node is not None:
            node.close()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
