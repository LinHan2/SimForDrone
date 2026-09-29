#!/usr/bin/env python3
"""在启动 target 航迹前，等待 tracker 相机连续检测到 target 的 Oracle FOV 门。"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from tracking.fov_readiness import evaluate_oracle_fov_depth
from utils.capture_rgbd_sample import image_to_array

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


class TrackerFovGate(Node):
    """只读订阅器；连续有效帧才释放 target 轨迹。"""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("simfordrone_tracker_fov_gate")
        self.args = args
        self.camera_matrix: np.ndarray | None = None
        self.tracker_pose: PoseStamped | None = None
        self.target_pose: PoseStamped | None = None
        self.valid_frames = 0
        self.last_reason = "waiting_for_inputs"
        self.last_status_at = 0.0
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.on_camera_info, image_qos)
        self.create_subscription(PoseStamped, TRACKER_POSE_TOPIC, self.on_tracker_pose, pose_qos)
        self.create_subscription(PoseStamped, TARGET_POSE_TOPIC, self.on_target_pose, pose_qos)
        self.create_subscription(Image, RGB_DEPTH_TOPIC, self.on_depth, image_qos)

    @property
    def ready(self) -> bool:
        return self.valid_frames >= self.args.required_frames

    def on_camera_info(self, message: CameraInfo) -> None:
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    def on_tracker_pose(self, message: PoseStamped) -> None:
        self.tracker_pose = message

    def on_target_pose(self, message: PoseStamped) -> None:
        self.target_pose = message

    def on_depth(self, message: Image) -> None:
        if self.camera_matrix is None or self.tracker_pose is None or self.target_pose is None:
            self.valid_frames = 0
            self.last_reason = "waiting_for_inputs"
            return
        result = evaluate_oracle_fov_depth(
            target_world=pose_vector(self.target_pose),
            tracker_world=pose_vector(self.tracker_pose),
            tracker_attitude=pose_quaternion(self.tracker_pose),
            camera_offset_body=tuple(self.args.camera_offset_flu),
            camera_matrix=self.camera_matrix,
            depth=image_to_array(message),
            roi_radius_px=self.args.roi_radius_px,
            max_range_m=self.args.max_range_m,
            max_oracle_range_error_m=self.args.max_oracle_range_error_m,
            camera_forward_sign=self.args.camera_forward_sign,
        )
        self.last_reason = result.reason
        self.valid_frames = self.valid_frames + 1 if result.ready else 0
        now = time.monotonic()
        if now - self.last_status_at >= self.args.status_period:
            pixel = "-" if result.pixel_u is None else f"({result.pixel_u:.1f}, {result.pixel_v:.1f})"
            print(f"FOV GATE: {result.reason}; pixel={pixel}; valid_frames={self.valid_frames}/{self.args.required_frames}", flush=True)
            self.last_status_at = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--required-frames", type=int, default=15)
    parser.add_argument("--status-period", type=float, default=1.0)
    parser.add_argument("--roi-radius-px", type=int, default=12)
    parser.add_argument("--max-range-m", type=float, default=10.0)
    parser.add_argument("--max-oracle-range-error-m", type=float, default=0.5)
    parser.add_argument("--camera-offset-flu", nargs=3, type=float, default=(0.30, 0.0, 0.0))
    parser.add_argument("--camera-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0.0 or args.required_frames <= 0 or args.status_period <= 0.0:
        raise ValueError("timeout、required-frames 和 status-period 必须为正")
    rclpy.init()
    node = TrackerFovGate(args)
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.ready:
            print("FOV GATE PASSED: Oracle 投影与深度连续有效；允许启动 target 航迹和 shadow EKF。", flush=True)
            return 0
        print(f"FOV GATE FAILED: timeout; last_reason={node.last_reason}; valid_frames={node.valid_frames}/{args.required_frames}", flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())