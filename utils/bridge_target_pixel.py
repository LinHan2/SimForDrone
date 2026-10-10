#!/usr/bin/env python3
"""Extract RGB-D target pixels, using ROS poses only to locate the search region."""

import argparse
import json
import socket
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from tracking.shadow_observation import world_to_camera
from utils.capture_rgbd_sample import image_to_array, stamp_seconds
from utils.rgbd_target_detection import detect_target_pixel

RGB_TOPIC = "/tracker_uav_1/front_camera/color/image_raw"
DEPTH_TOPIC = "/tracker_uav_1/front_camera/depth"
INFO_TOPIC = "/tracker_uav_1/front_camera/color/camera_info"
TRACKER_POSE_TOPIC = "/tracker_uav_1/state/pose"
TARGET_POSE_TOPIC = "/target_uav_0/state/pose"


class TargetPixelBridge(Node):
    def __init__(self, port: int) -> None:
        super().__init__("simfordrone_target_pixel_bridge")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = ("127.0.0.1", port)
        self.matrix = None
        self.rgb = None
        self.camera_info = None
        self.tracker_pose = None
        self.target_pose = None
        self.rgb_received_at = None
        self.tracker_pose_received_at = None
        self.target_pose_received_at = None
        self.rejections = {}
        self.last_status = 0.0
        self.detected_frames = 0
        self.missing_frames = 0
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, RGB_TOPIC, self.on_rgb, image_qos)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, image_qos)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, image_qos)
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, TRACKER_POSE_TOPIC, self.on_tracker_pose, pose_qos)
        self.create_subscription(PoseStamped, TARGET_POSE_TOPIC, self.on_target_pose, pose_qos)

    def on_info(self, message: CameraInfo) -> None:
        self.matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self.camera_info = message

    def on_rgb(self, message: Image) -> None:
        self.rgb = message
        self.rgb_received_at = time.monotonic()

    def on_tracker_pose(self, message: PoseStamped) -> None:
        self.tracker_pose = message
        self.tracker_pose_received_at = time.monotonic()

    def on_target_pose(self, message: PoseStamped) -> None:
        self.target_pose = message
        self.target_pose_received_at = time.monotonic()

    def input_rejection(self, message: Image, now: float) -> str | None:
        if (self.matrix is None or self.camera_info is None or self.rgb is None
            or self.tracker_pose is None or self.target_pose is None):
            return "waiting_for_inputs"
        if not all(received is not None and 0 <= now - received <= 0.2 for received in (
                self.rgb_received_at, self.tracker_pose_received_at, self.target_pose_received_at)):
            return "stale_input_arrival"
        if abs(stamp_seconds(message) - stamp_seconds(self.rgb)) > 0.05:
            return "rgb_depth_stamp_skew"
        if abs(stamp_seconds(self.tracker_pose) - stamp_seconds(self.target_pose)) > 0.1:
            return "pose_stamp_skew"
        if (self.rgb.width != message.width or self.rgb.height != message.height
                or self.camera_info.width != message.width or self.camera_info.height != message.height
                or not np.isfinite(self.matrix).all()
                or self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0):
            return "invalid_camera_calibration"
        return None

    def on_depth(self, message: Image) -> None:
        now = time.monotonic()
        image_stamp = stamp_seconds(message)
        if image_stamp <= 0:
            return
        pixel = None
        reason = self.input_rejection(message, now)
        try:
            rgb = self.rgb
            if reason is None:
                assert rgb is not None and self.matrix is not None
                assert self.target_pose is not None and self.tracker_pose is not None
                depth = image_to_array(message)
                if message.encoding == "16UC1":
                    depth = depth.astype(np.float32) * 0.001
                elif message.encoding != "32FC1":
                    raise ValueError("unsupported depth encoding")
                target = self.target_pose.pose.position
                tracker = self.tracker_pose.pose.position
                attitude = self.tracker_pose.pose.orientation
                optical = world_to_camera((target.x, target.y, target.z),
                    (tracker.x, tracker.y, tracker.z),
                    (attitude.x, attitude.y, attitude.z, attitude.w), (0.30, 0.0, 0.0))
                if optical[2] > 0.01:
                    expected = (self.matrix[0, 0] * optical[0] / optical[2] + self.matrix[0, 2],
                                self.matrix[1, 1] * optical[1] / optical[2] + self.matrix[1, 2])
                    colors = image_to_array(rgb)
                    if rgb.encoding == "bgr8":
                        colors = colors[..., ::-1].copy()
                    if rgb.encoding in ("rgb8", "bgr8"):
                        pixel = detect_target_pixel(colors, depth, expected, float(optical[2]))
        except (ValueError, TypeError):
            reason = "invalid_image_or_geometry"
        if pixel is None:
            reason = reason or "rgbd_not_detected"
            self.rejections[reason] = self.rejections.get(reason, 0) + 1
        self.detected_frames += int(pixel is not None)
        self.missing_frames += int(pixel is None)
        if now - self.last_status >= 2.0:
            print(f"RGBD DETECTION: valid={self.detected_frames} missing={self.missing_frames} "
                  f"latest_uv={pixel} rejections={self.rejections}", flush=True)
            self.last_status = now
        if pixel is None:
            return
        assert self.matrix is not None
        payload = {"received_at": now, "image_stamp": image_stamp,
                   "pixel": pixel, "source": "rgbd_detection",
                   "cross_clock_alignment": "monotonic_arrival_gate_not_acquisition_sync",
                   "intrinsics": self.matrix.tolist(), "resolution": [message.width, message.height]}
        self.socket.sendto(json.dumps(payload).encode("utf-8"), self.destination)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=14602)
    args = parser.parse_args()
    rclpy.init()
    node = TargetPixelBridge(args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.socket.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()