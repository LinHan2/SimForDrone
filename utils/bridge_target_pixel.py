#!/usr/bin/env python3
"""Relay detected target pixels paired with aligned depth to the PX4 control process.

Input: geometry_msgs/PointStamped on /tracker_uav_1/front_camera/target_pixel.
point.x/y are pixel coordinates in the aligned depth image; header.stamp must be
the source image stamp. No target truth position or Oracle projection is used.
"""

import argparse
import json
import socket
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from tracking.shadow_observation import detected_relative_position_body
from utils.capture_rgbd_sample import image_to_array, stamp_seconds

PIXEL_TOPIC = "/tracker_uav_1/front_camera/target_pixel"
DEPTH_TOPIC = "/tracker_uav_1/front_camera/depth"
INFO_TOPIC = "/tracker_uav_1/front_camera/color/camera_info"


class TargetPixelBridge(Node):
    def __init__(self, port: int) -> None:
        super().__init__("simfordrone_target_pixel_bridge")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = ("127.0.0.1", port)
        self.matrix = None
        self.detection = None
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PointStamped, PIXEL_TOPIC, self.on_detection, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, image_qos)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, image_qos)

    def on_info(self, message: CameraInfo) -> None:
        self.matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    def on_detection(self, message: PointStamped) -> None:
        self.detection = message

    def on_depth(self, message: Image) -> None:
        detection = self.detection
        if detection is None or self.matrix is None:
            return
        # Both stamps must be from the same ROS clock; do not compare with host time.
        image_stamp = stamp_seconds(message)
        if image_stamp <= 0 or abs(image_stamp - stamp_seconds(detection)) > 0.05:
            return
        self.detection = None  # A detection may be consumed only once.
        try:
            body_position = detected_relative_position_body(
                image_to_array(message), detection.point.x, detection.point.y,
                self.matrix, (0.30, 0.0, 0.0),
            )
        except ValueError:
            return
        if body_position is None:
            return
        payload = {"received_at": time.monotonic(), "image_stamp": image_stamp,
                   "pixel": [detection.point.x, detection.point.y],
                   "body_position": body_position.tolist()}
        self.socket.sendto(json.dumps(payload).encode("utf-8"), self.destination)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=14602)
    args = parser.parse_args()
    rclpy.init()
    node = TargetPixelBridge(args.port)
    try:
        rclpy.spin(node)
    finally:
        node.socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()