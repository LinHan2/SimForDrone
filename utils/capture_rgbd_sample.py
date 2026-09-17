#!/usr/bin/env python3
"""采样并保存跟随机 RGB-D、内参与位姿，用于 P1 观测内容验收。

该工具不发送飞控指令；它只从系统 ROS Jazzy 终端订阅一次消息并保存到 logs。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image as RosImage


RGB_TOPIC = "/tracker_uav_1/front_camera/color/image_raw"
DEPTH_TOPIC = "/tracker_uav_1/front_camera/depth"
CAMERA_INFO_TOPIC = "/tracker_uav_1/front_camera/color/camera_info"
POSE_TOPIC = "/tracker_uav_1/state/pose"


def stamp_seconds(message: Any) -> float:
    """把 ROS Header 时间戳转为秒；不假设它一定是墙钟或仿真时间。"""

    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def image_to_array(message: RosImage) -> np.ndarray:
    """按 ROS 编码、行跨度和字节序将 Image 转成 NumPy 数组。"""

    encodings: dict[str, tuple[np.dtype, int]] = {
        "rgb8": (np.dtype(np.uint8), 3),
        "bgr8": (np.dtype(np.uint8), 3),
        "rgba8": (np.dtype(np.uint8), 4),
        "bgra8": (np.dtype(np.uint8), 4),
        "mono8": (np.dtype(np.uint8), 1),
        "16UC1": (np.dtype(np.uint16), 1),
        "16SC1": (np.dtype(np.int16), 1),
        "32FC1": (np.dtype(np.float32), 1),
        "64FC1": (np.dtype(np.float64), 1),
    }
    if message.encoding not in encodings:
        raise ValueError(f"暂不支持 ROS 图像编码: {message.encoding}")

    dtype, channels = encodings[message.encoding]
    if message.is_bigendian and dtype.itemsize > 1:
        dtype = dtype.newbyteorder(">")

    minimum_step = message.width * channels * dtype.itemsize
    if message.step < minimum_step:
        raise ValueError(
            f"图像 step={message.step} 小于最小行字节数 {minimum_step}"
        )
    expected_bytes = message.step * message.height
    if len(message.data) < expected_bytes:
        raise ValueError(
            f"图像有效负载不足: {len(message.data)} < {expected_bytes} bytes"
        )

    if channels == 1:
        shape = (message.height, message.width)
        strides = (message.step, dtype.itemsize)
    else:
        shape = (message.height, message.width, channels)
        strides = (message.step, channels * dtype.itemsize, dtype.itemsize)

    return np.ndarray(shape=shape, dtype=dtype, buffer=message.data, strides=strides).copy()


def rgb_for_png(array: np.ndarray, encoding: str) -> np.ndarray:
    """将常见 RGB/BGR/灰度编码转换为 Pillow 可保存的 RGB 图像。"""

    if encoding == "rgb8":
        return array
    if encoding == "bgr8":
        return array[:, :, ::-1]
    if encoding == "rgba8":
        return array[:, :, :3]
    if encoding == "bgra8":
        return array[:, :, [2, 1, 0]]
    if encoding == "mono8":
        return np.repeat(array[:, :, None], 3, axis=2)
    raise ValueError(f"RGB 话题不应使用编码 {encoding}")


def depth_visualization(depth: np.ndarray) -> tuple[np.ndarray, dict[str, float | int]]:
    """将深度归一化为 8-bit 灰度预览，同时保留原始数组供算法使用。"""

    scalar_depth = np.asarray(depth, dtype=np.float64)
    valid = scalar_depth[np.isfinite(scalar_depth) & (scalar_depth > 0.0)]
    if valid.size == 0:
        return np.zeros(scalar_depth.shape, dtype=np.uint8), {
            "valid_pixel_count": 0,
            "minimum": math.nan,
            "maximum": math.nan,
            "p02": math.nan,
            "p98": math.nan,
        }

    p02, p98 = np.percentile(valid, [2.0, 98.0])
    scale = max(float(p98 - p02), np.finfo(np.float64).eps)
    preview = np.clip((scalar_depth - p02) / scale * 255.0, 0.0, 255.0)
    preview[~np.isfinite(scalar_depth) | (scalar_depth <= 0.0)] = 0.0
    return preview.astype(np.uint8), {
        "valid_pixel_count": int(valid.size),
        "minimum": float(np.min(valid)),
        "maximum": float(np.max(valid)),
        "p02": float(p02),
        "p98": float(p98),
    }


class RgbdSampleCollector(Node):
    """等待四类消息各一帧，不持续缓存图像，避免占用服务器内存。"""

    def __init__(self) -> None:
        super().__init__("simfordrone_rgbd_sample_collector")
        # Isaac Replicator 图像 writer 为 RELIABLE；订阅侧使用相同 QoS。
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Pegasus/PX4 状态话题实际以 Best Effort 发布。图像与状态不能共用 QoS，
        # 否则 Reliable 订阅者会拒绝该位姿发布者，导致无法比较图像/状态时间戳。
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.rgb: RosImage | None = None
        self.depth: RosImage | None = None
        self.camera_info: CameraInfo | None = None
        self.pose: PoseStamped | None = None
        self.create_subscription(RosImage, RGB_TOPIC, self._on_rgb, image_qos)
        self.create_subscription(RosImage, DEPTH_TOPIC, self._on_depth, image_qos)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, image_qos)
        self.create_subscription(PoseStamped, POSE_TOPIC, self._on_pose, pose_qos)

    def _on_rgb(self, message: RosImage) -> None:
        if self.rgb is None:
            self.rgb = message

    def _on_depth(self, message: RosImage) -> None:
        if self.depth is None:
            self.depth = message

    def _on_camera_info(self, message: CameraInfo) -> None:
        if self.camera_info is None:
            self.camera_info = message

    def _on_pose(self, message: PoseStamped) -> None:
        if self.pose is None:
            self.pose = message

    @property
    def complete(self) -> bool:
        return all((self.rgb, self.depth, self.camera_info, self.pose))


def save_sample(collector: RgbdSampleCollector, output_dir: Path) -> None:
    """验证并保存样本；任何编码或负载异常都明确失败，不生成伪成功结果。"""

    assert collector.rgb is not None
    assert collector.depth is not None
    assert collector.camera_info is not None
    assert collector.pose is not None

    rgb = image_to_array(collector.rgb)
    depth = image_to_array(collector.depth)
    if rgb.shape[:2] != depth.shape[:2]:
        raise ValueError(f"RGB/深度分辨率不一致: {rgb.shape[:2]} vs {depth.shape[:2]}")

    rgb_png = rgb_for_png(rgb, collector.rgb.encoding)
    depth_preview, depth_stats = depth_visualization(depth)
    output_dir.mkdir(parents=True, exist_ok=False)
    Image.fromarray(rgb_png, mode="RGB").save(output_dir / "rgb.png")
    Image.fromarray(depth_preview, mode="L").save(output_dir / "depth_preview.png")
    np.save(output_dir / "depth_raw.npy", depth)

    rgb_stamp = stamp_seconds(collector.rgb)
    depth_stamp = stamp_seconds(collector.depth)
    info_stamp = stamp_seconds(collector.camera_info)
    pose_stamp = stamp_seconds(collector.pose)
    metadata = {
        "topics": {
            "rgb": RGB_TOPIC,
            "depth": DEPTH_TOPIC,
            "camera_info": CAMERA_INFO_TOPIC,
            "pose": POSE_TOPIC,
        },
        "rgb": {
            "encoding": collector.rgb.encoding,
            "height": collector.rgb.height,
            "width": collector.rgb.width,
            "step": collector.rgb.step,
            "payload_bytes": len(collector.rgb.data),
            "frame_id": collector.rgb.header.frame_id,
            "stamp_seconds": rgb_stamp,
        },
        "depth": {
            "encoding": collector.depth.encoding,
            "height": collector.depth.height,
            "width": collector.depth.width,
            "step": collector.depth.step,
            "payload_bytes": len(collector.depth.data),
            "frame_id": collector.depth.header.frame_id,
            "stamp_seconds": depth_stamp,
            "statistics": depth_stats,
        },
        "camera_info": {
            "frame_id": collector.camera_info.header.frame_id,
            "stamp_seconds": info_stamp,
            "k": list(collector.camera_info.k),
            "distortion_model": collector.camera_info.distortion_model,
        },
        "pose": {
            "frame_id": collector.pose.header.frame_id,
            "stamp_seconds": pose_stamp,
        },
        "timestamp_deltas_ms": {
            "rgb_minus_depth": (rgb_stamp - depth_stamp) * 1e3,
            "rgb_minus_camera_info": (rgb_stamp - info_stamp) * 1e3,
            "rgb_minus_pose": (rgb_stamp - pose_stamp) * 1e3,
        },
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=True) + "\n",
        encoding="utf-8",
    )

    print(f"RGB:   {collector.rgb.encoding} {rgb.shape}, {len(collector.rgb.data)} bytes")
    print(f"Depth: {collector.depth.encoding} {depth.shape}, {len(collector.depth.data)} bytes")
    print("Depth valid pixels: {valid_pixel_count}, range: {minimum:.6g} .. {maximum:.6g}".format(**depth_stats))
    print("Timestamp deltas (ms): RGB-depth={rgb_minus_depth:.3f}, RGB-info={rgb_minus_camera_info:.3f}, RGB-pose={rgb_minus_pose:.3f}".format(**metadata["timestamp_deltas_ms"]))
    print(f"SAVED: {output_dir}")
    print("FILES: rgb.png, depth_preview.png, depth_raw.npy, metadata.json")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0, help="等待四类消息的最长秒数")
    parser.add_argument("--output-root", type=Path, default=Path("logs/rgbd_samples"), help="样本根目录")
    args = parser.parse_args()

    rclpy.init()
    collector = RgbdSampleCollector()
    deadline = time.monotonic() + args.timeout
    try:
        while not collector.complete and time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.25)
        if not collector.complete:
            missing = [
                name
                for name, value in (
                    ("RGB", collector.rgb),
                    ("Depth", collector.depth),
                    ("CameraInfo", collector.camera_info),
                    ("Pose", collector.pose),
                )
                if value is None
            ]
            raise TimeoutError(f"等待超时，未收到: {', '.join(missing)}")

        sample_dir = args.output_root / time.strftime("%Y%m%d-%H%M%S")
        save_sample(collector, sample_dir)
        return 0
    except Exception as error:
        print(f"FAIL: RGB-D 样本采集失败: {error}", file=sys.stderr)
        return 1
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
