#!/usr/bin/env python3
"""记录跟随机起飞验证的 ROS 位姿轨迹，并判断离地和落地条件。"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


POSE_TOPIC = "/tracker_uav_1/state/pose"


class PoseRecorder(Node):
    """用与 Pegasus 状态发布者兼容的 Best Effort QoS 记录 ENU 位姿。"""

    def __init__(self) -> None:
        super().__init__("simfordrone_tracker_takeoff_recorder")
        self.samples: list[dict[str, float | str]] = []
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PoseStamped, POSE_TOPIC, self._on_pose, qos)

    def _on_pose(self, message: PoseStamped) -> None:
        self.samples.append(
            {
                "wall_time": time.time(),
                "ros_stamp": float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) * 1e-9,
                "frame_id": message.header.frame_id,
                "x": float(message.pose.position.x),
                "y": float(message.pose.position.y),
                "z": float(message.pose.position.z),
            }
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--min-takeoff-height", type=float, default=0.8)
    parser.add_argument("--max-landed-height", type=float, default=0.30)
    parser.add_argument(
        "--ground-stable-seconds",
        type=float,
        default=1.0,
        help="起飞前必须已连续处于地面高度的最短记录时长。",
    )
    parser.add_argument("--output-root", type=Path, default=Path("logs/takeoff_validation"))
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration 必须大于 0")
    if args.ground_stable_seconds < 0.0:
        parser.error("--ground-stable-seconds 不能为负")

    rclpy.init()
    recorder = PoseRecorder()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(recorder, timeout_sec=0.25)
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

    output_dir = args.output_root / time.strftime("%Y%m%d-%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=False)
    csv_path = output_dir / "tracker_pose.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["wall_time", "ros_stamp", "frame_id", "x", "y", "z"],
        )
        writer.writeheader()
        writer.writerows(recorder.samples)

    if not recorder.samples:
        summary = {"result": "failed", "reason": "no_pose_messages", "sample_count": 0}
    else:
        z_values = [float(sample["z"]) for sample in recorder.samples]
        wall_times = [float(sample["wall_time"]) for sample in recorder.samples]

        # 场景刚重启时，机体会从 USD 初始高度自然落到地面。该下降过程不能被当成
        # “已起飞”。先取得稳定地面基线，再只检查基线之后是否真正离地。
        first_ground_index = next(
            (index for index, z_value in enumerate(z_values) if z_value <= args.max_landed_height),
            None,
        )
        baseline_index = None
        if first_ground_index is not None:
            first_ground_time = wall_times[first_ground_index]
            for index in range(first_ground_index, len(z_values)):
                if wall_times[index] - first_ground_time >= args.ground_stable_seconds:
                    baseline_index = index
                    break

        post_baseline_z = z_values[baseline_index:] if baseline_index is not None else []
        post_baseline_max = max(post_baseline_z) if post_baseline_z else None
        passed = (
            baseline_index is not None
            and post_baseline_max is not None
            and post_baseline_max >= args.min_takeoff_height
            and z_values[-1] <= args.max_landed_height
        )
        summary = {
            "result": "passed" if passed else "failed",
            "sample_count": len(recorder.samples),
            "z_start": z_values[0],
            "z_max": max(z_values),
            "z_final": z_values[-1],
            "ground_baseline_found": baseline_index is not None,
            "ground_baseline_elapsed_s": (
                wall_times[baseline_index] - wall_times[0]
                if baseline_index is not None
                else None
            ),
            "z_max_after_ground_baseline": post_baseline_max,
            "min_takeoff_height": args.min_takeoff_height,
            "max_landed_height": args.max_landed_height,
            "ground_stable_seconds": args.ground_stable_seconds,
        }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False))
    print(f"LOG: {output_dir}")
    return 0 if summary["result"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
