import math
import unittest

import numpy as np

from tracking.shadow_observation import (
    detected_relative_position_body,
    depth_roi_median,
    observation_time_skew,
    pixel_ray_body,
    project_oracle_pixel,
    range_from_optical_depth,
    relative_position_from_depth,
    track_depth_roi,
)


class ShadowObservationGeometryTest(unittest.TestCase):
    def test_pixel_bridge_interrupt_cleanup_is_idempotent(self) -> None:
        import sys
        from unittest.mock import Mock, patch
        try:
            from utils import bridge_target_pixel
        except ImportError as error:
            self.skipTest(f"System ROS environment required: {error}")
        node = Mock()
        with patch.object(sys, "argv", ["bridge"]), \
                patch.object(bridge_target_pixel, "TargetPixelBridge", return_value=node), \
                patch.object(bridge_target_pixel.rclpy, "init"), \
                patch.object(bridge_target_pixel.rclpy, "spin", side_effect=KeyboardInterrupt), \
                patch.object(bridge_target_pixel.rclpy, "try_shutdown") as shutdown:
            bridge_target_pixel.main()
        node.socket.close.assert_called_once()
        node.destroy_node.assert_called_once()
        shutdown.assert_called_once()

    def test_pixel_bridge_mixed_clocks_and_stale_inputs(self) -> None:
        from unittest.mock import Mock, patch
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import CameraInfo, Image
            from utils.bridge_target_pixel import TargetPixelBridge
        except ImportError as error:
            self.skipTest(f"System ROS environment required: {error}")
        import json

        rclpy.init()
        node = TargetPixelBridge(14602)
        node.socket.close()
        node.socket = Mock()
        try:
            with patch("utils.bridge_target_pixel.time.monotonic", return_value=10.0):
                info = CameraInfo()
                info.width, info.height = 1280, 720
                info.k = [921., 0., 640., 0., 921., 360., 0., 0., 1.]
                node.on_info(info)
                tracker = PoseStamped()
                tracker.pose.orientation.w = 1.
                tracker.header.stamp.sec = 1791560000
                node.on_tracker_pose(tracker)
                target = PoseStamped()
                target.pose.position.x = 5.3
                target.header.stamp.sec = 1791560000
                node.on_target_pose(target)
                rgb = Image()
                rgb.header.stamp.sec = 449
                rgb.width, rgb.height, rgb.encoding, rgb.step = 1280, 720, "rgb8", 3840
                colors = np.zeros((720, 1280, 3), dtype=np.uint8)
                colors[350:370, 630:650] = 255
                rgb.data = colors.tobytes()
                node.on_rgb(rgb)
                depth = Image()
                depth.header.stamp.sec = 449
                depth.width, depth.height, depth.encoding, depth.step = 1280, 720, "32FC1", 5120
                values = np.full((720, 1280), 20., dtype=np.float32)
                values[350:370, 630:650] = 5.
                depth.data = values.tobytes()
                node.on_depth(depth)
                node.socket.sendto.assert_called_once()
                packet = json.loads(node.socket.sendto.call_args.args[0])
                self.assertEqual(packet["source"], "rgbd_detection")
                np.testing.assert_allclose(packet["pixel"], (639.5, 359.5), atol=1.)
                node.socket.sendto.reset_mock()
                node.target_pose_received_at = 9.7
                node.on_depth(depth)
                self.assertEqual(node.rejections["stale_input_arrival"], 1)
                node.on_target_pose(target)
                depth.header.stamp.sec = 450
                node.on_depth(depth)
                self.assertEqual(node.rejections["rgb_depth_stamp_skew"], 1)
                depth.header.stamp.sec = 449
                target.header.stamp.sec -= 1
                node.on_depth(depth)
                self.assertEqual(node.rejections["pose_stamp_skew"], 1)
                node.socket.sendto.assert_not_called()
        finally:
            node.socket.close()
            node.destroy_node()
            rclpy.try_shutdown()

    def test_rgbd_shadow_tangent_update_with_mixed_clock_inputs(self) -> None:
        import sys
        import time
        from unittest.mock import Mock, patch
        try:
            import rclpy
            from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
            from sensor_msgs.msg import Image, Imu
            from utils.shadow_relative_ekf import ShadowRelativeEkf, parse_args
        except ImportError as error:
            self.skipTest(f"System ROS environment required: {error}")
        from tracking.state_io import StampedTargetState

        with patch.object(sys, "argv", ["shadow", "--state-port", "0"]):
            args = parse_args()
        rclpy.init()
        node = ShadowRelativeEkf(args)
        try:
            node.camera_matrix = np.array(((640., 0., 640.), (0., 640., 360.), (0., 0., 1.)))
            tracker = PoseStamped()
            tracker.pose.orientation.w = 1.
            tracker.header.stamp.sec = 1791560000
            node.on_tracker_pose(tracker)
            target = PoseStamped()
            target.pose.orientation.w = 1.
            target.pose.position.x = 5.3
            target.header.stamp.sec = 1791560000
            node.on_target_pose(target)
            for quantity, speed in (("observer_velocity", 2.), ("target_velocity", 6.)):
                velocity = TwistStamped()
                velocity.header.stamp.sec = 1791560000
                velocity.header.frame_id = "map"
                velocity.twist.linear.x = speed
                node.on_gt(quantity, velocity)
            acceleration = AccelStamped()
            acceleration.header.stamp.sec = 1791560000
            acceleration.header.frame_id = "map"
            acceleration.accel.linear.y = 3.
            node.on_gt("target_acceleration", acceleration)
            imu = Imu()
            imu.header.stamp.sec = 1791560000
            imu.linear_acceleration.z = -9.81
            node.on_imu(imu)
            rgb = Image()
            rgb.header.stamp.sec = 449
            rgb.width, rgb.height, rgb.encoding, rgb.step = 1280, 720, "rgb8", 3840
            colors = np.zeros((720, 1280, 3), dtype=np.uint8)
            colors[350:370, 630:650] = 255
            rgb.data = colors.tobytes()
            node.on_rgb(rgb)
            depth = Image()
            depth.header.stamp.sec = 449
            depth.width, depth.height, depth.encoding, depth.step = 1280, 720, "32FC1", 5120
            values = np.full((720, 1280), 20., dtype=np.float32)
            values[350:370, 630:650] = 5.
            depth.data = values.tobytes()
            node.subscriber.poll = Mock()
            node.subscriber.latest = StampedTargetState(
                time.monotonic(), 1, (5.3, 0., 0.), (0., 0., 0.),
                q=(0., 0., 0., 1.), attitude_timestamp=time.monotonic())
            with patch.object(node.filter, "_kalman_update", wraps=node.filter._kalman_update) as update:
                node.on_depth(depth)
                self.assertEqual(len(node.samples), 1, node.rejections)
                self.assertEqual([call.args[0].shape for call in update.call_args_list], [(3, 9), (2, 9)])
            np.testing.assert_allclose(node.samples[0]["observer_acceleration"], (0., 0., 0.), atol=1e-10)
            self.assertEqual(node.samples[0]["optical_depth_m"], 5.)
            np.testing.assert_allclose(node.samples[0]["truth"]["relative_velocity"], (4., 0., 0.))
            np.testing.assert_allclose(node.samples[0]["truth"]["target_acceleration"], (0., 3., 0.))
            node.gt_messages.clear()
            velocity_gt, acceleration_gt, _ = node.truth_kinematics(time.monotonic())
            self.assertIsNone(velocity_gt)
            self.assertIsNone(acceleration_gt)
            acceleration.header.stamp.sec -= 1
            node.on_gt("target_acceleration", acceleration)
            self.assertIsNone(node.truth_kinematics(time.monotonic())[1])
            acceleration.header.stamp.sec += 1
            acceleration.header.frame_id = "base_link"
            node.on_gt("target_acceleration", acceleration)
            self.assertIsNone(node.truth_kinematics(time.monotonic())[1])
            node.on_depth(depth)
            self.assertEqual(len(node.samples), 1)
            self.assertEqual(node.rejections["nonincreasing_image_stamp"], 1)
        finally:
            node.close()
            rclpy.try_shutdown()

    def test_observation_time_skew_uses_monotonic_arrivals(self) -> None:
        self.assertAlmostEqual(observation_time_skew(10.1, 10.0, 10.04, 10.08), 0.1)
        self.assertGreater(observation_time_skew(10.3, 10.0, 10.04, 10.08), 0.15)
        self.assertGreater(observation_time_skew(10.0, 10.1, 10.04, 10.08), 0.0)
        with self.assertRaises(ValueError):
            observation_time_skew(10.1, math.nan, 10.04, 10.08)

    def test_depth_roi_median_rejects_invalid_and_resists_outlier(self) -> None:
        depth = np.array(((0.0, 2.0, 2.0), (2.0, 100.0, np.nan), (2.0, 2.0, 2.0)))
        self.assertEqual(depth_roi_median(depth, 1.0, 1.0, 1), 2.0)
        self.assertIsNone(depth_roi_median(np.zeros((3, 3)), 1.0, 1.0, 1))

    def test_depth_roi_tracks_moving_patch_without_new_oracle_position(self) -> None:
        depth = np.full((31, 31), 8.0)
        depth[11:16, 14:19] = 4.0
        self.assertEqual(track_depth_roi(depth, 13.0, 13.0, 4.0, 6, 0.3), (16.0, 13.0, 4.0))
        self.assertIsNone(track_depth_roi(depth, 2.0, 2.0, 4.0, 4, 0.3))

    def test_center_pixel_points_forward_in_flu(self) -> None:
        ray = pixel_ray_body(640.0, 360.0, np.array(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0))))
        np.testing.assert_allclose(ray, (1.0, 0.0, 0.0), atol=1e-12)
        rear_ray = pixel_ray_body(
            640.0,
            360.0,
            np.array(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0))),
            -1.0,
        )
        np.testing.assert_allclose(rear_ray, (-1.0, 0.0, 0.0), atol=1e-12)

    def test_oracle_projection_matches_centered_forward_target(self) -> None:
        projection = project_oracle_pixel(
            target_world=(6.0, 0.0, 0.0),
            tracker_world=(0.0, 0.0, 0.0),
            tracker_attitude=(0.0, 0.0, 0.0, 1.0),
            camera_offset_body=(0.0, 0.0, 0.0),
            camera_matrix=np.array(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0))),
        )
        self.assertEqual(projection, (640.0, 360.0, 6.0))
        rear_projection = project_oracle_pixel(
            target_world=(-6.0, 0.0, 0.0),
            tracker_world=(0.0, 0.0, 0.0),
            tracker_attitude=(0.0, 0.0, 0.0, 1.0),
            camera_offset_body=(0.0, 0.0, 0.0),
            camera_matrix=np.array(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0))),
            camera_forward_sign=-1.0,
        )
        self.assertEqual(rear_projection, (640.0, 360.0, 6.0))

    def test_range_recovery_uses_forward_component_magnitude(self) -> None:
        """后向相机不得因前向分量为负而被算成负距离。

        这条断言对应 2026-09-28 的缺陷：``optical_depth / ray_body[0]`` 在
        ``camera_forward_sign=-1`` 时恒为负，导致交换部署下每一帧都以
        ``range_out_of_bounds`` 被拒绝，而相机图像本身是可用的。
        """

        matrix = np.array(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0)))
        self.assertAlmostEqual(range_from_optical_depth(6.0, pixel_ray_body(640.0, 360.0, matrix)), 6.0)
        self.assertAlmostEqual(
            range_from_optical_depth(6.0, pixel_ray_body(640.0, 360.0, matrix, -1.0)),
            6.0,
        )
        # 离轴 45 度：沿光轴深度小于欧氏距离，比值应为 1/cos(45)=sqrt(2)。
        self.assertAlmostEqual(
            range_from_optical_depth(1.0, pixel_ray_body(1280.0, 360.0, matrix)),
            math.sqrt(2.0),
        )
        # 非正深度与非有限值必须显式失败，不能静默变成可用量测。
        self.assertIsNone(range_from_optical_depth(0.0, pixel_ray_body(640.0, 360.0, matrix)))
        self.assertIsNone(range_from_optical_depth(-1.0, pixel_ray_body(640.0, 360.0, matrix)))
        self.assertIsNone(range_from_optical_depth(math.nan, pixel_ray_body(640.0, 360.0, matrix)))

    def test_metric_relative_position_includes_camera_mount_offset(self) -> None:
        ray = pixel_ray_body(640.0, 360.0, np.array(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0))))
        np.testing.assert_allclose(
            relative_position_from_depth(ray, 5.0, (0.3, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            (5.3, 0.0, 0.0),
        )

    def test_detected_pixel_depth_rejects_outside_image(self) -> None:
        matrix = np.array(((100.0, 0.0, 10.0), (0.0, 100.0, 10.0), (0.0, 0.0, 1.0)))
        depth = np.full((21, 21), 5.0)
        np.testing.assert_allclose(
            detected_relative_position_body(depth, 10.0, 10.0, matrix, (0.3, 0.0, 0.0)),
            (5.3, 0.0, 0.0),
        )
        self.assertIsNone(detected_relative_position_body(depth, 21.0, 10.0, matrix, (0.3, 0.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
