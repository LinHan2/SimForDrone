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
