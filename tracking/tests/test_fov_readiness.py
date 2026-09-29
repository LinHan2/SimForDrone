from __future__ import annotations

import unittest

import numpy as np

from tracking.fov_readiness import evaluate_oracle_fov_depth


class FovReadinessTest(unittest.TestCase):
    def setUp(self) -> None:
        self.matrix = np.array(((100.0, 0.0, 50.0), (0.0, 100.0, 50.0), (0.0, 0.0, 1.0)))
        self.common = {
            "tracker_world": (0.0, 0.0, 0.0),
            "tracker_attitude": (0.0, 0.0, 0.0, 1.0),
            "camera_offset_body": (0.0, 0.0, 0.0),
            "camera_matrix": self.matrix,
            "roi_radius_px": 1,
            "max_range_m": 10.0,
            "max_oracle_range_error_m": 0.2,
        }

    def test_accepts_centered_target_with_consistent_depth(self) -> None:
        result = evaluate_oracle_fov_depth(
            target_world=(2.0, 0.0, 0.0),
            depth=np.full((100, 100), 2.0),
            **self.common,
        )

        self.assertTrue(result.ready)
        self.assertEqual(result.reason, "ready")
        self.assertEqual((result.pixel_u, result.pixel_v), (50.0, 50.0))
        self.assertEqual(result.oracle_range_m, 2.0)

    def test_rejects_target_outside_image(self) -> None:
        result = evaluate_oracle_fov_depth(
            target_world=(2.0, -2.0, 0.0),
            depth=np.full((100, 100), 2.0),
            **self.common,
        )

        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "target_outside_image")

    def test_rejects_inconsistent_depth(self) -> None:
        result = evaluate_oracle_fov_depth(
            target_world=(2.0, 0.0, 0.0),
            depth=np.full((100, 100), 4.0),
            **self.common,
        )

        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "depth_oracle_disagreement")


if __name__ == "__main__":
    unittest.main()