"""五次多项式航点轨迹的单元测试。

重点验证三件事，它们直接决定位置环会不会饱和：
1. 首末位置精确、首末速度与加速度为零；
2. 位置单调、不超调；
3. 实际峰值速度/加速度不超过给定上限。
"""

from __future__ import annotations

import math
import unittest

from tracking.trajectory import (
    TrajectoryCycle,
    WaypointSegment,
    quintic_reference,
    segment_duration,
)


class SegmentDurationTest(unittest.TestCase):
    def test_zero_delta_returns_positive_minimum(self) -> None:
        self.assertGreater(segment_duration((0.0, 0.0, 0.0), 1.0, 1.0), 0.0)

    def test_rejects_non_positive_limits(self) -> None:
        with self.assertRaises(ValueError):
            segment_duration((1.0, 0.0, 0.0), 0.0, 1.0)
        with self.assertRaises(ValueError):
            segment_duration((1.0, 0.0, 0.0), 1.0, -1.0)

    def test_longer_distance_needs_longer_time(self) -> None:
        short = segment_duration((1.0, 0.0, 0.0), 1.0, 1.0)
        long = segment_duration((4.0, 0.0, 0.0), 1.0, 1.0)

        self.assertGreater(long, short)


class QuinticReferenceTest(unittest.TestCase):
    def test_boundary_conditions(self) -> None:
        start = (0.0, 0.0, 2.0)
        goal = (3.0, 0.0, 2.0)
        duration = 4.0

        p0, v0, a0 = quintic_reference(start, goal, duration, 0.0)
        p1, v1, a1 = quintic_reference(start, goal, duration, duration)

        self.assertEqual(p0, start)
        self.assertEqual(p1, goal)
        for value in (*v0, *a0, *v1, *a1):
            self.assertAlmostEqual(value, 0.0, places=9)

    def test_is_monotonic_without_overshoot(self) -> None:
        start = (0.0, 0.0, 0.0)
        goal = (3.0, 0.0, 0.0)
        duration = 4.0

        previous = start[0]
        for step in range(1, 41):
            p, _, _ = quintic_reference(start, goal, duration, duration * step / 40.0)
            self.assertGreaterEqual(p[0], previous - 1e-12)
            self.assertLessEqual(p[0], goal[0] + 1e-12)
            previous = p[0]

    def test_peak_speed_and_accel_respect_limits(self) -> None:
        # 这正是"不饱和"的量化保证：峰值不得超过给定上限。
        max_speed = 0.8
        max_accel = 1.0
        start = (0.0, 0.0, 0.0)
        goal = (3.0, 0.0, 0.5)
        segment = WaypointSegment(start, goal, max_speed, max_accel)

        peak_speed = 0.0
        peak_accel = 0.0
        for step in range(1, 200):
            _, velocity, acceleration = segment.sample(segment.duration * step / 200.0)
            peak_speed = max(peak_speed, math.dist((0.0, 0.0, 0.0), velocity))
            peak_accel = max(peak_accel, math.dist((0.0, 0.0, 0.0), acceleration))

        self.assertLessEqual(peak_speed, max_speed + 1e-6)
        self.assertLessEqual(peak_accel, max_accel + 1e-6)

    def test_holds_goal_after_duration(self) -> None:
        goal = (3.0, 1.0, 2.0)

        p, v, a = quintic_reference((0.0, 0.0, 0.0), goal, 2.0, 5.0)

        self.assertEqual(p, goal)
        self.assertEqual(v, (0.0, 0.0, 0.0))
        self.assertEqual(a, (0.0, 0.0, 0.0))

    def test_rejects_non_positive_duration(self) -> None:
        with self.assertRaises(ValueError):
            quintic_reference((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.0, 0.0)


class TrajectoryCycleTest(unittest.TestCase):
    def test_closed_trajectories_have_zero_boundary_derivatives(self) -> None:
        for pattern in ("circle", "figure8", "helix"):
            trajectory = TrajectoryCycle(pattern, 1.0, 0.3, 0.25, 0.25)
            for elapsed in (0.0, trajectory.duration):
                position, velocity, acceleration = trajectory.sample(elapsed)
                self.assertEqual(position, (0.0, 0.0, 0.0))
                self.assertEqual(velocity, (0.0, 0.0, 0.0))
                self.assertEqual(acceleration, (0.0, 0.0, 0.0))

    def test_trajectory_respects_kinematic_limits(self) -> None:
        max_speed, max_accel = 0.25, 0.25
        for pattern in ("circle", "figure8", "helix"):
            trajectory = TrajectoryCycle(pattern, 1.0, 0.3, max_speed, max_accel)
            peak_speed = 0.0
            peak_accel = 0.0
            for step in range(1, 1_000):
                _, velocity, acceleration = trajectory.sample(trajectory.duration * step / 1_000)
                peak_speed = max(peak_speed, math.dist((0.0, 0.0, 0.0), velocity))
                peak_accel = max(peak_accel, math.dist((0.0, 0.0, 0.0), acceleration))
            self.assertLessEqual(peak_speed, max_speed + 1e-6, pattern)
            self.assertLessEqual(peak_accel, max_accel + 1e-6, pattern)


if __name__ == "__main__":
    unittest.main()
