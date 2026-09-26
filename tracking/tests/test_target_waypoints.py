"""target 航点任务的输入解析与安全默认值测试。"""

import math
import unittest
from unittest.mock import patch

from tracking.target_waypoints import (
    parse_args,
    parse_interactive_command,
    reference_from_trajectory,
)
from tracking.trajectory import TrajectoryCycle


class TargetWaypointTest(unittest.TestCase):
    def test_parse_interactive_waypoint(self) -> None:
        self.assertEqual(parse_interactive_command("1 2 3"), ("waypoint", (1.0, 2.0, 3.0)))

    def test_direct_execution_is_default_and_probe_is_explicit(self) -> None:
        with patch("sys.argv", ["target_waypoints.py"]):
            self.assertFalse(parse_args().probe)

        with patch("sys.argv", ["target_waypoints.py", "--probe"]):
            self.assertTrue(parse_args().probe)

        with patch("sys.argv", ["target_waypoints.py", "--execute"]):
            self.assertFalse(parse_args().probe)

    def test_default_waypoint_limits_reserve_tilt_budget_for_feedback(self) -> None:
        with patch("sys.argv", ["target_waypoints.py"]):
            args = parse_args()

        self.assertEqual(args.max_speed, 0.25)
        self.assertEqual(args.max_accel, 0.25)

    def test_trajectory_arguments_are_parsed(self) -> None:
        with patch("sys.argv", ["target_waypoints.py", "--trajectory", "figure8", "--trajectory-cycles", "2"]):
            args = parse_args()

        self.assertEqual(args.trajectory, "figure8")
        self.assertEqual(args.trajectory_cycles, 2)

    def test_trajectory_reference_is_relative_to_hover_origin(self) -> None:
        trajectory = TrajectoryCycle("circle", 1.0, 0.0, 0.25, 0.25)

        position, velocity, acceleration = reference_from_trajectory(
            trajectory, (3.0, -2.0, 1.5), trajectory.duration / 2.0
        )

        self.assertNotEqual(position, (3.0, -2.0, 1.5))
        self.assertGreater(math.dist((0.0, 0.0, 0.0), velocity), 0.0)
        self.assertTrue(all(math.isfinite(value) for value in acceleration))


if __name__ == "__main__":
    unittest.main()