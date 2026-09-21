"""target 航点任务的输入解析与安全默认值测试。"""

import unittest
from unittest.mock import patch

from tracking.target_waypoints import parse_args, parse_interactive_command


class TargetWaypointTest(unittest.TestCase):
    def test_parse_interactive_waypoint(self) -> None:
        self.assertEqual(parse_interactive_command("1 2 3"), ("waypoint", (1.0, 2.0, 3.0)))

    def test_probe_duration_default_is_long_enough_for_second_process(self) -> None:
        with patch("sys.argv", ["target_waypoints.py"]):
            self.assertEqual(parse_args().probe_seconds, 15.0)

        with patch("sys.argv", ["target_waypoints.py", "--probe-seconds", "3"]):
            self.assertEqual(parse_args().probe_seconds, 3.0)

    def test_default_waypoint_limits_reserve_tilt_budget_for_feedback(self) -> None:
        with patch("sys.argv", ["target_waypoints.py"]):
            args = parse_args()

        self.assertEqual(args.max_speed, 0.25)
        self.assertEqual(args.max_accel, 0.25)


if __name__ == "__main__":
    unittest.main()