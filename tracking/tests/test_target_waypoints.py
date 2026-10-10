"""target 航点任务的输入解析与安全默认值测试。"""

import math
from pathlib import Path
import unittest
from unittest.mock import patch

import yaml

from tracking.target_waypoints import (
    parse_args,
    parse_interactive_command,
    reference_from_trajectory,
    trajectory_elapsed,
)
from tracking.trajectory import TrajectoryCycle


class TargetWaypointTest(unittest.TestCase):
    def test_high_speed_experiment_reference_exceeds_five_meters_per_second(self):
        path = Path(__file__).resolve().parents[2] / "tracking/config/experiment.yaml"
        config = yaml.safe_load(path.read_text())["target"]
        trajectory = TrajectoryCycle(config["trajectory"], config["trajectory_radius"],
                                     config["trajectory_vertical_amplitude"],
                                     config["max_speed"], config["max_accel"])
        samples = [trajectory.sample(trajectory.duration * index / 1000) for index in range(1001)]
        peak_speed = max(math.hypot(*velocity) for _, velocity, _ in samples)
        peak_acceleration = max(math.hypot(*acceleration) for _, _, acceleration in samples)
        self.assertGreater(peak_speed, 5.0)
        self.assertLessEqual(peak_speed, config["max_speed"] + 1e-9)
        self.assertLessEqual(peak_acceleration, config["max_accel"] + 1e-9)
    def test_trajectory_uses_physical_sample_time(self) -> None:
        self.assertAlmostEqual(trajectory_elapsed(10.25, 10.0), 0.25)
        self.assertEqual(trajectory_elapsed(10.25, 10.0), trajectory_elapsed(10.25, 10.0))
        for source, started in ((None, 10.0), (10.0, None), (9.0, 10.0), (math.nan, 10.0)):
            with self.subTest(source=source, started=started):
                with self.assertRaises(RuntimeError):
                    trajectory_elapsed(source, started)

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

        self.assertEqual(args.max_speed, 0.20)
        self.assertEqual(args.max_accel, 0.15)

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