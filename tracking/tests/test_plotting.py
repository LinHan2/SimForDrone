"""飞行响应图的离线生成测试。"""

import tempfile
import unittest
from pathlib import Path

from px4ctrl.plotting import write_response_plot


class ResponsePlotTest(unittest.TestCase):
    def test_step_response_plot_is_written(self) -> None:
        record = {
            "task": "step-response",
            "role": "tracker",
            "step": {"axis": "east", "amplitude_m": 0.5},
            "samples": [
                {
                    "t": 0.0,
                    "position": 0.0,
                    "velocity": 0.0,
                    "roll_rad": 0.0,
                    "pitch_rad": 0.1,
                    "bodyrate_roll": 0.0,
                    "bodyrate_pitch": 0.2,
                    "bodyrate_yaw": 0.0,
                },
                {
                    "t": 1.0,
                    "position": 0.5,
                    "velocity": 0.0,
                    "roll_rad": 0.0,
                    "pitch_rad": 0.0,
                    "bodyrate_roll": 0.0,
                    "bodyrate_pitch": 0.0,
                    "bodyrate_yaw": 0.0,
                },
            ],
        }
        self._assert_plot(record)

    def test_tracker_plot_is_written(self) -> None:
        record = {
            "task": "tracker-v0",
            "samples": [
                {
                    "t": 0.0,
                    "error": 0.1,
                    "measurement_error": 0.0,
                    "target_e": 1.0,
                    "target_n": 2.0,
                    "target_u": 3.0,
                    "tracker_e": 0.0,
                    "tracker_n": 2.0,
                    "tracker_u": 3.0,
                    "desired_e": 0.0,
                    "desired_n": 2.0,
                    "desired_u": 3.0,
                },
                {
                    "t": 1.0,
                    "error": 0.0,
                    "measurement_error": 0.0,
                    "target_e": 2.0,
                    "target_n": 2.0,
                    "target_u": 3.0,
                    "tracker_e": 1.0,
                    "tracker_n": 2.0,
                    "tracker_u": 3.0,
                    "desired_e": 1.0,
                    "desired_n": 2.0,
                    "desired_u": 3.0,
                },
            ],
        }
        self._assert_plot(record)

    def test_target_plot_is_written(self) -> None:
        record = {
            "task": "target-waypoints",
            "samples": [
                {
                    "t": 0.0,
                    "error": 0.5,
                    "actual_e": 0.0,
                    "actual_n": 0.0,
                    "actual_u": 2.0,
                    "reference_e": 0.0,
                    "reference_n": 0.0,
                    "reference_u": 2.0,
                },
                {
                    "t": 1.0,
                    "error": 0.0,
                    "actual_e": 0.5,
                    "actual_n": 0.0,
                    "actual_u": 2.0,
                    "reference_e": 0.5,
                    "reference_n": 0.0,
                    "reference_u": 2.0,
                },
            ],
        }
        self._assert_plot(record)

    def test_hover_plot_is_written(self) -> None:
        record = {
            "task": "hold",
            "role": "tracker",
            "samples": [
                {"t": 0.0, "altitude": 1.8, "error": 0.2, "thrust": 0.29},
                {"t": 1.0, "altitude": 2.0, "error": 0.0, "thrust": 0.29},
            ],
        }
        self._assert_plot(record)

    def _assert_plot(self, record: dict) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = write_response_plot(Path(directory), record)
            self.assertIsNotNone(path)
            self.assertTrue(path.is_file())
            self.assertGreater(path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
