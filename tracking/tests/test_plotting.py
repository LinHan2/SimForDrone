"""飞行响应图的离线生成测试。"""

import tempfile
import unittest
from pathlib import Path

from px4ctrl.plotting import _plot_target_response, _plot_tracker_response, _plot_tracker_trajectory_3d, _target_samples, write_response_plot, write_tracker_estimation_plots
from tracking.shadow_plotting import _plot_nine_state_response, plot_observation


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

    def test_target_speed_plot_contains_actual_and_reference(self) -> None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plot

        samples = [
            {"t": timestamp, "actual_e": 0.0, "actual_n": 0.0, "actual_u": 2.0,
             "reference_e": 0.0, "reference_n": 0.0, "reference_u": 2.0,
             "actual_speed": speed, "reference_speed": 2.2, "error": 0.0}
            for timestamp, speed in ((0.0, 0.0), (1.0, 2.1))
        ]
        figure = _plot_target_response(plot, samples)
        try:
            speed_axis = next(axis for axis in figure.axes if axis.get_title() == "target speed")
            lines = speed_axis.lines
            self.assertEqual([line.get_label() for line in lines], ["target actual", "reference", "2 m/s"])
            self.assertEqual(list(lines[0].get_ydata()), [0.0, 2.1])
        finally:
            plot.close(figure)

    def test_tracker_distance_plot_contains_actual_and_desired(self) -> None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plot

        samples = [{"t": 0.0, "target_e": 5.0, "target_n": 0.0, "target_u": 2.0,
                    "desired_e": 0.0, "desired_n": 0.0, "desired_u": 2.0,
                    "tracker_e": 0.0, "tracker_n": 0.0, "tracker_u": 2.0,
                    "horizontal_distance": 5.2, "desired_distance": 5.0,
                    "error": 0.2, "measurement_error": 0.0}]
        figure = _plot_tracker_response(plot, samples, samples)
        try:
            distance_axis = next(axis for axis in figure.axes if axis.get_title() == "horizontal follow distance")
            self.assertEqual([line.get_label() for line in distance_axis.lines], ["actual", "desired"])
        finally:
            plot.close(figure)

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
            "task": "target-trajectory",
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
                    "actual_speed": 0.0,
                    "reference_speed": 0.0,
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
                    "actual_speed": 0.5,
                    "reference_speed": 0.5,
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

    def test_tracker_estimation_plots_cover_nine_axes(self) -> None:
        record = {
            "task": "tracker-v0",
            "samples": [
                {
                    "t": 0.0,
                    "error": 0.1,
                    "measurement_error": 0.02,
                    "estimate_error": 0.03,
                    "target_e": 1.0, "target_n": 2.0, "target_u": 3.0,
                    "target_ve": 0.0, "target_vn": 0.0, "target_vu": 0.0,
                    "target_ae": 0.0, "target_an": 0.0, "target_au": 0.0,
                    "measurement_e": 1.02, "measurement_n": 1.98, "measurement_u": 3.01,
                    "estimate_e": 1.0, "estimate_n": 2.0, "estimate_u": 3.0,
                    "estimate_ve": 0.0, "estimate_vn": 0.0, "estimate_vu": 0.0,
                    "estimate_ae": 0.0, "estimate_an": 0.0, "estimate_au": 0.0,
                    "tracker_e": 0.0, "tracker_n": 2.0, "tracker_u": 3.0,
                    "desired_e": 0.0, "desired_n": 2.0, "desired_u": 3.0,
                },
                {
                    "t": 1.0,
                    "error": 0.0,
                    "measurement_error": 0.0,
                    "estimate_error": 0.01,
                    "target_e": 2.0, "target_n": 2.2, "target_u": 3.0,
                    "target_ve": 1.0, "target_vn": 0.2, "target_vu": 0.0,
                    "target_ae": 1.0, "target_an": 0.2, "target_au": 0.0,
                    "measurement_e": 1.98, "measurement_n": 2.21, "measurement_u": 2.99,
                    "estimate_e": 1.99, "estimate_n": 2.2, "estimate_u": 3.0,
                    "estimate_ve": 0.8, "estimate_vn": 0.2, "estimate_vu": 0.0,
                    "estimate_ae": 0.8, "estimate_an": 0.2, "estimate_au": 0.0,
                    "tracker_e": 1.0, "tracker_n": 2.0, "tracker_u": 3.0,
                    "desired_e": 1.0, "desired_n": 2.0, "desired_u": 3.0,
                },
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory) / "tracking-run"
            paths = write_tracker_estimation_plots(output_dir, record)
            self.assertEqual(len(paths), 13)
            for path in paths:
                self.assertTrue(path.is_file(), str(path))
                self.assertGreater(path.stat().st_size, 0, str(path))
            for quantity in ("position", "velocity", "acceleration"):
                for axis in ("east", "north", "up"):
                    self.assertTrue((output_dir / "figures" / f"{quantity}_{axis}.png").is_file())
            for name in (
                "tracker_trajectory_3d.png",
                "tracker_horizontal_trajectory.png",
                "tracker_position_tracking.png",
                "tracker_error.png",
            ):
                self.assertTrue((output_dir / "figures" / name).is_file())

            for sample in record["samples"]:
                sample["target_ae"] = float("nan")
                sample["target_an"] = float("nan")
                sample["target_au"] = float("nan")
            record["target_samples"] = record["samples"]
            self.assertEqual(len(write_tracker_estimation_plots(output_dir, record)), 13)

    def test_3d_trajectory_keeps_enu_altitude_and_distinct_streams(self) -> None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plot

        samples = [{"tracker_e": -5.0, "tracker_n": 0.0, "tracker_u": 2.0,
                    "desired_e": -4.9, "desired_n": 0.1, "desired_u": 2.1}]
        target_samples = [{"target_e": 5.0, "target_n": 0.0, "target_u": 2.05},
                          {"target_e": 5.1, "target_n": 0.2, "target_u": 2.06}]
        figure = _plot_tracker_trajectory_3d(plot, samples, target_samples)
        try:
            axis = figure.axes[0]
            self.assertEqual(axis.name, "3d")
            self.assertEqual(axis.get_zlabel(), "Z / Up (m)")
            self.assertEqual(list(axis.lines[0].get_data_3d()[2]), [2.05, 2.06])
            self.assertEqual(list(axis.lines[1].get_data_3d()[2]), [2.0])
            self.assertEqual(list(axis.lines[2].get_data_3d()[2]), [2.1])
            self.assertLessEqual(axis.get_zlim()[0], 2.0)
            self.assertGreaterEqual(axis.get_zlim()[1], 2.1)
        finally:
            plot.close(figure)

    def test_shadow_nine_state_axes_preserve_missing_values_and_gaps(self) -> None:
        import matplotlib
        import math

        matplotlib.use("Agg")
        import matplotlib.pyplot as plot

        record = {"samples": [
            {"t": timestamp, "position_error_m": 0.1,
             "measurement_relative_position": [9.9, 0.0, 0.1],
             "truth": {"relative_position": [10.0, 0.0, 0.1],
                       "relative_velocity": None if timestamp == 2.0 else [0.1, 0.2, 0.3],
                       "target_acceleration": None},
             "estimate": {"relative_position": [9.9, 0.0, 0.1],
                          "relative_velocity": [0.4, 0.5, 0.6],
                          "target_acceleration": [0.7, 0.8, 0.9]}}
            for timestamp in (2.0, 2.1, 3.0)
        ]}
        figure = _plot_nine_state_response(plot, record)
        try:
            self.assertEqual(len(figure.axes), 9)
            self.assertIn("Z / Up", figure.axes[2].get_title())
            self.assertEqual(list(figure.axes[0].lines[0].get_xdata()), [2.0, 2.1])
            self.assertEqual(list(figure.axes[0].lines[1].get_xdata()), [3.0])
            self.assertTrue(math.isnan(figure.axes[3].lines[0].get_ydata()[0]))
            self.assertEqual(figure.axes[3].lines[0].get_ydata()[1], 0.1)
            self.assertTrue(all(math.isnan(value) for value in figure.axes[6].lines[0].get_ydata()))
            self.assertEqual(list(figure.axes[8].lines[2].get_ydata()), [0.9, 0.9])
            self.assertIn("proxy", figure.axes[3].lines[0].get_label())
        finally:
            plot.close(figure)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "shadow-observation.png"
            plot_observation(record, output)
            self.assertTrue(output.is_file())
            self.assertGreater(output.with_name("shadow-kinematics.png").stat().st_size, 0)
        with self.assertRaises(ValueError):
            _plot_nine_state_response(plot, {"samples": []})

    def test_unique_target_stream_overrides_held_control_samples(self) -> None:
        held_control_samples = [{"t": 0.0, "target_e": 1.0}, {"t": 0.2, "target_e": 1.0}]
        unique_target_samples = [{"t": 0.0, "target_e": 1.0}, {"t": 0.1, "target_e": 1.1}]

        selected = _target_samples(
            {"target_samples": unique_target_samples}, held_control_samples
        )

        self.assertIs(selected, unique_target_samples)
        self.assertTrue(all(
            later["t"] > earlier["t"]
            for earlier, later in zip(selected, selected[1:])
        ))

    def _assert_plot(self, record: dict) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory) / "tracking-run"
            picture_dir = Path(directory) / "picture"
            path = write_response_plot(output_dir, record, picture_dir=picture_dir)
            self.assertIsNotNone(path)
            self.assertTrue(path.is_file())
            self.assertGreater(path.stat().st_size, 0)
            picture_path = picture_dir / "tracking-run.png"
            self.assertTrue(picture_path.is_file())
            self.assertGreater(picture_path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
