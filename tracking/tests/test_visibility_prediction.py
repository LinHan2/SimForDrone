import json
import csv
import math
import tempfile
import unittest
from dataclasses import asdict, replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from tracking.guidance import TargetState
from tracking.shadow_observation import project_oracle_pixel, rotate_vector
from tracking.visibility_prediction import CameraGeometry, VisibilityConfig, VisibilityPredictor
from tracking.visibility_diagnostics import VisibilityDiagnostics, _flat_row, export_visibility, summarize_tracking


class TrackingAssessmentTest(unittest.TestCase):
    @staticmethod
    def row(timestamp, status="safe", normal=(0.0, 0.0), valid=True):
        return {"t": timestamp, "valid": valid, "predicted_fov_exit": True,
                "attitude_predictions": {"constant_attitude": {
                    "projection_status": [status], "normalized_pixels_pred": [normal]}}}

    def test_current_visibility_not_future_prediction_or_position_error(self):
        record = {"visibility_samples": [self.row(value) for value in (0.0, 0.1, 0.2)],
                  "error_max_m": 4.0}
        result = summarize_tracking(record)
        self.assertTrue(result["fov_passed"])
        self.assertEqual(result["geometric_loss_fraction"], 0.0)
        self.assertEqual(result["status"], "requires_stability_review")
        self.assertFalse(result["position_error_is_acceptance_gate"])
        json.dumps(result, allow_nan=False)

    def test_loss_duration_and_central_fraction_are_time_weighted(self):
        rows = [self.row(0.0, "behind_camera", None), self.row(0.05, "outside", (1.2, 0.0)),
                self.row(0.15), self.row(0.2)]
        result = summarize_tracking({"visibility_samples": rows})
        self.assertAlmostEqual(result["geometric_loss_fraction"], 0.75)
        self.assertAlmostEqual(result["longest_observed_loss_s"], 0.15)
        self.assertFalse(result["fov_passed"])

    def test_missing_and_short_recording_cannot_pass(self):
        rows = [self.row(0.0), self.row(0.1), self.row(0.2, valid=False), self.row(3.0)]
        result = summarize_tracking({"visibility_samples": rows,
                                     "control_samples": [{"t": 0.0}, {"t": 10.0}]})
        self.assertAlmostEqual(result["valid_coverage_fraction"], 0.01)
        self.assertIsNone(result["fov_passed"])
        self.assertEqual(result["status"], "not_evaluated")
        self.assertIsNone(summarize_tracking({})["fov_passed"])

    def test_attitude_and_limit_diagnostics_use_full_control_samples(self):
        rows = [{"t": 0.0, "actual_roll": 0.0, "jerk_limited": True},
                {"t": 0.1, "actual_roll": math.radians(2), "jerk_limited": False}]
        result = summarize_tracking({"control_samples": rows})
        self.assertAlmostEqual(result["actual_roll_peak_deg"], 2.0)
        self.assertAlmostEqual(result["actual_roll_rate_rms_deg_s"], 20.0)
        self.assertEqual(result["jerk_limited_sample_fraction"], 0.5)


class VisibilityPredictorTest(unittest.TestCase):
    def setUp(self):
        self.camera = CameraGeometry(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0)),
                                     1280, 720, (0.3, 0.0, 0.0))
        self.predictor = VisibilityPredictor(self.camera)
        self.tracker = TargetState((0.0, 0.0, 0.0))
        self.attitude = (0.0, 0.0, 0.0, 1.0)

    def predict(self, target, **kwargs):
        return self.predictor.predict(target, self.tracker, self.attitude, **kwargs)

    def test_stationary_target_and_tracker(self):
        prediction = self.predict(TargetState((6.0, 0.0, 0.0)))
        self.assertTrue(prediction.valid)
        self.assertEqual(len(prediction.prediction_times), 17)
        self.assertEqual(prediction.min_distance, 6.0)
        self.assertEqual(prediction.max_distance, 6.0)
        self.assertTrue(all(pixel == (640.0, 360.0) for pixel in prediction.pixels_pred))
        self.assertFalse(prediction.predicted_fov_exit)
        self.assertIsNone(prediction.predicted_fov_exit_time_sec)

    def test_lateral_motion_projects_left_for_positive_body_y(self):
        prediction = self.predict(TargetState((6.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
        for left, right in zip(prediction.pixels_pred, prediction.pixels_pred[1:]):
            assert left is not None and right is not None
            self.assertGreater(left[0], right[0])

    def test_approaching_and_receding_distance(self):
        for velocity, direction in ((-1.0, -1.0), (1.0, 1.0)):
            with self.subTest(velocity=velocity):
                prediction = self.predict(TargetState((6.0, 0.0, 0.0), (velocity, 0.0, 0.0)))
                self.assertTrue(all(direction * (right - left) > 0 for left, right in
                                    zip(prediction.distances_pred, prediction.distances_pred[1:])))

    def test_predicts_exit_half_second_in_advance(self):
        prediction = self.predict(TargetState((5.3, 0.0, 0.0), (0.0, -10.0, 0.0)))
        self.assertTrue(prediction.predicted_fov_exit)
        self.assertEqual(prediction.predicted_fov_exit_time_sec, 0.5)
        self.assertFalse(prediction.attitude_predictions["constant_attitude"].out_of_fov[0])

    def test_optical_axis_with_rotated_body_and_camera_translation(self):
        attitude = (0.0, math.sin(0.12), 0.0, math.cos(0.12))
        tracker = TargetState((4.0, 3.0, 2.0))
        target = TargetState(tuple(np.asarray(tracker.p) + rotate_vector(attitude, (6.3, 0.0, 0.0))))
        prediction = self.predictor.predict(target, tracker, attitude)
        np.testing.assert_allclose(prediction.pixels_pred, [(640.0, 360.0)] * 17, atol=1e-10)
        np.testing.assert_allclose(prediction.relative_positions_camera[0], (0, 0, 6), atol=1e-10)
        assert prediction.current_distance is not None
        self.assertAlmostEqual(prediction.current_distance, 6.3)
        expected = project_oracle_pixel(target.p, tracker.p, attitude, self.camera.offset_body,
                                       np.asarray(self.camera.intrinsics))
        assert expected is not None
        np.testing.assert_allclose(prediction.pixels_pred[0], expected[:2])

    def test_constant_acceleration_and_unreliable_fallback(self):
        target = TargetState((6.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0))
        tracker = TargetState((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0))
        prediction = self.predictor.predict(target, tracker, self.attitude)
        self.assertAlmostEqual(prediction.target_positions_pred[-1][0], 7.44)
        self.assertAlmostEqual(prediction.target_velocities_pred[-1][0], 2.6)
        self.assertAlmostEqual(prediction.distances_pred[-1], 6.0)
        fallback = self.predictor.predict(target, tracker, self.attitude, tracker_acceleration_reliable=False)
        self.assertEqual(fallback.translation_mode_used, "constant_velocity")
        self.assertAlmostEqual(fallback.tracker_positions_pred[-1][0], 0.8)
        invalid_acceleration = replace(tracker, a=(math.nan, 0.0, 0.0))
        self.assertEqual(self.predictor.predict(target, invalid_acceleration, self.attitude).translation_mode_used,
                         "constant_velocity")

    def test_commanded_attitude_is_comparison_not_state_mutation(self):
        target = TargetState((6.0, 0.0, 0.0))
        commanded = (0.0, math.sin(0.35), 0.0, math.cos(0.35))
        prediction = self.predict(target, commanded_attitude=commanded)
        constant = prediction.attitude_predictions["constant_attitude"]
        command = prediction.attitude_predictions["command_based_attitude"]
        self.assertEqual(command.pixels_pred[0], constant.pixels_pred[0])
        self.assertNotEqual(command.pixels_pred[1], constant.pixels_pred[1])
        self.assertTrue(command.predicted_fov_exit)
        self.assertFalse(constant.predicted_fov_exit)
        self.assertEqual(target, TargetState((6.0, 0.0, 0.0)))

    def test_behind_camera_is_exit_not_invalid_state(self):
        prediction = self.predict(TargetState((-1.0, 0.0, 0.0)))
        self.assertTrue(prediction.valid)
        self.assertEqual(prediction.predicted_fov_exit_time_sec, 0.0)
        self.assertTrue(all(prediction.attitude_predictions["constant_attitude"].behind_camera))
        self.assertTrue(all(pixel is None for pixel in prediction.pixels_pred))
        self.assertIsNone(prediction.max_image_error)
        json.dumps(asdict(prediction), allow_nan=False)

    def test_cost_dead_zones_and_invalid_state(self):
        self.assertEqual(self.predict(TargetState((7.0, 0.0, 0.0))).distance_cost, 0.0)
        distance_cost = self.predict(TargetState((12.0, 0.0, 0.0))).distance_cost
        assert distance_cost is not None
        self.assertGreater(distance_cost, 0.0)
        self.assertEqual(self.predict(TargetState((7.0, 0.0, 0.0))).fov_cost, 0.0)
        fov_cost = self.predict(TargetState((7.0, 5.0, 0.0))).fov_cost
        assert fov_cost is not None
        self.assertGreater(fov_cost, 0.0)
        self.assertFalse(self.predict(TargetState((math.nan, 0.0, 0.0))).valid)
        self.assertFalse(self.predictor.predict(TargetState((6.0, 0.0, 0.0)), self.tracker, (0, 0, 0, 0)).valid)

    def test_configuration_and_nonuniform_diagnostic_anchors(self):
        predictor = VisibilityPredictor(self.camera, VisibilityConfig(prediction_dt=0.07, prediction_horizon_sec=0.83))
        for value in (0.0, 0.2, 0.5, 0.83):
            self.assertIn(value, predictor.times)
        self.assertEqual(len(predictor.times), len(set(predictor.times)))
        for factory in (lambda: VisibilityConfig(prediction_dt=0),
                        lambda: VisibilityConfig(prediction_horizon_sec=math.inf),
                        lambda: VisibilityConfig(fov_safe_ratio=0.9),
                        lambda: VisibilityConfig(distance_min=11),
                        lambda: VisibilityConfig(translation_mode="unknown")):
            with self.subTest(factory=factory), self.assertRaises(ValueError):
                factory()

    def test_adapter_logs_dual_modes_and_isolates_predictor_failure(self):
        root = Path(__file__).resolve().parents[2]
        diagnostics = VisibilityDiagnostics({}, root)
        self.assertIsNotNone(diagnostics.predictor)
        link = SimpleNamespace(
            shared_position=(0.0, 0.0, 0.0),
            odom=SimpleNamespace(v=(0.0, 0.0, 0.0), q=self.attitude, recv_time=10.0, attitude_recv_time=10.0),
            imu=SimpleNamespace(acc=(0.0, 0.0, 9.81), q=self.attitude, recv_time=10.0),
        )
        reference = SimpleNamespace(p=(0, 0, 0), v=(0, 0, 0), a=(0, 0, 0), yaw=0.0)
        output = SimpleNamespace(q=self.attitude, thrust=0.5)
        debug = SimpleNamespace(roll=0.0, pitch=0.0, tilt_saturated=False)
        arguments = dict(now=10.05, started=10.0, target_state=TargetState((6.0, 0.0, 0.0)),
                         target_message=SimpleNamespace(timestamp=10.0, sequence=1), link=link,
                 reference=reference, output=output, debug=debug, state_source="truth")
        row = diagnostics.sample(**arguments)
        self.assertTrue(row["valid"], row)
        self.assertEqual(len(row["attitude_predictions"]), 2)
        self.assertEqual(_flat_row(row)["constant_attitude_0p5_u"], 640.0)
        self.assertEqual(_flat_row(row)["distance_0p5_m"], 6.0)
        json.dumps(row, allow_nan=False)
        debug.tilt_saturated = True
        fallback = diagnostics.sample(**arguments)
        self.assertEqual(fallback["translation_mode_used"], "constant_velocity")
        self.assertEqual(fallback["acceleration_fallback_reason"], "tilt_saturated")
        arguments["now"] = 11.0
        self.assertEqual(diagnostics.sample(**arguments)["reason"], "stale_or_future_timestamp")
        arguments["now"] = 10.05
        with patch.object(diagnostics.predictor, "predict", side_effect=RuntimeError("injected")):
            self.assertIn("injected", diagnostics.sample(**arguments)["reason"])
        self.assertEqual(reference.a, (0, 0, 0))
        self.assertEqual(output.thrust, 0.5)

    def test_invalid_camera_configuration_is_reported_not_raised(self):
        diagnostics = VisibilityDiagnostics({"camera_config": "missing.yaml"}, Path(__file__).parent)
        self.assertIsNone(diagnostics.predictor)
        self.assertIn("initialization_error", diagnostics.metadata)

    def test_live_camera_info_snapshot_overrides_static_scene_only_in_diagnostics(self):
        from tracking.run_tracker import parse_args

        with patch("sys.argv", ["run_tracker", "--visibility-prediction"]):
            args = parse_args()
        snapshot = args.visibility_prediction_params["camera_intrinsics_override"]
        self.assertEqual(snapshot[0][0], 3054.16357421875)
        root = Path(__file__).resolve().parents[2]
        diagnostics = VisibilityDiagnostics(args.visibility_prediction_params, root)
        self.assertEqual(diagnostics.predictor.camera.intrinsics[0][0], snapshot[0][0])
        self.assertIn("CameraInfo", diagnostics.metadata["calibration_source"])
        self.assertAlmostEqual(diagnostics.predictor.predict(
            TargetState((6.0, 1.0, 0.0)), self.tracker, self.attitude).pixels_pred[0][0],
            640.0 - snapshot[0][0] / 5.7)
        invalid = VisibilityDiagnostics({"camera_intrinsics_override": [[0, 0, 0]]}, root)
        self.assertIsNone(invalid.predictor)

    def test_tracker_visibility_flags_preserve_band_and_control_prediction(self):
        from tracking.run_tracker import parse_args

        for flags, enabled in (([], True), (["--visibility-prediction"], True),
                               (["--no-visibility-prediction"], False)):
            with self.subTest(flags=flags), patch("sys.argv", ["run_tracker", *flags]):
                args = parse_args()
                self.assertEqual(args.visibility_prediction, enabled)
                self.assertEqual((args.follow_band_min, args.follow_band_max), (5.0, 10.0))
                self.assertEqual(args.prediction_horizon, 0.0)
                self.assertEqual(args.visibility_prediction_params["prediction_horizon_sec"], 0.8)

    def test_threshold_regions_camera_plane_and_calibration(self):
        for ratio, region in ((0.0, "safe"), (0.6, "margin"), (0.7, "margin"),
                              (0.8, "margin"), (0.9, "warning"), (1.0, "boundary"), (1.1, "outside")):
            with self.subTest(ratio=ratio):
                prediction = self.predict(TargetState((5.3, -5 * ratio, 0.0)))
                projection = prediction.attitude_predictions["constant_attitude"]
                self.assertEqual(projection.projection_status[0], region)
                self.assertEqual(prediction.predicted_fov_exit, ratio >= 1)
                self.assertEqual(projection.out_of_fov[0], ratio > 1)
        for position in ((0.3, 0.0, 0.0), (0.30000001, 0.0, 0.0)):
            prediction = self.predict(TargetState(position))
            self.assertIsNone(prediction.pixels_pred[0])
            json.dumps(asdict(prediction), allow_nan=False)
        with self.assertRaises(ValueError):
            replace(self.camera, width=0)
        with self.assertRaises(ValueError):
            replace(self.camera, intrinsics=((0, 0, 640), (0, 640, 360), (0, 0, 1)))

    def test_real_controller_outputs_identical_with_diagnostics_enabled_or_disabled(self):
        from px4ctrl.controller import DesiredState, LinearControl
        from px4ctrl.inputs import ImuData, OdomData
        from px4ctrl.params import load_params
        from tracking.guidance import ObserverState, PositionTrackerV0, band_offset

        root = Path(__file__).resolve().parents[2]
        sequences = []
        for enabled in (False, True):
            controller = LinearControl(load_params(root / "px4ctrl/config/sim.yaml"))
            diagnostics = VisibilityDiagnostics({}, root) if enabled else None
            sequence = []
            for index, distance in enumerate((4.0, 5.0, 6.0, 8.0, 10.0, 11.0)):
                now = 10.0 + index * 0.05
                target = TargetState((distance, 0.0, 2.0), (0.2, 0.0, 0.0), (0.1, 0.0, 0.0))
                tracker_position = (0.0, 0.0, 2.0)
                guidance = PositionTrackerV0(band_offset((-distance, 0.0, 0.0), 5.0, 10.0))
                reference = guidance.generate(target, ObserverState(tracker_position, tracker_position))
                if 5 <= distance <= 10:
                    self.assertEqual(reference.p[:2], tracker_position[:2])
                odom = OdomData(recv_time=now, attitude_recv_time=now, p=tracker_position)
                imu = ImuData(recv_time=now, acc=(0.0, 0.0, 9.81))
                output = controller.calculate_control(DesiredState(**asdict(reference)), odom, imu, now=now)
                original = (asdict(reference), asdict(output), asdict(controller.debug))
                if diagnostics is not None:
                    row = diagnostics.sample(
                        now=now, started=10.0, target_state=target,
                        target_message=SimpleNamespace(timestamp=now, sequence=index),
                        link=SimpleNamespace(shared_position=tracker_position, odom=odom, imu=imu),
                        reference=reference, output=output, debug=controller.debug, state_source="truth")
                    self.assertTrue(row["valid"], row)
                self.assertEqual(original, (asdict(reference), asdict(output), asdict(controller.debug)))
                sequence.append(original)
            sequences.append(sequence)
        self.assertEqual(sequences[0], sequences[1])

    def test_csv_and_png_export_with_invalid_samples(self):
        valid = asdict(self.predict(TargetState((6.0, 0.0, 0.0)), commanded_attitude=self.attitude))
        valid.update(t=0.0, monotonic_s=10.0)
        record = {"visibility_samples": [valid, {"t": 0.05, "monotonic_s": 10.05, "valid": False}],
                  "visibility_prediction": {"camera": asdict(self.camera)}}
        with tempfile.TemporaryDirectory() as directory:
            paths = export_visibility(record, Path(directory))
            self.assertEqual(len(paths), 2)
            with paths[0].open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["constant_attitude_0p5_u"], "640.0")
            self.assertEqual(rows[1]["constant_attitude_0p5_u"], "")
            self.assertEqual(paths[1].read_bytes()[:8], b"\x89PNG\r\n\x1a\n")


if __name__ == "__main__":
    unittest.main()