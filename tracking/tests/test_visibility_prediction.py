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

from tracking.guidance import DesiredState, TargetState
from tracking.shadow_observation import project_oracle_pixel, rotate_vector
from tracking.visibility_prediction import CameraGeometry, VisibilityConfig, VisibilityPredictor
from tracking.visibility_diagnostics import VisibilityDiagnostics, _flat_row, export_visibility, summarize_tracking


class PixelVelocityFollowerTest(unittest.TestCase):
    def setUp(self):
        from px4ctrl.params import load_params
        from tracking.pixel_velocity import PixelVelocityConfig, PixelVelocityFollower
        from tracking.tracker_control import TrackerLinearControl, TrackingLimits

        self.settings = PixelVelocityConfig(enabled=True)
        self.controller = TrackerLinearControl(replace(load_params(
            Path(__file__).resolve().parents[2] / "px4ctrl/config/sim.yaml"), max_angle=30.0),
            TrackingLimits(horizontal=4.0, jerk=12.0))
        self.controller.active = lambda: True
        self.controller._previous_time = 1.0
        self.link = SimpleNamespace(shared_position=(0.0, 0.0, 5.0), odom=SimpleNamespace(
            p=(0.0, 0.0, 5.0), v=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0)),
            imu=SimpleNamespace(w=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0), acc=(0.0, 0.0, 9.81)))
        self.predictor = VisibilityPredictor(CameraGeometry(
            ((921.0, 0.0, 640.0), (0.0, 921.0, 360.0), (0.0, 0.0, 1.0)),
            1280, 720, (0.3, 0.0, 0.0)), VisibilityConfig(pixel_occupancy_weight=0.0))
        self.follower = PixelVelocityFollower(self.settings, self.predictor)
        self.nominal = DesiredState(p=self.link.odom.p)

    def apply(self, target, reference=None):
        from tracking.shadow_observation import world_to_camera

        try:
            optical = world_to_camera(target.p, self.link.shared_position, self.link.odom.q,
                                      self.predictor.camera.offset_body)
        except ValueError:
            optical = np.array((0.0, 0.0, -1.0))
        matrix = self.predictor.camera.intrinsics
        pixel = [matrix[0][0] * optical[0] / optical[2] + matrix[0][2],
                 matrix[1][1] * optical[1] / optical[2] + matrix[1][2]] if optical[2] > 0 else [640, 360]
        return self.follower.apply(reference or self.nominal, target, self.link,
                                   self.controller, now=1.05,
                                   image_detection={"pixel": pixel, "image_stamp": 10.0})

    def test_safe_and_disabled_preserve_nominal_reference(self):
        from tracking.pixel_velocity import PixelVelocityFollower

        target = TargetState((7.0, 0.0, 5.0))
        reference, diagnostics = self.apply(target)
        self.assertIs(reference, self.nominal)
        self.assertFalse(diagnostics["pixel_velocity_active"])
        disabled = PixelVelocityFollower(replace(self.settings, enabled=False), self.predictor)
        reference, diagnostics = disabled.apply(self.nominal, target, self.link, self.controller, now=1.05)
        self.assertIs(reference, self.nominal)
        self.assertEqual(diagnostics["pixel_velocity_reason"], "disabled")

    def test_no_image_does_not_use_projection_as_feedback(self):
        result, diagnostics = self.follower.apply(self.nominal,
            TargetState((7.0, -4.0, 5.0), (0.0, -3.0, 0.0)), self.link, self.controller, now=1.05)
        self.assertIs(result, self.nominal)
        self.assertEqual(diagnostics["pixel_velocity_reason"], "no_fresh_image_detection")

    def test_image_subscriber_requires_fresh_real_detection_and_calibration(self):
        from tracking.image_pixel import ImagePixelSubscriber
        import socket

        subscriber = ImagePixelSubscriber(port=0, max_age=0.2)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            port = subscriber.socket.getsockname()[1]
            packet = {"pixel": [1050.0, 360.0], "received_at": 1.0, "image_stamp": 12.0,
                      "intrinsics": self.predictor.camera.intrinsics, "resolution": [1280, 720],
                      "source": "state_projection"}
            sender.sendto(json.dumps(packet).encode(), ("127.0.0.1", port))
            self.assertIsNone(subscriber.read(1.1))
            packet["source"] = "rgbd_detection"
            sender.sendto(json.dumps(packet).encode(), ("127.0.0.1", port))
            result = subscriber.read(1.1)
            self.assertEqual(result["pixel"], (1050.0, 360.0))
            self.assertIsNone(subscriber.read(1.21))
            packet["intrinsics"] = [[500.0, 0.0, 640.0], [0.0, 921.0, 360.0], [0.0, 0.0, 1.0]]
            reference, diagnostics = self.follower.apply(self.nominal, TargetState((7.0, 0.0, 5.0)),
                self.link, self.controller, now=1.05, image_detection=packet)
            self.assertIs(reference, self.nominal)
            self.assertEqual(diagnostics["pixel_velocity_reason"], "invalid_detection_or_camera_calibration")
        finally:
            sender.close()
            subscriber.close()

    def test_measured_image_pixel_changes_feedback_even_if_projection_is_safe(self):
        target = TargetState((7.0, 0.0, 5.0))
        reference, diagnostics = self.follower.apply(self.nominal, target, self.link,
            self.controller, now=1.05, image_detection={"pixel": [1050, 360], "image_stamp": 10.0})
        self.assertTrue(diagnostics["pixel_velocity_active"])
        self.assertEqual(diagnostics["pixel_current"][0], (1050 - 640) / 921)
        self.assertEqual(diagnostics["pixel_source"], "rgbd_detection")
        self.assertNotEqual(reference.v, self.nominal.v)

    def test_tuned_vertical_margin_corrects_low_pixel_before_old_threshold(self):
        from tracking.pixel_velocity import PixelVelocityConfig, PixelVelocityFollower
        import yaml

        config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                 "config/tracker.yaml").read_text())
        settings = PixelVelocityConfig(**dict(config["pixel_velocity_control"], enabled=True))
        follower = PixelVelocityFollower(settings, self.predictor)
        target = TargetState((7.0, 0.0, 5.0))
        detection = {"pixel": [640., 535.], "image_stamp": 10.0}
        old_reference, old_diagnostics = self.follower.apply(
            self.nominal, target, self.link, self.controller, now=1.05, image_detection=detection)
        reference, diagnostics = follower.apply(
            self.nominal, target, self.link, self.controller, now=1.05, image_detection=detection)
        self.assertIs(old_reference, self.nominal)
        self.assertFalse(old_diagnostics["pixel_risk_requested"])
        self.assertTrue(diagnostics["pixel_velocity_active"])
        self.assertLess(diagnostics["J_total"], diagnostics["J_nominal"])
        self.assertLess(diagnostics["pixel_pred"][1], diagnostics["pixel_pred_nominal"][1])
        self.assertLessEqual(np.linalg.norm(diagnostics["delta_v"]), settings.max_velocity_correction)
        self.assertEqual(reference.p, self.nominal.p)
        self.assertEqual(reference.a, self.nominal.a)

    def test_right_edge_motion_is_corrected_by_geometry(self):
        target = TargetState((7.0, -3.0, 5.0), (0.0, -3.0, 0.0))
        reference, diagnostics = self.apply(target)
        self.assertLess(reference.v[1], 0.0)
        self.assertTrue(diagnostics["pixel_velocity_active"])
        self.assertLess(diagnostics["J_total"], diagnostics["J_nominal"])
        self.assertLess(abs(diagnostics["pixel_pred"][0]), abs(diagnostics["pixel_pred_nominal"][0]))

    def test_right_edge_correction_rotates_with_observer_yaw(self):
        self.link.odom.q = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
        nominal = replace(self.nominal, yaw=math.pi / 2.0)
        result, diagnostics = self.apply(TargetState((3.0, 7.0, 5.0), (3.0, 0.0, 0.0)), nominal)
        self.assertGreater(result.v[0], 0.0)
        self.assertTrue(diagnostics["pixel_velocity_active"])

    def test_safe_now_but_predicted_exit_intervenes_early(self):
        target = TargetState((7.0, -2.0, 5.0), (0.0, -16.0, 0.0))
        reference, diagnostics = self.apply(target)
        self.assertTrue(diagnostics["pixel_current_inside_safe"])
        self.assertTrue(diagnostics["pixel_nominal_predicted_fov_violation"])
        self.assertTrue(diagnostics["pixel_velocity_active"])
        self.assertLess(reference.v[1], 0.0)

    def test_nose_up_differs_from_translation_only(self):
        reference = replace(self.nominal, v=(-1.0, 0.0, 0.0))
        _, diagnostics = self.apply(TargetState((7.0, 0.0, 5.0)), reference)
        self.assertGreater(diagnostics["pixel_pred_nominal"][1],
                           diagnostics["pixel_pred_translation_only"][1] + 0.02)
        self.assertIn("first_order", diagnostics["pixel_attitude_approximation"])

    def test_correction_diagnostics_distinguish_risk_and_clipping(self):
        from tracking.pixel_velocity import PixelVelocityFollower

        self.follower = PixelVelocityFollower(replace(self.settings,
            max_velocity_correction=0.001), self.predictor)
        target = TargetState((7.0, -3.0, 5.0), (0.0, -3.0, 0.0))
        reference, diagnostics = self.apply(target)
        self.assertTrue(diagnostics["pixel_correction_saturated"])
        self.assertGreater(np.linalg.norm(diagnostics["delta_v_raw"]), 0.001)
        self.assertAlmostEqual(np.linalg.norm(diagnostics["delta_v_bounded"]), 0.001)
        np.testing.assert_allclose(diagnostics["delta_v"],
            np.asarray(diagnostics["delta_v_bounded"]) * diagnostics["pixel_correction_step_fraction"])
        self.assertTrue(diagnostics["pixel_current_safe_violation_axes"][0])
        self.assertEqual(diagnostics["pixel_nominal_first_safe_exit_s"][0], 0.0)
        self.assertEqual(len(diagnostics["pixel_pred_nominal_path"]),
                         len(diagnostics["pixel_prediction_times_s"]))
        self.assertEqual(reference.p, self.nominal.p)
        self.assertEqual(reference.a, self.nominal.a)
        self.assertEqual(reference.yaw, self.nominal.yaw)
        json.dumps(diagnostics, allow_nan=False)

    def test_prediction_diagnostics_use_same_image_anchor(self):
        reference = replace(self.nominal, v=(-1.0, 0.0, 0.0))
        _, diagnostics = self.follower.apply(reference, TargetState((7.0, 0.0, 5.0)),
            self.link, self.controller, now=1.05, image_detection={
                "pixel": [640., 400.], "image_stamp": 10., "received_at": 1.0})
        self.assertAlmostEqual(diagnostics["image_arrival_age_s"], 0.05)
        self.assertAlmostEqual(diagnostics["pixel_current_model"][1], 0.0)
        self.assertAlmostEqual(diagnostics["pixel_current"][1], 40. / 921.)
        np.testing.assert_allclose(diagnostics["pixel_pred_attitude_delta_nominal"],
            np.asarray(diagnostics["pixel_pred_nominal"]) -
            np.asarray(diagnostics["pixel_pred_translation_only_anchored"]))
        json.dumps(diagnostics, allow_nan=False)

    def test_safe_distance_intent_is_not_taken_over(self):
        reference = replace(self.nominal, p=(1.0, 0.0, 5.0), v=(0.5, 0.0, 0.0))
        result, diagnostics = self.apply(TargetState((12.0, 0.0, 5.0)), reference)
        self.assertIs(result, reference)
        self.assertFalse(diagnostics["pixel_velocity_active"])

    def test_rotation_and_camera_lever_rate_match_finite_difference(self):
        from tracking.shadow_observation import world_to_camera

        target = TargetState((7.0, -1.0, 5.8), (1.0, -2.0, 0.3))
        tracker = TargetState(self.link.shared_position, (0.4, 0.2, -0.1))
        omega = np.array((0.3, -0.8, 0.4))
        result = self.predictor.image_kinematics(target, tracker, self.link.odom.q, tuple(omega))
        elapsed = 1e-6
        angle = np.linalg.norm(omega) * elapsed
        attitude = (*tuple(omega / np.linalg.norm(omega) * math.sin(angle / 2.0)), math.cos(angle / 2.0))
        optical = world_to_camera(tuple(np.asarray(target.p) + elapsed * np.asarray(target.v)),
            tuple(np.asarray(tracker.p) + elapsed * np.asarray(tracker.v)), attitude,
            self.predictor.camera.offset_body)
        numerical = (optical[:2] / optical[2] - np.asarray(result["pixel_current"])) / elapsed
        np.testing.assert_allclose(numerical, result["pixel_rate_total"], atol=2e-6)
        np.testing.assert_allclose(np.asarray(result["pixel_rate_translation"]) +
                                  result["pixel_rate_rotation"], result["pixel_rate_total"])

    def test_invalid_depth_and_attitude_fall_back_without_nan(self):
        for target, quaternion in ((TargetState((0.3, 0.0, 5.0)), (0.0, 0.0, 0.0, 1.0)),
                                   (TargetState((-7.0, 0.0, 5.0)), (0.0, 0.0, 0.0, 1.0)),
                                   (TargetState((7.0, 0.0, 5.0)), (0.0, 0.0, 0.0, 0.0)),
                                   (TargetState((7.0, math.nan, 5.0)), (0.0, 0.0, 0.0, 1.0))):
            self.link.odom.q = quaternion
            result, diagnostics = self.apply(target)
            self.assertIs(result, self.nominal)
            self.assertFalse(diagnostics["pixel_prediction_valid"])
            json.dumps(diagnostics, allow_nan=False)

    def test_comparison_metrics_use_actual_projection_and_mark_missing(self):
        from tracking.pixel_velocity import summarize_velocity_follower

        prediction = self.predictor.predict(TargetState((7.0, 0.0, 5.0)),
            TargetState(self.link.shared_position), self.link.odom.q)
        row = {"target_v": (1.0, 0.0, 0.0), "odom_v": (0.0, 0.0, 0.0), "t": 0.0,
               "nominal_reference_p": self.nominal.p, "odom_p": self.nominal.p,
               "horizontal_distance": 7.0, "limited_acceleration": (0.0, 0.0, 0.0)}
        summary = summarize_velocity_follower({"control_samples": [row], "visual_tracking_mode": "nominal",
            "visibility_samples": [asdict(prediction), {"valid": False}]}, self.settings, (5.0, 10.0))
        self.assertEqual(summary["target_velocity_tracking_rmse_m_s"], 1.0)
        self.assertEqual(summary["pixel_center_rmse_optical"], 0.0)
        self.assertEqual(summary["fov_loss_sample_count"], 0)
        self.assertEqual(summary["visibility_missing_sample_count"], 1)
        self.assertEqual(summary["velocity_intervention_sample_fraction"], 0.0)
        json.dumps(summary, allow_nan=False)

    def test_conflict_is_bounded_and_preserves_single_pd_and_limits(self):
        from px4ctrl.inputs import ImuData, OdomData

        history = (self.controller._previous_time, self.controller._previous_acceleration)
        nominal = replace(self.nominal, v=(0.0, 4.0, 0.0), a=(0.2, 0.0, 0.0))
        result, diagnostics = self.apply(TargetState((7.0, -3.0, 5.0), (0.0, -10.0, 0.0)), nominal)
        self.assertLessEqual(math.dist(result.v, nominal.v), self.settings.max_velocity_correction + 1e-10)
        self.assertEqual((result.p, result.a, result.yaw), (nominal.p, nominal.a, nominal.yaw))
        self.assertEqual(history, (self.controller._previous_time, self.controller._previous_acceleration))
        self.controller.calculate_control(result, OdomData(p=self.link.odom.p), ImuData(), now=1.05)
        raw = tuple(nominal.a[axis] + getattr(self.controller.params.gain, f"kv{axis}") * result.v[axis]
                    for axis in range(3))
        np.testing.assert_allclose(self.controller.tracking_debug["raw_acceleration"], raw)
        limited = self.controller.tracking_debug["limited_acceleration"]
        self.assertLessEqual(math.hypot(*limited[:2]), self.controller.limits.horizontal)
        self.assertLessEqual(math.hypot(*limited), self.controller.limits.jerk * 0.05 + 1e-10)
        json.dumps(diagnostics, allow_nan=False)


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
    def test_optimization_does_not_require_follow_band(self):
        from tracking.run_tracker import make_visibility_filter

        args = SimpleNamespace(visibility_optimization=True, follow_band_min=None, follow_band_max=None)
        diagnostics = SimpleNamespace(predictor=VisibilityPredictor(
            CameraGeometry(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0)),
                           1280, 720, (0.3, 0.0, 0.0))))
        self.assertIsNotNone(make_visibility_filter(args, diagnostics))
        with self.assertRaises(ValueError):
            make_visibility_filter(args, None)

    def test_fov_priority_and_recovery_with_acceleration_input(self):
        from tracking.run_tracker import visibility_optimized_reference

        link = SimpleNamespace(shared_position=(0.0, 0.0, 0.0),
                               odom=SimpleNamespace(p=(0.0, 0.0, 0.0), v=(0.0, 0.0, 0.0), q=self.attitude),
                               imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))
        controller = SimpleNamespace(params=SimpleNamespace(gra=9.81,
            gain=SimpleNamespace(kp0=2.0, kp1=2.0, kp2=2.0, kv0=1.0, kv1=1.0, kv2=1.0)))
        target = TargetState((7.0, 0.0, 0.0), a=(1.0, 0.0, 0.0))
        base = VisibilityPredictor(self.camera, VisibilityConfig(pixel_occupancy_weight=0.0))
        for recovery in (False, True):
            diagnostics = {}

            def predict(target_state, tracker_state, *args, **kwargs):
                self.assertEqual(target_state.a, target.a)
                prediction = base.predict(target_state, tracker_state, *args, **kwargs)
                pixel = (0.7 if recovery else 0.0, 0.0) if tracker_state.a[0] > 0 else (1.5, 0.0)
                projection = replace(prediction.attitude_predictions["command_based_attitude"],
                                     normalized_pixels_pred=[(0.0, 0.0)] + [pixel] * (len(base.times) - 1))
                return replace(prediction, distances_pred=[1e6 if tracker_state.a[0] > 0 else 7.0] * len(base.times),
                               attitude_predictions={"command_based_attitude": projection})

            predictor = SimpleNamespace(config=base.config, predict=predict)
            reference = visibility_optimized_reference(DesiredState(), target, link, controller, predictor,
                vertical_step=0.0, los_step=0.3, smoothness_weight=0.05,
                diagnostics=diagnostics, fov_weight=0.0)
            self.assertGreaterEqual(reference.a[0], 0.3)
            self.assertEqual(diagnostics["fov_candidate_feasible"], not recovery)
            self.assertFalse(diagnostics["fov_guarantee"])

    def test_all_invalid_predictions_fail_closed(self):
        from tracking.run_tracker import visibility_optimized_reference
        from tracking.visibility_prediction import VisibilityPrediction

        link = SimpleNamespace(shared_position=self.tracker.p, odom=SimpleNamespace(
            p=self.tracker.p, v=self.tracker.v, q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))
        controller = SimpleNamespace(params=SimpleNamespace(gra=9.81,
            gain=SimpleNamespace(kp0=2.0, kp1=2.0, kp2=2.0, kv0=1.0, kv1=1.0, kv2=1.0)))
        predictor = SimpleNamespace(predict=lambda *args, **kwargs: VisibilityPrediction())
        with self.assertRaises(RuntimeError):
            visibility_optimized_reference(DesiredState(), TargetState((7.0, 0.0, 0.0)),
                link, controller, predictor, vertical_step=0.3, los_step=0.3, smoothness_weight=0.05)

    def test_current_ideal_pixel_adds_vertical_feedback_candidate(self):
        from tracking.run_tracker import visibility_optimized_reference

        link = SimpleNamespace(shared_position=self.tracker.p, odom=SimpleNamespace(
            p=self.tracker.p, v=self.tracker.v, q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))
        controller = SimpleNamespace(params=SimpleNamespace(gra=9.81,
            gain=SimpleNamespace(kp0=2.0, kp1=2.0, kp2=2.0, kv0=1.0, kv1=1.0, kv2=1.0)))
        predictor = VisibilityPredictor(self.camera, VisibilityConfig(pixel_occupancy_weight=0.0))
        target = TargetState((7.0, 0.0, -0.5))
        evaluated = []
        original = predictor.predict

        def predict(target_state, tracker_state, *args, **kwargs):
            evaluated.append(tracker_state.a)
            return original(target_state, tracker_state, *args, **kwargs)

        diagnostics = {}
        with patch.object(predictor, "predict", side_effect=predict):
            visibility_optimized_reference(DesiredState(), target, link, controller, predictor,
                vertical_step=0.3, los_step=0.0, smoothness_weight=0.05, diagnostics=diagnostics)
        pixel_vertical = diagnostics["current_normalized_pixel"][1]
        self.assertGreater(pixel_vertical, 0.0)
        self.assertLess(pixel_vertical, 0.3)
        self.assertIn((0.0, 0.0, -pixel_vertical), evaluated)
        self.assertEqual(diagnostics["pixel_feedback_source"], "ideal_state_projection_not_image_detection")

    def test_safe_lower_image_target_selects_descent_not_only_a_candidate(self):
        from tracking.run_tracker import visibility_optimized_reference

        link = SimpleNamespace(shared_position=self.tracker.p, odom=SimpleNamespace(
            p=self.tracker.p, v=self.tracker.v, q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))
        controller = SimpleNamespace(params=SimpleNamespace(gra=9.81,
            gain=SimpleNamespace(kp0=2.0, kp1=2.0, kp2=2.0, kv0=1.0, kv1=1.0, kv2=1.0)))
        diagnostics = {}
        reference = visibility_optimized_reference(DesiredState(), TargetState((7.0, 0.0, -1.0)),
            link, controller, VisibilityPredictor(self.camera), vertical_step=0.3, los_step=0.0,
            smoothness_weight=0.05, diagnostics=diagnostics)
        self.assertTrue(diagnostics["current_fov_safe"])
        self.assertGreater(diagnostics["current_normalized_pixel"][1], 0.0)
        self.assertLess(reference.a[2], 0.0)

    def test_primary_only_projection_preserves_selected_attitude_prediction(self):
        target = TargetState((7.0, 0.0, 0.0))
        commanded_attitude = (0.0, math.sin(0.1), 0.0, math.cos(0.1))
        for mode in ("constant_attitude", "command_based_attitude"):
            with self.subTest(mode=mode):
                predictor = VisibilityPredictor(self.camera, VisibilityConfig(attitude_mode=mode))
                compared = predictor.predict(target, self.tracker, self.attitude, commanded_attitude)
                primary = predictor.predict(target, self.tracker, self.attitude, commanded_attitude,
                                            include_attitude_comparison=False)
                self.assertTrue(primary.valid)
                self.assertEqual(set(primary.attitude_predictions), {mode})
                self.assertEqual(primary.normalized_pixels_pred, compared.normalized_pixels_pred)
                self.assertEqual(primary.relative_positions_camera, compared.relative_positions_camera)
                self.assertEqual(primary.fov_cost, compared.fov_cost)
        fallback = predictor.predict(target, self.tracker, self.attitude,
                                     include_attitude_comparison=False)
        self.assertEqual(fallback.attitude_mode_used, "constant_attitude")
        self.assertEqual(set(fallback.attitude_predictions), {"constant_attitude"})

    def test_nose_up_braking_selects_joint_descent_and_respects_floor(self):
        from px4ctrl.params import load_params
        from tracking.run_tracker import visibility_optimized_reference
        from tracking.tracker_control import TrackerLinearControl, TrackingLimits

        params = replace(load_params(Path(__file__).resolve().parents[2] / "px4ctrl/config/sim.yaml"),
                         max_angle=30.0)
        controller = TrackerLinearControl(params, TrackingLimits(horizontal=4.0, jerk=12.0))
        controller.active = lambda: True
        controller._previous_acceleration = (-3.0, 0.0, 0.0)
        controller._previous_time = 1.0
        attitude = (0.0, math.sin(-0.1), 0.0, math.cos(-0.1))
        camera = replace(self.camera, intrinsics=((921.0, 0.0, 640.0), (0.0, 921.0, 360.0), (0.0, 0.0, 1.0)))
        predictor = VisibilityPredictor(camera, VisibilityConfig(pixel_occupancy_weight=0.0))
        target = TargetState((7.0, 0.0, 5.0), (6.0, 0.0, 0.0), (-3.0, 0.0, 0.0))
        for local_height in (5.0, 1.5):
            with self.subTest(local_height=local_height):
                link = SimpleNamespace(shared_position=(0.0, 0.0, 5.0), odom=SimpleNamespace(
                    p=(0.0, 0.0, local_height), v=(6.0, 0.0, 0.0), q=attitude),
                    imu=SimpleNamespace(q=attitude, acc=(0.0, 0.0, 9.81)))
                nominal = DesiredState(p=link.odom.p, v=link.odom.v, a=(-3.0, 0.0, 0.0))
                diagnostics = {}
                reference = visibility_optimized_reference(nominal, target, link, controller, predictor,
                    vertical_step=0.3, los_step=0.3, smoothness_weight=0.05,
                    min_height=1.5, now=1.05, diagnostics=diagnostics)
                if local_height > 1.5:
                    self.assertLess(reference.a[2], 0.0)
                    self.assertLess(diagnostics["predicted_effective_acceleration"][2], 0.0)
                else:
                    self.assertGreaterEqual(diagnostics["predicted_effective_acceleration"][2], 0.0)
                self.assertGreater(reference.a[0], nominal.a[0])
                self.assertLess(diagnostics["predicted_terminal_pixel"][1],
                                diagnostics["current_normalized_pixel"][1])
                self.assertTrue(diagnostics["motion_constraints_feasible"])
                self.assertLessEqual(diagnostics["optimizer_candidate_count"], 38)
                json.dumps(diagnostics, allow_nan=False)
                self.assertEqual(controller._previous_acceleration, (-3.0, 0.0, 0.0))
                self.assertEqual(controller._previous_time, 1.0)

    def test_descent_speed_limit_overrides_lower_image_feedback(self):
        from tracking.run_tracker import visibility_optimized_reference

        link = SimpleNamespace(shared_position=(0.0, 0.0, 5.0), odom=SimpleNamespace(
            p=(0.0, 0.0, 5.0), v=(0.0, 0.0, -1.5), q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))
        controller = SimpleNamespace(params=SimpleNamespace(gra=9.81,
            gain=SimpleNamespace(kp0=2.0, kp1=2.0, kp2=5.0, kv0=1.0, kv1=1.0, kv2=5.0)))
        diagnostics = {}
        reference = visibility_optimized_reference(DesiredState(p=link.odom.p, v=link.odom.v),
            TargetState((7.0, 0.0, 3.0), (0.0, 0.0, -1.5)), link, controller,
            VisibilityPredictor(self.camera), vertical_step=0.3, los_step=0.3,
            smoothness_weight=0.05, max_vertical_speed=1.5, diagnostics=diagnostics)
        self.assertGreater(diagnostics["current_normalized_pixel"][1], 0.0)
        self.assertGreaterEqual(reference.a[2], 0.0)
        self.assertTrue(diagnostics["motion_constraints_feasible"])

    def test_motion_infeasible_recovery_does_not_sacrifice_floor_for_pixels(self):
        from tracking.run_tracker import visibility_optimized_reference

        link = SimpleNamespace(shared_position=(0.0, 0.0, 5.0), odom=SimpleNamespace(
            p=(0.0, 0.0, 1.0), v=(0.0, 0.0, 0.0), q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))
        controller = SimpleNamespace(params=SimpleNamespace(gra=9.81,
            gain=SimpleNamespace(kp0=2.0, kp1=2.0, kp2=5.0, kv0=1.0, kv1=1.0, kv2=5.0)))
        arguments = dict(vertical_step=0.3, los_step=0.3, smoothness_weight=0.05, min_height=1.5)
        diagnostics = {}
        reference = visibility_optimized_reference(DesiredState(p=link.odom.p),
            TargetState((7.0, 0.0, 4.0)), link, controller, VisibilityPredictor(self.camera),
            diagnostics=diagnostics, **arguments)
        self.assertGreater(reference.a[2], 0.0)
        self.assertFalse(diagnostics["motion_constraints_feasible"])
        self.assertEqual(diagnostics["fov_selection_mode"], "recovery_motion_constraints")
        for key, value in (("pixel_center_weight", -1.0), ("attitude_response_time", math.nan),
                   ("max_vertical_speed", 0.0), ("motion_lead_time", -0.1),
                           ("catchup_speed_limit", 0.0), ("separation_min", 0.0)):
            with self.assertRaises(ValueError):
                visibility_optimized_reference(DesiredState(p=link.odom.p), TargetState((7.0, 0.0, 5.0)),
                    link, controller, VisibilityPredictor(self.camera), **arguments, **{key: value})

    def setUp(self):
        self.camera = CameraGeometry(((640.0, 0.0, 640.0), (0.0, 640.0, 360.0), (0.0, 0.0, 1.0)),
                                     1280, 720, (0.3, 0.0, 0.0))
        self.predictor = VisibilityPredictor(self.camera)
        self.tracker = TargetState((0.0, 0.0, 0.0))
        self.attitude = (0.0, 0.0, 0.0, 1.0)

    def predict(self, target, **kwargs):
        return self.predictor.predict(target, self.tracker, self.attitude, **kwargs)

    def test_optimizer_actions_change_future_distance_and_velocity(self):
        from tracking.run_tracker import visibility_optimized_reference

        predictor = VisibilityPredictor(self.camera, VisibilityConfig(pixel_occupancy_weight=0.0))
        link = SimpleNamespace(
            shared_position=self.tracker.p,
            odom=SimpleNamespace(p=self.tracker.p, v=self.tracker.v, q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)),
        )
        controller = SimpleNamespace(params=SimpleNamespace(
            gra=9.81,
            gain=SimpleNamespace(kp0=2.0, kp1=2.0, kp2=2.0, kv0=1.0, kv1=1.0, kv2=1.0),
        ))
        for distance, velocity, expected in ((12.0, 0.0, 0.3), (3.0, 0.0, -0.3),
                                              (7.0, 1.0, 0.3)):
            with self.subTest(distance=distance, velocity=velocity):
                target = TargetState((distance, 0.0, 0.0), (velocity, 0.0, 0.0))
                reference = visibility_optimized_reference(
                    DesiredState(), target, link, controller, predictor,
                    vertical_step=0.0, los_step=0.3, smoothness_weight=0.05,
                    fov_weight=0.0, distance_weight=1.0, tracking_weight=0.25,
                )
                self.assertGreaterEqual(math.copysign(1.0, expected) * reference.a[0], abs(expected))
                self.assertEqual(reference.a[1:], (0.0, 0.0))

    def test_target_trend_and_large_distance_generate_strong_actions(self):
        from px4ctrl.params import load_params
        from tracking.run_tracker import visibility_optimized_reference
        from tracking.tracker_control import TrackerLinearControl, TrackingLimits

        params = replace(load_params(Path(__file__).resolve().parents[2] / "px4ctrl/config/sim.yaml"),
                         max_angle=30.0)
        controller = TrackerLinearControl(params, TrackingLimits(horizontal=4.0, jerk=12.0))
        controller.active = lambda: True
        controller._previous_time = 1.0
        link = SimpleNamespace(shared_position=(0.0, 0.0, 5.0), odom=SimpleNamespace(
            p=(0.0, 0.0, 5.0), v=(6.0, 0.0, 0.0), q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))
        for focal_length, distance, target_acceleration, nominal_acceleration, direction in (
            (640.0, 7.0, 3.0, 1.5, 1.0), (640.0, 7.0, -3.0, -1.5, -1.0), (640.0, 20.0, 0.0, 0.0, 1.0),
            (921.0, 7.0, 3.0, 1.5, 1.0), (921.0, 7.0, -3.0, -1.5, -1.0), (921.0, 20.0, 0.0, 0.0, 1.0),
        ):
            with self.subTest(focal_length=focal_length, distance=distance, target_acceleration=target_acceleration):
                camera = replace(self.camera, intrinsics=((focal_length, 0.0, 640.0),
                    (0.0, focal_length, 360.0), (0.0, 0.0, 1.0)))
                predictor = VisibilityPredictor(camera, VisibilityConfig(pixel_occupancy_weight=0.0))
                controller._previous_acceleration = (nominal_acceleration, 0.0, 0.0)
                target = TargetState((distance, 0.0, 5.0), link.odom.v, (target_acceleration, 0.0, 0.0))
                nominal = DesiredState(p=link.odom.p, v=link.odom.v, a=(nominal_acceleration, 0.0, 0.375))
                diagnostics = {}
                reference = visibility_optimized_reference(nominal, target, link, controller, predictor,
                    vertical_step=0.3, los_step=0.3, smoothness_weight=0.02,
                    pixel_center_weight=2.0, distance_weight=4.0, tracking_weight=1.0,
                    min_height=1.5, diagnostics=diagnostics, now=1.05)
                self.assertGreater(direction * reference.a[0], 2.0)
                self.assertGreater(direction * diagnostics["predicted_effective_acceleration"][0],
                                   direction * nominal_acceleration)
                self.assertTrue(diagnostics["fov_candidate_feasible"])
                self.assertLessEqual(diagnostics["optimizer_candidate_count"], 38)
                self.assertAlmostEqual(diagnostics["target_trend_velocity"][0], 6.0 + target_acceleration * 0.2)
                self.assertEqual(diagnostics["distance_correction_speed"] > 0.0, distance > 10.0)
                self.assertLessEqual(diagnostics["distance_correction_speed"], 2.0)

    def test_acceleration_onset_does_not_wait_for_velocity_error(self):
        import yaml

        from px4ctrl.params import load_params
        from tracking.run_tracker import visibility_optimized_reference
        from tracking.tracker_control import TrackerLinearControl, TrackingLimits

        settings = yaml.safe_load((Path(__file__).resolve().parents[1]
                                   / "config/experiment.yaml").read_text())["tracker"]
        weights = {name + "_weight": settings["visibility_" + name + "_weight"]
                   for name in ("pixel_center", "distance", "tracking", "effort", "smoothness")}
        params = replace(load_params(Path(__file__).resolve().parents[2] / "px4ctrl/config/sim.yaml"),
                         max_angle=30.0)
        camera = replace(self.camera, intrinsics=((921.0, 0.0, 640.0),
            (0.0, 921.0, 360.0), (0.0, 0.0, 1.0)))
        predictor = VisibilityPredictor(camera, VisibilityConfig(pixel_occupancy_weight=0.0))
        link = SimpleNamespace(shared_position=(0.0, 0.0, 5.0), odom=SimpleNamespace(
            p=(0.0, 0.0, 5.0), v=(0.0, 0.0, 0.0), q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))
        for acceleration in (0.3, 1.0, -0.3, -1.0):
            with self.subTest(acceleration=acceleration):
                controller = TrackerLinearControl(params, TrackingLimits(horizontal=4.0, jerk=12.0))
                controller.active = lambda: True
                controller._previous_time = 1.0
                target = TargetState((7.0, 0.0, 5.0), link.odom.v, (acceleration, 0.0, 0.0))
                nominal = DesiredState(p=link.odom.p, v=link.odom.v, a=(0.5 * acceleration, 0.0, 0.375))
                diagnostics = {}
                without_lead = visibility_optimized_reference(nominal, target, link, controller, predictor,
                    vertical_step=0.3, los_step=0.3, motion_lead_time=0.0,
                    min_height=1.5, now=1.05, **weights)
                reference = visibility_optimized_reference(nominal, target, link, controller, predictor,
                    vertical_step=0.3, los_step=0.3, motion_lead_time=settings["visibility_motion_lead_time"],
                    min_height=1.5, diagnostics=diagnostics, now=1.05, **weights)
                self.assertGreaterEqual(math.copysign(1.0, acceleration) * reference.a[0], abs(acceleration))
                self.assertGreater(math.copysign(1.0, acceleration) * reference.a[0],
                                   math.copysign(1.0, acceleration) * without_lead.a[0])
                self.assertGreater(math.copysign(1.0, acceleration)
                                   * diagnostics["predicted_effective_acceleration"][0], 0.0)
                self.assertTrue(diagnostics["fov_candidate_feasible"])
                self.assertAlmostEqual(diagnostics["tracking_velocity_lead"][0],
                                       acceleration * settings["visibility_motion_lead_time"])

    def test_large_velocity_error_keeps_moderate_acceleration_candidates(self):
        from px4ctrl.params import load_params
        from tracking.run_tracker import visibility_optimized_reference
        from tracking.tracker_control import TrackerLinearControl, TrackingLimits

        params = replace(load_params(Path(__file__).resolve().parents[2] / "px4ctrl/config/sim.yaml"),
                         max_angle=30.0)
        controller = TrackerLinearControl(params, TrackingLimits(horizontal=4.0, jerk=12.0))
        controller.active = lambda: True
        controller._previous_time = 1.0
        attitude = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
        link = SimpleNamespace(shared_position=(0.0, 0.0, 5.0), odom=SimpleNamespace(
            p=(0.0, 0.0, 5.0), v=(0.0, 0.0, 0.0), q=attitude),
            imu=SimpleNamespace(q=attitude, acc=(0.0, 0.0, 9.81)))
        camera = replace(self.camera, intrinsics=((921.0, 0.0, 640.0),
            (0.0, 921.0, 360.0), (0.0, 0.0, 1.0)))
        predictor = VisibilityPredictor(camera, VisibilityConfig(pixel_occupancy_weight=0.0))
        target = TargetState((0.0, 7.0, 5.0), (0.0, 4.7, 0.0), (0.0, 1.0, 0.0))
        nominal = DesiredState(p=link.odom.p, v=target.v, a=(0.0, 0.5, 0.375), yaw=math.pi / 2)
        diagnostics = {}
        reference = visibility_optimized_reference(nominal, target, link, controller, predictor,
            vertical_step=0.3, los_step=0.3, smoothness_weight=0.02,
            pixel_center_weight=2.0, distance_weight=4.0, tracking_weight=2.0,
            effort_weight=0.02, min_height=1.5, diagnostics=diagnostics, now=1.05)
        self.assertGreater(reference.a[1], 1.0)
        self.assertLessEqual(math.hypot(*reference.a[:2]), 4.0)
        self.assertGreater(diagnostics["predicted_effective_acceleration"][1], 0.0)
        self.assertTrue(diagnostics["fov_candidate_feasible"])

    def test_separation_guard_brakes_before_distance_boundary_and_overrides_fov(self):
        from px4ctrl.params import load_params
        from tracking.run_tracker import visibility_optimized_reference
        from tracking.tracker_control import TrackerLinearControl, TrackingLimits

        params = replace(load_params(Path(__file__).resolve().parents[2] / "px4ctrl/config/sim.yaml"),
                         max_angle=30.0)
        camera = replace(self.camera, intrinsics=((921.0, 0.0, 640.0),
            (0.0, 921.0, 360.0), (0.0, 0.0, 1.0)))
        base = VisibilityPredictor(camera, VisibilityConfig(pixel_occupancy_weight=0.0))
        link = SimpleNamespace(shared_position=(0.0, 0.0, 5.0), odom=SimpleNamespace(
            p=(0.0, 0.0, 5.0), v=(4.0, 0.0, 0.0), q=self.attitude),
            imu=SimpleNamespace(q=self.attitude, acc=(0.0, 0.0, 9.81)))

        def predict(target, tracker, *args, **kwargs):
            prediction = base.predict(target, tracker, *args, **kwargs)
            if kwargs.get("tracker_acceleration_path") is not None:
                projection = prediction.attitude_predictions["command_based_attitude"]
                pixel = (0.9, 0.0) if tracker.a[0] < 0.0 else (0.0, 0.0)
                projection = replace(projection, normalized_pixels_pred=[(0.0, 0.0)]
                                     + [pixel] * (len(base.times) - 1))
                return replace(prediction, attitude_predictions={"command_based_attitude": projection})
            return prediction

        predictor = SimpleNamespace(config=base.config, predict=predict)
        for distance in (10.0, 7.0, 3.5):
            with self.subTest(distance=distance):
                controller = TrackerLinearControl(params, TrackingLimits(horizontal=4.0, jerk=12.0))
                controller.active = lambda: True
                controller._previous_time = 1.0
                diagnostics = {}
                reference = visibility_optimized_reference(DesiredState(p=link.odom.p, v=link.odom.v),
                    TargetState((distance, 0.0, 5.0)), link, controller, predictor,
                    vertical_step=0.3, los_step=0.3, smoothness_weight=0.02,
                    separation_min=3.0, min_height=1.5, now=1.05, diagnostics=diagnostics)
                self.assertLess(reference.a[0], -1.0)
                self.assertLess(diagnostics["predicted_effective_acceleration"][0], 0.0)
                self.assertEqual(diagnostics["fov_selection_mode"],
                                 "recovery_separation_fov" if distance == 10.0 else "recovery_separation_constraints")
                self.assertEqual(diagnostics["separation_constraints_feasible"], distance == 10.0)
                self.assertGreater(diagnostics["separation_braking_reserve_m"], 0.0)
                self.assertEqual(controller._previous_acceleration, (0.0, 0.0, 0.0))
                json.dumps(diagnostics, allow_nan=False)

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

    def test_pixel_occupancy_cost_prefers_usable_apparent_size(self):
        nominal = self.predict(TargetState((6.0, 0.0, 0.0)))
        far = self.predict(TargetState((20.0, 0.0, 0.0)))
        near = self.predict(TargetState((3.5, 0.0, 0.0)))
        self.assertAlmostEqual(nominal.pixel_occupancy_pred[0], 0.00388, places=4)
        self.assertEqual(nominal.pixel_occupancy_cost, 0.0)
        self.assertGreater(far.pixel_occupancy_cost, nominal.pixel_occupancy_cost)
        self.assertGreater(near.pixel_occupancy_cost, nominal.pixel_occupancy_cost)

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

    def test_batch_camera_path_matches_single_frame_geometry(self):
        from tracking.shadow_observation import world_to_camera, world_to_camera_path

        targets = [(6.0, 2.0, 1.0), (4.0, -1.0, 3.0), (7.0, 0.0, -2.0)]
        trackers = [(1.0, 0.0, 0.0), (0.0, 2.0, 1.0), (-1.0, 0.0, 1.0)]
        attitudes = [(0.1, 0.2, -0.3, 0.9), (0.4, -0.1, 0.2, 1.1), self.attitude]
        for sign in (1.0, -1.0):
            expected = [world_to_camera(target, tracker, attitude, self.camera.offset_body, sign)
                        for target, tracker, attitude in zip(targets, trackers, attitudes)]
            np.testing.assert_allclose(world_to_camera_path(targets, trackers, attitudes,
                self.camera.offset_body, sign), expected, atol=1e-12)

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

    def test_control_path_integrates_velocity_and_preserves_attitude_lag(self):
        commanded = (0.0, math.sin(-0.2), 0.0, math.cos(-0.2))
        accelerations = [(0.0, 0.0, -elapsed) for elapsed in self.predictor.times]
        target = TargetState((7.0, 0.0, 0.0))
        prediction = self.predict(target, commanded_attitude=commanded,
            tracker_acceleration_path=accelerations, attitude_response_time=0.2)
        instant = self.predict(target, commanded_attitude=commanded)
        self.assertEqual(prediction.tracker_positions_pred[1], self.tracker.p)
        self.assertLess(prediction.tracker_velocities_pred[-1][2], 0.0)
        self.assertGreater(prediction.tracker_positions_pred[-1][2], -0.5 * 0.8**3)
        self.assertLess(abs(prediction.normalized_pixels_pred[1][1]),
                        abs(instant.normalized_pixels_pred[1][1]))
        self.assertEqual(prediction.normalized_pixels_pred[0], instant.normalized_pixels_pred[0])
        for options in ({"tracker_acceleration_path": []}, {"attitude_response_time": -0.1},
                        {"commanded_attitude_path": [(0.0, 0.0, 0.0, 0.0)] * len(self.predictor.times)}):
            self.assertFalse(self.predict(target, **options).valid)

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
                        lambda: VisibilityConfig(pixel_occupancy_min_ratio=0.01,
                                                 pixel_occupancy_max_ratio=0.005),
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
        self.assertGreater(_flat_row(row)["pixel_occupancy_now_ratio"], 0.0)
        self.assertGreater(_flat_row(row)["pixel_occupancy_0p5_ratio"], 0.0)
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
        # 场景镜头已按 D435 等效 fx=921 配置，诊断必须采用场景内参。
        self.assertIsNone(args.visibility_prediction_params.get("camera_intrinsics_override"))
        root = Path(__file__).resolve().parents[2]
        diagnostics = VisibilityDiagnostics(args.visibility_prediction_params, root)
        self.assertEqual(diagnostics.predictor.camera.intrinsics[0][0], 921.0)
        self.assertIn("calibration_source", diagnostics.metadata)
        self.assertAlmostEqual(diagnostics.predictor.predict(
            TargetState((6.0, 1.0, 0.0)), self.tracker, self.attitude).pixels_pred[0][0],
            640.0 - 921.0 / 5.7)
        invalid = VisibilityDiagnostics({"camera_intrinsics_override": [[0, 0, 0]]}, root)
        self.assertIsNone(invalid.predictor)

    def test_tracker_visibility_flags_disable_fixed_band_by_default_and_keep_control_prediction(self):
        from tracking.run_tracker import parse_args

        for flags, enabled in (([], True), (["--visibility-prediction"], True),
                               (["--no-visibility-prediction"], False)):
            with self.subTest(flags=flags), patch("sys.argv", ["run_tracker", *flags]):
                args = parse_args()
                self.assertEqual(args.visibility_prediction, enabled)
                self.assertEqual((args.follow_band_min, args.follow_band_max), (None, None))
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
            self.assertEqual(len(paths), 3)
            with paths[0].open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["constant_attitude_0p5_u"], "640.0")
            self.assertEqual(rows[1]["constant_attitude_0p5_u"], "")
            self.assertEqual(paths[1].read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            self.assertEqual(paths[2].name, "pixel_occupancy.png")
            self.assertEqual(paths[2].read_bytes()[:8], b"\x89PNG\r\n\x1a\n")


if __name__ == "__main__":
    unittest.main()