"""Single short-horizon velocity correction; no solver dependency or controller mutation."""

from dataclasses import dataclass, replace
import math
import time

import numpy as np

from px4ctrl.controller import _quat_from_zyx
from tracking.guidance import TargetState, predict_commanded_tilt
from tracking.visibility_prediction import VisibilityPredictor


@dataclass(frozen=True)
class PixelVelocityConfig:
    enabled: bool = False
    prediction_horizon: float = 0.25
    safe_x: float = 0.417
    safe_y: float = 0.235
    weight_velocity: float = 1.0
    weight_pixel: float = 100.0
    weight_distance: float = 1.0
    weight_smooth: float = 0.0
    max_velocity_correction: float = 1.0

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError("pixel_velocity_control.enabled must be boolean")
        values = [getattr(self, name) for name in self.__dataclass_fields__ if name != "enabled"]
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in values):
            raise ValueError("pixel velocity settings must be finite numbers")
        if (min(self.prediction_horizon, self.safe_x, self.safe_y, self.weight_velocity,
                self.max_velocity_correction) <= 0.0 or not 0.2 <= self.prediction_horizon <= 0.4
                or min(self.weight_pixel, self.weight_distance, self.weight_smooth) < 0.0):
            raise ValueError("invalid pixel velocity horizon, safe region, weights or bound")


class PixelVelocityFollower:
    def __init__(self, settings, predictor, *, attitude_response_time=0.15, distance_band=None):
        self.settings = settings
        if not math.isfinite(attitude_response_time) or attitude_response_time < 0.0:
            raise ValueError("invalid attitude response time")
        self.response_time = attitude_response_time
        config = replace(predictor.config, prediction_horizon_sec=settings.prediction_horizon,
                         prediction_dt=min(0.05, settings.prediction_horizon),
                         translation_mode="constant_acceleration", attitude_mode="command_based_attitude")
        self.predictor = VisibilityPredictor(predictor.camera, config)
        self.distance_band = distance_band or (config.distance_min, config.distance_max)
        if not 0.0 < self.distance_band[0] <= self.distance_band[1]:
            raise ValueError("invalid nominal distance band")
        camera = predictor.camera
        matrix = np.asarray(camera.intrinsics)
        self.image_bounds = np.array(((-matrix[0, 2] / matrix[0, 0],
                                       (camera.width - matrix[0, 2]) / matrix[0, 0]),
                                      (-matrix[1, 2] / matrix[1, 1],
                                       (camera.height - matrix[1, 2]) / matrix[1, 1])))
        if np.any(np.array((settings.safe_x, settings.safe_y)) >=
                  np.minimum(-self.image_bounds[:, 0], self.image_bounds[:, 1])):
            raise ValueError("pixel safe region must be inside calibrated image bounds")
        self.previous_velocity = None

    def apply(self, reference, target, link, controller, *, now, image_detection=None):
        settings = self.settings
        started = time.perf_counter()
        nominal = np.asarray(reference.v, dtype=float)
        diagnostics = {"v_nom": nominal.tolist(), "v_cmd": nominal.tolist(), "delta_v": [0.0] * 3,
                       "pixel_velocity_active": False, "pixel_velocity_reason": "disabled",
                       "pixel_coordinate_convention": "optical_X_over_Z_Y_over_Z",
                       "pixel_attitude_approximation": "PD_jerk_preview_first_order_command_attitude",
                       "pixel_attitude_response_time_s": self.response_time,
                       "pixel_velocity_horizon_s": settings.prediction_horizon}
        if not settings.enabled:
            return reference, diagnostics
        if image_detection is None:
            self.previous_velocity = None
            diagnostics.update(pixel_velocity_reason="no_fresh_image_detection", pixel_prediction_valid=False)
            return reference, diagnostics
        if not all(math.isfinite(value) for vector in (reference.p, reference.v, reference.a,
            target.p, target.v, target.a, link.odom.p, link.odom.v, link.shared_position)
                   for value in vector):
            if not np.isfinite(nominal).all():
                raise ValueError("nonfinite nominal velocity")
            self.previous_velocity = None
            diagnostics.update(pixel_velocity_reason="invalid_input", pixel_prediction_valid=False)
            return reference, diagnostics
        gains = controller.params.gain
        position_gains = np.array((gains.kp0, gains.kp1, gains.kp2))
        velocity_gains = np.array((gains.kv0, gains.kv1, gains.kv2))
        tracker = TargetState(link.shared_position, link.odom.v)
        safe = np.array((settings.safe_x, settings.safe_y))
        gravity = controller.params.gra
        times = self.predictor.times
        angular_velocity = getattr(link.imu, "w", (0.0, 0.0, 0.0))
        kinematics = self.predictor.image_kinematics(target, tracker, link.odom.q, angular_velocity)
        if kinematics is not None:
            diagnostics.update(kinematics)
            diagnostics["pixel_current_model"] = kinematics["pixel_current"]

        def hinge(value):
            excess = np.maximum(value, 0.0)
            return excess**2 / (excess + 0.01)

        def evaluate(velocity):
            raw = np.asarray(reference.a) + position_gains * (
                np.asarray(reference.p) - np.asarray(link.odom.p)) + velocity_gains * (
                velocity - np.asarray(link.odom.v))
            path = (controller.preview_acceleration_path(tuple(raw), now, times)
                    if controller.active() else [tuple(raw)] * len(times))
            attitudes = []
            for acceleration in path:
                roll, pitch = predict_commanded_tilt(
                    (acceleration[0], acceleration[1], acceleration[2] + gravity), reference.yaw, gravity)
                attitudes.append(_quat_from_zyx(reference.yaw, pitch, roll))
            prediction = self.predictor.predict(target, tracker, link.odom.q, attitudes[0],
                tracker_acceleration_path=path, commanded_attitude_path=attitudes,
                attitude_response_time=self.response_time)
            optical = np.asarray(prediction.relative_positions_camera)
            if not prediction.valid or np.any(optical[:, 2] <= 1e-6):
                return None
            pixels = optical[:, :2] / optical[:, 2, None]
            if not np.isfinite(pixels).all():
                return None
            future = pixels[1:]
            pixel_residual = hinge(np.abs(future) - safe)
            boundary_residual = hinge(np.maximum(self.image_bounds[:, 0] - future,
                                                future - self.image_bounds[:, 1]))
            relative = np.asarray(prediction.relative_positions_world)[1:]
            distances = np.linalg.norm(relative[:, :2], axis=1)
            distance_residual = np.maximum(self.distance_band[0] - distances, 0.0) + np.maximum(
                distances - self.distance_band[1], 0.0)
            count = len(future)
            residual = np.concatenate((math.sqrt(settings.weight_pixel / count) * pixel_residual.ravel(),
                math.sqrt(10.0 * settings.weight_pixel / count) * boundary_residual.ravel(),
                math.sqrt(settings.weight_distance / count) * distance_residual))
            translation = prediction.attitude_predictions["constant_attitude"].relative_positions_camera[-1]
            parts = {"J_velocity": float(np.sum((velocity - nominal)**2)),
                     "J_pixel": float(np.sum(pixel_residual**2 + 10.0 * boundary_residual**2) / count),
                     "J_distance": float(np.mean(distance_residual**2)),
                     "J_smooth": float(np.sum((velocity - self.previous_velocity)**2))
                         if self.previous_velocity is not None else 0.0}
            total = (settings.weight_velocity * parts["J_velocity"] + settings.weight_pixel * parts["J_pixel"]
                     + settings.weight_distance * parts["J_distance"] + settings.weight_smooth * parts["J_smooth"])
            if not math.isfinite(total) or not np.isfinite(residual).all():
                return None
            translation_pixel = (np.asarray(translation[:2]) / translation[2]).tolist() if translation[2] > 1e-6 else None
            return residual, total, parts, pixels, translation_pixel

        measured = np.asarray(image_detection["pixel"], dtype=float)
        camera = self.predictor.camera
        intrinsics = np.asarray(camera.intrinsics)
        live_intrinsics = np.asarray(image_detection.get("intrinsics", intrinsics), dtype=float)
        resolution = image_detection.get("resolution", (camera.width, camera.height))
        if (measured.shape != (2,) or not np.isfinite(measured).all()
                or live_intrinsics.shape != (3, 3) or not np.isfinite(live_intrinsics).all()
                or not np.allclose(live_intrinsics, intrinsics, rtol=0.01, atol=1.0)
                or tuple(resolution) != (camera.width, camera.height)
                or not 0 <= measured[0] < camera.width or not 0 <= measured[1] < camera.height):
            self.previous_velocity = None
            diagnostics.update(pixel_velocity_reason="invalid_detection_or_camera_calibration", pixel_prediction_valid=False)
            return reference, diagnostics
        measured_optical = (measured - intrinsics[:2, 2]) / np.diag(intrinsics)[:2]
        baseline = evaluate(nominal)
        if baseline is None:
            self.previous_velocity = None
            diagnostics.update(pixel_velocity_reason="unprojectable_nominal_prediction", pixel_prediction_valid=False)
            return reference, diagnostics
        projected_now = baseline[3][0].copy()
        offset = measured_optical - projected_now

        def observed_prediction(evaluation):
            residual, _, parts, pixels, translation = evaluation
            corrected_pixels = pixels + offset
            future = corrected_pixels[1:]
            pixel_residual = hinge(np.abs(future) - safe)
            boundary_residual = hinge(np.maximum(self.image_bounds[:, 0] - future,
                                                future - self.image_bounds[:, 1]))
            count = len(future)
            new_parts = dict(parts)
            new_parts["J_pixel"] = float(np.sum(pixel_residual**2 + 10.0 * boundary_residual**2) / count)
            pixel_count = 4 * count
            new_residual = residual.copy()
            new_residual[:2 * count] = math.sqrt(settings.weight_pixel / count) * pixel_residual.ravel()
            new_residual[2 * count:pixel_count] = math.sqrt(10.0 * settings.weight_pixel / count) * boundary_residual.ravel()
            new_total = (settings.weight_velocity * new_parts["J_velocity"] +
                         settings.weight_pixel * new_parts["J_pixel"] +
                         settings.weight_distance * new_parts["J_distance"] +
                         settings.weight_smooth * new_parts["J_smooth"])
            return new_residual, new_total, new_parts, corrected_pixels, translation

        baseline = observed_prediction(baseline)
        best = baseline
        velocity = nominal
        safe_excess = np.abs(baseline[3]) > safe
        first_safe_exit = [next((float(times[index]) for index in range(len(times))
                                if safe_excess[index, axis]), None) for axis in range(2)]
        translation_anchored = (np.asarray(baseline[4]) + offset if baseline[4] is not None else None)
        diagnostics.update(pixel_prediction_valid=True, pixel_pred_nominal=baseline[3][-1].tolist(),
            pixel_pred_translation_only=baseline[4], J_nominal=baseline[1],
            pixel_prediction_times_s=list(times),
            pixel_pred_nominal_path=baseline[3].tolist(),
            pixel_safe_limits_optical=safe.tolist(),
            pixel_nominal_first_safe_exit_s=first_safe_exit,
            pixel_current_safe_violation_axes=safe_excess[0].tolist(),
            pixel_pred_translation_only_anchored=(translation_anchored.tolist()
                if translation_anchored is not None else None),
            pixel_pred_attitude_delta_nominal=(baseline[3][-1] - translation_anchored).tolist()
                if translation_anchored is not None else None,
            delta_v_raw=None, delta_v_bounded=None, pixel_correction_saturated=False,
            pixel_correction_limit_m_s=settings.max_velocity_correction,
            pixel_correction_step_fraction=0.0,
            pixel_source="rgbd_detection", image_pixel_uv=measured.tolist(),
            image_stamp=image_detection["image_stamp"],
            image_arrival_age_s=(now - image_detection["received_at"]
                if "received_at" in image_detection else None),
            pixel_projection_disagreement=offset.tolist(),
            pixel_current_inside_safe=bool(np.all(np.abs(baseline[3][0]) <= safe)),
            pixel_current_fov_violation=bool(np.any(baseline[3][0] < self.image_bounds[:, 0])
                or np.any(baseline[3][0] >= self.image_bounds[:, 1])),
            pixel_nominal_predicted_fov_violation=bool(np.any(baseline[3][1:] < self.image_bounds[:, 0])
                or np.any(baseline[3][1:] > self.image_bounds[:, 1])))
        risk_requested = bool(np.any(np.abs(baseline[3]) > safe))
        if risk_requested:
            columns = []
            for axis in range(3):
                perturbation = np.zeros(3)
                perturbation[axis] = 0.01
                plus, minus = evaluate(nominal + perturbation), evaluate(nominal - perturbation)
                if plus is None or minus is None:
                    columns = []
                    break
                plus, minus = observed_prediction(plus), observed_prediction(minus)
                columns.append((plus[0] - minus[0]) / 0.02)
            if columns:
                jacobian = np.column_stack(columns)
                smooth = settings.weight_smooth if self.previous_velocity is not None else 0.0
                matrix = (settings.weight_velocity + smooth + 1e-9) * np.eye(3) + jacobian.T @ jacobian
                gradient = jacobian.T @ baseline[0]
                if smooth:
                    gradient += smooth * (nominal - self.previous_velocity)
                correction = np.linalg.solve(matrix, -gradient)
                norm = np.linalg.norm(correction)
                diagnostics.update(delta_v_raw=correction.tolist(),
                    pixel_correction_saturated=bool(norm > settings.max_velocity_correction))
                if norm > settings.max_velocity_correction:
                    correction *= settings.max_velocity_correction / norm
                diagnostics["delta_v_bounded"] = correction.tolist()
                for fraction in (1.0, 0.5, 0.25, 0.125):
                    candidate = nominal + fraction * correction
                    predicted = evaluate(candidate)
                    evaluated = observed_prediction(predicted) if predicted is not None else None
                    if evaluated is not None and evaluated[1] < best[1] - 1e-12:
                        velocity, best = candidate, evaluated
                        diagnostics["pixel_correction_step_fraction"] = fraction
        self.previous_velocity = velocity.copy()
        delta = velocity - nominal
        active = bool(np.linalg.norm(delta) > 1e-9)
        diagnostics.update(best[2], J_total=best[1], pixel_current=best[3][0].tolist(),
            pixel_pred_command_path=best[3].tolist(),
            pixel_pred=best[3][-1].tolist(), v_cmd=velocity.tolist(), delta_v=delta.tolist(),
            pixel_predicted_fov_violation=bool(np.any(best[3][1:] < self.image_bounds[:, 0])
                or np.any(best[3][1:] > self.image_bounds[:, 1])),
            pixel_velocity_active=active, pixel_risk_requested=risk_requested,
            pixel_velocity_reason=("corrected" if active else
                                   "no_improving_local_correction" if risk_requested else "safe_pass_through"),
            pixel_velocity_elapsed_ms=1000.0 * (time.perf_counter() - started))
        return (replace(reference, v=tuple(float(value) for value in velocity)) if active else reference), diagnostics


def summarize_velocity_follower(record, settings, distance_band):
    controls = record.get("control_samples", [])
    visibility = [row for row in record.get("visibility_samples", []) if row.get("valid")]
    camera = record.get("visibility_prediction", {}).get("camera", {})
    if record["visual_tracking_mode"] == "predictive" and camera:
        intrinsic = np.asarray(camera["intrinsics"])
        pixels = np.asarray([(np.asarray(row["image_pixel_uv"]) - intrinsic[:2, 2]) /
                             np.diag(intrinsic)[:2] for row in controls if "image_pixel_uv" in row])
        missing_pixels = len(controls) - len(pixels)
        loss_count = sum(bool(row.get("pixel_current_fov_violation")) for row in controls
                         if "image_pixel_uv" in row)
        pixel_basis = "observed_rgbd_detection"
    else:
        optical = [row["relative_positions_camera"][0] for row in visibility
                   if row.get("relative_positions_camera")]
        pixels = np.asarray([np.asarray(point[:2]) / point[2] for point in optical if point[2] > 1e-6])
        missing_pixels = len(record.get("visibility_samples", [])) - len(visibility)
        loss_count = sum(point[2] <= 1e-6 for point in optical) + sum(
            bool(row["attitude_predictions"]["constant_attitude"]["out_of_fov"][0])
            for row in visibility if row.get("relative_positions_camera") and row["relative_positions_camera"][0][2] > 1e-6)
        pixel_basis = "state_projection_not_image_detection"

    def rmse(values):
        return float(np.sqrt(np.mean(np.sum(np.asarray(values)**2, axis=-1)))) if values else None

    corrections = [np.asarray(row.get("delta_v", (0.0, 0.0, 0.0))) for row in controls]
    accelerations = [row["limited_acceleration"] for row in controls if "limited_acceleration" in row]
    jerks = [(np.asarray(right["limited_acceleration"]) - np.asarray(left["limited_acceleration"]))
             / (right["t"] - left["t"]) for left, right in zip(controls, controls[1:])
             if "limited_acceleration" in left and "limited_acceleration" in right
             and 0 < right["t"] - left["t"] <= 0.2]
    velocity_errors = [np.asarray(row["target_v"]) - np.asarray(row["odom_v"]) for row in controls]
    position_errors = [np.asarray(row["nominal_reference_p"]) - np.asarray(row["odom_p"])
                       for row in controls if "nominal_reference_p" in row]
    distance_errors = [[max(distance_band[0] - row["horizontal_distance"], 0.0,
                           row["horizontal_distance"] - distance_band[1])] for row in controls]
    return {"mode": record["visual_tracking_mode"], "control_sample_count": len(controls),
        "visibility_valid_sample_count": len(visibility), "projectable_sample_count": len(pixels),
        "visibility_missing_sample_count": missing_pixels, "pixel_metric_source": pixel_basis,
        "nominal_position_tracking_rmse_m": rmse(position_errors),
        "target_velocity_tracking_rmse_m_s": rmse(velocity_errors),
        "pixel_center_rmse_optical": float(np.sqrt(np.mean(np.sum(pixels**2, axis=1)))) if len(pixels) else None,
        "pixel_max_abs_xy_optical": np.max(np.abs(pixels), axis=0).tolist() if len(pixels) else None,
        "fov_loss_sample_count": loss_count,
        "outside_safe_projectable_sample_fraction": float(np.mean(np.any(np.abs(pixels) >
            (settings.safe_x, settings.safe_y), axis=1))) if len(pixels) else None,
        "velocity_correction_rmse_m_s": rmse(corrections),
        "velocity_intervention_sample_fraction": sum(row.get("pixel_velocity_active", False)
            for row in controls) / len(controls) if controls else None,
        "pixel_intervention_sample_fraction": sum(row.get("visibility_filter_active", False)
            for row in controls) / len(controls) if controls else None,
        "command_acceleration_rmse_m_s2": rmse(accelerations), "command_jerk_rmse_m_s3": rmse(jerks),
        "distance_band_error_rmse_m": rmse(distance_errors),
        "statistics_basis": "sample_weighted_valid_pixels; acceleration_and_jerk_command_proxies"}