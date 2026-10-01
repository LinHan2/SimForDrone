"""Passive tracker adapter and offline CSV/figure export; no transport or control writes."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path
import time

import yaml

from px4ctrl.controller import _quat_from_zyx
from px4ctrl.inputs import yaw_from_quaternion
from tracking.estimation import observer_world_acceleration
from tracking.guidance import TargetState
from tracking.visibility_prediction import CameraGeometry, VisibilityConfig, VisibilityPredictor


class VisibilityDiagnostics:
    def __init__(self, config: dict, root: Path) -> None:
        self.predictor = None
        self.metadata = {
            "enabled": True,
            "pixel_source": "state_projection_not_detection",
            "calibration_source": "static_scene_yaml_not_live_CameraInfo",
            "position_source": "PX4_shared_ENU_not_Isaac_GT",
            "time_alignment": "latest_available_unaligned; publication/arrival_times_not_acquisition_times",
            "command_attitude_model": "actual_at_t0; instantaneous_held_world_setpoint_for_future",
            "distance_metric": "3D_Euclidean_not_horizontal_control_band",
            "occlusion_and_target_extent_modeled": False,
        }
        self.max_input_age_sec = 0.2
        try:
            options = dict(config)
            camera_path = root / options.pop("camera_config", "configs/dual_uav_outdoor.yaml")
            camera_override = options.pop("camera_intrinsics_override", None)
            self.max_input_age_sec = float(options.pop("max_input_age_sec", 0.2))
            if not math.isfinite(self.max_input_age_sec) or self.max_input_age_sec <= 0:
                raise ValueError("max_input_age_sec must be finite and positive")
            camera_options = yaml.safe_load(camera_path.read_text(encoding="utf-8"))["observer_camera"]
            orientation = camera_options["orientation_deg"]
            if orientation != [0.0, 0.0, 180.0]:
                raise ValueError("Only the verified front FLU mount [0, 0, 180] is supported by this adapter")
            # 运行时 CameraInfo 可能不同于场景 YAML；显式快照只影响预测投影。
            intrinsics = camera_override if camera_override is not None else camera_options["intrinsics"]
            camera = CameraGeometry(
                tuple(tuple(row) for row in intrinsics),
                camera_options["resolution"][0], camera_options["resolution"][1],
                tuple(camera_options["position"]),
            )
            settings = VisibilityConfig(**options)
            self.predictor = VisibilityPredictor(camera, settings)
            self.metadata.update(camera=asdict(camera), config=asdict(settings),
                                 camera_config=str(camera_path), max_input_age_sec=self.max_input_age_sec)
            if camera_override is not None:
                self.metadata["calibration_source"] = "explicit_CameraInfo_snapshot_override_not_live_subscription"
        except Exception as error:
            self.metadata["initialization_error"] = f"{type(error).__name__}: {error}"

    def sample(self, *, now, started, target_state, target_message, link, reference,
               output, debug, state_source) -> dict:
        started_compute = time.perf_counter()
        row = {"t": now - started, "monotonic_s": now, "valid": False}
        try:
            if self.predictor is None:
                raise ValueError(self.metadata["initialization_error"])
            target_age = now - target_message.timestamp
            odom_age = now - link.odom.recv_time
            attitude_age = now - link.odom.attitude_recv_time
            imu_age = now - link.imu.recv_time
            reliable = (0 <= imu_age <= self.max_input_age_sec
                        and 0 <= attitude_age <= self.max_input_age_sec and not debug.tilt_saturated)
            acceleration = (0.0, 0.0, 0.0)
            reason = None
            if reliable:
                try:
                    acceleration = observer_world_acceleration(link.imu.q, link.imu.acc)
                    if not all(math.isfinite(value) for value in acceleration):
                        raise ValueError("nonfinite acceleration")
                except (ValueError, TypeError):
                    reliable = False
                    reason = "invalid_imu"
            else:
                reason = "tilt_saturated" if debug.tilt_saturated else "stale_imu_or_attitude"
            tracker_state = TargetState(link.shared_position, link.odom.v, acceleration)
            desired_world_attitude = _quat_from_zyx(reference.yaw, debug.pitch, debug.roll)
            attitude = link.odom.q
            norm = math.sqrt(sum(value * value for value in attitude))
            quat_x, quat_y, quat_z, quat_w = (value / norm for value in attitude)
            rpy = (math.atan2(2 * (quat_w * quat_x + quat_y * quat_z),
                              1 - 2 * (quat_x**2 + quat_y**2)),
                   math.asin(max(-1.0, min(1.0, 2 * (quat_w * quat_y - quat_z * quat_x)))),
                   yaw_from_quaternion((quat_x, quat_y, quat_z, quat_w)))
            row.update(
                target_age_sec=target_age, odom_age_sec=odom_age,
                attitude_age_sec=attitude_age, imu_age_sec=imu_age,
                target_timestamp_monotonic_s=target_message.timestamp,
                target_sequence=target_message.sequence, state_source=state_source,
                target_acceleration_source=("trajectory_reference_or_zero" if state_source == "truth"
                                            else "selected_estimator"),
                tracker_acceleration_source="IMU_specific_force_rotated_minus_gravity",
                tracker_acceleration_reliable=reliable, acceleration_fallback_reason=reason,
                target_state=asdict(target_state), tracker_state=asdict(tracker_state),
                tracker_attitude=attitude, tracker_rpy=rpy, desired_world_attitude=desired_world_attitude,
                command={"p": reference.p, "v": reference.v, "a": reference.a, "yaw": reference.yaw,
                         "output_q": output.q, "output_thrust": output.thrust,
                         "tilt_saturated": debug.tilt_saturated},
            )
            if not all(0 <= age <= self.max_input_age_sec for age in (target_age, odom_age, attitude_age)):
                row["reason"] = "stale_or_future_timestamp"
            else:
                row.update(asdict(self.predictor.predict(
                    target_state, tracker_state, attitude, desired_world_attitude,
                    tracker_acceleration_reliable=reliable)))
                row["prediction_timestamps_monotonic_s"] = [now + value for value in row["prediction_times"]]
            json.dumps(row, allow_nan=False)
        except Exception as error:
            row = {"t": now - started, "monotonic_s": now, "valid": False,
                   "reason": f"{type(error).__name__}: {error}"}
        row["compute_ms"] = (time.perf_counter() - started_compute) * 1000
        return row


def summarize_tracking(record, *, central_margin=0.7, min_coverage=0.95,
                       min_central_fraction=0.9, max_loss_fraction=0.0, max_gap=0.2):
    """按完整跟踪时间统计当前投影；缺测不算可见，预测不冒充观测验收。"""
    if (not 0 < central_margin < 1 or not 0 < min_coverage <= 1
            or not 0 <= min_central_fraction <= 1 or not 0 <= max_loss_fraction <= 1
            or not math.isfinite(max_gap) or max_gap <= 0):
        raise ValueError("FOV 验收参数无效")
    controls = record.get("control_samples", [])
    rows = record.get("visibility_samples", [])
    timeline = controls or rows
    start = timeline[0]["t"] if timeline else 0.0
    end = timeline[-1]["t"] if timeline else 0.0
    duration = max(0.0, end - start)
    covered = lost = central = longest_loss = loss_streak = 0.0

    def current_state(row):
        if not row.get("valid"):
            return None
        projection = row.get("attitude_predictions", {}).get("constant_attitude", {})
        states = projection.get("projection_status", [])
        normal = projection.get("normalized_pixels_pred", [])
        if not states or not normal:
            return None
        if states[0] in ("outside", "boundary", "behind_camera", "near_camera_plane"):
            return True, False
        if normal[0] is None or not all(math.isfinite(value) for value in normal[0]):
            return None
        return False, max(abs(value) for value in normal[0]) <= central_margin

    for left, right in zip(rows, rows[1:]):
        dt = right["t"] - left["t"]
        interval = max(0.0, min(end, right["t"]) - max(start, left["t"]))
        state = current_state(left)
        if not 0 < dt <= max_gap or state is None or current_state(right) is None:
            loss_streak = 0.0
            continue
        covered += interval
        if state[0]:
            lost += interval
            loss_streak += interval
            longest_loss = max(longest_loss, loss_streak)
        else:
            loss_streak = 0.0
        if state[1]:
            central += interval
    coverage = covered / duration if duration else 0.0
    loss_fraction = lost / covered if covered else None
    central_fraction = central / covered if covered else None
    fov_passed = None
    if covered > 0 and coverage >= min_coverage:
        fov_passed = loss_fraction <= max_loss_fraction and central_fraction >= min_central_fraction
    result = {
        "status": "not_evaluated" if fov_passed is None else
                  "requires_stability_review" if fov_passed else "fov_failed",
        "scope": "current_point_projection_not_detection; no_occlusion_or_target_extent",
        "fov_passed": fov_passed, "stability_passed": None,
        "duration_s": duration, "valid_coverage_fraction": coverage,
        "unknown_duration_s": max(0.0, duration - covered),
        "geometric_loss_fraction": loss_fraction, "central_fraction": central_fraction,
        "longest_observed_loss_s": longest_loss,
        "thresholds": {"central_margin": central_margin, "min_coverage": min_coverage,
                       "min_central_fraction": min_central_fraction,
                       "max_loss_fraction": max_loss_fraction, "max_gap_s": max_gap},
        "position_error_is_acceptance_gate": False,
    }
    for label in ("actual_roll", "actual_pitch", "desired_roll", "desired_pitch"):
        values = [row[label] for row in controls if label in row and math.isfinite(row[label])]
        rates = []
        for left, right in zip(controls, controls[1:]):
            clock = "attitude_recv_monotonic_s" if label.startswith("actual") else "t"
            dt = right.get(clock, right["t"]) - left.get(clock, left["t"])
            if label in left and label in right and 0 < dt <= max_gap:
                delta = math.atan2(math.sin(right[label] - left[label]), math.cos(right[label] - left[label]))
                if math.isfinite(delta):
                    rates.append(math.degrees(delta) / dt)
        result[label + "_peak_deg"] = max(map(abs, map(math.degrees, values)), default=None)
        result[label + "_rate_rms_deg_s"] = math.sqrt(sum(value**2 for value in rates) / len(rates)) if rates else None
    distances = [row["horizontal_distance"] for row in controls if "horizontal_distance" in row]
    violations = [row["distance_band_violation"] for row in controls
                  if row.get("distance_band_violation") is not None]
    result["horizontal_distance_min_m"] = min(distances, default=None)
    result["horizontal_distance_max_m"] = max(distances, default=None)
    result["distance_band_violation_max_m"] = max(violations, default=None)
    for label in ("acceleration_limited", "jerk_limited", "tilt_limited"):
        flags = [row[label] for row in controls if label in row]
        result[label + "_sample_fraction"] = sum(flags) / len(flags) if flags else None
    return result


def _anchor(row: dict, key: str, horizon: float):
    times = row.get("prediction_times", [])
    index = next((index for index, value in enumerate(times) if abs(value - horizon) < 1e-9), None)
    values = row.get(key, [])
    return values[index] if index is not None and index < len(values) else None


def _flat_row(row: dict) -> dict:
    flat = {key: row.get(key) for key in (
        "t", "monotonic_s", "valid", "reason", "compute_ms", "target_age_sec", "odom_age_sec",
        "attitude_age_sec", "imu_age_sec", "predicted_fov_exit", "predicted_fov_exit_time_sec",
        "current_distance", "min_distance", "max_distance", "predicted_distance_at_horizon",
        "max_image_error", "fov_cost", "distance_cost", "translation_mode_used", "attitude_mode_used",
        "acceleration_fallback_reason",
    )}
    flat["distance_0p5_m"] = _anchor(row, "distances_pred", 0.5)
    for mode in ("constant_attitude", "command_based_attitude"):
        projection = row.get("attitude_predictions", {}).get(mode, {})
        flat[f"{mode}_exit_sec"] = projection.get("predicted_fov_exit_time_sec")
        for elapsed, label in ((0.0, "now"), (0.2, "0p2"), (0.5, "0p5")):
            point = _anchor({**projection, "prediction_times": row.get("prediction_times", [])}, "pixels_pred", elapsed)
            for axis, name in enumerate(("u", "v")):
                flat[f"{mode}_{label}_{name}"] = point[axis] if point is not None else None
    for key, value, names in (
        ("target_v", row.get("target_state", {}).get("v"), ("e", "n", "u")),
        ("target_a", row.get("target_state", {}).get("a"), ("e", "n", "u")),
        ("tracker", row.get("tracker_rpy"), ("roll", "pitch", "yaw")),
        ("command_a", row.get("command", {}).get("a"), ("e", "n", "u")),
        ("command_q", row.get("command", {}).get("output_q"), ("x", "y", "z", "w")),
    ):
        for index, name in enumerate(names):
            flat[f"{key}_{name}"] = value[index] if value is not None else None
    flat["command_thrust"] = row.get("command", {}).get("output_thrust")
    return flat


def export_visibility(record: dict, output_dir: Path) -> list[Path]:
    rows = record.get("visibility_samples", [])
    if not rows:
        return []
    output_dir.mkdir(parents=True, exist_ok=True)
    flat_rows = [_flat_row(row) for row in rows]
    csv_path = output_dir / "visibility_prediction.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat_rows[0]))
        writer.writeheader()
        writer.writerows(flat_rows)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def column(name):
        return [float(row[name]) if row.get(name) is not None else math.nan for row in flat_rows]

    times = column("t")
    figure, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    for index, name in enumerate(("u", "v")):
        axes[index].plot(times, column(f"constant_attitude_now_{name}"), color="black", label="Observed state projection (not detection)")
        for mode, color in (("constant_attitude", "tab:blue"), ("command_based_attitude", "tab:orange")):
            for delay, tag, style in ((0.2, "0p2", ":"), (0.5, "0p5", "--")):
                axes[index].plot([value + delay for value in times], column(f"{mode}_{tag}_{name}"),
                                 style, color=color, label=f"{mode}, +{delay}s (time aligned)")
        camera = record.get("visibility_prediction", {}).get("camera", {})
        limit = camera.get("width" if name == "u" else "height")
        if limit:
            axes[index].axhline(0, color="red", linewidth=0.7)
            axes[index].axhline(limit, color="red", linewidth=0.7)
        axes[index].set_ylabel(f"{name} [px]")
    axes[2].plot(times, column("current_distance"), label="Current 3D distance")
    axes[2].plot([value + 0.5 for value in times], column("distance_0p5_m"), "--", label="Predicted +0.5s (time aligned)")
    config = record.get("visibility_prediction", {}).get("config", {})
    for value in (config.get("distance_min", 5), config.get("distance_max", 10)):
        axes[2].axhline(value, color="gray", linewidth=0.7)
    axes[2].set_ylabel("Distance [m]")
    for mode in ("constant_attitude", "command_based_attitude"):
        axes[3].plot(times, column(f"{mode}_exit_sec"), ".-", label=mode)
    axes[3].set_ylabel("Time to FOV exit [s]")
    axes[3].set_xlabel("Run time [s]; missing exit = none within horizon")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(fontsize=7, loc="best")
    figure.suptitle("Visibility diagnostics: point geometry only; no control feedback")
    figure.tight_layout()
    image_path = output_dir / "visibility_prediction.png"
    figure.savefig(str(image_path), dpi=140)
    plt.close(figure)
    return [csv_path, image_path]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_json", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    record = json.loads(args.run_json.read_text(encoding="utf-8"))
    for path in export_visibility(record, args.output_dir or args.run_json.parent):
        print(path)


if __name__ == "__main__":
    main()