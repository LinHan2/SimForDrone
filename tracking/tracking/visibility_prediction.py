"""仅预测未来几何与可见性；不持有控制器，不增广动力学状态。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from tracking.guidance import TargetState, Vector3
from tracking.shadow_observation import rotate_vector, world_to_camera, world_to_camera_path

Quaternion = tuple[float, float, float, float]
Pixel = tuple[float, float]


@dataclass(frozen=True)
class CameraGeometry:
    intrinsics: tuple[tuple[float, ...], ...]
    width: int
    height: int
    offset_body: Vector3
    forward_sign: float = 1.0

    def __post_init__(self) -> None:
        matrix = np.asarray(self.intrinsics, dtype=float)
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], (0, 0, 1))
                or matrix[0, 1] != 0 or matrix[1, 0] != 0):
            raise ValueError("需要无 skew 的有限针孔内参")
        if (not isinstance(self.width, int) or not isinstance(self.height, int)
                or self.width <= 0 or self.height <= 0
                or not 0 <= matrix[0, 2] < self.width
                or not 0 <= matrix[1, 2] < self.height):
            raise ValueError("分辨率或主点无效")
        if not _finite_vector(self.offset_body, 3) or self.forward_sign not in (-1.0, 1.0):
            raise ValueError("相机安装平移或前向符号无效")


@dataclass(frozen=True)
class VisibilityConfig:
    prediction_horizon_sec: float = 0.8
    prediction_dt: float = 0.05
    translation_mode: str = "constant_acceleration"
    attitude_mode: str = "command_based_attitude"
    fov_safe_ratio: float = 0.6
    fov_warning_ratio: float = 0.8
    distance_min: float = 5.0
    distance_max: float = 10.0
    fov_weight: float = 1.0
    distance_min_weight: float = 1.0
    distance_max_weight: float = 1.0
    target_radius_m: float = 0.30
    pixel_occupancy_min_ratio: float = 0.003
    pixel_occupancy_max_ratio: float = 0.008
    pixel_occupancy_weight: float = 10.0

    def __post_init__(self) -> None:
        numbers = (self.prediction_horizon_sec, self.prediction_dt, self.fov_safe_ratio,
                   self.fov_warning_ratio, self.distance_min, self.distance_max,
                   self.fov_weight, self.distance_min_weight, self.distance_max_weight,
                   self.target_radius_m, self.pixel_occupancy_min_ratio,
                   self.pixel_occupancy_max_ratio, self.pixel_occupancy_weight)
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError("预测配置必须有限")
        if not 0 < self.prediction_dt <= self.prediction_horizon_sec:
            raise ValueError("需要 0 < prediction_dt <= prediction_horizon_sec")
        if self.prediction_horizon_sec / self.prediction_dt > 200:
            raise ValueError("旁路预测最多支持 200 个时间步")
        if not 0 < self.fov_safe_ratio < self.fov_warning_ratio < 1:
            raise ValueError("需要 0 < safe < warning < 1")
        if not 0 < self.distance_min <= self.distance_max:
            raise ValueError("距离带无效")
        if min(self.fov_weight, self.distance_min_weight, self.distance_max_weight) < 0:
            raise ValueError("代价权重不得为负")
        if (self.target_radius_m <= 0 or self.pixel_occupancy_weight < 0
                or not 0 < self.pixel_occupancy_min_ratio < self.pixel_occupancy_max_ratio < 1):
            raise ValueError("目标尺寸、像素占比范围或占比权重无效")
        if self.translation_mode not in ("constant_velocity", "constant_acceleration"):
            raise ValueError("未知平移预测模式")
        if self.attitude_mode not in ("constant_attitude", "command_based_attitude"):
            raise ValueError("未知姿态预测模式")


@dataclass(frozen=True)
class FovPrediction:
    relative_positions_camera: list[Vector3]
    pixels_pred: list[Pixel | None]
    normalized_pixels_pred: list[Pixel | None]
    behind_camera: list[bool]
    safe: list[bool]
    warning: list[bool]
    out_of_fov: list[bool]
    projection_status: list[str]
    predicted_fov_exit: bool
    predicted_fov_exit_time_sec: float | None
    max_image_error: float | None
    fov_cost: float | None
    pixel_occupancy_pred: list[float | None]


@dataclass(frozen=True)
class VisibilityPrediction:
    valid: bool = False
    reason: str | None = None
    prediction_times: list[float] = field(default_factory=list)
    target_positions_pred: list[Vector3] = field(default_factory=list)
    target_velocities_pred: list[Vector3] = field(default_factory=list)
    tracker_positions_pred: list[Vector3] = field(default_factory=list)
    tracker_velocities_pred: list[Vector3] = field(default_factory=list)
    relative_positions_world: list[Vector3] = field(default_factory=list)
    relative_positions_camera: list[Vector3] = field(default_factory=list)
    pixels_pred: list[Pixel | None] = field(default_factory=list)
    normalized_pixels_pred: list[Pixel | None] = field(default_factory=list)
    distances_pred: list[float] = field(default_factory=list)
    current_distance: float | None = None
    min_distance: float | None = None
    max_distance: float | None = None
    predicted_distance_at_horizon: float | None = None
    predicted_fov_exit: bool = False
    predicted_fov_exit_time_sec: float | None = None
    max_image_error: float | None = None
    fov_cost: float | None = None
    distance_cost: float | None = None
    pixel_occupancy_pred: list[float | None] = field(default_factory=list)
    pixel_occupancy_cost: float | None = None
    translation_mode_used: str = "unavailable"
    attitude_mode_used: str = "unavailable"
    command_attitude_available: bool = False
    attitude_predictions: dict[str, FovPrediction] = field(default_factory=dict)


def _finite_vector(value, size: int) -> bool:
    return len(value) == size and all(math.isfinite(component) for component in value)


def _valid_attitude(attitude: Quaternion | None) -> bool:
    return (attitude is not None and _finite_vector(attitude, 4)
            and sum(component * component for component in attitude) > 1e-12)


def _vector3(values) -> Vector3:
    return (float(values[0]), float(values[1]), float(values[2]))


def _trajectory(state: TargetState, times: list[float], acceleration: Vector3) -> list[Vector3]:
    return [_vector3([state.p[axis] + state.v[axis] * elapsed
                     + 0.5 * acceleration[axis] * elapsed**2 for axis in range(3)])
            for elapsed in times]


def _acceleration_trajectory(state, times, accelerations):
    positions, velocities = [state.p], [state.v]
    for index in range(1, len(times)):
        dt = times[index] - times[index - 1]
        acceleration = accelerations[index - 1]
        positions.append(tuple(positions[-1][axis] + velocities[-1][axis] * dt
                               + 0.5 * acceleration[axis] * dt**2 for axis in range(3)))
        velocities.append(tuple(velocities[-1][axis] + acceleration[axis] * dt for axis in range(3)))
    return positions, velocities


def _attitude_trajectory(current, commands, times, response_time):
    norm = math.hypot(*current)
    attitude = tuple(value / norm for value in current)
    path = [attitude]
    for index in range(1, len(times)):
        norm = math.hypot(*commands[index])
        command = tuple(value / norm for value in commands[index])
        if sum(actual * desired for actual, desired in zip(attitude, command)) < 0.0:
            command = tuple(-value for value in command)
        blend = (1.0 if response_time == 0.0 else
                 -math.expm1(-(times[index] - times[index - 1]) / response_time))
        attitude = tuple((1.0 - blend) * actual + blend * desired
                 for actual, desired in zip(attitude, command))
        norm = math.hypot(*attitude)
        attitude = tuple(value / norm for value in attitude)
        path.append(attitude)
    return path


class VisibilityPredictor:
    def image_kinematics(self, target, tracker, attitude, angular_velocity_body):
        """Optical X/Z,Y/Z and separate translation/rotation rates; inputs ENU/FLU."""
        if (not _valid_attitude(attitude) or not all(_finite_vector(vector, 3) for vector in
            (target.p, target.v, tracker.p, tracker.v, angular_velocity_body))):
            return None
        optical = world_to_camera(target.p, tracker.p, attitude, self.camera.offset_body,
                                  self.camera.forward_sign)
        if optical[2] <= 1e-6:
            return None
        rotation = np.column_stack([world_to_camera(tuple(axis), (0.0, 0.0, 0.0), attitude,
            (0.0, 0.0, 0.0), self.camera.forward_sign) for axis in np.eye(3)])
        relative_velocity = np.asarray(target.v) - np.asarray(tracker.v)
        omega_world = rotate_vector(attitude, angular_velocity_body)
        omega_camera = rotation @ omega_world
        lever_velocity = rotate_vector(attitude, tuple(np.cross(
            angular_velocity_body, self.camera.offset_body)))
        translation = rotation @ relative_velocity
        rotational = -rotation @ lever_velocity - np.cross(omega_camera, optical)
        depth = optical[2]
        jacobian = np.array(((1.0 / depth, 0.0, -optical[0] / depth**2),
                             (0.0, 1.0 / depth, -optical[1] / depth**2)))
        return {"pixel_current": (optical[:2] / depth).tolist(),
                "pixel_rate_translation": (jacobian @ translation).tolist(),
                "pixel_rate_rotation": (jacobian @ rotational).tolist(),
                "pixel_rate_total": (jacobian @ (translation + rotational)).tolist()}

    def __init__(self, camera: CameraGeometry, config: VisibilityConfig | None = None) -> None:
        self.camera = camera
        self.config = config or VisibilityConfig()
        count = math.ceil(self.config.prediction_horizon_sec / self.config.prediction_dt)
        times = [min(index * self.config.prediction_dt, self.config.prediction_horizon_sec)
                 for index in range(count + 1)]
        # 诊断锚点精确求值，避免把相邻采样误标为 0.2/0.5 秒预测。
        times.extend(value for value in (0.2, 0.5) if value <= self.config.prediction_horizon_sec)
        self.times = sorted({round(value, 12) for value in times})

    def predict(
        self, target_state: TargetState, tracker_state: TargetState,
        tracker_attitude: Quaternion, commanded_attitude: Quaternion | None = None,
        *, tracker_acceleration_reliable: bool = True,
        tracker_acceleration_path: list[Vector3] | None = None,
        commanded_attitude_path: list[Quaternion] | None = None,
        attitude_response_time: float = 0.0,
        include_attitude_comparison: bool = True,
    ) -> VisibilityPrediction:
        """两机状态须已对齐至同一 ENU 时刻；t=0 总用实测姿态，未来可用保持的指令姿态。"""
        vectors = (target_state.p, target_state.v, target_state.a, tracker_state.p, tracker_state.v)
        if not all(_finite_vector(vector, 3) for vector in vectors) or not _valid_attitude(tracker_attitude):
            return VisibilityPrediction(reason="invalid_state_or_attitude")
        if (not math.isfinite(attitude_response_time) or attitude_response_time < 0.0
                or (tracker_acceleration_path is not None and
                    (len(tracker_acceleration_path) != len(self.times)
                     or not all(_finite_vector(value, 3) for value in tracker_acceleration_path)))
                or (commanded_attitude_path is not None and
                    (len(commanded_attitude_path) != len(self.times)
                     or not all(_valid_attitude(value) for value in commanded_attitude_path)))):
            return VisibilityPrediction(reason="invalid_control_path")
        use_acceleration = (self.config.translation_mode == "constant_acceleration"
                            and tracker_acceleration_reliable and _finite_vector(tracker_state.a, 3))
        acceleration = tracker_state.a if use_acceleration else (0.0, 0.0, 0.0)
        target_positions = _trajectory(target_state, self.times, target_state.a)
        acceleration_path = (tracker_acceleration_path if use_acceleration and tracker_acceleration_path is not None
                     else [acceleration] * len(self.times))
        tracker_positions, tracker_velocities = _acceleration_trajectory(tracker_state, self.times, acceleration_path)
        if not all(_finite_vector(position, 3) for position in target_positions + tracker_positions):
            return VisibilityPrediction(reason="prediction_overflow")
        relative = [_vector3([target[axis] - tracker[axis] for axis in range(3)])
                    for target, tracker in zip(target_positions, tracker_positions)]
        distances = [math.hypot(*vector) for vector in relative]
        command_available = commanded_attitude_path is not None or _valid_attitude(commanded_attitude)
        mode = self.config.attitude_mode
        if mode == "command_based_attitude" and not command_available:
            mode = "constant_attitude"
        projections = {}
        if include_attitude_comparison or mode == "constant_attitude":
            projections["constant_attitude"] = self._project(
                target_positions, tracker_positions, tracker_attitude, tracker_attitude)
        if command_available and (include_attitude_comparison or mode == "command_based_attitude"):
            commands = (commanded_attitude_path if commanded_attitude_path is not None
                        else [commanded_attitude] * len(self.times))
            attitude_path = _attitude_trajectory(tracker_attitude, commands, self.times, attitude_response_time)
            projections["command_based_attitude"] = self._project(
                target_positions, tracker_positions, tracker_attitude, commanded_attitude,
                attitude_path=attitude_path)
        elif include_attitude_comparison:
            projections["command_based_attitude"] = projections["constant_attitude"]
        primary = projections[mode]
        distance_cost = sum(
            self.config.distance_min_weight * max(0.0, self.config.distance_min - distance)**2
            + self.config.distance_max_weight * max(0.0, distance - self.config.distance_max)**2
            for distance in distances)
        occupancy_cost = 0.0
        for ratio in primary.pixel_occupancy_pred:
            if ratio is None:
                occupancy_cost += 4.0
            elif ratio < self.config.pixel_occupancy_min_ratio:
                occupancy_cost += math.log(self.config.pixel_occupancy_min_ratio / ratio) ** 2
            elif ratio > self.config.pixel_occupancy_max_ratio:
                occupancy_cost += math.log(ratio / self.config.pixel_occupancy_max_ratio) ** 2
        return VisibilityPrediction(
            valid=True, prediction_times=list(self.times), target_positions_pred=target_positions,
            target_velocities_pred=[_vector3([target_state.v[axis] + target_state.a[axis] * elapsed
                                             for axis in range(3)]) for elapsed in self.times],
            tracker_positions_pred=tracker_positions, tracker_velocities_pred=tracker_velocities,
            relative_positions_world=relative,
            relative_positions_camera=primary.relative_positions_camera,
            pixels_pred=primary.pixels_pred, normalized_pixels_pred=primary.normalized_pixels_pred,
            distances_pred=distances, current_distance=distances[0], min_distance=min(distances),
            max_distance=max(distances), predicted_distance_at_horizon=distances[-1],
            predicted_fov_exit=primary.predicted_fov_exit,
            predicted_fov_exit_time_sec=primary.predicted_fov_exit_time_sec,
            max_image_error=primary.max_image_error, fov_cost=primary.fov_cost,
            distance_cost=distance_cost, pixel_occupancy_pred=primary.pixel_occupancy_pred,
            pixel_occupancy_cost=occupancy_cost,
            translation_mode_used="constant_acceleration" if use_acceleration else "constant_velocity",
            attitude_mode_used=mode, command_attitude_available=command_available,
            attitude_predictions=projections,
        )

    def _project(self, target_positions, tracker_positions, current_attitude, future_attitude,
                 *, attitude_path=None) -> FovPrediction:
        camera = self.camera
        matrix = np.asarray(camera.intrinsics)
        optical_positions, pixels, normalized, occupancies = [], [], [], []
        behind, safe, warning, outside, statuses = [], [], [], [], []
        errors, costs = [], []
        exit_time = None
        attitudes = (attitude_path if attitude_path is not None else
                     [current_attitude] + [future_attitude] * (len(target_positions) - 1))
        optical_path = world_to_camera_path(target_positions, tracker_positions, attitudes,
                                            camera.offset_body, camera.forward_sign)
        for index, optical in enumerate(optical_path):
            projection = None if optical[2] <= 1e-6 else (
                matrix[0, 0] * optical[0] / optical[2] + matrix[0, 2],
                matrix[1, 1] * optical[1] / optical[2] + matrix[1, 2])
            optical_positions.append(_vector3(optical))
            # 无检测框时用球形目标代理估算投影面积；半径来自配置，不能冒充视觉实测占比。
            depth = float(optical[2])
            radius = self.config.target_radius_m
            if depth <= radius:
                occupancies.append(None)
            else:
                tangent_depth = math.sqrt(depth * depth - radius * radius)
                radius_x = matrix[0, 0] * radius / tangent_depth
                radius_y = matrix[1, 1] * radius / tangent_depth
                occupancy = math.pi * radius_x * radius_y / (camera.width * camera.height)
                occupancies.append(min(1.0, float(occupancy)))
            behind.append(bool(optical[2] <= 0))
            if projection is None:
                pixels.append(None)
                normalized.append(None)
                safe.append(False)
                warning.append(True)
                outside.append(True)
                statuses.append("behind_camera" if behind[-1] else "near_camera_plane")
                exits = True
            else:
                pixel = (float(projection[0]), float(projection[1]))
                normal = (float((pixel[0] - matrix[0, 2]) / (camera.width / 2)),
                          float((pixel[1] - matrix[1, 2]) / (camera.height / 2)))
                error = max(abs(value) for value in normal)
                pixels.append(pixel)
                normalized.append(normal)
                safe.append(error < self.config.fov_safe_ratio)
                warning.append(error > self.config.fov_warning_ratio)
                # 主点偏心时同时检查真实图像边界。
                outside.append(error > 1 or not (0 <= pixel[0] <= camera.width and 0 <= pixel[1] <= camera.height))
                exits = error >= 1 or not (0 < pixel[0] < camera.width and 0 < pixel[1] < camera.height)
                statuses.append("outside" if outside[-1] else "boundary" if exits
                                else "safe" if safe[-1] else "warning" if warning[-1] else "margin")
                errors.append(error)
                costs.append(self.config.fov_weight * sum(max(0.0, abs(value) - self.config.fov_safe_ratio)**2
                                                         for value in normal))
            if exits and exit_time is None:
                exit_time = self.times[index]
        projectable = all(pixel is not None for pixel in pixels)
        return FovPrediction(optical_positions, pixels, normalized, behind, safe, warning, outside, statuses,
                             exit_time is not None, exit_time, max(errors) if projectable else None,
                             sum(costs) if projectable else None, occupancies)