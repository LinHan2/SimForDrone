"""仅预测未来几何与可见性；不持有控制器，不增广动力学状态。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from tracking.guidance import TargetState, Vector3
from tracking.shadow_observation import project_oracle_pixel, world_to_camera

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

    def __post_init__(self) -> None:
        numbers = (self.prediction_horizon_sec, self.prediction_dt, self.fov_safe_ratio,
                   self.fov_warning_ratio, self.distance_min, self.distance_max,
                   self.fov_weight, self.distance_min_weight, self.distance_max_weight)
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


@dataclass(frozen=True)
class VisibilityPrediction:
    valid: bool = False
    reason: str | None = None
    prediction_times: list[float] = field(default_factory=list)
    target_positions_pred: list[Vector3] = field(default_factory=list)
    target_velocities_pred: list[Vector3] = field(default_factory=list)
    tracker_positions_pred: list[Vector3] = field(default_factory=list)
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


class VisibilityPredictor:
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
    ) -> VisibilityPrediction:
        """两机状态须已对齐至同一 ENU 时刻；t=0 总用实测姿态，未来可用保持的指令姿态。"""
        vectors = (target_state.p, target_state.v, target_state.a, tracker_state.p, tracker_state.v)
        if not all(_finite_vector(vector, 3) for vector in vectors) or not _valid_attitude(tracker_attitude):
            return VisibilityPrediction(reason="invalid_state_or_attitude")
        use_acceleration = (self.config.translation_mode == "constant_acceleration"
                            and tracker_acceleration_reliable and _finite_vector(tracker_state.a, 3))
        acceleration = tracker_state.a if use_acceleration else (0.0, 0.0, 0.0)
        target_positions = _trajectory(target_state, self.times, target_state.a)
        tracker_positions = _trajectory(tracker_state, self.times, acceleration)
        if not all(_finite_vector(position, 3) for position in target_positions + tracker_positions):
            return VisibilityPrediction(reason="prediction_overflow")
        relative = [_vector3([target[axis] - tracker[axis] for axis in range(3)])
                    for target, tracker in zip(target_positions, tracker_positions)]
        distances = [math.hypot(*vector) for vector in relative]
        projections = {"constant_attitude": self._project(target_positions, tracker_positions,
                                                        tracker_attitude, tracker_attitude)}
        command_available = _valid_attitude(commanded_attitude)
        if command_available:
            projections["command_based_attitude"] = self._project(
                target_positions, tracker_positions, tracker_attitude, commanded_attitude)
        else:
            projections["command_based_attitude"] = projections["constant_attitude"]
        mode = self.config.attitude_mode
        if mode == "command_based_attitude" and not command_available:
            mode = "constant_attitude"
        primary = projections[mode]
        distance_cost = sum(
            self.config.distance_min_weight * max(0.0, self.config.distance_min - distance)**2
            + self.config.distance_max_weight * max(0.0, distance - self.config.distance_max)**2
            for distance in distances)
        return VisibilityPrediction(
            valid=True, prediction_times=list(self.times), target_positions_pred=target_positions,
            target_velocities_pred=[_vector3([target_state.v[axis] + target_state.a[axis] * elapsed
                                             for axis in range(3)]) for elapsed in self.times],
            tracker_positions_pred=tracker_positions, relative_positions_world=relative,
            relative_positions_camera=primary.relative_positions_camera,
            pixels_pred=primary.pixels_pred, normalized_pixels_pred=primary.normalized_pixels_pred,
            distances_pred=distances, current_distance=distances[0], min_distance=min(distances),
            max_distance=max(distances), predicted_distance_at_horizon=distances[-1],
            predicted_fov_exit=primary.predicted_fov_exit,
            predicted_fov_exit_time_sec=primary.predicted_fov_exit_time_sec,
            max_image_error=primary.max_image_error, fov_cost=primary.fov_cost,
            distance_cost=distance_cost,
            translation_mode_used="constant_acceleration" if use_acceleration else "constant_velocity",
            attitude_mode_used=mode, command_attitude_available=command_available,
            attitude_predictions=projections,
        )

    def _project(self, target_positions, tracker_positions, current_attitude, future_attitude) -> FovPrediction:
        camera = self.camera
        matrix = np.asarray(camera.intrinsics)
        optical_positions, pixels, normalized = [], [], []
        behind, safe, warning, outside, statuses = [], [], [], [], []
        errors, costs = [], []
        exit_time = None
        for index, (target, tracker) in enumerate(zip(target_positions, tracker_positions)):
            attitude = current_attitude if index == 0 else future_attitude
            optical = world_to_camera(target, tracker, attitude, camera.offset_body, camera.forward_sign)
            projection = project_oracle_pixel(target, tracker, attitude, camera.offset_body,
                                               matrix, camera.forward_sign)
            optical_positions.append(_vector3(optical))
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
                             sum(costs) if projectable else None)