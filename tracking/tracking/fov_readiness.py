"""tracker 相机视野初始化门的纯几何判定。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from tracking.shadow_observation import (
    depth_roi_median,
    project_oracle_pixel,
    range_from_optical_depth,
    pixel_ray_body,
)

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class FovReadiness:
    """单帧 FOV 门的结果；Oracle 只用于仿真初始化，不是视觉检测。"""

    ready: bool
    reason: str
    pixel_u: float | None = None
    pixel_v: float | None = None
    range_m: float | None = None
    oracle_range_m: float | None = None


def evaluate_oracle_fov_depth(
    *,
    target_world: Vector3,
    tracker_world: Vector3,
    tracker_attitude: Quaternion,
    camera_offset_body: Vector3,
    camera_matrix: np.ndarray,
    depth: np.ndarray,
    roi_radius_px: int,
    max_range_m: float,
    max_oracle_range_error_m: float,
    camera_forward_sign: float = 1.0,
) -> FovReadiness:
    """判定目标是否在相机内、深度是否可用于后续 shadow EKF。"""

    projection = project_oracle_pixel(
        target_world,
        tracker_world,
        tracker_attitude,
        camera_offset_body,
        camera_matrix,
        camera_forward_sign,
    )
    if projection is None:
        return FovReadiness(False, "target_behind_camera")
    pixel_u, pixel_v, oracle_range = projection
    if depth.ndim != 2 or not 0.0 <= pixel_u < depth.shape[1] or not 0.0 <= pixel_v < depth.shape[0]:
        return FovReadiness(False, "target_outside_image", pixel_u, pixel_v, oracle_range_m=oracle_range)
    optical_depth = depth_roi_median(depth, pixel_u, pixel_v, roi_radius_px)
    if optical_depth is None:
        return FovReadiness(False, "invalid_depth_roi", pixel_u, pixel_v, oracle_range_m=oracle_range)
    ray_body = pixel_ray_body(pixel_u, pixel_v, camera_matrix, camera_forward_sign)
    range_m = range_from_optical_depth(optical_depth, ray_body)
    if range_m is None or range_m > max_range_m:
        return FovReadiness(False, "range_out_of_bounds", pixel_u, pixel_v, range_m, oracle_range)
    if abs(range_m - oracle_range) > max_oracle_range_error_m:
        return FovReadiness(False, "depth_oracle_disagreement", pixel_u, pixel_v, range_m, oracle_range)
    return FovReadiness(True, "ready", pixel_u, pixel_v, range_m, oracle_range)