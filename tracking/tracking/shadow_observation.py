"""RGB-D 相对 EKF 影子验证的纯几何工具。"""

from __future__ import annotations

import math

import numpy as np

Vector3 = tuple[float, float, float]


def observation_time_skew(
    depth_received_at: float,
    tracker_pose_received_at: float,
    target_pose_received_at: float,
    target_attitude_received_at: float,
) -> float:
    """最大单调时钟到达时差；跨时钟域的 ROS header 不参与配对。"""

    timestamps = (
        depth_received_at,
        tracker_pose_received_at,
        target_pose_received_at,
        target_attitude_received_at,
    )
    if not all(math.isfinite(timestamp) for timestamp in timestamps):
        raise ValueError("量测时间戳必须有限")
    return max(timestamps) - min(timestamps)


def track_depth_roi(
    depth: np.ndarray,
    previous_u: float,
    previous_v: float,
    previous_depth: float,
    radius_px: int,
    depth_tolerance_m: float,
) -> tuple[float, float, float] | None:
    """在上一目标像素附近跟踪相近深度；不能重捕获丢失的目标。"""

    if depth.ndim != 2 or radius_px < 1 or depth_tolerance_m <= 0.0:
        raise ValueError("深度图、搜索半径或深度阈值无效")
    if not all(math.isfinite(value) for value in (previous_u, previous_v, previous_depth)) or previous_depth <= 0:
        return None
    left = max(0, int(round(previous_u)) - radius_px)
    right = min(depth.shape[1], int(round(previous_u)) + radius_px + 1)
    top = max(0, int(round(previous_v)) - radius_px)
    bottom = min(depth.shape[0], int(round(previous_v)) + radius_px + 1)
    if left >= right or top >= bottom:
        return None
    patch = np.asarray(depth[top:bottom, left:right], dtype=np.float64)
    rows, cols = np.indices(patch.shape)
    distances = (cols + left - previous_u) ** 2 + (rows + top - previous_v) ** 2
    valid = (
        np.isfinite(patch) & (patch > 0) & (np.abs(patch - previous_depth) <= depth_tolerance_m)
        & (distances <= radius_px ** 2)
    )
    if np.count_nonzero(valid) < 5:
        return None
    return (
        float(np.median((cols + left)[valid])),
        float(np.median((rows + top)[valid])),
        float(np.median(patch[valid])),
    )


def depth_roi_median(depth: np.ndarray, center_u: float, center_v: float, radius_px: int) -> float | None:
    """返回正方形 ROI 的有效深度中位数；无有效像素时拒绝该帧。"""

    if depth.ndim != 2 or radius_px < 0:
        raise ValueError("depth 必须是二维数组，radius_px 不得为负")
    center_x = int(round(center_u))
    center_y = int(round(center_v))
    left = max(0, center_x - radius_px)
    right = min(depth.shape[1], center_x + radius_px + 1)
    top = max(0, center_y - radius_px)
    bottom = min(depth.shape[0], center_y + radius_px + 1)
    if left >= right or top >= bottom:
        return None
    values = np.asarray(depth[top:bottom, left:right], dtype=np.float64)
    values = values[np.isfinite(values) & (values > 0.0)]
    return None if values.size == 0 else float(np.median(values))


def pixel_ray_body(
    u: float,
    v: float,
    camera_matrix: np.ndarray,
    camera_forward_sign: float = 1.0,
) -> np.ndarray:
    """将 ROS 光学像素光线变换为 FLU 机体系单位向量。

    项目相机前向与机体 +X 对齐；ROS 光学系为右、下、前，因此
    ``camera_forward_sign=1`` 时映射为 ``(z, -x, -y)_FLU``；取 ``-1``
    则相机绕机体 z 轴翻转 180 度。
    """

    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("camera_matrix 必须是有限的 3x3 矩阵")
    fx, fy = matrix[0, 0], matrix[1, 1]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("相机焦距必须为正")
    if camera_forward_sign not in (-1.0, 1.0):
        raise ValueError("camera_forward_sign 必须为 -1 或 1")
    ray_optical = np.array(((u - matrix[0, 2]) / fx, (v - matrix[1, 2]) / fy, 1.0))
    ray_body = np.array((
        camera_forward_sign * ray_optical[2],
        -camera_forward_sign * ray_optical[0],
        -ray_optical[1],
    ))
    return ray_body / np.linalg.norm(ray_body)


def range_from_optical_depth(optical_depth: float, ray_body: np.ndarray | Vector3) -> float | None:
    """由沿光轴的深度恢复相机到目标的欧氏距离；不可用时返回 ``None``。

    深度是沿相机光轴测得的，而 ``ray_body[0]`` 是机体系前向分量，其**符号编码了
    相机安装方向**（朝前为 ``+1``，绕机体 z 轴翻转后为 ``-1``），大小才是该像素
    相对光轴的余弦。这里必须除以该分量的绝对值：除以带符号的值会把后向安装的
    相机整段算成负距离，表现为所有帧都被判为超出量程，而图像本身完全正常。
    """

    if not math.isfinite(optical_depth) or optical_depth <= 0.0:
        return None
    vector = np.asarray(ray_body, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("ray_body 必须是有限的机体系三元向量")
    forward_component = abs(float(vector[0]))
    if forward_component <= 1e-9:
        return None
    return float(optical_depth) / forward_component


def relative_position_from_depth(
    ray_body: np.ndarray,
    range_m: float,
    camera_offset_body: Vector3,
    tracker_attitude: tuple[float, float, float, float],
) -> np.ndarray:
    """相机表面深度换算为以观察机机体为原点的 ENU 相对位置。"""

    if not math.isfinite(range_m) or range_m <= 0.0:
        raise ValueError("深度距离必须为正且有限")
    return rotate_vector(tracker_attitude, tuple(np.asarray(camera_offset_body) + ray_body * range_m))


def detected_relative_position_body(
    depth: np.ndarray, pixel_u: float, pixel_v: float, camera_matrix: np.ndarray,
    camera_offset_body: Vector3, roi_radius_px: int = 3,
    max_range_m: float = 12.0, camera_forward_sign: float = 1.0,
) -> np.ndarray | None:
    """从已检测到的目标像素及对齐深度得到机体系位置；无效帧不生成量测。"""

    if (depth.ndim != 2 or not np.isfinite(pixel_u) or not np.isfinite(pixel_v)
            or not 0 <= pixel_u < depth.shape[1] or not 0 <= pixel_v < depth.shape[0]):
        return None
    optical_depth = depth_roi_median(depth, pixel_u, pixel_v, roi_radius_px)
    if optical_depth is None:
        return None
    ray = pixel_ray_body(pixel_u, pixel_v, camera_matrix, camera_forward_sign)
    range_m = range_from_optical_depth(optical_depth, ray)
    if range_m is None or range_m > max_range_m:
        return None
    return np.asarray(camera_offset_body) + ray * range_m


def rotate_vector(attitude: tuple[float, float, float, float], vector: Vector3) -> np.ndarray:
    """按 ENU/FLU 四元数 ``(x, y, z, w)`` 把机体系向量转到世界系。"""

    x, y, z, w = (float(value) for value in attitude)
    norm_squared = x * x + y * y + z * z + w * w
    if norm_squared <= 1e-12 or not math.isfinite(norm_squared):
        raise ValueError("姿态四元数不得为零或非有限")
    scale = norm_squared**-0.5
    x, y, z, w = x * scale, y * scale, z * scale, w * scale
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.array((
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ))


def world_to_camera(
    target_world: Vector3,
    tracker_world: Vector3,
    tracker_attitude: tuple[float, float, float, float],
    camera_offset_body: Vector3,
    camera_forward_sign: float = 1.0,
) -> np.ndarray:
    """共享 ENU 点转为相机光学系坐标，包含 FLU 安装平移。"""

    world_from_body = _rotation_matrix(tracker_attitude)
    camera_world = np.asarray(tracker_world) + world_from_body @ np.asarray(camera_offset_body)
    target_body = world_from_body.T @ (np.asarray(target_world) - camera_world)
    if camera_forward_sign not in (-1.0, 1.0):
        raise ValueError("camera_forward_sign 必须为 -1 或 1")
    return np.array((
        -camera_forward_sign * target_body[1],
        -target_body[2],
        camera_forward_sign * target_body[0],
    ))


def project_oracle_pixel(
    target_world: Vector3,
    tracker_world: Vector3,
    tracker_attitude: tuple[float, float, float, float],
    camera_offset_body: Vector3,
    camera_matrix: np.ndarray,
    camera_forward_sign: float = 1.0,
) -> tuple[float, float, float] | None:
    """用仿真真值仅为 Oracle ROI 投影目标；返回 ``(u, v, camera_range)``。"""

    target_optical = world_to_camera(
        target_world, tracker_world, tracker_attitude, camera_offset_body, camera_forward_sign
    )
    if target_optical[2] <= 1e-6:
        return None
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    u = matrix[0, 0] * target_optical[0] / target_optical[2] + matrix[0, 2]
    v = matrix[1, 1] * target_optical[1] / target_optical[2] + matrix[1, 2]
    return float(u), float(v), float(np.linalg.norm(target_optical))


def _rotation_matrix(attitude: tuple[float, float, float, float]) -> np.ndarray:
    columns = [rotate_vector(attitude, axis) for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))]
    return np.column_stack(columns)
