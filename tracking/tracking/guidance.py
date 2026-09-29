"""V0 真值位置跟踪的参考生成器。

目标与观测机的几何关系发生在**共享 ENU** 系里，而 PX4 位置控制使用观测机自己的
**local ENU** 系。下面的转换只搬运共享系中的**相对位移**，两个原点因此永不混用。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TypeAlias

Vector3: TypeAlias = tuple[float, float, float]


def slew_yaw(current: float, target: float, max_step: float) -> float:
    delta = math.atan2(math.sin(target - current), math.cos(target - current))
    return current + max(-max_step, min(max_step, delta))


def _add(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _sub(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def follow_offset(initial: Vector3, distance: float, elapsed: float, approach_speed: float = 0.5) -> Vector3:
    """沿初始水平视线逐渐收敛到指定距离，保留初始高度差。"""
    radius = math.hypot(initial[0], initial[1])
    if radius < 1e-6 or distance <= 0 or approach_speed <= 0:
        raise ValueError("跟随距离、接近速度和初始水平间距必须为正")
    current_radius = radius + max(-approach_speed * max(elapsed, 0.0),
                                  min(approach_speed * max(elapsed, 0.0), distance - radius))
    return (initial[0] * current_radius / radius, initial[1] * current_radius / radius, initial[2])


@dataclass(frozen=True)
class DesiredState:
    """控制器的期望参考，位于观测机的 local ENU 系。"""

    p: Vector3 = (0.0, 0.0, 0.0)
    v: Vector3 = (0.0, 0.0, 0.0)
    a: Vector3 = (0.0, 0.0, 0.0)
    j: Vector3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class TargetState:
    """目标运动学状态，位于共享世界 ENU 系。"""

    p: Vector3
    v: Vector3 = (0.0, 0.0, 0.0)
    a: Vector3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ObserverState:
    """观测机位置，同时给出共享系与它自己的 PX4 local ENU 系坐标。"""

    shared_p: Vector3
    local_p: Vector3


@dataclass(frozen=True)
class PositionTrackerV0:
    """由目标状态生成固定世界系偏移的参考（V0 最小可用形态）。"""

    relative_offset: Vector3 = (-3.0, 0.0, 1.0)
    yaw: float = 0.0

    def generate(self, target: TargetState, observer: ObserverState) -> DesiredState:
        """按当前目标状态给出观测机 local 系下的期望状态。"""
        import math

        # 先在共享系里算出期望站位与相对误差。
        desired_shared = _add(target.p, self.relative_offset)
        shared_error = _sub(desired_shared, observer.shared_p)
        # 只把共享系的**相对位移**叠加到 tracker 的 local 位置：
        # 两机 PX4 EKF 的 local 原点各自建立、互不重合，直接使用共享系绝对坐标
        # 会让位置设定点整体偏移（实测两机 local 原点可相差数十厘米以上）。
        desired_local = _add(observer.local_p, shared_error)
        
        # 动态偏航：指向目标在共享系中的相对方向
        delta_e = target.p[0] - observer.shared_p[0]
        delta_n = target.p[1] - observer.shared_p[1]
        desired_yaw = math.atan2(delta_n, delta_e)
        
        return DesiredState(
            p=desired_local,
            v=target.v,
            a=target.a,
            yaw=desired_yaw,
        )


class ImageBasedYawController:
    """基于图像反馈的偏航控制器，确保目标在视野中央。
    
    参考 Yang et al. 2025 "High-Speed Interception Multicopter Control by IBVS"
    使用 Barrier Lyapunov Function 保证目标在 FOV 内，增益随接近边界自适应增大。
    """
    
    def __init__(
        self,
        image_width: int = 640,
        image_height: int = 480,
        fov_horizontal_deg: float = 60.0,
        k_barrier: float = 0.8,  # 障碍边界（归一化）
        deadzone_pixels: float = 20.0,
        max_yaw_rate: float = 0.5,
    ):
        self.image_center_x = image_width / 2.0
        self.image_center_y = image_height / 2.0
        self.image_half_width = image_width / 2.0
        self.k_barrier = k_barrier  # z1 不能超过此值
        self.deadzone_pixels = deadzone_pixels
        self.max_yaw_rate = max_yaw_rate
        self.fov_horizontal_deg = fov_horizontal_deg
    
    def compute_yaw_rate(
        self,
        target_pixel_x: float,
        target_pixel_y: float,
    ) -> float:
        """根据目标在图像中的像素位置计算偏航角速率。
        
        使用 Barrier Lyapunov 增益：gain = z1 / (k_b^2 - z1^2)
        其中 z1 是归一化像素偏差，越接近边界增益越大。
        
        Args:
            target_pixel_x: 目标在图像中的 x 坐标（像素）
            target_pixel_y: 目标在图像中的 y 坐标（像素）
        
        Returns:
            偏航角速率 (rad/s)
        """
        # 计算水平偏差（像素）并归一化到 [-1, 1]
        error_x_px = target_pixel_x - self.image_center_x
        z1 = error_x_px / self.image_half_width  # 归一化偏差
        
        # 死区处理
        if abs(error_x_px) < self.deadzone_pixels:
            return 0.0
        
        # 边界检查：如果 |z1| >= k_barrier，强制限幅
        z1 = max(-self.k_barrier * 0.99, min(self.k_barrier * 0.99, z1))
        
        # Barrier Lyapunov 增益：接近边界时自动增大
        kb2 = self.k_barrier ** 2
        barrier_gain = z1 / (kb2 - z1**2)
        
        # 偏航角速率：负号因为目标在右→需右转（负角速率）
        yaw_rate = -barrier_gain
        
        # 限幅
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        
        return yaw_rate


class PositionTrackerV1(PositionTrackerV0):
    """V1: 带加速度前馈 + Barrier Lyapunov 图像偏航的位置跟踪器。
    
    改进点（基于 Yang 2025 + EKF 加速度估计）：
    1. 加速度前馈：利用 EKF 估计的 target.a，补偿目标机动
    2. Barrier Lyapunov 偏航：自适应增益，保证目标在 FOV 内
    3. 分层控制：位置外环 + 偏航独立闭环
    """
    
    def __init__(
        self,
        relative_offset: Vector3,
        yaw: float = 0.0,
        image_width: int = 640,
        image_height: int = 480,
        k_yaw: float = 0.002,
        fov_horizontal_deg: float = 60.0,
        k_barrier: float = 0.8,
        max_yaw_rate: float = 0.5,
        deadzone_pixels: float = 20.0,
        feedforward_gain: float = 0.8,
    ):
        super().__init__(relative_offset, yaw)
        # 保持向后兼容：k_yaw 参数已废弃，使用 Barrier Lyapunov
        self.yaw_controller = ImageBasedYawController(
            image_width=image_width,
            image_height=image_height,
            fov_horizontal_deg=fov_horizontal_deg,
            k_barrier=k_barrier,
            max_yaw_rate=max_yaw_rate,
            deadzone_pixels=deadzone_pixels,
        )
        self.feedforward_gain = float(feedforward_gain)
        self._current_yaw = yaw
    
    def generate_with_image_feedback(
        self,
        target: TargetState,
        observer: ObserverState,
        target_pixel_x: float,
        target_pixel_y: float,
        dt: float,
    ) -> DesiredState:
        """根据目标状态、EKF 加速度估计和图像反馈生成期望状态。
        
        控制律（改进自 Yang 2025）：
        1. 位置外环：期望位置 = target.p + offset
        2. 速度前馈：期望速度 = target.v（跟随目标）
        3. 加速度前馈：期望加速度 = feedforward_gain * target.a（补偿机动）
        4. 偏航闭环：几何前馈 + Barrier Lyapunov 图像反馈
        
        Args:
            target: 目标状态（含 EKF 估计的加速度 a）
            observer: 观测机状态
            target_pixel_x: 目标在图像中的 x 坐标
            target_pixel_y: 目标在图像中的 y 坐标
            dt: 时间步长
        
        Returns:
            期望状态，包含加速度前馈和图像反馈修正的偏航
        """
        import math
        
        # 1. 位置控制（继承自 V0）
        desired_shared = _add(target.p, self.relative_offset)
        shared_error = _sub(desired_shared, observer.shared_p)
        desired_local = _add(observer.local_p, shared_error)
        
        # 2. 几何偏航作为前馈
        delta_e = target.p[0] - observer.shared_p[0]
        delta_n = target.p[1] - observer.shared_p[1]
        geometric_yaw = math.atan2(delta_n, delta_e)
        
        # 3. Barrier Lyapunov 图像偏航修正
        yaw_rate_correction = self.yaw_controller.compute_yaw_rate(
            target_pixel_x, target_pixel_y
        )
        
        # 4. 积分偏航角（前馈 + 反馈积分）
        self._current_yaw = geometric_yaw + yaw_rate_correction * dt
        
        # 5. 加速度前馈（关键改进！利用 EKF 估计的 target.a）
        # Yang 2025 假设加速度未知，你的 EKF 直接提供 a_target
        desired_accel = (
            target.a[0] * self.feedforward_gain,
            target.a[1] * self.feedforward_gain,
            target.a[2] * self.feedforward_gain,
        )
        
        return DesiredState(
            p=desired_local,
            v=target.v,
            a=desired_accel,
            yaw=self._current_yaw,
            yaw_rate=yaw_rate_correction,
        )
