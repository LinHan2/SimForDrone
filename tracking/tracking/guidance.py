"""V0 真值位置跟踪的参考生成器。

目标与观测机的几何关系发生在**共享 ENU** 系里，而 PX4 位置控制使用观测机自己的
**local ENU** 系。下面的转换只搬运共享系中的**相对位移**，两个原点因此永不混用。
"""

from __future__ import annotations

from dataclasses import dataclass, replace
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


def follow_offset(initial: Vector3, distance: float, elapsed: float, approach_speed: float = 0.5,
                  height_offset: float | None = None, height_speed: float = 0.05) -> Vector3:
    """沿初始水平视线收敛距离，并可缓慢调整目标相对高度。"""
    radius = math.hypot(initial[0], initial[1])
    if radius < 1e-6 or distance <= 0 or approach_speed <= 0 or height_speed <= 0:
        raise ValueError("跟随距离、接近速度和初始水平间距必须为正")
    current_radius = radius + max(-approach_speed * max(elapsed, 0.0),
                                  min(approach_speed * max(elapsed, 0.0), distance - radius))
    height = initial[2]
    if height_offset is not None:
        step = height_speed * max(elapsed, 0.0)
        height += max(-step, min(step, height_offset - height))
    return (initial[0] * current_radius / radius, initial[1] * current_radius / radius, height)


def band_offset(current_relative: Vector3, min_distance: float, max_distance: float,
                height_offset: float | None = None) -> Vector3:
    """把期望站位放到 5~10 m 这类软约束带内：带内沿用当前水平相对位置，不做径向牵引。

    只有水平间距越过下界/上界时才把偏移缩放回该边界，因此带内不会产生径向位置误差，
    避免固定距离约束在目标机动时持续拉扯出超调。高度差可单独指定。
    """

    if not 0.0 < min_distance <= max_distance:
        raise ValueError("软约束带需要 0 < min_distance <= max_distance")
    radius = math.hypot(current_relative[0], current_relative[1])
    if radius < 1e-6:
        raise ValueError("水平相对位置不能为零向量")
    target_radius = min(max(radius, min_distance), max_distance)
    scale = target_radius / radius
    height = current_relative[2] if height_offset is None else height_offset
    return (current_relative[0] * scale, current_relative[1] * scale, height)


def pitch_compensated_offset(offset: Vector3, pitch: float, max_height_gain: float) -> Vector3:
    """按观测机俯仰调整期望高度差，使固定前视相机在加速时仍指向目标。

    ``offset`` 是观测机相对目标的偏移，相机光轴固连机体 +X。低头 ``θ>0`` 时光轴下压，
    需把观测机抬高 ``d_h·tanθ`` 才能让视线回到光轴；抬头则相反。实测翻转该符号会使
    出画帧数从 279 升到 382，故保持 ``+d_h·tanθ``。
    限幅避免大倾角下高度参考跳变；本函数只改几何参考，不消费像素，因此不保证目标居中。
    """

    if max_height_gain < 0.0 or not math.isfinite(max_height_gain):
        raise ValueError("补偿高度上限必须是非负有限值")
    if not math.isfinite(pitch) or abs(pitch) >= math.pi / 2.0:
        raise ValueError("俯仰角必须有限且小于 90°")
    horizontal = math.hypot(offset[0], offset[1])
    gain = horizontal * math.tan(pitch)
    gain = max(-max_height_gain, min(max_height_gain, gain))
    return (offset[0], offset[1], offset[2] + gain)


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


def visibility_intervention_weight(max_image_error: float | None, safe_ratio: float,
                                  warning_ratio: float) -> float:
    """把预测时域内的归一化图像偏差映射成 [0,1] 干预权重。

    ``max_image_error`` 即 predictor 的 ``max(|u_n|,|v_n|)`` 时域最大值；它为 ``None``
    表示预测窗内存在不可投影帧（相机后方/近像平面），属最危险情形，取 1。
    """

    if not 0 <= safe_ratio < warning_ratio:
        raise ValueError("需要 0 <= safe_ratio < warning_ratio")
    if max_image_error is None:
        return 1.0
    if not math.isfinite(max_image_error):
        raise ValueError("图像偏差必须是有限值或 None")
    return max(0.0, min(1.0, (max_image_error - safe_ratio) / (warning_ratio - safe_ratio)))


@dataclass(frozen=True)
class VisibilitySafetyFilter:
    """按可见性风险权重连续向外站位换取角度余量；权重为 0 时原样透传 nominal 参考。

    偏移约定与 :func:`band_offset` 一致，都是**观测机减目标**（tracker − target）：
    共享系期望位置是 ``p_target + offset``，再只把相对位移搬到观测机 local 系。
    竖直方向一律沿用 nominal 参考的高度意图（含俯仰补偿），本滤波器只动水平分量，
    因此不会因为可见性保护而改变相对高度。
    """

    min_distance: float = 5.0
    max_distance: float = 10.0
    retreat_speed: float = 1.0

    def __post_init__(self) -> None:
        if not 0 < self.min_distance <= self.max_distance:
            raise ValueError("可见性安全带需要 0 < min_distance <= max_distance")
        if not math.isfinite(self.retreat_speed) or self.retreat_speed < 0:
            raise ValueError("后退速度必须是非负有限值")

    def apply(
        self,
        reference: DesiredState,
        target: TargetState,
        observer: ObserverState,
        *,
        intervention_weight: float = 0.0,
        dt: float = 0.0,
    ) -> DesiredState:
        if not 0.0 <= intervention_weight <= 1.0:
            raise ValueError("干预权重必须在 [0, 1]")
        relative = _sub(observer.shared_p, target.p)
        radius = math.hypot(relative[0], relative[1])
        if radius <= 1e-9:
            return reference
        # 角度余量随距离增大，故风险越高越向带外沿后退；按 dt 限速避免参考跳变。
        retreat = intervention_weight * self.retreat_speed * max(dt, 0.0)
        target_radius = min(self.max_distance,
                            min(max(radius, self.min_distance), self.max_distance) + retreat)
        if target_radius == radius:
            return reference
        nominal_shared = _add(_sub(reference.p, observer.local_p), observer.shared_p)
        scale = target_radius / radius
        safe_shared = (target.p[0] + relative[0] * scale, target.p[1] + relative[1] * scale,
                       nominal_shared[2])
        return replace(reference, p=_add(observer.local_p, _sub(safe_shared, observer.shared_p)))


def predict_commanded_tilt(desired_a: Vector3, yaw: float, gravity: float,
                           min_tilt_ratio: float = 0.1) -> tuple[float, float]:
    """复现 ``LinearControl`` 的几何倾角反解 :math:`a_o\\to b_{3,d}\\to R_{o,d}`。

    ``desired_a`` 是含重力的世界系期望加速度（即 PD 输出，非控制器限幅后的值）。
    这是把加速度候选变成未来姿态、进而变成未来相机位姿的唯一入口——roll/pitch
    不是独立变量，而是 ``desired_a`` 和 ``yaw`` 的确定性函数。不模拟 jerk/倾角限幅器
    的历史状态，只做单步前瞻，候选排序足够、不是精确复现。
    """

    if not math.isfinite(gravity) or gravity <= 0.0:
        raise ValueError("重力加速度必须为正")
    if not 0.0 < min_tilt_ratio < 1.0:
        raise ValueError("倾角反解除数下限比例必须在 (0, 1)")
    magnitude = math.sqrt(sum(value * value for value in desired_a))
    if magnitude <= 0.0 or not math.isfinite(magnitude):
        return 0.0, 0.0
    sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)
    sin_roll = (desired_a[0] * sin_yaw - desired_a[1] * cos_yaw) / magnitude
    roll = math.asin(max(-1.0, min(1.0, sin_roll)))
    tilt_divisor = desired_a[2] if desired_a[2] >= gravity * min_tilt_ratio else gravity * min_tilt_ratio
    pitch = math.atan2(desired_a[0] * cos_yaw + desired_a[1] * sin_yaw, tilt_divisor)
    return roll, pitch


def acceleration_candidates(nominal_a: Vector3, line_of_sight: Vector3,
                           vertical_step: float, los_step: float,
                           retreat_scale: float = 0.8) -> list[Vector3]:
    """围绕 nominal 加速度生成一组候选动作：缩放、竖直扰动、沿视线方向扰动。

    候选不直接是几何量（后退距离/高度），而是控制变量 ``a_o`` 本身——是否后退、
    是否降高只是某个候选恰好在预测后代价最低时呈现出的*结果*，不是预先写好的规则。
    ``line_of_sight`` 须是观测机到目标的水平单位向量；沿它扰动对应"多退/少退一步"，
    竖直扰动对应"多爬/多降一步"。``nominal_a`` 本身也作为候选之一（不改变时的基线）。
    """

    if vertical_step < 0.0 or not math.isfinite(vertical_step):
        raise ValueError("竖直扰动步长必须是非负有限值")
    if los_step < 0.0 or not math.isfinite(los_step):
        raise ValueError("视线方向扰动步长必须是非负有限值")
    if not 0.0 < retreat_scale < 1.0:
        raise ValueError("缩放候选比例必须在 (0, 1)")
    los_norm = math.hypot(line_of_sight[0], line_of_sight[1])
    los_unit = ((0.0, 0.0) if los_norm <= 1e-9
               else (line_of_sight[0] / los_norm, line_of_sight[1] / los_norm))
    candidates = [nominal_a, tuple(retreat_scale * value for value in nominal_a)]
    if vertical_step > 0.0:
        candidates.append((nominal_a[0], nominal_a[1], nominal_a[2] + vertical_step))
        candidates.append((nominal_a[0], nominal_a[1], nominal_a[2] - vertical_step))
    if los_step > 0.0 and los_norm > 1e-9:
        candidates.append((nominal_a[0] + los_unit[0] * los_step, nominal_a[1] + los_unit[1] * los_step,
                           nominal_a[2]))
        candidates.append((nominal_a[0] - los_unit[0] * los_step, nominal_a[1] - los_unit[1] * los_step,
                           nominal_a[2]))
    return candidates


def fov_cost(normalized_pixels: list[tuple[float, float] | None], safe_ratio: float) -> float:
    """由时域内逐帧归一化像素 ``(q_x,q_y)`` 直接算 FOV 代价，不先坍缩成单一标量。

    ``safe_ratio`` 内零代价（允许目标不居中，只要留在安全区），之外二次增长；
    不可投影帧（相机后方/近像平面）记一个固定大代价，使其总是劣于任何能投影的候选。
    """

    if not 0.0 < safe_ratio <= 1.0:
        raise ValueError("安全比例必须在 (0, 1]")
    cost = 0.0
    for pixel in normalized_pixels:
        if pixel is None:
            cost += 4.0 * (1.0 - safe_ratio) ** 2
            continue
        cost += max(0.0, abs(pixel[0]) - safe_ratio) ** 2 + max(0.0, abs(pixel[1]) - safe_ratio) ** 2
    return cost


def combined_visibility_cost(
    *,
    fov_cost: float | None = None,
    fov_cost_value: float | None = None,
    distance_cost: float = 0.0,
    pixel_occupancy_cost: float = 0.0,
    tracking_cost: float = 0.0,
    smooth_cost: float = 0.0,
    fov_weight: float = 1.0,
    distance_weight: float = 1.0,
    tracking_weight: float = 1.0,
    smoothness_weight: float = 0.0,
    pixel_occupancy_weight: float = 0.0,
) -> float:
    """把未来 FOV、目标像素占比、距离带、跟踪误差和平滑度汇总为一个成本。

    这对应于目标函数
        J = w_f J_FOV + w_p J_pixel + w_d J_distance + w_t J_tracking + w_u J_smooth
    其中 ：
    - J_FOV 来自未来像素是否离开安全中心区；
    - J_pixel 惩罚目标投影面积占比偏离成像质量范围；
    - J_distance 由 predictor 的距离软约束给出；
    - J_tracking 允许一个宽松的跟踪区间，不要求固定单点；
    - J_smooth 惩罚候选动作偏离 nominal，避免振荡。
    """

    fov_value = float(fov_cost if fov_cost is not None else fov_cost_value if fov_cost_value is not None else 0.0)
    for weight in (fov_weight, distance_weight, tracking_weight, smoothness_weight,
                   pixel_occupancy_weight):
        if not math.isfinite(weight) or weight < 0.0:
            raise ValueError("各权重必须为有限的非负值")
    return (
        fov_weight * fov_value
        + pixel_occupancy_weight * float(pixel_occupancy_cost)
        + distance_weight * float(distance_cost)
        + tracking_weight * float(tracking_cost)
        + smoothness_weight * float(smooth_cost)
    )


def relative_tracking_cost(
    relative_position: Vector3,
    relative_velocity: Vector3,
    min_distance: float = 5.0,
    max_distance: float = 10.0,
    tracking_weight: float = 1.0,
) -> float:
    """相对位姿在允许安全带内不罚分，只在越界时按平方代价回落。

    这不仅仅是距离安全：当目标靠近观测机或远离观测机过多时，未来可见性和跟踪控制都
    会变差，所以用一个允许带 ``[min_distance, max_distance]`` 作为宽松目标区，并且把
    相对速度误差加进“追踪”项，避免看到可见性风险就只知道往后退/下降。
    """

    if not 0.0 < min_distance <= max_distance:
        raise ValueError("跟踪带必须满足 0 < min_distance <= max_distance")
    if not math.isfinite(tracking_weight) or tracking_weight < 0.0:
        raise ValueError("跟踪权重必须为有限的非负值")
    radius = math.hypot(relative_position[0], relative_position[1])
    cost = 0.0
    if radius < min_distance:
        cost += (min_distance - radius) ** 2
    elif radius > max_distance:
        cost += (radius - max_distance) ** 2
    cost += tracking_weight * sum(component * component for component in relative_velocity)
    return cost


def select_visibility_optimized_acceleration(
    candidates: list[Vector3], *, predict_cost,
    smoothness_weight: float = 0.0, nominal_a: Vector3 | None = None,
) -> Vector3:
    """在候选加速度集合里选总代价最低者；``predict_cost(a) -> float`` 由调用方注入。

    ``predict_cost`` 通常是「反解姿态 -> 跑可见性预测器 -> J_FOV+λ_d·J_distance」，
    保持与 predictor 的耦合在调用方，这里只做纯粹的候选排序，可独立单测。
    平滑正则 ``J_smooth=||a-a_nominal||^2`` 可选：权重为 0 时退化为纯视野/距离代价。
    """

    if not candidates:
        raise ValueError("候选集不能为空")
    if smoothness_weight < 0.0 or not math.isfinite(smoothness_weight):
        raise ValueError("平滑正则权重必须是非负有限值")
    if smoothness_weight > 0.0 and nominal_a is None:
        raise ValueError("启用平滑正则时必须提供 nominal_a")
    anchor = nominal_a if nominal_a is not None else (0.0, 0.0, 0.0)
    best_a, best_cost = candidates[0], math.inf
    for candidate in candidates:
        cost = predict_cost(candidate)
        if smoothness_weight > 0.0:
            cost += smoothness_weight * sum((candidate[axis] - anchor[axis]) ** 2 for axis in range(3))
        if cost < best_cost:
            best_cost, best_a = cost, candidate
    return best_a


def predicted_target(target: TargetState, horizon: float) -> TargetState:
    """按 ``p + v*tau + 0.5*a*tau^2`` 外推目标位置，抵消控制响应滞后。

    只前移位置，速度/加速度参考保持不变：它们已是同一时刻的前馈量。``tau`` 过大会把
    滞后换成转弯处的超前冲出，需按实测 RMSE 与最大像素偏移选取。
    """

    if horizon < 0.0 or not math.isfinite(horizon):
        raise ValueError("预测时域必须是非负有限值")
    if horizon == 0.0:
        return target
    return TargetState(
        p=tuple(
            target.p[i] + target.v[i] * horizon + 0.5 * target.a[i] * horizon * horizon
            for i in range(3)
        ),
        v=target.v,
        a=target.a,
    )


def apply_feedforward_level(target: TargetState, level: str) -> TargetState:
    """按 A/B/C 基线裁剪前馈项，用于隔离速度与加速度前馈各自的贡献。"""

    if level == "position":
        return TargetState(p=target.p, v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0))
    if level == "velocity":
        return TargetState(p=target.p, v=target.v, a=(0.0, 0.0, 0.0))
    if level == "acceleration":
        return target
    raise ValueError(f"未知前馈层级：{level}")


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
    max_position_error: float | None = None
    max_correction_speed: float | None = None
    position_to_speed_gain: float = 1.0

    def generate(self, target: TargetState, observer: ObserverState) -> DesiredState:
        """按当前目标状态给出观测机 local 系下的期望状态。"""
        import math

        # 共享 ENU: p_ref^W = p_target^W + offset^W, e^W = p_ref^W - p_tracker^W。
        desired_shared = _add(target.p, self.relative_offset)
        shared_error = _sub(desired_shared, observer.shared_p)
        if self.max_position_error is not None:
            error_norm = math.sqrt(sum(value * value for value in shared_error))
            if error_norm > self.max_position_error:
                scale = self.max_position_error / error_norm
                shared_error = (shared_error[0] * scale, shared_error[1] * scale,
                                shared_error[2] * scale)
        if self.max_correction_speed is not None:
            horizontal_error = math.hypot(shared_error[0], shared_error[1])
            max_error = self.max_correction_speed / self.position_to_speed_gain
            if horizontal_error > max_error:
                shared_error = (shared_error[0] * max_error / horizontal_error,
                                shared_error[1] * max_error / horizontal_error, shared_error[2])
        # local ENU: p_ref^L = p_tracker^L + clip(e^W)；速度修正限幅相当于
        # ||e_xy|| <= max_correction_speed / position_to_speed_gain，垂直误差不变。
        # 只把共享系的**相对位移**叠加到 tracker 的 local 位置：
        # 两机 PX4 EKF 的 local 原点各自建立、互不重合，直接使用共享系绝对坐标
        # 会让位置设定点整体偏移（实测两机 local 原点可相差数十厘米以上）。
        desired_local = _add(observer.local_p, shared_error)
        
        # 几何偏航 psi_ref = atan2(p_target,n - p_tracker,n, p_target,e - p_tracker,e)。
        delta_e = target.p[0] - observer.shared_p[0]
        delta_n = target.p[1] - observer.shared_p[1]
        desired_yaw = math.atan2(delta_n, delta_e)
        
        # v_ref = v_target, a_ref = a_target；本函数不消费图像像素，不能保证目标入镜。
        return DesiredState(
            p=desired_local,
            v=target.v,
            a=target.a,
            yaw=desired_yaw,
        )

