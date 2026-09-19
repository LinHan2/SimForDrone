"""平滑航点轨迹：五次多项式（minimum-jerk）参考与速度/加速度前馈。

**为什么需要它**：直接把航点当作阶跃位置指令送给位置环会出问题。控制器用
``Kp≈6``，3 m 误差意味着期望加速度 ``18 m/s²``；而 ``max_angle: 25°`` 把水平
加速度限制在 ``g·tan(25°) ≈ 4.6 m/s²``。环路饱和后表现为大幅超调与持续振荡，
且水平饱和会减少垂直推力分量，把高度也一起带成耦合振荡。

改用五次多项式后：

- 参考位置连续、速度与加速度有界，且首末速度/加速度均为零；
- 控制器得到 ``v_des`` / ``a_des`` 前馈，不必靠位置误差"追"目标，因此不会饱和；
- 轨迹由解析式给出，是**确定性、可重复**的，便于 T3 阶段的对照实验。

曲线（``τ = t/T ∈ [0, 1]``）：

``s(τ) = 10τ³ − 15τ⁴ + 6τ⁵``，满足 ``s(1)=1``、``s'(0)=s'(1)=s''(0)=s''(1)=0``。

峰值：``s'`` 最大 ``1.875``（τ=0.5），``|s''|`` 最大 ``5.7735``（τ≈0.211/0.789）。
本模块只用这两个常数把"最大速度/加速度"约束换算成段时长 ``T``。
"""

from __future__ import annotations

import math

Vector3 = tuple[float, float, float]

#: 五次多项式 s(τ) 的峰值系数，用于由速度/加速度上限反推段时长。
_PEAK_FIRST_DERIVATIVE = 1.875
_PEAK_SECOND_DERIVATIVE = 5.7735


def _s(tau: float) -> float:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def _s_dot(tau: float) -> float:
    return 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4


def _s_ddot(tau: float) -> float:
    return 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3


def segment_duration(delta: Vector3, max_speed: float, max_accel: float) -> float:
    """按最大速度/加速度约束计算单个航点段的时长（秒）。

    用**向量模长**而非分轴距离反推：三个轴的 ``s'(τ)`` 同时达到峰值，因此：

    - 速度模长峰值 ``= 1.875·|Δ|/T``
    - 加速度模长峰值 ``= 5.7735·|Δ|/T²``

    若改成分轴取 max，只能保证每轴不超限，合成运动的模长仍会超出
    （实测 3D 段会到 ``1.014·v_max``，被单测 `test_peak_speed_and_accel_respect_limits` 抓到）。
    ``delta`` 为零向量时返回最小正时长，避免除零。
    """

    if max_speed <= 0.0 or max_accel <= 0.0:
        raise ValueError("max_speed 与 max_accel 必须为正")

    distance = math.sqrt(sum(float(value) ** 2 for value in delta))
    if distance <= 0.0:
        return 1e-3
    return max(
        _PEAK_FIRST_DERIVATIVE * distance / max_speed,
        math.sqrt(_PEAK_SECOND_DERIVATIVE * distance / max_accel),
    )


def quintic_reference(
    start: Vector3,
    goal: Vector3,
    duration: float,
    elapsed: float,
) -> tuple[Vector3, Vector3, Vector3]:
    """返回 ``(位置, 速度, 加速度)``；``elapsed`` 超出时长后夹到终点且前馈为零。"""

    if duration <= 0.0:
        raise ValueError("duration 必须为正")

    tau = min(max(elapsed / duration, 0.0), 1.0)
    position: list[float] = []
    velocity: list[float] = []
    acceleration: list[float] = []
    for axis in range(3):
        delta = float(goal[axis]) - float(start[axis])
        position.append(float(start[axis]) + delta * _s(tau))
        velocity.append(delta * _s_dot(tau) / duration)
        acceleration.append(delta * _s_ddot(tau) / duration**2)
    return (
        (position[0], position[1], position[2]),
        (velocity[0], velocity[1], velocity[2]),
        (acceleration[0], acceleration[1], acceleration[2]),
    )


class WaypointSegment:
    """一段确定性航点轨迹：给定起点、终点与限值，按时刻查询参考。"""

    def __init__(self, start: Vector3, goal: Vector3, max_speed: float, max_accel: float) -> None:
        self.start = start
        self.goal = goal
        self.duration = segment_duration(
            (goal[0] - start[0], goal[1] - start[1], goal[2] - start[2]),
            max_speed,
            max_accel,
        )

    def sample(self, elapsed: float) -> tuple[Vector3, Vector3, Vector3]:
        return quintic_reference(self.start, self.goal, self.duration, elapsed)

    def finished(self, elapsed: float) -> bool:
        """是否已到段末（用于决定何时切换到下一航点）。"""

        return elapsed >= self.duration
