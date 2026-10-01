"""Tracker 跟踪段的完整加速度限幅；不改变公共控制器或起降状态。"""

from dataclasses import dataclass, replace
import math

from px4ctrl.controller import LinearControl


@dataclass(frozen=True)
class TrackingLimits:
    horizontal: float = 1.5
    vertical_min: float = -0.8
    vertical_max: float = 0.8
    jerk: float = 3.0
    max_dt: float = 0.1

    def __post_init__(self):
        values = (self.horizontal, self.vertical_min, self.vertical_max, self.jerk, self.max_dt)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("跟踪加速度限值必须有限")
        if (min(self.horizontal, self.jerk, self.max_dt) <= 0
                or not self.vertical_min < 0 < self.vertical_max):
            raise ValueError("水平/jerk/周期限值须为正，垂向范围须包含零")


class TrackerLinearControl(LinearControl):
    """仅 active() 为真时限幅，使用 FSM 刷新后的同一份 odom 防止重复 PD。"""

    def __init__(self, params, limits: TrackingLimits):
        super().__init__(params)
        if not 0 < params.max_angle_rad < math.pi / 2:
            raise ValueError("跟踪限幅需要有限的正倾角上限")
        if params.gra + limits.vertical_min <= params.gra * self.MIN_TILT_DIVISOR_RATIO:
            raise ValueError("垂向下界必须保留足够重力补偿余量")
        self.limits = limits
        self.active = lambda: False
        self.tracking_debug = {}
        self._previous_acceleration = (0.0, 0.0, 0.0)
        self._previous_time = None

    def calculate_control(self, des, odom, imu, out=None, now=0.0):
        if not self.active():
            self._previous_time = None
            self._previous_acceleration = (0.0, 0.0, 0.0)
            self.tracking_debug = {}
            return super().calculate_control(des, odom, imu, out, now)

        gains = self.params.gain
        position_gains = (gains.kp0, gains.kp1, gains.kp2)
        velocity_gains = (gains.kv0, gains.kv1, gains.kv2)
        raw = tuple(des.a[axis] + position_gains[axis] * (des.p[axis] - odom.p[axis])
                    + velocity_gains[axis] * (des.v[axis] - odom.v[axis]) for axis in range(3))
        if not all(math.isfinite(value) for value in (*raw, now)):
            raise ValueError("跟踪控制量或时间无效")
        if self._previous_time is not None and now < self._previous_time:
            raise ValueError("跟踪控制时钟倒退")

        vertical = max(self.limits.vertical_min, min(self.limits.vertical_max, raw[2]))
        tilt_bound = (self.params.gra + vertical) * math.tan(self.params.max_angle_rad)
        horizontal_bound = min(self.limits.horizontal, tilt_bound)
        horizontal = math.hypot(raw[0], raw[1])
        scale = min(1.0, horizontal_bound / horizontal) if horizontal else 1.0
        bounded = (raw[0] * scale, raw[1] * scale, vertical)
        dt = 0.0 if self._previous_time is None else min(now - self._previous_time, self.limits.max_dt)
        delta = tuple(bounded[axis] - self._previous_acceleration[axis] for axis in range(3))
        magnitude = math.hypot(*delta)
        fraction = min(1.0, self.limits.jerk * dt / magnitude) if magnitude else 1.0
        limited = tuple(self._previous_acceleration[axis] + fraction * delta[axis] for axis in range(3))
        self._previous_acceleration = limited
        self._previous_time = now
        self.tracking_debug = {
            "raw_acceleration": raw, "limited_acceleration": limited,
            "acceleration_limited": bounded != raw, "jerk_limited": fraction < 1.0,
            "tilt_limited": horizontal > tilt_bound, "limiter_dt": dt,
        }
        neutral = replace(des, p=odom.p, v=odom.v, a=limited)
        return super().calculate_control(neutral, odom, imu, out, now)