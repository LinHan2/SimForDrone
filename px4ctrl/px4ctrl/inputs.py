"""输入状态层。

对应上游 ``input.h``：为每一路输入（状态、扩展状态、里程计、IMU、电池、位置指令）保存
最新值与**接收时刻**，并提供新鲜度判定。状态机只依据本层判定决定是否降级，绝不在数据
陈旧时继续控制飞机。

坐标系约定（与上游一致，也与 ROS REP-103 一致）：
- 位置/速度/加速度均为世界系 **ENU**（x 东、y 北、z 上）。
- 姿态四元数为 ``(x, y, z, w)``，表示从机体 **FLU** 到世界 **ENU** 的旋转。

PX4 内部使用 **NED/FRD**。两者之间的转换集中在 :mod:`px4ctrl.frames` 一处，避免散落。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # (x, y, z, w)


@dataclass
class Stamped:
    """带接收时刻的输入基类语义：``recv_time`` 为单调时钟秒。"""

    recv_time: float = 0.0

    def is_fresh(self, now: float, timeout: float) -> bool:
        """是否在 ``timeout`` 秒内更新过。``recv_time==0`` 表示从未收到。"""

        return self.recv_time > 0.0 and (now - self.recv_time) <= timeout


@dataclass
class StateData(Stamped):
    """飞控状态（对应 ``mavros/state``）。"""

    connected: bool = False
    armed: bool = False
    mode: str = ""
    custom_mode: int = 0


@dataclass
class ExtendedStateData(Stamped):
    """扩展状态：落地检测等。"""

    landed_state: int = 0
    vtol_state: int = 0


@dataclass
class ImuData(Stamped):
    """IMU 数据（对应上游 ``Imu_Data_t``）。

    ``q`` 是姿态补偿项所需的世界系姿态（ENU/FLU）；``w`` 为机体角速度。

    注意数据来源：仿真里 ``q`` 与 :class:`OdomData` 的 ``q`` 同源自飞控 EKF，因此
    :class:`~px4ctrl.controller.LinearControl` 中的补偿项退化为恒等变换；真机上
    里程计通常来自 VIO、IMU 来自飞控，该项才真正起作用。保留它是为了两侧算法一致。
    """

    acc: Vector3 = (0.0, 0.0, 0.0)
    q: Quaternion = (0.0, 0.0, 0.0, 1.0)
    w: Vector3 = (0.0, 0.0, 0.0)


@dataclass
class OdomData(Stamped):
    """里程计：世界 ENU 下的位置/姿态/速度。"""

    p: Vector3 = (0.0, 0.0, 0.0)
    q: Quaternion = (0.0, 0.0, 0.0, 1.0)
    v: Vector3 = (0.0, 0.0, 0.0)


@dataclass
class BatteryData(Stamped):
    """电池状态。"""

    voltage: float = 0.0


@dataclass
class CommandData(Stamped):
    """制导层给出的期望状态（对应上游 ``quadrotor_msgs/PositionCommand``）。

    位置/速度/加速度为 ENU，``yaw`` 为弧度（ENU 下绕 z 轴，0 表示朝东）。
    """

    p: Vector3 = (0.0, 0.0, 0.0)
    v: Vector3 = (0.0, 0.0, 0.0)
    a: Vector3 = (0.0, 0.0, 0.0)
    j: Vector3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class TakeoffLandData:
    """自动起降过程的内部状态（对应上游 ``AutoTakeoffLand_t``）。"""

    landed: bool = True
    start_pose: Vector3 = (0.0, 0.0, 0.0)
    toggle_time: float = 0.0
    delay_trigger: bool = False
    delay_trigger_time: float = 0.0

    # 上游常量：起飞前让电机空转，以及到达目标高度后的停顿时间。
    MOTORS_SPEEDUP_TIME: float = field(default=3.0, init=False)
    DELAY_TRIGGER_TIME: float = field(default=2.0, init=False)


def yaw_from_quaternion(q: Quaternion) -> float:
    """从四元数取偏航角（ENU 下绕 z 轴，弧度）。"""

    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """由偏航角构造水平姿态四元数（ENU）。"""

    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def vlen(v: Vector3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def vsub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vadd(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vscale(a: Vector3, s: float) -> Vector3:
    return (a[0] * s, a[1] * s, a[2] * s)


def optional(value: Optional[float]) -> float:
    """把可能为 None 的标量折叠为 0.0，便于直接进入数值运算。"""

    return 0.0 if value is None else float(value)
