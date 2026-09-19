"""V0 真值位置跟踪的参考生成器。

目标与观测机的几何关系发生在**共享 ENU** 系里，而 PX4 位置控制使用观测机自己的
**local ENU** 系。下面的转换只搬运共享系中的**相对位移**，两个原点因此永不混用。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias

Vector3: TypeAlias = tuple[float, float, float]


def _add(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _sub(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


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

        # 先在共享系里算出期望站位与相对误差。
        desired_shared = _add(target.p, self.relative_offset)
        shared_error = _sub(desired_shared, observer.shared_p)
        # 只把共享系的**相对位移**叠加到 tracker 的 local 位置：
        # 两机 PX4 EKF 的 local 原点各自建立、互不重合，直接使用共享系绝对坐标
        # 会让位置设定点整体偏移（实测两机 local 原点可相差数十厘米以上）。
        desired_local = _add(observer.local_p, shared_error)
        return DesiredState(
            p=desired_local,
            v=target.v,
            a=target.a,
            yaw=self.yaw,
        )
