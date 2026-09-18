"""V0 ground-truth position tracking reference generator.

The target and observer geometry lives in a shared ENU frame. PX4 position
control, however, uses the observer's local ENU frame. The conversion below
transfers only the shared-frame displacement so the two origins are never
mixed.
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
    """Controller-facing reference in the observer's local ENU frame."""

    p: Vector3 = (0.0, 0.0, 0.0)
    v: Vector3 = (0.0, 0.0, 0.0)
    a: Vector3 = (0.0, 0.0, 0.0)
    j: Vector3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class TargetState:
    """Target kinematics expressed in the shared world ENU frame."""

    p: Vector3
    v: Vector3 = (0.0, 0.0, 0.0)
    a: Vector3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ObserverState:
    """Observer position in both shared and its PX4-local ENU frames."""

    shared_p: Vector3
    local_p: Vector3


@dataclass(frozen=True)
class PositionTrackerV0:
    """Generate a fixed-world-offset reference from exact target state."""

    relative_offset: Vector3 = (-3.0, 0.0, 1.0)
    yaw: float = 0.0

    def generate(self, target: TargetState, observer: ObserverState) -> DesiredState:
        """Return the observer-local desired state for the current target state."""

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
