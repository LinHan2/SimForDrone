"""目标量测仿真与可替换的估计器接口。

为什么要有这一层：制导只能看到“目标状态”，而真实系统里它来自感知（视觉/雷达）而不是
真值。把“真值 → 量测 → 估计”三段显式分开，视觉模块接入时只需替换本文件，
``guidance`` 与 ``px4ctrl`` 都不用改。

安全约定
--------
- 量测可以**无效**（遮挡、丢包、目标出视场）：:meth:`NoisyTargetSensor.measure` 返回
  ``None``，由 :func:`select_target_state` 保持上一帧估计，绝不把 ``None`` 或 0 值
  混进控制环。
- 估计器输出必须是“当前可用的目标状态”；可信程度由调用方按
  :class:`TargetMeasurement` 的时间戳判断。
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Protocol

from tracking.guidance import TargetState, Vector3


@dataclass(frozen=True)
class TargetMeasurement:
    """共享 ENU 下的含噪目标运动学量测，时间戳为发布进程的单调时钟。"""

    timestamp: float
    p: Vector3
    v: Vector3


class TargetStateEstimator(Protocol):
    """未来 EKF/UKF 的稳定边界。"""

    #: 最近一次有效估计；量测丢失时由 :func:`select_target_state` 取用。
    last_estimate: TargetState | None

    def update(self, measurement: TargetMeasurement) -> TargetState:
        """消费一帧量测并返回当前目标估计。"""


@dataclass
class NoisyTargetSensor:
    """对真值叠加可复现的独立高斯噪声，并可模拟量测丢失。"""

    position_std: float = 0.05
    velocity_std: float = 0.02
    seed: int = 0
    #: 单帧丢失概率（0 = 从不丢失）。为遮挡/丢包场景提供可复现的入口，
    #: 默认 0 时行为与旧实现完全一致。
    dropout_probability: float = 0.0

    def __post_init__(self) -> None:
        if self.position_std < 0.0 or self.velocity_std < 0.0:
            raise ValueError("噪声标准差不得为负")
        if not 0.0 <= self.dropout_probability <= 1.0:
            raise ValueError("dropout_probability 必须落在 [0, 1]")
        # 每个通道一条独立随机流：若共用一条流按调用顺序取数，x/y/z 的噪声会隐含
        # 与调用顺序相关（顺序一变整段噪声就变），也无法保证 p 与 v 之间独立。
        # 用字符串种子派生（CPython 对字符串种子做确定性转换），同 seed 跨进程可复现。
        self._streams = {
            channel: random.Random(f"{self.seed}:{channel}")
            for channel in ("px", "py", "pz", "vx", "vy", "vz", "dropout")
        }

    def measure(self, truth: TargetState, timestamp: float) -> TargetMeasurement | None:
        """返回一帧量测；返回 ``None`` 表示本帧无效（模拟遮挡或丢包）。"""

        if self.dropout_probability > 0.0:
            # 用独立流抽丢失判定，保证 p/v 的噪声序列不被丢失事件影响。
            if self._streams["dropout"].random() < self.dropout_probability:
                return None
        return TargetMeasurement(
            timestamp=timestamp,
            p=self._perturb(truth.p, self.position_std, ("px", "py", "pz")),
            v=self._perturb(truth.v, self.velocity_std, ("vx", "vy", "vz")),
        )

    def _perturb(
        self, value: Vector3, standard_deviation: float, channels: tuple[str, str, str]
    ) -> Vector3:
        return tuple(
            component + self._streams[channel].gauss(0.0, standard_deviation)
            for component, channel in zip(value, channels)
        )  # type: ignore[return-value]


class PassthroughEstimator:
    """占位估计器：原样透传量测，不做任何滤波。

    存在的意义是**基线对照**——它把“噪声”与“控制”的贡献分开，便于后续 EKF 做 A/B。
    它不是可用的估计器：使用 ``--state-source estimator`` 时进入控制环的就是逐帧含噪
    量测，``--position-noise-std`` 会直接变成跟踪抖动。
    """

    def __init__(self) -> None:
        self.last_estimate: TargetState | None = None

    def update(self, measurement: TargetMeasurement) -> TargetState:
        self.last_estimate = TargetState(p=measurement.p, v=measurement.v)
        return self.last_estimate


def select_target_state(
    source: str,
    truth: TargetState,
    timestamp: float,
    sensor: NoisyTargetSensor,
    estimator: TargetStateEstimator,
) -> tuple[TargetState, TargetMeasurement | None]:
    """在不改动制导的前提下选择真值通路或估计器通路。

    返回 ``(进入控制环的目标状态, 本周期量测或 None)``。第二个值只用于统计量测误差，
    与控制无关，因此量测丢失时它是 ``None``。
    """

    if source == "truth":
        return truth, None
    if source == "estimator":
        measurement = sensor.measure(truth, timestamp)
        if measurement is None:
            # 本帧无效：保持上一帧估计，而不是让 None 进入控制环（那会让 tracker
            # 停止更新或飞向原点）。若从未取得过有效量测，必须报错而不是猜一个状态。
            held = estimator.last_estimate
            if held is None:
                raise RuntimeError("首个量测即无效，无法建立目标状态；请检查遮挡或丢包设置")
            return held, None
        return estimator.update(measurement), measurement
    raise ValueError(f"未知目标状态源: {source}")
