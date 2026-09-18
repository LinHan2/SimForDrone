"""Target measurement simulation and replaceable estimator interfaces."""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Protocol

from tracking.guidance import TargetState, Vector3


@dataclass(frozen=True)
class TargetMeasurement:
    """Noisy target kinematics in shared ENU, stamped with monotonic time."""

    timestamp: float
    p: Vector3
    v: Vector3


class TargetStateEstimator(Protocol):
    """Stable boundary for future filters such as an EKF or UKF."""

    def update(self, measurement: TargetMeasurement) -> TargetState:
        """Consume one measurement and return the current target estimate."""


@dataclass
class NoisyTargetSensor:
    """Add deterministic independent Gaussian noise to exact target state."""

    position_std: float = 0.05
    velocity_std: float = 0.02
    seed: int = 0

    def __post_init__(self) -> None:
        if self.position_std < 0.0 or self.velocity_std < 0.0:
            raise ValueError("噪声标准差不得为负")
        self._random = random.Random(self.seed)

    def measure(self, truth: TargetState, timestamp: float) -> TargetMeasurement:
        return TargetMeasurement(
            timestamp=timestamp,
            p=self._perturb(truth.p, self.position_std),
            v=self._perturb(truth.v, self.velocity_std),
        )

    def _perturb(self, value: Vector3, standard_deviation: float) -> Vector3:
        return tuple(
            component + self._random.gauss(0.0, standard_deviation)
            for component in value
        )  # type: ignore[return-value]


class PassthroughEstimator:
    """Estimator placeholder that exposes noisy measurements unchanged."""

    def update(self, measurement: TargetMeasurement) -> TargetState:
        return TargetState(p=measurement.p, v=measurement.v)


def select_target_state(
    source: str,
    truth: TargetState,
    timestamp: float,
    sensor: NoisyTargetSensor,
    estimator: TargetStateEstimator,
) -> tuple[TargetState, TargetMeasurement | None]:
    """Select exact truth or the estimator path without changing guidance."""

    if source == "truth":
        return truth, None
    if source == "estimator":
        measurement = sensor.measure(truth, timestamp)
        return estimator.update(measurement), measurement
    raise ValueError(f"未知目标状态源: {source}")
