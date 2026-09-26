"""共享 ENU 下的 RGB-D 相对目标状态 EKF。

这是估计器的影子模式内核，不负责读取相机或发送飞控指令。状态定义为
``x = [delta_p, delta_v, a_target]``，其中 ``delta_p = p_target - p_observer``。
RGB-D 位姿提供相对位置；目标姿态提供推力方向，用于施加论文中的加速度投影约束。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

Vector3 = tuple[float, float, float]
_GRAVITY_ENU = np.array((0.0, 0.0, -9.81), dtype=np.float64)


@dataclass(frozen=True)
class RelativePoseMeasurement:
    """一帧由 RGB-D 与目标姿态得到的共享 ENU 量测。

    ``target_thrust_direction`` 是目标机推力在世界 ENU 中的作用方向。因而悬停时
    它为 ``(0, 0, 1)``，而不是旧 AirSim/NED 实现中使用的向下正方向。
    """

    timestamp: float
    relative_position: Vector3
    target_thrust_direction: Vector3


@dataclass(frozen=True)
class RelativeTargetEstimate:
    """相对 EKF 输出；所有向量均在共享 ENU 系。"""

    timestamp: float
    relative_position: Vector3
    relative_velocity: Vector3
    target_acceleration: Vector3


class RelativeTargetEKF:
    """将 RGB-D 相对位置和姿态-加速度约束融合为相对状态。

    过程模型为

    ``delta_p += delta_v * dt + 0.5 * (a_target - a_observer) * dt**2``
    ``delta_v += (a_target - a_observer) * dt``。

    与旧 AirSim 的 10 维 bearing-box EKF 不同，RGB-D 已给出带尺度的相对位置，故不再
    估计不可观的尺度变量 ``alpha``。保留的姿态伪量测为
    ``P_h a_target = P_h g_enu``，其中 ``P_h = I - h h^T``。
    """

    def __init__(
        self,
        position_std: float = 0.08,
        attitude_constraint_std: float = 0.15,
        acceleration_process_std: float = 1.0,
    ) -> None:
        if position_std <= 0.0 or attitude_constraint_std <= 0.0:
            raise ValueError("量测标准差必须为正")
        if acceleration_process_std <= 0.0:
            raise ValueError("加速度过程标准差必须为正")
        self._position_variance = position_std**2
        self._attitude_variance = attitude_constraint_std**2
        self._acceleration_variance = acceleration_process_std**2
        self._x = np.zeros(9, dtype=np.float64)
        self._P = np.diag((4.0, 4.0, 4.0, 1.0, 1.0, 1.0, 9.0, 9.0, 9.0))
        self._timestamp: float | None = None

    @property
    def covariance(self) -> np.ndarray:
        """返回协方差副本，供影子模式记录与数值诊断使用。"""

        return self._P.copy()

    def update(
        self, measurement: RelativePoseMeasurement, observer_acceleration: Vector3
    ) -> RelativeTargetEstimate:
        """融合一帧量测与观测机世界系加速度。"""

        relative_position = self._vector(measurement.relative_position, "relative_position")
        thrust_direction = self._unit_vector(
            measurement.target_thrust_direction, "target_thrust_direction"
        )
        observer_acceleration_array = self._vector(
            observer_acceleration, "observer_acceleration"
        )
        if not np.isfinite(measurement.timestamp):
            raise ValueError("timestamp 必须为有限数")

        if self._timestamp is None:
            self._x[:3] = relative_position
        else:
            dt = measurement.timestamp - self._timestamp
            if dt <= 0.0:
                raise ValueError("timestamp 必须严格递增")
            self._predict(dt, observer_acceleration_array)

        self._position_update(relative_position)
        self._attitude_update(thrust_direction)
        self._sanitize_covariance()
        self._timestamp = measurement.timestamp
        return RelativeTargetEstimate(
            timestamp=measurement.timestamp,
            relative_position=tuple(float(value) for value in self._x[:3]),
            relative_velocity=tuple(float(value) for value in self._x[3:6]),
            target_acceleration=tuple(float(value) for value in self._x[6:9]),
        )

    def _predict(self, dt: float, observer_acceleration: np.ndarray) -> None:
        identity = np.eye(3)
        relative_acceleration = self._x[6:9] - observer_acceleration
        self._x[:3] += self._x[3:6] * dt + 0.5 * relative_acceleration * dt**2
        self._x[3:6] += relative_acceleration * dt

        transition = np.eye(9)
        transition[:3, 3:6] = identity * dt
        transition[:3, 6:9] = identity * (0.5 * dt**2)
        transition[3:6, 6:9] = identity * dt
        process_noise = np.zeros((9, 9))
        process_noise[:3, :3] = identity * self._acceleration_variance * dt**4 / 4.0
        process_noise[3:6, 3:6] = identity * self._acceleration_variance * dt**2
        process_noise[6:9, 6:9] = identity * self._acceleration_variance * dt
        self._P = transition @ self._P @ transition.T + process_noise

    def _position_update(self, relative_position: np.ndarray) -> None:
        observation = np.zeros((3, 9))
        observation[:, :3] = np.eye(3)
        self._kalman_update(observation, relative_position, np.eye(3) * self._position_variance)

    def _attitude_update(self, thrust_direction: np.ndarray) -> None:
        projection = np.eye(3) - np.outer(thrust_direction, thrust_direction)
        observation = np.zeros((3, 9))
        observation[:, 6:9] = projection
        self._kalman_update(
            observation,
            projection @ _GRAVITY_ENU,
            np.eye(3) * self._attitude_variance,
        )

    def _kalman_update(self, observation: np.ndarray, value: np.ndarray, noise: np.ndarray) -> None:
        innovation = value - observation @ self._x
        innovation_covariance = observation @ self._P @ observation.T + noise
        gain = np.linalg.solve(innovation_covariance, observation @ self._P).T
        self._x += gain @ innovation
        identity = np.eye(9)
        residual = identity - gain @ observation
        self._P = residual @ self._P @ residual.T + gain @ noise @ gain.T

    def _sanitize_covariance(self) -> None:
        if not np.all(np.isfinite(self._P)):
            raise RuntimeError("EKF 协方差出现非有限值")
        symmetric = 0.5 * (self._P + self._P.T)
        eigenvalues, eigenvectors = np.linalg.eigh(symmetric)
        self._P = (eigenvectors * np.clip(eigenvalues, 1e-10, None)) @ eigenvectors.T
        self._P = 0.5 * (self._P + self._P.T)

    @staticmethod
    def _vector(value: Vector3, name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} 必须是三个有限数")
        return vector

    @classmethod
    def _unit_vector(cls, value: Vector3, name: str) -> np.ndarray:
        vector = cls._vector(value, name)
        norm = float(np.linalg.norm(vector))
        if norm < 1e-8:
            raise ValueError(f"{name} 不得为零向量")
        return vector / norm