"""共享 ENU 下的论文式 bearing-box 相对状态 EKF。

这是 Airsim2box ``StrictPaperRelativeEKF`` 到当前 PX4/ENU 链路的等价迁移。状态定义为
``x = [delta_p, delta_v, a_target, alpha]``，其中 ``delta_p = p_target - p_observer``。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

Vector3 = tuple[float, float, float]
_GRAVITY_ENU = np.array((0.0, 0.0, -9.81), dtype=np.float64)


@dataclass(frozen=True)
class RelativePoseMeasurement:
    """一帧由 bearing-box 前端与目标姿态得到的共享 ENU 量测。

    当前 ``relative_position`` 是过渡期位置代理；滤波器只使用其归一化方向作为
    ``p_bar``，仅在首帧用范数初始化尺度 ``alpha``。完整视觉前端应替换为 Lemma 1
    从 3D box 八角点得到的归一化位置方向。双目前端可额外提供 ``stereo_range_m``：它是
    相机到目标参考点的 metric 距离，不要求等于尺度状态 ``alpha``。
    """

    timestamp: float
    relative_position: Vector3
    target_thrust_direction: Vector3
    stereo_range_m: float | None = None


@dataclass(frozen=True)
class RelativeTargetEstimate:
    """相对 EKF 输出；所有向量均在共享 ENU 系。"""

    timestamp: float
    relative_position: Vector3
    relative_velocity: Vector3
    target_acceleration: Vector3
    scale: float


@dataclass(frozen=True)
class MetricTargetEstimate:
    timestamp: float
    relative_position: Vector3
    relative_velocity: Vector3
    target_acceleration: Vector3


@dataclass(frozen=True)
class MetricPoseMeasurement:
    timestamp: float
    relative_position: Vector3 | None
    target_thrust_direction: Vector3 | None


class RelativeTargetEKF:
    """按论文式 (34)、(39) 融合 bearing-box 与姿态-加速度约束。

    状态为 ``x=[delta_p, delta_v, a_target, alpha]``，预测使用常加速度模型：
    ``delta_p += delta_v*dt + 0.5*(a_target-a_observer)*dt**2``，
    ``delta_v += (a_target-a_observer)*dt``。

    量测为 ``delta_p-alpha*p_bar=0``、可选的双目范围 ``||delta_p||=rho`` 和
    ``B(h).T a_target=B(h).T g_enu``，其中 ``B(h)`` 是与推力方向正交的单位切空间基。
    过程噪声、量测噪声、遗忘因子、PSD 清理与限幅均取自
    Airsim2box 的已调参 ``StrictPaperRelativeEKF``；不加入论文之外的 jerk 状态或
    垂直加速度伪先验。
    """

    def __init__(
        self,
        position_std: float = 0.102,
        attitude_constraint_std: float = 0.03,
        position_process_std: float = 0.002,
        velocity_process_std: float = 0.1095445115,
        acceleration_process_std: float = 0.2449489743,
        scale_process_std: float = 0.0948683298,
        fading: float = 1.001,
        tilt_direction_std: float | None = None,
        stereo_range_std: float | None = None,
    ) -> None:
        if position_std <= 0.0 or attitude_constraint_std <= 0.0:
            raise ValueError("量测标准差必须为正")
        if (
            position_process_std <= 0.0
            or velocity_process_std <= 0.0
            or acceleration_process_std <= 0.0
            or scale_process_std <= 0.0
        ):
            raise ValueError("过程噪声标准差必须为正")
        if fading < 1.0:
            raise ValueError("遗忘因子不得小于 1")
        if tilt_direction_std is not None and tilt_direction_std <= 0.0:
            raise ValueError("tilt_direction_std 必须为正")
        if stereo_range_std is not None and stereo_range_std <= 0.0:
            raise ValueError("stereo_range_std 必须为正")
        self._position_variance = position_std**2
        self._attitude_variance = attitude_constraint_std**2
        self._position_process_variance = position_process_std**2
        self._velocity_process_variance = velocity_process_std**2
        self._acceleration_process_variance = acceleration_process_std**2
        self._scale_process_variance = scale_process_std**2
        self._fading = fading
        self._tilt_direction_std = tilt_direction_std
        self._stereo_range_std = stereo_range_std
        self._x = np.zeros(10, dtype=np.float64)
        self._x[9] = 1.0
        self._P = np.eye(10, dtype=np.float64) * 10.0
        self._timestamp: float | None = None
        # Airsim2box 物理限幅（数值保护，不构成动力学约束）。
        self._max_rel_dist = 120.0
        self._max_rel_speed = 10.0
        self._max_target_acc = 20.0
        self._min_scale = 0.2
        self._max_scale = 80.0

    @property
    def covariance(self) -> np.ndarray:
        """返回协方差副本，供影子模式记录与数值诊断使用。"""

        return self._P.copy()

    def get_covariance_diagonal(self) -> np.ndarray:
        """返回协方差对角线，供影子模式轻量记录不确定度。"""

        return np.diag(self._P)

    def initialize(
        self,
        relative_position: Vector3,
        relative_velocity: Vector3 = (0.0, 0.0, 0.0),
        scale: float | None = None,
    ) -> None:
        """按 Airsim2box 语义初始化十维状态。

        ``scale`` 对应论文的目标物理尺度 ``alpha``；过渡期位置代理没有真实的
        Lemma 1 尺度时，使用初始相对距离仅作为数值初值，后续只由 bearing-box 方程更新。
        """

        position = self._vector(relative_position, "relative_position")
        velocity = self._vector(relative_velocity, "relative_velocity")
        initial_scale = float(np.linalg.norm(position)) if scale is None else float(scale)
        if initial_scale <= 0.0 or not np.isfinite(initial_scale):
            raise ValueError("scale 必须为正且有限")
        self._x[:3] = position
        self._x[3:6] = velocity
        self._x[6:9] = 0.0
        self._x[9] = float(np.clip(initial_scale, self._min_scale, self._max_scale))
        self._P = np.eye(10, dtype=np.float64) * 10.0
        self._timestamp = None

    @property
    def tuning(self) -> dict[str, float | None]:
        """返回本滤波器实际使用的标准差，供实验记录复现。"""

        return {
            "position_std_m": self._position_variance**0.5,
            "attitude_constraint_std_mps2": self._attitude_variance**0.5,
            "position_process_std_m": self._position_process_variance**0.5,
            "velocity_process_std_mps": self._velocity_process_variance**0.5,
            "acceleration_process_std_mps2": self._acceleration_process_variance**0.5,
            "scale_process_std_m": self._scale_process_variance**0.5,
            "fading_lambda": self._fading,
            "stereo_range_std_m": self._stereo_range_std,
        }

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
        stereo_range = measurement.stereo_range_m
        if stereo_range is not None and (stereo_range <= 0.0 or not np.isfinite(stereo_range)):
            raise ValueError("stereo_range_m 必须为正且有限")

        if self._timestamp is None:
            self.initialize(
                (
                    float(relative_position[0]),
                    float(relative_position[1]),
                    float(relative_position[2]),
                )
            )
        else:
            dt = measurement.timestamp - self._timestamp
            if dt <= 0.0:
                raise ValueError("timestamp 必须严格递增")
            self._predict(dt, observer_acceleration_array)

        self._position_update(relative_position)
        if stereo_range is not None and self._stereo_range_std is not None:
            self._stereo_range_update(float(stereo_range))
        self._attitude_update(thrust_direction)
        self._sanitize_covariance()
        self._clamp_state()
        self._timestamp = measurement.timestamp
        return RelativeTargetEstimate(
            timestamp=measurement.timestamp,
            relative_position=(float(self._x[0]), float(self._x[1]), float(self._x[2])),
            relative_velocity=(float(self._x[3]), float(self._x[4]), float(self._x[5])),
            target_acceleration=(float(self._x[6]), float(self._x[7]), float(self._x[8])),
            scale=float(self._x[9]),
        )

    def _predict(self, dt: float, observer_acceleration: np.ndarray) -> None:
        identity = np.eye(3)
        relative_acceleration = self._x[6:9] - observer_acceleration
        self._x[:3] += self._x[3:6] * dt + 0.5 * relative_acceleration * dt**2
        self._x[3:6] += relative_acceleration * dt

        transition = np.eye(10)
        transition[:3, 3:6] = identity * dt
        transition[:3, 6:9] = identity * (0.5 * dt**2)
        transition[3:6, 6:9] = identity * dt
        process_noise = np.zeros((10, 10))
        process_noise[:3, :3] = identity * self._position_process_variance
        process_noise[3:6, 3:6] = identity * self._velocity_process_variance
        process_noise[6:9, 6:9] = identity * self._acceleration_process_variance
        process_noise[9, 9] = self._scale_process_variance
        self._P = transition @ self._P @ transition.T + process_noise
        self._P *= self._fading
        self._sanitize_covariance()
        self._clamp_state()

    def _position_update(self, relative_position: np.ndarray) -> None:
        distance = float(np.linalg.norm(relative_position))
        if distance < 1e-8:
            raise ValueError("relative_position 不得为零向量")
        normalized_position = relative_position / distance
        observation = np.zeros((3, 10))
        observation[:, :3] = np.eye(3)
        observation[:, 9] = -normalized_position
        self._kalman_update(
            observation,
            np.zeros(3),
            np.eye(3) * self._position_variance,
        )

    def _attitude_update(self, thrust_direction: np.ndarray) -> None:
        tangent_basis = self._tangent_basis(thrust_direction)
        observation = np.zeros((2, 10))
        observation[:, 6:9] = tangent_basis.T
        self._kalman_update(
            observation,
            tangent_basis.T @ _GRAVITY_ENU,
            self._tilt_measurement_noise(),
        )

    def _stereo_range_update(self, stereo_range: float) -> None:
        """融合双目提供的 metric range，而不将 range 错当作 ``alpha``。"""

        assert self._stereo_range_std is not None
        predicted_range = float(np.linalg.norm(self._x[:3]))
        if predicted_range < 1e-8:
            return
        observation = np.zeros((1, 10))
        observation[0, :3] = self._x[:3] / predicted_range
        self._kalman_update(
            observation,
            np.array((stereo_range,)),
            np.array(((self._stereo_range_std**2,),)),
        )

    def _tilt_measurement_noise(self) -> np.ndarray:
        """返回二维切空间量测噪声；默认保留已调的固定加速度方差。"""

        if self._tilt_direction_std is None:
            return np.eye(2) * self._attitude_variance
        thrust_acceleration = float(np.linalg.norm(self._x[6:9] - _GRAVITY_ENU))
        scale = max(thrust_acceleration, 1e-6) * self._tilt_direction_std
        return np.eye(2) * scale**2

    def _kalman_update(self, observation: np.ndarray, value: np.ndarray, noise: np.ndarray) -> None:
        innovation = value - observation @ self._x
        innovation_covariance = observation @ self._P @ observation.T + noise
        gain = np.linalg.solve(innovation_covariance, observation @ self._P).T
        self._x += gain @ innovation
        identity = np.eye(10)
        residual = identity - gain @ observation
        self._P = residual @ self._P @ residual.T + gain @ noise @ gain.T

    def _sanitize_covariance(self) -> None:
        if not np.all(np.isfinite(self._P)):
            # Airsim2box 语义：协方差失效时硬重置为初始值，而不是让异常传播。
            self._P = np.eye(10, dtype=np.float64) * 10.0
            return
        symmetric = 0.5 * (self._P + self._P.T)
        eigenvalues, eigenvectors = np.linalg.eigh(symmetric)
        self._P = (eigenvectors * np.clip(eigenvalues, 1e-8, 1e6)) @ eigenvectors.T
        self._P = 0.5 * (self._P + self._P.T)

    def _clamp_state(self) -> None:
        """Airsim2box 物理限幅：仅作数值保护，不构成动力学约束。"""

        for block, limit in (
            (slice(0, 3), self._max_rel_dist),
            (slice(3, 6), self._max_rel_speed),
            (slice(6, 9), self._max_target_acc),
        ):
            norm = float(np.linalg.norm(self._x[block]))
            if norm > limit:
                self._x[block] *= limit / norm
        self._x[9] = float(np.clip(self._x[9], self._min_scale, self._max_scale))

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

    @staticmethod
    def _tangent_basis(thrust_direction: np.ndarray) -> np.ndarray:
        """返回单位推力轴在 ``S²`` 上的确定性二维正交切空间基。"""

        reference_axis = np.eye(3)[int(np.argmin(np.abs(thrust_direction)))]
        first = reference_axis - thrust_direction * float(reference_axis @ thrust_direction)
        first /= float(np.linalg.norm(first))
        second = np.cross(thrust_direction, first)
        return np.column_stack((first, second))


class MetricRelativeTargetEKF(RelativeTargetEKF):
    """九维米制相对位置、速度和目标加速度滤波器。"""

    def __init__(
        self,
        position_std: float = 0.102,
        attitude_constraint_std: float = 0.03,
        position_process_std: float = 0.002,
        velocity_process_std: float = 0.1095445115,
        acceleration_process_std: float = 0.2449489743,
        fading: float = 1.001,
        tilt_direction_std: float | None = None,
    ) -> None:
        super().__init__(position_std, attitude_constraint_std, position_process_std,
                         velocity_process_std, acceleration_process_std, fading=fading,
                         tilt_direction_std=tilt_direction_std)
        self._x = np.zeros(9, dtype=np.float64)
        self._P = np.eye(9, dtype=np.float64) * 10.0

    @property
    def tuning(self) -> dict[str, float | None]:
        return {key: value for key, value in super().tuning.items()
                if key not in ("scale_process_std_m", "stereo_range_std_m")}

    def initialize(self, relative_position: Vector3, relative_velocity: Vector3 = (0.0, 0.0, 0.0)) -> None:
        self._x[:3] = self._vector(relative_position, "relative_position")
        self._x[3:6] = self._vector(relative_velocity, "relative_velocity")
        self._x[6:9] = 0.0
        self._P = np.eye(9, dtype=np.float64) * 10.0
        self._timestamp = None

    def update(
        self, measurement: MetricPoseMeasurement, observer_acceleration: Vector3
    ) -> MetricTargetEstimate:
        # x = [delta_p, delta_v, a_target] (共享 ENU), delta_p = p_target - p_tracker。
        # 可用时分别融合 z_p = delta_p + noise 和 B(h)^T a_target = B(h)^T g；
        # B(h) 是目标推力方向 h 的正交切空间基，缺失的量测跳过对应更新。
        position = (None if measurement.relative_position is None else
                    self._vector(measurement.relative_position, "relative_position"))
        thrust = (None if measurement.target_thrust_direction is None else
                  self._unit_vector(measurement.target_thrust_direction, "target_thrust_direction"))
        acceleration = self._vector(observer_acceleration, "observer_acceleration")
        if not np.isfinite(measurement.timestamp):
            raise ValueError("timestamp 必须为有限数")
        if self._timestamp is None:
            if position is None:
                raise ValueError("首帧需要有效米制相对位置")
            self.initialize((float(position[0]), float(position[1]), float(position[2])))
        else:
            dt = measurement.timestamp - self._timestamp
            if dt <= 0.0:
                raise ValueError("timestamp 必须严格递增")
            self._predict(dt, acceleration)
        if position is not None:
            observation = np.zeros((3, 9))
            observation[:, :3] = np.eye(3)
            self._kalman_update(observation, position, np.eye(3) * self._position_variance)
        if thrust is not None:
            basis = self._tangent_basis(thrust)
            observation = np.zeros((2, 9))
            observation[:, 6:9] = basis.T
            self._kalman_update(observation, basis.T @ _GRAVITY_ENU, self._tilt_measurement_noise())
        self._sanitize_covariance()
        self._clamp_state()
        self._timestamp = measurement.timestamp
        return MetricTargetEstimate(
            timestamp=measurement.timestamp,
            relative_position=(float(self._x[0]), float(self._x[1]), float(self._x[2])),
            relative_velocity=(float(self._x[3]), float(self._x[4]), float(self._x[5])),
            target_acceleration=(float(self._x[6]), float(self._x[7]), float(self._x[8])),
        )

    def _kalman_update(self, observation: np.ndarray, value: np.ndarray, noise: np.ndarray) -> None:
        # y = z - H x, S = H P H^T + R, K = P H^T S^-1；
        # x+ = x + K y, P+ = (I-KH) P (I-KH)^T + K R K^T (Joseph 形式)。
        innovation = value - observation @ self._x
        covariance = observation @ self._P @ observation.T + noise
        gain = np.linalg.solve(covariance, observation @ self._P).T
        self._x += gain @ innovation
        residual = np.eye(9) - gain @ observation
        self._P = residual @ self._P @ residual.T + gain @ noise @ gain.T

    def _clamp_state(self) -> None:
        for block, limit in ((slice(0, 3), self._max_rel_dist),
                             (slice(3, 6), self._max_rel_speed),
                             (slice(6, 9), self._max_target_acc)):
            norm = float(np.linalg.norm(self._x[block]))
            if norm > limit:
                self._x[block] *= limit / norm

    def _predict(self, dt: float, observer_acceleration: np.ndarray) -> None:
        # a_rel = a_target - a_tracker, delta_p+ = delta_p + delta_v dt + a_rel dt^2/2,
        # delta_v+ = delta_v + a_rel dt, a_target+ = a_target；P+ = lambda(F P F^T + Q)。
        # F 的 p-v、p-a、v-a 块分别是 dt I、dt^2 I/2、dt I。
        relative_acceleration = self._x[6:9] - observer_acceleration
        self._x[:3] += self._x[3:6] * dt + 0.5 * relative_acceleration * dt**2
        self._x[3:6] += relative_acceleration * dt
        transition = np.eye(9)
        transition[:3, 3:6] = np.eye(3) * dt
        transition[:3, 6:9] = np.eye(3) * (0.5 * dt**2)
        transition[3:6, 6:9] = np.eye(3) * dt
        noise = np.diag([self._position_process_variance] * 3
                        + [self._velocity_process_variance] * 3
                        + [self._acceleration_process_variance] * 3)
        self._P = (transition @ self._P @ transition.T + noise) * self._fading
        self._sanitize_covariance()
        self._clamp_state()