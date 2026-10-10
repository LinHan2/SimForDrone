"""共享 ENU 下的相对状态滤波核心，不负责图像检测、GT 评分或飞控指令。

RelativeTargetEKF：保留的十维 bearing-box/尺度版本，x = [r, v_rel, a_target, alpha]。
MetricRelativeTargetEKF：当前 RGB-D shadow 使用的九维米制版本，x = [r, v_rel, a_target]。
两者均定义 r = p_target - p_observer，v_rel = v_target - v_observer。
输入 observer_acceleration 是观察机世界系线加速度，已去除比力中的重力影响。
目标姿态在调用端转成世界系单位推力轴 h；本文件只处理 h，不直接处理四元数。

流程：检查输入 -> 首帧初始化/后续预测 -> 位置更新 -> 二维姿态更新 -> 数值保护 -> 输出。
九维版本在给定 h 后使用线性位置/姿态观测；EKF 名称沿用项目接口。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

Vector3 = tuple[float, float, float]
# ENU 的正 Z 向上；g 是世界系重力加速度，不是控制器用于补偿的正向 +9.81。
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
    """九维后验输出：米制相对位置、相对速度和目标绝对线加速度，均为 ENU。

    a_target 不是相对加速度；调用端可用 p_target_hat = p_observer + r_hat、
    v_target_hat = v_observer + v_rel_hat 恢复目标绝对状态。
    """

    timestamp: float
    relative_position: Vector3
    relative_velocity: Vector3
    target_acceleration: Vector3


@dataclass(frozen=True)
class MetricPoseMeasurement:
    """米制量测输入；位置单位 m，推力轴为世界系方向，不携带 GT 速度或加速度。

    timestamp 必须来自同一时间域且严格递增；None 表示跳过对应量测更新。
    首帧必须有位置，后续两项均缺失时仅执行预测。
    """

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
        # 标准差决定 Q/R 的权重；必须为正，否则会出现零噪声的过强约束。
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
        # 接口接收标准差，矩阵使用方差；位置 m、速度 m/s、加速度 m/s^2。
        self._position_variance = position_std**2
        self._attitude_variance = attitude_constraint_std**2
        self._position_process_variance = position_process_std**2
        self._velocity_process_variance = velocity_process_std**2
        self._acceleration_process_variance = acceleration_process_std**2
        self._scale_process_variance = scale_process_std**2
        # fading >= 1 放大不确定度，使滤波器不因长期运行而过度自信。
        self._fading = fading
        self._tilt_direction_std = tilt_direction_std
        self._stereo_range_std = stereo_range_std
        # 十维状态块：0:3 位置、3:6 速度、6:9 目标加速度、9 尺度。
        self._x = np.zeros(10, dtype=np.float64)
        self._x[9] = 1.0
        # P_ij = Cov(x_i, x_j)；较大的初始对角值允许量测迅速修正初值。
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

        # 初始化只设先验，不融合量测；首帧 update 随后仍会执行观测更新。
        position = self._vector(relative_position, "relative_position")
        velocity = self._vector(relative_velocity, "relative_velocity")
        initial_scale = float(np.linalg.norm(position)) if scale is None else float(scale)
        if initial_scale <= 0.0 or not np.isfinite(initial_scale):
            raise ValueError("scale 必须为正且有限")
        self._x[:3] = position
        self._x[3:6] = velocity
        # 未提供加速度先验，先置零；速度默认置零，不从目标 GT 读取。
        self._x[6:9] = 0.0
        self._x[9] = float(np.clip(initial_scale, self._min_scale, self._max_scale))
        self._P = np.eye(10, dtype=np.float64) * 10.0
        # 清除历史时间，避免复位后与上一段序列计算 dt。
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

        # 1. 校验输入，并归一化推力轴；量测有效性/传感器过期由调用端判断。
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

        # 2. 首帧建立先验；后续帧按真实时间差推进常加速度模型。
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

        # 3. 依次融合方向、可选距离和姿态；每次都用上一次更新后的 x/P。
        self._position_update(relative_position)
        if stereo_range is not None and self._stereo_range_std is not None:
            self._stereo_range_update(float(stereo_range))
        self._attitude_update(thrust_direction)
        # 4. 清理浮点误差、限制状态幅值，再记录本帧时间并导出后验。
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
        # 十维预测：r^- = r + v_rel dt + (a_target-a_observer) dt^2/2，
        # v_rel^- = v_rel + (a_target-a_observer) dt；a_target、alpha 的均值保持不变。
        identity = np.eye(3)
        relative_acceleration = self._x[6:9] - observer_acceleration
        self._x[:3] += self._x[3:6] * dt + 0.5 * relative_acceleration * dt**2
        self._x[3:6] += relative_acceleration * dt

        # F = df/dx；位置/速度对加速度的依赖建立交叉协方差，供后续间接修正。
        transition = np.eye(10)
        transition[:3, 3:6] = identity * dt
        transition[:3, 6:9] = identity * (0.5 * dt**2)
        transition[3:6, 6:9] = identity * dt
        # Q 按状态块加入独立过程噪声；当前为每步固定方差，不随 dt 缩放。
        process_noise = np.zeros((10, 10))
        process_noise[:3, :3] = identity * self._position_process_variance
        process_noise[3:6, 3:6] = identity * self._velocity_process_variance
        process_noise[6:9, 6:9] = identity * self._acceleration_process_variance
        process_noise[9, 9] = self._scale_process_variance
        # P^- = fading * (F P^+ F^T + Q)；观察机加速度在此当作确定的已知输入。
        self._P = transition @ self._P @ transition.T + process_noise
        self._P *= self._fading
        self._sanitize_covariance()
        self._clamp_state()

    def _position_update(self, relative_position: np.ndarray) -> None:
        # 旧版本只保留方向 p_bar = r_measured / ||r_measured||，不是米制位置更新。
        distance = float(np.linalg.norm(relative_position))
        if distance < 1e-8:
            raise ValueError("relative_position 不得为零向量")
        normalized_position = relative_position / distance
        # 伪量测 0 = r - alpha p_bar + n；H = [I3, 0, 0, -p_bar]。
        observation = np.zeros((3, 10))
        observation[:, :3] = np.eye(3)
        observation[:, 9] = -normalized_position
        self._kalman_update(
            observation,
            np.zeros(3),
            np.eye(3) * self._position_variance,
        )

    def _attitude_update(self, thrust_direction: np.ndarray) -> None:
        # a_target = g + lambda h；左乘 B^T 消掉未知推力大小 lambda。
        # 因而 z = B^T g，H = [0, 0, B^T, 0]，每帧只施加两个独立方向约束。
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
        # 非线性量测 rho = ||r|| + n；在当前 r_hat 处线性化。
        predicted_range = float(np.linalg.norm(self._x[:3]))
        if predicted_range < 1e-8:
            return
        # H_rho = [r_hat^T/||r_hat||, 0, 0, 0]。
        # H_rho x_hat = ||r_hat||，因此通用更新中的 z-Hx 等于真实范围创新。
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
            # 默认：R_h = sigma_constraint^2 I2，为软动力学约束保留模型误差。
            return np.eye(2) * self._attitude_variance
        # 可选方向噪声近似：sigma_acc ~= ||a_target-g|| sigma_direction。
        # 推力越大，同一方向误差产生的切向加速度误差越大；这不是完整姿态协方差传播。
        thrust_acceleration = float(np.linalg.norm(self._x[6:9] - _GRAVITY_ENU))
        scale = max(thrust_acceleration, 1e-6) * self._tilt_direction_std
        return np.eye(2) * scale**2

    def _kalman_update(self, observation: np.ndarray, value: np.ndarray, noise: np.ndarray) -> None:
        # 创新 y=z-Hx：当前量测与先验预测的差；S=HPH^T+R：创新的不确定度。
        innovation = value - observation @ self._x
        innovation_covariance = observation @ self._P @ observation.T + noise
        # K=PH^T S^-1；用 solve 解线性方程，避免显式求逆。
        gain = np.linalg.solve(innovation_covariance, observation @ self._P).T
        # 后验均值 x^+=x^-+Ky；增益自动在模型先验与量测之间分配权重。
        self._x += gain @ innovation
        identity = np.eye(10)
        residual = identity - gain @ observation
        # Joseph 形式 P^+=(I-KH)P^-(I-KH)^T+KRK^T，比直接相减更抗舍入误差。
        self._P = residual @ self._P @ residual.T + gain @ noise @ gain.T

    def _sanitize_covariance(self) -> None:
        """保护协方差的有限性、对称性及正定性，不属于额外物理观测。"""

        if not np.all(np.isfinite(self._P)):
            # Airsim2box 语义：协方差失效时硬重置为初始值，而不是让异常传播。
            # 当前重置写死十维；九维子类沿用此分支存在维度风险，注释不改变原行为。
            self._P = np.eye(10, dtype=np.float64) * 10.0
            return
        # 1. 去掉数值运算导致的非对称部分，确保使用对称矩阵特征分解。
        symmetric = 0.5 * (self._P + self._P.T)
        eigenvalues, eigenvectors = np.linalg.eigh(symmetric)
        # 2. P=V diag(d) V^T；裁剪特征值以避免负方差、奇异或无限膨胀。
        self._P = (eigenvectors * np.clip(eigenvalues, 1e-8, 1e6)) @ eigenvectors.T
        # 3. 重建后再对称化，清除最后一轮舍入误差。
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
                # 按向量范数同比缩放，保留方向；不会同步修正 P，不能当作约束滤波。
                self._x[block] *= limit / norm
        self._x[9] = float(np.clip(self._x[9], self._min_scale, self._max_scale))

    @staticmethod
    def _vector(value: Vector3, name: str) -> np.ndarray:
        # 所有矩阵运算统一 float64；拒绝 NaN/Inf 和错误维数，防止污染整个滤波状态。
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} 必须是三个有限数")
        return vector

    @classmethod
    def _unit_vector(cls, value: Vector3, name: str) -> np.ndarray:
        # 推力轴只表示方向；去掉输入长度，零向量没有方向而必须拒绝。
        vector = cls._vector(value, name)
        norm = float(np.linalg.norm(vector))
        if norm < 1e-8:
            raise ValueError(f"{name} 不得为零向量")
        return vector / norm

    @staticmethod
    def _tangent_basis(thrust_direction: np.ndarray) -> np.ndarray:
        """返回单位推力轴在 ``S²`` 上的确定性二维正交切空间基。"""

        # 1. 选与 h 最不平行的坐标轴，避免投影后长度接近零。
        reference_axis = np.eye(3)[int(np.argmin(np.abs(thrust_direction)))]
        # 2. Gram-Schmidt：b1 = (e-h(h^T e))/||e-h(h^T e)||，故 b1^T h=0。
        first = reference_axis - thrust_direction * float(reference_axis @ thrust_direction)
        first /= float(np.linalg.norm(first))
        # 3. b2=h x b1；单位 h/b1 正交，所以 b2 也是单位向量且同时垂直二者。
        second = np.cross(thrust_direction, first)
        # 4. B=[b1,b2]，B^T B=I2，BB^T=I3-hh^T；以两维表达同一正交平面。
        # 基的符号/方向可随 h 改变，但各向同性 R 下不改变本帧的信息内容。
        return np.column_stack((first, second))


class MetricRelativeTargetEKF(RelativeTargetEKF):
    """九维滤波器：x = [r^T, v_rel^T, a_target^T]^T。

    r = p_target - p_observer，v_rel = v_target - v_observer，均在共享 ENU。
    g = (0, 0, -9.81)，h = R_WB_target e3；理想推力模型 a_target - g = lambda h。
    lambda 不作为状态估计；二维切空间观测消去推力大小，仅约束加速度方向。
    """

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
        # 复用基类的噪声设置、输入检查、切空间构造和数值保护；不用其尺度状态。
        super().__init__(position_std, attitude_constraint_std, position_process_std,
                         velocity_process_std, acceleration_process_std, fading=fading,
                         tilt_direction_std=tilt_direction_std)
        # 将基类的十维存储替换为九维；RGB-D 已提供米制位置，无需 alpha。
        self._x = np.zeros(9, dtype=np.float64)
        self._P = np.eye(9, dtype=np.float64) * 10.0

    @property
    def tuning(self) -> dict[str, float | None]:
        # 日志只保留九维版本真正使用的参数，移除尺度和独立范围量测参数。
        return {key: value for key, value in super().tuning.items()
                if key not in ("scale_process_std_m", "stereo_range_std_m")}

    def initialize(self, relative_position: Vector3, relative_velocity: Vector3 = (0.0, 0.0, 0.0)) -> None:
        """建立九维先验；无速度/加速度量测时以零为初值，而不是读取 GT。"""

        # r 来自首帧深度量测；可选 v_rel 只用于显式初始化，自动首帧默认零。
        self._x[:3] = self._vector(relative_position, "relative_position")
        self._x[3:6] = self._vector(relative_velocity, "relative_velocity")
        self._x[6:9] = 0.0
        # P0=10 I9 表示较宽松先验；各状态分量单位不同，数值 10 并非同一种物理量。
        self._P = np.eye(9, dtype=np.float64) * 10.0
        self._timestamp = None

    def update(
        self, measurement: MetricPoseMeasurement, observer_acceleration: Vector3
    ) -> MetricTargetEstimate:
        """处理一帧：校验 -> 初始化/预测 -> 位置 -> 二维姿态 -> 保护 -> 后验输出。

        不使用目标 GT 速度或加速度；两项可选量测缺失只跳过该项更新。
        本函数不判定姿态过期、图像同步或遮挡，这些必须由观测入口处理。
        """

        # x = [delta_p, delta_v, a_target] (共享 ENU), delta_p = p_target - p_tracker。
        # 可用时分别融合 z_p = delta_p + noise 和 B(h)^T a_target = B(h)^T g；
        # B(h) 是目标推力方向 h 的正交切空间基，缺失的量测跳过对应更新。
        # 步骤 1：校验三个输入向量；h 必须单位化，a_observer 是 ENU 线加速度。
        position = (None if measurement.relative_position is None else
                    self._vector(measurement.relative_position, "relative_position"))
        thrust = (None if measurement.target_thrust_direction is None else
                  self._unit_vector(measurement.target_thrust_direction, "target_thrust_direction"))
        acceleration = self._vector(observer_acceleration, "observer_acceleration")
        if not np.isfinite(measurement.timestamp):
            raise ValueError("timestamp 必须为有限数")
        # 步骤 2：首帧没有 dt，只初始化；后续先预测到当前量测时刻。
        if self._timestamp is None:
            if position is None:
                raise ValueError("首帧需要有效米制相对位置")
            self.initialize((float(position[0]), float(position[1]), float(position[2])))
        else:
            # 当前没有大 dt 截断；重复/倒退时间直接拒绝，不能把同一帧反复融合。
            dt = measurement.timestamp - self._timestamp
            if dt <= 0.0:
                raise ValueError("timestamp 必须严格递增")
            self._predict(dt, acceleration)
        if position is not None:
            # 步骤 3：米制位置观测直接修正 r，并经 P 的交叉块修正 v_rel/a_target。
            # z_r = r + n_r；H_r = [I3, 0, 0]，R_r = sigma_r^2 I3。
            observation = np.zeros((3, 9))
            observation[:, :3] = np.eye(3)
            self._kalman_update(observation, position, np.eye(3) * self._position_variance)
        if thrust is not None:
            # 步骤 4：姿态只约束 a_target-g 的切向分量，不能独立测出沿 h 的大小。
            # B 为 3x2 正交基：B^T h = 0，B^T B = I2。
            # 0 = B^T(a_target - g) + n_h，等价 z_h = B^T g，H_h = [0, 0, B^T]。
            # innovation_h = -B^T(a_target_hat - g)；只更新二维，不再叠加 projector。
            # R_h 表示加速度约束误差（包括阻力等模型误差），不是姿态角方差。
            basis = self._tangent_basis(thrust)
            observation = np.zeros((2, 9))
            observation[:, 6:9] = basis.T
            self._kalman_update(observation, basis.T @ _GRAVITY_ENU, self._tilt_measurement_noise())
        # 步骤 5：两项观测完成后清理 P、限制状态幅值，保存时间以供下一帧预测。
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
        # 1. innovation 是观测残差；position 时单位 m，tangent 时单位 m/s^2。
        innovation = value - observation @ self._x
        # 2. S 综合先验投影的不确定度和量测噪声，维度随量测为 3x3 或 2x2。
        covariance = observation @ self._P @ observation.T + noise
        # 3. 解 S (K^T) = H P；最后转置得到 9xm 增益，避免显式计算 S^-1。
        gain = np.linalg.solve(covariance, observation @ self._P).T
        # P 的 r-a、v-a 交叉块使姿态观测也能修正相对位置和速度。
        self._x += gain @ innovation
        # 4. A=I9-KH；Joseph 更新保留量测噪声经增益传播的 KRK^T 项。
        residual = np.eye(9) - gain @ observation
        self._P = residual @ self._P @ residual.T + gain @ noise @ gain.T

    def _clamp_state(self) -> None:
        """逐个三维状态块按范数限幅，仅防止异常数值扩散，不保证飞行安全。"""

        for block, limit in ((slice(0, 3), self._max_rel_dist),
                             (slice(3, 6), self._max_rel_speed),
                             (slice(6, 9), self._max_target_acc)):
            norm = float(np.linalg.norm(self._x[block]))
            if norm > limit:
                # 保持三维方向，缩到限制球面；没有同步更新协方差或执行额外量测。
                self._x[block] *= limit / norm

    def _predict(self, dt: float, observer_acceleration: np.ndarray) -> None:
        """常目标加速度预测；观察机运动作为已知输入从相对运动中扣除。"""

        # a_rel = a_target - a_tracker, delta_p+ = delta_p + delta_v dt + a_rel dt^2/2,
        # delta_v+ = delta_v + a_rel dt, a_target+ = a_target；P+ = lambda(F P F^T + Q)。
        # F 的 p-v、p-a、v-a 块分别是 dt I、dt^2 I/2、dt I。
        # x^- = F x^+ + G a_observer；G = [-dt^2 I/2; -dt I; 0]。
        # F = [[I, dt I, dt^2 I/2], [0, I, dt I], [0, 0, I]]。
        # 1. 两机绝对加速度之差才是相对加速度；不能把 a_target 当作 a_rel。
        relative_acceleration = self._x[6:9] - observer_acceleration
        # 2. 用旧 v_rel 更新位置，再更新速度；顺序保证不会多算一份 a_rel dt^2。
        self._x[:3] += self._x[3:6] * dt + 0.5 * relative_acceleration * dt**2
        self._x[3:6] += relative_acceleration * dt
        # 3. a_target 的均值本步保持不变，其变化由 Q 及后续量测更新容纳。
        # F 的耦合使 P_ra/P_va 随时间增长，姿态约束才能间接影响位置和速度。
        transition = np.eye(9)
        transition[:3, 3:6] = np.eye(3) * dt
        transition[:3, 6:9] = np.eye(3) * (0.5 * dt**2)
        transition[3:6, 6:9] = np.eye(3) * dt
        # Q = diag(sigma_p^2 I3, sigma_v^2 I3, sigma_a^2 I3)，当前每步固定、不随 dt 缩放。
        # fading 是协方差膨胀系数，与推力模型中的 lambda 无关。
        noise = np.diag([self._position_process_variance] * 3
                        + [self._velocity_process_variance] * 3
                        + [self._acceleration_process_variance] * 3)
        # 4. 传播不确定度并加入模型误差；未单独传播 a_observer 的测量协方差。
        self._P = (transition @ self._P @ transition.T + noise) * self._fading
        # 5. 预测后也做数值保护，防止异常先验进入量测更新的线性求解。
        self._sanitize_covariance()
        self._clamp_state()