"""线性几何控制器（``LinearControl`` 的忠实移植）。
对应上游 ``Fast-Gamma/src/realflight_modules/px4ctrl/src/controller.cpp``。保留其算法与
命名（``Kp0/Kv0`` 等）以便与上游调参对照；坐标系与姿态补偿逻辑保持原样。

控制律
------
1. 期望加速度：``des_a = a_ff + Kv·(v_des − v) + Kp·(p_des − p)``，再加 +g 做重力补偿。
2. 由水平分量反解倾角：``roll = a_h^b_y / a_z``、``pitch = a_h^b_x / a_z``，除数是
   **含重力的垂向指令** ``des_a_z``，不是常数 g（只有悬停时两者相等，见
   :meth:`LinearControl._tilt_divisor`）。
3. 归一化推力：``thrust = des_a_z / (thr2acc · cosφ·cosθ)``，其中 ``cosφ·cosθ`` 是
   机体 z 轴在世界系的垂直分量，用于补掉倾斜造成的垂向推力损失。
4. 期望姿态按 **Z-Y-X**（偏航-俯仰-横滚）合成。
5. 姿态补偿：``u.q = q_imu ⊗ q_odom⁻¹ ⊗ q_des``，抵消里程计与 IMU 的姿态偏差。
6. 可选：用带遗忘因子的递推最小二乘在线估计 ``thr2acc``（油门/推力模型），
   由 ``thrust_model.online_estimate`` 控制，见 :meth:`LinearControl.estimate_thrust_model`。

注意：本模块输出的 ``u.q`` 仍在 **ENU/FLU** 下（与上游一致，上游交由 MAVROS 转换）。
真正的 NED/FRD 转换由 :mod:`px4ctrl.frames` 在发送前完成。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass, field
from typing import Deque, Tuple

from px4ctrl.frames import quat_conjugate, quat_mul, quat_normalize, quat_rotate
from px4ctrl.inputs import (
    ImuData,
    OdomData,
    Quaternion,
    Vector3,
    yaw_from_quaternion,
)
from px4ctrl.params import Params


@dataclass
class DesiredState:
    """期望状态（对应上游 ``Desired_State_t``）。全部为世界系 ENU。"""

    p: Vector3 = (0.0, 0.0, 0.0)
    v: Vector3 = (0.0, 0.0, 0.0)
    a: Vector3 = (0.0, 0.0, 0.0)
    j: Vector3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    yaw_rate: float = 0.0

    @classmethod
    def from_odom(cls, odom: OdomData, yaw: float | None = None) -> "DesiredState":
        """以当前里程计位置构造悬停期望（速度/加速度置零）。"""

        return cls(
            p=odom.p,
            yaw=yaw_from_quaternion(odom.q) if yaw is None else yaw,
        )


@dataclass
class ControllerOutput:
    """控制器输出（对应上游 ``Controller_Output_t``）。姿态为 ENU/FLU。"""

    q: Quaternion = (0.0, 0.0, 0.0, 1.0)
    bodyrates: Vector3 = (0.0, 0.0, 0.0)
    thrust: float = 0.0


@dataclass
class DebugInfo:
    """用于日志与离线分析的中间量。"""

    des_a: Vector3 = (0.0, 0.0, 0.0)
    des_v: Vector3 = (0.0, 0.0, 0.0)
    roll: float = 0.0
    pitch: float = 0.0
    thr2acc: float = 0.0
    #: 倾角反解实际使用的除数（垂向指令，异常时已被下限截断）。
    tilt_divisor: float = 0.0
    #: 推力使用的倾斜垂向分量补偿系数 ``cosφ·cosθ``（未启用补偿时恒为 1）。
    tilt_scale: float = 1.0
    #: 限幅前是否有任一倾角超出 ``max_angle``。这是 FSM 安全看门狗的唯一输入，
    #: 不能从限幅后的 roll/pitch 是否恰好等于上限反推。
    tilt_saturated: bool = False
    #: SO(3) 主值旋转误差 ``Log(R^T R_d)``，仅 body-rate 模式启用时更新。
    so3_rotation_error: Vector3 = (0.0, 0.0, 0.0)
    #: 机体系角速度误差 ``Omega - R^T R_d Omega_d``。
    so3_rate_error: Vector3 = (0.0, 0.0, 0.0)


@dataclass
class ThrustEstimateStats:
    """在线推力估计的运行统计（只用于诊断与日志，不参与控制）。"""

    #: 完成的有效 RLS 更新次数。
    updates: int = 0
    #: 因超出允许范围被拒绝的更新次数（IMU 符号/坐标系可疑或标定差一个量级）。
    rejected: int = 0
    #: 本周期无可配对样本的周期数（队首还太新，高频控制下的正常现象；
    #: 若持续增长说明控制周期大于配对窗口且不允许降级）。
    no_sample: int = 0
    #: 使用降级配对（窗口外最近样本）的次数。
    degraded: int = 0
    #: 观测到 ``est_a[2] < 0`` 的次数：机体 z 轴比力在飞行中不应持续为负。
    negative_accel: int = 0

    def as_dict(self) -> dict[str, int]:
        return asdict(self)


def _quat_from_zyx(yaw: float, pitch: float, roll: float) -> Quaternion:
    """按 Z-Y-X 顺序合成姿态四元数（返回 (x, y, z, w)）。"""

    half_yaw, half_pitch, half_roll = yaw / 2.0, pitch / 2.0, roll / 2.0
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cr, sr = math.cos(half_roll), math.sin(half_roll)

    # Rz(yaw) ⊗ Ry(pitch) ⊗ Rx(roll)
    qz = (0.0, 0.0, sy, cy)
    qy = (0.0, sp, 0.0, cp)
    qx = (sr, 0.0, 0.0, cr)
    return quat_normalize(quat_mul(quat_mul(qz, qy), qx))


class LinearControl:
    """位置→推力/姿态的几何控制器。"""

    #: 倾角反解除数的下限（重力倍数）：垂向指令接近 0 甚至为负时（大幅掉高、
    #: 强扰动）继续用 ``a_z`` 作除数会把噪声放大成满舵横向指令，因此截断到这个
    #: 下限。下限取很小值是有意的——它只防除零，不改变正常飞行时的倾角。
    MIN_TILT_DIVISOR_RATIO = 0.1

    def __init__(self, params: Params) -> None:
        self.params = params
        self.debug = DebugInfo()
        self._timed_thrust: Deque[Tuple[float, float]] = deque(maxlen=100)
        self._rho2 = 0.998
        #: 是否补偿倾斜造成的垂向推力损失（来自 YAML ``thrust_model.tilt_compensation``）。
        self._tilt_compensation = params.thrust_model.tilt_compensation
        #: 在线推力估计的运行统计，供状态机定期打印。
        self.estimate_stats = ThrustEstimateStats()
        #: 由相邻两次在线估计调用估出的控制周期（s）；用于日志与降级配对的界限。
        self.estimate_period_s = 0.0
        self._last_estimate_call: float | None = None
        self.reset_thrust_mapping()

    # ------------------------------------------------------------ 推力模型

    def reset_thrust_mapping(self) -> None:
        """按悬停推力百分比重置推力→加速度映射（对应 ``resetThrustMapping``）。"""

        initial = self.params.gra / self.params.thrust_model.hover_percentage
        self._thr2acc = initial
        # 允许范围以本次标定值为基准：RLS 只能在标定值附近修正，不能把模型带跑。
        self._thr2acc_bounds = (
            initial * self.params.thrust_model.estimate_min_ratio,
            initial * self.params.thrust_model.estimate_max_ratio,
        )
        self._P = 1e6
        self.debug.thr2acc = self._thr2acc

    @property
    def thrust_to_accel(self) -> float:
        """当前推力→加速度映射系数（100% 推力对应的加速度）。"""

        return self._thr2acc

    def suggested_hover_percentage(self) -> float:
        """由当前 ``thr2acc`` 反算的 `hover_percentage`。

        ``thr2acc = gra / hover_percentage``，因此在线估计收敛后可直接把它写回 YAML，
        把“在线值”固化为下一次起飞用的标定值（见 runbook 的油门模型标定步骤）。
        """

        return self.params.gra / self._thr2acc

    def estimate_thrust_model(self, est_a: Vector3, now: float) -> bool:
        """用带遗忘因子的递推最小二乘在线估计 ``thr2acc``（油门/推力模型）。

        模型与上游完全一致：``est_a[2] = thrust · thr2acc``，即“机体 z 轴加速度 =
        推力指令 × 推力系数”。

        对 ``est_a`` 的要求：必须是**机体 FLU 系的比力 z 分量**，也就是 IMU 原始加速度
        （:attr:`px4ctrl.link.MavlinkLink.imu` 的 ``acc``）。悬停时它约为 ``+g``。
        不能用世界系垂向加速度：那会在倾斜时多出 ``cos`` 因子，而机体 z 轴的比力在
        任何倾角下都等于推力产生的加速度，这正是上游选它的原因。

        与倾角补偿的关系：补偿（除以 ``cosφ·cosθ``）是**指令的一部分**，被控对象会
        忠实执行，因此 ``est_a[2] = 已发送指令 · thr2acc`` 在任意倾角下仍然成立，
        补偿不会给估计带来系统性偏差（两者可以同时开启）。

        配对：取 ``[estimate_delay_min_s, estimate_delay_max_s]`` 窗口内**最旧**的推力
        指令与当前加速度配对，覆盖执行器与传输滞后。若窗口内没有样本（控制周期过大），
        在 ``estimate_allow_degraded`` 为真时退化为“使用最近的可用样本”并计入
        :attr:`estimate_stats` 的 ``degraded``；否则跳过本周期。

        安全：越界更新被拒绝（保持上一有效值不变，``rejected`` 加一），避免 IMU 符号
        接错或标定差一个数量级时把推力模型拉跑。

        返回值：本周期是否完成了一次有效更新。
        """

        model = self.params.thrust_model
        window_min = model.estimate_delay_min_s
        window_max = model.estimate_delay_max_s

        # 实际控制周期由相邻两次调用估出：既是日志里最有用的一个数，也是降级配对的
        # 界限（控制周期越大，可接受的配对年龄就越宽）。
        if self._last_estimate_call is not None:
            self.estimate_period_s = max(0.0, now - self._last_estimate_call)
        self._last_estimate_call = now
        degraded_limit = max(window_max, 2.0 * self.estimate_period_s)

        # 丢弃过老样本；但保留不超过 degraded_limit 的最旧样本，供降级配对使用。
        while self._timed_thrust and (now - self._timed_thrust[0][0]) > degraded_limit:
            self._timed_thrust.popleft()

        if not self._timed_thrust:
            self.estimate_stats.no_sample += 1
            return False

        stamp, thrust = self._timed_thrust[0]
        age = now - stamp
        if age < window_min:
            # 队首还太新：本周期窗口内不可能有样本（高频控制下的正常现象）。
            self.estimate_stats.no_sample += 1
            return False

        degraded = age > window_max
        if degraded and not model.estimate_allow_degraded:
            # 控制周期大于窗口且不允许降级：不配对，但要把它计入 diagnostics。
            self.estimate_stats.no_sample += 1
            self._timed_thrust.popleft()
            return False
        self._timed_thrust.popleft()
        if degraded:
            self.estimate_stats.degraded += 1

        if est_a[2] < 0.0:
            # 机体 z 轴比力在飞行中不应持续为负；持续为负说明符号/坐标系接错。
            self.estimate_stats.negative_accel += 1

        # ---- 带遗忘因子的递推最小二乘（与上游同式）----
        gamma = 1.0 / (self._rho2 + thrust * self._P * thrust)
        gain = gamma * self._P * thrust
        updated = self._thr2acc + gain * (est_a[2] - thrust * self._thr2acc)

        low, high = self._thr2acc_bounds
        if not math.isfinite(updated) or not low <= updated <= high:
            # 保持上一有效值，P 不更新：下次仍从同一个状态重试，不会被带跑。
            self.estimate_stats.rejected += 1
            return False

        self._thr2acc = updated
        self._P = (1.0 - gain * thrust) * self._P / self._rho2
        self.debug.thr2acc = self._thr2acc
        self.estimate_stats.updates += 1
        return True

    # -------------------------------------------------------------- 控制律

    def _tilt_divisor(self, des_a_z: float) -> float:
        """pitch 反解的除数：垂向指令加速度，异常时截断到保守下限。

        为何不能是常数 g：几何控制器要求机体 z 轴对齐**总**期望加速度向量，精确解为

        .. math:: \\sin\\varphi = \\frac{a_x \\sin\\psi - a_y \\cos\\psi}{|a|}, \\quad
                  \\theta = \\operatorname{atan2}(a_x \\cos\\psi + a_y \\sin\\psi,\\ a_z)

        其中 :math:`a_z` 是含重力的垂向指令。只有悬停（``a_z == g``）时用 g 才正确。
        高度环活跃时 ``a_z = g + Kp_z·Δz + Kv_z·Δv_z``：实测 Δz 可达 1.3 m，对应
        ``a_z ∈ [3.3, 16.3]``，用 g 会让倾角差 0.34~1.66 倍，等效使水平环增益随垂向
        状态变化，并与高度环耦合（旧实现中 x–z 同步摆动的候选机制）。

        限幅只作用于 pitch：roll 的除数（总模长 ``|a|``）由 PD 输出确定，不会趋于 0。
        """

        floor = self.params.gra * self.MIN_TILT_DIVISOR_RATIO
        if not math.isfinite(des_a_z) or des_a_z < floor:
            return floor
        return des_a_z

    def _tilt_from_acceleration(
        self, des_a: Vector3, yaw: float, tilt_divisor: float
    ) -> tuple[float, float]:
        """由总期望加速度反解 roll/pitch（**精确解**，不是小角度近似）。

        几何要求是机体 z 轴对齐总期望加速度向量 ``a``（ENU，含重力）。设
        ``R = Rz(ψ)·Ry(θ)·Rx(φ)``，机体 z 轴在世界系为
        ``(cosψ·sinθcosφ + sinψ·sinφ, sinψ·sinθcosφ − cosψ·sinφ, cosθcosφ)``；
        令它等于 ``a/|a|`` 可得：

        .. math::

            \\sin\\varphi = \\frac{a_x \\sin\\psi - a_y \\cos\\psi}{|a|}, \\qquad
            \\tan\\theta = \\frac{a_x \\cos\\psi + a_y \\sin\\psi}{a_z}

        注意两式的分母不同：roll 用**总模长**、pitch 用**垂向分量**。这是 ZYX 欧拉角
        顺序的必然结果——纯 roll 时 ``a_y = −|a|·sinφ``，与 ``a_z`` 无关。

        与上游的差异：上游用线性形式 ``θ ≈ a_h/a_z``（把 tan 当成角度），会系统性多给
        倾角约 ``θ²/3``（25° 时约 6%），使实际水平加速度超出指令同样的比例。这里用精确
        形式，未限幅时实际加速度严格等于指令值（由单测的几何一致性用例钉住）。

        偏航用**实测值**（``odom``）而非 ``des.yaw``：倾角必须从当前姿态可达，并且
        机体 z 轴方向与偏航有关。代价是 ``des.yaw`` 与当前偏航不一致的过渡期间，实际
        加速度方向会随偏航一并旋转；本项目 ``des.yaw`` 恒为 0 且两机不偏航，故不构成
        问题。
        """

        magnitude = math.sqrt(des_a[0] ** 2 + des_a[1] ** 2 + des_a[2] ** 2)
        if magnitude <= 0.0 or not math.isfinite(magnitude):
            return 0.0, 0.0
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        # asin 只在浮点误差下可能略微越界，截断到定义域内。
        sin_roll = (des_a[0] * sin_yaw - des_a[1] * cos_yaw) / magnitude
        roll = math.asin(max(-1.0, min(1.0, sin_roll)))
        pitch = math.atan2(des_a[0] * cos_yaw + des_a[1] * sin_yaw, tilt_divisor)
        return roll, pitch

    def _effective_tilt_scale(self, geometric_scale: float) -> float:
        """实际生效的倾斜垂向补偿系数。

        ``geometric_scale`` 是机体 z 轴在世界系的垂直分量 ``cosφ·cosθ``。关闭补偿
        （``thrust_model.tilt_compensation=false``）时返回 1，使推力回到不补偿的形式。
        """

        if not self._tilt_compensation:
            return 1.0
        if not math.isfinite(geometric_scale) or geometric_scale <= 0.0:
            # 理论上 |roll|,|pitch| < 90°，cos 恒为正；此分支只防御非法输入。
            return 1.0
        return geometric_scale

    def _compute_collective_thrust(self, des_a: Vector3, tilt_scale: float = 1.0) -> float:
        """把期望 z 加速度换算成归一化推力信号。

        ``tilt_scale`` 是 :meth:`_effective_tilt_scale` 给出的**实际生效**系数：倾斜后
        推力沿机体 z 轴，其垂直分量只占 ``cosφ·cosθ``，因此不除以它就会在倾斜时缺失
        垂向推力——实测 25° 倾角缺 9.4%（约 0.9 m/s²），配合 Kp_z=5 会留下约 0.18 m 的
        稳态高度偏差。
        """

        return des_a[2] / (self._thr2acc * tilt_scale)

    @staticmethod
    def _so3_log(q: Quaternion) -> Vector3:
        """返回单位四元数所表示旋转的主值对数 ``Log(R)``。

        这里直接用 ``2*atan2(||q_v||, q_w)``，不使用小角度近似。四元数先翻到
        ``q_w >= 0`` 的半球，使结果为 ``[-pi, pi]`` 内的最短旋转向量；恰好 180 度时
        旋转轴由四元数向量部分决定。
        """

        x, y, z, w = quat_normalize(q)
        if w < 0.0:
            x, y, z, w = -x, -y, -z, -w
        axis_norm = math.sqrt(x * x + y * y + z * z)
        if axis_norm <= 1e-12:
            return (0.0, 0.0, 0.0)
        angle = 2.0 * math.atan2(axis_norm, max(0.0, w))
        scale = angle / axis_norm
        return (x * scale, y * scale, z * scale)

    def _so3_bodyrates(
        self, q_cmd: Quaternion, des: DesiredState, odom: OdomData, imu: ImuData
    ) -> Vector3:
        """以群对数姿态误差生成 FLU 机体系角速度设定点。

        ``q_error = q_current^* q_cmd`` 对应 ``R^T R_d``，其主值对数是从当前姿态转向
        期望姿态的精确旋转向量。偏航角速度前馈先在期望机体系表达，再运输到当前机体系；
        ``imu.w`` 是当前机体系角速度。PX4 跟踪输出角速度，继续承担内层力矩控制。
        """

        current_q = imu.q if imu.recv_time > 0.0 else odom.q
        q_error = quat_mul(quat_conjugate(current_q), q_cmd)
        rotation_error = self._so3_log(q_error)

        # yaw_rate 是世界竖直轴的角速度，旋转到期望机体系后才是 Omega_d。
        omega_d = quat_rotate(quat_conjugate(q_cmd), (0.0, 0.0, des.yaw_rate))
        omega_feedforward = quat_rotate(q_error, omega_d)
        omega = imu.w if imu.recv_time > 0.0 else (0.0, 0.0, 0.0)
        rate_error = tuple(omega[index] - omega_feedforward[index] for index in range(3))

        gains = (self.params.gain.kang_r, self.params.gain.kang_p, self.params.gain.kang_y)
        damping = self.params.so3.rate_damping
        limit = self.params.so3.max_bodyrate
        bodyrates = tuple(
            max(
                -limit,
                min(
                    limit,
                    omega_feedforward[index]
                    + gains[index] * rotation_error[index]
                    - damping * rate_error[index],
                ),
            )
            for index in range(3)
        )
        self.debug.so3_rotation_error = rotation_error
        self.debug.so3_rate_error = rate_error
        return bodyrates

    def calculate_control(
        self,
        des: DesiredState,
        odom: OdomData,
        imu: ImuData,
        out: ControllerOutput | None = None,
        now: float = 0.0,
    ) -> ControllerOutput:
        """由期望状态与观测量计算推力与期望姿态。"""

        out = ControllerOutput() if out is None else out
        gain = self.params.gain

        # 步骤 1：PD 控制律 + 重力补偿。
        des_a = (
            des.a[0] + gain.kv0 * (des.v[0] - odom.v[0]) + gain.kp0 * (des.p[0] - odom.p[0]),
            des.a[1] + gain.kv1 * (des.v[1] - odom.v[1]) + gain.kp1 * (des.p[1] - odom.p[1]),
            des.a[2] + gain.kv2 * (des.v[2] - odom.v[2]) + gain.kp2 * (des.p[2] - odom.p[2]),
        )
        des_a = (des_a[0], des_a[1], des_a[2] + self.params.gra)

        # 步骤 2：由总期望加速度反解倾角（含限幅）。
        tilt_divisor = self._tilt_divisor(des_a[2])
        roll, pitch = self._tilt_from_acceleration(des_a, yaw_from_quaternion(odom.q), tilt_divisor)

        # 倾角限幅（上游由 max_angle 参数表达；负值表示不限制）。
        limit = self.params.max_angle_rad
        tilt_saturated = False
        if math.isfinite(limit):
            tilt_saturated = abs(roll) > limit or abs(pitch) > limit
            roll = max(-limit, min(limit, roll))
            pitch = max(-limit, min(limit, pitch))

        # 步骤 3：归一化推力。补偿系数取机体 z 轴在世界系的垂直分量 cosφ·cosθ：
        # 由 ``Rz(ψ)Ry(θ)Rx(φ)·ẑ = (sinθ·cosφ, −sinφ, cosθ·cosφ)`` 可知该式对任意
        # 倾角**精确**成立，不是小角度近似。debug 里记的是实际生效值（关闭补偿时为 1）。
        tilt_scale = self._effective_tilt_scale(math.cos(roll) * math.cos(pitch))
        out.thrust = self._compute_collective_thrust(des_a, tilt_scale)

        # 步骤 4：Z-Y-X 合成期望姿态。
        q_des = _quat_from_zyx(des.yaw, pitch, roll)

        # 步骤 5：姿态补偿，要求 q_imu 与 q_odom 均已收到；否则退化为直接使用期望姿态。
        if imu.recv_time > 0.0 and odom.recv_time > 0.0:
            out.q = quat_normalize(quat_mul(quat_mul(imu.q, quat_conjugate(odom.q)), q_des))
        else:
            out.q = q_des

        if self.params.use_bodyrate_ctrl:
            out.bodyrates = self._so3_bodyrates(out.q, des, odom, imu)
        else:
            out.bodyrates = (0.0, 0.0, 0.0)

        self.debug.des_a = des_a
        self.debug.des_v = des.v
        self.debug.roll = roll
        self.debug.pitch = pitch
        self.debug.tilt_divisor = tilt_divisor
        self.debug.tilt_scale = tilt_scale
        self.debug.tilt_saturated = tilt_saturated

        # 步骤 7：推力历史，供在线估计使用。
        self._timed_thrust.append((now, out.thrust))
        return out
