"""线性几何控制器（``LinearControl`` 的忠实移植）。
对应上游 ``Fast-Gamma/src/realflight_modules/px4ctrl/src/controller.cpp``。保留其算法与
命名（``Kp0/Kv0`` 等）以便与上游调参对照；坐标系与姿态补偿逻辑保持原样。

控制律
------
1. 期望加速度：``des_a = a_ff + Kv·(v_des − v) + Kp·(p_des − p)``，再加 +g 做重力补偿。
2. 归一化推力：``thrust = des_a_z / thr2acc``。
3. 由水平加速度分量反解倾角（roll/pitch），按当前偏航投影到机体系。
4. 期望姿态按 **Z-Y-X**（偏航-俯仰-横滚）合成。
5. 姿态补偿：``u.q = q_imu ⊗ q_odom⁻¹ ⊗ q_des``，抵消里程计与 IMU 的姿态偏差。
6. 用带遗忘因子的递推最小二乘在线估计 ``thr2acc``。

注意：本模块输出的 ``u.q`` 仍在 **ENU/FLU** 下（与上游一致，上游交由 MAVROS 转换）。
真正的 NED/FRD 转换由 :mod:`px4ctrl.frames` 在发送前完成。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Tuple

from px4ctrl.frames import quat_mul, quat_normalize
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

    #: 上游常量：低于该归一化推力时认为电机仍在怠速。
    MIN_NORMALIZED_COLLECTIVE_THRUST = 3.0

    def __init__(self, params: Params) -> None:
        self.params = params
        self.debug = DebugInfo()
        self._timed_thrust: Deque[Tuple[float, float]] = deque(maxlen=100)
        self._rho2 = 0.998
        self.reset_thrust_mapping()

    # ------------------------------------------------------------ 推力模型

    def reset_thrust_mapping(self) -> None:
        """按悬停推力百分比重置推力→加速度映射（对应 ``resetThrustMapping``）。"""

        self._thr2acc = self.params.gra / self.params.thrust_model.hover_percentage
        self._P = 1e6
        self.debug.thr2acc = self._thr2acc

    @property
    def thrust_to_accel(self) -> float:
        """当前推力→加速度映射系数（100% 推力对应的加速度）。"""

        return self._thr2acc

    def estimate_thrust_model(self, est_a: Vector3, now: float) -> bool:
        """用带遗忘因子的 RLS 在线更新 ``thr2acc``。

        取 35~45 ms 之前的推力指令与当前 z 加速度配对：这段延迟覆盖指令传输、电机响应
        与传感器滞后。返回是否完成了一次更新。
        """

        while self._timed_thrust:
            stamp, thrust = self._timed_thrust[0]
            elapsed = now - stamp
            if elapsed > 0.045:
                self._timed_thrust.popleft()
                continue
            if elapsed < 0.035:
                return False
            self._timed_thrust.popleft()

            gamma = 1.0 / (self._rho2 + thrust * self._P * thrust)
            gain = gamma * self._P * thrust
            self._thr2acc = self._thr2acc + gain * (est_a[2] - thrust * self._thr2acc)
            self._P = (1.0 - gain * thrust) * self._P / self._rho2
            self.debug.thr2acc = self._thr2acc
            return True
        return False

    # -------------------------------------------------------------- 控制律

    def _compute_collective_thrust(self, des_a: Vector3) -> float:
        """把期望 z 加速度换算成归一化推力信号。"""

        return des_a[2] / self._thr2acc

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

        # 步骤 2：归一化推力。
        out.thrust = self._compute_collective_thrust(des_a)

        # 步骤 3：按当前偏航把水平加速度投影到机体轴，反解倾角。
        yaw_odom = yaw_from_quaternion(odom.q)
        sin_yaw = math.sin(yaw_odom)
        cos_yaw = math.cos(yaw_odom)
        roll = (des_a[0] * sin_yaw - des_a[1] * cos_yaw) / self.params.gra
        pitch = (des_a[0] * cos_yaw + des_a[1] * sin_yaw) / self.params.gra

        # 倾角限幅（上游由 max_angle 参数表达；负值表示不限制）。
        limit = self.params.max_angle_rad
        if math.isfinite(limit):
            roll = max(-limit, min(limit, roll))
            pitch = max(-limit, min(limit, pitch))

        # 步骤 4：Z-Y-X 合成期望姿态。
        q_des = _quat_from_zyx(des.yaw, pitch, roll)

        # 步骤 5：姿态补偿，要求 q_imu 与 q_odom 均已收到；否则退化为直接使用期望姿态。
        if imu.recv_time > 0.0 and odom.recv_time > 0.0:
            from px4ctrl.frames import quat_conjugate

            out.q = quat_normalize(quat_mul(quat_mul(imu.q, quat_conjugate(odom.q)), q_des))
        else:
            out.q = q_des

        self.debug.des_a = des_a
        self.debug.des_v = des.v
        self.debug.roll = roll
        self.debug.pitch = pitch

        # 步骤 7：推力历史，供在线估计使用。
        self._timed_thrust.append((now, out.thrust))
        return out
