"""飞控连接层：把 MAVLink 命令**统一包装**成一组受控操作。

设计目标（对应上游 px4ctrl 通过 MAVROS 承担的角色）
----------------------------------------------------
上层（状态机、制导、算法）**不允许**接触 MAVLink。本模块是唯一的出口，提供：

- 连接与身份校验（拒绝控制错误的无人机）；
- 包装好的命令：:meth:`MavlinkLink.arm` / :meth:`~MavlinkLink.disarm` /
  :meth:`~MavlinkLink.set_offboard` / :meth:`~MavlinkLink.land` /
  :meth:`~MavlinkLink.reboot`，每个都等待并仲裁 ACK；
- 包装好的设定点：:meth:`~MavlinkLink.send_attitude_thrust`（px4ctrl 的控制输出）与
  :meth:`~MavlinkLink.send_position_target`；
- 入站遥测到 :mod:`px4ctrl.inputs` 的映射，并在此完成 NED/FRD → ENU/FLU 的转换。

**仿真与真机的唯一差异是连接串**：仿真 ``udpin:0.0.0.0:14540``，真机
``serial:/dev/ttyACM0:921600`` 或 ``udpout:<飞控IP>:14550``。控制器与状态机无感知。

两个必须遵守的 PX4 细节（均已实测，见 README）
----------------------------------------------
1. ``MAV_CMD_DO_SET_MODE`` 在主模式/子模式上按**逐字节**解析：``param2``=主模式、
   ``param3``=子模式。沿旧版打包约定会被 ``(uint8_t)`` 截断为 0 并报
   ``Unsupported main mode``。
2. Offboard 要求设定点流 >2 Hz 持续下发，中断即触发 failsafe；因此本层的发送与接收
   在同一个循环里完成，绝不能阻塞在等待 ACK 上。
"""

from __future__ import annotations

import math
import time
from typing import Any

from pymavlink import mavutil

from px4ctrl.frames import (
    enu_flu_to_ned_frd,
    enu_to_ned_position,
    geodetic_to_enu,
    ned_frd_to_enu_flu,
    ned_to_enu_position,
)
from px4ctrl.inputs import (
    BatteryData,
    ExtendedStateData,
    ImuData,
    OdomData,
    Quaternion,
    StateData,
    Vector3,
)
from px4ctrl.params import LinkParams, SharedFrameParams
from px4ctrl.vehicle import VehicleRole

# PX4 自定义主模式（src/modules/commander/px4_custom_mode.h）。
PX4_MAIN_MODE_OFFBOARD = 6

# 本模块作为 GCS 侧客户端时的 MAVLink 源标识。
GCS_SOURCE_SYSTEM = 245
GCS_SOURCE_COMPONENT = 190

# 位置 + 偏航控制掩码：忽略速度、加速度与偏航速率。
_POSITION_YAW_IGNORE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# 姿态（四元数）+ 推力：忽略三轴机体角速率。
_ATTITUDE_ONLY_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)

# 机体角速率 + 推力：忽略姿态四元数。
_BODYRATE_ONLY_MASK = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE


def _pwm_to_normalized(value: float) -> float:
    """把 ``SERVO_OUTPUT_RAW`` 的取值统一成 [0, 1] 的归一化执行器指令。

    PX4 视执行器类型给出不同量纲：PWM 类为 1000~2000，OneShot/DShot 类为 -10000~10000
    的原始值，个别配置直接给 0~1。这里按量级判定，并把结果夹到 [0, 1]。
    """

    if abs(value) <= 1.0:
        normalized = value
    elif value > 100.0:
        normalized = (value - 1000.0) / 1000.0
    else:
        # OneShot 类原始值：以 10000 为满量程。
        normalized = (value + 10000.0) / 20000.0
    return max(0.0, min(1.0, normalized))


def _wrap_pi(angle: float) -> float:
    """把角度归一化到 ``(-pi, pi]``。"""

    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


class LinkError(RuntimeError):
    """连接、身份校验或命令仲裁失败时抛出，保证调用者得到非零退出码。"""


class MavlinkLink:
    """单个载具的 MAVLink 连接与包装命令。"""

    def __init__(
        self,
        params: LinkParams,
        role: VehicleRole,
        *,
        shared_frame: SharedFrameParams | None = None,
        ack_timeout: float = 5.0,
        log=print,
    ) -> None:
        self.params = params
        self.role = role
        self.ack_timeout = float(ack_timeout)
        self._log = log

        # 共享坐标系：启用后位置改由 GLOBAL_POSITION_INT 换算，使多机位置可比。
        self.shared_frame = shared_frame if shared_frame is not None else SharedFrameParams()
        self._use_shared_frame = bool(self.shared_frame.enabled)

        self.connection: Any = None
        self._boot_ms = 0.0
        self._acks: dict[int, dict[str, Any]] = {}

        # 入站遥测。
        self.state = StateData()
        self.extended_state = ExtendedStateData()
        self.odom = OdomData()
        self.imu = ImuData()
        self.battery = BatteryData()
        self.last_statustext = ""
        #: 最近一次 GPS 定位 (lat_deg, lon_deg, alt_m)，共享系换算的原始输入。
        self.global_position: tuple[float, float, float] | None = None
        #: 共享坐标系下的 ENU 位置，供**多机相对几何**使用（T3 制导）。
        #: 注意与 :attr:`odom` 的分工：``odom.p`` 是本机 local 系的 ENU，
        #: **控制与位置设定点必须用它**；``shared_position`` 只在比较两台载具时使用。
        #: 两者原点不同，混用会导致位置设定点整体偏移。
        self.shared_position: Vector3 | None = None
        #: PX4 下发的归一化执行器指令（HIL_ACTUATOR_CONTROLS），用于推力标定与饱和统计。
        self.actuators: list[float] = []
        self.actuators_time = 0.0

    # ------------------------------------------------------------ 连接管理

    @property
    def connection_string(self) -> str:
        """实际使用的连接串；未显式指定时回落到仿真端点。"""

        return self.params.connection or self.role.sim_connection

    def open(self, heartbeat_timeout: float | None = None) -> None:
        """建立连接并校验 system id，拒绝控制错误的无人机。"""

        timeout = self.params.heartbeat_timeout if heartbeat_timeout is None else heartbeat_timeout
        self._log(f"连接 {self.connection_string}（期望 system id {self.role.system_id}）")
        self.connection = mavutil.mavlink_connection(
            self.connection_string,
            source_system=GCS_SOURCE_SYSTEM,
            source_component=GCS_SOURCE_COMPONENT,
        )
        self._boot_ms = time.monotonic()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            heartbeat = self.connection.wait_heartbeat(timeout=1.0)
            if heartbeat is None:
                continue
            actual = int(self.connection.target_system)
            if actual != self.role.system_id:
                raise LinkError(
                    f"{self.connection_string} 收到 system id {actual}，期望 "
                    f"{self.role.system_id}；拒绝控制错误的无人机"
                )
            self._on_heartbeat(heartbeat)
            self._log(f"HEARTBEAT 就绪：system={actual}")
            return
        raise LinkError(f"在 {timeout:.1f}s 内未收到 {self.connection_string} 的 heartbeat")

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def __enter__(self) -> "MavlinkLink":
        self.open()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    # -------------------------------------------------------------- 消息泵

    def pump(self) -> None:
        """非阻塞排空消息队列，更新遥测与 ACK。必须与设定点流同频调用。"""

        if self.connection is None:
            return
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            kind = message.get_type()
            now = time.monotonic()
            if kind == "HEARTBEAT":
                self._on_heartbeat(message, now)
            elif kind == "EXTENDED_SYS_STATE":
                self.extended_state = ExtendedStateData(
                    recv_time=now,
                    landed_state=int(message.landed_state),
                    vtol_state=int(getattr(message, "vtol_state", 0)),
                )
            elif kind == "LOCAL_POSITION_NED":
                # 速度与位置同样需要 NED→ENU 转换。漏掉它会令控制器的 D 项恒为零，
                # 表现为阻尼缺失、超调明显，且难以从姿态上直接看出原因。
                self.odom.p = ned_to_enu_position(
                    (float(message.x), float(message.y), float(message.z))
                )
                self.odom.v = ned_to_enu_position(
                    (float(message.vx), float(message.vy), float(message.vz))
                )
                self.odom.recv_time = now
            elif kind == "GLOBAL_POSITION_INT":
                latitude = float(message.lat) * 1e-7
                longitude = float(message.lon) * 1e-7
                altitude = float(message.alt) * 1e-3
                self.global_position = (latitude, longitude, altitude)
                if self._use_shared_frame:
                    # 只写入 shared_position，**不**覆盖 odom.p：位置设定点由 PX4 在
                    # 本机 local 系解释，而共享系是另一套原点，两者不可混用。
                    self.shared_position = geodetic_to_enu(
                        latitude,
                        longitude,
                        altitude,
                        self.shared_frame.origin_latitude,
                        self.shared_frame.origin_longitude,
                        self.shared_frame.origin_altitude,
                        earth_radius=self.shared_frame.earth_radius,
                    )
            elif kind == "ATTITUDE_QUATERNION":
                # PX4 的 ATTITUDE_QUATERNION 为 NED/FRD；转换为控制器使用的 ENU/FLU。
                attitude = ned_frd_to_enu_flu(
                    (
                        float(message.q2),  # x
                        float(message.q3),  # y
                        float(message.q4),  # z
                        float(message.q1),  # w
                    )
                )
                self.odom.q = attitude
                self.odom.recv_time = now
                # IMU 姿态用于控制器的姿态补偿。仿真里与 odom.q 同源，补偿退化为恒等；
                # 真机上里程计来自 VIO 而 IMU 来自飞控，该项才真正起作用。
                self.imu.q = attitude
                if self.imu.recv_time <= 0.0:
                    self.imu.recv_time = now
            elif kind == "HIGHRES_IMU":
                # 机体 FRD → FLU。
                self.imu.acc = (
                    float(message.xacc),
                    -float(message.yacc),
                    -float(message.zacc),
                )
                self.imu.recv_time = now
            elif kind == "SYS_STATUS":
                self.battery = BatteryData(
                    recv_time=now, voltage=float(message.voltage_battery) / 1000.0
                )
            elif kind == "SERVO_OUTPUT_RAW":
                # onboard 链路上没有 HIL_ACTUATOR_CONTROLS（那只发往仿真链路），
                # 但 SERVO_OUTPUT_RAW 承载同一组执行器输出。
                self.actuators = [
                    _pwm_to_normalized(float(getattr(message, f"servo{index}_raw")))
                    for index in range(1, 5)
                ]
                self.actuators_time = now
            elif kind == "COMMAND_ACK":
                self._acks[int(message.command)] = {
                    "command": int(message.command),
                    "result": int(message.result),
                    "progress": int(getattr(message, "progress", 0)),
                    "result_param2": int(getattr(message, "result_param2", 0)),
                }
            elif kind == "STATUSTEXT":
                # onboard 实例默认不下发该消息；这里仅作尽力记录。
                self.last_statustext = message.text

    def _on_heartbeat(self, heartbeat: Any, now: float | None = None) -> None:
        stamp = time.monotonic() if now is None else now
        self.state = StateData(
            recv_time=stamp,
            connected=True,
            armed=bool(int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
            custom_mode=int(heartbeat.custom_mode),
        )

    # ------------------------------------------------------ 包装好的命令

    def send_command(self, command: int, *params: float) -> None:
        """发送 ``COMMAND_LONG``；ACK 由 :meth:`pump` 收集，因此本调用不会阻塞。"""

        if len(params) != 7:
            raise ValueError("MAVLink COMMAND_LONG 必须包含 7 个参数")
        if self.connection is None:
            raise LinkError("连接尚未建立")
        self._acks.pop(command, None)
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            command,
            0,
            *params,
        )

    def await_ack(self, command: int, name: str, tick=None) -> dict[str, Any]:
        """等待命令的**最终** ACK，只接受 ``ACCEPTED``。

        ``IN_PROGRESS`` 表示 PX4 仍在处理，必须继续等待而非判失败。``tick`` 用于在等待
        期间维持设定点流，避免 Offboard 因流中断触发 failsafe。
        """

        deadline = time.monotonic() + self.ack_timeout
        in_progress = mavutil.mavlink.MAV_RESULT_IN_PROGRESS
        while time.monotonic() < deadline:
            # 收包必须始终进行，否则 ACK 永远无法被收集。``tick`` 只额外负责维持
            # 设定点流（它内部同样会 pump），不能替代收包本身。
            if tick is not None:
                tick()
            else:
                self.pump()

            if command in self._acks:
                ack = self._acks.pop(command)
                if ack["result"] == in_progress:
                    self._log(f"{name} 处理中（progress={ack['progress']}）")
                elif ack["result"] != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise LinkError(f"{name} 被 PX4 拒绝: {ack}")
                else:
                    return ack
            time.sleep(0.005)
        raise LinkError(f"等待 {name} 的 ACK 超时（{self.ack_timeout:.1f}s）")

    def command_and_wait(self, command: int, name: str, *params: float, tick=None) -> dict[str, Any]:
        self.send_command(command, *params)
        return self.await_ack(command, name, tick=tick)

    def arm(self, *, tick=None) -> dict[str, Any]:
        """解锁。"""

        return self.command_and_wait(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            "arm_command",
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            tick=tick,
        )

    def disarm(self, *, force: bool = False, tick=None) -> dict[str, Any]:
        """上锁；``force=True`` 时使用 PX4 的强制上锁魔术参数 ``21196``。"""

        return self.command_and_wait(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            "disarm_command",
            0.0,
            21196.0 if force else 0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            tick=tick,
        )

    def set_offboard(self, *, tick=None) -> dict[str, Any]:
        """切换到 Offboard 主模式。

        ``param2``=主模式、``param3``=子模式：本工程使用的 PX4 分支按逐字节解析，
        不能沿用旧版把 ``(main<<16)|(sub<<8)`` 打包进 ``param2`` 的约定。
        """

        return self.command_and_wait(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            "set_offboard_mode",
            float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(PX4_MAIN_MODE_OFFBOARD),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            tick=tick,
        )

    def land(self, *, tick=None) -> dict[str, Any]:
        """请求自动降落（切到 AUTO_LAND）。"""

        nan = math.nan
        return self.command_and_wait(
            mavutil.mavlink.MAV_CMD_NAV_LAND, "land_command", nan, nan, nan, nan, nan, nan, nan, tick=tick
        )

    def reboot(self, *, tick=None) -> dict[str, Any]:
        """重启飞控（上游 ``reboot_FCU``）。"""

        return self.command_and_wait(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            "reboot_fcu",
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            tick=tick,
        )

    def is_offboard(self) -> bool:
        """由 HEARTBEAT 判断是否真的处于 Offboard。"""

        return ((self.state.custom_mode >> 16) & 0xFF) == PX4_MAIN_MODE_OFFBOARD

    def is_landed(self) -> bool:
        return self.extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

    @property
    def mean_actuator(self) -> float | None:
        """四个执行器指令的均值；未收到时返回 None。"""

        if not self.actuators:
            return None
        return sum(self.actuators) / len(self.actuators)

    # ------------------------------------------------------ 包装好的设定点

    def _time_boot_ms(self) -> int:
        return int((time.monotonic() - self._boot_ms) * 1000.0)

    def send_attitude_thrust(
        self,
        q_enu_flu: Quaternion,
        thrust: float,
        *,
        bodyrates: Vector3 | None = None,
    ) -> None:
        """发送姿态 + 推力设定点（px4ctrl 的控制输出）。

        ``q_enu_flu`` 为控制器输出的 ENU/FLU 姿态；在此转换为 PX4 需要的 NED/FRD。
        """

        if self.connection is None:
            return
        if bodyrates is None:
            q_ned_frd = enu_flu_to_ned_frd(q_enu_flu)
            # MAVLink 的 q 数组顺序为 (w, x, y, z)。
            q_msg = [q_ned_frd[3], q_ned_frd[0], q_ned_frd[1], q_ned_frd[2]]
            mask = _ATTITUDE_ONLY_MASK
            roll_rate = pitch_rate = yaw_rate = 0.0
        else:
            q_msg = [1.0, 0.0, 0.0, 0.0]
            mask = _BODYRATE_ONLY_MASK
            # 机体角速率为 FRD；仅翻转 y/z 分量。
            roll_rate = bodyrates[0]
            pitch_rate = -bodyrates[1]
            yaw_rate = -bodyrates[2]

        self.connection.mav.set_attitude_target_send(
            self._time_boot_ms(),
            self.connection.target_system,
            self.connection.target_component,
            mask,
            q_msg,
            roll_rate,
            pitch_rate,
            yaw_rate,
            float(thrust),
        )

    def send_position_target(self, p_enu: Vector3, yaw_enu: float = 0.0) -> None:
        """发送位置 + 偏航设定点（**位置与偏航均为世界系 ENU**，内部转 NED）。

        偏航必须一并转换：ENU 的偏航自东轴逆时针度量，NED 的偏航自北轴顺时针度量，
        两者相差 ``π/2 − yaw``。若把 ENU 偏航直接下发，飞机会平白转向。
        """

        if self.connection is None:
            return
        north, east, down = enu_to_ned_position(p_enu)
        yaw_ned = _wrap_pi(math.pi / 2.0 - yaw_enu)
        self.connection.mav.set_position_target_local_ned_send(
            self._time_boot_ms(),
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _POSITION_YAW_IGNORE_MASK,
            north,
            east,
            down,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_ned,
            0.0,
        )
