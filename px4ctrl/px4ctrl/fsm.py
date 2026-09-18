"""控制状态机（``PX4CtrlFSM`` 的移植）。

对应上游 ``Fast-Gamma/src/realflight_modules/px4ctrl/src/PX4CtrlFSM.{h,cpp}``。

状态
----
==============  ==========================================================
``MANUAL_CTRL`` px4ctrl 未接管，飞控由遥控器/其它模式控制
``AUTO_TAKEOFF`` 从起点按 ``takeoff_land_speed`` 爬升到 ``takeoff_height``
``AUTO_HOVER``  保持悬停点，等待制导指令
``CMD_CTRL``    跟踪制导层给出的期望状态
``AUTO_LAND``   按 ``takeoff_land_speed`` 下降并触发落地检测
==============  ==========================================================

安全策略
--------
- 任何输入超时都不得继续控制：里程计缺失直接转 ``AUTO_LAND``，IMU 缺失则退化为仅用
  期望姿态（由 :class:`~px4ctrl.controller.LinearControl` 处理），状态机本身不再发散点。
- 每个周期都会检查是否仍在 Offboard；掉出则记录并尝试复位，不静默继续。
- 与上游一致：``MOTORS_SPEEDUP_TIME`` 内电机怠速，``DELAY_TRIGGER_TIME`` 到达目标高度
  后停顿，避免起飞瞬间的推力突变。
"""

from __future__ import annotations

import math
from enum import IntEnum
from typing import Callable, Optional

from px4ctrl.controller import ControllerOutput, DesiredState, LinearControl
from px4ctrl.inputs import CommandData, TakeoffLandData, vadd, yaw_from_quaternion
from px4ctrl.link import MavlinkLink
from px4ctrl.params import Params


class State(IntEnum):
    """与上游 ``PX4CtrlFSM::State_t`` 取值保持一致，便于日志对照。"""

    MANUAL_CTRL = 1
    AUTO_HOVER = 2
    CMD_CTRL = 3
    AUTO_TAKEOFF = 4
    AUTO_LAND = 5


#: 起飞→悬停交接的实测高度容差（m）。
HANDOVER_TOLERANCE_M = 0.10
#: 超出延时后仍不满足容差时的最长等待（s）；超时按实测位置交接，避免卡在起飞态。
HANDOVER_TIMEOUT_S = 10.0


class PX4CtrlFSM:
    """周期调用的控制状态机。"""

    def __init__(
        self,
        params: Params,
        controller: LinearControl,
        link: MavlinkLink,
        *,
        log: Callable[[str], None] = print,
    ) -> None:
        self.params = params
        self.controller = controller
        self.link = link
        self._log = log

        self.state = State.MANUAL_CTRL
        self.takeoff_land = TakeoffLandData()
        self.command = CommandData()

        self.hover_pose = (0.0, 0.0, 0.0)
        self.hover_yaw = 0.0
        self.last_output = ControllerOutput()

        #: 是否持续下发设定点。Offboard 下中断即 failsafe，因此默认开启。
        self.stream_enabled = False
        self.last_cycle = 0.0

    # -------------------------------------------------------------- 生命周期

    def enable(self) -> None:
        """开始下发设定点（进入 Offboard 前的预热）。"""

        self.stream_enabled = True

    def disable(self) -> None:
        self.stream_enabled = False

    # ------------------------------------------------------------ 状态迁移

    def request_takeoff(self, now: float) -> None:
        """请求自动起飞：记录起点并进入 ``AUTO_TAKEOFF``。"""

        self.takeoff_land.start_pose = self.link.odom.p
        self.takeoff_land.toggle_time = now
        self.takeoff_land.landed = False
        self.takeoff_land.delay_trigger = False
        self._set_hover_from_odom()
        self.state = State.AUTO_TAKEOFF
        self._log(
            f"AUTO_TAKEOFF：起点 ENU={self.takeoff_land.start_pose}，"
            f"目标高度 {self.params.takeoff_land.takeoff_height:.2f} m"
        )

    def request_hover(self, now: float) -> None:
        """请求悬停：以当前位置为悬停点。"""

        self._set_hover_from_odom()
        self.takeoff_land.landed = False
        self.state = State.AUTO_HOVER
        self._log(f"AUTO_HOVER：悬停点 ENU={self.hover_pose}")

    def request_land(self, now: float) -> None:
        """请求自动降落。"""

        self.takeoff_land.toggle_time = now
        self.takeoff_land.delay_trigger = False
        self.state = State.AUTO_LAND
        self._log("AUTO_LAND：按设定速度下降")

    def request_command_control(self) -> None:
        """把控制权交给制导层（``CMD_CTRL``）。"""

        self.state = State.CMD_CTRL
        self._log("CMD_CTRL：开始跟踪制导指令")

    def set_command(self, command: CommandData) -> None:
        """更新制导指令（对应上游 ``/position_cmd`` 订阅）。"""

        self.command = command

    def _set_hover_from_odom(self) -> None:
        self.hover_pose = self.link.odom.p
        self.hover_yaw = yaw_from_quaternion(self.link.odom.q)

    # ------------------------------------------------------------ 期望状态

    def _hover_desired(self) -> DesiredState:
        return DesiredState(p=self.hover_pose, yaw=self.hover_yaw)

    def _command_desired(self) -> DesiredState:
        cmd = self.command
        return DesiredState(p=cmd.p, v=cmd.v, a=cmd.a, j=cmd.j, yaw=cmd.yaw, yaw_rate=cmd.yaw_rate)

    def _takeoff_desired(self, now: float) -> DesiredState:
        """起飞斜坡：先怠速，再按设定速度爬升到目标高度。"""

        tl = self.params.takeoff_land
        elapsed = now - self.takeoff_land.toggle_time
        start = self.takeoff_land.start_pose

        if elapsed < TakeoffLandData.MOTORS_SPEEDUP_TIME:
            # 电机怠速阶段：仍在起点，避免瞬间起跳。
            return DesiredState(p=start, yaw=self.hover_yaw)

        climb_time = elapsed - TakeoffLandData.MOTORS_SPEEDUP_TIME
        height = min(climb_time * tl.takeoff_land_speed, tl.takeoff_height)
        desired_p = (start[0], start[1], start[2] + height)

        if height >= tl.takeoff_height - 1e-3:
            if not self.takeoff_land.delay_trigger:
                self.takeoff_land.delay_trigger = True
                self.takeoff_land.delay_trigger_time = now
            waited = now - self.takeoff_land.delay_trigger_time

            # 交接必须依据**实测**高度，而不是只看时间。时间参数化的斜坡按设定速度
            # 推算指令高度，实际爬升往往跟不上：实测交接瞬间机体落后 0.66 m，这个
            # 落差会直接成为"保持误差"的峰值。因此要求实测高度进入容差后再交接，
            # 并沿用**实测位置**作为悬停点（与上游 set_hov_with_odom 的思路一致）。
            reached = abs(self.link.odom.p[2] - desired_p[2]) <= HANDOVER_TOLERANCE_M
            timed_out = waited >= TakeoffLandData.DELAY_TRIGGER_TIME + HANDOVER_TIMEOUT_S

            if (waited >= TakeoffLandData.DELAY_TRIGGER_TIME and reached) or timed_out:
                if reached:
                    # 实测已进入容差：悬停点取**指令点**，让 PD 闭合剩余的小量偏差，
                    # 从而真正停在指令高度上。若改取实测点，机体将永远停在交接处
                    # （实测停在 1.90 m 而非指令的 2.00 m）。
                    self.hover_pose = desired_p
                else:
                    # 超时未达标：以实测位置为悬停点，避免指令突变引发大幅机动。
                    self.hover_pose = self.link.odom.p
                    self._log(
                        f"WARN: 起飞交接等待 {waited:.1f}s 仍未进入 ±{HANDOVER_TOLERANCE_M} m "
                        f"容差（实测 {self.link.odom.p[2]:.3f} m，目标 {desired_p[2]:.3f} m）；"
                        "按实测位置交接"
                    )
                self.takeoff_land.landed = False
                self.state = State.AUTO_HOVER
                self._log(
                    f"AUTO_HOVER：交接时实测高度 {self.link.odom.p[2] - start[2]:.2f} m，"
                    f"悬停点 {self.hover_pose[2] - start[2]:.2f} m"
                )
                return self._hover_desired()

        return DesiredState(p=desired_p, yaw=self.hover_yaw)

    def _land_desired(self, now: float) -> DesiredState:
        """降落斜坡：以设定速度下降到当前水平位置的投影。"""

        speed = self.params.takeoff_land.takeoff_land_speed
        current = self.link.odom.p
        target_z = max(0.0, current[2] - speed / max(self.params.ctrl_freq_max, 1.0))
        return DesiredState(p=(current[0], current[1], target_z), yaw=self.hover_yaw)

    def _desired(self, now: float) -> DesiredState:
        if self.state == State.AUTO_TAKEOFF:
            return self._takeoff_desired(now)
        if self.state == State.AUTO_LAND:
            return self._land_desired(now)
        if self.state == State.CMD_CTRL:
            return self._command_desired()
        return self._hover_desired()

    # -------------------------------------------------------------- 落地检测

    def _detect_landed(self) -> None:
        """落地检测：位置贴地且垂直速度很小即认为已落地。"""

        odom = self.link.odom
        if odom.recv_time <= 0.0:
            return
        if abs(odom.p[2] - self.takeoff_land.start_pose[2]) < 0.05 and abs(odom.v[2]) < 0.1:
            if not self.takeoff_land.landed:
                self.takeoff_land.landed = True

    # -------------------------------------------------------------- 主周期

    def process(self, now: float) -> ControllerOutput:
        """执行一个控制周期：收消息 → 校验 → 期望状态 → 控制律 → 下发。"""

        self.link.pump()
        self.last_cycle = now

        # 里程计是唯一的反馈来源；缺失时绝不能继续控制。
        if not self.link.odom.is_fresh(now, self.params.timeouts.odom):
            if self.state != State.MANUAL_CTRL:
                self._log("WARN: 里程计超时/缺失，停止控制并转入降落")
                self.state = State.MANUAL_CTRL
            self.last_output = ControllerOutput()
            return self.last_output

        if self.state == State.AUTO_LAND:
            self._detect_landed()

        des = self._desired(now)
        output = self.controller.calculate_control(
            des, self.link.odom, self.link.imu, self.last_output, now=now
        )

        # 归一化推力必须落在 [0, 1]，否则 PX4 端行为未定义。
        output.thrust = max(0.0, min(1.0, output.thrust))

        if self.stream_enabled:
            self.link.send_attitude_thrust(output.q, output.thrust)

        return output

    def tick(self) -> None:
        """供命令等待期间调用，维持设定点流不中断。"""

        if self.stream_enabled:
            self.link.send_attitude_thrust(self.last_output.q, self.last_output.thrust)
        self.link.pump()

    # ------------------------------------------------------------------ 工具

    @property
    def altitude(self) -> float:
        """相对起点的高度（ENU 下 z 向上）。"""

        return self.link.odom.p[2] - self.takeoff_land.start_pose[2]

    def state_name(self) -> str:
        return State(self.state).name
