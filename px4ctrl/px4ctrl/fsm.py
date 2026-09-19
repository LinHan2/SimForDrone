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
- 任何输入超时都不得继续控制：里程计缺失直接退出控制，制导指令停更则退出
  ``CMD_CTRL`` 回到悬停（而不是带着最后一条陈旧设定点一直飞），IMU 缺失则退化为
  仅用期望姿态（由 :class:`~px4ctrl.controller.LinearControl` 处理）。
- 每个周期都会检查是否仍在 Offboard；掉出则记录并尝试复位，不静默继续。
- 与上游一致：``MOTORS_SPEEDUP_TIME`` 内电机怆速，``DELAY_TRIGGER_TIME`` 到达目标高度
  后停顿，避免起飞瞬间的推力突变。
"""

from __future__ import annotations

import math
import time
from dataclasses import replace
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
#: 推力模型日志的打印周期（s），仅当 ``thrust_model.print_value`` 为真时生效。
THRUST_LOG_PERIOD_S = 5.0
#: 判定“IMU 加速度符号/坐标系可疑”所需的最少负值样本数。
NEGATIVE_ACCEL_WARN_SAMPLES = 20
#: CMD_CTRL 的倾角持续受限超过该时长即回退悬停。一个正常的短暂限幅不触发；持续
#: 饱和意味着当前位置环无法实现制导加速度，继续追踪只会累积位置误差并放大风险。
TILT_SATURATION_HOVER_DELAY_S = 0.5


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

        # 在线推力估计与推力模型日志的一次性告警/节流状态。
        self._thrust_imu_warned = False
        self._thrust_sign_warned = False
        self._thrust_log_due = 0.0
        self._tilt_saturation_started_at: float | None = None

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

        self._tilt_saturation_started_at = None
        self.state = State.CMD_CTRL
        self._log("CMD_CTRL：开始跟踪制导指令")

    def set_command(self, command: CommandData, now: float | None = None) -> None:
        """更新制导指令（对应上游 ``/position_cmd`` 订阅）。

        到达时刻**在本函数内统一盖章**：每个制导调用点都必须让状态机知道“这条指令
        是什么时候到的”，靠调用方各自传 ``recv_time`` 早晚会漏（实测有两处漏传，
        导致 ``MSG_TIMEOUT_CMD`` 形同虚设）。外部已给同进程时钟的 ``recv_time`` 时
        尊重它，否则补当前单调时钟。
        """

        if command.recv_time <= 0.0:
            stamp = time.monotonic() if now is None else now
            command = replace(command, recv_time=stamp)
        self.command = command

    def _set_hover_from_odom(self) -> None:
        self.hover_pose = self.link.odom.p
        self.hover_yaw = yaw_from_quaternion(self.link.odom.q)

    # ------------------------------------------------------------ 期望状态

    def _hover_desired(self) -> DesiredState:
        return DesiredState(p=self.hover_pose, yaw=self.hover_yaw)

    def _command_desired(self) -> DesiredState:
        """制导层的期望状态。

        ``j``/``yaw_rate`` 只做透传（与上游一致）：当前控制律不使用加加速度，
        ``yaw_rate`` 仅在 ``use_bodyrate_ctrl`` 的角速度模式下才会被使用，而本移植
        只用姿态+推力设定点。保留它们是为了让制导层的接口契约不变。
        """

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

        # 制导指令新鲜度：CMD_CTRL 完全依赖制导层，指令停更时继续飞下去就会永久
        # 用同一条陈旧设定点（制导进程崩溃、卡死或减速都会触发）。降级为悬停而不是
        # 立即降落：跟踪任务被中断不等于飞行器应当立即落地。
        if self.state == State.CMD_CTRL and not self.command.is_fresh(now, self.params.timeouts.cmd):
            age = now - self.command.recv_time if self.command.recv_time > 0.0 else float("inf")
            self._log(
                f"WARN: 制导指令超时 {age:.3f}s（门限 {self.params.timeouts.cmd:.3f}s），"
                "转 AUTO_HOVER 悬停"
            )
            self.request_hover(now)

        # 在线推力模型估计（上游在 AUTO_HOVER/CMD_CTRL 每周期调用）。必须放在
        # calculate_control 之前：它要用上一周期存入的推力历史与本周期的加速度配对。
        if self.params.thrust_model.online_estimate and self.state in (
            State.AUTO_HOVER,
            State.CMD_CTRL,
        ):
            self._update_thrust_estimate(now)

        # 推力模型日志：``print_value`` 为真时周期打印（真机标定时用它看收敛）。
        if self.params.thrust_model.print_value:
            self._log_thrust_model(now)

        if self.state == State.AUTO_LAND:
            self._detect_landed()

        des = self._desired(now)
        output = self.controller.calculate_control(
            des, self.link.odom, self.link.imu, self.last_output, now=now
        )

        # 只监测制导控制：起飞/降落阶段可能因任务本身短暂接近上限，不能把它们误判为
        # 制导故障。控制器提供的是限幅**前**的显式标记，避免“恰好等于上限”的误判。
        if self.state == State.CMD_CTRL and self.controller.debug.tilt_saturated:
            if self._tilt_saturation_started_at is None:
                self._tilt_saturation_started_at = now
            elif now - self._tilt_saturation_started_at >= TILT_SATURATION_HOVER_DELAY_S:
                duration = now - self._tilt_saturation_started_at
                self._log(
                    f"WARN: 倾角持续饱和 {duration:.3f}s（阈值 "
                    f"{TILT_SATURATION_HOVER_DELAY_S:.3f}s），转 AUTO_HOVER 悬停"
                )
                self.request_hover(now)
                self._tilt_saturation_started_at = None
                # 本周期也必须立即改发悬停输出，不能再多发一次已饱和的制导设定点。
                output = self.controller.calculate_control(
                    self._hover_desired(), self.link.odom, self.link.imu, output, now=now
                )
        else:
            self._tilt_saturation_started_at = None

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

    # -------------------------------------------------------------- 推力模型

    def _update_thrust_estimate(self, now: float) -> None:
        """按配置调用在线推力估计，并对缺少 IMU / 符号可疑给出一次性告警。"""

        if self.link.imu.recv_time <= 0.0:
            # 没有 HIGHRES_IMU 就无法估计：显式告警，而不是静默不工作。
            if not self._thrust_imu_warned:
                self._thrust_imu_warned = True
                self._log("WARN: 未收到 HIGHRES_IMU，在线推力估计无法进行（检查链路报文流）")
            return

        self.controller.estimate_thrust_model(self.link.imu.acc, now)
        stats = self.controller.estimate_stats
        if stats.negative_accel >= NEGATIVE_ACCEL_WARN_SAMPLES and not self._thrust_sign_warned:
            self._thrust_sign_warned = True
            self._log(
                "WARN: 机体 z 轴比力持续为负（"
                f"{stats.negative_accel} 次）：IMU 加速度的符号/坐标系可能接错，"
                "应为 FLU 的比力（悬停时约为 +g）"
            )

    def _log_thrust_model(self, now: float) -> None:
        """周期打印推力模型与其估计统计（仅 ``thrust_model.print_value`` 打开时）。

        上游把这两个开关的语义留得很模糊（``print_value`` 实际上只会在加载时提醒你
        “该关掉”）；这里给它一个明确作用：真机标定时用它观察 ``thr2acc`` 的收敛过程。
        """

        if now < self._thrust_log_due:
            return
        self._thrust_log_due = now + THRUST_LOG_PERIOD_S
        stats = self.controller.estimate_stats
        initial = self.params.gra / self.params.thrust_model.hover_percentage
        online = "开" if self.params.thrust_model.online_estimate else "关"
        self._log(
            f"[thrust-model] thr2acc={self.controller.thrust_to_accel:.3f} "
            f"(标定初值 {initial:.3f}, hover_percentage 反算 "
            f"{self.controller.suggested_hover_percentage():.4f}) 在线估计={online} "
            f"更新={stats.updates} 拒绝={stats.rejected} 空窗={stats.no_sample} "
            f"降级={stats.degraded} 控制周期={self.controller.estimate_period_s * 1000.0:.1f}ms"
        )

    # ------------------------------------------------------------------ 工具

    @property
    def altitude(self) -> float:
        """相对起点的高度（ENU 下 z 向上）。"""

        return self.link.odom.p[2] - self.takeoff_land.start_pose[2]

    def state_name(self) -> str:
        return State(self.state).name
