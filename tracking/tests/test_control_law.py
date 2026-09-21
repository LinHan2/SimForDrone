"""控制律与状态机的回归测试。

这两个模块在 ``px4ctrl`` 里，是唯一会直接改变飞行行为的地方，因此这里只测**可判定的
不变量**（除数取值、补偿系数、超时降级），不测具体数值曲线。
"""

import math
import unittest
from dataclasses import replace
from pathlib import Path

from px4ctrl.controller import DesiredState, LinearControl
from px4ctrl.fsm import PX4CtrlFSM, State, TILT_SATURATION_HOVER_DELAY_S
from px4ctrl.inputs import CommandData, ImuData, OdomData, quaternion_from_yaw
from px4ctrl.params import load_params

SIM_CONFIG = Path(__file__).resolve().parents[2] / "px4ctrl" / "config" / "sim.yaml"


class FakeLink:
    """只实现 PX4CtrlFSM 实际调用的接口，避免测试里打开真实 MAVLink 端口。"""

    def __init__(self, clock: float = 1000.0) -> None:
        self.odom = OdomData(recv_time=clock, q=quaternion_from_yaw(0.0))
        self.imu = ImuData(recv_time=clock)
        self.sent: list[tuple[tuple[float, float, float, float], float, tuple[float, float, float] | None]] = []

    def pump(self) -> None:
        """真实链路在这里收报文；测试里没有报文可收。"""

    def send_attitude_thrust(self, q, thrust, *, bodyrates=None) -> None:
        self.sent.append((q, thrust, bodyrates))


class ControlLawTest(unittest.TestCase):
    """倾角反解与推力归一化。"""

    def setUp(self) -> None:
        self.params = load_params(SIM_CONFIG)
        self.controller = LinearControl(self.params)
        self.odom = OdomData(recv_time=1.0, q=quaternion_from_yaw(0.0))

    def test_hover_has_zero_tilt_and_hover_thrust(self) -> None:
        output = self.controller.calculate_control(
            DesiredState(p=(0.0, 0.0, 0.0)), self.odom, ImuData(), now=1.0
        )

        self.assertAlmostEqual(self.controller.debug.roll, 0.0, places=12)
        self.assertAlmostEqual(self.controller.debug.pitch, 0.0, places=12)
        # 悬停时推力指令必须恰好等于标定的 hover_percentage。
        self.assertAlmostEqual(output.thrust, self.params.thrust_model.hover_percentage, places=12)

    def test_tilt_divisor_is_vertical_command_not_gravity(self) -> None:
        # 高度误差 0.8 m 使 a_z = g + Kp2·0.8 ≈ 13.8 m/s²，与 g 相差约 41%。
        self.controller.calculate_control(
            DesiredState(p=(0.3, 0.0, 0.8)), self.odom, ImuData(), now=1.0
        )

        a_x = self.params.gain.kp0 * 0.3
        a_z = self.params.gra + self.params.gain.kp2 * 0.8
        self.assertAlmostEqual(self.controller.debug.tilt_divisor, a_z, places=12)
        # 精确解为 atan2(a_h, a_z)，不是线性形式 a_h/a_z。
        self.assertAlmostEqual(self.controller.debug.pitch, math.atan2(a_x, a_z), places=12)
        self.assertAlmostEqual(self.controller.debug.roll, 0.0, places=12)
        # 旧实现除以常数 g：即使在未限幅的区间，两者也必须明确不同。
        self.assertGreater(abs(self.controller.debug.pitch - a_x / self.params.gra), 1e-3)

    def test_tilt_scales_with_vertical_state(self) -> None:
        # 同一个水平指令，在不同垂向指令下必须给出不同倾角——这正是旧实现的缺陷。
        self.controller.calculate_control(
            DesiredState(p=(0.3, 0.0, 0.0)), self.odom, ImuData(), now=1.0
        )
        level_pitch = self.controller.debug.pitch
        self.controller.calculate_control(
            DesiredState(p=(0.3, 0.0, 0.8)), self.odom, ImuData(), now=1.0
        )
        climbing_pitch = self.controller.debug.pitch

        self.assertLess(climbing_pitch, level_pitch)

    def test_thrust_direction_matches_desired_acceleration(self) -> None:
        """几何一致性：由倾角合成的机体 z 轴必须对齐总期望加速度方向。

        这就是倾角反解的**定义**，也是除以常数 g 会失败的根本原因——本用例在旧实现
        下会直接不成立。为避免限幅掩盖错误，这里只取未触限的指令。
        """

        cases = ((0.3, 0.0, 0.8), (0.3, -0.4, -0.5), (0.6, 0.6, 0.2))
        for yaw in (0.0, math.pi / 2.0):
            odom = OdomData(recv_time=1.0, q=quaternion_from_yaw(yaw))
            for desired_p in cases:
                self.controller.calculate_control(
                    DesiredState(p=desired_p), odom, ImuData(), now=1.0
                )

                des_a = self.controller.debug.des_a
                roll = self.controller.debug.roll
                pitch = self.controller.debug.pitch
                self.assertLess(abs(roll), self.params.max_angle_rad)
                self.assertLess(abs(pitch), self.params.max_angle_rad)

                # 机体 z 轴在机体系为 (0, 0, 1)，经 Rz(ψ)Ry(θ)Rx(φ) 后在机头系为
                # (sinθ·cosφ, −sinφ, cosθ·cosφ)，再按偏航旋到世界系。
                body_x, body_y, body_z = (
                    math.sin(pitch) * math.cos(roll),
                    -math.sin(roll),
                    math.cos(pitch) * math.cos(roll),
                )
                direction = (
                    body_x * math.cos(yaw) - body_y * math.sin(yaw),
                    body_x * math.sin(yaw) + body_y * math.cos(yaw),
                    body_z,
                )
                norm = math.sqrt(sum(component * component for component in des_a))
                for computed, expected in zip(direction, des_a):
                    self.assertAlmostEqual(computed, expected / norm, places=9)

    def test_thrust_compensates_tilt(self) -> None:
        output = self.controller.calculate_control(
            DesiredState(p=(0.3, 0.0, 0.8)), self.odom, ImuData(), now=1.0
        )

        roll = self.controller.debug.roll
        pitch = self.controller.debug.pitch
        scale = math.cos(roll) * math.cos(pitch)
        self.assertAlmostEqual(self.controller.debug.tilt_scale, scale, places=12)
        expected = self.controller.debug.des_a[2] / (self.controller.thrust_to_accel * scale)
        self.assertAlmostEqual(output.thrust, expected, places=12)
        # 不补偿会偏小：这正是 25° 倾角下约 9.4% 垂向推力缺口的来源。
        uncompensated = self.controller.debug.des_a[2] / self.controller.thrust_to_accel
        self.assertGreater(output.thrust, uncompensated)

    def test_tilt_compensation_can_be_disabled(self) -> None:
        params = replace(
            self.params,
            thrust_model=replace(self.params.thrust_model, tilt_compensation=False),
        )
        controller = LinearControl(params)
        output = controller.calculate_control(
            DesiredState(p=(0.3, 0.0, 0.8)), self.odom, ImuData(), now=1.0
        )

        self.assertAlmostEqual(controller.debug.tilt_scale, 1.0, places=12)
        self.assertAlmostEqual(
            output.thrust, controller.debug.des_a[2] / controller.thrust_to_accel, places=12
        )

    def test_negative_vertical_command_does_not_blow_up(self) -> None:
        # 高度误差 −5 m 使 a_z = g − 25 < 0：除数必须被截断到下限，倾角仍受限幅约束。
        self.controller.calculate_control(
            DesiredState(p=(0.5, 0.0, -5.0)), self.odom, ImuData(), now=1.0
        )

        floor = self.params.gra * LinearControl.MIN_TILT_DIVISOR_RATIO
        self.assertAlmostEqual(self.controller.debug.tilt_divisor, floor, places=12)
        self.assertTrue(math.isfinite(self.controller.debug.pitch))
        self.assertAlmostEqual(
            abs(self.controller.debug.pitch), self.params.max_angle_rad, places=12
        )

    def test_tilt_is_limited_by_max_angle(self) -> None:
        self.controller.calculate_control(
            DesiredState(p=(10.0, 0.0, 0.0)), self.odom, ImuData(), now=1.0
        )

        self.assertAlmostEqual(
            abs(self.controller.debug.pitch), self.params.max_angle_rad, places=12
        )
        self.assertTrue(self.controller.debug.tilt_saturated)

    def test_so3_bodyrate_uses_exact_large_rotation_error(self) -> None:
        params = replace(
            self.params,
            use_bodyrate_ctrl=True,
            gain=replace(self.params.gain, kang_r=1.0),
        )
        controller = LinearControl(params)
        angle = 1.2
        current_q = (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0))
        odom = OdomData(recv_time=1.0, q=current_q)
        imu = ImuData(recv_time=1.0, q=current_q)

        output = controller.calculate_control(DesiredState(), odom, imu, now=1.0)

        self.assertAlmostEqual(controller.debug.so3_rotation_error[0], -angle, places=12)
        self.assertAlmostEqual(output.bodyrates[0], -angle, places=12)
        self.assertEqual(output.bodyrates[1:], (0.0, 0.0))

    def test_so3_bodyrate_is_limited_before_sending(self) -> None:
        params = replace(
            self.params,
            use_bodyrate_ctrl=True,
            gain=replace(self.params.gain, kang_r=20.0),
            so3=replace(self.params.so3, max_bodyrate=1.0),
        )
        controller = LinearControl(params)
        angle = 1.2
        current_q = (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0))

        output = controller.calculate_control(
            DesiredState(),
            OdomData(recv_time=1.0, q=current_q),
            ImuData(recv_time=1.0, q=current_q),
            now=1.0,
        )

        self.assertAlmostEqual(output.bodyrates[0], -1.0, places=12)

    def test_so3_yaw_rate_feedback_damps_measured_rotation(self) -> None:
        params = replace(
            self.params,
            use_bodyrate_ctrl=True,
            gain=replace(self.params.gain, kang_y=1.0),
            so3=replace(self.params.so3, rate_damping=1.0, max_bodyrate=10.0),
        )
        controller = LinearControl(params)
        angle = 0.2
        current_q = quaternion_from_yaw(angle)
        output = controller.calculate_control(
            DesiredState(),
            OdomData(recv_time=1.0, q=current_q),
            ImuData(recv_time=1.0, q=current_q, w=(0.0, 0.0, -0.4)),
            now=1.0,
        )

        # e_R,z=-0.2，e_Omega,z=-0.4；阻尼应令指令为 -0.2-(-0.4)=+0.2 rad/s。
        self.assertAlmostEqual(controller.debug.so3_rate_error[2], -0.4, places=12)
        self.assertAlmostEqual(output.bodyrates[2], 0.2, places=12)


class CommandFreshnessTest(unittest.TestCase):
    """制导指令的时间戳与超时降级。"""

    def setUp(self) -> None:
        self.params = load_params(SIM_CONFIG)
        self.link = FakeLink(clock=1000.0)
        self.fsm = PX4CtrlFSM(
            self.params, LinearControl(self.params), self.link, log=lambda message: None
        )
        self.fsm.enable()

    def _advance(self, now: float) -> None:
        """把链路接收时间与状态机时钟一起推进，模拟正常的主循环。"""

        self.link.odom.recv_time = now
        self.link.imu.recv_time = now
        self.fsm.process(now)

    def test_set_command_stamps_receive_time(self) -> None:
        self.fsm.set_command(CommandData(p=(1.0, 0.0, 0.0)), now=1000.0)

        self.assertEqual(self.fsm.command.recv_time, 1000.0)

    def test_existing_receive_time_is_respected(self) -> None:
        self.fsm.set_command(CommandData(p=(1.0, 0.0, 0.0), recv_time=42.0), now=1000.0)

        self.assertEqual(self.fsm.command.recv_time, 42.0)

    def test_fresh_command_keeps_command_control(self) -> None:
        self.fsm.set_command(CommandData(p=(1.0, 0.0, 0.0)), now=1000.0)
        self.fsm.request_command_control()

        self._advance(1000.0 + self.params.timeouts.cmd * 0.5)

        self.assertEqual(self.fsm.state, State.CMD_CTRL)

    def test_stale_command_falls_back_to_hover(self) -> None:
        self.fsm.set_command(CommandData(p=(5.0, 0.0, 0.0)), now=1000.0)
        self.fsm.request_command_control()

        self._advance(1000.0 + self.params.timeouts.cmd + 0.1)

        self.assertEqual(self.fsm.state, State.AUTO_HOVER)
        # 降级后必须悬停在**当前**位置，而不是继续追那条陈旧设定点。
        self.assertEqual(self.fsm.hover_pose, self.link.odom.p)

    def test_command_control_without_command_never_flies_away(self) -> None:
        # 从未收到指令就进 CMD_CTRL：不得把默认的零位设定点当有效指令用。
        self.fsm.request_command_control()

        self._advance(1000.1)

        self.assertEqual(self.fsm.state, State.AUTO_HOVER)

    def test_short_tilt_saturation_does_not_trip_watchdog(self) -> None:
        self.fsm.set_command(CommandData(p=(10.0, 0.0, 0.0)), now=1000.0)
        self.fsm.request_command_control()

        self._advance(1000.0)
        self.fsm.set_command(CommandData(p=(10.0, 0.0, 0.0)), now=1000.2)
        self._advance(1000.2)

        self.assertTrue(self.fsm.controller.debug.tilt_saturated)
        self.assertEqual(self.fsm.state, State.CMD_CTRL)

    def test_sustained_tilt_saturation_falls_back_to_hover(self) -> None:
        self.fsm.set_command(CommandData(p=(10.0, 0.0, 0.0)), now=1000.0)
        self.fsm.request_command_control()

        self._advance(1000.0)
        trip_time = 1000.0 + TILT_SATURATION_HOVER_DELAY_S
        self.fsm.set_command(CommandData(p=(10.0, 0.0, 0.0)), now=trip_time)
        self._advance(trip_time)

        self.assertEqual(self.fsm.state, State.AUTO_HOVER)
        self.assertEqual(self.fsm.hover_pose, self.link.odom.p)
        self.assertFalse(self.fsm.controller.debug.tilt_saturated)

    def test_so3_mode_sends_bodyrate_setpoints(self) -> None:
        self.fsm.params = replace(self.params, use_bodyrate_ctrl=True)
        self.fsm.controller.params = self.fsm.params
        self.fsm.request_hover(1000.0)

        self._advance(1000.0)

        self.assertIsNotNone(self.link.sent[-1][2])


if __name__ == "__main__":
    unittest.main()
