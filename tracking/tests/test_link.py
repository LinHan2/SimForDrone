"""MAVLink 入站遥测映射的回归测试。"""

import unittest
from pathlib import Path

from pymavlink import mavutil

from px4ctrl.link import CommandRejectedError, MavlinkLink
from px4ctrl.params import load_params
from px4ctrl.vehicle import resolve_role

SIM_CONFIG = Path(__file__).resolve().parents[2] / "px4ctrl" / "config" / "sim.yaml"


class FakeHighresImu:
    xacc = 1.0
    yacc = 2.0
    zacc = 3.0
    xgyro = 0.4
    ygyro = -0.5
    zgyro = 0.6

    @staticmethod
    def get_type() -> str:
        return "HIGHRES_IMU"


class FakeConnection:
    def __init__(self, messages: list[object]) -> None:
        self._messages = iter(messages)

    def recv_match(self, *, blocking: bool) -> object | None:
        return next(self._messages, None)


class FakeCommandAck:
    command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    result = mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
    progress = 0
    result_param2 = 0

    @staticmethod
    def get_type() -> str:
        return "COMMAND_ACK"


class MavlinkInputTest(unittest.TestCase):
    def test_force_arm_is_enabled_only_in_simulation_profile(self) -> None:
        real_config = SIM_CONFIG.with_name("real.yaml")

        self.assertTrue(load_params(SIM_CONFIG).link.force_arm)
        self.assertFalse(load_params(real_config).link.force_arm)

    def test_highres_imu_converts_angular_rate_from_frd_to_flu(self) -> None:
        params = load_params(SIM_CONFIG)
        link = MavlinkLink(params.link, resolve_role("target"), log=lambda _: None)
        link.connection = FakeConnection([FakeHighresImu()])
        link.pump()
        self.assertEqual(link.imu.acc, (1.0, -2.0, -3.0))
        self.assertEqual(link.imu.w, (0.4, 0.5, -0.6))
        self.assertGreater(link.imu.recv_time, 0.0)

    def test_command_rejection_preserves_mavlink_result_for_retry_policy(self) -> None:
        params = load_params(SIM_CONFIG)
        link = MavlinkLink(params.link, resolve_role("target"), log=lambda _: None)
        link.connection = FakeConnection([FakeCommandAck()])

        with self.assertRaises(CommandRejectedError) as raised:
            link.await_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, "arm_command")

        self.assertEqual(raised.exception.result, mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
        self.assertEqual(raised.exception.command, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)


if __name__ == "__main__":
    unittest.main()