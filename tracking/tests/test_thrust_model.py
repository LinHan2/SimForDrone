"""在线推力模型（RLS 估计 thr2acc）的回归测试。

为什么用合成被控对象而不是实飞：模型 ``est_a[2] = thrust · thr2acc`` 是**线性单参数**
问题，因此可以构造真值已知的被控对象，直接检验估计器是否收敛到真值。这样在一次单测里
就能覆盖“输入符号接反”“配对窗口不可达”“越界不设防”这三类实飞上最难查、代价最大的失效。
"""

import random
import unittest
from dataclasses import replace
from pathlib import Path

from px4ctrl.controller import DesiredState, LinearControl
from px4ctrl.fsm import PX4CtrlFSM, State
from px4ctrl.inputs import CommandData, ImuData, OdomData, quaternion_from_yaw
from px4ctrl.params import ParamError, load_params

SIM_CONFIG = Path(__file__).resolve().parents[2] / "px4ctrl" / "config" / "sim.yaml"
HOVER_P = (0.0, 0.0, 0.0)
#: 悬停时 IMU 测到的机体 z 轴比力（FLU，向上为正）。
HOVER_SPECIFIC_FORCE_Z = 9.81


def params_with_estimate(**overrides):
    """取仿真参数并打开在线估计（可再覆盖推力模型字段）。"""

    model = replace(load_params(SIM_CONFIG).thrust_model, online_estimate=True, **overrides)
    return replace(load_params(SIM_CONFIG), thrust_model=model)


def simulate(controller, *, true_thr2acc, rate_hz, seconds, noise_std=0.0, seed=0):
    """合成被控对象：真实机体 z 轴加速度 = 已发送推力指令 × 真值系数。

    调用顺序**与状态机一致**（先估计、再用上一周期已发送的推力计算新指令），否则配对
    年龄会一直是 0，估计器永远得不到样本。
    """

    random_source = random.Random(seed)
    odom = OdomData(recv_time=1.0, p=HOVER_P, q=quaternion_from_yaw(0.0))
    dt = 1.0 / rate_hz
    thrust_sent = controller.thrust_to_accel and (
        controller.params.gra / controller.params.thrust_model.hover_percentage
    )
    for step in range(int(seconds * rate_hz)):
        now = 1.0 + step * dt
        measured = thrust_sent * true_thr2acc
        if noise_std > 0.0:
            measured *= 1.0 + random_source.gauss(0.0, noise_std)
        controller.estimate_thrust_model((0.0, 0.0, measured), now)
        output = controller.calculate_control(
            DesiredState(p=HOVER_P), odom, ImuData(recv_time=now), now=now
        )
        thrust_sent = output.thrust
    return controller


class ThrustModelEstimationTest(unittest.TestCase):
    def test_converges_to_known_ratio_at_50hz(self) -> None:
        controller = LinearControl(params_with_estimate())
        initial = controller.thrust_to_accel
        # 真值取标定值的 1.3 倍：模拟“真机油门比仿真更强”的常见情形。
        true_value = initial * 1.3

        simulate(controller, true_thr2acc=true_value, rate_hz=50.0, seconds=3.0)

        self.assertAlmostEqual(controller.thrust_to_accel, true_value, delta=true_value * 0.01)
        self.assertGreater(controller.estimate_stats.updates, 50)
        # 50 Hz 周期 20 ms < 窗口下限，应严格按窗口配对，不进入降级。
        self.assertEqual(controller.estimate_stats.degraded, 0)

    def test_converges_with_degraded_pairing_at_20hz(self) -> None:
        # 20 Hz（50 ms 周期）大于窗口上界 45 ms：这是仿真的实际配置，
        # 也是“窗口不可达时不能静默失效”的典型场景。
        controller = LinearControl(params_with_estimate())
        true_value = controller.thrust_to_accel * 0.8

        simulate(controller, true_thr2acc=true_value, rate_hz=20.0, seconds=5.0)

        self.assertAlmostEqual(controller.thrust_to_accel, true_value, delta=true_value * 0.01)
        self.assertGreater(controller.estimate_stats.degraded, 10)
        self.assertAlmostEqual(controller.estimate_stats.degraded, controller.estimate_stats.updates)

    def test_converges_under_measurement_noise(self) -> None:
        controller = LinearControl(params_with_estimate())
        true_value = controller.thrust_to_accel * 1.2

        simulate(
            controller, true_thr2acc=true_value, rate_hz=50.0, seconds=8.0, noise_std=0.02, seed=3
        )

        self.assertAlmostEqual(controller.thrust_to_accel, true_value, delta=true_value * 0.1)

    def test_sign_inversion_is_detected_and_never_accepted(self) -> None:
        # IMU 加速度符号接反（例如把 NED 当成 FLU 用）：估计必须拒绝，且不能把
        # 推力模型带跑——这是实机上最容易犯且最难查的错。
        controller = LinearControl(params_with_estimate())
        initial = controller.thrust_to_accel

        simulate(controller, true_thr2acc=-initial, rate_hz=50.0, seconds=2.0)

        self.assertEqual(controller.thrust_to_accel, initial)
        self.assertGreater(controller.estimate_stats.rejected, 0)
        self.assertGreater(controller.estimate_stats.negative_accel, 0)

    def test_out_of_range_truth_is_rejected_not_tracked(self) -> None:
        # 真值超出允许范围（标定差一个数量级）：拒绝并计数，而不是悄悄跟随。
        controller = LinearControl(params_with_estimate())
        initial = controller.thrust_to_accel

        simulate(controller, true_thr2acc=initial * 5.0, rate_hz=50.0, seconds=2.0)

        self.assertEqual(controller.thrust_to_accel, initial)
        self.assertGreater(controller.estimate_stats.rejected, 0)
        low, high = initial * 0.5, initial * 2.0
        self.assertTrue(low <= controller.thrust_to_accel <= high)

    def test_no_sample_when_history_is_empty(self) -> None:
        controller = LinearControl(params_with_estimate())

        self.assertFalse(
            controller.estimate_thrust_model((0.0, 0.0, HOVER_SPECIFIC_FORCE_Z), 5.0)
        )
        self.assertEqual(controller.estimate_stats.no_sample, 1)
        self.assertEqual(controller.estimate_stats.updates, 0)

    def test_disabling_degraded_pairing_blocks_slow_loops(self) -> None:
        controller = LinearControl(params_with_estimate(estimate_allow_degraded=False))

        simulate(controller, true_thr2acc=controller.thrust_to_accel * 1.3, rate_hz=20.0, seconds=2.0)

        self.assertEqual(controller.estimate_stats.updates, 0)
        self.assertGreater(controller.estimate_stats.no_sample, 0)

    def test_suggested_hover_percentage_round_trip(self) -> None:
        controller = LinearControl(params_with_estimate())

        # thr2acc = gra / hover_percentage，因此反算必须回到 YAML 里的值。
        self.assertAlmostEqual(
            controller.suggested_hover_percentage(),
            controller.params.thrust_model.hover_percentage,
            places=12,
        )

    def test_estimate_period_tracks_call_rate(self) -> None:
        controller = LinearControl(params_with_estimate())

        simulate(controller, true_thr2acc=controller.thrust_to_accel, rate_hz=50.0, seconds=0.5)

        self.assertAlmostEqual(controller.estimate_period_s, 0.02, delta=0.002)

    def test_invalid_window_and_bounds_are_rejected_at_load(self) -> None:
        with self.assertRaises(ParamError):
            params_with_estimate(estimate_delay_min_s=0.05, estimate_delay_max_s=0.04)
        with self.assertRaises(ParamError):
            params_with_estimate(estimate_delay_min_s=0.0)
        with self.assertRaises(ParamError):
            params_with_estimate(estimate_min_ratio=3.0, estimate_max_ratio=2.0)
        with self.assertRaises(ParamError):
            params_with_estimate(hover_percentage=0.0)


class FakeLink:
    """只实现 PX4CtrlFSM 实际调用的接口。"""

    def __init__(self) -> None:
        self.odom = OdomData(recv_time=1000.0, p=HOVER_P, q=quaternion_from_yaw(0.0))
        self.imu = ImuData(recv_time=1000.0, acc=(0.0, 0.0, HOVER_SPECIFIC_FORCE_Z))
        self.sent: list[tuple[tuple[float, float, float, float], float]] = []

    def pump(self) -> None:
        """测试里没有报文可收。"""

    def send_attitude_thrust(self, q, thrust, *, bodyrates=None) -> None:
        self.sent.append((q, thrust))


class FsmEstimateWiringTest(unittest.TestCase):
    """状态机是否真的按开关调用估计器。"""

    def _build(self, params):
        self.link = FakeLink()
        fsm = PX4CtrlFSM(params, LinearControl(params), self.link, log=lambda message: None)
        fsm.enable()
        fsm.request_hover(1000.0)
        fsm.request_command_control()
        return fsm

    def _run_cycles(self, fsm, cycles: int = 60, period: float = 0.02) -> None:
        for index in range(cycles):
            now = 1000.0 + index * period
            self.link.odom.recv_time = now
            self.link.imu.recv_time = now
            fsm.set_command(CommandData(p=HOVER_P), now=now)
            fsm.process(now)

    def test_disabled_by_default_never_touches_the_model(self) -> None:
        fsm = self._build(load_params(SIM_CONFIG))

        self._run_cycles(fsm)

        self.assertEqual(fsm.controller.estimate_stats.updates, 0)
        self.assertFalse(fsm.params.thrust_model.online_estimate)

    def test_enabled_runs_the_estimator_every_cycle(self) -> None:
        fsm = self._build(params_with_estimate())

        self._run_cycles(fsm)

        self.assertGreater(fsm.controller.estimate_stats.updates, 10)
        self.assertAlmostEqual(fsm.controller.estimate_period_s, 0.02, delta=0.002)

    def test_estimator_is_not_run_in_takeoff_or_land_states(self) -> None:
        # 上游只在 AUTO_HOVER/CMD_CTRL 估计：起飞与降落阶段推力与加速度的关系不同
        # （地面效应、地速为零），纳入估计只会污染模型。
        fsm = self._build(params_with_estimate())
        fsm.state = State.AUTO_LAND

        self._run_cycles(fsm, cycles=20)

        self.assertEqual(fsm.controller.estimate_stats.updates, 0)


if __name__ == "__main__":
    unittest.main()
