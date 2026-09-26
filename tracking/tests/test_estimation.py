import math
import unittest

import numpy as np

from tracking.estimation import NoisyTargetSensor, PassthroughEstimator, select_target_state
from tracking.guidance import TargetState
from tracking.relative_ekf import RelativePoseMeasurement, RelativeTargetEKF


class EstimationBoundaryTest(unittest.TestCase):
    def test_truth_source_bypasses_noise_and_estimator(self) -> None:
        truth = TargetState(p=(1.0, 2.0, 3.0), v=(0.1, 0.2, 0.3))
        selected, measurement = select_target_state(
            "truth", truth, 10.0, NoisyTargetSensor(position_std=1.0), PassthroughEstimator()
        )

        self.assertIs(selected, truth)
        self.assertIsNone(measurement)

    def test_estimator_source_uses_noisy_measurement(self) -> None:
        truth = TargetState(p=(1.0, 2.0, 3.0), v=(0.1, 0.2, 0.3))
        selected, measurement = select_target_state(
            "estimator",
            truth,
            10.0,
            NoisyTargetSensor(position_std=0.1, velocity_std=0.05, seed=7),
            PassthroughEstimator(),
        )

        self.assertIsNotNone(measurement)
        self.assertEqual(selected.p, measurement.p)
        self.assertEqual(selected.v, measurement.v)
        self.assertNotEqual(selected.p, truth.p)

    def test_noise_is_reproducible_for_same_seed(self) -> None:
        truth = TargetState(p=(1.0, 2.0, 3.0), v=(0.0, 0.0, 0.0))
        first = NoisyTargetSensor(seed=42).measure(truth, 1.0)
        second = NoisyTargetSensor(seed=42).measure(truth, 1.0)

        self.assertEqual(first, second)

    def test_negative_noise_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            NoisyTargetSensor(position_std=-0.1)

    def test_dropout_probability_is_validated(self) -> None:
        with self.assertRaises(ValueError):
            NoisyTargetSensor(dropout_probability=1.5)

    def test_default_sensor_never_drops(self) -> None:
        sensor = NoisyTargetSensor(position_std=0.05, seed=3)
        truth = TargetState(p=(1.0, 2.0, 3.0))

        self.assertTrue(all(sensor.measure(truth, float(index)) is not None for index in range(20)))

    def test_total_dropout_returns_no_measurement(self) -> None:
        sensor = NoisyTargetSensor(dropout_probability=1.0, seed=3)

        self.assertIsNone(sensor.measure(TargetState(p=(1.0, 2.0, 3.0)), 1.0))

    def test_dropout_is_reproducible_for_same_seed(self) -> None:
        truth = TargetState(p=(1.0, 2.0, 3.0))
        first = NoisyTargetSensor(seed=9, dropout_probability=0.5)
        second = NoisyTargetSensor(seed=9, dropout_probability=0.5)

        pattern = [first.measure(truth, float(index)) is None for index in range(30)]
        self.assertEqual(pattern, [second.measure(truth, float(index)) is None for index in range(30)])
        # 同一组参数下必须真的产生丢包，否则这个测试就没有区分力。
        self.assertTrue(any(pattern) and not all(pattern))

    def test_invalid_measurement_keeps_previous_estimate(self) -> None:
        truth = TargetState(p=(1.0, 2.0, 3.0))
        estimator = PassthroughEstimator()
        always_lost = NoisyTargetSensor(dropout_probability=1.0, seed=1)

        # 从未取得有效量测：必须报错，而不是猜一个状态送进控制环。
        with self.assertRaises(RuntimeError):
            select_target_state("estimator", truth, 1.0, always_lost, estimator)

        good = NoisyTargetSensor(position_std=0.1, seed=1)
        first, measurement = select_target_state("estimator", truth, 2.0, good, estimator)
        self.assertIsNotNone(measurement)

        held, lost_measurement = select_target_state("estimator", truth, 3.0, always_lost, estimator)
        self.assertIsNone(lost_measurement)
        self.assertEqual(held, first)


class RelativeTargetEKFTest(unittest.TestCase):
    def test_hover_constraint_uses_enu_gravity_sign(self) -> None:
        estimator = RelativeTargetEKF()
        estimate = estimator.update(
            RelativePoseMeasurement(
                timestamp=0.0,
                relative_position=(3.0, -1.0, 0.5),
                target_thrust_direction=(0.0, 0.0, 1.0),
            ),
            observer_acceleration=(0.0, 0.0, 0.0),
        )

        self.assertLess(math.dist(estimate.target_acceleration, (0.0, 0.0, 0.0)), 1e-8)

    def test_attitude_constraint_is_invariant_to_yaw(self) -> None:
        first = RelativeTargetEKF().update(
            RelativePoseMeasurement(0.0, (2.0, 0.0, 1.0), (0.2, 0.0, 0.98)),
            observer_acceleration=(0.0, 0.0, 0.0),
        )
        # 绕世界 z 轴改变 yaw，不改变同一个倾角对应的推力方向与重力夹角。
        second = RelativeTargetEKF().update(
            RelativePoseMeasurement(0.0, (2.0, 0.0, 1.0), (0.0, 0.2, 0.98)),
            observer_acceleration=(0.0, 0.0, 0.0),
        )

        self.assertAlmostEqual(
            math.dist(first.target_acceleration, (0.0, 0.0, -9.81)),
            math.dist(second.target_acceleration, (0.0, 0.0, -9.81)),
            places=10,
        )

    def test_constant_velocity_estimate_converges(self) -> None:
        estimator = RelativeTargetEKF(position_std=0.03, acceleration_process_std=0.2)
        estimate = None
        for step in range(101):
            timestamp = step * 0.1
            truth = (2.0 + 0.4 * timestamp, -1.0, 0.5)
            estimate = estimator.update(
                RelativePoseMeasurement(timestamp, truth, (0.0, 0.0, 1.0)),
                observer_acceleration=(0.0, 0.0, 0.0),
            )

        assert estimate is not None
        self.assertLess(math.dist(estimate.relative_position, truth), 0.03)
        self.assertLess(math.dist(estimate.relative_velocity, (0.4, 0.0, 0.0)), 0.05)

    def test_covariance_remains_symmetric_positive_semidefinite(self) -> None:
        estimator = RelativeTargetEKF(position_std=0.05)
        for step in range(40):
            timestamp = step * 0.05
            estimator.update(
                RelativePoseMeasurement(
                    timestamp,
                    (1.0 + 0.2 * timestamp, -0.5, 0.8),
                    (0.0, 0.0, 1.0),
                ),
                observer_acceleration=(0.0, 0.0, 0.0),
            )

        covariance = estimator.covariance
        self.assertTrue(np.all(np.isfinite(covariance)))
        self.assertTrue(np.allclose(covariance, covariance.T))
        self.assertGreaterEqual(float(np.min(np.linalg.eigvalsh(covariance))), -1e-9)

    def test_rejects_non_increasing_timestamps(self) -> None:
        estimator = RelativeTargetEKF()
        measurement = RelativePoseMeasurement(1.0, (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        estimator.update(measurement, (0.0, 0.0, 0.0))

        with self.assertRaises(ValueError):
            estimator.update(measurement, (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
