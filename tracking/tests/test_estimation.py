import math
import unittest
from unittest.mock import patch

import numpy as np

from tracking.estimation import (
    FreshTargetTimestampGate,
    NoisyTargetSensor,
    ObserverKinematics,
    PassthroughEstimator,
    RelativeEkfTargetEstimator,
    TargetMeasurement,
    observer_world_acceleration,
    select_target_state,
)
from tracking.guidance import TargetState
from tracking.relative_ekf import MetricPoseMeasurement, MetricRelativeTargetEKF, RelativePoseMeasurement, RelativeTargetEKF
from tracking.run_tracker import parse_args as parse_tracker_args


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


class MetricRelativeTargetEKFTest(unittest.TestCase):
    def test_tracker_constructs_metric_filter_without_scale(self) -> None:
        from tracking.run_tracker import make_relative_ekf, parse_args

        with patch("sys.argv", ["tracker"]):
            estimator = make_relative_ekf(parse_args())
        self.assertIsInstance(estimator._filter, MetricRelativeTargetEKF)
        self.assertEqual(estimator._filter.covariance.shape, (9, 9))

    def test_metric_position_has_no_scale_state(self) -> None:
        estimator = MetricRelativeTargetEKF(position_std=0.05)
        measurement = MetricPoseMeasurement(0.0, (3.0, 4.0, 1.0), (0.0, 0.0, 1.0))
        estimate = estimator.update(measurement, (0.0, 0.0, 0.0))
        np.testing.assert_allclose(estimate.relative_position, measurement.relative_position)
        self.assertEqual(estimator.covariance.shape, (9, 9))
        self.assertFalse(hasattr(estimate, "scale"))
        self.assertNotIn("scale_process_std_m", estimator.tuning)

    def test_missing_tilt_and_depth_predict_without_fabricated_measurement(self) -> None:
        estimator = MetricRelativeTargetEKF()
        estimator.update(MetricPoseMeasurement(0.0, (2.0, 0.0, 1.0), None), (0.0, 0.0, 0.0))
        predicted = estimator.update(MetricPoseMeasurement(0.1, None, None), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(predicted.relative_position[0], 2.0)
        with self.assertRaises(ValueError):
            estimator.update(MetricPoseMeasurement(0.1, None, None), (0.0, 0.0, 0.0))

    def test_three_metric_positions_estimate_velocity(self) -> None:
        estimator = MetricRelativeTargetEKF(position_std=0.01)
        for step in range(30):
            time_s = 0.1 * step
            estimate = estimator.update(
                MetricPoseMeasurement(time_s, (2.0 + 0.4 * time_s, 1.0, 0.5), None),
                (0.0, 0.0, 0.0),
            )
        self.assertAlmostEqual(estimate.relative_velocity[0], 0.4, delta=0.04)


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

    def test_attitude_constraint_uses_only_thrust_direction(self) -> None:
        first = RelativeTargetEKF().update(
            RelativePoseMeasurement(0.0, (2.0, 0.0, 1.0), (0.2, 0.0, 0.98)),
            observer_acceleration=(0.0, 0.0, 0.0),
        )
        # 滤波器接口没有 yaw；只要视觉前端给出世界系推力方向 h，投影约束即可成立。
        # 将 h 绕世界 z 轴旋转会改变水平分量朝向，但不改变其与重力的夹角。
        second = RelativeTargetEKF().update(
            RelativePoseMeasurement(0.0, (2.0, 0.0, 1.0), (0.0, 0.2, 0.98)),
            observer_acceleration=(0.0, 0.0, 0.0),
        )

        self.assertAlmostEqual(
            math.dist(first.target_acceleration, (0.0, 0.0, -9.81)),
            math.dist(second.target_acceleration, (0.0, 0.0, -9.81)),
            places=10,
        )

    def test_tangent_basis_is_an_orthonormal_projector_factor(self) -> None:
        thrust_direction = np.array((0.31, -0.47, 0.82))
        thrust_direction /= np.linalg.norm(thrust_direction)

        basis = RelativeTargetEKF._tangent_basis(thrust_direction)
        projection = np.eye(3) - np.outer(thrust_direction, thrust_direction)

        np.testing.assert_allclose(basis.T @ basis, np.eye(2), atol=1e-12)
        np.testing.assert_allclose(basis.T @ thrust_direction, np.zeros(2), atol=1e-12)
        np.testing.assert_allclose(basis @ basis.T, projection, atol=1e-12)
        self.assertEqual(np.linalg.matrix_rank(basis.T), 2)

    def test_tangent_update_matches_redundant_projector_update(self) -> None:
        estimator = RelativeTargetEKF(attitude_constraint_std=0.07)
        estimator.initialize((2.0, -1.0, 0.5), (0.3, 0.1, -0.2), scale=2.5)
        estimator._x[6:9] = (1.2, -0.7, 0.4)
        estimator._P = np.diag(np.linspace(0.5, 2.0, 10))
        thrust_direction = np.array((0.31, -0.47, 0.82))
        thrust_direction /= np.linalg.norm(thrust_direction)
        prior_state = estimator._x.copy()
        prior_covariance = estimator._P.copy()

        estimator._attitude_update(thrust_direction)

        projection = np.eye(3) - np.outer(thrust_direction, thrust_direction)
        observation = np.zeros((3, 10))
        observation[:, 6:9] = projection
        noise = np.eye(3) * estimator._attitude_variance
        innovation = projection @ np.array((0.0, 0.0, -9.81)) - observation @ prior_state
        innovation_covariance = observation @ prior_covariance @ observation.T + noise
        gain = np.linalg.solve(innovation_covariance, observation @ prior_covariance).T
        expected_state = prior_state + gain @ innovation
        residual = np.eye(10) - gain @ observation
        expected_covariance = residual @ prior_covariance @ residual.T + gain @ noise @ gain.T

        np.testing.assert_allclose(estimator._x, expected_state, atol=1e-12)
        np.testing.assert_allclose(estimator.covariance, expected_covariance, atol=1e-12)

    def test_optional_tilt_direction_uncertainty_scales_measurement_noise(self) -> None:
        estimator = RelativeTargetEKF(tilt_direction_std=0.02)
        estimator.initialize((2.0, -1.0, 0.5), scale=2.5)
        estimator._x[6:9] = (0.0, 0.0, 0.0)

        np.testing.assert_allclose(
            estimator._tilt_measurement_noise(),
            np.eye(2) * (9.81 * 0.02) ** 2,
            atol=1e-12,
        )

    def test_default_tilt_measurement_noise_keeps_calibrated_variance(self) -> None:
        estimator = RelativeTargetEKF(attitude_constraint_std=0.07)

        np.testing.assert_allclose(
            estimator._tilt_measurement_noise(), np.eye(2) * 0.07**2, atol=1e-12
        )

    def test_stereo_range_corrects_distance_and_bearing_box_scale(self) -> None:
        without_range = RelativeTargetEKF(position_std=1.0)
        with_range = RelativeTargetEKF(position_std=1.0, stereo_range_std=0.01)
        for estimator in (without_range, with_range):
            estimator.initialize((4.0, 0.0, 0.0), scale=2.0)
            estimator._timestamp = 0.0
        base_measurement = RelativePoseMeasurement(
            timestamp=0.1,
            relative_position=(6.0, 0.0, 0.0),
            target_thrust_direction=(0.0, 0.0, 1.0),
        )
        range_measurement = RelativePoseMeasurement(
            timestamp=0.1,
            relative_position=(6.0, 0.0, 0.0),
            target_thrust_direction=(0.0, 0.0, 1.0),
            stereo_range_m=6.0,
        )

        estimate_without_range = without_range.update(base_measurement, (0.0, 0.0, 0.0))
        estimate_with_range = with_range.update(range_measurement, (0.0, 0.0, 0.0))

        self.assertLess(
            abs(math.dist(estimate_with_range.relative_position, (0.0, 0.0, 0.0)) - 6.0),
            abs(math.dist(estimate_without_range.relative_position, (0.0, 0.0, 0.0)) - 6.0),
        )
        self.assertLess(abs(estimate_with_range.scale - 6.0), abs(estimate_without_range.scale - 6.0))

    def test_stereo_range_requires_positive_finite_value(self) -> None:
        estimator = RelativeTargetEKF(stereo_range_std=0.1)
        for invalid_range in (0.0, -1.0, float("nan")):
            with self.assertRaises(ValueError):
                estimator.update(
                    RelativePoseMeasurement(0.0, (3.0, 0.0, 0.0), (0.0, 0.0, 1.0), invalid_range),
                    (0.0, 0.0, 0.0),
                )

    def test_constant_acceleration_state_transition(self) -> None:
        estimator = RelativeTargetEKF()
        estimator.initialize((2.0, -1.0, 0.5), (0.4, -0.2, 0.1), scale=2.0)
        estimator._x[6:9] = (0.3, 0.2, -0.1)

        estimator._predict(0.5, np.array((0.1, -0.2, 0.3)))

        np.testing.assert_allclose(estimator._x[:3], (2.225, -1.05, 0.5))
        np.testing.assert_allclose(estimator._x[3:6], (0.5, 0.0, -0.1))
        np.testing.assert_allclose(estimator._x[6:9], (0.3, 0.2, -0.1))
        self.assertAlmostEqual(estimator._x[9], 2.0)

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

    def test_bearing_box_residual_is_bounded(self) -> None:
        """bearing-box 仅约束 delta_p-alpha*p_bar，不能按全位置量测验收。"""

        random_source = np.random.default_rng(21)
        estimator = RelativeTargetEKF(position_std=0.12, acceleration_process_std=0.35)
        residuals = []
        for step in range(301):
            timestamp = step * 0.05
            truth = np.array((1.0 + 0.3 * timestamp, -0.4, 0.8))
            measurement = truth + random_source.normal(0.0, 0.12, size=3)
            estimate = estimator.update(
                RelativePoseMeasurement(timestamp, tuple(measurement), (0.0, 0.0, 1.0)),
                observer_acceleration=(0.0, 0.0, 0.0),
            )
            if step >= 60:
                direction = measurement / np.linalg.norm(measurement)
                residuals.append(float(np.linalg.norm(
                    np.asarray(estimate.relative_position) - estimate.scale * direction
                )))

        self.assertTrue(all(math.isfinite(value) for value in residuals))
        self.assertLess(sum(residuals) / len(residuals), 1.0)

    def test_initial_scale_uses_first_bearing_proxy_range(self) -> None:
        estimate = RelativeTargetEKF().update(
            RelativePoseMeasurement(0.0, (3.0, 4.0, 0.0), (0.0, 0.0, 1.0)),
            observer_acceleration=(0.0, 0.0, 0.0),
        )

        self.assertAlmostEqual(estimate.scale, 5.0)


class RelativeEkfTargetEstimatorTest(unittest.TestCase):
    def test_estimator_forwards_optional_stereo_range(self) -> None:
        filter_ = RelativeTargetEKF(stereo_range_std=0.01)
        estimator = RelativeEkfTargetEstimator(filter_)
        observer = ObserverKinematics(
            p=(0.0, 0.0, 0.0), v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0)
        )

        estimate = estimator.update(
            TargetMeasurement(0.0, (3.0, 4.0, 0.0), (0.0, 0.0, 0.0), stereo_range_m=5.0),
            target_attitude=(0.0, 0.0, 0.0, 1.0),
            observer=observer,
        )

        self.assertTrue(all(math.isfinite(value) for value in estimate.p + estimate.v + estimate.a))

    def test_body_yaw_symmetry_leaves_observer_update_unchanged(self) -> None:
        """R and R*Rz(gamma) share a thrust axis, so yaw cannot alter the EKF update."""

        attitude = (0.2, -0.1, 0.3, 0.92)
        body_yaw = (0.0, 0.0, math.sin(0.65), math.cos(0.65))
        yaw_ambiguous_attitude = _quaternion_product(attitude, body_yaw)
        observer = ObserverKinematics(
            p=(1.0, -2.0, 0.5), v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0)
        )
        measurement = TargetMeasurement(0.0, (4.0, 1.0, 2.5), (0.0, 0.0, 0.0))

        first = RelativeEkfTargetEstimator().update(measurement, attitude, observer)
        second = RelativeEkfTargetEstimator().update(
            measurement, yaw_ambiguous_attitude, observer
        )

        np.testing.assert_allclose(first.p, second.p, atol=1e-12)
        np.testing.assert_allclose(first.v, second.v, atol=1e-12)
        np.testing.assert_allclose(first.a, second.a, atol=1e-12)

    def test_uses_filter_velocity_instead_of_measurement_velocity(self) -> None:
        estimator = RelativeEkfTargetEstimator(
            RelativeTargetEKF(position_std=0.02, acceleration_process_std=0.1)
        )
        estimate = None
        for step in range(101):
            timestamp = step * 0.1
            position = (3.0 + 0.4 * timestamp, -1.0, 2.0)
            estimate = estimator.update(
                TargetMeasurement(timestamp, position, (99.0, 99.0, 99.0)),
                target_attitude=(0.0, 0.0, 0.0, 1.0),
                observer=ObserverKinematics(
                    p=(1.0, 2.0, 2.0), v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0)
                ),
            )

        assert estimate is not None
        self.assertTrue(all(math.isfinite(value) for value in estimate.p + estimate.v))
        self.assertGreater(math.dist(estimate.v, (99.0, 99.0, 99.0)), 100.0)
        self.assertLess(math.dist(estimate.a, (0.0, 0.0, 0.0)), 1e-6)

    def test_hover_specific_force_maps_to_zero_world_acceleration(self) -> None:
        acceleration = observer_world_acceleration(
            attitude=(0.0, 0.0, 0.0, 1.0), specific_force_body=(0.0, 0.0, 9.81)
        )

        self.assertLess(math.dist(acceleration, (0.0, 0.0, 0.0)), 1e-12)


def _quaternion_product(
    left: tuple[float, float, float, float], right: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    left_x, left_y, left_z, left_w = left
    right_x, right_y, right_z, right_w = right
    return (
        left_w * right_x + left_x * right_w + left_y * right_z - left_z * right_y,
        left_w * right_y - left_x * right_z + left_y * right_w + left_z * right_x,
        left_w * right_z + left_x * right_y - left_y * right_x + left_z * right_w,
        left_w * right_w - left_x * right_x - left_y * right_y - left_z * right_z,
    )


class FreshTargetTimestampGateTest(unittest.TestCase):
    """tracker 控制频率高于状态发布频率时的时间戳去重语义。"""

    def test_accepts_strictly_new_timestamps(self) -> None:
        gate = FreshTargetTimestampGate()

        self.assertTrue(gate.accept(1.0))
        self.assertTrue(gate.accept(1.05))
        self.assertEqual(gate.duplicate_count, 0)
        self.assertEqual(gate.regression_count, 0)

    def test_duplicate_timestamp_is_rejected_and_counted(self) -> None:
        gate = FreshTargetTimestampGate()
        gate.accept(2.0)

        self.assertFalse(gate.accept(2.0))
        self.assertFalse(gate.accept(2.0))
        self.assertEqual(gate.duplicate_count, 2)
        self.assertEqual(gate.regression_count, 0)
        self.assertTrue(gate.accept(2.1))

    def test_regressed_timestamp_is_rejected_and_counted(self) -> None:
        gate = FreshTargetTimestampGate()
        gate.accept(3.0)
        gate.accept(3.1)

        self.assertFalse(gate.accept(3.05))
        self.assertEqual(gate.regression_count, 1)
        self.assertTrue(gate.accept(3.2))

    def test_duplicate_frame_keeps_last_estimate_without_updating(self) -> None:
        gate = FreshTargetTimestampGate()
        estimator = RelativeEkfTargetEstimator(
            RelativeTargetEKF(position_std=0.1, acceleration_process_std=0.2)
        )
        observer = ObserverKinematics(p=(0.0, 0.0, 0.0), v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0))
        self.assertTrue(gate.accept(10.0))
        first = estimator.update(
            TargetMeasurement(10.0, (3.0, -1.0, 2.0), (0.0, 0.0, 0.0)),
            target_attitude=(0.0, 0.0, 0.0, 1.0),
            observer=observer,
        )
        # 重复时间戳被门控拒绝：不推进 EKF，估计保持上一帧。
        self.assertFalse(gate.accept(10.0))
        self.assertEqual(estimator.last_estimate, first)
        # 新时间戳继续更新，滤波器不会因重复帧报错。
        self.assertTrue(gate.accept(10.05))
        second = estimator.update(
            TargetMeasurement(10.05, (3.01, -1.0, 2.0), (0.0, 0.0, 0.0)),
            target_attitude=(0.0, 0.0, 0.0, 1.0),
            observer=observer,
        )
        self.assertIsNot(estimator.last_estimate, first)
        self.assertEqual(estimator.last_estimate, second)

    def test_stationary_noisy_warmup_keeps_state_finite(self) -> None:
        gate = FreshTargetTimestampGate()
        estimator = RelativeEkfTargetEstimator(
            RelativeTargetEKF(
                position_std=0.12,
                attitude_constraint_std=0.18,
                acceleration_process_std=0.35,
            )
        )
        sensor = NoisyTargetSensor(position_std=0.12, seed=21)
        observer = ObserverKinematics(p=(0.0, 0.0, 0.0), v=(0.0, 0.0, 0.0), a=(0.0, 0.0, 0.0))
        truth = TargetState(p=(3.0, -1.0, 2.0), v=(0.0, 0.0, 0.0))
        estimate = None
        timestamp = 0.0
        while timestamp < 2.0:
            measurement = sensor.measure(truth, timestamp)
            if gate.accept(timestamp) and measurement is not None:
                estimate = estimator.update(
                    measurement, target_attitude=(0.0, 0.0, 0.0, 1.0), observer=observer
                )
            timestamp += 0.05
        assert estimate is not None
        # 无 3D box 尺度前端时不可将其作为全位置误差验收；只验数值安全约束。
        self.assertTrue(all(math.isfinite(value) for value in estimate.p + estimate.v + estimate.a))
        self.assertLessEqual(math.dist(estimate.p, observer.p), 120.0)
        self.assertLessEqual(math.dist(estimate.v, (0.0, 0.0, 0.0)), 10.0)


class Airsim2boxParameterMigrationTest(unittest.TestCase):
    """迁移 Airsim2box StrictPaperRelativeEKF 参数后的回归保护。"""

    def test_default_tuning_matches_airsim2box(self) -> None:
        tuning = RelativeTargetEKF().tuning
        self.assertAlmostEqual(tuning["position_std_m"], 0.102)
        self.assertAlmostEqual(tuning["attitude_constraint_std_mps2"], 0.03)
        self.assertAlmostEqual(tuning["position_process_std_m"], 0.002)
        self.assertAlmostEqual(tuning["velocity_process_std_mps"], 0.1095, places=4)
        self.assertAlmostEqual(tuning["acceleration_process_std_mps2"], 0.2449, places=4)
        self.assertAlmostEqual(tuning["scale_process_std_m"], 0.0949, places=4)
        self.assertAlmostEqual(tuning["fading_lambda"], 1.001)

    def test_scale_process_noise_rejects_non_positive(self) -> None:
        with self.assertRaises(ValueError):
            RelativeTargetEKF(scale_process_std=0.0)

    def test_attitude_and_state_limits_keep_vertical_terms_bounded(self) -> None:
        """论文姿态投影与 Airsim2box 状态限幅应保持垂向状态有限。"""

        random_source = np.random.default_rng(21)
        estimator = RelativeTargetEKF(position_std=0.08)
        max_acceleration_z = 0.0
        max_velocity_z = 0.0
        for step in range(200):
            timestamp = step * 0.1
            truth = np.array((1.0 + 0.3 * timestamp, -0.4, 0.8))
            measurement = truth + random_source.normal(0.0, 0.08, size=3)
            estimate = estimator.update(
                RelativePoseMeasurement(timestamp, tuple(measurement), (0.0, 0.0, 1.0)),
                observer_acceleration=(0.0, 0.0, 0.0),
            )
            max_acceleration_z = max(max_acceleration_z, abs(estimate.target_acceleration[2]))
            max_velocity_z = max(max_velocity_z, abs(estimate.relative_velocity[2]))

        self.assertLess(max_acceleration_z, 1.0)
        self.assertLess(max_velocity_z, 1.5)

    def test_fading_below_one_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            RelativeTargetEKF(fading=0.99)

    def test_observer_acceleration_noise_does_not_diverge(self) -> None:
        """观测机加速度噪声下速度估计保持有界（旧参数曾发散到 11 m/s）。

        按真实量测配置标定：10 Hz、位置噪声 σ=0.08 m、观测机加速度为带宽受限
        （低通 τ=0.5 s）的 σ=0.3 m/s² 噪声，模拟机体动力学而非 20 Hz 白噪声。
        """

        random_source = np.random.default_rng(21)
        estimator = RelativeTargetEKF(position_std=0.08)
        raw_acceleration = random_source.normal(0.0, 0.3, size=(200, 3))
        observer_acceleration = np.zeros_like(raw_acceleration)
        smoothing = 1.0 - math.exp(-0.1 / 0.5)
        for index in range(1, 200):
            observer_acceleration[index] = observer_acceleration[index - 1] + smoothing * (
                raw_acceleration[index] - observer_acceleration[index - 1]
            )
        max_speed = 0.0
        for step in range(200):
            timestamp = step * 0.1
            truth = np.array((1.0 + 0.3 * timestamp, -0.4, 0.8))
            measurement = truth + random_source.normal(0.0, 0.08, size=3)
            estimate = estimator.update(
                RelativePoseMeasurement(timestamp, tuple(measurement), (0.0, 0.0, 1.0)),
                observer_acceleration=tuple(observer_acceleration[step]),
            )
            max_speed = max(max_speed, float(np.linalg.norm(estimate.relative_velocity)))

        self.assertLess(max_speed, 5.0)

    def test_state_clamps_cap_adversarial_measurements(self) -> None:
        estimator = RelativeTargetEKF()
        estimate = None
        for step in range(60):
            timestamp = step * 0.05
            estimate = estimator.update(
                RelativePoseMeasurement(
                    timestamp, (120.0 + step, -150.0, 90.0), (0.0, 0.0, 1.0)
                ),
                observer_acceleration=(0.0, 0.0, 0.0),
            )

        assert estimate is not None
        covariance = estimator.covariance
        self.assertTrue(np.all(np.isfinite(covariance)))
        self.assertLessEqual(float(np.linalg.norm(estimate.relative_position)), 120.0 + 1e-9)
        self.assertLessEqual(float(np.linalg.norm(estimate.relative_velocity)), 10.0 + 1e-9)
        self.assertLessEqual(float(np.linalg.norm(estimate.target_acceleration)), 20.0 + 1e-9)


if __name__ == "__main__":
    unittest.main()
