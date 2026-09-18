import unittest

from tracking.estimation import NoisyTargetSensor, PassthroughEstimator, select_target_state
from tracking.guidance import TargetState


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


if __name__ == "__main__":
    unittest.main()
