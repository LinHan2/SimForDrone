import argparse
import unittest

from tracking.evaluate_relative_ekf import evaluate


class RelativeEkfEvaluationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.args = argparse.Namespace(
            duration=30.0,
            dt=0.05,
            radius=6.0,
            angular_rate=0.35,
            bearing_noise=0.05,
            tilt_noise_deg=0.0,
            seed=17,
        )

    def test_calibrated_tilt_improves_low_noise_acceleration_estimation(self) -> None:
        self.args.tilt_noise_deg = 0.5

        no_tilt = evaluate(0.102, 1_000.0, 0.2449489743, self.args)
        tilt = evaluate(0.102, 0.03, 0.2449489743, self.args, tilt_direction_std=0.05)

        self.assertLess(tilt["acceleration_rmse_mps2"], no_tilt["acceleration_rmse_mps2"])
        self.assertLess(tilt["position_rmse_m"], no_tilt["position_rmse_m"])

    def test_truth_pose_baseline_validates_the_tilt_observer(self) -> None:
        self.args.bearing_noise = 0.0
        no_tilt = evaluate(0.102, 1_000.0, 0.2449489743, self.args)
        tilt = evaluate(0.102, 0.03, 0.2449489743, self.args)

        self.assertLess(tilt["position_rmse_m"], 0.1)
        self.assertLess(tilt["velocity_rmse_mps"], 0.04)
        self.assertLess(tilt["acceleration_rmse_mps2"], 0.01)
        self.assertLess(tilt["position_rmse_m"], no_tilt["position_rmse_m"] * 0.1)
        self.assertLess(tilt["acceleration_rmse_mps2"], no_tilt["acceleration_rmse_mps2"] * 0.1)

    def test_large_tilt_error_is_safer_to_gate_than_to_force_update(self) -> None:
        self.args.tilt_noise_deg = 5.0

        no_tilt = evaluate(0.102, 1_000.0, 0.2449489743, self.args)
        tilt = evaluate(0.102, 0.03, 0.2449489743, self.args, tilt_direction_std=0.07)

        self.assertLess(no_tilt["acceleration_rmse_mps2"], tilt["acceleration_rmse_mps2"])
        self.assertLess(no_tilt["velocity_rmse_mps"], tilt["velocity_rmse_mps"])