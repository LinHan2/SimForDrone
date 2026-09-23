"""单机阶跃响应指标的纯函数测试。"""

import unittest

from px4ctrl.cli import step_response_metrics


class StepResponseMetricsTest(unittest.TestCase):
    def test_metrics_capture_rise_overshoot_and_settling(self) -> None:
        samples = [
            {"t": 0.0, "position": 0.0},
            {"t": 1.0, "position": 0.46},
            {"t": 2.0, "position": 0.56},
            {"t": 3.0, "position": 0.49},
            {"t": 4.0, "position": 0.50},
        ]

        metrics = step_response_metrics(samples, 0.5)

        self.assertEqual(metrics["rise_time_s"], 1.0)
        self.assertEqual(metrics["settling_time_s"], 3.0)
        self.assertAlmostEqual(metrics["overshoot_m"], 0.06)
        self.assertAlmostEqual(metrics["final_error_m"], 0.098)

    def test_negative_step_uses_same_direction_normalization(self) -> None:
        samples = [
            {"t": 0.0, "position": 0.0},
            {"t": 1.0, "position": -0.30},
            {"t": 2.0, "position": -0.55},
            {"t": 3.0, "position": -0.50},
        ]

        metrics = step_response_metrics(samples, -0.5)

        self.assertEqual(metrics["rise_time_s"], 2.0)
        self.assertAlmostEqual(metrics["overshoot_m"], 0.05)

    def test_never_reaching_target_reports_no_rise_or_settling(self) -> None:
        samples = [{"t": 0.0, "position": 0.0}, {"t": 1.0, "position": 0.2}]

        metrics = step_response_metrics(samples, 0.5)

        self.assertIsNone(metrics["rise_time_s"])
        self.assertIsNone(metrics["settling_time_s"])


if __name__ == "__main__":
    unittest.main()
