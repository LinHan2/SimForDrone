import math
import unittest

from tracking.guidance import (
    DesiredState,
    ObserverState,
    PositionTrackerV0,
    TargetState,
    VisibilitySafetyFilter,
    combined_visibility_cost,
    visibility_intervention_weight,
    apply_feedforward_level,
    band_offset,
    follow_offset,
    pitch_compensated_offset,
    predicted_target,
    slew_yaw,
)


class PositionTrackerV0Test(unittest.TestCase):
    def test_horizontal_correction_speed_is_bounded_without_touching_height(self) -> None:
        tracker = PositionTrackerV0(relative_offset=(-5.0, 0.0, 1.0),
                                    max_correction_speed=0.45, position_to_speed_gain=1.5)
        desired = tracker.generate(TargetState(p=(10.0, 0.0, 2.0), v=(0.2, 0.0, 0.0)),
                                   ObserverState(shared_p=(0.0, 0.0, 2.0), local_p=(0.0, 0.0, 2.0)))
        self.assertAlmostEqual(desired.p[0], 0.3)
        self.assertEqual(desired.p[2], 3.0)
        self.assertEqual(desired.v, (0.2, 0.0, 0.0))

    def test_follow_reference_limits_position_step_without_changing_feedforward(self) -> None:
        tracker = PositionTrackerV0(relative_offset=(-5.0, 0.0, 0.0), max_position_error=1.0)
        desired = tracker.generate(
            TargetState(p=(10.0, 0.0, 2.0), v=(0.5, 0.0, 0.0)),
            ObserverState(shared_p=(-10.0, 0.0, 2.0), local_p=(0.0, 0.0, 2.0)),
        )
        self.assertEqual(desired.p, (1.0, 0.0, 2.0))
        self.assertEqual(desired.v, (0.5, 0.0, 0.0))

    def test_follow_offset_converges_along_initial_bearing(self) -> None:
        initial = (-8.0, 6.0, 0.2)
        self.assertEqual(follow_offset(initial, 5.0, 0.0), initial)
        self.assertEqual(follow_offset(initial, 5.0, 2.0), (-7.2, 5.4, 0.2))
        self.assertEqual(follow_offset(initial, 5.0, 20.0), (-4.0, 3.0, 0.2))

    def test_feedforward_levels_isolate_velocity_and_acceleration(self) -> None:
        target = TargetState(p=(1.0, 2.0, 3.0), v=(0.5, -0.2, 0.1), a=(0.3, 0.4, -0.1))

        only_p = apply_feedforward_level(target, "position")
        self.assertEqual((only_p.p, only_p.v, only_p.a), (target.p, (0.0,) * 3, (0.0,) * 3))

        with_v = apply_feedforward_level(target, "velocity")
        self.assertEqual((with_v.v, with_v.a), (target.v, (0.0,) * 3))

        self.assertEqual(apply_feedforward_level(target, "acceleration"), target)
        with self.assertRaises(ValueError):
            apply_feedforward_level(target, "jerk")

    def test_prediction_advances_position_only(self) -> None:
        target = TargetState(p=(0.0, 0.0, 0.0), v=(2.0, 0.0, 0.0), a=(0.0, 1.0, 0.0))

        self.assertEqual(predicted_target(target, 0.0), target)
        predicted = predicted_target(target, 0.1)
        self.assertAlmostEqual(predicted.p[0], 0.2)
        self.assertAlmostEqual(predicted.p[1], 0.005)
        self.assertEqual((predicted.v, predicted.a), (target.v, target.a))
        with self.assertRaises(ValueError):
            predicted_target(target, -0.1)

    def test_band_offset_keeps_current_bearing_and_radius_inside_band(self) -> None:
        current = (-6.0, 0.0, 0.4)

        self.assertEqual(band_offset(current, 5.0, 10.0), current)
        self.assertEqual(band_offset((-8.0, 6.0, 0.4), 5.0, 10.0), (-8.0, 6.0, 0.4))

    def test_band_offset_pulls_back_only_outside_the_band(self) -> None:
        near = band_offset((-3.0, 4.0, 0.2), 5.0, 10.0)
        self.assertAlmostEqual(math.hypot(near[0], near[1]), 5.0)

        far = band_offset((-12.0, 16.0, 0.2), 5.0, 10.0)
        self.assertAlmostEqual(math.hypot(far[0], far[1]), 10.0)
        self.assertAlmostEqual(far[0] / far[1], -12.0 / 16.0)

    def test_band_offset_rejects_invalid_band(self) -> None:
        with self.assertRaises(ValueError):
            band_offset((-6.0, 0.0, 0.0), 10.0, 5.0)
        with self.assertRaises(ValueError):
            band_offset((0.0, 0.0, 0.0), 5.0, 10.0)

    def test_pitch_compensation_raises_height_when_observer_pitches_down(self) -> None:
        offset = (-4.0, 3.0, 0.5)
        compensated = pitch_compensated_offset(offset, math.radians(10.0), 2.5)

        self.assertEqual(compensated[:2], offset[:2])
        self.assertAlmostEqual(compensated[2], 0.5 + 5.0 * math.tan(math.radians(10.0)))

    def test_pitch_compensation_is_bounded_and_signed(self) -> None:
        offset = (-5.0, 0.0, 0.0)

        self.assertAlmostEqual(pitch_compensated_offset(offset, math.radians(60.0), 1.5)[2], 1.5)
        self.assertAlmostEqual(pitch_compensated_offset(offset, math.radians(-60.0), 1.5)[2], -1.5)
        self.assertEqual(pitch_compensated_offset(offset, 0.0, 1.5), offset)

    def test_pitch_compensation_rejects_invalid_inputs(self) -> None:
        with self.assertRaises(ValueError):
            pitch_compensated_offset((-5.0, 0.0, 0.0), math.pi / 2.0, 1.5)
        with self.assertRaises(ValueError):
            pitch_compensated_offset((-5.0, 0.0, 0.0), 0.1, -1.0)

    def test_follow_offset_slews_height_without_changing_horizontal_offset(self) -> None:
        initial = (-5.0, 0.0, 0.3)
        self.assertEqual(follow_offset(initial, 5.0, 0.0, height_offset=0.05), initial)
        partial = follow_offset(initial, 5.0, 2.0, height_offset=0.05, height_speed=0.05)
        self.assertAlmostEqual(partial[2], 0.2)
        self.assertEqual(partial[:2], initial[:2])
        self.assertAlmostEqual(follow_offset(initial, 5.0, 20.0, height_offset=0.05)[2], 0.05)

    def test_yaw_slew_uses_shortest_path_and_rate_limit(self) -> None:
        self.assertAlmostEqual(slew_yaw(0.0, math.pi / 2, 0.1), 0.1)
        self.assertAlmostEqual(slew_yaw(math.pi - 0.05, -math.pi + 0.05, 0.02), math.pi - 0.03)

    def test_static_target_uses_fixed_world_offset(self) -> None:
        tracker = PositionTrackerV0(relative_offset=(-3.0, 0.0, 1.0))

        desired = tracker.generate(
            TargetState(p=(10.0, 5.0, 2.0)),
            ObserverState(shared_p=(6.0, 5.0, 3.0), local_p=(1.0, 2.0, 3.0)),
        )

        self.assertEqual(desired.p, (2.0, 2.0, 3.0))
        self.assertEqual(desired.v, (0.0, 0.0, 0.0))
        self.assertEqual(desired.yaw, 0.0)

    def test_target_feedforward_is_preserved(self) -> None:
        tracker = PositionTrackerV0(relative_offset=(0.0, -2.0, 0.0))

        desired = tracker.generate(
            TargetState(p=(4.0, 8.0, 2.0), v=(1.0, -0.5, 0.2), a=(0.1, 0.0, -0.1)),
            ObserverState(shared_p=(4.0, 5.0, 2.0), local_p=(20.0, 30.0, 1.5)),
        )

        self.assertEqual(desired.p, (20.0, 31.0, 1.5))
        self.assertEqual(desired.v, (1.0, -0.5, 0.2))
        self.assertEqual(desired.a, (0.1, 0.0, -0.1))

    def test_shared_origin_shift_does_not_change_local_reference(self) -> None:
        tracker = PositionTrackerV0(relative_offset=(-3.0, 0.0, 1.0))
        original = tracker.generate(
            TargetState(p=(3.0, 0.0, 4.0)),
            ObserverState(shared_p=(0.0, 0.0, 3.0), local_p=(0.6, -0.2, 2.0)),
        )
        shifted = tracker.generate(
            TargetState(p=(103.0, 200.0, 4.0)),
            ObserverState(shared_p=(100.0, 200.0, 3.0), local_p=(0.6, -0.2, 2.0)),
        )

        self.assertEqual(original.p, (0.6, -0.2, 4.0))
        self.assertEqual(shifted.p, original.p)

    @staticmethod
    def retreat(weight, radius=-6.0, dt=0.05, retreat_speed=1.0, height=0.0):
        filter_ = VisibilitySafetyFilter(min_distance=5.0, max_distance=10.0,
                                        retreat_speed=retreat_speed)
        reference = DesiredState(p=(radius, 0.0, height))
        return filter_.apply(
            reference,
            TargetState(p=(0.0, 0.0, 0.0)),
            ObserverState(shared_p=(radius, 0.0, height), local_p=(radius, 0.0, height)),
            intervention_weight=weight, dt=dt,
        ), reference

    def test_intervention_weight_is_piecewise_linear_between_safe_and_warning(self) -> None:
        self.assertEqual(visibility_intervention_weight(0.5, 0.6, 0.8), 0.0)
        self.assertEqual(visibility_intervention_weight(0.6, 0.6, 0.8), 0.0)
        self.assertAlmostEqual(visibility_intervention_weight(0.7, 0.6, 0.8), 0.5)
        self.assertEqual(visibility_intervention_weight(0.8, 0.6, 0.8), 1.0)
        self.assertEqual(visibility_intervention_weight(1.4, 0.6, 0.8), 1.0)
        # 不可投影（相机后方/近像平面）是最危险情形，必须给最大干预。
        self.assertEqual(visibility_intervention_weight(None, 0.6, 0.8), 1.0)

    def test_zero_weight_leaves_nominal_reference_untouched(self) -> None:
        safe, reference = self.retreat(0.0)
        self.assertIs(safe, reference)

    def test_partial_weight_produces_partial_retreat(self) -> None:
        safe, _ = self.retreat(0.5)
        self.assertAlmostEqual(safe.p[0], -6.025)
        full, _ = self.retreat(1.0)
        self.assertAlmostEqual(full.p[0], -6.05)

    def test_retreat_is_capped_at_max_distance(self) -> None:
        safe, _ = self.retreat(1.0, radius=-9.99, dt=1.0, retreat_speed=5.0)
        self.assertAlmostEqual(safe.p[0], -10.0)

    def test_retreat_never_changes_vertical_reference(self) -> None:
        for weight in (0.0, 0.3, 1.0):
            safe, _ = self.retreat(weight, height=1.5)
            self.assertEqual(safe.p[2], 1.5)

    def test_filter_pulls_reference_back_inside_band_without_touching_height(self) -> None:
        filter_ = VisibilitySafetyFilter(min_distance=5.0, max_distance=10.0)
        reference = DesiredState(p=(15.0, 0.0, 1.0), v=(1.0, 0.0, 0.0))
        safe = filter_.apply(
            reference,
            TargetState(p=(0.0, 0.0, 0.0)),
            ObserverState(shared_p=(15.0, 0.0, 1.0), local_p=(15.0, 0.0, 1.0)),
        )
        self.assertAlmostEqual(safe.p[0], 10.0)
        self.assertEqual(safe.p[2], 1.0)

    def test_filter_rejects_out_of_range_weight(self) -> None:
        for weight in (-0.1, 1.1):
            with self.assertRaises(ValueError):
                self.retreat(weight)

    def test_combined_visibility_cost_scores_safe_candidates_better(self) -> None:
        safe_cost = combined_visibility_cost(
            fov_cost=0.0,
            distance_cost=0.0,
            tracking_cost=0.0,
            smooth_cost=0.0,
        )
        risky_cost = combined_visibility_cost(
            fov_cost=0.25,
            distance_cost=2.0,
            pixel_occupancy_cost=0.5,
            tracking_cost=0.5,
            smooth_cost=0.1,
            fov_weight=1.5,
            distance_weight=1.0,
            tracking_weight=0.5,
            smoothness_weight=0.2,
            pixel_occupancy_weight=2.0,
        )
        self.assertEqual(safe_cost, 0.0)
        self.assertGreater(risky_cost, safe_cost)


if __name__ == "__main__":
    unittest.main()
