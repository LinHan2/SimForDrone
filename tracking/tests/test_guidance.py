import math
import unittest

from tracking.guidance import (
    DesiredState,
    ObserverState,
    PositionTrackerV0,
    TargetState,
    VisibilitySafetyFilter,
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

    def test_visibility_safety_filter_is_noop_when_nominal_reference_is_safe(self) -> None:
        filter_ = VisibilitySafetyFilter(min_distance=5.0, max_distance=10.0)
        reference = DesiredState(p=(7.0, 0.0, 1.0), v=(0.2, 0.0, 0.0))
        safe = filter_.apply(
            reference,
            TargetState(p=(0.0, 0.0, 0.0)),
            ObserverState(shared_p=(7.0, 0.0, 1.0), local_p=(7.0, 0.0, 1.0)),
            predicted_exit=False,
        )
        self.assertEqual(safe, reference)

    def test_visibility_safety_filter_uses_tracker_minus_target_and_preserves_height(self) -> None:
        filter_ = VisibilitySafetyFilter(min_distance=5.0, max_distance=10.0)
        reference = DesiredState(p=(15.0, 0.0, 1.0), v=(1.0, 0.0, 0.0))
        safe = filter_.apply(
            reference,
            TargetState(p=(0.0, 0.0, 0.0)),
            ObserverState(shared_p=(15.0, 0.0, 1.0), local_p=(15.0, 0.0, 1.0)),
            predicted_exit=False,
        )
        self.assertAlmostEqual(safe.p[0], 10.0)
        self.assertEqual(safe.p[2], 1.0)

    def test_visibility_safety_filter_retreats_outward_on_predicted_exit(self) -> None:
        filter_ = VisibilitySafetyFilter(min_distance=5.0, max_distance=10.0, retreat_speed=1.0)
        reference = DesiredState(p=(-6.0, 0.0, 0.0))
        safe = filter_.apply(
            reference,
            TargetState(p=(0.0, 0.0, 0.0)),
            ObserverState(shared_p=(-6.0, 0.0, 0.0), local_p=(-6.0, 0.0, 0.0)),
            predicted_exit=True,
            dt=0.05,
        )
        self.assertAlmostEqual(safe.p[0], -6.05)
        self.assertEqual(safe.p[2], 0.0)

    def test_visibility_safety_filter_keeps_inside_band_reference_unchanged(self) -> None:
        filter_ = VisibilitySafetyFilter(min_distance=5.0, max_distance=10.0)
        reference = DesiredState(p=(7.0, 0.0, 1.0), v=(0.2, 0.0, 0.0))
        safe = filter_.apply(
            reference,
            TargetState(p=(0.0, 0.0, 0.0)),
            ObserverState(shared_p=(7.0, 0.0, 1.0), local_p=(7.0, 0.0, 1.0)),
            predicted_exit=True,
            dt=0.0,
        )
        self.assertEqual(safe, reference)

    def test_visibility_safety_filter_does_not_snap_reference_across_the_band(self) -> None:
        filter_ = VisibilitySafetyFilter(min_distance=5.0, max_distance=10.0)
        reference = DesiredState(p=(-3.0, 0.0, 0.0))
        safe = filter_.apply(
            reference,
            TargetState(p=(0.0, 0.0, 0.0)),
            ObserverState(shared_p=(-3.0, 0.0, 0.0), local_p=(-3.0, 0.0, 0.0)),
            predicted_exit=True,
            dt=0.05,
        )
        self.assertAlmostEqual(safe.p[0], -3.05)


if __name__ == "__main__":
    unittest.main()
