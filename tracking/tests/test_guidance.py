import unittest

from tracking.guidance import ObserverState, PositionTrackerV0, TargetState


class PositionTrackerV0Test(unittest.TestCase):
    def test_static_target_uses_fixed_world_offset(self) -> None:
        tracker = PositionTrackerV0(relative_offset=(-3.0, 0.0, 1.0), yaw=0.25)

        desired = tracker.generate(
            TargetState(p=(10.0, 5.0, 2.0)),
            ObserverState(shared_p=(6.0, 5.0, 3.0), local_p=(1.0, 2.0, 3.0)),
        )

        self.assertEqual(desired.p, (2.0, 2.0, 3.0))
        self.assertEqual(desired.v, (0.0, 0.0, 0.0))
        self.assertEqual(desired.yaw, 0.25)

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


if __name__ == "__main__":
    unittest.main()
