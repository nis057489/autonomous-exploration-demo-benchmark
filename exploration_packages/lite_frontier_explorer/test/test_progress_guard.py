import unittest
from lite_frontier_explorer.progress_guard import ProgressGuard


class ProgressGuardTests(unittest.TestCase):
    def test_oscillation_exceeding_old_epsilon_does_not_count_as_progress(self):
        guard = ProgressGuard()
        for t in range(0, 45, 3):
            self.assertIsNone(guard.update(t, (.3 * (t % 2), .4 * (t % 2)), True))
        self.assertIn('failed to leave', guard.update(45, (.3, .4), True))

    def test_detour_away_from_goal_is_still_progress(self):
        guard = ProgressGuard()
        for t in range(0, 180, 3):
            self.assertIsNone(guard.update(t, (-t * .1, 0), True))

    def test_rotation_or_stationary_pose_eventually_fails(self):
        guard = ProgressGuard()
        self.assertIsNone(guard.update(0, (0, 0), True))
        self.assertIsNotNone(guard.update(45, (0, 0), True))

    def test_unreachable_requires_consecutive_assessed_checks(self):
        guard = ProgressGuard()
        for t in range(3):
            self.assertIsNone(guard.update(t, (t, 0), False))
        self.assertIsNone(guard.update(3, (3, 0), None))
        for t in range(4, 7):
            self.assertIsNone(guard.update(t, (t, 0), False))
        self.assertIn('unreachable', guard.update(7, (7, 0), False))

    def test_clock_rollback_and_new_goal_reset(self):
        guard = ProgressGuard()
        guard.update(100, (0, 0), False)
        self.assertIsNone(guard.update(1, (0, 0), True))
        guard.reset()
        self.assertIsNone(guard.update(1000, (0, 0), True))


if __name__ == '__main__':
    unittest.main()
