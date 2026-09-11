import tempfile
import unittest
from pathlib import Path
from tools.run_experiment_sweep import navigation_health


class NavigationHealthTests(unittest.TestCase):
    def test_startup_warning_is_not_a_runtime_stall(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'run.log'
            log.write_text('Still waiting for first costmap\nSending goal to frontier\n')
            self.assertFalse(navigation_health(log)['review_required'])

    def test_stall_remains_reported_after_other_goals(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'run.log'
            log.write_text('[robot1] NAVIGATION_STALL: oscillating\nSending goal to frontier\n')
            health = navigation_health(log)
            self.assertTrue(health['review_required'])
            self.assertEqual(health['stall_count'], 1)


if __name__ == '__main__':
    unittest.main()
