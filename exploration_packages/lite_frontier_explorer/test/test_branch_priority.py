"""Map-only branch choice remains stable under gain and small-hole variation."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite_frontier_explorer.branch_priority import BranchPriority, unknown_fraction
import test_communication_replanning as replanning


class BranchPriorityTests(unittest.TestCase):
    def policy(self):
        return BranchPriority([-16., 0., -8., 8.], [8., 0., 16., 8.])

    def map(self):
        grid = np.full((40, 40), 100, dtype=np.int8)
        grid[16:20, 1:39] = 0
        grid[20:28, 4:12] = -1
        grid[20:28, 28:36] = -1
        return grid

    def robot(self, grid):
        robot = replanning.CommunicationReplanningTests().robot(grid)
        robot._branch_policy = self.policy()
        robot._rank_assignment = False
        robot._goal_active = False
        robot._lookup_robot_pose = lambda: (8.5, -2.5)  # right is much nearer
        robot._information_map_topic = '/robot2/nav_map'
        robot._information_occ_threshold = 50
        robot._latest_costmap.header = NS(frame_id='map')
        robot._latest_costmap.info.origin.position = NS(x=-20., y=-20.)
        robot._latest_costmap.info.origin.orientation = NS(x=0., y=0., z=0., w=1.)
        robot._nav_client = Mock()
        robot._nav_client.server_is_ready.return_value = False
        self.deliver(robot, grid)
        return robot

    def deliver(self, robot, grid):
        robot._latest_information_map = NS(header=NS(frame_id='map'),
            info=robot._latest_costmap.info, data=grid.ravel())

    def test_small_map_extent_is_not_completed_branch(self):
        fraction = unknown_fraction(np.zeros((4, 4)), 4, 4, 1., 0., 0.,
                                    [0., 0., 8., 8.])
        self.assertEqual(fraction, 0.75)
        self.assertEqual(unknown_fraction(np.zeros((4, 4)), 4, 4, 1., 0., 0.,
                                         [-8., -8., -4., -4.]), 1.0)

    def test_left_choice_survives_pose_and_frontier_gain_variation(self):
        tick = replanning.planning_tick()
        for seed in range(12):
            with self.subTest(seed=seed):
                grid = self.map()
                rng = np.random.default_rng(seed)
                # Vary the left frontier edge and approach pose independently.
                for col in range(4, 12):
                    grid[20:20 + rng.integers(0, 3), col] = 0
                robot = self.robot(grid)
                robot._lookup_robot_pose = lambda: (float(rng.uniform(0., 12.)), -2.5)
                tick(robot)
                goal = robot._publish_frontier_markers.call_args.args[2]
                self.assertIsNotNone(goal)
                self.assertLess(goal[0], 0.)

    def test_delivery_switches_to_right_despite_small_slam_holes(self):
        grid = self.map()
        robot = self.robot(grid)
        tick = replanning.planning_tick()
        tick(robot)
        left_goal = robot._publish_frontier_markers.call_args.args[2]
        robot._goal_active = True
        robot._pending_goal_xy = left_goal
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()
        delivered = grid.copy()
        delivered[20:28, 4:12] = 0
        delivered[24, 6:9] = -1  # 3/64 unknown: below the 5% tolerance.
        # Only the delivered occupancy map changes; costmap and pose are fixed.
        self.deliver(robot, delivered)
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_called_once()
        self.assertEqual(robot._blacklisted_goals, [])
        self.assertEqual(robot._branch_policy.stage, 1)
        self.assertGreater(robot._publish_frontier_markers.call_args.args[2][0], 0.)
        # Returning noise cannot send the robot back to its completed branch.
        self.deliver(robot, grid)
        tick(robot)
        self.assertEqual(robot._branch_policy.stage, 1)

    def test_partial_delivery_does_not_complete_left(self):
        grid = self.map()
        grid[20:24, 4:12] = 0
        policy = self.policy()
        policy.update(grid, 40, 40, 1., -20., -20.)
        self.assertEqual(policy.stage, 0)
        self.assertEqual(policy.fractions[0], 0.5)

    def test_no_left_candidate_does_not_fall_back_to_right(self):
        policy = self.policy()
        self.assertEqual(policy.filter_clusters([[(19, 28)]], 1., -20.), [])

    def test_both_known_finishes_instead_of_chasing_holes(self):
        policy = self.policy()
        policy.update(np.zeros((40, 40)), 40, 40, 1., -20., -20.)
        self.assertEqual(policy.stage, 2)
        self.assertEqual(policy.filter_clusters([[(19, 4)], [(19, 28)]], 1., -20.), [])


if __name__ == '__main__':
    unittest.main()
