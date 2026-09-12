"""Exercise the real planning tick on delivered maps without a ROS runtime."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from lite_frontier_explorer import frontier_detection as detection
from lite_frontier_explorer.reservations import GoalReservations


def planning_tick(selector=None):
    tree = ast.parse((PACKAGE / 'lite_frontier_explorer/frontier_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    tick = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_tick')
    scope = {**vars(detection), 'math': math}
    if selector is not None:
        scope['select_visible_gain_frontier'] = selector
    exec(compile(ast.Module(body=[tick], type_ignores=[]), '<frontier_node>', 'exec'), scope)
    return scope['_tick']


class CommunicationReplanningTests(unittest.TestCase):
    def robot(self, grid):
        return NS(
            _latest_costmap=NS(data=grid.ravel(), info=NS(width=grid.shape[1],
                height=grid.shape[0], resolution=1., origin=NS(position=NS(x=0., y=0.)))),
            _information_map_topic='', _reservation_topic='',
            _reservation_radius_m=6.0, _min_gain_m2=0.5,
            _lookup_robot_pose=lambda: (4.5, 7.5), _explore_start_delay_s=0.,
            _occ_threshold=65, _min_frontier_size=1, _is_blacklisted=lambda xy: False,
            _min_frontier_distance_m=0., _selection_strategy='visible_gain',
            _rank_assignment=True, _path_occ_threshold=99, _sensor_range_m=3.,
            _gain_distance_weight=1., _gain_max_viewpoints=5, _goal_active=True,
            _pending_goal_xy=(4.5, 4.5), _robot_index=0, _team_size=3,
            _invalid_goal_key=None, _invalid_goal_last_map=None, _invalid_goal_count=0,
            _blacklisted_goals=[], _goal_blacklist_radius_m=1.,
            get_logger=lambda: Mock(), _publish_frontier_markers=Mock(),
            _goal_stuck_timeout_s=0., _preempting=False, _abandoning_stuck=False,
            _goal_preempt_distance_m=1., _goal_preempt_utility_ratio=1.25,
            _max_consecutive_preemptions=3, _preempt_streak=3, _goal_handle=Mock())

    def test_occupancy_delivery_revalues_partial_goal_after_switching_budget(self):
        grid = np.full((10, 20), 100, dtype=np.int8)
        grid[7:, :] = 0
        grid[6, 2] = 0
        grid[5, 2] = -1
        grid[6, 10:18] = 0
        grid[:6, 10:18] = -1
        robot = self.robot(grid)
        robot._rank_assignment = False
        robot._lookup_robot_pose = lambda: (2.5, 8.5)
        robot._pending_goal_xy = detection.select_visible_gain_frontier(
            detection.find_frontier_clusters(grid.ravel(), 20, 10, min_size=1),
            grid.ravel(), 20, 10, 2.5, 8.5, 1., 0., 0., sensor_range_m=5.)
        self.assertGreaterEqual(robot._pending_goal_xy[0], 10)
        robot._sensor_range_m = 5.
        robot._information_map_topic = '/robot2/nav_map'
        robot._information_occ_threshold = 50
        robot._latest_costmap.header = NS(frame_id='map')
        robot._latest_costmap.info.origin.orientation = NS(x=0, y=0, z=0, w=1)
        def deliver(data):
            robot._latest_information_map = NS(header=NS(frame_id='map'),
                info=robot._latest_costmap.info, data=data.ravel())
        deliver(grid.copy())
        tick = planning_tick()
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()
        # Only occupancy delivery changes. Nav2, pose, reservations, active
        # goal and exhausted preemption budget remain exactly as before.
        received = grid.copy()
        received[:6, 10:18] = 0
        received[5, int(robot._pending_goal_xy[0])] = -1  # Still above threshold.
        deliver(received)
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_called_once()
        self.assertEqual(robot._blacklisted_goals, [])

    def test_hysteresis_still_applies_after_switching_budget(self):
        def selector(*args, diagnostics, active_score, **kwargs):
            diagnostics.append(dict(x=1.5, y=1.5, gain_m2=2., path_m=1., utility=1.1,
                                    cluster_rank=0, assignment_fallback=False))
            active_score.update(gain_m2=2., path_m=1., utility=1.)
            return (1.5, 1.5)
        robot = self.robot(np.zeros((9, 9), dtype=np.int8))
        planning_tick(selector)(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()

    def test_fresh_occupancy_can_confirm_invalid_goal_without_nav2_update(self):
        robot = self.robot(np.zeros((9, 9), dtype=np.int8))
        robot._rank_assignment = False
        robot._pending_goal_xy = (6.5, 4.5)
        robot._information_map_topic = '/robot2/nav_map'
        robot._information_occ_threshold = 50
        robot._latest_costmap.header = NS(frame_id='map')
        robot._latest_costmap.info.origin.orientation = NS(x=0, y=0, z=0, w=1)
        info = np.zeros((9, 9), dtype=np.int8)
        info[:, 5] = 56
        info[0, 2] = -1
        robot._latest_information_map = NS(header=NS(frame_id='map'),
            info=robot._latest_costmap.info, data=info.ravel())
        tick = planning_tick()
        tick(robot)
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()
        robot._latest_information_map = NS(**vars(robot._latest_information_map))
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_called_once()
        self.assertEqual(robot._blacklisted_goals, [])

    def test_peer_update_cancels_committed_rank_goal_without_alternative(self):
        grid = np.zeros((9, 9), dtype=np.int8)
        grid[:4, :] = -1
        robot = self.robot(grid)
        tick = planning_tick()
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()
        # Same robot pose, goal and policy: only newly delivered knowledge differs.
        grid[:4, :] = 0
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_called_once()
        self.assertTrue(robot._preempting)
        self.assertEqual(robot._preempt_streak, 0)
        self.assertEqual(robot._blacklisted_goals, [])

    def test_rank_assignment_can_leave_recorded_low_utility_detour(self):
        # Values from robot2 in 20260912_085742_vxch_long_t: a newer owned
        # destination had nearly twice the utility, but rank mode blocked it.
        def selector(*args, diagnostics, active_score, **kwargs):
            diagnostics.append(dict(x=11.225, y=-10.825, gain_m2=11.28,
                                    path_m=13.65, utility=0.770,
                                    cluster_rank=4, assignment_fallback=False))
            active_score.update(gain_m2=11.9225, path_m=28.55, utility=0.403)
            return (11.225, -10.825)
        robot = self.robot(np.zeros((9, 9), dtype=np.int8))
        robot._pending_goal_xy = (5.575, 16.325)
        robot._preempt_streak = 0
        robot._goal_preempt_distance_m = 1.0
        robot._goal_preempt_utility_ratio = 1.25
        planning_tick(selector)(robot)
        robot._goal_handle.cancel_goal_async.assert_called_once()
        self.assertEqual(robot._preempt_streak, 1)

    def test_small_rank_utility_change_does_not_cause_churn(self):
        def selector(*args, diagnostics, active_score, **kwargs):
            diagnostics.append(dict(x=1.5, y=1.5, gain_m2=2., path_m=1.,
                                    utility=1.1, cluster_rank=4,
                                    assignment_fallback=False))
            active_score.update(gain_m2=2., path_m=1., utility=1.)
            return (1.5, 1.5)
        robot = self.robot(np.zeros((9, 9), dtype=np.int8))
        robot._preempt_streak = 0
        robot._goal_preempt_distance_m = 1.0
        robot._goal_preempt_utility_ratio = 1.25
        planning_tick(selector)(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()

    def test_owned_work_is_compared_by_travel_cost(self):
        grid = np.full((10, 40), 100, dtype=np.int8)
        grid[7:, :] = 0
        for col in (2, 10, 30):
            grid[6, col] = 0
            grid[5, col] = -1
        clusters = detection.find_frontier_clusters(grid.ravel(), 40, 10, min_size=1)
        # Robot0 owns ranks 0 and 2 with equal information gain. Rank 2 is
        # next to it; selecting rank 0 would require crossing the whole map.
        goal = detection.select_visible_gain_frontier(
            clusters, grid.ravel(), 40, 10, 30.5, 8.5, 1., 0., 0.,
            assignment_mode='robot_rank', robot_index=0, team_size=2)
        self.assertEqual(goal, (30.5, 6.5))

    def test_tick_uses_occupancy_map_to_cancel_phantom_wall_goal(self):
        costs = np.zeros((9, 9), dtype=np.int8)
        robot = self.robot(costs)
        robot._information_map_topic = '/robot2/nav_map'
        robot._information_occ_threshold = 50
        robot._latest_costmap.header = NS(frame_id='map')
        robot._latest_costmap.info.origin.orientation = NS(x=0, y=0, z=0, w=1)
        information = costs.copy()
        information[:, 6] = 56
        information[:, 7:] = -1
        robot._latest_information_map = NS(
            header=NS(frame_id='map'), info=robot._latest_costmap.info,
            data=information.ravel())
        robot._pending_goal_xy = (6.5, 4.5)
        planning_tick()(robot)
        robot._goal_handle.cancel_goal_async.assert_called_once()

    def test_simultaneous_claim_conflict_has_one_winner(self):
        def selector(*args, active_score, **kwargs):
            active_score.update(gain_m2=4., path_m=1., utility=2.)
            return None
        for peer, should_yield in ((0, True), (2, False)):
            robot = self.robot(np.zeros((9, 9), dtype=np.int8))
            robot._robot_index = 1
            robot._reservation_topic = '/robot2/explore/reservation'
            robot._reservations = GoalReservations()
            robot._reservations.receive(peer, 10, (4.5, 4.5), 10)
            robot.get_clock = lambda: NS(now=lambda: NS(nanoseconds=11_000_000_000))
            planning_tick(selector)(robot)
            self.assertEqual(robot._goal_handle.cancel_goal_async.called, should_yield)

    def test_residual_gain_cannot_lock_robot_into_recorded_long_return(self):
        def selector(*args, active_score, **kwargs):
            active_score.update(gain_m2=.295, path_m=30.5, utility=.0094)
            return None
        robot = self.robot(np.zeros((9, 9), dtype=np.int8))
        planning_tick(selector)(robot)
        robot._goal_handle.cancel_goal_async.assert_called_once()
        self.assertEqual(robot._preempt_streak, 0)

    def test_invalid_goal_replaced_after_two_fresh_maps_despite_commitment(self):
        def selector(*args, active_score, **kwargs):
            active_score.update(status='blocked_or_unknown')
            return (4.5, 1.5)
        robot = self.robot(np.zeros((9, 9), dtype=np.int8))
        tick = planning_tick(selector)
        tick(robot)
        tick(robot)  # Same map must not confirm invalidity twice.
        robot._goal_handle.cancel_goal_async.assert_not_called()
        robot._latest_costmap = NS(**vars(robot._latest_costmap))
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_called_once()
        self.assertEqual(robot._blacklisted_goals, [])
        self.assertEqual(robot._preempt_streak, 0)

    def test_invalidity_must_be_consecutive_and_have_an_alternative(self):
        status = ['unreachable']
        alternative = [None]
        def selector(*args, active_score, **kwargs):
            active_score.update(status=status[0])
            return alternative[0]
        robot = self.robot(np.zeros((9, 9), dtype=np.int8))
        tick = planning_tick(selector)
        tick(robot)
        robot._latest_costmap = NS(**vars(robot._latest_costmap))
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()
        status[0] = 'valid'
        tick(robot)
        status[0] = 'unreachable'
        alternative[0] = (4.5, 1.5)
        robot._latest_costmap = NS(**vars(robot._latest_costmap))
        tick(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()

    def test_partial_delivery_preserves_useful_goal(self):
        grid = np.zeros((9, 9), dtype=np.int8)
        grid[3, 4] = -1
        robot = self.robot(grid)
        planning_tick()(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()

    def test_unreachable_goal_is_not_evidence_of_completed_work(self):
        grid = np.zeros((9, 9), dtype=np.int8)
        grid[4, 4] = 100
        robot = self.robot(grid)
        planning_tick()(robot)
        robot._goal_handle.cancel_goal_async.assert_not_called()

    def test_completion_before_action_acceptance_marks_pending_cancellation(self):
        robot = self.robot(np.zeros((9, 9), dtype=np.int8))
        robot._goal_handle = None
        planning_tick()(robot)
        self.assertTrue(robot._preempting)


if __name__ == '__main__':
    unittest.main()
