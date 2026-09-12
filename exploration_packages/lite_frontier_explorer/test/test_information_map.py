"""Occupancy evidence must not be confused with Nav2 inflation costs."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite_frontier_explorer.frontier_detection import (
    align_information_grid, find_frontier_clusters, select_visible_gain_frontier,
    visible_unknown_area,
)


class InformationMapTests(unittest.TestCase):
    def test_received_wall_blocks_stale_navigation_route_and_reservation(self):
        nav = np.zeros((9, 13), dtype=np.int8)
        info = nav.copy()
        info[0, 2] = -1
        info[0, 10] = -1
        clusters = find_frontier_clusters(info.ravel(), 13, 9, min_size=1)
        def choose(information, **kwargs):
            return select_visible_gain_frontier(
                clusters, nav.ravel(), 13, 9, 2.5, 4.5, 1., 0., 0.,
                information_grid=information, information_occ_threshold=50,
                **kwargs)
        # A claim across the open hallway excludes all candidates.
        self.assertIsNone(choose(info, reserved_goals=[(10.5, 1.5)],
                                 reservation_radius_m=20.))
        # The received wall separates the claim from our reachable frontier.
        # Nav2 remains entirely free, deliberately simulating update lag.
        info[:, 6] = 56
        active = {}
        goal = choose(info, active_goal=(10.5, 1.5), active_score=active,
                      reserved_goals=[(10.5, 1.5)], reservation_radius_m=20.)
        self.assertIsNotNone(goal)
        self.assertLess(goal[0], 6)
        self.assertEqual(active['status'], 'unreachable')
        self.assertTrue(np.all(nav == 0))

    def test_probabilistic_wall_cannot_create_gain_outside_known_corridor(self):
        info = np.zeros((11, 11), dtype=np.int8)
        info[:, 6] = 56  # The recorded goal sat on a partially reconstructed wall.
        info[:, 7:] = -1
        self.assertGreater(visible_unknown_area(info, 5, 6, 1, 3), 0)
        self.assertEqual(visible_unknown_area(info, 5, 6, 1, 3, 50), 0)
        self.assertEqual(visible_unknown_area(info, 5, 5, 1, 3, 50), 0)
        clusters = find_frontier_clusters(info.ravel(), 11, 11, occ_threshold=50, min_size=1)
        self.assertEqual(clusters, [])
        score = {}
        goal = select_visible_gain_frontier(
            clusters, np.zeros(121, dtype=np.int8), 11, 11, 4.5, 5.5, 1, 0, 0,
            information_grid=info, information_occ_threshold=50,
            active_goal=(6.5, 5.5), active_score=score)
        self.assertIsNone(goal)
        self.assertEqual(score['gain_m2'], 0)

    def test_navigation_inflation_does_not_hide_real_information(self):
        info = np.zeros((9, 9), dtype=np.int8)
        info[:3, :] = -1
        costs = info.copy()
        costs[3:, :] = 90
        clusters = find_frontier_clusters(info.ravel(), 9, 9, min_size=1)
        self.assertIsNotNone(select_visible_gain_frontier(
            clusters, costs.ravel(), 9, 9, 4.5, 6.5, 1, 0, 0,
            information_grid=info, information_occ_threshold=50))
        costs[3, :] = 100
        self.assertIsNone(select_visible_gain_frontier(
            clusters, costs.ravel(), 9, 9, 4.5, 6.5, 1, 0, 0,
            information_grid=info, information_occ_threshold=50))

    def test_active_goal_invalidity_is_explicit_while_alternative_is_available(self):
        for expected in ('blocked_or_unknown', 'outside_map', 'unreachable'):
            with self.subTest(status=expected):
                costs = np.zeros((11, 11), dtype=np.int8)
                costs[0, :] = -1
                active = (5.5, 5.5)
                if expected == 'blocked_or_unknown':
                    costs[5, 5] = 100
                elif expected == 'outside_map':
                    active = (15.5, 5.5)
                else:
                    for row, col in ((4, 5), (6, 5), (5, 4), (5, 6)):
                        costs[row, col] = 100
                clusters = find_frontier_clusters(costs.ravel(), 11, 11, min_size=1)
                score = {}
                goal = select_visible_gain_frontier(
                    clusters, costs.ravel(), 11, 11, 2.5, 8.5, 1, 0, 0,
                    active_goal=active, active_score=score)
                self.assertEqual(score['status'], expected)
                self.assertNotIn('utility', score)
                self.assertIsNotNone(goal)

    def test_alignment_handles_resolution_origin_and_unknown_extent(self):
        def msg(data, width, height, resolution, x=0):
            return NS(data=data, header=NS(frame_id='map'), info=NS(
                width=width, height=height, resolution=resolution,
                origin=NS(position=NS(x=x, y=0), orientation=NS(x=0,y=0,z=0,w=1))))
        src = msg([0, 56], 2, 1, 1.)
        dst = msg([], 6, 2, .5, -.5)
        expected = [[-1, 0, 0, 56, 56, -1]] * 2
        np.testing.assert_array_equal(align_information_grid(src, dst), expected)
        dst.header.frame_id = 'other'
        with self.assertRaises(ValueError):
            align_information_grid(src, dst)


if __name__ == '__main__':
    unittest.main()
