"""Configured NavFn clearance must reject gaps narrower than the turning envelope."""
import ast
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
NAV = ROOT / 'simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation'
sys.path.insert(0, str(ROOT / 'exploration_packages/lite_frontier_explorer'))
from lite_frontier_explorer.frontier_detection import _free_space_distances


def radii(points, padding):
    # Nav2 pads each coordinate away from the footprint origin.
    points = np.array(points, dtype=float)
    points += np.sign(points) * padding
    edges = np.roll(points, -1, axis=0) - points
    projections = np.clip(-np.sum(points * edges, axis=1) / np.sum(edges**2, axis=1), 0, 1)
    return (np.linalg.norm(points + projections[:, None] * edges, axis=1).min(),
            np.linalg.norm(points, axis=1).max())


class NavigationClearanceTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((NAV / 'config/navigation.yaml').read_text())
        self.local = self.config['local_costmap']['local_costmap']['ros__parameters']
        self.global_ = self.config['global_costmap']['global_costmap']['ros__parameters']

    def test_launch_and_costmaps_agree_on_conservative_envelope(self):
        tree = ast.parse((NAV / 'launch/multi_robot_navigation_with_slam.launch.py').read_text())
        override = next(ast.literal_eval(n.value) for n in tree.body
            if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name)
                and t.id == '_MOGI_BOT_FOOTPRINT' for t in n.targets))
        for params in (self.local, self.global_):
            self.assertEqual(params['footprint'], override['footprint'])
            self.assertEqual(params['footprint_padding'], override['footprint_padding'])
            inner, outer = radii(json.loads(params['footprint']), params['footprint_padding'])
            self.assertGreater(inner, math.hypot(.23, .20))
            self.assertLessEqual(outer, params['inflation_layer']['inflation_radius'])
            self.assertLess(inner, .35)  # Preserve the checked office circulation routes.

    def test_narrow_cubicle_route_is_rejected_but_hallway_remains_open(self):
        inner, _ = radii(json.loads(self.global_['footprint']), self.global_['footprint_padding'])
        def reachable(gap_cells, clearance):
            # Wall separates robot and destination, with one central opening.
            occupied = np.zeros((61, 81), dtype=bool)
            occupied[:, 40] = True
            occupied[30-gap_cells//2:30+(gap_cells+1)//2, 40] = False
            rr, cc = np.indices(occupied.shape)
            blocked = occupied.copy()
            for r, c in np.argwhere(occupied):
                blocked |= (rr-r)**2 + (cc-c)**2 <= (clearance/.05)**2
            costs = np.where(blocked, 99, 0).astype(np.int8)
            distances = _free_space_distances(costs.ravel(), 81, 61,
                .525, 1.525, .05, 0., 0., 99)
            return distances[30, 70] >= 0
        self.assertTrue(reachable(10, .20))  # Old footprint admits a 0.5 m gap.
        self.assertFalse(reachable(10, inner))
        self.assertTrue(reachable(18, inner))  # A 0.9 m passage remains usable.


if __name__ == '__main__':
    unittest.main()
