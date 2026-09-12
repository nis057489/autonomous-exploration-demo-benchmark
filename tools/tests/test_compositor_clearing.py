"""Map-frame and evidence-preservation regressions for robot self-clearing."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np


class Grid:
    def __init__(self):
        self.header = NS(frame_id='map')
        self.info = NS(width=0, height=0, resolution=.1, origin=NS(
            position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
        self.data = []


def compositor_type():
    path = Path(__file__).resolve().parents[2] / (
        'simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/scripts/per_robot_map_compositor.py')
    tree = ast.parse(path.read_text())
    definitions = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))]
    scope = dict(math=math, np=np, OccupancyGrid=Grid, Node=object, Time=lambda: 0,
                 tf2_ros=NS(LookupException=LookupError, ConnectivityException=ConnectionError,
                            ExtrapolationException=TimeoutError))
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), 'exec'), scope)
    return scope['PerRobotMapCompositor']


def grid(values):
    msg = Grid()
    msg.info.height, msg.info.width = values.shape
    msg.data = values.ravel().tolist()
    return msg


class CompositorClearingTests(unittest.TestCase):
    def node(self, x=1.55, y=1.55):
        cls = compositor_type()
        node = cls.__new__(cls)
        node._robot_name = 'robot2'
        node._offset_x = node._offset_y = node._offset_yaw = 0.
        node._self_clear_radius_m = .3
        node._tf_buffer = NS(lookup_transform=Mock(return_value=NS(
            transform=NS(translation=NS(x=x, y=y)))))
        return node

    def test_global_tf_pose_is_not_transformed_again(self):
        node = self.node(3.55, 4.55)
        node._offset_x, node._offset_y, node._offset_yaw = 4., 16., -math.pi/2
        canvas = np.full((200, 200), 100, dtype=np.int8)
        node._clear_self_in_canvas(canvas, 0., 0., .1, 'shared_map')
        node._tf_buffer.lookup_transform.assert_called_once_with('shared_map', 'robot2/base_footprint', 0)
        rows, cols = np.where(canvas == 0)
        self.assertGreater(len(rows), 0)
        distances = np.hypot((cols+.5)*.1-3.55, (rows+.5)*.1-4.55)
        self.assertTrue(np.all(distances <= .3 + 1e-12))
        self.assertEqual(canvas[124, 85], 100)  # Near the old double-transformed disc.

    def test_unknown_cells_and_remote_walls_are_preserved(self):
        node = self.node()
        canvas = np.full((30, 30), -1, dtype=np.int8)
        canvas[15, 15] = 100
        canvas[15, 23] = 100  # Previously inside the one-metre clearing disc.
        node._clear_self_in_canvas(canvas, 0., 0., .1)
        self.assertEqual(canvas[15, 15], 0)
        self.assertEqual(canvas[15, 23], 100)
        self.assertEqual(np.count_nonzero(canvas == -1), 898)

    def test_local_observed_wall_wins_over_peer_clearing(self):
        node = self.node()
        local = np.full((30, 30), -1, dtype=np.int8)
        local[15, 15] = 100
        team = np.full((30, 30), 100, dtype=np.int8)
        out = node._merge_local_onto_team(grid(local), grid(team))
        values = np.array(out.data).reshape(out.info.height, out.info.width)
        self.assertEqual(values[15, 15], 100)
        self.assertEqual(values[15, 16], 0)  # Peer obstacle within footprint cleared.

    def test_local_only_map_does_not_have_obstacles_erased(self):
        node = self.node()
        local = np.full((30, 30), -1, dtype=np.int8)
        local[15, 15] = 100
        out = node._local_to_global(grid(local))
        values = np.array(out.data).reshape(out.info.height, out.info.width)
        self.assertEqual(values[15, 15], 100)
        node._tf_buffer.lookup_transform.assert_not_called()

    def test_missing_transform_skips_clearing(self):
        node = self.node()
        node._tf_buffer.lookup_transform.side_effect = LookupError()
        canvas = np.full((30, 30), 100, dtype=np.int8)
        node._clear_self_in_canvas(canvas, 0., 0., .1)
        self.assertTrue(np.all(canvas == 100))


if __name__ == '__main__':
    unittest.main()
