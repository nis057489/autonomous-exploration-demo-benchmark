"""Stagger controls goal release in ROS time, not mapping or wall time."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import unittest


class StartStaggerTests(unittest.TestCase):
    def test_hold_until_ready_then_release_once_at_deadline(self):
        path = Path(__file__).resolve().parents[1] / 'lite_frontier_explorer/frontier_node.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_start_allowed')
        scope = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<stagger>', 'exec'), scope)
        for delay in (60., 120.):
            now = [10.]
            ready = [False]
            logger = Mock()
            robot = NS(_nav_client=NS(server_is_ready=lambda: ready[0]),
                       _ready_since=None, _start_release_logged=False,
                       _explore_start_delay_s=delay, get_logger=lambda: logger,
                       get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(now[0]*1e9))))
            gate = scope['_start_allowed']
            self.assertFalse(gate(robot))
            self.assertIsNone(robot._ready_since)
            ready[0] = True
            now[0] = 20.
            self.assertFalse(gate(robot))
            now[0] = 20. + delay - .001
            self.assertFalse(gate(robot))
            # Repeated calls with a paused simulation clock do not advance it.
            self.assertFalse(gate(robot))
            now[0] = 20. + delay
            self.assertTrue(gate(robot))
            self.assertTrue(gate(robot))
            self.assertEqual(logger.info.call_count, 2)  # ready, released

    def test_launch_assigns_three_offsets_and_no_reservation_relays(self):
        root = Path(__file__).resolve().parents[3]
        path = root / 'rviz/launch/multi_robot_frontier_explorer.launch.py'
        tree = ast.parse(path.read_text())
        assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Subscript) and isinstance(t.slice, ast.Constant)
                    and t.slice.value == 'explore_start_delay_s' for t in n.targets))
        offsets = []
        for i in range(3):
            scope = {'generated_params': {}, 'index': i, 'start_stagger_s': 60.}
            exec(compile(ast.Module(body=[assignment], type_ignores=[]), '<launch>', 'exec'), scope)
            offsets.append(scope['generated_params']['explore_start_delay_s'])
        self.assertEqual(offsets, [0., 60., 120.])
        transport = root / 'simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/launch/multi_robot_vxch_experiment.launch.py'
        self.assertNotIn('/reservation', transport.read_text())


if __name__ == '__main__':
    unittest.main()
