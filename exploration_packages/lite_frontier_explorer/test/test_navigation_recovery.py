"""Exercise navigation outcomes without a running action server."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / 'lite_frontier_explorer/frontier_node.py'
        tree = ast.parse(path.read_text())
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        methods = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in
                   ('_on_result', '_on_goal_response', '_blacklist_pending_goal', '_is_blacklisted')]
        cls = ast.ClassDef(name='Explorer', bases=[], keywords=[], body=methods, decorator_list=[])
        scope = {'math': math, 'GoalStatus': NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5)}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                     '<recovery>', 'exec'), scope)
        self.robot = scope['Explorer']()
        self.time = 10.
        self.robot.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(self.time * 1e9)))
        self.robot.get_logger = lambda: Mock()
        self.robot._goal_blacklist_duration_s = 60.
        self.robot._goal_blacklist_radius_m = 1.
        self.robot._blacklist_expiry = {}
        self.robot._blacklisted_goals = []
        self.robot._pending_goal_xy = (2., 3.)
        self.robot._preempting = False
        self.robot._abandoning_stuck = False

    def test_abort_defers_target_but_does_not_erase_it_forever(self):
        self.robot._on_result(NS(result=lambda: NS(status=6)))
        self.assertTrue(self.robot._is_blacklisted((2., 3.)))
        self.assertFalse(self.robot._is_blacklisted((5., 6.)))
        self.time = 70.
        self.assertFalse(self.robot._is_blacklisted((2., 3.)))
        self.assertEqual(self.robot._blacklisted_goals, [])
        self.assertFalse(self.robot._goal_active)

    def test_cancellation_does_not_classify_frontier_as_unreachable(self):
        self.robot._on_result(NS(result=lambda: NS(status=5)))
        self.assertFalse(self.robot._is_blacklisted((2., 3.)))
        self.assertIsNone(self.robot._pending_goal_xy)

    def test_server_rejection_does_not_poison_map(self):
        self.robot._on_goal_response(NS(result=lambda: NS(accepted=False)))
        self.assertFalse(self.robot._is_blacklisted((2., 3.)))
        self.assertFalse(self.robot._goal_active)
        self.assertIsNone(self.robot._pending_goal_xy)

    def test_explicit_legacy_watchdog_abandonment_is_temporary(self):
        self.robot._abandoning_stuck = True
        self.robot._on_result(NS(result=lambda: NS(status=5)))
        self.assertTrue(self.robot._is_blacklisted((2., 3.)))
        self.time = 71.
        self.assertFalse(self.robot._is_blacklisted((2., 3.)))


if __name__ == '__main__':
    unittest.main()
