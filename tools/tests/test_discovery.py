import ast
from pathlib import Path
import unittest
import numpy as np
from tools.analyze_discovery import discovery_series, report


def entry(events, end=10, resolution=1):
    return (0, 0, [], [(end, 0)], [(t, np.array(cells)) for t, cells in events], [], resolution)


class DiscoveryTests(unittest.TestCase):
    def test_team_discovery_counts_each_cell_once(self):
        run = {'a': entry([(1, [1, 2]), (2, [2, 3])]),
               'b': entry([(2, [2, 3, 4])])}
        self.assertEqual(discovery_series(run)[-1], (2, 4, 6, 2))

    def test_same_robot_same_timestamp_and_reobservation(self):
        run = {'a': entry([(1, [1]), (1, [1, 2]), (2, [1, 2])])}
        self.assertEqual(discovery_series(run), [(1, 2, 2, 0), (2, 2, 2, 0)])

    def test_rate_budget_and_unreached_target(self):
        run = {'a': entry([(1, [1, 2]), (5, [3, 4])], end=6)}
        result = report(run, [0, 4, 6, 7], [2, 5], 2)
        self.assertEqual(result['samples'][2]['discovery_m2_s'], 1)
        self.assertFalse(result['samples'][3]['observed'])
        self.assertEqual(result['first_passage_s'], {'2': 1, '5': None})

    def test_resolution_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            discovery_series({'a': entry([], resolution=1), 'b': entry([], resolution=.5)})


class PhysicalCoveragePlotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Exercise the actual plotting implementation without importing ROS,
        # which is only required to read bags, not to union their cell sets.
        source = Path(__file__).resolve().parents[2] / 'generate_comparison_figure.py'
        tree = ast.parse(source.read_text())
        names = {'union_coverage_over_time', 'run_resolution', 'to_area_m2',
                 'local_physical_union_series'}
        functions = [node for node in tree.body
                     if isinstance(node, ast.FunctionDef) and node.name in names]
        scope = {}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), scope)
        cls.physical = staticmethod(scope['local_physical_union_series'])

    def test_teammate_revisits_never_increase_physical_coverage(self):
        run = {'a': entry([(1, [1, 2]), (4, [1, 2])], resolution=.5),
               'b': entry([(2, [2, 3]), (5, [1, 2, 3])], resolution=.5)}
        self.assertEqual(self.physical(run), [(1, .5), (2, .75), (4, .75), (5, .75)])

    def test_out_of_order_events_and_simultaneous_observations(self):
        run = {'a': entry([(3, [10, 20]), (1, [10, 10])]),
               'b': entry([(1, [10]), (2, [10])])}
        self.assertEqual(self.physical(run), [(1, 1), (1, 1), (2, 1), (3, 2)])

    def test_received_maps_never_add_physical_cells(self):
        local = list(entry([(1, [7])]))
        local[5] = [(2, np.array([100, 101, 102]))]  # Received/fused cells.
        self.assertEqual(self.physical({'a': tuple(local)}), [(1, 1)])


if __name__ == '__main__':
    unittest.main()
