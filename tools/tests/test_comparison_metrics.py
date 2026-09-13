"""Coverage geometry and metric semantics, without requiring ROS."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import unittest
import tempfile
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import generate_comparison_figure as metrics
from tools.export_run_summary import load_summary, StaleSummary


def check_aligned_grid(origin):
    msg = NS(data=[0] * 36, info=NS(width=6, height=6, resolution=1.0,
             origin=NS(position=NS(x=origin, y=origin),
                       orientation=NS(x=0, y=0, z=0, w=1))))
    assert len(metrics.known_cells(msg)) == 36
    # Moving a local origin and compensating with the spawn transform
    # must preserve the shared-frame keys.
    expected = metrics.known_cells(msg)
    msg.info.origin.position.x += 4
    np.testing.assert_array_equal(metrics.known_cells(msg, (-4, 0, 0)), expected)


def entry(events, nav_events=()):
    return (0, 0, [], [], [(t, np.array(cells, dtype=np.int64))
                          for t, cells in events],
            [(t, np.array(cells, dtype=np.int64)) for t, cells in nav_events],
            1.0, {})


class ComparisonMetricsTest(unittest.TestCase):
    def test_complete_sharing_is_100_despite_overlap(self):
        run = {'a': entry([(1, [1, 2]), (2, [3])], [(1, [1, 2, 3])]),
               'b': entry([(1, [2, 3])], [(1, [1, 2, 3])])}
        assert metrics.delivery_fraction_series(run) == [(1, 100.0), (2, 100.0)]

    def test_partial_sharing_excludes_own_ground_from_denominator(self):
        run = {'a': entry([(1, [1, 2]), (2, [3])], [(1, [1, 2, 3, 99])]),
               'b': entry([(1, [2, 3, 4])], [(1, [2, 3, 4])])}
        robots, events = metrics.peer_derived_series(run)
        assert robots == ['a', 'b']
        # Shared cell 2 and unobserved fusion cell 99 must not inflate delivery.
        assert events == [(1, (1, 0), (2, 1)), (2, (0, 0), (1, 1))]
        assert metrics.delivery_fraction_series(run) == [(1, 25.0), (2, 0.0)]

    def test_no_sharing_has_zero_completeness_when_peer_ground_is_missing(self):
        run = {'a': entry([(1, [1, 2])], [(1, [1, 2])]),
               'b': entry([(1, [2, 3])], [(1, [2, 3])])}
        assert metrics.delivery_fraction_series(run) == [(1, 0.0)]

    def test_no_eligible_ground_is_complete(self):
        run = {'a': entry([(1, [1])]), 'b': entry([(1, [1])])}
        assert metrics.delivery_fraction_series(run) == [(1, 100.0)]
        assert metrics.delivery_fraction_series({'a': run['a']}) == [(1, 100.0)]

    def test_empty_relay_namespaces_do_not_change_completeness(self):
        run = {'a': entry([(1, [1])], [(1, [1, 2])]),
               'b': entry([(1, [2])], [(1, [1, 2])]),
               'ddil_proxy_a_from_b': entry([])}
        assert metrics.peer_derived_series(run)[0] == ['a', 'b']
        assert metrics.delivery_fraction_series(run) == [(1, 100.0)]

    def test_robot_discovery_excludes_relay_nodes(self):
        # Robot discovery is headless; stub only the optional GUI toolkit.
        with patch.dict(sys.modules, {'tkinter': NS(Tk=object,
                         ttk=NS(Frame=object), messagebox=NS(), font=NS())}):
            from replay_gui import robots_in_bag
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / 'metadata.yaml').write_text(
                '  name: /robot1/map\n  name: /robot2/map\n'
                '  name: /ddil_proxy_robot1_from_robot2/ddil_stats\n'
                '  name: /robot1/nav_map\n')
            assert robots_in_bag(directory) == ['robot1', 'robot2']

    def test_overlap_counts_territory_once_and_batches_simultaneous_events(self):
        run = {'a': entry([(1, [1, 2]), (2, [1])]),
               'b': entry([(1, [1, 3])]),
               'c': entry([(1, [1]), (2, [2])])}
        assert metrics.run_redundant_series(run, 4) == [(1, 1), (2, 2)]
        assert metrics.local_physical_union_series(run)[-1][1] == 3


    def test_disjoint_maps_have_no_overlap(self):
        run = {'a': entry([(1, [1])]), 'b': entry([(1, [2])])}
        assert metrics.run_redundant_series(run, 4) == [(1, 0)]


    def test_aligned_grids(self):
        for origin in (0.0, -3.0, 0.25):
            with self.subTest(origin=origin):
                check_aligned_grid(origin)

    def test_old_coordinate_caches_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'old.npz'
            np.savez(path, meta='{"version": 3}')
            with self.assertRaises(StaleSummary):
                load_summary(path)


if __name__ == '__main__':
    unittest.main()
