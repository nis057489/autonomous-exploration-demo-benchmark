"""Coverage geometry and metric semantics, without requiring ROS."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import unittest
import tempfile

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


def entry(events):
    return (0, 0, [], [], [(t, np.array(cells, dtype=np.int64))
                          for t, cells in events], [], 1.0, {})


class ComparisonMetricsTest(unittest.TestCase):
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
