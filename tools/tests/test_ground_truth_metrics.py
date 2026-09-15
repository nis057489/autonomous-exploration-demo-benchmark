"""Ground-truth coverage semantics and sidecar compatibility."""
import csv
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from tools.ground_truth_metrics import load_comparison, read_ground_truth

ROOT = Path(__file__).resolve().parents[2]


class GroundTruthTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)
        self.bag = self.run / "bag"
        self.bag.mkdir()
        (self.bag / "metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n"
            "  starting_time: {nanoseconds_since_epoch: 1000000000}\n"
            "  duration: {nanoseconds: 10000000000}\n")
        self.robots = {"robot1": self.bag, "robot2": self.bag}

    def write_csv(self, rows=None):
        path = self.run / "ground_truth_coverage.csv"
        with path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["wall_ns", "sim_time_s", "robot", "scans", "observed_cells",
                             "union_cells", "overlap_cells", "resolution_m"])
            writer.writerows(rows or [
                [2000000000, 1, "robot1", 1, 4, 4, 0, .5],
                [3000000000, 2, "robot2", 1, 4, 6, 2, .5],
                [6000000000, 5, "robot1", 2, 4, 6, 2, .5]])
        path.with_suffix(".csv.complete").touch()
        return path

    def test_area_time_alignment_and_flat_observation_horizon(self):
        self.write_csv()
        data = read_ground_truth(self.robots)
        self.assertEqual(data["union"], [(0., 0.), (1., 1.), (2., 1.5), (5., 1.5)])
        self.assertEqual(data["overlap"][-1], (5., .5))
        self.assertEqual(read_ground_truth(self.robots, 2.5)["union"][-1], (2., 1.5))

    def test_old_bags_do_not_acquire_fabricated_ground_truth(self):
        self.assertIsNone(read_ground_truth(self.robots))
        self.assertIsNone(load_comparison({"none": [self.robots]}))

    def test_partial_team_and_failed_recorder_are_rejected(self):
        path = self.write_csv([[2000000000, 1, "robot1", 1, 4, 4, 0, .5]])
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            read_ground_truth(self.robots)
        self.write_csv()
        path.with_suffix(".csv.invalid").touch()
        with self.assertRaisesRegex(ValueError, "Invalid"):
            read_ground_truth(self.robots)

    def test_comparison_cannot_mix_old_and_new_runs(self):
        self.write_csv()
        other_bag = self.run / "other" / "bag"
        other_bag.mkdir(parents=True)
        with self.assertRaisesRegex(ValueError, "missing"):
            load_comparison({"none": [self.robots], "vxch": [{"robot1": other_bag}]})

    def test_interrupted_recording_is_not_accepted_as_complete(self):
        path = self.write_csv()
        path.with_suffix(".csv.complete").unlink()
        with self.assertRaisesRegex(ValueError, "finish cleanly"):
            read_ground_truth(self.robots)

    def test_export_keeps_truth_separate_from_slam(self):
        import numpy as np
        import generate_comparison_figure as figure
        self.write_csv()
        paths = {"none": [self.robots]}
        truth = load_comparison(paths)
        entry = (0, 0, [], [], [(0, np.array([1], dtype=np.int64))], [], 1., {})
        results = {"none": [{"robot1": entry, "robot2": entry}]}
        data = figure.figure_data(results, ["none"], {"none": {}}, paths, None, truth)
        self.assertEqual(data["series"]["union"]["none"][0][-1][1], 1.)
        self.assertEqual(data["series"]["ground_truth_union"]["none"][0][-1][1], 1.5)
        self.assertEqual(data["ground_truth"]["runs"]["none"][0]["resolution_m"], .5)

    @unittest.skipUnless(shutil.which("g++"), "C++ compiler required")
    def test_rays_deduplicate_across_time_and_robots_and_stop_at_hits(self):
        source = self.run / "rays.cpp"
        source.write_text(r'''
#include "observed_cells.hpp"
#include <cassert>
int main() {
  ObservedCells c(1.0, 3);
  c.ray(0, .5, .5, 3.5, .5);
  assert(c.union_count() == 3 && c.own[0] == 3 && c.overlap == 0);
  c.ray(0, .5, .5, 3.5, .5);  // Same robot revisits.
  assert(c.union_count() == 3 && c.own[0] == 3 && c.overlap == 0);
  c.ray(1, .5, .5, 3.5, .5);  // Another robot sees the same area.
  c.ray(2, .5, .5, 3.5, .5);  // Three owners still count overlap once.
  assert(c.union_count() == 3 && c.overlap == 3);
  c.ray(1, 3.5, .5, 5.5, .5);
  assert(c.union_count() == 5 && c.own[1] == 5 && c.overlap == 3);
  ObservedCells wall(1.0, 1);
  wall.ray(0, .5, .5, 1.1, .5);
  assert(wall.union_count() == 1);  // Hit cell and cells behind it excluded.
  ObservedCells negative(1.0, 1);
  negative.ray(0, .5, .5, -2.5, .5);
  assert(negative.union_count() == 3);
  ObservedCells diagonal(1.0, 1);
  diagonal.ray(0, .5, .5, 2.5, 2.5);
  assert(diagonal.union_count() == 2);  // No cells touched at corners only.
  diagonal.ray(0, .5, .5, .5, .5);
  assert(diagonal.union_count() == 2);
}
''')
        include = ROOT / "simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/scripts"
        executable = self.run / "rays"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I", str(include),
                        str(source), "-o", str(executable)], check=True, capture_output=True)
        subprocess.run([str(executable)], check=True, timeout=5)


if __name__ == "__main__":
    unittest.main()
