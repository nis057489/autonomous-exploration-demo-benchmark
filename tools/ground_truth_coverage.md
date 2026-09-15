# Coverage from the simulator's sensor pose

Simulation runs with `RECORD_METRICS=true` start `ground_truth_coverage` alongside
the bag recorder. Build `bme_ros2_navigation` (or rebuild the Docker image) first.
The recorder subscribes to the existing native Gazebo laser topics. It does not
change the world, add sensors, or provide information to the robots.

Each Gazebo `LaserScan` supplies ranges and the sensor's `world_pose` at its
timestamp. The evaluator projects those rays onto a fixed world XY grid at
0.05 m resolution. It marks traversed cells between the minimum and measured
maximum range, excluding each beam's terminal cell so obstacle returns are not
counted as free area. Infinite positive ranges extend to the sensor's range
limit. NaNs and readings below minimum range contribute nothing. Each cell is
counted once per robot and once across the entire team; overlap counts cells
seen by at least two distinct robots once, even with three or more observers.

The output is `experiment_runs/<run>/ground_truth_coverage.csv`. It contains
native scan timestamps, receipt wall timestamps, scan counts, cumulative local
cell counts, team union and overlap, and lattice resolution. The comparison
script discovers it next to the bag and adds two separate panels:

- Ground-truth-pose observed union
- Ground-truth-pose observed overlap

The figure JSON includes these series and their source paths. Receipt time is
aligned to the bag's starting wall timestamp, just like the existing plots;
simulation timestamps remain in the CSV. `--max-duration` applies to both.
All compared runs must have complete sidecars at the same resolution to display
these panels. Existing bags without sidecars retain their SLAM-only plots;
their maps and tracked paths cannot reconstruct this metric retroactively.
An invalid recorder writes a `.csv.invalid` marker which analysis rejects.
Successful shutdown writes `.csv.complete`; interrupted or still-running
recordings without that marker are not accepted as completed evaluations.

This removes SLAM and wheel-odometry drift from the coverage metric. It is
**not noise-free geometric floor area**: sensor range noise remains as configured,
cells are discretized, and only received laser beams count. Free-space returns
from dynamic objects stop rays too. The evaluator supports a single horizontal
laser layer and projects it onto XY, using the full world orientation. It does
not infer visibility between beams or count all space in a sensor-radius disc.
Small range-noise variations can change boundary cells; a strict noise-free
visibility metric would require a separate ideal raycast evaluation.

Validation: `python3 -m unittest tools.tests.test_ground_truth_metrics`.
In a sourced Gazebo development environment, the native transport integration
test is `python3 tools/tests/run_ground_truth_transport_test.py
<path-to-ground_truth_coverage>`. It publishes synthetic scans with known world
translations and rotations and checks the resulting union/overlap counts.
