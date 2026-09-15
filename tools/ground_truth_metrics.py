"""Read evaluation-only Gazebo laser coverage, independent of SLAM caches."""
import csv
import math
from pathlib import Path

import yaml


def read_ground_truth(run_robots, max_duration=None):
    bags = {Path(p).resolve() for p in run_robots.values()}
    if len(bags) != 1:
        return None  # Simulation records the team in one bag.
    bag = bags.pop()
    path = bag.parent / "ground_truth_coverage.csv"
    if not path.exists():
        return None
    if path.with_suffix(".csv.invalid").exists():
        raise ValueError(f"Invalid ground-truth recording: {path}")
    if not path.with_suffix(".csv.complete").exists():
        raise ValueError(f"Ground-truth recorder did not finish cleanly: {path}")
    meta = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    start_ns = int(meta["starting_time"]["nanoseconds_since_epoch"])
    horizon = meta["duration"]["nanoseconds"] / 1e9
    if max_duration is not None:
        horizon = min(horizon, max_duration)
    union, overlap = [(0.0, 0.0)], [(0.0, 0.0)]
    own = {robot: [(0.0, 0.0)] for robot in run_robots}
    observed, resolutions = set(), set()
    counts = {}
    last_union, last_overlap, last_wall = 0, 0, -1
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            robot = row["robot"]
            wall, res = int(row["wall_ns"]), float(row["resolution_m"])
            u, o, local = (int(row[k]) for k in ("union_cells", "overlap_cells", "observed_cells"))
            if (robot not in own or not math.isfinite(res) or res <= 0 or wall < last_wall
                    or u < last_union or o < last_overlap or not 0 <= o <= u
                    or not counts.get(robot, 0) <= local <= u or int(row["scans"]) < 1):
                raise ValueError(f"Inconsistent ground-truth row: {path}")
            observed.add(robot)
            resolutions.add(res)
            counts[robot] = local
            last_union, last_overlap, last_wall = u, o, wall
            t = (wall - start_ns) / 1e9
            if t > horizon:
                continue
            # Scans received just before the first bag message form the t=0 state.
            t = max(0.0, t)
            union.append((t, u * res**2))
            overlap.append((t, o * res**2))
            own[robot].append((t, local * res**2))
    if observed != set(own) or len(resolutions) != 1 or len(union) == 1:
        raise ValueError(f"Incomplete ground-truth recording: {path}")
    return {"union": union, "overlap": overlap, "local": own,
            "resolution_m": resolutions.pop(), "source": str(path)}


def load_comparison(robot_paths, max_duration=None):
    result = {c: [read_ground_truth(run, max_duration) for run in runs]
              for c, runs in robot_paths.items()}
    present = [run is not None for runs in result.values() for run in runs]
    if not any(present):
        return None
    if not all(present):
        raise ValueError("Ground-truth coverage is missing from some selected runs; "
                         "compare runs recorded with the evaluator enabled together")
    if len({run["resolution_m"] for runs in result.values() for run in runs}) != 1:
        raise ValueError("Ground-truth grid resolution differs between runs")
    return result
