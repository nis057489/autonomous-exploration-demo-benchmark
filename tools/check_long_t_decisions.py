#!/usr/bin/env python3
"""Idealized decision check using the real selector and long_t collision boxes.

This is NOT a Gazebo mission result. Perfect localization, 360-degree raster
lidar, no other robot bodies, inflation, SLAM, controller or action preemption.
Each robot first scans a prescribed straight approach in its own start branch.
Then its policy runs independently with either local data or the union of those
three approach maps. Checks whether it revisits a peer's start before entering
the work spine, while the six work branches remain substantially unexplored.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'exploration_packages/lite_frontier_explorer'))
from lite_frontier_explorer.frontier_detection import (  # noqa: E402
    _free_space_distances_with_parents, _visibility_rays,
    find_frontier_clusters, select_visible_gain_frontier,
)


def collision_boxes():
    world = ET.parse(ROOT / 'simulation/worlds/long_t/long_t.sdf')
    result = {}
    for link_name in ('floor', 'walls'):
        boxes = []
        for collision in world.findall(f'.//link[@name="{link_name}"]/collision'):
            x, y, z, roll, pitch, yaw = map(float, collision.findtext('pose').split())
            assert roll == pitch == yaw == 0
            sx, sy, sz = map(float, collision.findtext('geometry/box/size').split())
            boxes.append((x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2))
        result[link_name] = boxes
    return result


class Scene:
    def __init__(self, resolution=0.1):
        self.resolution = resolution
        boxes = collision_boxes()
        self.ox = math.floor(min(b[0] for b in boxes['walls'])) - 1
        self.oy = math.floor(min(b[1] for b in boxes['walls'])) - 1
        width = math.ceil((max(b[2] for b in boxes['walls']) + 1 - self.ox) / resolution)
        height = math.ceil((max(b[3] for b in boxes['walls']) + 1 - self.oy) / resolution)
        self.xs = self.ox + (np.arange(width) + 0.5) * resolution
        self.ys = self.oy + (np.arange(height) + 0.5) * resolution
        self.truth = np.full((height, width), 100, dtype=np.int8)
        for kind, value in [('floor', 0), ('walls', 100)]:
            for bounds in boxes[kind]:
                self.truth[self.mask(bounds)] = value

    def mask(self, bounds):
        x0, y0, x1, y1 = bounds
        return ((self.ys[:, None] >= y0) & (self.ys[:, None] < y1)
                & (self.xs >= x0) & (self.xs < x1))

    def cell(self, x, y):
        return math.floor((y - self.oy) / self.resolution), math.floor((x - self.ox) / self.resolution)

    def observe(self, local, x, y, radius):
        row, col = self.cell(x, y)
        dr, dc = _visibility_rays(radius / self.resolution)
        rows, cols = row + dr, col + dc
        h, w = local.shape
        valid = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        rows, cols = np.clip(rows, 0, h - 1), np.clip(cols, 0, w - 1)
        blocked = (self.truth[rows, cols] == 100) | ~valid
        # A sensor observes the first wall cell, but nothing behind it.
        before = np.concatenate((np.zeros((len(rows), 1), dtype=bool),
                                 np.maximum.accumulate(blocked, axis=1)[:, :-1]), axis=1)
        seen = valid & ~before
        local[rows[seen], cols[seen]] = self.truth[rows[seen], cols[seen]]

    def approaches(self, starts, radius):
        maps = []
        for start in starts:
            local = np.full_like(self.truth, -1)
            for y in np.arange(start['y'], 0.49, -0.5):
                self.observe(local, start['x'], y, radius)
            maps.append(local)
        return maps

    def trace(self, local, shared, start, radius, max_goals=12,
              assignment='independent', robot_index=0, team_size=3):
        local = local.copy()
        pos = (start['x'], 0.5)
        distance = 0.0
        goals = []
        for _ in range(max_goals):
            plan = local if shared is None else np.maximum(local, shared)
            h, w = plan.shape
            clusters = find_frontier_clusters(plan.ravel(), w, h, min_size=2)
            goal = select_visible_gain_frontier(
                clusters, plan.ravel(), w, h, *pos, self.resolution, self.ox, self.oy,
                min_distance_m=2, sensor_range_m=3, assignment_mode=assignment,
                robot_index=robot_index, team_size=team_size)
            if goal is None:
                break
            goals.append(goal)
            _, pr, pc = _free_space_distances_with_parents(
                plan.ravel(), w, h, *pos, self.resolution, self.ox, self.oy, 99)
            row, col = self.cell(*goal)
            path = []
            while pr[row, col] >= 0:
                path.append((row, col))
                row, col = int(pr[row, col]), int(pc[row, col])
            for step, (row, col) in enumerate(reversed(path)):
                pos = (float(self.xs[col]), float(self.ys[row]))
                distance += self.resolution
                # Gate is 0.5 m inside a start branch, past the hall's y=2 edge.
                outcome = ('peer_start' if pos[1] > 2.5 and abs(pos[0] - start['x']) > 2
                           else 'work_spine' if pos[1] < -4 else None)
                if outcome:
                    return dict(outcome=outcome, path_m=round(distance, 2), goals=goals, exit_xy=pos)
                if step % max(1, round(0.5 / self.resolution)) == 0:
                    self.observe(local, *pos, radius)
            self.observe(local, *pos, radius)
        return dict(outcome='undecided', path_m=round(distance, 2), goals=goals, exit_xy=pos)


def check(ranges=(3.5, 10.0), assignment='independent'):
    scene = Scene()
    metadata = json.loads((ROOT / 'simulation/worlds/long_t/geometry.json').read_text())
    rows = []
    for radius in ranges:
        maps = scene.approaches(metadata['starts'], radius)
        union = np.maximum.reduce(maps)
        unexplored = int(np.count_nonzero((scene.truth == 0) & (union == -1))) * scene.resolution ** 2
        for index, (start, local) in enumerate(zip(metadata['starts'], maps), 1):
            for arm, shared in [('none', None), ('oracle', union)]:
                result = scene.trace(local, shared, start, radius, assignment=assignment,
                                     robot_index=index-1, team_size=len(maps))
                rows.append(dict(sensor_range_m=radius, robot=index, arm=arm,
                                 team_unobserved_m2=round(unexplored, 2), **result))
    return dict(kind='idealized_policy_diagnostic_not_mission_measurement',
                frontier_assignment=assignment, results=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json-out', type=Path)
    parser.add_argument('--assignment', choices=('independent', 'robot_rank'), default='robot_rank')
    args = parser.parse_args()
    result = check(assignment=args.assignment)
    for row in result['results']:
        print(f"range={row['sensor_range_m']:4.1f} robot{row['robot']} {row['arm']:6s} "
              f"-> {row['outcome']:10s}; {row['path_m']:5.1f} m to exit; "
              f"{row['team_unobserved_m2']:.1f} m² not yet observed by team")
    if args.json_out:
        args.json_out.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
