"""Ordered branch exploration for controlled, known-layout experiments.

Geometry defines regions to inspect, not their observed state. Completion uses
only the robot's delivered occupancy map, including unknown cells beyond its
current extent. Completed branches stay completed to tolerate SLAM flicker.
"""
import math

import numpy as np


def unknown_fraction(data, width, height, resolution, origin_x, origin_y, bounds):
    """Fraction of region cell centres unknown, including outside-map cells."""
    xmin, ymin, xmax, ymax = bounds
    if resolution <= 0 or xmin >= xmax or ymin >= ymax:
        raise ValueError('invalid region geometry')
    c0, c1 = (math.ceil((x - origin_x) / resolution - 0.5) for x in (xmin, xmax))
    r0, r1 = (math.ceil((y - origin_y) / resolution - 0.5) for y in (ymin, ymax))
    total = (r1 - r0) * (c1 - c0)
    if total <= 0:
        raise ValueError('branch region is smaller than the map resolution')
    grid = np.asarray(data).reshape(height, width)
    known = 0
    if max(0, r0) < min(height, r1) and max(0, c0) < min(width, c1):
        known = np.count_nonzero(grid[max(0, r0):min(height, r1),
                                      max(0, c0):min(width, c1)] >= 0)
    return 1.0 - known / total


class BranchPriority:
    def __init__(self, left_region, right_region, split_x=0.0, tolerance=0.05):
        self.regions = (tuple(left_region), tuple(right_region))
        if any(len(b) != 4 or not all(math.isfinite(v) for v in b)
               or b[0] >= b[2] or b[1] >= b[3] for b in self.regions):
            raise ValueError('branch regions must be [xmin, ymin, xmax, ymax]')
        if not math.isfinite(split_x) or not 0 <= tolerance < 1:
            raise ValueError('invalid branch split or unknown tolerance')
        self.split_x = split_x
        self.tolerance = tolerance
        self.stage = 0  # left, right, complete
        self.fractions = (1.0, 1.0)

    def update(self, data, width, height, resolution, origin_x, origin_y):
        self.fractions = tuple(unknown_fraction(
            data, width, height, resolution, origin_x, origin_y, b) for b in self.regions)
        while self.stage < 2 and self.fractions[self.stage] <= self.tolerance:
            self.stage += 1

    def accepts(self, x):
        return ((self.stage == 0 and x < self.split_x)
                or (self.stage == 1 and x >= self.split_x))

    def filter_clusters(self, clusters, resolution, origin_x):
        return [cells for cluster in clusters if (cells := [
            (r, c) for r, c in cluster
            if self.accepts(origin_x + (c + 0.5) * resolution)])]
