"""Pure frontier-detection logic, kept free of ROS/rclpy deps so it's testable
without a running node. Operates on a nav2-style costmap: -1 unknown,
0..100 cost (free ~= 0), row-major width*height array.
"""

import math

import numpy as np

_NEIGHBOR_OFFSETS = ((-1, 0), (1, 0), (0, -1), (0, 1))


def find_frontier_clusters(data, width, height, occ_threshold=50, min_size=6):
    """Return a list of clusters, each a list of (row, col) cells.

    A cell is a frontier cell if it's free (cost < occ_threshold) and
    4-connected-adjacent to at least one unknown (-1) cell. The per-cell
    scan (the part that's O(width*height)) is vectorized with numpy:
    shift the unknown-cell mask by one in each direction and OR the
    shifted copies together, so every cell's "do I have an unknown
    neighbor" check happens in a handful of whole-array ops instead of a
    width*height Python loop. The frontier cells found are then grouped
    into 4-connected clusters via BFS -- that stays plain Python since it
    only touches the much smaller frontier-cell set, not the whole grid.
    Clusters smaller than min_size are dropped.
    """
    grid = np.asarray(data, dtype=np.int8).reshape(height, width)

    free_mask = (grid >= 0) & (grid < occ_threshold)
    unknown_mask = grid == -1

    # 1. Vectorized neighbor check: shift the unknown-cell mask one step
    #    in each of the 4 directions and OR the results together, so every
    #    cell learns "do I border an unknown cell?" in whole-array ops
    #    rather than a width*height Python loop.
    neighbor_unknown = np.zeros_like(unknown_mask)
    neighbor_unknown[1:, :] |= unknown_mask[:-1, :]   # neighbor above is unknown
    neighbor_unknown[:-1, :] |= unknown_mask[1:, :]   # neighbor below
    neighbor_unknown[:, 1:] |= unknown_mask[:, :-1]   # neighbor to the left
    neighbor_unknown[:, :-1] |= unknown_mask[:, 1:]   # neighbor to the right

    # 2. A frontier cell is free and touches at least one unknown cell.
    frontier_mask = free_mask & neighbor_unknown
    frontier_cells = {(int(r), int(c)) for r, c in np.argwhere(frontier_mask)}

    # 1. Group frontier cells into connected clusters via BFS/flood-fill.
    clusters = []
    visited = set()
    for cell in frontier_cells:
        # 2. Skip cells already swept into an earlier cluster.
        if cell in visited:
            continue
        visited.add(cell)
        stack = [cell]
        cluster = []
        # 3. Flood-fill outward from this seed cell, walking only
        #    4-connected neighbors that are themselves frontier cells.
        while stack:
            r, c = stack.pop()
            cluster.append((r, c))
            for dr, dc in _NEIGHBOR_OFFSETS:
                nb = (r + dr, c + dc)
                if nb in frontier_cells and nb not in visited:
                    visited.add(nb)
                    stack.append(nb)
        # 4. Discard clusters too small to be a meaningful frontier.
        if len(cluster) >= min_size:
            clusters.append(cluster)

    return clusters


def cluster_centroid_world(cluster, resolution, origin_x, origin_y):
    avg_r = sum(r for r, _ in cluster) / len(cluster)
    avg_c = sum(c for _, c in cluster) / len(cluster)
    return (
        origin_x + (avg_c + 0.5) * resolution,
        origin_y + (avg_r + 0.5) * resolution,
    )


<<<<<<< HEAD
def select_nearest_frontier(clusters, robot_x, robot_y, resolution, origin_x, origin_y,
                             min_distance_m=0.0):
    """Return the (x, y) world centroid of the nearest cluster at least
    min_distance_m from the robot, or None if no cluster qualifies."""
    best_xy = None
    best_dist = None
    for cluster in clusters:
        x, y = cluster_centroid_world(cluster, resolution, origin_x, origin_y)
=======
def cluster_nearest_point_world(cluster, ref_x, ref_y, resolution, origin_x, origin_y):
    """World (x, y) of the cluster's own cell closest to (ref_x, ref_y).

    Always a real member of the cluster (free, adjacent to unknown) --
    never the arithmetic-mean centroid, which is not guaranteed to land on
    a member of the cluster at all: for a non-convex frontier (one that
    wraps around a corner, hugs an L-shaped wall, or traces along a
    doorway) the mean can fall outside the frontier entirely, into an
    occupied or unknown cell nav2's planner can never reach within its goal
    tolerance. Anchoring to ref_x/ref_y (typically the robot's current
    position) instead of the cluster's own fixed centroid also means
    ranking clusters by this point actually reflects which one is nearest:
    a centroid-anchored point can sit meters from the robot on a large or
    elongated cluster even while part of that same cluster is only
    centimeters away, making a farther cluster look "nearer" by its
    centroid than a closer cluster's true nearest edge.
    """
    best_xy = None
    best_dist2 = None
    for r, c in cluster:
        wx = origin_x + (c + 0.5) * resolution
        wy = origin_y + (r + 0.5) * resolution
        dist2 = (wx - ref_x) ** 2 + (wy - ref_y) ** 2
        if best_dist2 is None or dist2 < best_dist2:
            best_dist2 = dist2
            best_xy = (wx, wy)
    return best_xy


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _bearing_band(frontier_x, frontier_y, robot_x, robot_y, robot_yaw):
    """Classify a frontier point's bearing relative to robot_yaw: 'ahead'
    (within 45 degrees), 'behind' (within 45 degrees of the opposite
    direction), or 'beside' (the two 90-degree side wedges in between)."""
    bearing = math.atan2(frontier_y - robot_y, frontier_x - robot_x)
    abs_diff = abs(_wrap_to_pi(bearing - robot_yaw))
    if abs_diff <= math.pi / 4:
        return 'ahead'
    if abs_diff >= 3.0 * math.pi / 4:
        return 'behind'
    return 'beside'


def select_nearest_frontier(clusters, robot_x, robot_y, resolution, origin_x, origin_y,
                             min_distance_m=0.0, robot_yaw=None):
    """Return the (x, y) world goal point of the best qualifying cluster, or
    None if none qualify.

    When robot_yaw is given: frontiers behind the robot (more than 135
    degrees from its heading) are excluded entirely. Among the rest, the
    nearest frontier ahead of the robot (within 45 degrees) always wins over
    any frontier beside it, even a closer one -- beside candidates are only
    considered at all when there is no ahead candidate. When robot_yaw is
    omitted, ranks purely by distance with no directional preference.
    """
    if robot_yaw is None:
        best_xy = None
        best_dist = None
        for cluster in clusters:
            x, y = cluster_nearest_point_world(
                cluster, robot_x, robot_y, resolution, origin_x, origin_y)
            dist = math.hypot(x - robot_x, y - robot_y)
            if dist < min_distance_m:
                continue
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_xy = (x, y)
        return best_xy

    best_by_band = {'ahead': None, 'beside': None}
    for cluster in clusters:
        x, y = cluster_nearest_point_world(
            cluster, robot_x, robot_y, resolution, origin_x, origin_y)
>>>>>>> parent of c9b9310 (Revert "prioritize frontier cells ahead or beside and finally behind")
        dist = math.hypot(x - robot_x, y - robot_y)
        if dist < min_distance_m:
            continue
        band = _bearing_band(x, y, robot_x, robot_y, robot_yaw)
        if band == 'behind':
            continue
        current = best_by_band[band]
        if current is None or dist < current[1]:
            best_by_band[band] = ((x, y), dist)

    if best_by_band['ahead'] is not None:
        return best_by_band['ahead'][0]
    if best_by_band['beside'] is not None:
        return best_by_band['beside'][0]
    return None
