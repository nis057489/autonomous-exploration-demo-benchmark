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


def cluster_representative_cell(cluster):
    """Return the cluster's own (row, col) cell closest to its arithmetic-mean
    centroid.

    The plain average of a cluster's cells is not guaranteed to land on a
    member of the cluster at all -- for a non-convex frontier (one that
    wraps around a corner, hugs an L-shaped wall, or traces along a doorway)
    the mean position can fall outside the frontier entirely, landing in an
    occupied or unknown cell nav2's planner can never reach within its goal
    tolerance. Snapping to the nearest actual cluster member guarantees the
    point is always a real frontier cell (free, adjacent to unknown).
    """
    avg_r = sum(r for r, _ in cluster) / len(cluster)
    avg_c = sum(c for _, c in cluster) / len(cluster)
    return min(cluster, key=lambda rc: (rc[0] - avg_r) ** 2 + (rc[1] - avg_c) ** 2)


def cluster_goal_world(cluster, resolution, origin_x, origin_y):
    """World (x, y) of a real, always-reachable-in-principle cell in the
    cluster -- use this (not cluster_centroid_world) for anything that
    becomes an actual nav2 goal or a blacklist key, since cluster_centroid_world's
    plain average can fall outside the cluster. See cluster_representative_cell."""
    r, c = cluster_representative_cell(cluster)
    return (
        origin_x + (c + 0.5) * resolution,
        origin_y + (r + 0.5) * resolution,
    )


def select_nearest_frontier(clusters, robot_x, robot_y, resolution, origin_x, origin_y,
                             min_distance_m=0.0):
    """Return the (x, y) world goal point of the nearest cluster at least
    min_distance_m from the robot, or None if no cluster qualifies."""
    best_xy = None
    best_dist = None
    for cluster in clusters:
        x, y = cluster_goal_world(cluster, resolution, origin_x, origin_y)
        dist = math.hypot(x - robot_x, y - robot_y)
        if dist < min_distance_m:
            continue
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_xy = (x, y)
    return best_xy
