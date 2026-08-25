"""Pure frontier-detection logic, kept free of ROS/rclpy deps so it's testable
without a running node. Operates on a nav2-style costmap: -1 unknown,
0..100 cost (free ~= 0), row-major width*height array.
"""

import math
from collections import deque

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


def _free_space_distances(data, width, height, robot_x, robot_y, resolution,
                           origin_x, origin_y, occ_threshold):
    """BFS out from the robot's cell over 4-connected free space (cost <
    occ_threshold; unknown/occupied cells block the walk). Returns a
    height x width array of step counts, -1 where unreached."""
    grid = np.asarray(data, dtype=np.int8).reshape(height, width)
    free_mask = (grid >= 0) & (grid < occ_threshold)

    dist = np.full((height, width), -1, dtype=np.int32)
    start_row = int((robot_y - origin_y) / resolution)
    start_col = int((robot_x - origin_x) / resolution)
    if not (0 <= start_row < height and 0 <= start_col < width):
        return dist
    if not free_mask[start_row, start_col]:
        return dist

    dist[start_row, start_col] = 0
    queue = deque([(start_row, start_col)])
    while queue:
        r, c = queue.popleft()
        d = dist[r, c]
        for dr, dc in _NEIGHBOR_OFFSETS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and free_mask[nr, nc] and dist[nr, nc] < 0:
                dist[nr, nc] = d + 1
                queue.append((nr, nc))
    return dist


def select_nearest_frontier(clusters, data, width, height, robot_x, robot_y, resolution,
                             origin_x, origin_y, occ_threshold=50, min_distance_m=0.0):
    """Return the (x, y) world centroid of the cluster with the shortest
    walkable path through known free space, at least min_distance_m away
    (measured along that path), or None if no cluster qualifies.

    Straight-line nearest can pick a frontier that's close as the crow
    flies but behind a wall, sending the robot on a long detour (or into
    a costmap-inflated no-path zone) instead of a farther-but-open one.
    Ranking by BFS step count through free space avoids that.
    """
    dist_grid = _free_space_distances(
        data, width, height, robot_x, robot_y, resolution, origin_x, origin_y, occ_threshold)

    best_xy = None
    best_dist = None
    for cluster in clusters:
        path_dist = min(
            (dist_grid[r, c] for r, c in cluster if dist_grid[r, c] >= 0),
            default=None,
        )
        if path_dist is None:
            continue  # no known-free path from the robot to this cluster
        if path_dist * resolution < min_distance_m:
            continue
        if best_dist is None or path_dist < best_dist:
            best_dist = path_dist
            best_xy = cluster_centroid_world(cluster, resolution, origin_x, origin_y)
    return best_xy
