"""Pure frontier-detection logic, kept free of ROS/rclpy deps so it's testable
without a running node. Operates on a nav2-style costmap: -1 unknown,
0..100 cost (free ~= 0), row-major width*height array.
"""

import math

_NEIGHBOR_OFFSETS = ((-1, 0), (1, 0), (0, -1), (0, 1))


def find_frontier_clusters(data, width, height, occ_threshold=50, min_size=6):
    """Return a list of clusters, each a list of (row, col) cells.

    A cell is a frontier cell if it's free (cost < occ_threshold) and
    4-connected-adjacent to at least one unknown (-1) cell. Frontier cells
    are then grouped into 4-connected clusters; clusters smaller than
    min_size are dropped.
    """
    def at(r, c):
        return data[r * width + c]

    frontier_cells = set()
    for r in range(height):
        for c in range(width):
            v = at(r, c)
            if not (0 <= v < occ_threshold):
                continue
            for dr, dc in _NEIGHBOR_OFFSETS:
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width and at(nr, nc) == -1:
                    frontier_cells.add((r, c))
                    break

    clusters = []
    visited = set()
    for cell in frontier_cells:
        if cell in visited:
            continue
        visited.add(cell)
        stack = [cell]
        cluster = []
        while stack:
            r, c = stack.pop()
            cluster.append((r, c))
            for dr, dc in _NEIGHBOR_OFFSETS:
                nb = (r + dr, c + dc)
                if nb in frontier_cells and nb not in visited:
                    visited.add(nb)
                    stack.append(nb)
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


def select_nearest_frontier(clusters, robot_x, robot_y, resolution, origin_x, origin_y):
    """Return the (x, y) world centroid of the cluster nearest the robot, or
    None if clusters is empty."""
    best_xy = None
    best_dist = None
    for cluster in clusters:
        x, y = cluster_centroid_world(cluster, resolution, origin_x, origin_y)
        dist = math.hypot(x - robot_x, y - robot_y)
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_xy = (x, y)
    return best_xy
