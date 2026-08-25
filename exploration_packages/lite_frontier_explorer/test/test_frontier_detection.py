import math

from lite_frontier_explorer.frontier_detection import (
    cluster_centroid_world,
    find_frontier_clusters,
    select_nearest_frontier,
)

U, F, O = -1, 0, 100  # unknown, free, occupied


def _grid(rows):
    width = len(rows[0])
    height = len(rows)
    data = [cell for row in rows for cell in row]
    return data, width, height


def test_finds_single_frontier_cluster_between_free_and_unknown():
    # A 5x5 grid: known free on the left, unknown on the right.
    rows = [
        [F, F, U, U, U],
        [F, F, U, U, U],
        [F, F, U, U, U],
        [F, F, U, U, U],
        [F, F, U, U, U],
    ]
    data, width, height = _grid(rows)

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=3)

    assert len(clusters) == 1
    # Every free cell in column 1 borders an unknown cell in column 2.
    assert all(c == 1 for _, c in clusters[0])
    assert len(clusters[0]) == 5


def test_drops_clusters_smaller_than_min_size():
    rows = [
        [F, U, F, F, F],
        [F, F, F, F, F],
        [F, F, F, F, F],
    ]
    data, width, height = _grid(rows)

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=3)

    assert clusters == []


def test_occupied_cells_are_never_frontier_cells():
    rows = [
        [O, U, U],
        [O, U, U],
        [O, U, U],
    ]
    data, width, height = _grid(rows)

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=1)

    assert clusters == []


def _free_grid(width, height):
    return [F] * (width * height), width, height


def test_select_nearest_frontier_picks_closest_centroid():
    near_cluster = [(0, 0), (0, 1), (0, 2)]
    far_cluster = [(9, 9), (9, 10), (9, 11)]

    resolution = 0.05
    origin_x, origin_y = 0.0, 0.0
    data, width, height = _free_grid(12, 10)

    goal = select_nearest_frontier(
        [far_cluster, near_cluster], data, width, height,
        robot_x=0.0, robot_y=0.0,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
    )

    near_x = origin_x + (1 + 0.5) * resolution
    near_y = origin_y + (0 + 0.5) * resolution
    assert goal == (near_x, near_y)


def test_select_nearest_frontier_returns_none_for_no_clusters():
    data, width, height = _free_grid(1, 1)
    assert select_nearest_frontier([], data, width, height, 0.0, 0.0, 0.05, 0.0, 0.0) is None


def test_select_nearest_frontier_skips_clusters_within_min_distance():
    near_cluster = [(0, 0), (0, 1), (0, 2)]  # ~0.075m from origin
    far_cluster = [(9, 9), (9, 10), (9, 11)]  # far along the open grid

    resolution = 0.1
    origin_x, origin_y = 0.0, 0.0
    data, width, height = _free_grid(12, 10)

    goal = select_nearest_frontier(
        [near_cluster, far_cluster], data, width, height,
        robot_x=0.0, robot_y=0.0,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
        min_distance_m=1.0,
    )

    far_x, far_y = cluster_centroid_world(far_cluster, resolution, origin_x, origin_y)
    assert goal == (far_x, far_y)


def test_select_nearest_frontier_returns_none_when_all_clusters_too_close():
    near_cluster = [(0, 0), (0, 1), (0, 2)]
    data, width, height = _free_grid(5, 1)

    goal = select_nearest_frontier(
        [near_cluster], data, width, height,
        robot_x=0.0, robot_y=0.0,
        resolution=0.05, origin_x=0.0, origin_y=0.0,
        min_distance_m=2.0,
    )

    assert goal is None


def test_select_nearest_frontier_still_paths_out_when_robot_cell_at_path_occ_threshold():
    # Regression: the real robot's own costmap cell can sit AT
    # path_occ_threshold (e.g. inflation decay from a nearby wall puts it
    # exactly at the cutoff). That must not make the whole grid look
    # unreachable -- the robot is physically there, so the walk has to
    # be seeded from its cell regardless of that cell's own cost.
    path_occ_threshold = 99
    rows = [
        [path_occ_threshold, F, F, U],  # robot stands on the path_occ_threshold cell
    ]
    width = len(rows[0])
    height = len(rows)
    data = [cell for row in rows for cell in row]

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=1)
    assert len(clusters) == 1

    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0
    robot_x, robot_y = 0.5, 0.5  # cell (0, 0), the path_occ_threshold cell

    goal = select_nearest_frontier(
        clusters, data, width, height, robot_x, robot_y,
        resolution, origin_x, origin_y, path_occ_threshold=path_occ_threshold,
    )

    assert goal is not None


def test_select_nearest_frontier_walks_through_inflated_but_drivable_cost():
    # Regression: nav2 only refuses to drive through cost ~99 (inscribed)
    # and 100 (lethal) -- moderate inflation cost well above the frontier
    # occ_threshold (50) is still real, drivable space. A live robot1
    # capture showed cost 79 on the robot's own cell and a smooth 30-99
    # gradient all around it; using the frontier threshold (50) for path
    # BFS treated that entire gradient as impassable and rejected every
    # frontier. The default path_occ_threshold (99) must treat cost 79
    # cells as passable so a path through them is still found.
    rows = [
        [79, 79, 79, 79, F, U],  # a band of moderate inflation cost, not free
    ]
    width = len(rows[0])
    height = len(rows)
    data = [cell for row in rows for cell in row]

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=1)
    assert len(clusters) == 1  # the (0, 4)/free cell bordering unknown

    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0
    robot_x, robot_y = 0.5, 0.5  # cell (0, 0), cost 79 -- above the frontier threshold

    goal = select_nearest_frontier(
        clusters, data, width, height, robot_x, robot_y, resolution, origin_x, origin_y,
    )

    assert goal is not None


def test_select_nearest_frontier_prefers_reachable_over_closer_but_walled_off():
    # Column 2 (rows 0-7) is a frontier strip that LOOKS closest to the
    # robot by straight-line distance, but it's sealed off: column 1 is a
    # wall for rows 0-8, and row 8 walls off columns 2-5 too, so there is
    # no known-free path into it at all. The only actual way out is down
    # column 0, right along the open row 9, into the frontier bordering
    # row 10. A euclidean-only picker would aim the robot at the sealed
    # strip; the path-aware picker must skip it and pick the reachable one.
    rows = []
    for _ in range(8):
        rows.append([F, O, F, U, U, U])
    rows.append([F, O, O, O, O, O])  # row 8: separator wall
    rows.append([F, F, F, F, F, F])  # row 9: open corridor
    rows.append([U, U, U, U, U, U])  # row 10: unknown, borders row 9

    width = len(rows[0])
    height = len(rows)
    data = [cell for row in rows for cell in row]

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=6)
    assert len(clusters) == 2

    sealed = next(c for c in clusters if any(col == 2 for _, col in c))
    reachable = next(c for c in clusters if c is not sealed)

    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0
    robot_x, robot_y = 0.5, 0.5  # cell (0, 0)

    sealed_xy = cluster_centroid_world(sealed, resolution, origin_x, origin_y)
    reachable_xy = cluster_centroid_world(reachable, resolution, origin_x, origin_y)
    # Sanity check the trap: sealed cluster really is euclidean-closer.
    assert math.hypot(sealed_xy[0] - robot_x, sealed_xy[1] - robot_y) < \
        math.hypot(reachable_xy[0] - robot_x, reachable_xy[1] - robot_y)

    goal = select_nearest_frontier(
        clusters, data, width, height, robot_x, robot_y,
        resolution, origin_x, origin_y,
    )

    assert goal == reachable_xy
