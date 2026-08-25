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


def test_select_nearest_frontier_picks_closest_centroid():
    near_cluster = [(0, 0), (0, 1), (0, 2)]
    far_cluster = [(9, 9), (9, 10), (9, 11)]

    resolution = 0.05
    origin_x, origin_y = 0.0, 0.0

    goal = select_nearest_frontier(
        [far_cluster, near_cluster], robot_x=0.0, robot_y=0.0,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
    )

    near_x = origin_x + (1 + 0.5) * resolution
    near_y = origin_y + (0 + 0.5) * resolution
    assert goal == (near_x, near_y)


def test_select_nearest_frontier_returns_none_for_no_clusters():
    assert select_nearest_frontier([], 0.0, 0.0, 0.05, 0.0, 0.0) is None


def test_select_nearest_frontier_skips_clusters_within_min_distance():
    near_cluster = [(0, 0), (0, 1), (0, 2)]  # ~0.075m from origin
    far_cluster = [(9, 9), (9, 10), (9, 11)]  # ~1.36m from origin

    resolution = 0.1
    origin_x, origin_y = 0.0, 0.0

    goal = select_nearest_frontier(
        [near_cluster, far_cluster], robot_x=0.0, robot_y=0.0,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
        min_distance_m=1.0,
    )

    far_x, far_y = cluster_centroid_world(far_cluster, resolution, origin_x, origin_y)
    assert goal == (far_x, far_y)


def test_select_nearest_frontier_returns_none_when_all_clusters_too_close():
    near_cluster = [(0, 0), (0, 1), (0, 2)]

    goal = select_nearest_frontier(
        [near_cluster], robot_x=0.0, robot_y=0.0,
        resolution=0.05, origin_x=0.0, origin_y=0.0,
        min_distance_m=2.0,
    )

    assert goal is None
