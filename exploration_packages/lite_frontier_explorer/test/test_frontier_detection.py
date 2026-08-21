from lite_frontier_explorer.frontier_detection import (
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
