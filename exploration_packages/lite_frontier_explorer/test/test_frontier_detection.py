import math

from lite_frontier_explorer.frontier_detection import (
    cluster_centroid_world,
    cluster_nearest_point_world,
    find_frontier_clusters,
    select_nearest_frontier,
    yaw_from_quaternion,
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


def test_select_nearest_frontier_picks_closest_cluster():
    near_cluster = [(0, 0), (0, 1), (0, 2)]
    far_cluster = [(9, 9), (9, 10), (9, 11)]

    resolution = 0.05
    origin_x, origin_y = 0.0, 0.0

    goal = select_nearest_frontier(
        [far_cluster, near_cluster], robot_x=0.0, robot_y=0.0,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
    )

    # (0, 0) is near_cluster's own cell closest to the robot at the origin.
    near_x = origin_x + (0 + 0.5) * resolution
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

    far_x, far_y = cluster_nearest_point_world(
        far_cluster, 0.0, 0.0, resolution, origin_x, origin_y)
    assert goal == (far_x, far_y)


def test_select_nearest_frontier_returns_none_when_all_clusters_too_close():
    near_cluster = [(0, 0), (0, 1), (0, 2)]

    goal = select_nearest_frontier(
        [near_cluster], robot_x=0.0, robot_y=0.0,
        resolution=0.05, origin_x=0.0, origin_y=0.0,
        min_distance_m=2.0,
    )

    assert goal is None


def test_cluster_nearest_point_world_is_always_a_cluster_member():
    # L-shaped cluster: its arithmetic-mean centroid (0.6, 0.6) is not a
    # member of the cluster at all (it's not even an integer cell), and would
    # fall outside the frontier if used directly as a nav2 goal -- e.g. into
    # occupied or unknown space nav2 can never plan a path into.
    l_shaped_cluster = [(0, 0), (0, 1), (0, 2), (1, 0), (2, 0)]
    resolution, origin_x, origin_y = 0.05, 0.0, 0.0

    goal = cluster_nearest_point_world(
        l_shaped_cluster, 100.0, 100.0, resolution, origin_x, origin_y)

    cell_centers = {
        (origin_x + (c + 0.5) * resolution, origin_y + (r + 0.5) * resolution)
        for r, c in l_shaped_cluster
    }
    centroid = cluster_centroid_world(l_shaped_cluster, resolution, origin_x, origin_y)
    assert centroid not in cell_centers
    assert goal in cell_centers


def test_cluster_nearest_point_world_reflects_true_proximity_not_centroid():
    # A large cluster spanning from very close to the robot (col 0) out to
    # far away (col 100): its centroid would sit around col 50, making it
    # look far away even though part of the same cluster is right next to
    # the robot. The nearest point must be the close end, not the centroid.
    large_cluster = [(0, c) for c in range(0, 101, 10)]
    resolution, origin_x, origin_y = 1.0, 0.0, 0.0

    x, y = cluster_nearest_point_world(large_cluster, 0.0, 0.0, resolution, origin_x, origin_y)

    assert x < 5.0  # near the col=0 end, nowhere close to the col~50 centroid


def test_select_nearest_frontier_true_nearest_beats_centroid_anchored_cluster():
    # Regression test for the "not choosing the nearest frontier" bug: a
    # large/elongated cluster spans x=10..100, so its own nearest edge is
    # only 1.0m from the robot -- but its centroid sits around x=55.5, which
    # would look ~46m away. A second, smaller cluster genuinely 2.0m away
    # (perpendicular, so it can't be confused with the large cluster's own
    # extent) is farther than the large cluster's true nearest point but
    # nearer than its centroid: the old centroid-anchored ranking would have
    # picked this smaller cluster (2.0m < ~46m); true nearest-point ranking
    # must instead pick the large cluster's near edge (1.0m < 2.0m).
    large_cluster = [(0, c) for c in range(10, 101, 10)]  # spans x=10..100 at y=0.5
    small_far_cluster = [(2, 9), (2, 10), (2, 11)]  # nearest point (9.5, 2.5), 2.0m away

    resolution, origin_x, origin_y = 1.0, 0.0, 0.0
    # Put the robot right next to the near end of the large cluster.
    goal = select_nearest_frontier(
        [large_cluster, small_far_cluster], robot_x=9.5, robot_y=0.5,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
    )

    assert goal == (10.5, 0.5)


def test_yaw_from_quaternion_identity_is_zero():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0


def test_yaw_from_quaternion_90_degrees():
    half = math.pi / 4
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-9)


def test_select_nearest_frontier_excludes_frontier_directly_behind():
    # Robot at origin facing +x. A frontier directly behind (-x) must never
    # be picked, even when it is the only candidate.
    behind_cluster = [(0, -3)]  # world (-2.5, 0.5) with this grid -- behind
    resolution, origin_x, origin_y = 1.0, 0.0, 0.0

    goal = select_nearest_frontier(
        [behind_cluster], robot_x=0.5, robot_y=0.5,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
        robot_yaw=0.0,
    )

    assert goal is None


def test_select_nearest_frontier_prefers_ahead_over_closer_beside():
    # Robot at origin facing +x. A "beside" frontier is closer in raw
    # distance than an "ahead" one, but ahead must still win.
    resolution, origin_x, origin_y = 1.0, 0.0, 0.0
    ahead_cluster = [(0, 4)]     # world (4.5, 0.5) -- straight ahead, 4.0m
    beside_cluster = [(2, 0)]    # world (0.5, 2.5) -- directly to the side, 2.0m

    goal = select_nearest_frontier(
        [beside_cluster, ahead_cluster], robot_x=0.5, robot_y=0.5,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
        robot_yaw=0.0,
    )

    assert goal == (4.5, 0.5)


def test_select_nearest_frontier_falls_back_to_beside_when_nothing_ahead():
    resolution, origin_x, origin_y = 1.0, 0.0, 0.0
    beside_cluster = [(2, 0)]   # world (0.5, 2.5) -- directly to the side
    behind_cluster = [(0, -4)]  # directly behind -- must be excluded

    goal = select_nearest_frontier(
        [beside_cluster, behind_cluster], robot_x=0.5, robot_y=0.5,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
        robot_yaw=0.0,
    )

    assert goal == (0.5, 2.5)


def test_select_nearest_frontier_ignores_direction_when_yaw_omitted():
    resolution, origin_x, origin_y = 1.0, 0.0, 0.0
    ahead_cluster = [(0, 4)]     # 4.0m ahead
    beside_cluster = [(2, 0)]    # 2.0m to the side, closer

    goal = select_nearest_frontier(
        [beside_cluster, ahead_cluster], robot_x=0.5, robot_y=0.5,
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
    )

    assert goal == (0.5, 2.5)  # plain nearest-distance, no directional filtering
