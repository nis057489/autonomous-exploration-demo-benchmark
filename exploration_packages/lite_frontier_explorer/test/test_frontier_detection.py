import math

from lite_frontier_explorer.frontier_detection import (
    cluster_centroid_world,
    estimate_frontier_gain,
    estimate_frontier_region_size,
    find_frontier_clusters,
    select_best_frontier,
    select_nearest_frontier,
    select_nearest_high_gain_frontier,
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


def test_estimate_frontier_gain_counts_unknown_cells_in_sensor_range():
    # Frontier centroid at (0, 2); unknown cells fill columns 3-6 on row 0
    # (4 cells) plus a stray unknown cell far away that must not count.
    rows = [
        [F, F, F, U, U, U, U],
    ]
    data, width, height = _grid(rows)
    cluster = [(0, 2)]

    gain = estimate_frontier_gain(cluster, data, width, height, resolution=1.0, sensor_range_m=3.0)

    # Unknown cells at columns 3, 4, 5 are within 3.0 of column 2; column 6 (dist 4) is not.
    assert gain == 3


def test_estimate_frontier_gain_zero_when_no_unknown_in_range():
    rows = [[F, F, F, F, F]]
    data, width, height = _grid(rows)
    cluster = [(0, 2)]

    gain = estimate_frontier_gain(cluster, data, width, height, resolution=1.0, sensor_range_m=3.0)

    assert gain == 0


def test_select_best_frontier_prefers_higher_gain_over_nearest():
    # A small pocket frontier (opens onto a single unknown cell) sits
    # closer to the robot than a wide frontier (opens onto five unknown
    # cells). A pure-nearest picker takes the closer pocket; the
    # gain-aware picker should take the farther, wider one instead since
    # its gain-per-cost utility wins even after the distance penalty.
    rows = [
        [F, F, F, F, F, F, F, F, F, F, F, F, F],
        [F, F, F, F, F, F, F, F, F, F, F, F, F],
        [F, F, F, F, F, F, F, F, F, F, F, F, F],
        [O, O, U, O, O, O, O, O, U, U, U, U, U],
    ]
    data, width, height = _grid(rows)

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=1)
    assert len(clusters) == 2

    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0
    robot_x, robot_y = 3.5, 0.5  # cell (0, 3): closer to the pocket than the wide frontier

    goal = select_best_frontier(
        clusters, data, width, height, robot_x, robot_y,
        resolution, origin_x, origin_y,
        sensor_range_m=4.0, distance_weight=0.1,
    )
    nearest_goal = select_nearest_frontier(
        clusters, data, width, height, robot_x, robot_y,
        resolution, origin_x, origin_y,
    )

    pocket_xy = cluster_centroid_world([(2, 2)], resolution, origin_x, origin_y)
    wide_xy = cluster_centroid_world([(2, c) for c in range(8, 13)], resolution, origin_x, origin_y)

    assert nearest_goal == pocket_xy
    assert goal == wide_xy


def test_select_best_frontier_returns_none_for_no_clusters():
    data, width, height = _free_grid(1, 1)
    assert select_best_frontier([], data, width, height, 0.0, 0.0, 0.05, 0.0, 0.0) is None


def test_select_nearest_high_gain_frontier_picks_nearer_of_two_equally_rich_frontiers():
    # Three frontiers: a nearby pocket (gain 1, too low to clear the gain
    # floor), and two wide frontiers (gain 5 each) at different distances.
    # select_best_frontier's raw gain/cost utility would tend toward
    # whichever wide frontier has the better utility -- often the single
    # global best -- which is what sends every robot on the team to the
    # same spot. select_nearest_high_gain_frontier should instead treat
    # both wide frontiers as "high gain" and pick the nearer one.
    row3 = (
        [O, O, U] + [O] * 4 + [U] * 5 + [O] * 4 + [U] * 5
    )
    rows = [
        [F] * len(row3),
        [F] * len(row3),
        [F] * len(row3),
        row3,
    ]
    data, width, height = _grid(rows)

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=1)
    assert len(clusters) == 3

    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0
    robot_x, robot_y = 3.5, 0.5  # cell (0, 3): near the pocket, closer to the first wide frontier

    goal = select_nearest_high_gain_frontier(
        clusters, data, width, height, robot_x, robot_y,
        resolution, origin_x, origin_y,
        gain_threshold_ratio=0.75,
    )

    near_wide = next(c for c in clusters if any(7 <= col <= 11 for _, col in c))
    near_wide_xy = cluster_centroid_world(near_wide, resolution, origin_x, origin_y)

    assert goal == near_wide_xy


def test_select_nearest_high_gain_frontier_returns_none_for_no_clusters():
    data, width, height = _free_grid(1, 1)
    assert select_nearest_high_gain_frontier([], data, width, height, 0.0, 0.0, 0.05, 0.0, 0.0) is None


def test_select_nearest_high_gain_frontier_turn_penalty_prefers_straight_route():
    # Two frontiers exactly 6 steps from the robot: a straight corridor
    # along row 0 (0 turns), and an L-shaped route -- forced by walls,
    # since rows 1-2 are only passable at column 0 -- down 3 then right 3
    # (1 turn). Equal step count, equal (single-cell) gain: without a
    # turn penalty the tie is broken by whichever the loop happens to
    # visit last with a strictly-lower cost; with turn_penalty_m > 0 the
    # straight route must win since the L-shaped one now costs strictly
    # more.
    rows = [
        [F, F, F, F, F, F, F, U],  # row 0: straight corridor, frontier at (0, 6)
        [F, O, O, O, O, O, O, O],  # row 1: only col 0 passable -- forces the L route down
        [F, O, O, O, O, O, O, O],  # row 2: only col 0 passable
        [F, F, F, F, U, O, O, O],  # row 3: L route turns right here, frontier at (3, 3)
    ]
    data, width, height = _grid(rows)

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=1)
    assert len(clusters) == 2

    straight = next(c for c in clusters if any(r == 0 for r, _ in c))
    dogleg = next(c for c in clusters if c is not straight)

    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0
    robot_x, robot_y = 0.5, 0.5  # cell (0, 0)

    straight_xy = cluster_centroid_world(straight, resolution, origin_x, origin_y)

    goal_with_penalty = select_nearest_high_gain_frontier(
        clusters, data, width, height, robot_x, robot_y,
        resolution, origin_x, origin_y,
        gain_threshold_ratio=0.5, turn_penalty_m=2.0,
    )

    assert goal_with_penalty == straight_xy


def test_select_nearest_high_gain_frontier_hysteresis_favors_preferred_direction():
    # Two equally-good frontiers to the left and right of the robot. With
    # a preferred_direction pointing right and a hysteresis bonus, the
    # picker should favor the right-hand frontier over the left even
    # though both are otherwise identical.
    rows = [
        [U, U, F, F, F, F, F, U, U],
    ]
    data, width, height = _grid(rows)

    clusters = find_frontier_clusters(data, width, height, occ_threshold=50, min_size=1)
    assert len(clusters) == 2

    left = next(c for c in clusters if any(col < 4 for _, col in c))
    right = next(c for c in clusters if c is not left)

    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0
    robot_x, robot_y = 4.5, 0.5  # cell (0, 4), centered between both frontiers

    left_xy = cluster_centroid_world(left, resolution, origin_x, origin_y)
    right_xy = cluster_centroid_world(right, resolution, origin_x, origin_y)

    goal = select_nearest_high_gain_frontier(
        clusters, data, width, height, robot_x, robot_y,
        resolution, origin_x, origin_y,
        gain_threshold_ratio=0.5,
        preferred_direction=(1.0, 0.0), hysteresis_bonus_m=5.0,
    )

    assert goal == right_xy
    assert goal != left_xy


def test_estimate_frontier_region_size_scales_with_actual_unknown_area():
    # A 1-cell pocket vs. a 6x6 open unknown block, both bordered by a
    # single-cell-wide frontier gap. estimate_frontier_gain (fixed-radius
    # window) would score these similarly once the window is bigger than
    # the pocket; region size should reflect that the block is genuinely
    # much bigger, with no radius cap involved.
    pocket_rows = [
        [F, F, U],
        [F, F, F],
    ]
    data, width, height = _grid(pocket_rows)
    pocket_cluster = [(0, 1)]  # borders (0, 2) which is unknown
    pocket_size = estimate_frontier_region_size(pocket_cluster, data, width, height)
    assert pocket_size == 1

    block_rows = [[F] * 7] + [[F] + [U] * 6 for _ in range(6)]
    data, width, height = _grid(block_rows)
    block_cluster = [(0, i) for i in range(1, 7)]  # row 0 borders the unknown block below
    block_size = estimate_frontier_region_size(block_cluster, data, width, height)
    assert block_size == 36  # the full 6x6 unknown block
    assert block_size > pocket_size


def test_estimate_frontier_region_size_respects_cap():
    rows = [[F] * 11] + [[F] + [U] * 10 for _ in range(10)]
    data, width, height = _grid(rows)
    cluster = [(0, i) for i in range(1, 11)]

    size = estimate_frontier_region_size(cluster, data, width, height, cap=20)

    assert size == 20
