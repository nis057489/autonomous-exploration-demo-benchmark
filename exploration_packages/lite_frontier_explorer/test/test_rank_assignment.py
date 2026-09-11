import numpy as np
import pytest

from lite_frontier_explorer.frontier_detection import (
    find_frontier_clusters, rank_frontier_clusters, select_visible_gain_frontier,
)


def scene():
    grid = np.full((10, 36), 100, dtype=np.int8)
    grid[7:, :] = 0
    grid[6, 2] = 0
    grid[5, 2] = -1
    for left in (10, 26):
        grid[6, left:left+8] = 0
        grid[:6, left:left+8] = -1
    return grid


def select(grid, index, count=3, robot=(18.5, 8.5), clusters=None, **kwargs):
    if clusters is None:
        clusters = find_frontier_clusters(grid.ravel(), grid.shape[1], grid.shape[0], min_size=1)
    return select_visible_gain_frontier(
        clusters, grid.ravel(), grid.shape[1], grid.shape[0], *robot,
        1, 0, 0, sensor_range_m=5, assignment_mode='robot_rank',
        robot_index=index, team_size=count, **kwargs)


def which_cluster(goal, clusters):
    return next(i for i, cluster in enumerate(clusters)
                if (int(goal[1]), int(goal[0])) in cluster)


def test_three_robots_get_three_distinct_clusters_on_identical_map():
    grid = scene()
    clusters = find_frontier_clusters(grid.ravel(), 36, 10, min_size=1)
    ranked = rank_frontier_clusters(clusters, grid, 1, 5)
    assert len(ranked) == 3
    # Each robot has a different personal distance ranking. It must still
    # select its common map rank, rather than its own nth-best utility.
    for i, pose in enumerate(((2.5, 8.5), (32.5, 8.5), (14.5, 8.5))):
        goal = select(grid, i, robot=pose)
        assert which_cluster(goal, ranked) == i


def test_cluster_and_cell_input_order_cannot_change_assignment():
    grid = scene()
    clusters = find_frontier_clusters(grid.ravel(), 36, 10, min_size=1)
    shuffled = [list(reversed(c)) for c in reversed(clusters)]
    for i in range(3):
        assert select(grid, i, clusters=clusters) == select(grid, i, clusters=shuffled)


def test_multiple_viewpoints_are_not_allocated_as_separate_frontiers():
    grid = scene()
    goals = [select(grid, i, max_viewpoints=7) for i in range(3)]
    assert sorted(int(goal[0]) // 10 for goal in goals) == [0, 1, 2]


def test_private_blacklist_does_not_renumber_common_ranking():
    grid = scene()
    original_second = select(grid, 1)
    scores = []
    # Robot1 cannot use its entire assigned room; it falls back, rather than
    # silently deleting that room from the ranking used by other robots.
    goal = select(grid, 0, blacklisted_goals=[(14, 6)], blacklist_radius_m=6,
                  diagnostics=scores)
    assert goal is not None
    assert scores and all(s['assignment_fallback'] for s in scores)
    assert select(grid, 1) == original_second


def test_next_owned_rank_precedes_another_robots_assignment():
    grid = scene()
    grid[6, 6] = 0
    grid[5, 6] = -1  # Fourth cluster, after the existing one-cell pocket.
    scores = []
    goal = select(grid, 0, blacklisted_goals=[(14, 6)], blacklist_radius_m=6,
                  diagnostics=scores)
    assert goal == (6.5, 6.5)
    assert scores and all(s['cluster_rank'] == 3 and not s['assignment_fallback'] for s in scores)


def test_single_frontier_wraps_ids_without_parking_none_robots():
    grid = scene()
    grid[:7, 10:] = 100
    goals = [select(grid, i) for i in range(3)]
    assert goals == [(2.5, 6.5)] * 3


def test_two_frontiers_wrap_third_robot_only():
    grid = scene()
    grid[:7, :8] = 100
    goals = [select(grid, i) for i in range(3)]
    assert goals[0] != goals[1]
    assert goals[2] == goals[0]


def test_unreachable_assignment_falls_back_to_local_work():
    grid = scene()
    grid[7:, 8] = 100
    scores = []
    assert select(grid, 0, robot=(2.5, 8.5), diagnostics=scores) == (2.5, 6.5)
    assert scores and all(s['assignment_fallback'] for s in scores)


def test_no_frontiers_returns_none_for_every_robot():
    grid = np.zeros((10, 36), dtype=np.int8)
    assert all(select(grid, i) is None for i in range(3))


def test_single_robot_keeps_original_utility_policy():
    grid = scene()
    clusters = find_frontier_clusters(grid.ravel(), 36, 10, min_size=1)
    independent = select_visible_gain_frontier(
        clusters, grid.ravel(), 36, 10, 18.5, 8.5, 1, 0, 0, sensor_range_m=5)
    assert select(grid, 0, count=1) == independent


@pytest.mark.parametrize('index,count', [(-1, 3), (3, 3), (0, 0)])
def test_invalid_membership_is_rejected(index, count):
    with pytest.raises(ValueError):
        select(scene(), index, count=count)


def split_boundary():
    """One boundary onto one area, broken in two by an inflated cell."""
    grid = np.full((10, 13), 100, dtype=np.int8)
    grid[:5, 1:12] = -1   # the unexplored area
    grid[5, 1:12] = 0     # the boundary along it
    grid[5, 6] = 99       # inflation from something: not a frontier cell
    grid[6:, :] = 0       # corridor
    return grid


def test_one_area_behind_two_clusters_is_a_single_ranked_task():
    # Detection cannot join these two pieces -- an impassable cell sits
    # between them -- but they are not two tasks: whoever takes either piece
    # observes the same space. Ranking them separately is what let rank 1 and
    # rank 2 send two robots to one frontier.
    grid = split_boundary()
    clusters = find_frontier_clusters(grid.ravel(), 13, 10, min_size=1)
    assert len(clusters) == 2

    merged = rank_frontier_clusters(clusters, grid, 1, 5)
    assert len(merged) == 1
    # The second piece stays usable as another approach, it is not discarded.
    assert sorted(merged[0]) == sorted(c for cluster in clusters for c in cluster)

    assert len(rank_frontier_clusters(clusters, grid, 1, 5, merge_overlap=0)) == 2


def twin_rooms():
    """Two identical rooms, equidistant from a robot in the corridor."""
    grid = np.full((10, 25), 100, dtype=np.int8)
    grid[7:, :] = 0
    for left in (2, 16):
        grid[6, left:left + 7] = 0
        grid[:6, left:left + 7] = -1
    return grid


def choose(grid, index, robot, bonus, count=3, **kwargs):
    clusters = find_frontier_clusters(grid.ravel(), grid.shape[1], grid.shape[0], min_size=1)
    return select_visible_gain_frontier(
        clusters, grid.ravel(), grid.shape[1], grid.shape[0], *robot,
        1, 0, 0, sensor_range_m=5, assignment_mode='local_rank', robot_index=index,
        team_size=count, ownership_bonus=bonus, **kwargs)


def test_robots_in_the_same_place_take_different_tasks():
    # Two robots at one spot with two equally good rooms in front of them.
    # Nothing about their own utility distinguishes them, so without a
    # strong enough ownership preference they both choose the same room --
    # which is what the fielded 1.25 did.
    grid, pose = twin_rooms(), (12.5, 8.5)
    assert choose(grid, 0, pose, 1.0) == choose(grid, 1, pose, 1.0)
    assert choose(grid, 0, pose, 2.0) != choose(grid, 1, pose, 2.0)


def far_prize():
    """A small room at the robot's feet; the map's best room far away."""
    grid = np.full((10, 40), 100, dtype=np.int8)
    grid[7:, :] = 0
    grid[6, 2:6] = 0
    grid[:6, 2:6] = -1     # small, adjacent
    grid[6, 30:38] = 0
    grid[:6, 30:38] = -1   # large, ~28 m away
    return grid


def test_ownership_does_not_send_a_robot_across_the_mapped_map():
    # The robot owns the distant rank. robot_rank drives there regardless of
    # cost -- that is how robot3 came to pick a frontier 44 m away with work
    # 4 m in front of it -- while local_rank keeps the bonus a preference.
    grid, pose = far_prize(), (3.5, 8.5)
    clusters = find_frontier_clusters(grid.ravel(), 40, 10, min_size=1)
    strict = select_visible_gain_frontier(
        clusters, grid.ravel(), 40, 10, *pose, 1, 0, 0, sensor_range_m=5,
        assignment_mode='robot_rank', robot_index=2, team_size=3)
    assert strict[0] > 25                      # owns the far room, goes there

    assert choose(grid, 2, pose, 2.0)[0] < 10  # ...the preference does not
    # A big enough bonus buys separation with travel again: that is the knob.
    assert choose(grid, 2, pose, 20.0)[0] > 25


def test_ownership_bonus_below_one_is_rejected():
    with pytest.raises(ValueError):
        choose(twin_rooms(), 0, (12.5, 8.5), 0.9)
