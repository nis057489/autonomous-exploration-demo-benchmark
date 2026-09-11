import numpy as np
import pytest

from lite_frontier_explorer.frontier_detection import (
    find_frontier_clusters, obsolete_goal_ticks, select_visible_gain_frontier,
    visible_unknown_area,
)

U, F, O = -1, 0, 100


def choose(grid, robot=(2.5, 8.5), **kwargs):
    clusters = find_frontier_clusters(grid.ravel(), grid.shape[1], grid.shape[0], min_size=1)
    return select_visible_gain_frontier(
        clusters, grid.ravel(), grid.shape[1], grid.shape[0], *robot,
        1.0, 0.0, 0.0, **kwargs)


def rooms():
    grid = np.full((10, 20), O, dtype=np.int8)
    grid[7:, :] = F
    grid[6, 2] = F
    grid[5, 2] = U  # One-cell nearby pocket.
    grid[6, 10:18] = F
    grid[:6, 10:18] = U  # Substantially larger, farther room.
    return grid


def test_larger_visible_room_beats_nearby_sliver():
    grid = rooms()
    goal = choose(grid, sensor_range_m=5)
    assert goal is not None and goal[0] >= 10


def test_peer_knowledge_changes_destination():
    grid = rooms()
    assert choose(grid, sensor_range_m=5)[0] >= 10
    grid[:6, 10:18] = F  # Teammate mapped the large room.
    assert choose(grid, sensor_range_m=5) == (2.5, 6.5)


def test_wide_opening_has_more_gain_than_slit_into_same_unknown_region():
    grid = np.full((12, 24), U, dtype=np.int8)
    grid[8, :] = O
    grid[9:, :] = F
    grid[8, 3] = F
    grid[8, 13:22] = F
    slit = visible_unknown_area(grid, 8, 3, 1, 7)
    wide = visible_unknown_area(grid, 8, 17, 1, 7)
    assert wide > slit


def test_active_and_selected_goal_use_same_gain_and_path_cost():
    grid = rooms()
    active = {}
    scores = []
    goal = choose(grid, sensor_range_m=5, diagnostics=scores,
                  active_goal=(2.5, 6.5), active_score=active)
    selected = next(s for s in scores if (s['x'], s['y']) == goal)
    assert selected['path_m'] > active['path_m']
    assert selected['utility'] > 1.25 * active['utility']
    # Peer observations remove the active destination's remaining information.
    grid[5, 2] = F
    choose(grid, sensor_range_m=5, active_goal=(2.5, 6.5), active_score=active)
    assert active['utility'] == 0


def test_wall_blocks_gain_and_inflation_does_not():
    grid = np.full((9, 9), F, dtype=np.int8)
    grid[:3, :] = U
    grid[3, :] = O
    assert visible_unknown_area(grid, 5, 4, 1, 5) == 0
    grid[3, :] = 90  # Inflation is not a physical obstacle.
    assert visible_unknown_area(grid, 5, 4, 1, 5) > 0


def test_no_diagonal_visibility_through_wall_corner():
    grid = np.array([[F, O], [O, U]], dtype=np.int8)
    assert visible_unknown_area(grid, 0, 0, 1, 3) == 0


def test_no_five_square_metre_cap_and_each_cell_counted_once():
    grid = np.full((161, 161), U, dtype=np.int8)
    grid[80, 80] = F
    area = visible_unknown_area(grid, 80, 80, 0.05, 3)
    assert 20 < area <= np.pi * 3 ** 2
    assert visible_unknown_area(np.array([[F, U]], dtype=np.int8), 0, 0, 1, 10) == 1


def test_extended_cluster_returns_reachable_cell_not_centroid():
    grid = np.full((9, 9), O, dtype=np.int8)
    grid[2:7, 2:7] = U
    grid[1, 1:8] = F
    grid[7, 1:8] = F
    grid[1:8, 1] = F
    grid[1:8, 7] = F
    goal = choose(grid, robot=(1.5, 1.5), sensor_range_m=4)
    assert goal is not None
    assert grid[int(goal[1]), int(goal[0])] == F


def test_equal_gain_prefers_nearer_and_is_order_independent():
    grid = np.array([[U, F, F, F, F, F, F, F, U]], dtype=np.int8)
    clusters = [[(0, 1)], [(0, 7)]]
    for ordered in (clusters, clusters[::-1]):
        assert select_visible_gain_frontier(
            ordered, grid.ravel(), 9, 1, 3.5, 0.5, 1, 0, 0,
            sensor_range_m=2) == (1.5, 0.5)


def test_unreachable_rich_frontier_cannot_win():
    grid = rooms()
    grid[7:, 8] = O
    assert choose(grid, sensor_range_m=5) == (2.5, 6.5)


def test_sampling_covers_long_frontier_and_reports_scores():
    grid = rooms()
    scores = []
    choose(grid, sensor_range_m=5, diagnostics=scores, max_viewpoints=3)
    rich = [s for s in scores if s['x'] >= 10]
    assert len(rich) == 3
    assert max(s['x'] for s in rich) - min(s['x'] for s in rich) >= 6
    for score in scores:
        assert score['utility'] == pytest.approx(score['gain_m2'] / (1 + score['path_m']))


def test_no_unknown_means_no_goal():
    assert choose(np.zeros((10, 20), dtype=np.int8)) is None


@pytest.mark.parametrize('kwargs', [{'sensor_range_m': 0}, {'distance_weight': -1}, {'max_viewpoints': 0}])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        choose(rooms(), **kwargs)


def obsolete(ticks, gain, goal=(20.0, 0.0), active=(0.0, 0.0), min_gain=0.5, separation=1.0):
    score = {} if gain is None else dict(gain_m2=gain, path_m=5.0, utility=gain / 6)
    return obsolete_goal_ticks(ticks, score, goal, active, min_gain, separation)


def test_a_destination_with_nothing_left_to_see_is_counted_out():
    # A teammate mapped the area this robot is still driving toward. The
    # count has to survive consecutive ticks, so one stale costmap cannot
    # discard a live goal, and it resets the moment gain reappears.
    assert obsolete(0, 0.0) == 1
    assert obsolete(1, 0.0) == 2
    assert obsolete(4, 12.0) == 0


def test_an_obsolete_goal_is_kept_when_there_is_nowhere_else_to_go():
    # "Finished" is only actionable next to "so go here instead": with no
    # alternative, or only one at the same place, the robot keeps driving
    # rather than cancelling into an empty choice.
    assert obsolete(3, 0.0, goal=None) == 0
    assert obsolete(3, 0.0, goal=(0.5, 0.0)) == 0


def test_an_unscorable_active_goal_is_not_called_obsolete():
    # No active_score means the selector could not score that cell at all
    # (unreachable, or lethal on this costmap) -- which is a job for the
    # stuck timer, not evidence that the frontier is explored.
    assert obsolete(3, None) == 0
    assert obsolete(3, 0.0, active=None) == 0
