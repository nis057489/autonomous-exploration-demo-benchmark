import numpy as np
from lite_frontier_explorer.reservations import GoalReservations
from lite_frontier_explorer.frontier_detection import find_frontier_clusters, select_visible_gain_frontier


def test_lease_expiry_release_and_reordered_delivery():
    claims = GoalReservations(ttl=8)
    claims.receive(0, 10, (2., 3.), 11)
    assert claims.active(11) == {0: (2., 3.)}
    assert claims.active(19) == {}
    claims.receive(0, 20, None, 21)
    claims.receive(0, 15, (2., 3.), 21)
    assert claims.active(21) == {}
    claims.receive(0, 30, (2., 3.), 21)  # Future clock must not pin a claim.
    assert claims.active(21) == {}


def test_delayed_claim_does_not_get_a_fresh_lease_on_receipt():
    claims = GoalReservations(ttl=8)
    claims.receive(0, 1, (2., 3.), 20)
    assert claims.active(20) == {}


def test_delivered_reservation_changes_assignment_without_transport_label():
    grid = np.full((10, 36), 100, dtype=np.int8)
    grid[7:, :] = 0
    for col in (2, 16, 30):
        grid[6, col] = 0
        grid[5, col] = -1
    clusters = find_frontier_clusters(grid.ravel(), 36, 10, min_size=1)
    def choose(claims):
        return select_visible_gain_frontier(clusters, grid.ravel(), 36, 10,
            16.5, 8.5, 1, 0, 0, reserved_goals=claims, reservation_radius_m=6)
    first = choose([])
    second = choose([first])
    assert first == (16.5, 6.5)
    assert second != first
    assert choose([]) == first  # No delivered claim means no hidden coordination.
    assert choose([first, second, (30.5, 6.5)]) is None


def test_reservation_does_not_exclude_a_disconnected_parallel_hallway():
    grid = np.zeros((9, 9), dtype=np.int8)
    grid[:, 4] = 100
    grid[0, :] = -1
    clusters = find_frontier_clusters(grid.ravel(), 9, 9, min_size=1)
    goal = select_visible_gain_frontier(clusters, grid.ravel(), 9, 9,
        6.5, 4.5, 1, 0, 0, reserved_goals=[(2.5, 1.5)], reservation_radius_m=6)
    assert goal is not None and goal[0] > 4
