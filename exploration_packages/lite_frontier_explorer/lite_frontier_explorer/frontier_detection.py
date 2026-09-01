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


def partition_frontiers_by_ownership(cluster_world_xys, nav_map_data, nav_map_width,
                                      nav_map_height, nav_map_resolution,
                                      nav_map_origin_x, nav_map_origin_y,
                                      self_id, self_x, self_y, peer_poses):
    """Return a boolean list, aligned with cluster_world_xys, marking which
    frontier centroids "self" owns in a local Manhattan-distance Voronoi
    partition over self + peers.

    peer_poses: dict[peer_id, (x, y)] world poses, e.g. {"robot2": (1.2, -0.4)}.
    Ownership: self's L1 distance |dx|+|dy| to a point must be strictly
    less than every peer's, or tied with self_id sorting lower (plain str
    comparison) among tied ids -- so every robot computing this
    independently agrees on the same winner without communicating.
    peer_poses={} (no peers known this tick) makes every point owned by
    self -- the no-op / single-robot / degraded-peer-visibility default.

    A point that nav_map still shows as unknown (-1) at its own location is
    excluded regardless of distance -- claiming ownership of a point this
    robot doesn't yet know about via its own (locally-observed + relayed
    teammate) map is meaningless, and this is the deliberate hook for
    making territory assignment sensitive to how well the active
    map_transport (baseline/vxch/zstd) has actually communicated the
    shared map: a robot with a lagged/degraded nav_map gates out more
    candidates here. A point outside nav_map's current bounds is treated
    the same way (unknown).
    """
    def _is_known(x, y):
        row = int((y - nav_map_origin_y) / nav_map_resolution)
        col = int((x - nav_map_origin_x) / nav_map_resolution)
        if not (0 <= row < nav_map_height and 0 <= col < nav_map_width):
            return False
        return nav_map_data[row * nav_map_width + col] != -1

    owned = []
    for x, y in cluster_world_xys:
        if not _is_known(x, y):
            owned.append(False)
            continue

        self_dist = abs(x - self_x) + abs(y - self_y)
        is_owned = True
        for peer_id, (px, py) in peer_poses.items():
            peer_dist = abs(x - px) + abs(y - py)
            if peer_dist < self_dist or (peer_dist == self_dist and peer_id < self_id):
                is_owned = False
                break
        owned.append(is_owned)
    return owned


def _free_space_distances(data, width, height, robot_x, robot_y, resolution,
                           origin_x, origin_y, path_occ_threshold):
    """BFS out from the robot's cell over 4-connected passable space (cost <
    path_occ_threshold; unknown/occupied cells block the walk). Returns a
    height x width array of step counts, -1 where unreached.

    path_occ_threshold is deliberately not the same knob as the
    occ_threshold used to classify frontier cells. That one has to stay
    low (default 50) so a frontier cell means "genuinely open ground",
    but nav2 itself only refuses to drive through cost ~99 (inscribed --
    the footprint can't fit) and 100 (lethal); everything below that is
    real, costed-but-drivable space it will happily cross. Gating this
    walk on the frontier threshold treated most of the map near any wall
    as impassable and made every frontier look unreachable.
    """
    grid = np.asarray(data, dtype=np.int8).reshape(height, width)
    free_mask = (grid >= 0) & (grid < path_occ_threshold)

    dist = np.full((height, width), -1, dtype=np.int32)
    start_row = int((robot_y - origin_y) / resolution)
    start_col = int((robot_x - origin_x) / resolution)
    if not (0 <= start_row < height and 0 <= start_col < width):
        return dist

    # Seed the walk at the robot's cell unconditionally, even if its own
    # cost happens to sit at/above occ_threshold (e.g. inflation from a
    # wall the robot is currently next to). The robot is physically
    # there regardless of what the costmap says about that one cell --
    # gating the seed on free_mask would make the whole grid unreachable
    # (dist all -1) any time the robot's momentary cell cost creeps up,
    # rejecting every frontier including ones plainly in the open.
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


def _free_space_distances_with_parents(data, width, height, robot_x, robot_y, resolution,
                                        origin_x, origin_y, path_occ_threshold):
    """Same BFS as _free_space_distances, but also records each cell's
    parent so a shortest path back to the robot can be reconstructed (used
    to count turns along the path, not just its length).
    """
    grid = np.asarray(data, dtype=np.int8).reshape(height, width)
    free_mask = (grid >= 0) & (grid < path_occ_threshold)

    dist = np.full((height, width), -1, dtype=np.int32)
    parent_r = np.full((height, width), -1, dtype=np.int32)
    parent_c = np.full((height, width), -1, dtype=np.int32)
    start_row = int((robot_y - origin_y) / resolution)
    start_col = int((robot_x - origin_x) / resolution)
    if not (0 <= start_row < height and 0 <= start_col < width):
        return dist, parent_r, parent_c

    dist[start_row, start_col] = 0
    queue = deque([(start_row, start_col)])
    while queue:
        r, c = queue.popleft()
        d = dist[r, c]
        for dr, dc in _NEIGHBOR_OFFSETS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and free_mask[nr, nc] and dist[nr, nc] < 0:
                dist[nr, nc] = d + 1
                parent_r[nr, nc] = r
                parent_c[nr, nc] = c
                queue.append((nr, nc))
    return dist, parent_r, parent_c


def _count_path_turns(parent_r, parent_c, start_row, start_col, target_row, target_col):
    """Walk the BFS parent chain back from target to start and count how
    many times the direction of travel changes -- a cheap stand-in for
    how much a path through that route would make the robot turn, since
    raw step count treats a straight corridor and a zig-zag of the same
    length as equally cheap.
    """
    path = []
    r, c = target_row, target_col
    while (r, c) != (start_row, start_col):
        path.append((r, c))
        pr, pc = parent_r[r, c], parent_c[r, c]
        if pr < 0:
            return 0  # disconnected -- shouldn't happen for a reachable target
        r, c = int(pr), int(pc)
    path.append((start_row, start_col))
    path.reverse()

    turns = 0
    prev_dir = None
    for i in range(1, len(path)):
        d = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        if prev_dir is not None and d != prev_dir:
            turns += 1
        prev_dir = d
    return turns


def _count_path_unknown_exposure(parent_r, parent_c, start_row, start_col, target_row,
                                  target_col, unknown_mask, height, width):
    """Walk the BFS parent chain back from target to start and count how
    many cells along that route border at least one unknown cell.

    A destination's own gain (estimate_frontier_region_sizes) only scores
    what's unknown right at the frontier -- it gives zero credit to a
    route that spends its whole trip skirting a big unexplored area versus
    one of identical length that cuts straight through territory that's
    already fully mapped. This is a cheap per-cell proxy for "how much
    currently-unknown space would the robot's sensors sweep past just by
    driving this route," used to bias selection toward tours that reveal
    new area along the way, not just at the endpoint.
    """
    exposure = 0
    r, c = target_row, target_col
    while True:
        for dr, dc in _NEIGHBOR_OFFSETS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and unknown_mask[nr, nc]:
                exposure += 1
                break
        if (r, c) == (start_row, start_col):
            break
        pr, pc = parent_r[r, c], parent_c[r, c]
        if pr < 0:
            break  # disconnected -- shouldn't happen for a reachable target
        r, c = int(pr), int(pc)
    return exposure


def estimate_frontier_gains(clusters, data, width, height, resolution, sensor_range_m):
    """Like estimate_frontier_gain, but scored for all clusters at once
    with each unknown cell's credit given to only its nearest cluster
    centroid (Voronoi-style). Adjacent frontiers' sensor-range windows
    overlap, so scoring them independently lets them both claim the same
    patch of unknown space -- inflating whichever frontiers happen to be
    clustered together relative to isolated ones. Returns a list of gains
    in the same order as clusters.
    """
    if not clusters:
        return []

    grid = np.asarray(data, dtype=np.int8).reshape(height, width)
    unknown_mask = grid == -1

    centroids = np.array([
        [sum(r for r, _ in cl) / len(cl), sum(c for _, c in cl) / len(cl)]
        for cl in clusters
    ])
    radius_cells = sensor_range_m / resolution

    r0 = max(0, int(math.floor(centroids[:, 0].min() - radius_cells)))
    r1 = min(height, int(math.ceil(centroids[:, 0].max() + radius_cells)) + 1)
    c0 = max(0, int(math.floor(centroids[:, 1].min() - radius_cells)))
    c1 = min(width, int(math.ceil(centroids[:, 1].max() + radius_cells)) + 1)
    if r0 >= r1 or c0 >= c1:
        return [0] * len(clusters)

    local_unknown = np.argwhere(unknown_mask[r0:r1, c0:c1])
    if local_unknown.size == 0:
        return [0] * len(clusters)
    unknown_rc = local_unknown + np.array([r0, c0])

    diff = unknown_rc[:, None, :] - centroids[None, :, :]
    dist2 = (diff ** 2).sum(axis=2)
    nearest_cluster = np.argmin(dist2, axis=1)
    within_range = dist2[np.arange(len(unknown_rc)), nearest_cluster] <= radius_cells ** 2

    gains = [0] * len(clusters)
    for k in range(len(clusters)):
        gains[k] = int(np.count_nonzero(within_range & (nearest_cluster == k)))
    return gains


def estimate_frontier_region_size(cluster, data, width, height, cap=2000):
    """Flood-fill the connected unknown region this frontier opens onto
    and return its size (capped at `cap` cells for performance).

    estimate_frontier_gain/estimate_frontier_gains only count unknown
    cells within a fixed sensor_range_m window of the frontier -- so a
    tiny closet and a huge unmapped wing score almost the same the moment
    both are bigger than that window, since the window just fills up
    either way. That made the gain-aware selectors indifferent between
    "leads to a big empty room" and "leads to a broom closet next door,"
    so distance ended up deciding, and a nearby closet would beat a
    farther room every time. Region size instead scales with how much
    unknown area is actually back there, unbounded by sensor range, so a
    frontier onto a large unexplored area scores higher than one onto a
    small pocket regardless of how close either is.

    Uncapped this could walk the entire remaining unknown map for a
    frontier at the edge of one big unexplored area, so it stops at
    `cap` -- callers only need "this is clearly bigger than that", not an
    exact count once a region is already large enough to be an obvious
    win.
    """
    grid = np.asarray(data, dtype=np.int8).reshape(height, width)
    unknown = grid == -1

    visited = set()
    queue = deque()
    for r, c in cluster:
        for dr, dc in _NEIGHBOR_OFFSETS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and unknown[nr, nc] and (nr, nc) not in visited:
                visited.add((nr, nc))
                queue.append((nr, nc))

    count = 0
    while queue and count < cap:
        r, c = queue.popleft()
        count += 1
        for dr, dc in _NEIGHBOR_OFFSETS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and unknown[nr, nc] and (nr, nc) not in visited:
                visited.add((nr, nc))
                queue.append((nr, nc))
    return count


def estimate_frontier_region_sizes(clusters, data, width, height, cap=2000):
    """estimate_frontier_region_size for every cluster, in order."""
    return [estimate_frontier_region_size(cl, data, width, height, cap=cap) for cl in clusters]


def estimate_frontier_gain(cluster, data, width, height, resolution, sensor_range_m):
    """Estimate how many new (currently unknown) cells a robot standing at
    this cluster's centroid could observe: the count of unknown cells
    within sensor_range_m of the centroid.

    This is a cheap proxy for information gain -- it doesn't raycast, so
    it can overcount unknown cells that are actually hidden behind a wall
    from the centroid's vantage point. That's an acceptable trade for a
    per-tick numpy op; only the bounding box around the centroid (not the
    whole grid) is scanned, so cost stays proportional to sensor range,
    not map size.
    """
    grid = np.asarray(data, dtype=np.int8).reshape(height, width)
    avg_r = sum(r for r, _ in cluster) / len(cluster)
    avg_c = sum(c for _, c in cluster) / len(cluster)

    radius_cells = sensor_range_m / resolution
    r0 = max(0, int(math.floor(avg_r - radius_cells)))
    r1 = min(height, int(math.ceil(avg_r + radius_cells)) + 1)
    c0 = max(0, int(math.floor(avg_c - radius_cells)))
    c1 = min(width, int(math.ceil(avg_c + radius_cells)) + 1)
    if r0 >= r1 or c0 >= c1:
        return 0

    window = grid[r0:r1, c0:c1]
    rows = np.arange(r0, r1).reshape(-1, 1)
    cols = np.arange(c0, c1).reshape(1, -1)
    within_range = (rows - avg_r) ** 2 + (cols - avg_c) ** 2 <= radius_cells ** 2

    return int(np.count_nonzero((window == -1) & within_range))


def select_best_frontier(clusters, data, width, height, robot_x, robot_y, resolution,
                          origin_x, origin_y, path_occ_threshold=99, min_distance_m=0.0,
                          sensor_range_m=3.0, distance_weight=1.0):
    """Return the (x, y) world centroid of the cluster with the best
    gain-vs-cost utility, or None if no cluster qualifies.

    Unlike select_nearest_frontier (pure path-distance), this scores each
    reachable cluster by expected new observations (estimate_frontier_gain)
    divided by how expensive it is to reach, so a frontier that opens onto
    a lot of unknown space can beat one that's merely closer.
    distance_weight trades off cost vs. gain: higher values penalize
    distant frontiers more heavily relative to their gain.
    """
    dist_grid = _free_space_distances(
        data, width, height, robot_x, robot_y, resolution, origin_x, origin_y, path_occ_threshold)
    gains = estimate_frontier_gains(clusters, data, width, height, resolution, sensor_range_m)

    best_xy = None
    best_utility = None
    for cluster, gain in zip(clusters, gains):
        path_dist = min(
            (dist_grid[r, c] for r, c in cluster if dist_grid[r, c] >= 0),
            default=None,
        )
        if path_dist is None:
            continue  # no known-free path from the robot to this cluster
        path_dist_m = path_dist * resolution
        if path_dist_m < min_distance_m:
            continue

        utility = gain / (1.0 + distance_weight * path_dist_m)

        if best_utility is None or utility > best_utility:
            best_utility = utility
            best_xy = cluster_centroid_world(cluster, resolution, origin_x, origin_y)
    return best_xy


def select_nearest_high_gain_frontier(clusters, data, width, height, robot_x, robot_y, resolution,
                                       origin_x, origin_y, path_occ_threshold=99, min_distance_m=0.0,
                                       gain_region_cap=2000, gain_threshold_ratio=0.75,
                                       turn_penalty_m=0.0, preferred_direction=None,
                                       hysteresis_bonus_m=0.0, path_exposure_bonus_m=0.0):
    """Return the (x, y) world centroid of the "nearest" cluster among
    those whose gain is within gain_threshold_ratio of the best gain seen,
    or None if no cluster qualifies.

    select_best_frontier's single gain/cost utility tends to send every
    robot on the team at the same globally-best frontier, since it's
    everyone's top pick regardless of where they each started -- multiple
    robots converging on one spot while the rest of the map sits
    unexplored. This instead treats "high gain" as a bar to clear (any
    cluster scoring at least gain_threshold_ratio * best_gain) and then
    picks whichever qualifying cluster is cheapest for *this* robot to
    reach, so robots starting in different places head to different rich
    frontiers instead of racing each other to the single richest one.

    Gain here is estimate_frontier_region_size -- the size of the
    connected unknown area a frontier opens onto, not a fixed-radius
    window -- so a frontier leading into a big unexplored room clears the
    bar over one leading into a small pocket, regardless of which is
    nearer. (A fixed sensor-radius window saturates the moment the
    unknown area behind it exceeds the window, so a tiny pocket and a
    huge room score almost the same and the nearer one wins by default --
    that's what sent robots to small nearby frontiers while a much richer
    one sat farther away.)

    "Cheapest" is path distance plus two optional terms that make it
    track actual drive cost instead of pure step count, which is what let
    the robot zig-zag between similarly-close frontiers: turn_penalty_m
    adds a cost per direction change along the BFS path (a straight
    corridor beats a same-length zig-zag), and preferred_direction (a
    (dx, dy) world-frame vector, typically the robot's recent heading)
    applies hysteresis_bonus_m of bonus/penalty scaled by how aligned a
    candidate is with that direction, so the robot favors continuing the
    way it was already going over backtracking for a marginally better
    option.

    path_exposure_bonus_m discounts cost by this amount per path cell
    that borders unknown space (_count_path_unknown_exposure), so a route
    that skirts an unexplored area on the way to its frontier beats an
    equal-length, equal-turn route that cuts through territory that's
    already fully mapped. Gain (above) only scores what's unknown at the
    destination -- this is what makes the *trip itself* worth more when
    it will sweep past more new area, not just the endpoint.
    """
    dist_grid, parent_r, parent_c = _free_space_distances_with_parents(
        data, width, height, robot_x, robot_y, resolution, origin_x, origin_y, path_occ_threshold)
    gains = estimate_frontier_region_sizes(clusters, data, width, height, cap=gain_region_cap)
    unknown_mask = None
    if path_exposure_bonus_m:
        unknown_mask = np.asarray(data, dtype=np.int8).reshape(height, width) == -1

    start_row = int((robot_y - origin_y) / resolution)
    start_col = int((robot_x - origin_x) / resolution)

    pref_unit = None
    if preferred_direction is not None:
        pdx, pdy = preferred_direction
        pref_norm = math.hypot(pdx, pdy)
        if pref_norm > 1e-9:
            pref_unit = (pdx / pref_norm, pdy / pref_norm)

    reachable = []
    for cluster, gain in zip(clusters, gains):
        best_cell = min(
            ((dist_grid[r, c], r, c) for r, c in cluster if dist_grid[r, c] >= 0),
            default=None,
        )
        if best_cell is None:
            continue  # no known-free path from the robot to this cluster
        path_dist, target_row, target_col = best_cell
        path_dist_m = path_dist * resolution
        if path_dist_m < min_distance_m:
            continue

        cost_m = path_dist_m
        if turn_penalty_m:
            turns = _count_path_turns(parent_r, parent_c, start_row, start_col, target_row, target_col)
            cost_m += turn_penalty_m * turns
        if path_exposure_bonus_m:
            exposure = _count_path_unknown_exposure(
                parent_r, parent_c, start_row, start_col, target_row, target_col,
                unknown_mask, height, width)
            cost_m -= path_exposure_bonus_m * exposure

        goal_xy = cluster_centroid_world(cluster, resolution, origin_x, origin_y)
        if pref_unit is not None and hysteresis_bonus_m:
            cdx, cdy = goal_xy[0] - robot_x, goal_xy[1] - robot_y
            cand_norm = math.hypot(cdx, cdy)
            if cand_norm > 1e-9:
                alignment = (cdx / cand_norm * pref_unit[0] + cdy / cand_norm * pref_unit[1])
                cost_m -= hysteresis_bonus_m * alignment

        reachable.append((goal_xy, cost_m, gain))

    if not reachable:
        return None

    best_gain = max(gain for _, _, gain in reachable)
    gain_floor = best_gain * gain_threshold_ratio

    best_xy = None
    best_cost = None
    for goal_xy, cost_m, gain in reachable:
        if gain < gain_floor:
            continue
        if best_cost is None or cost_m < best_cost:
            best_cost = cost_m
            best_xy = goal_xy
    return best_xy


def select_nearest_frontier(clusters, data, width, height, robot_x, robot_y, resolution,
                             origin_x, origin_y, path_occ_threshold=99, min_distance_m=0.0):
    """Return the (x, y) world centroid of the cluster with the shortest
    walkable path through known free space, at least min_distance_m away
    (measured along that path), or None if no cluster qualifies.

    Straight-line nearest can pick a frontier that's close as the crow
    flies but behind a wall, sending the robot on a long detour (or into
    a costmap-inflated no-path zone) instead of a farther-but-open one.
    Ranking by BFS step count through free space avoids that.

    path_occ_threshold defaults to 99 (nav2's inscribed-obstacle cutoff,
    the point at which the robot's footprint genuinely can't fit) rather
    than the much lower occ_threshold used elsewhere for frontier-cell
    classification -- see _free_space_distances for why.
    """
    dist_grid = _free_space_distances(
        data, width, height, robot_x, robot_y, resolution, origin_x, origin_y, path_occ_threshold)

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
