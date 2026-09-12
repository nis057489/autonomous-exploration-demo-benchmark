"""Choose frontier work from delivered maps and expiring peer reservations.

Nav2 supplies reachability costs. Occupancy maps supply information gain.
Reservation inputs must be relayed through the experiment's impaired links.
"""

import math

import numpy as np

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import qos_profile_sensor_data
from lite_frontier_explorer.reservations import GoalReservations
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from lite_frontier_explorer.frontier_detection import (
    _free_space_distances,
    align_information_grid,
    cluster_centroid_world,
    find_frontier_clusters,
    occupancy_constrained_costmap,
    select_best_frontier,
    select_nearest_frontier,
    select_nearest_high_gain_frontier,
    select_visible_gain_frontier,
)


class LiteFrontierExplorer(Node):
    def __init__(self):
        super().__init__('lite_frontier_explorer')

        self.declare_parameter('costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        # 1 cell = no size filtering: at mission start (small/fragmented
        # initial costmap) a >1 threshold can discard every detected
        # cluster, which _tick() then misreads as "no frontiers left --
        # exploration complete" even though frontier cells exist. Selection
        # strategies already rank by gain/distance, so the best frontier
        # wins regardless of cluster size -- there's no need to also gate
        # on a minimum to avoid picking a "bad" one.
        self.declare_parameter('min_frontier_size_cells', 1)
        self.declare_parameter('min_frontier_distance_m', 0.5)
        self.declare_parameter('goal_blacklist_radius_m', 1.0)
        self.declare_parameter('goal_blacklist_duration_s', 60.0)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('path_occ_threshold', 99)
        self.declare_parameter('information_map_topic', '')
        self.declare_parameter('information_occ_threshold', 50)
        # visible_gain scores wall-occluded unknown area at reachable viewpoints.
        # nearest, best_gain and nearest_high_gain remain as legacy comparisons.
        self.declare_parameter('selection_strategy', 'visible_gain')
        self.declare_parameter('frontier_assignment', 'independent')
        self.declare_parameter('robot_index', 0)  # zero-based, supplied by launch
        self.declare_parameter('team_size', 1)
        self.declare_parameter('reservation_topic', '')
        self.declare_parameter('reservation_peer_topics', [''])
        self.declare_parameter('reservation_ttl_s', 8.0)
        self.declare_parameter('reservation_radius_m', 6.0)
        self.declare_parameter('min_gain_m2', 0.5)
        self.declare_parameter('sensor_range_m', 3.0)
        self.declare_parameter('gain_max_viewpoints', 5)
        self.declare_parameter('gain_distance_weight', 1.0)
        self.declare_parameter('gain_threshold_ratio', 0.75)
        # Cap on the connected-unknown-region flood fill used by
        # 'nearest_high_gain' to size how much unexplored area a frontier
        # opens onto -- unlike sensor_range_m's fixed window, this scales
        # with the real size of the area behind the frontier, so it stops
        # small nearby frontiers from outscoring big distant ones just
        # because both fill up the same small window.
        self.declare_parameter('gain_region_cap', 2000)
        # Cost per direction change along the candidate path -- penalizes a
        # zig-zag route over a straight one of the same step count. Stays
        # small on purpose: measured on the real 0.05 m/cell global costmap
        # the BFS yields L-shaped paths with 0-1 direction changes (not
        # per-cell staircases), so this lands under a metre and acts as a
        # tiebreaker rather than swamping path distance. It penalizes
        # off-axis goals, not reversals -- hysteresis_bonus_m below is the
        # knob that discourages turning around.
        self.declare_parameter('turn_penalty_m', 0.5)
        # Bonus/penalty (scaled by directional alignment) applied to a
        # candidate relative to the direction of the last goal sent, so the
        # robot favors continuing the way it was already heading over
        # backtracking for a marginally-better frontier.
        #
        # Alignment runs +1 (dead ahead) to -1 (directly behind), so the
        # ahead-vs-behind spread is 2 * this value: a frontier BEHIND the
        # robot has to be that much closer to win. The old 1.5 put that at
        # only 3 m, which anything mid-corridor clears easily -- that's what
        # had robots abandoning a corridor partway to double back for a
        # marginally nearer frontier. 6.0 sets the spread to ~12 m, about one
        # full corridor leg in long_t (legs ~13-15 m, map ~20x19 m), so a
        # robot committed to a leg keeps going unless something behind it is
        # dramatically cheaper -- in practice, until the direction it's facing
        # is close to exhausted.
        #
        # This is a relative cost term, not a gate: when every remaining
        # frontier is behind, they all take the same penalty and one still
        # wins normally, so the robot can't deadlock refusing to turn around.
        self.declare_parameter('hysteresis_bonus_m', 6.0)
        self.declare_parameter('replan_period_s', 3.0)
        # If, while a goal is in flight, replanning selects a frontier this
        # far (or more) from the goal actually being driven to, cancel the
        # in-flight goal and switch to the new one instead of riding out the
        # stale goal until it succeeds/aborts.
        self.declare_parameter('goal_preempt_distance_m', 1.0)
        # ...and only if the new frontier is genuinely CLOSER by this margin.
        #
        # goal_preempt_distance_m on its own tests that the new pick is in a
        # different PLACE, not that it is a better one, so any instability in
        # the selector's ranking is enough to trigger a swap. Measured on the
        # 20260911 office run that is exactly what happens: robot2 ping-ponged
        # between frontiers ~19 m apart, preempting 33 of its 34 goals and
        # never travelling far enough to reach any of them. Every flip cleared
        # a 1 m bar by 19 m.
        #
        # Requiring the new goal to be materially closer to the robot than the
        # one already in flight breaks that symmetry: A -> B needs B nearer
        # than A by the margin, so B -> A cannot also qualify.
        self.declare_parameter('goal_preempt_improvement_m', 2.0)
        # visible_gain compares the same utility used to select destinations,
        # so a farther but much more informative goal can replace a poor one.
        self.declare_parameter('goal_preempt_utility_ratio', 1.25)
        # Backstop for any oscillation the margin above does not catch (the
        # robot is moving, so both distances change every tick). After this
        # many preemptions with no goal ever reaching a terminal state, commit
        # to the current goal and stop preempting until nav2 succeeds/aborts
        # it or goal_stuck_timeout_s fires. Committing is what lets a goal
        # FAIL, and a failure is the only thing that puts an unreachable
        # frontier in the blacklist -- thrashing forever never does.
        # Legacy selectors only: visible_gain always compares current map
        # utility with ratio hysteresis, so new evidence can change a decision.
        # 0 disables the backstop.
        self.declare_parameter('max_consecutive_preemptions', 3)
        # Distinguishing "this frontier is unreachable" from "this ROBOT is
        # wedged". Both look identical to the no-progress timer -- the robot
        # stops moving -- but they need opposite responses, and getting it
        # wrong is destructive. Measured on the 20260911 office run: robot1
        # jammed in a doorway at (4.65, 14.05), sat at that exact pose for
        # five consecutive goals, and blacklisted frontiers at (19.41, 0.63),
        # (0.88, 11.01), (2.34, 21.43) and (-5.95, 16.58) -- scattered across
        # the whole building, every one of them perfectly reachable. It was
        # poisoning its own blacklist with the entire map because it could not
        # move at all.
        #
        # If two consecutive no-progress events happen within this radius of
        # each other, the robot has not moved between them, so the frontier is
        # not what failed. Blame nothing, and let nav2's recovery chain (spin /
        # back-up / clear-costmap) actually run instead of cancelling it at
        # goal_stuck_timeout_s -- recovery is exactly what frees a wedged
        # footprint, and cancelling early is what prevents it.
        self.declare_parameter('wedge_detect_radius_m', 0.5)
        # How many times to let recovery try before giving up on the goal
        # anyway. Still never blacklists it -- a different goal means a
        # different approach direction, which is itself often what unwedges
        # the robot.
        self.declare_parameter('max_wedge_cycles', 3)
        # Optional legacy watchdog, disabled by default. Nav2's pose progress
        # checker and bounded BT retries own stall recovery. Translation alone
        # cannot distinguish being stuck from rotating, waiting, or recovering.
        self.declare_parameter('goal_stuck_timeout_s', 0.0)
        self.declare_parameter('goal_stuck_epsilon_m', 0.15)
        self.declare_parameter('navigate_to_pose_action_name', 'navigate_to_pose')
        self.declare_parameter('frontier_marker_topic', 'explore/frontiers')
        self.declare_parameter('frontier_marker_scale', 0.15)
        self.declare_parameter('frontier_marker_color_r', 0.15)
        self.declare_parameter('frontier_marker_color_g', 0.9)
        self.declare_parameter('frontier_marker_color_b', 0.2)
        # Staggered start: hold this robot still for N seconds after it first
        # becomes ABLE to explore, so a teammate can map a region before this
        # one chooses where to go. That knowledge asymmetry is the whole point
        # -- with every robot starting from an empty map at once, what a peer
        # can tell you is always small and incremental.
        #
        # Deliberately gates goal-sending rather than delaying node startup.
        # A robot whose stack comes up late does its DDS discovery late, under
        # whatever link conditions happen to be live then, and FastDDS does not
        # reliably retry a handshake that failed -- an unmatched pair stays
        # silently unmatched for the whole run and looks exactly like "the
        # transport was bad". Here the full stack is up and discovered from
        # t=0; only the driving waits.
        self.declare_parameter('explore_start_delay_s', 0.0)

        self._costmap_topic = self.get_parameter('costmap_topic').value
        self._global_frame = self.get_parameter('global_frame').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._min_frontier_size = self.get_parameter('min_frontier_size_cells').value
        self._min_frontier_distance_m = self.get_parameter('min_frontier_distance_m').value
        self._goal_blacklist_radius_m = self.get_parameter('goal_blacklist_radius_m').value
        self._occ_threshold = self.get_parameter('occ_threshold').value
        self._path_occ_threshold = self.get_parameter('path_occ_threshold').value
        self._selection_strategy = self.get_parameter('selection_strategy').value
        self._frontier_assignment = self.get_parameter('frontier_assignment').value
        self._robot_index = self.get_parameter('robot_index').value
        self._team_size = self.get_parameter('team_size').value
        if self._frontier_assignment not in ('independent', 'robot_rank'):
            raise ValueError('frontier_assignment must be independent or robot_rank')
        if self._team_size < 1 or not 0 <= self._robot_index < self._team_size:
            raise ValueError('robot_index must be within team_size')
        self._rank_assignment = (self._frontier_assignment == 'robot_rank'
                                 and self._team_size > 1
                                 and self._selection_strategy == 'visible_gain')
        self._sensor_range_m = self.get_parameter('sensor_range_m').value
        self._gain_max_viewpoints = self.get_parameter('gain_max_viewpoints').value
        self._gain_distance_weight = self.get_parameter('gain_distance_weight').value
        if self._selection_strategy not in ('visible_gain', 'nearest_high_gain', 'best_gain', 'nearest'):
            raise ValueError(f'Unknown selection_strategy: {self._selection_strategy}')
        if (self._sensor_range_m <= 0 or self._gain_distance_weight < 0
                or self._gain_max_viewpoints < 1):
            raise ValueError('Invalid frontier gain parameters')
        self._gain_threshold_ratio = self.get_parameter('gain_threshold_ratio').value
        self._gain_region_cap = self.get_parameter('gain_region_cap').value
        self._turn_penalty_m = self.get_parameter('turn_penalty_m').value
        self._hysteresis_bonus_m = self.get_parameter('hysteresis_bonus_m').value
        self._goal_preempt_distance_m = self.get_parameter('goal_preempt_distance_m').value
        self._goal_preempt_improvement_m = float(
            self.get_parameter('goal_preempt_improvement_m').value)
        self._goal_preempt_utility_ratio = float(
            self.get_parameter('goal_preempt_utility_ratio').value)
        if self._goal_preempt_utility_ratio <= 1.0:
            raise ValueError('goal_preempt_utility_ratio must be > 1')
        self._max_consecutive_preemptions = int(
            self.get_parameter('max_consecutive_preemptions').value)
        self._wedge_detect_radius_m = float(
            self.get_parameter('wedge_detect_radius_m').value)
        self._max_wedge_cycles = int(self.get_parameter('max_wedge_cycles').value)
        self._goal_stuck_timeout_s = self.get_parameter('goal_stuck_timeout_s').value
        self._goal_stuck_epsilon_m = self.get_parameter('goal_stuck_epsilon_m').value
        replan_period_s = self.get_parameter('replan_period_s').value
        action_name = self.get_parameter('navigate_to_pose_action_name').value
        self._marker_scale = self.get_parameter('frontier_marker_scale').value
        self._marker_color = (
            self.get_parameter('frontier_marker_color_r').value,
            self.get_parameter('frontier_marker_color_g').value,
            self.get_parameter('frontier_marker_color_b').value,
        )

        self._explore_start_delay_s = float(
            self.get_parameter('explore_start_delay_s').value)
        # Measured from readiness, not construction: bringup is already
        # staggered per robot (ROBOT_STAGGER_S) and nav2 activation time
        # varies, so timing from __init__ would fold that jitter into the
        # experimental variable.
        self._ready_since = None
        self._start_release_logged = False

        self._information_map_topic = self.get_parameter('information_map_topic').value
        self._information_occ_threshold = int(self.get_parameter('information_occ_threshold').value)
        if not 0 < self._information_occ_threshold <= 100:
            raise ValueError('information_occ_threshold must be in 1..100')
        self._reservation_topic = self.get_parameter('reservation_topic').value
        self._reservation_radius_m = float(self.get_parameter('reservation_radius_m').value)
        self._min_gain_m2 = float(self.get_parameter('min_gain_m2').value)
        self._reservations = GoalReservations(float(self.get_parameter('reservation_ttl_s').value))
        if self._reservation_radius_m <= 0 or self._min_gain_m2 < 0 or self._reservations.ttl <= 0:
            raise ValueError('invalid reservation/gain parameters')
        if self._reservation_topic:
            self._reservation_pub = self.create_publisher(PoseArray, self._reservation_topic, 1)
            for peer, topic in enumerate(self.get_parameter('reservation_peer_topics').value):
                if topic:
                    self.create_subscription(PoseArray, topic,
                        lambda msg, peer=peer: self._on_reservation(peer, msg), qos_profile_sensor_data)
            self.create_timer(1.0, self._publish_reservation)
        self._latest_information_map = None
        self._latest_costmap = None
        self._goal_active = False
        self._goal_handle = None
        self._preempting = False
        self._pending_goal_xy = None
        self._invalid_goal_key = None
        self._invalid_goal_last_map = None
        self._invalid_goal_count = 0
        self._blacklisted_goals = []
        self._blacklist_expiry = {}
        self._goal_blacklist_duration_s = float(self.get_parameter('goal_blacklist_duration_s').value)
        if not math.isfinite(self._goal_blacklist_duration_s) or self._goal_blacklist_duration_s <= 0:
            raise ValueError('goal_blacklist_duration_s must be finite and positive')
        self._last_goal_direction = None  # (dx, dy) of the most recently sent goal
        # Consecutive preemptions since the last goal reached a terminal state.
        self._preempt_streak = 0
        # Where the robot was when the no-progress timer last fired, and how
        # many times it has fired without the robot moving away from there.
        self._last_stuck_xy = None
        self._wedge_cycles = 0
        # No-progress tracking for the active goal (see goal_stuck_timeout_s).
        self._progress_ref_xy = None      # last pose we counted as progress
        self._progress_ref_time = None    # when we counted it
        self._abandoning_stuck = False    # cancel already requested this goal

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._marker_publisher = self.create_publisher(
            MarkerArray, self.get_parameter('frontier_marker_topic').value, 1)

        self.create_subscription(
            OccupancyGrid, self._costmap_topic, self._on_costmap, 1)
        if self._information_map_topic:
            self.create_subscription(OccupancyGrid, self._information_map_topic,
                                     self._on_information_map, 1)
        self.create_timer(replan_period_s, self._tick)

        self.get_logger().info(
            f"lite_frontier_explorer: watching '{self._costmap_topic}', "
            f"assignment={'robot_rank' if self._rank_assignment else 'independent'}, "
            f"robot={self._robot_index + 1}/{self._team_size}, "
            f"reservations={bool(self._reservation_topic)}")

    def _publish_reservation(self):
        msg = PoseArray()
        msg.header.frame_id = self._global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._goal_active and not self._preempting and not self._abandoning_stuck and self._pending_goal_xy:
            pose = Pose()
            pose.position.x, pose.position.y = self._pending_goal_xy
            pose.orientation.w = 1.0
            msg.poses = [pose]
        self._reservation_pub.publish(msg)

    def _on_reservation(self, peer, msg):
        if msg.header.frame_id != self._global_frame or len(msg.poses) > 1:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        goal = (msg.poses[0].position.x, msg.poses[0].position.y) if msg.poses else None
        self._reservations.receive(peer, stamp, goal, self.get_clock().now().nanoseconds / 1e9)

    def _on_information_map(self, msg):
        self._latest_information_map = msg

    def _on_costmap(self, msg):
        self._latest_costmap = msg

    def _lookup_robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"TF lookup {self._global_frame} -> {self._robot_base_frame} "
                f"failed: {exc}", throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        return t.x, t.y

    def _start_allowed(self):
        # Called only after costmap, pose and configured occupancy input exist.
        # All mapping/communication stays live while navigation is held.
        if not self._nav_client.server_is_ready():
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        if self._ready_since is None:
            self._ready_since = now
            self.get_logger().info(
                f'Staggered start: ready at ROS time {now:.3f}; '
                f'release after {self._explore_start_delay_s:.1f}s.')
        if now - self._ready_since < self._explore_start_delay_s:
            return False
        if not self._start_release_logged:
            self.get_logger().info(f'Exploration released at ROS time {now:.3f}.')
            self._start_release_logged = True
        return True

    def _tick(self):
        costmap = self._latest_costmap
        if costmap is None:
            # A stuck nav2 lifecycle bringup (e.g. a change_state RPC
            # timeout during simultaneous multi-robot startup) can leave
            # global_costmap configured but never activated, so it never
            # publishes -- this node would otherwise sit completely
            # silent forever with nothing in its own log to say why.
            self.get_logger().warn(
                f"Still waiting for first costmap on '{self._costmap_topic}' -- "
                "nav2's global_costmap may not have activated (check its "
                "lifecycle_manager for a stuck/timed-out change_state call).",
                throttle_duration_sec=15.0)
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        information_grid = None
        information_threshold = 100  # Legacy standalone costmap-only mode.
        frontier_data = costmap.data
        frontier_threshold = self._occ_threshold
        if self._selection_strategy == 'visible_gain' and self._information_map_topic:
            if self._latest_information_map is None:
                self.get_logger().info('Waiting for occupancy map to score exploration gain.',
                                       throttle_duration_sec=10.0)
                return
            try:
                information_grid = align_information_grid(self._latest_information_map, costmap)
            except ValueError as exc:
                self.get_logger().warn(str(exc), throttle_duration_sec=10.0)
                return
            information_threshold = self._information_occ_threshold
            frontier_data = information_grid.ravel()
            frontier_threshold = information_threshold

        if self._explore_start_delay_s > 0.0 and not self._start_allowed():
            return

        clusters = find_frontier_clusters(
            frontier_data, costmap.info.width, costmap.info.height,
            occ_threshold=frontier_threshold, min_size=self._min_frontier_size,
        )

        # Classify every detected cluster so the markers can show *why* a
        # frontier that's visible on the map isn't being driven to --
        # otherwise "no frontiers beyond min_frontier_distance_m" looks like
        # a lie when RViz still shows plenty of green dots.
        cluster_xy = [
            cluster_centroid_world(
                cluster, costmap.info.resolution,
                costmap.info.origin.position.x, costmap.info.origin.position.y)
            for cluster in clusters
        ]
        cluster_status = []
        for xy in cluster_xy:
            if self._is_blacklisted(xy):
                cluster_status.append('blacklisted')
            elif math.hypot(xy[0] - robot_pose[0], xy[1] - robot_pose[1]) < self._min_frontier_distance_m:
                cluster_status.append('too_close')
            else:
                cluster_status.append('eligible')

        candidates = [
            cluster for cluster, status in zip(clusters, cluster_status)
            if status == 'eligible'
        ]
        if self._selection_strategy == 'visible_gain':
            # Eligibility must apply to the actual viewpoints, not the centroid
            # of a long/curved cluster that may sit in an obstacle or blacklist.
            candidates = []
            for index, cluster in enumerate(clusters):
                unblocked = [cell for cell in cluster if not self._is_blacklisted(
                    cluster_centroid_world([cell], costmap.info.resolution,
                                           costmap.info.origin.position.x,
                                           costmap.info.origin.position.y))]
                eligible = [cell for cell in unblocked if math.hypot(
                    costmap.info.origin.position.x + (cell[1] + 0.5) * costmap.info.resolution - robot_pose[0],
                    costmap.info.origin.position.y + (cell[0] + 0.5) * costmap.info.resolution - robot_pose[1])
                    >= self._min_frontier_distance_m]
                cluster_status[index] = ('eligible' if eligible else
                                         'too_close' if unblocked else 'blacklisted')
                if eligible:
                    candidates.append(eligible)
            if self._rank_assignment:
                # Rank the complete map-derived list first. Private blacklists
                # and distance gates must not renumber other robots' ranks.
                candidates = clusters

        reservations = (self._reservations.active(self.get_clock().now().nanoseconds / 1e9)
                        if self._reservation_topic else {})
        # Every new assignment respects every live claim. Simultaneous claims
        # are resolved below by static robot index, so exactly one yields.
        goal = None
        active_score = {}
        selected_utility = None
        if candidates or (self._selection_strategy == 'visible_gain' and self._goal_active):
            if self._selection_strategy == 'visible_gain':
                scores = []
                goal = select_visible_gain_frontier(
                    candidates, costmap.data, costmap.info.width, costmap.info.height,
                    robot_pose[0], robot_pose[1], costmap.info.resolution,
                    costmap.info.origin.position.x, costmap.info.origin.position.y,
                    path_occ_threshold=self._path_occ_threshold,
                    min_distance_m=self._min_frontier_distance_m,
                    sensor_range_m=self._sensor_range_m,
                    distance_weight=self._gain_distance_weight,
                    max_viewpoints=self._gain_max_viewpoints, diagnostics=scores,
                    active_goal=self._pending_goal_xy if self._goal_active else None,
                    active_score=active_score,
                    assignment_mode='robot_rank' if self._rank_assignment else 'independent',
                    robot_index=self._robot_index, team_size=self._team_size,
                    blacklisted_goals=self._blacklisted_goals,
                    blacklist_radius_m=self._goal_blacklist_radius_m,
                    information_grid=information_grid,
                    information_occ_threshold=information_threshold,
                    reserved_goals=list(reservations.values()),
                    reservation_radius_m=self._reservation_radius_m,
                    min_gain_m2=self._min_gain_m2,
                )
                selected_utility = next((s['utility'] for s in scores
                                         if (s['x'], s['y']) == goal), None)
                ranked = sorted(scores, key=lambda s: (-s['utility'], s['path_m']))[:3]
                self.get_logger().info(
                    'Frontier scores: ' + '; '.join(
                        f"({s['x']:.2f},{s['y']:.2f}) gain={s['gain_m2']:.2f}m2 "
                        f"path={s['path_m']:.2f}m utility={s['utility']:.3f}"
                        f" rank={s['cluster_rank'] + 1} fallback={s['assignment_fallback']}"
                        for s in ranked)
                    + f'; selected={goal}; active={self._pending_goal_xy if self._goal_active else None}'
                    + f'; active_score={active_score}',
                    throttle_duration_sec=9.0,
                )
            elif self._selection_strategy == 'nearest_high_gain':
                goal = select_nearest_high_gain_frontier(
                    candidates, costmap.data, costmap.info.width, costmap.info.height,
                    robot_pose[0], robot_pose[1],
                    costmap.info.resolution,
                    costmap.info.origin.position.x, costmap.info.origin.position.y,
                    path_occ_threshold=self._path_occ_threshold,
                    min_distance_m=self._min_frontier_distance_m,
                    gain_region_cap=self._gain_region_cap,
                    gain_threshold_ratio=self._gain_threshold_ratio,
                    turn_penalty_m=self._turn_penalty_m,
                    preferred_direction=self._last_goal_direction,
                    hysteresis_bonus_m=self._hysteresis_bonus_m,
                )
            elif self._selection_strategy == 'best_gain':
                goal = select_best_frontier(
                    candidates, costmap.data, costmap.info.width, costmap.info.height,
                    robot_pose[0], robot_pose[1],
                    costmap.info.resolution,
                    costmap.info.origin.position.x, costmap.info.origin.position.y,
                    path_occ_threshold=self._path_occ_threshold,
                    min_distance_m=self._min_frontier_distance_m,
                    sensor_range_m=self._sensor_range_m,
                    distance_weight=self._gain_distance_weight,
                )
            else:
                goal = select_nearest_frontier(
                    candidates, costmap.data, costmap.info.width, costmap.info.height,
                    robot_pose[0], robot_pose[1],
                    costmap.info.resolution,
                    costmap.info.origin.position.x, costmap.info.origin.position.y,
                    path_occ_threshold=self._path_occ_threshold,
                    min_distance_m=self._min_frontier_distance_m,
                )

        self._publish_frontier_markers(cluster_xy, cluster_status, goal)

        if self._goal_active:
            # Invalid active viewpoints previously returned {}, which made
            # the utility comparison fail forever. Require two distinct map
            # updates to reject a goal, so a transient costmap obstruction
            # does not cause churn. Missing scores are not invalidity evidence.
            invalid = active_score.get('status') in (
                'outside_map', 'blocked_or_unknown', 'unreachable')
            if not invalid or self._invalid_goal_key != self._pending_goal_xy:
                self._invalid_goal_count = 0
                self._invalid_goal_last_map = None
            self._invalid_goal_key = self._pending_goal_xy
            evidence = (costmap, self._latest_information_map
                        if information_grid is not None else None)
            fresh_evidence = (self._invalid_goal_last_map is None or any(
                old is not new for old, new in zip(self._invalid_goal_last_map, evidence)))
            if invalid and fresh_evidence:
                self._invalid_goal_count += 1
                self._invalid_goal_last_map = evidence
            if (invalid and self._invalid_goal_count >= 2 and goal is not None
                    and not self._preempting and not self._abandoning_stuck):
                self.get_logger().info(
                    f"Replacing invalid goal {self._pending_goal_xy}: "
                    f"{active_score['status']} on {self._invalid_goal_count} map updates; "
                    f"reachable unreserved alternative={goal}.")
                self._preempting = True
                self._preempt_streak = 0
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                return

            # A delivered lower-index claim wins a simultaneous conflict.
            # Use the same free-space distance as candidate exclusion.
            conflicting_peer = None
            if self._pending_goal_xy is not None:
                reservation_data = costmap.data
                if information_grid is not None:
                    reservation_data = occupancy_constrained_costmap(
                        np.asarray(costmap.data, dtype=np.int8).reshape(
                            costmap.info.height, costmap.info.width),
                        information_grid, information_threshold).ravel()
                row = math.floor((self._pending_goal_xy[1] - costmap.info.origin.position.y) / costmap.info.resolution)
                col = math.floor((self._pending_goal_xy[0] - costmap.info.origin.position.x) / costmap.info.resolution)
                for peer, (x, y) in reservations.items():
                    if peer >= self._robot_index:
                        continue
                    distances = _free_space_distances(reservation_data, costmap.info.width,
                        costmap.info.height, x, y, costmap.info.resolution,
                        costmap.info.origin.position.x, costmap.info.origin.position.y,
                        self._path_occ_threshold)
                    if (0 <= row < costmap.info.height and 0 <= col < costmap.info.width
                            and 0 <= distances[row, col] * costmap.info.resolution <= self._reservation_radius_m):
                        conflicting_peer = peer
                        break
            if conflicting_peer is not None and not self._preempting and not self._abandoning_stuck:
                self.get_logger().info(f'Yielding goal {self._pending_goal_xy} to robot{conflicting_peer + 1} reservation.')
                self._preempting = True
                self._preempt_streak = 0
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                return

            # Map delivery must affect work already in progress, including
            # robot-rank assignments. A valid viewpoint with no unknown area
            # left cannot gather information; don't finish a redundant trip.
            # Missing/unreachable scores are not proof of completion.
            if (not self._preempting and not self._abandoning_stuck
                    and self._selection_strategy == 'visible_gain'
                    and active_score.get('gain_m2', math.inf) <= self._min_gain_m2):
                self.get_logger().info(
                    f"Cancelling explored frontier {self._pending_goal_xy}: "
                    f"remaining gain {active_score['gain_m2']:.3f} m² is below the useful-work threshold.")
                self._preempting = True
                # Completion by observation isn't rank churn and shouldn't
                # consume the ordinary utility-switching budget.
                self._preempt_streak = 0
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                return

            # Abandon a goal the robot is not actually getting anywhere with,
            # rather than waiting out nav2's full recovery chain.
            if (self._goal_stuck_timeout_s > 0.0 and not self._preempting
                    and not self._abandoning_stuck):
                now = self.get_clock().now()
                if self._progress_ref_xy is None or self._progress_ref_time is None:
                    self._progress_ref_xy = (robot_pose[0], robot_pose[1])
                    self._progress_ref_time = now
                elif math.hypot(robot_pose[0] - self._progress_ref_xy[0],
                                robot_pose[1] - self._progress_ref_xy[1]) \
                        >= self._goal_stuck_epsilon_m:
                    # Moved far enough to count as headway -- reset the clock.
                    self._progress_ref_xy = (robot_pose[0], robot_pose[1])
                    self._progress_ref_time = now
                else:
                    stalled_s = (now - self._progress_ref_time).nanoseconds / 1e9
                    if stalled_s >= self._goal_stuck_timeout_s:
                        goal_desc = (
                            f"({self._pending_goal_xy[0]:.2f}, {self._pending_goal_xy[1]:.2f})"
                            if self._pending_goal_xy is not None else "(unknown)")

                        # Did the robot move at all since the LAST time this
                        # fired? If not it is wedged, and the frontier is
                        # innocent -- see wedge_detect_radius_m.
                        wedged = (
                            self._last_stuck_xy is not None
                            and math.hypot(
                                robot_pose[0] - self._last_stuck_xy[0],
                                robot_pose[1] - self._last_stuck_xy[1])
                            < self._wedge_detect_radius_m)
                        self._last_stuck_xy = (robot_pose[0], robot_pose[1])

                        if wedged:
                            self._wedge_cycles += 1
                            if self._wedge_cycles <= self._max_wedge_cycles:
                                # Leave the goal in flight deliberately: nav2's
                                # recovery behaviours are what free a jammed
                                # footprint, and cancelling here is precisely
                                # what has been stopping them finishing.
                                self.get_logger().warn(
                                    f"Robot has not moved from ({robot_pose[0]:.2f}, "
                                    f"{robot_pose[1]:.2f}) since the last stall -- WEDGED, "
                                    f"not a bad frontier. Holding goal {goal_desc} so nav2 "
                                    f"recovery can run (cycle {self._wedge_cycles}/"
                                    f"{self._max_wedge_cycles}).")
                                self._progress_ref_xy = (robot_pose[0], robot_pose[1])
                                self._progress_ref_time = now
                                return
                            # Recovery has had its chances. Switch goals for a
                            # different approach angle, but still do NOT blame
                            # the frontier.
                            self.get_logger().warn(
                                f"Still wedged at ({robot_pose[0]:.2f}, {robot_pose[1]:.2f}) "
                                f"after {self._wedge_cycles} recovery cycles -- switching "
                                f"goals for a new approach angle. {goal_desc} is NOT "
                                f"blacklisted; the robot is what is stuck.")
                            self._wedge_cycles = 0
                            self._preempting = True   # cancel WITHOUT blacklisting
                            if self._goal_handle is not None:
                                self._goal_handle.cancel_goal_async()
                            return

                        self._wedge_cycles = 0
                        self.get_logger().warn(
                            f"No progress toward goal {goal_desc} for {stalled_s:.1f}s "
                            f"(moved < {self._goal_stuck_epsilon_m}m) -- abandoning and "
                            "blacklisting it instead of waiting out nav2 recovery.")
                        # _preempting stays False so _on_result() blacklists it.
                        self._abandoning_stuck = True
                        if self._goal_handle is not None:
                            self._goal_handle.cancel_goal_async()
                        return

            # Replanning may have found a materially better/closer frontier
            # than the one currently being driven to -- preempt the stale
            # goal instead of riding it out to success/abort. Without this,
            # the markers show the newly identified frontier while the
            # robot keeps executing whatever goal was in flight when it was
            # found, which looks like it's ignoring the frontier entirely.
            committed = (self._selection_strategy != 'visible_gain'
                         and self._max_consecutive_preemptions > 0
                         and self._preempt_streak >= self._max_consecutive_preemptions)
            if (not self._preempting and goal is not None
                    and self._pending_goal_xy is not None
                    # Rank membership alone cannot justify switching, but a
                    # materially better travel-adjusted utility can. The same
                    # ratio applies to every assignment mode. A lifetime
                    # switching budget must not override new map evidence.
                    and not committed
                    and math.hypot(goal[0] - self._pending_goal_xy[0],
                                    goal[1] - self._pending_goal_xy[1])
                    >= self._goal_preempt_distance_m):
                # Legacy strategies compare distance; visible_gain compares
                # utility at both destinations on this same current map.
                current_dist = math.hypot(
                    self._pending_goal_xy[0] - robot_pose[0],
                    self._pending_goal_xy[1] - robot_pose[1])
                new_dist = math.hypot(goal[0] - robot_pose[0],
                                      goal[1] - robot_pose[1])
                if self._selection_strategy == 'visible_gain':
                    improved = (selected_utility is not None and 'utility' in active_score
                                and selected_utility > self._goal_preempt_utility_ratio * active_score['utility'])
                    reason = f'higher-utility frontier (utility={selected_utility})'
                else:
                    improved = current_dist - new_dist >= self._goal_preempt_improvement_m
                    reason = 'closer frontier'
                if improved:
                    self._preempt_streak += 1
                    self.get_logger().info(
                        f"Preempting in-flight goal ({self._pending_goal_xy[0]:.2f}, "
                        f"{self._pending_goal_xy[1]:.2f}) at {current_dist:.1f}m for "
                        f"{reason} ({goal[0]:.2f}, {goal[1]:.2f}) at "
                        f"{new_dist:.1f}m [streak {self._preempt_streak}].")
                    self._preempting = True
                    if self._goal_handle is not None:
                        self._goal_handle.cancel_goal_async()
            elif (committed and not self._preempting
                    and self._pending_goal_xy is not None):
                # The not-None guard is repeated deliberately: it lives in the
                # `if` above, so without it here this branch would dereference
                # a None goal and kill the explorer outright.
                self.get_logger().info(
                    f"Committed to goal ({self._pending_goal_xy[0]:.2f}, "
                    f"{self._pending_goal_xy[1]:.2f}) after "
                    f"{self._preempt_streak} preemptions -- riding it out so it "
                    "can succeed, fail, or trip the stuck timer.",
                    throttle_duration_sec=15.0)
            return  # still navigating -- _on_result() clears this when nav2 is done

        if not clusters:
            self.get_logger().info(
                "No frontiers left -- exploration complete.",
                throttle_duration_sec=10.0)
            return

        if goal is None:
            self.get_logger().info(
                "No usable unreserved frontier above the gain/distance limits -- waiting.",
                throttle_duration_sec=10.0)
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().info(
                "navigate_to_pose action server not available yet, waiting...",
                throttle_duration_sec=5.0)
            return

        goal_x, goal_y = goal
        self._last_goal_direction = (goal_x - robot_pose[0], goal_y - robot_pose[1])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self._global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        yaw = math.atan2(goal_y - robot_pose[1], goal_x - robot_pose[0])
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f"Sending goal to frontier at ({goal_x:.2f}, {goal_y:.2f})")
        self._goal_active = True
        self._goal_handle = None
        self._preempting = False
        self._pending_goal_xy = (goal_x, goal_y)
        # Fresh goal -- restart no-progress tracking.
        self._progress_ref_xy = None
        self._progress_ref_time = None
        self._abandoning_stuck = False
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _is_blacklisted(self, xy):
        now = self.get_clock().now().nanoseconds / 1e9
        self._blacklist_expiry = {goal: until for goal, until in self._blacklist_expiry.items()
                                  if now < until}
        self._blacklisted_goals = list(self._blacklist_expiry)
        x, y = xy
        return any(
            math.hypot(x - bx, y - by) <= self._goal_blacklist_radius_m
            for bx, by in self._blacklisted_goals
        )

    # Eligible reuses the configured marker color; the other two states get
    # fixed colors so they read the same regardless of what marker_color is
    # tuned to.
    _STATUS_COLORS = {
        'too_close': (1.0, 0.75, 0.05),   # amber -- inside min_frontier_distance_m
        'blacklisted': (0.55, 0.55, 0.55),  # grey -- inside a failed-goal exclusion zone
    }

    def _publish_frontier_markers(self, cluster_xy, cluster_status, goal):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        delete_all = Marker()
        delete_all.header.frame_id = self._global_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        eligible_color = self._marker_color
        for idx, ((x, y), status) in enumerate(zip(cluster_xy, cluster_status)):
            color_r, color_g, color_b = self._STATUS_COLORS.get(status, eligible_color)
            marker = Marker()
            marker.header.frame_id = self._global_frame
            marker.header.stamp = stamp
            marker.ns = "frontier_candidates"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = self._marker_scale
            marker.color.r = color_r
            marker.color.g = color_g
            marker.color.b = color_b
            marker.color.a = 0.4 if status == 'blacklisted' else 0.8
            marker_array.markers.append(marker)

        # Draw each blacklisted goal's actual exclusion circle so it's clear
        # *why* nearby frontiers are greyed out, not just that they are.
        for idx, (bx, by) in enumerate(self._blacklisted_goals):
            zone = Marker()
            zone.header.frame_id = self._global_frame
            zone.header.stamp = stamp
            zone.ns = "blacklist_zones"
            zone.id = idx
            zone.type = Marker.CYLINDER
            zone.action = Marker.ADD
            zone.pose.position.x = bx
            zone.pose.position.y = by
            zone.pose.position.z = 0.02
            zone.pose.orientation.w = 1.0
            diameter = self._goal_blacklist_radius_m * 2.0
            zone.scale.x = zone.scale.y = diameter
            zone.scale.z = 0.01
            zone.color.r = 0.8
            zone.color.g = 0.1
            zone.color.b = 0.1
            zone.color.a = 0.15
            marker_array.markers.append(zone)

        if goal is not None:
            goal_x, goal_y = goal
            selected = Marker()
            selected.header.frame_id = self._global_frame
            selected.header.stamp = stamp
            selected.ns = "selected_frontier"
            selected.id = 0
            selected.type = Marker.SPHERE
            selected.action = Marker.ADD
            selected.pose.position.x = goal_x
            selected.pose.position.y = goal_y
            selected.pose.position.z = 0.15
            selected.pose.orientation.w = 1.0
            selected.scale.x = selected.scale.y = selected.scale.z = self._marker_scale * 1.8
            selected.color.r = 1.0
            selected.color.g = 0.1
            selected.color.b = 0.1
            selected.color.a = 1.0
            marker_array.markers.append(selected)

        self._marker_publisher.publish(marker_array)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by nav2.")
            # Action-server rejection is not evidence of an unreachable target.
            self._pending_goal_xy = None
            self._goal_handle = None
            self._preempt_streak = 0
            self._goal_active = False
            self._preempting = False
            self._abandoning_stuck = False
            return
        self._goal_handle = goal_handle
        # A preemption (or stuck-abandon) request may have arrived between
        # send and accept; honor it now that we finally have a handle to
        # cancel. Without the _abandoning_stuck arm here, a goal that stalled
        # before nav2 accepted it would never be cancelled and _goal_active
        # would stay set forever, wedging the explorer itself.
        if self._preempting or self._abandoning_stuck:
            goal_handle.cancel_goal_async()
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._pending_goal_xy = None
            self._preempt_streak = 0
        elif status == GoalStatus.STATUS_CANCELED and not self._abandoning_stuck:
            # Cancellation (including external cancellation) is not evidence
            # that the frontier is unreachable.
            # _preempt_streak is deliberately NOT reset here: this is the
            # outcome it exists to count, and clearing it would let the
            # explorer preempt forever without ever reaching the backstop.
            self._pending_goal_xy = None
        else:
            self.get_logger().warn(
                f"Goal did not succeed (status={status}) -- excluding it for "
                f"{self._goal_blacklist_duration_s:g}s before retrying.")
            self._blacklist_pending_goal()
            self._preempt_streak = 0
        self._goal_active = False
        self._goal_handle = None
        self._preempting = False
        self._abandoning_stuck = False
        self._progress_ref_xy = None
        self._progress_ref_time = None

    def _blacklist_pending_goal(self):
        if self._pending_goal_xy is not None:
            self._blacklist_expiry[self._pending_goal_xy] = (
                self.get_clock().now().nanoseconds / 1e9 + self._goal_blacklist_duration_s)
            self._blacklisted_goals = list(self._blacklist_expiry)
            self._pending_goal_xy = None


def main():
    rclpy.init()
    node = LiteFrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
