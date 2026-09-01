"""Minimal frontier explorer: watches nav2's global costmap, picks the
nearest frontier cluster, and hands it to nav2 via a plain NavigateToPose
action client. No peer coordination, no map fusion of its own -- the
costmap it reads already reflects whatever teammates' data has been
relayed in by the active map_transport (baseline/vxch/zstd) and fused into
nav_map upstream.
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from lite_frontier_explorer.frontier_detection import (
    cluster_centroid_world,
    find_frontier_clusters,
    select_best_frontier,
    select_nearest_frontier,
    select_nearest_high_gain_frontier,
)


class LiteFrontierExplorer(Node):
    def __init__(self):
        super().__init__('lite_frontier_explorer')

        self.declare_parameter('costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('min_frontier_size_cells', 10)
        self.declare_parameter('min_frontier_distance_m', 0.5)
        self.declare_parameter('goal_blacklist_radius_m', 1.0)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('path_occ_threshold', 99)
        # 'nearest', 'best_gain' (global argmax gain/cost -- tends to send every
        # robot at the same richest frontier), or 'nearest_high_gain' (nearest
        # frontier whose gain clears gain_threshold_ratio * best gain seen).
        self.declare_parameter('selection_strategy', 'nearest_high_gain')
        self.declare_parameter('sensor_range_m', 3.0)  # used by 'best_gain' only
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
        # zig-zag route over a straight one of the same step count.
        self.declare_parameter('turn_penalty_m', 0.3)
        # Bonus/penalty (scaled by directional alignment) applied to a
        # candidate relative to the direction of the last goal sent, so the
        # robot favors continuing the way it was already heading over
        # backtracking for a marginally-better frontier.
        self.declare_parameter('hysteresis_bonus_m', 1.5)
        self.declare_parameter('replan_period_s', 3.0)
        self.declare_parameter('navigate_to_pose_action_name', 'navigate_to_pose')
        self.declare_parameter('frontier_marker_topic', 'explore/frontiers')
        self.declare_parameter('frontier_marker_scale', 0.15)
        self.declare_parameter('frontier_marker_color_r', 0.15)
        self.declare_parameter('frontier_marker_color_g', 0.9)
        self.declare_parameter('frontier_marker_color_b', 0.2)

        self._costmap_topic = self.get_parameter('costmap_topic').value
        self._global_frame = self.get_parameter('global_frame').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._min_frontier_size = self.get_parameter('min_frontier_size_cells').value
        self._min_frontier_distance_m = self.get_parameter('min_frontier_distance_m').value
        self._goal_blacklist_radius_m = self.get_parameter('goal_blacklist_radius_m').value
        self._occ_threshold = self.get_parameter('occ_threshold').value
        self._path_occ_threshold = self.get_parameter('path_occ_threshold').value
        self._selection_strategy = self.get_parameter('selection_strategy').value
        self._sensor_range_m = self.get_parameter('sensor_range_m').value
        self._gain_distance_weight = self.get_parameter('gain_distance_weight').value
        self._gain_threshold_ratio = self.get_parameter('gain_threshold_ratio').value
        self._gain_region_cap = self.get_parameter('gain_region_cap').value
        self._turn_penalty_m = self.get_parameter('turn_penalty_m').value
        self._hysteresis_bonus_m = self.get_parameter('hysteresis_bonus_m').value
        replan_period_s = self.get_parameter('replan_period_s').value
        action_name = self.get_parameter('navigate_to_pose_action_name').value
        self._marker_scale = self.get_parameter('frontier_marker_scale').value
        self._marker_color = (
            self.get_parameter('frontier_marker_color_r').value,
            self.get_parameter('frontier_marker_color_g').value,
            self.get_parameter('frontier_marker_color_b').value,
        )

        self._latest_costmap = None
        self._goal_active = False
        self._pending_goal_xy = None
        self._blacklisted_goals = []
        self._last_goal_direction = None  # (dx, dy) of the most recently sent goal

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._marker_publisher = self.create_publisher(
            MarkerArray, self.get_parameter('frontier_marker_topic').value, 1)

        self.create_subscription(
            OccupancyGrid, self._costmap_topic, self._on_costmap, 1)
        self.create_timer(replan_period_s, self._tick)

        self.get_logger().info(
            f"lite_frontier_explorer: watching '{self._costmap_topic}'")

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

        clusters = find_frontier_clusters(
            costmap.data, costmap.info.width, costmap.info.height,
            occ_threshold=self._occ_threshold, min_size=self._min_frontier_size,
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

        goal = None
        if candidates:
            if self._selection_strategy == 'nearest_high_gain':
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
            return  # still navigating -- _on_result() clears this when nav2 is done

        if not clusters:
            self.get_logger().info(
                "No frontiers left -- exploration complete.",
                throttle_duration_sec=10.0)
            return

        if goal is None:
            self.get_logger().info(
                "No frontiers beyond min_frontier_distance_m -- waiting.",
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
        self._pending_goal_xy = (goal_x, goal_y)
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _is_blacklisted(self, xy):
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
            self._blacklist_pending_goal()
            self._goal_active = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._pending_goal_xy = None
        else:
            self.get_logger().warn(
                f"Goal did not succeed (status={status}) -- blacklisting it.")
            self._blacklist_pending_goal()
        self._goal_active = False

    def _blacklist_pending_goal(self):
        if self._pending_goal_xy is not None:
            self._blacklisted_goals.append(self._pending_goal_xy)
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
