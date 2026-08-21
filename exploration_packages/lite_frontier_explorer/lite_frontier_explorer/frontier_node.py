"""Minimal frontier explorer: watches nav2's global costmap, picks the
nearest frontier cluster, and hands it to nav2 via BasicNavigator. No peer
coordination, no map fusion of its own -- the costmap it reads already
reflects whatever teammates' data has been relayed in by the active
map_transport (baseline/vxch/zstd) and fused into nav_map upstream.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from lite_frontier_explorer.frontier_detection import (
    find_frontier_clusters,
    select_nearest_frontier,
)


class LiteFrontierExplorer(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='lite_frontier_explorer')

        self.declare_parameter('costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('min_frontier_size_cells', 6)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('replan_period_s', 3.0)

        self._costmap_topic = self.get_parameter('costmap_topic').value
        self._global_frame = self.get_parameter('global_frame').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._min_frontier_size = self.get_parameter('min_frontier_size_cells').value
        self._occ_threshold = self.get_parameter('occ_threshold').value
        replan_period_s = self.get_parameter('replan_period_s').value

        self._latest_costmap = None
        self._goal_active = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, self._costmap_topic, self._on_costmap, 1)
        self.create_timer(replan_period_s, self._tick)

        # Deliberately not calling waitUntilNav2Active(): its default
        # localizer='amcl' blocks forever on amcl/get_state, which never
        # exists in this stack (slam_toolbox does localization, not AMCL).
        # goToPose() below already waits for the NavigateToPose action
        # server itself before sending a goal, which is all we need.
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
        if self._goal_active and not self.isTaskComplete():
            return
        self._goal_active = False

        costmap = self._latest_costmap
        if costmap is None:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        clusters = find_frontier_clusters(
            costmap.data, costmap.info.width, costmap.info.height,
            occ_threshold=self._occ_threshold, min_size=self._min_frontier_size,
        )
        if not clusters:
            self.get_logger().info(
                "No frontiers left -- exploration complete.",
                throttle_duration_sec=10.0)
            return

        goal_x, goal_y = select_nearest_frontier(
            clusters, robot_pose[0], robot_pose[1],
            costmap.info.resolution,
            costmap.info.origin.position.x, costmap.info.origin.position.y,
        )

        goal = PoseStamped()
        goal.header.frame_id = self._global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        yaw = math.atan2(goal_y - robot_pose[1], goal_x - robot_pose[0])
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f"Sending goal to frontier at ({goal_x:.2f}, {goal_y:.2f})")
        self.goToPose(goal)
        self._goal_active = True


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
