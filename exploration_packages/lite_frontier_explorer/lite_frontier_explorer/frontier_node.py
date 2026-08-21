"""Minimal frontier explorer: watches nav2's global costmap, picks the
nearest frontier cluster, and hands it to nav2 via a plain NavigateToPose
action client. No peer coordination, no map fusion of its own -- the
costmap it reads already reflects whatever teammates' data has been
relayed in by the active map_transport (baseline/vxch/zstd) and fused into
nav_map upstream.
"""

import math

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from lite_frontier_explorer.frontier_detection import (
    find_frontier_clusters,
    select_nearest_frontier,
)


class LiteFrontierExplorer(Node):
    def __init__(self):
        super().__init__('lite_frontier_explorer')

        self.declare_parameter('costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('min_frontier_size_cells', 6)
        self.declare_parameter('min_frontier_distance_m', 2.0)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('replan_period_s', 3.0)
        self.declare_parameter('navigate_to_pose_action_name', 'navigate_to_pose')

        self._costmap_topic = self.get_parameter('costmap_topic').value
        self._global_frame = self.get_parameter('global_frame').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._min_frontier_size = self.get_parameter('min_frontier_size_cells').value
        self._min_frontier_distance_m = self.get_parameter('min_frontier_distance_m').value
        self._occ_threshold = self.get_parameter('occ_threshold').value
        replan_period_s = self.get_parameter('replan_period_s').value
        action_name = self.get_parameter('navigate_to_pose_action_name').value

        self._latest_costmap = None
        self._goal_active = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, action_name)

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
        if self._goal_active:
            return  # still navigating -- _on_result() clears this when nav2 is done

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

        if not self._nav_client.server_is_ready():
            self.get_logger().info(
                "navigate_to_pose action server not available yet, waiting...",
                throttle_duration_sec=5.0)
            return

        goal = select_nearest_frontier(
            clusters, robot_pose[0], robot_pose[1],
            costmap.info.resolution,
            costmap.info.origin.position.x, costmap.info.origin.position.y,
            min_distance_m=self._min_frontier_distance_m,
        )
        if goal is None:
            self.get_logger().info(
                "No frontiers beyond min_frontier_distance_m -- waiting.",
                throttle_duration_sec=10.0)
            return
        goal_x, goal_y = goal

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
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by nav2.")
            self._goal_active = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _future):
        # Don't inspect the result -- succeeded, aborted, or canceled all
        # just mean "free to pick a new frontier next tick."
        self._goal_active = False


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
