#!/usr/bin/env python3
"""
Republishes this robot's own /{robot}/odom at a fixed, low rate, for
ddil_proxy_node to relay to peers as /{peer}/pose_broadcast.

Peers only use this for a coarse collision-avoidance keepout zone (see
peer_keepout_mask.py) -- relaying raw odom at its native ~20-50Hz rate over
real WiFi would waste bandwidth these consumers never need.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__("pose_broadcaster")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("publish_period_s", 1.0)

        robot_name = self.get_parameter("robot_name").value
        if not robot_name:
            raise RuntimeError("robot_name parameter is required")
        publish_period_s = max(0.1, float(self.get_parameter("publish_period_s").value))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._latest_odom = None
        self.create_subscription(Odometry, f"/{robot_name}/odom", self._on_odom, 10)
        self._pub = self.create_publisher(Odometry, f"/{robot_name}/pose_broadcast", qos)
        self.create_timer(publish_period_s, self._publish)

        self.get_logger().info(
            f"pose_broadcaster for {robot_name}: publishing every {publish_period_s}s")

    def _on_odom(self, msg):
        self._latest_odom = msg

    def _publish(self):
        if self._latest_odom is not None:
            self._pub.publish(self._latest_odom)


def main():
    rclpy.init()
    node = PoseBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
