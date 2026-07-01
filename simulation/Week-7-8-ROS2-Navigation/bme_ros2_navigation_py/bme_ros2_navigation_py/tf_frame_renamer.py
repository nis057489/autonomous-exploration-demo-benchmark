#!/usr/bin/env python3
"""
Bridges /tf (absolute, published by C++ TransformBroadcaster) to /{ns}/tf
(relative, subscribed by Nav2's TransformListener under ROS_NAMESPACE).

In ROS2 Jazzy, StaticTransformBroadcaster uses relative tf_static topics and
connects to Nav2 directly. Only dynamic TF needs bridging.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFBridge(Node):
    def __init__(self):
        super().__init__("tf_frame_renamer")
        self.declare_parameter("namespace", "")
        ns = self.get_parameter("namespace").get_parameter_value().string_value
        if not ns:
            self.get_logger().fatal("Parameter 'namespace' must be non-empty")
            sys.exit(1)

        reliable = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        best_effort = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(TFMessage, f"/{ns}/tf", reliable)
        self.create_subscription(TFMessage, "/tf", self._cb, best_effort)

        self.get_logger().info(f"tf bridge: /tf → /{ns}/tf")

    def _cb(self, msg: TFMessage) -> None:
        # Drop messages more than 1s old (avoids replaying DDS queue backlog).
        now = self.get_clock().now().nanoseconds
        for t in msg.transforms:
            stamp_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
            if stamp_ns > 0 and (now - stamp_ns) > 1_000_000_000:
                return
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = TFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
