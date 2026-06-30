#!/usr/bin/env python3
"""
Subscribes to /{namespace}/tf and /{namespace}/tf_static, prefixes any bare
(non-namespaced, non-global) frame IDs with the robot namespace, then
republishes to /tf and /tf_static.

Usage:
  ros2 run bme_ros2_navigation_py tf_frame_renamer --ros-args \
      -p namespace:=robot1
"""

import copy
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

# Frames that belong to the shared global coordinate system and should
# never be prefixed with the robot namespace.
_GLOBAL_FRAMES = {"map", "world", "earth"}


class TFFrameRenamer(Node):
    def __init__(self):
        super().__init__("tf_frame_renamer")
        self.declare_parameter("namespace", "")
        ns = self.get_parameter("namespace").get_parameter_value().string_value
        if not ns:
            self.get_logger().fatal("Parameter 'namespace' must be non-empty")
            sys.exit(1)

        self._prefix = ns + "/"

        latching = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort = QoSProfile(depth=100)

        self._pub_tf = self.create_publisher(TFMessage, "/tf", best_effort)
        self._pub_static = self.create_publisher(TFMessage, "/tf_static", latching)

        self.create_subscription(
            TFMessage, f"/{ns}/tf", self._cb_tf, best_effort
        )
        self.create_subscription(
            TFMessage, f"/{ns}/tf_static", self._cb_static, latching
        )

        self.get_logger().info(
            f"tf_frame_renamer: relaying /{ns}/tf[_static] → /tf[_static], "
            f"prefixing bare frames with '{self._prefix}'"
        )

    def _rename_frame(self, frame: str) -> str:
        if frame in _GLOBAL_FRAMES or frame.startswith(self._prefix):
            return frame
        return self._prefix + frame

    def _rename_msg(self, msg: TFMessage) -> TFMessage:
        out = TFMessage()
        for t in msg.transforms:
            new_t = copy.deepcopy(t)
            new_t.header.frame_id = self._rename_frame(t.header.frame_id)
            new_t.child_frame_id = self._rename_frame(t.child_frame_id)
            out.transforms.append(new_t)
        return out

    def _cb_tf(self, msg: TFMessage) -> None:
        self._pub_tf.publish(self._rename_msg(msg))

    def _cb_static(self, msg: TFMessage) -> None:
        self._pub_static.publish(self._rename_msg(msg))


def main():
    rclpy.init()
    node = TFFrameRenamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
