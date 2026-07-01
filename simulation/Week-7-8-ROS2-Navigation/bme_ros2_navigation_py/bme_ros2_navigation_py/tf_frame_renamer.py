#!/usr/bin/env python3
"""
Subscribes to /{namespace}/tf and /{namespace}/tf_static, prefixes any bare
(non-namespaced, non-global) frame IDs with the robot namespace, then
republishes to /tf, /{namespace}/tf, and /tf_static.

Two publish targets for dynamic TF:
  /tf            — global, for Python tf2_ros listeners (RViz, frontier_path_tracker)
  /{ns}/tf       — namespaced, for C++ tf2_ros listeners (Nav2 costmap etc.)

Loop guard: we track the renamed-message signature (stamp+child_frame_id) of each
message we publish. When we receive our own republish on /{ns}/tf the signature
matches the cache → skip.
"""

import collections
import copy
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

_GLOBAL_FRAMES = {"map", "world", "earth"}

_SIG_CACHE_SIZE = 500  # stamps to remember for loop detection


class TFFrameRenamer(Node):
    def __init__(self):
        super().__init__("tf_frame_renamer")
        self.declare_parameter("namespace", "")
        ns = self.get_parameter("namespace").get_parameter_value().string_value
        if not ns:
            self.get_logger().fatal("Parameter 'namespace' must be non-empty")
            sys.exit(1)

        self._prefix = ns + "/"
        # OrderedDict keeps insertion order so we can evict oldest entries cheaply.
        self._seen_sigs: collections.OrderedDict = collections.OrderedDict()

        reliable = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        latching = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Dynamic TF publishers
        self._pub_tf = self.create_publisher(TFMessage, "/tf", reliable)
        self._pub_tf_ns = self.create_publisher(TFMessage, f"/{ns}/tf", reliable)
        # Static TF publisher
        self._pub_static = self.create_publisher(TFMessage, "/tf_static", latching)

        # Subscribe to /{ns}/tf with BEST_EFFORT to receive from any publisher reliability.
        self.create_subscription(TFMessage, f"/{ns}/tf", self._cb_tf, best_effort)
        self.create_subscription(TFMessage, f"/{ns}/tf_static", self._cb_static, latching)

        self.get_logger().info(
            f"tf_frame_renamer: relaying /{ns}/tf[_static] → /tf[_static] and /{ns}/tf, "
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

    def _msg_sig(self, msg: TFMessage):
        """Fingerprint based on (stamp, child_frame_id) of each transform."""
        return tuple(
            (t.header.stamp.sec, t.header.stamp.nanosec, t.child_frame_id)
            for t in msg.transforms
        )

    def _cb_tf(self, msg: TFMessage) -> None:
        renamed = self._rename_msg(msg)
        sig = self._msg_sig(renamed)

        # Skip messages we already published (our own /{ns}/tf republish looping back).
        if sig in self._seen_sigs:
            return

        # Evict oldest entries to keep cache bounded.
        while len(self._seen_sigs) >= _SIG_CACHE_SIZE:
            self._seen_sigs.popitem(last=False)
        self._seen_sigs[sig] = True

        self._pub_tf.publish(renamed)
        self._pub_tf_ns.publish(renamed)

    def _cb_static(self, msg: TFMessage) -> None:
        # Relay /{ns}/tf_static → /tf_static. No loop risk (different topics).
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
