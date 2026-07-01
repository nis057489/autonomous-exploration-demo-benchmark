#!/usr/bin/env python3
"""
Subscribes to /{namespace}/tf and /{namespace}/tf_static, prefixes any bare
(non-namespaced, non-global) frame IDs with the robot namespace, then
republishes to /tf, /{namespace}/tf, and /tf_static.
"""

import collections
import copy
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

_GLOBAL_FRAMES = {"map", "world", "earth"}
_SIG_CACHE_SIZE = 500


class TFFrameRenamer(Node):
    def __init__(self):
        super().__init__("tf_frame_renamer")
        self.declare_parameter("namespace", "")
        ns = self.get_parameter("namespace").get_parameter_value().string_value
        if not ns:
            self.get_logger().fatal("Parameter 'namespace' must be non-empty")
            sys.exit(1)

        self._prefix = ns + "/"
        self._seen_sigs: collections.OrderedDict = collections.OrderedDict()
        self._seen_static: set = set()
        self._tf_rx_count = 0
        self._static_rx_count = 0

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

        self._pub_tf = self.create_publisher(TFMessage, "/tf", reliable)
        self._pub_tf_ns = self.create_publisher(TFMessage, f"/{ns}/tf", reliable)
        self._pub_static = self.create_publisher(TFMessage, "/tf_static", latching)

        # C++ TransformBroadcaster always publishes to absolute /tf and /tf_static,
        # regardless of ROS_NAMESPACE. Subscribe globally and filter by frame name.
        self.create_subscription(TFMessage, "/tf", self._cb_tf, best_effort)
        self.create_subscription(TFMessage, "/tf_static", self._cb_static, latching)

        self.get_logger().info(
            f"tf_frame_renamer: subscribing to /tf[_static] (global), "
            f"publishing to /tf, /{ns}/tf, /tf_static. prefix='{self._prefix}'"
        )

    def _rename_frame(self, frame: str) -> str:
        # Skip global frames and any frame that is already namespaced (contains /)
        if frame in _GLOBAL_FRAMES or "/" in frame:
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
        return tuple(
            (t.header.stamp.sec, t.header.stamp.nanosec, t.child_frame_id)
            for t in msg.transforms
        )

    def _cb_tf(self, msg: TFMessage) -> None:
        renamed = self._rename_msg(msg)
        sig = self._msg_sig(renamed)

        if sig in self._seen_sigs:
            return

        while len(self._seen_sigs) >= _SIG_CACHE_SIZE:
            self._seen_sigs.popitem(last=False)
        self._seen_sigs[sig] = True

        self._tf_rx_count += 1
        if self._tf_rx_count <= 3 or self._tf_rx_count % 200 == 0:
            frames = [(t.header.frame_id, t.child_frame_id) for t in renamed.transforms]
            self.get_logger().info(f"[TF relay #{self._tf_rx_count}] {frames}")

        self._pub_tf.publish(renamed)
        self._pub_tf_ns.publish(renamed)

    def _cb_static(self, msg: TFMessage) -> None:
        renamed = self._rename_msg(msg)
        # Content-based dedup: static frames don't move, so identical (parent, child)
        # pairs mean we've already relayed this transform. Prevents TRANSIENT_LOCAL loop.
        sig = frozenset(
            (t.header.frame_id, t.child_frame_id) for t in renamed.transforms
        )
        if sig in self._seen_static:
            return
        self._seen_static.add(sig)
        self._static_rx_count += 1
        frames = [(t.header.frame_id, t.child_frame_id) for t in renamed.transforms]
        self.get_logger().info(f"[TF_STATIC relay #{self._static_rx_count}] {frames}")
        self._pub_static.publish(renamed)


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
