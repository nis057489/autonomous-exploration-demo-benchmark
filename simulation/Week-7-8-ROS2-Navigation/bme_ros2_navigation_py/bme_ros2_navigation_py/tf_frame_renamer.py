#!/usr/bin/env python3
"""
Bridges /tf and /tf_static (absolute, published by C++ TransformBroadcaster /
StaticTransformBroadcaster regardless of node namespace) to /{ns}/tf and
/{ns}/tf_static (relative, subscribed by Nav2's C++ TransformListener).

Also renames any still-bare frame IDs (odom, base_footprint, base_link,
base_scan) to {namespace}/frame, as a safety net for sources that don't
apply the namespace prefix themselves. Already-namespaced frames (containing
'/') pass through unchanged.

Publishes renamed frames to BOTH:
  - /{ns}/tf(_static)  — Nav2 C++ TransformListener
  - /tf(_static)        — RViz, Python tf2_ros.TransformListener

Subscribes BEST_EFFORT to /tf (dynamic transforms may be published best-effort)
and RELIABLE/TRANSIENT_LOCAL to /tf_static (matching StaticTransformBroadcaster).
"""

import copy
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

_BARE_FRAMES = {"odom", "base_footprint", "base_link", "base_scan"}


class TFFrameRenamer(Node):
    def __init__(self):
        super().__init__("tf_frame_renamer")
        self.declare_parameter("namespace", "")
        self._ns = self.get_parameter("namespace").get_parameter_value().string_value
        if not self._ns:
            self.get_logger().fatal("Parameter 'namespace' must be non-empty")
            sys.exit(1)

        reliable_volatile = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        best_effort_volatile = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        reliable_transient_local = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # /{ns}/tf — Nav2 C++ TransformListener (subscribes to relative "tf")
        self._pub_ns = self.create_publisher(TFMessage, f"/{self._ns}/tf", reliable_volatile)
        # /tf — RViz and Python tf2_ros.TransformListener (RELIABLE)
        self._pub_global = self.create_publisher(TFMessage, "/tf", reliable_volatile)

        self._pub_ns_static = self.create_publisher(
            TFMessage, f"/{self._ns}/tf_static", reliable_transient_local
        )
        self._pub_global_static = self.create_publisher(
            TFMessage, "/tf_static", reliable_transient_local
        )

        # diff_drive_controller's C++ TransformBroadcaster always publishes to /tf
        self.create_subscription(TFMessage, "/tf", self._cb, best_effort_volatile)
        # robot_state_publisher's StaticTransformBroadcaster publishes to /tf_static
        self.create_subscription(
            TFMessage, "/tf_static", self._cb_static, reliable_transient_local
        )

        self._recv_count = 0
        self._fwd_count = 0
        # Warn if nothing arrives on /tf within 5 s of startup
        self.create_timer(5.0, self._check_alive)

        self.get_logger().info(
            f"tf_frame_renamer: /tf(_static) → (rename bare→{self._ns}/) → "
            f"/{self._ns}/tf(_static) + /tf(_static)"
        )

    def _check_alive(self) -> None:
        self.get_logger().info(
            f"[tf_frame_renamer] received={self._recv_count} forwarded={self._fwd_count}"
        )
        if self._recv_count == 0:
            self.get_logger().warn(
                "No transforms received on /tf yet — diff_drive_controller may not be "
                "publishing TF, or QoS mismatch on /tf"
            )

    def _rename_frame(self, frame_id: str) -> str:
        if "/" not in frame_id and frame_id in _BARE_FRAMES:
            return f"{self._ns}/{frame_id}"
        return frame_id

    def _rename_msg(self, msg: TFMessage) -> TFMessage:
        out = TFMessage()
        for t in msg.transforms:
            new_t = copy.deepcopy(t)
            new_t.header.frame_id = self._rename_frame(t.header.frame_id)
            new_t.child_frame_id = self._rename_frame(t.child_frame_id)
            out.transforms.append(new_t)
        return out

    def _any_renamed(self, msg: TFMessage, renamed: TFMessage) -> bool:
        return any(
            orig.header.frame_id != ren.header.frame_id
            or orig.child_frame_id != ren.child_frame_id
            for orig, ren in zip(msg.transforms, renamed.transforms)
        )

    def _cb(self, msg: TFMessage) -> None:
        self._recv_count += 1

        renamed = self._rename_msg(msg)

        # Avoid re-publishing our own output back to /tf (loop guard):
        # only publish to /tf when at least one frame was actually renamed.
        self._pub_ns.publish(renamed)
        self._fwd_count += 1
        if self._any_renamed(msg, renamed):
            self._pub_global.publish(renamed)

    def _cb_static(self, msg: TFMessage) -> None:
        renamed = self._rename_msg(msg)

        self._pub_ns_static.publish(renamed)
        if self._any_renamed(msg, renamed):
            self._pub_global_static.publish(renamed)


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
