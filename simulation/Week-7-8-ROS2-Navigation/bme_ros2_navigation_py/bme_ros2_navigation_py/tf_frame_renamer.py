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

Publishing renamed frames back onto /tf(_static) means this node's own /tf(_static)
subscription receives its own output as new input (tf2_msgs carries no publisher
identity in the plain rclpy callback, so there's no cheaper way to recognize "this
is mine" than content). Left unhandled, every already-renamed message gets treated
as fresh, forwarded to /{ns}/tf again, and counted in received/forwarded -- doubling
real traffic on /{ns}/tf and, transitively, CPU in every Nav2 node subscribed there
(measured on hardware: /{ns}/tf running at ~2.6x the rate of bare /tf). _RecentSignatures
recognizes and drops that bounced-back copy before it's reprocessed.
"""

import sys
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

_BARE_FRAMES = {"odom", "base_footprint", "base_link", "base_scan"}


def _signature(t: TransformStamped) -> tuple[str, str, int, int]:
    return (t.header.frame_id, t.child_frame_id, t.header.stamp.sec, t.header.stamp.nanosec)


class _RecentSignatures:
    """Bounded, O(1)-amortized recently-seen-transform set.

    A plain deque.__contains__ scan is fine at this size (default maxlen 256,
    well under 82-219 Hz, and small in absolute node count too), but a
    growing per-message set() would be needless allocation on a hot path.
    """

    def __init__(self, maxlen: int = 256) -> None:
        self._order: deque[tuple[str, str, int, int]] = deque(maxlen=maxlen)
        self._set: set[tuple[str, str, int, int]] = set()

    def __contains__(self, sig: tuple[str, str, int, int]) -> bool:
        return sig in self._set

    def add(self, sig: tuple[str, str, int, int]) -> None:
        if len(self._order) == self._order.maxlen:
            self._set.discard(self._order[0])
        self._order.append(sig)
        self._set.add(sig)


class TFFrameRenamer(Node):
    def __init__(self):
        super().__init__("tf_frame_renamer")
        self.declare_parameter("namespace", "")
        self._ns = self.get_parameter(
            "namespace").get_parameter_value().string_value
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
        self._pub_ns = self.create_publisher(
            TFMessage, f"/{self._ns}/tf", reliable_volatile)
        # /tf — RViz and Python tf2_ros.TransformListener (RELIABLE)
        self._pub_global = self.create_publisher(
            TFMessage, "/tf", reliable_volatile)

        self._pub_ns_static = self.create_publisher(
            TFMessage, f"/{self._ns}/tf_static", reliable_transient_local
        )
        self._pub_global_static = self.create_publisher(
            TFMessage, "/tf_static", reliable_transient_local
        )

        # diff_drive_controller's C++ TransformBroadcaster always publishes to /tf
        self.create_subscription(
            TFMessage, "/tf", self._cb, best_effort_volatile)
        # robot_state_publisher's StaticTransformBroadcaster publishes to /tf_static
        self.create_subscription(
            TFMessage, "/tf_static", self._cb_static, reliable_transient_local
        )

        self._recent_published = _RecentSignatures()
        self._recent_published_static = _RecentSignatures()

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

    def _rename_msg(self, msg: TFMessage) -> tuple[TFMessage, bool]:
        """Rename bare frames, reusing the original message untouched (no copy at
        all) when nothing in it needs renaming -- e.g. slam_toolbox already
        publishes namespaced frames, so most of its /tf traffic is a whole-message
        no-op. Previously this still allocated a new TFMessage and rebuilt the
        transforms list on every single callback even in that case; on a hot /tf
        topic under CPU pressure that per-message object churn is real cost for
        no reason, since nothing in the message ever changed.
        """
        out_transforms = None  # stays None until the first frame that actually needs renaming
        for i, t in enumerate(msg.transforms):
            new_frame = self._rename_frame(t.header.frame_id)
            new_child = self._rename_frame(t.child_frame_id)
            if new_frame == t.header.frame_id and new_child == t.child_frame_id:
                if out_transforms is not None:
                    out_transforms.append(t)
                continue
            if out_transforms is None:
                # First rename in this message -- start a real copy, backfilling
                # everything already scanned unchanged.
                out_transforms = list(msg.transforms[:i])
            new_t = TransformStamped()
            new_t.header.stamp = t.header.stamp
            new_t.header.frame_id = new_frame
            new_t.child_frame_id = new_child
            new_t.transform = t.transform  # unmodified, safe to share by reference
            out_transforms.append(new_t)

        if out_transforms is None:
            return msg, False

        out = TFMessage()
        out.transforms = out_transforms
        return out, True

    def _cb(self, msg: TFMessage) -> None:
        self._recv_count += 1

        # Drop transforms that are our own renamed output bouncing back through
        # this same /tf subscription (see module docstring) before doing any
        # renaming/forwarding work on them -- otherwise every renamed transform
        # gets forwarded to /{ns}/tf twice: once as itself, once as its own echo.
        fresh = [t for t in msg.transforms if _signature(t) not in self._recent_published]
        if not fresh:
            return
        if len(fresh) != len(msg.transforms):
            filtered = TFMessage()
            filtered.transforms = fresh
            msg = filtered

        renamed, changed = self._rename_msg(msg)

        self._pub_ns.publish(renamed)
        self._fwd_count += 1
        # Avoid re-publishing our own output back to /tf (loop guard):
        # only publish to /tf when at least one frame was actually renamed.
        if changed:
            for t in renamed.transforms:
                self._recent_published.add(_signature(t))
            self._pub_global.publish(renamed)

    def _cb_static(self, msg: TFMessage) -> None:
        fresh = [t for t in msg.transforms if _signature(t) not in self._recent_published_static]
        if not fresh:
            return
        if len(fresh) != len(msg.transforms):
            filtered = TFMessage()
            filtered.transforms = fresh
            msg = filtered

        renamed, changed = self._rename_msg(msg)

        self._pub_ns_static.publish(renamed)
        if changed:
            for t in renamed.transforms:
                self._recent_published_static.add(_signature(t))
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
