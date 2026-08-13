#!/usr/bin/env python3
"""
Blocks until required topics have published at least one message and required
TF transforms are available, or a timeout elapses.

Replaces fixed-delay TimerActions in hw_namespaced_stack.launch.py, which
assumed hardware bringup / SLAM would always finish within a guessed number of
seconds. When that guess is wrong (e.g. turtlebot3_ros stalls during startup),
the previous behavior started the next stage anyway against missing inputs --
which only surfaced many minutes later as a cryptic "TF has two or more
unconnected trees" error, with no indication of what actually went wrong.

Exits 0 if everything became ready in time, exits 1 (after logging exactly
what's still missing) if the timeout elapsed first. The launch file treats a
nonzero exit as fatal for this robot's whole launch (see the on_exit handlers
in hw_namespaced_stack.launch.py) rather than continuing on with known-broken
inputs -- if this is unready, downstream stages (SLAM, Nav2) are going to fail
anyway, just more confusingly and later.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


class WaitForReady(Node):
    def __init__(self):
        super().__init__("wait_for_ready")
        self.declare_parameter("topics", [""])
        self.declare_parameter("tf_target_frames", [""])
        self.declare_parameter("tf_source_frames", [""])
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("label", "bringup")

        self.topics = [t for t in self.get_parameter("topics").value if t]
        tf_targets = [t for t in self.get_parameter("tf_target_frames").value if t]
        tf_sources = [t for t in self.get_parameter("tf_source_frames").value if t]
        self.tf_pairs = list(zip(tf_targets, tf_sources))
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.label = str(self.get_parameter("label").value)

        self._received = {t: False for t in self.topics}
        self._subs = {}

        # tf2_ros imported lazily/locally so this script still runs (for the
        # topic-only case) even in an environment missing the tf2_ros python
        # bindings for some reason.
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _try_subscribe_unresolved(self):
        # Topic type isn't known until its publisher exists, so this retries
        # every poll until each topic can actually be subscribed to.
        names_and_types = dict(self.get_topic_names_and_types())
        for topic in self.topics:
            if topic in self._subs:
                continue
            types = names_and_types.get(topic)
            if not types:
                continue
            try:
                msg_type = get_message(types[0])
            except (ValueError, ModuleNotFoundError):
                continue
            self._subs[topic] = self.create_subscription(
                msg_type, topic,
                lambda _msg, t=topic: self._received.__setitem__(t, True),
                qos_profile_sensor_data,
            )

    def _missing(self):
        missing = []
        for t in self.topics:
            if not self._received[t]:
                missing.append(f"topic '{t}' has not published yet")
        for target, source in self.tf_pairs:
            if not self.tf_buffer.can_transform(target, source, rclpy.time.Time()):
                missing.append(f"tf '{source}' -> '{target}' not available")
        return missing

    def wait(self):
        deadline = time.monotonic() + self.timeout_sec
        last_log = 0.0
        while rclpy.ok():
            self._try_subscribe_unresolved()
            rclpy.spin_once(self, timeout_sec=0.5)
            missing = self._missing()
            if not missing:
                self.get_logger().info(f"[{self.label}] ready.")
                return True
            now = time.monotonic()
            if now >= deadline:
                self.get_logger().error(
                    f"[{self.label}] NOT ready after {self.timeout_sec:.0f}s -- "
                    "aborting (check hardware bringup / driver connection): "
                    + "; ".join(missing)
                )
                return False
            if now - last_log > 5.0:
                self.get_logger().warn(f"[{self.label}] still waiting: " + "; ".join(missing))
                last_log = now
        return False


def main():
    rclpy.init()
    node = WaitForReady()
    try:
        ready = node.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ready else 1)


if __name__ == "__main__":
    main()
