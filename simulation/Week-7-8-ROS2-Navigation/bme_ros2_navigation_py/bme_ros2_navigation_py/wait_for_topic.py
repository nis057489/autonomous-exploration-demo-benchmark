#!/usr/bin/env python3
"""
Blocks until a first message arrives on a topic, then exits 0.

Used by hw_namespaced_stack.launch.py to gate SLAM on turtlebot3 bringup
actually publishing scans, and to gate Nav2 on slam_toolbox actually
publishing a first map -- replacing fixed TimerAction delays (which assume
a worst-case startup time instead of observing the real one) with a wait on
the actual readiness signal each stage depends on. Exits 0 either way (even
on timeout) so a slow-but-eventually-fine robot never gets stuck behind a
hard launch failure; the timeout log line exists so a genuinely stuck robot
is loud and diagnosable instead of just quietly guessed past.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--type", required=True,
                         help="Message type, e.g. sensor_msgs/msg/LaserScan")
    parser.add_argument("--timeout", type=float, default=90.0,
                         help="Seconds to wait before giving up and exiting anyway (default 90).")
    parser.add_argument("--reliability", choices=["reliable", "best_effort"], default="reliable")
    parser.add_argument("--durability", choices=["transient_local", "volatile"],
                         default="transient_local")
    parser.add_argument("--log-interval", type=float, default=10.0,
                         help="Seconds between 'still waiting' log lines.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("wait_for_topic_" + args.topic.strip("/").replace("/", "_"))

    msg_type = get_message(args.type)
    qos = QoSProfile(
        depth=1,
        reliability=(
            QoSReliabilityPolicy.RELIABLE if args.reliability == "reliable"
            else QoSReliabilityPolicy.BEST_EFFORT
        ),
        durability=(
            QoSDurabilityPolicy.TRANSIENT_LOCAL if args.durability == "transient_local"
            else QoSDurabilityPolicy.VOLATILE
        ),
    )

    received = False

    def _cb(_msg):
        nonlocal received
        received = True

    node.create_subscription(msg_type, args.topic, _cb, qos)

    node.get_logger().info(f"Waiting for first message on '{args.topic}' (timeout={args.timeout}s)...")

    deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout
    next_log = node.get_clock().now().nanoseconds / 1e9 + args.log_interval
    while rclpy.ok() and not received:
        now = node.get_clock().now().nanoseconds / 1e9
        if now >= deadline:
            node.get_logger().error(
                f"Timed out after {args.timeout}s waiting for '{args.topic}' -- "
                "proceeding anyway; downstream stage may start against a not-yet-ready dependency."
            )
            break
        if now >= next_log:
            node.get_logger().warn(f"Still waiting for '{args.topic}'...")
            next_log = now + args.log_interval
        rclpy.spin_once(node, timeout_sec=0.2)

    if received:
        node.get_logger().info(f"Received first message on '{args.topic}'.")

    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
