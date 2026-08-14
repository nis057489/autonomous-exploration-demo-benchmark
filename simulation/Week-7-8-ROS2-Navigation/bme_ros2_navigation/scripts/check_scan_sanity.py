#!/usr/bin/env python3
"""
Sanity-check a LaserScan topic: frame_id, declared range_min/max, and
whether the actual range readings look sane (not all zero/inf/nan, roughly
matching the sensor's real range spec).

Usage:
    python3 check_scan_sanity.py /robot1/scan
    python3 check_scan_sanity.py /robot1/scan --expect-frame base_scan --expect-max 8.0
"""

import argparse
import statistics
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSanityCheck(Node):
    def __init__(self, topic, count, expect_frame=None, expect_max=None):
        super().__init__("scan_sanity_check")
        self._count = count
        self._seen = 0
        self._expect_frame = expect_frame
        self._expect_max = expect_max
        self._sub = self.create_subscription(LaserScan, topic, self._cb, 10)

    def _cb(self, msg: LaserScan):
        self._seen += 1

        n = len(msg.ranges)
        finite = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        zero = sum(1 for r in msg.ranges if r == 0.0)
        below_min = sum(1 for r in msg.ranges if 0.0 < r < msg.range_min)
        above_max = n - len(finite) - zero - below_min

        print(f"--- message {self._seen} ---")
        print(f"frame_id:            {msg.frame_id!r}")
        print(f"declared range_min:  {msg.range_min:.3f} m")
        print(f"declared range_max:  {msg.range_max:.3f} m")
        print(f"angle_min/max:       {msg.angle_min:.3f} / {msg.angle_max:.3f} rad")
        print(f"readings:            {n} total, {len(finite)} in-range, "
              f"{zero} zero, {below_min} below range_min, {above_max} above range_max")
        if finite:
            print(f"actual min/max/mean: "
                  f"{min(finite):.3f} / {max(finite):.3f} / {statistics.mean(finite):.3f} m")
        else:
            print("WARNING: no in-range readings at all")

        if n and zero / n > 0.5:
            print("WARNING: over half the readings are exactly 0.0 -- likely a driver/wiring issue")
        if n and len(finite) / n < 0.1:
            print("WARNING: fewer than 10% of readings are in-range -- check mounting/obstruction")
        if self._expect_frame is not None and self._expect_frame not in msg.frame_id:
            print(f"WARNING: frame_id {msg.frame_id!r} does not contain expected "
                  f"{self._expect_frame!r}")
        if self._expect_max is not None and abs(msg.range_max - self._expect_max) > 0.5:
            print(f"WARNING: declared range_max {msg.range_max:.3f} m differs from expected "
                  f"{self._expect_max:.3f} m by more than 0.5 m")

        if self._seen >= self._count:
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic", nargs="?", default="/scan", help="LaserScan topic (default: /scan)")
    parser.add_argument("--count", type=int, default=1, help="number of messages to sample (default: 1)")
    parser.add_argument("--expect-frame", default=None,
                         help="substring expected in frame_id, e.g. 'base_scan'")
    parser.add_argument("--expect-max", type=float, default=None,
                         help="expected sensor max range in meters, e.g. 8.0 for LDS-02")
    parser.add_argument("--timeout", type=float, default=10.0,
                         help="seconds to wait for messages before giving up (default: 10)")
    args = parser.parse_args()

    rclpy.init()
    node = ScanSanityCheck(args.topic, args.count, args.expect_frame, args.expect_max)

    print(f"Listening on {args.topic!r} for {args.count} message(s), timeout {args.timeout}s...\n")

    import time
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass

    if node._seen == 0:
        print(f"No messages received on {args.topic!r} within {args.timeout}s "
              f"-- topic may not be publishing.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
