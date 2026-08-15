#!/usr/bin/env python3
"""Report exact per-topic message count and byte total for a recorded run's
bag, e.g. to measure how many bits were actually sent/received on a given
map-transport topic (baseline: /map, /incoming/<peer>/map; vxch: vxch/map/
band_*, incoming/<peer>/band_*). `ros2 bag info` only reports message counts,
not byte sizes, so this reads the bag directly via rosbag2_py and sums each
message's serialized length.

Must run where rosbag2_py is importable (i.e. with a ROS environment
sourced, such as inside the jazzy_env distrobox).

Usage: ./bag_bandwidth.py <bag_dir> [topic_substring]
  bag_dir          Path to a run's bag/ directory (needs metadata.yaml --
                    see replay_compare.sh's reindex step if it's missing).
  topic_substring  Only report topics containing this substring.
"""
import sys

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_dir> [topic_substring]", file=sys.stderr)
        sys.exit(1)
    bag_dir = sys.argv[1]
    topic_filter = sys.argv[2] if len(sys.argv) > 2 else None

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id=""),
        ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )

    counts = {}
    totals = {}
    for topic_meta in reader.get_all_topics_and_types():
        counts[topic_meta.name] = 0
        totals[topic_meta.name] = 0

    while reader.has_next():
        topic, data, _ts = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        totals[topic] = totals.get(topic, 0) + len(data)

    rows = sorted(totals.items(), key=lambda kv: kv[1], reverse=True)
    if topic_filter:
        rows = [(t, b) for t, b in rows if topic_filter in t]

    grand_total = 0
    print(f"{'topic':<55} {'msgs':>8} {'bytes':>12} {'KB':>10}")
    for topic, nbytes in rows:
        grand_total += nbytes
        print(f"{topic:<55} {counts[topic]:>8} {nbytes:>12} {nbytes / 1024:>10.1f}")
    print(f"\ntotal: {grand_total} bytes ({grand_total / 1024:.1f} KB, {grand_total * 8} bits)")


if __name__ == "__main__":
    main()
