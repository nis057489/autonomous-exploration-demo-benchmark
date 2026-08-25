#!/usr/bin/env python3
"""Keeps a lower-priority robot's global planner from routing through a
higher-priority robot's currently active path.

Subscribes to one or more peers' relayed nav_msgs/Path (nav2 planner_server's
own `plan` topic, DDIL-relayed the same way peer maps already are), rasterizes
a `clearance_m` corridor around each into a small nav2 costmap-filter keepout
mask, and republishes it for this robot's own global_costmap KeepoutFilter to
consume.

Deliberately asymmetric and fail-open: which peers this instance is even told
about is decided once, at launch time, by robot priority (see
hw_namespaced_stack.launch.py's _robot_priority/yield_to_peers) -- a robot
only ever yields to strictly-higher-priority peers, never the reverse, so two
robots can't both defer to each other and deadlock. If a peer's plan goes
stale or never arrives, its contribution to the mask just disappears -- there
is no negotiation protocol and this is not meant to be a safety-critical
collision guarantee on its own. Actual robot-to-robot collision avoidance is
unchanged and stays with each robot's own LIDAR-fed obstacle_layer and
collision_monitor, which react to a real nearby robot regardless of what this
node knows.
"""
import time

import numpy as np
import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_KEEPOUT_FILTER_TYPE = 0  # nav2_msgs/CostmapFilterInfo: 0 = keepout/lanes filter


def _mark_segment(grid, origin_x, origin_y, resolution, x0, y0, x1, y1, clearance_cells):
    """Mark cells of `grid` within clearance_cells of segment (x0,y0)-(x1,y1)
    as 100 (occupied). Only touches the segment's own local bounding box, not
    the whole grid, so cost stays proportional to path length, not map size.
    """
    height, width = grid.shape
    col0 = (x0 - origin_x) / resolution
    row0 = (y0 - origin_y) / resolution
    col1 = (x1 - origin_x) / resolution
    row1 = (y1 - origin_y) / resolution

    min_col = max(0, int(np.floor(min(col0, col1) - clearance_cells)))
    max_col = min(width - 1, int(np.ceil(max(col0, col1) + clearance_cells)))
    min_row = max(0, int(np.floor(min(row0, row1) - clearance_cells)))
    max_row = min(height - 1, int(np.ceil(max(row0, row1) + clearance_cells)))
    if min_col > max_col or min_row > max_row:
        return

    cols = np.arange(min_col, max_col + 1)
    rows = np.arange(min_row, max_row + 1)
    grid_cols, grid_rows = np.meshgrid(cols, rows)

    seg_dx = col1 - col0
    seg_dy = row1 - row0
    seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
    if seg_len_sq < 1e-9:
        t = np.zeros_like(grid_cols, dtype=np.float64)
    else:
        t = ((grid_cols - col0) * seg_dx + (grid_rows - row0) * seg_dy) / seg_len_sq
        t = np.clip(t, 0.0, 1.0)

    closest_col = col0 + t * seg_dx
    closest_row = row0 + t * seg_dy
    dist_cells = np.hypot(grid_cols - closest_col, grid_rows - closest_row)

    sub = grid[min_row:max_row + 1, min_col:max_col + 1]
    sub[dist_cells <= clearance_cells] = 100


def rasterize_keepout_mask(paths_xy, clearance_m, resolution, padding_m, max_cells_per_dim=2000):
    """paths_xy: list of peer paths, each a list of (x, y) waypoints in the
    global frame. Returns (data, width, height, origin_x, origin_y) for an
    OccupancyGrid-style mask: -1 = no opinion, 100 = within clearance_m of a
    peer's path. Always returns something publishable, even with no input --
    callers don't need a special case for "no peers currently active".
    """
    points = [xy for path in paths_xy for xy in path]
    if not points:
        return [-1], 1, 1, 0.0, 0.0

    margin = clearance_m + padding_m
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin

    origin_x = float(np.floor(min_x / resolution) * resolution)
    origin_y = float(np.floor(min_y / resolution) * resolution)
    width = min(max_cells_per_dim, int(np.ceil((max_x - origin_x) / resolution)) + 1)
    height = min(max_cells_per_dim, int(np.ceil((max_y - origin_y) / resolution)) + 1)

    grid = np.full((height, width), -1, dtype=np.int8)
    clearance_cells = clearance_m / resolution

    for path in paths_xy:
        if len(path) == 1:
            x, y = path[0]
            _mark_segment(grid, origin_x, origin_y, resolution, x, y, x, y, clearance_cells)
        else:
            for (x0, y0), (x1, y1) in zip(path, path[1:]):
                _mark_segment(
                    grid, origin_x, origin_y, resolution, x0, y0, x1, y1, clearance_cells
                )

    return grid.flatten().tolist(), width, height, origin_x, origin_y


class PeerPlanKeepout(Node):
    def __init__(self):
        super().__init__("peer_plan_keepout")

        self.declare_parameter("robot_names", [""])
        self.declare_parameter("plan_topic_template", "/{name}/plan")
        self.declare_parameter("clearance_m", 0.5)
        self.declare_parameter("padding_m", 0.1)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("plan_ttl_s", 5.0)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("mask_topic", "peer_plan_keepout/mask")
        self.declare_parameter("filter_info_topic", "peer_plan_keepout/filter_info")

        robot_names = [name for name in self.get_parameter("robot_names").value if name]
        plan_topic_template = str(self.get_parameter("plan_topic_template").value)
        self.clearance_m = float(self.get_parameter("clearance_m").value)
        self.padding_m = float(self.get_parameter("padding_m").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.plan_ttl_s = float(self.get_parameter("plan_ttl_s").value)
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.global_frame = str(self.get_parameter("global_frame").value)
        mask_topic = str(self.get_parameter("mask_topic").value)
        filter_info_topic = str(self.get_parameter("filter_info_topic").value)

        # Matches the ddil_proxy relay's own output QoS for "reliable"-flagged
        # entries (RELIABLE + TRANSIENT_LOCAL, depth 1) and the QoS nav2's
        # KeepoutFilter/map-style consumers expect on filter info + masks.
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._peer_paths = {}    # name -> list[(x, y)]
        self._peer_seen_at = {}  # name -> monotonic seconds

        self.mask_pub = self.create_publisher(OccupancyGrid, mask_topic, latched_qos)
        self.filter_info_pub = self.create_publisher(
            CostmapFilterInfo, filter_info_topic, latched_qos
        )

        self._plan_subs = [
            self.create_subscription(
                Path, plan_topic_template.format(name=name),
                lambda msg, peer_name=name: self._on_plan(peer_name, msg),
                latched_qos,
            )
            for name in robot_names
        ]

        filter_info = CostmapFilterInfo()
        filter_info.header.stamp = self.get_clock().now().to_msg()
        filter_info.type = _KEEPOUT_FILTER_TYPE
        filter_info.filter_mask_topic = mask_topic
        filter_info.base = 0.0
        filter_info.multiplier = 1.0
        self.filter_info_pub.publish(filter_info)

        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_mask)

        if robot_names:
            self.get_logger().info(
                f"peer_plan_keepout: yielding {self.clearance_m}m to "
                + ", ".join(robot_names)
            )
        else:
            self.get_logger().info("peer_plan_keepout: no higher-priority peers, idle.")

    def _on_plan(self, peer_name, msg):
        self._peer_paths[peer_name] = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        self._peer_seen_at[peer_name] = time.monotonic()

    def _publish_mask(self):
        now = time.monotonic()
        active_paths = [
            path for name, path in self._peer_paths.items()
            if path and now - self._peer_seen_at.get(name, 0.0) <= self.plan_ttl_s
        ]

        data, width, height, origin_x, origin_y = rasterize_keepout_mask(
            active_paths, self.clearance_m, self.resolution, self.padding_m
        )

        mask = OccupancyGrid()
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.header.frame_id = self.global_frame
        mask.info.resolution = self.resolution
        mask.info.width = width
        mask.info.height = height
        mask.info.origin.position.x = origin_x
        mask.info.origin.position.y = origin_y
        mask.info.origin.orientation.w = 1.0
        mask.data = data
        self.mask_pub.publish(mask)


def main():
    rclpy.init()
    node = PeerPlanKeepout()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
