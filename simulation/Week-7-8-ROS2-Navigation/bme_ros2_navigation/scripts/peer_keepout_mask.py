#!/usr/bin/env python3
"""
Publishes a small, dynamically-updated OccupancyGrid keepout mask (for
nav2_costmap_2d's KeepoutFilter, fed via costmap_filter_info_server) marking
a circle around each higher-priority peer's current live position as
lethal. Consumed by both local_costmap and global_costmap so a robot can
neither plan through nor drive through where a peer currently is.

This mask is purely a costmap-layer construct: it is never part of
/{robot}/map (raw SLAM) or /{robot}/nav_map / team_map_ddil (the maps
actually relayed to peers), so it never leaks into shared/communicated map
data -- only this robot's own local planning sees it.

Priority: which peers to avoid at all is decided by the launch file, not
here (see hw_namespaced_stack.launch.py) -- lower numeric robot ID wins, so
e.g. robot2 avoids robot1 but not vice versa, and robot3 avoids both. This
node just paints whatever peer list it's given; it has no opinion on
priority itself.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _apply_pose(origin_x, origin_y, yaw, px, py):
    c, s = math.cos(yaw), math.sin(yaw)
    return origin_x + c * px - s * py, origin_y + s * px + c * py


class PeerKeepoutMask(Node):
    def __init__(self):
        super().__init__("peer_keepout_mask")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("avoid_peer_names", [""])
        self.declare_parameter("avoid_peer_offsets_x", [0.0])
        self.declare_parameter("avoid_peer_offsets_y", [0.0])
        self.declare_parameter("avoid_peer_offsets_yaw", [0.0])
        self.declare_parameter("keepout_radius_m", 2.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_rate_hz", 2.0)

        robot_name = self.get_parameter("robot_name").value
        if not robot_name:
            raise RuntimeError("robot_name parameter is required")

        names = self.get_parameter("avoid_peer_names").value
        offsets_x = self.get_parameter("avoid_peer_offsets_x").value
        offsets_y = self.get_parameter("avoid_peer_offsets_y").value
        offsets_yaw = self.get_parameter("avoid_peer_offsets_yaw").value
        self._peer_offsets = {
            name: (ox, oy, oyaw)
            for name, ox, oy, oyaw in zip(names, offsets_x, offsets_y, offsets_yaw)
            if name
        }

        self._keepout_radius_m = float(self.get_parameter("keepout_radius_m").value)
        self._resolution = float(self.get_parameter("resolution").value)
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))

        # (map-frame x, map-frame y) per peer, updated as pose_broadcast
        # messages arrive via ddil_proxy_node's relay.
        self._peer_map_positions = {}

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for peer_name in self._peer_offsets:
            topic = f"/{robot_name}/incoming/{peer_name}/odom"
            self.create_subscription(
                Odometry, topic,
                lambda msg, name=peer_name: self._on_peer_odom(name, msg),
                qos)

        self._pub = self.create_publisher(OccupancyGrid, f"/{robot_name}/keepout_mask", qos)
        self.create_timer(1.0 / publish_rate_hz, self._publish)

        self.get_logger().info(
            f"peer_keepout_mask for {robot_name}: avoiding {list(self._peer_offsets)} "
            f"within {self._keepout_radius_m}m")

    def _on_peer_odom(self, name, msg):
        offset_x, offset_y, offset_yaw = self._peer_offsets[name]
        p = msg.pose.pose.position
        # Odometry is in the peer's own {peer}/odom frame -- transform into
        # the shared map frame the same way per_robot_map_compositor.py
        # transforms that peer's local SLAM map, using its spawn offset.
        # This ignores the peer's own map->odom drift correction (the same
        # simplification per_robot_map_compositor already makes for map
        # data), which is fine for a coarse 2m keepout margin.
        mx, my = _apply_pose(offset_x, offset_y, offset_yaw, p.x, p.y)
        self._peer_map_positions[name] = (mx, my)

    def _publish(self):
        if not self._peer_map_positions:
            return

        pad = self._keepout_radius_m + self._resolution
        xs = [x for x, _ in self._peer_map_positions.values()]
        ys = [y for _, y in self._peer_map_positions.values()]
        min_x, max_x = min(xs) - pad, max(xs) + pad
        min_y, max_y = min(ys) - pad, max(ys) + pad

        res = self._resolution
        origin_x = math.floor(min_x / res) * res
        origin_y = math.floor(min_y / res) * res
        width = max(1, int(math.ceil((max_x - origin_x) / res)))
        height = max(1, int(math.ceil((max_y - origin_y) / res)))

        grid = np.zeros((height, width), dtype=np.int8)
        cols = np.arange(width)
        rows = np.arange(height)
        cell_x = origin_x + (cols + 0.5) * res
        cell_y = origin_y + (rows + 0.5) * res
        cx, cy = np.meshgrid(cell_x, cell_y)

        for px, py in self._peer_map_positions.values():
            within = (cx - px) ** 2 + (cy - py) ** 2 <= self._keepout_radius_m ** 2
            grid[within] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).tolist()
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = PeerKeepoutMask()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
