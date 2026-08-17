#!/usr/bin/env python3
"""
Merge a robot's own fresh SLAM map with the (DDIL-affected) team-fused map into a
single nav_map that Nav2 and the frontier explorer subscribe to.

  /{robot}/map         — own SLAM, robot-local frame, always fresh
  /{robot}/global_map  — team fused (output of DdilProxy or VXCH decoder), global frame
       ↓
  /{robot}/nav_map     — merged result, global frame → Nav2 + frontier_explorer

Merge rule
----------
The team map (global frame) is the base canvas.  Every non-unknown cell from the
robot's own local SLAM is transformed to the global frame using the robot's spawn
offset, then WRITTEN INTO the canvas — overriding stale team data with fresh local
observations.  Unknown cells (-1) in the local SLAM never clear team knowledge.

When the team map is absent (link fully blocked) the node falls back to transforming
the local SLAM map alone into the global frame.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _apply_pose(origin_x, origin_y, yaw, px, py):
    c, s = math.cos(yaw), math.sin(yaw)
    return origin_x + c * px - s * py, origin_y + s * px + c * py


class PerRobotMapCompositor(Node):
    def __init__(self):
        super().__init__("per_robot_map_compositor")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_yaw", 0.0)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("team_map_stale_threshold_s", 8.0)

        robot_name = self.get_parameter("robot_name").value
        if not robot_name:
            raise RuntimeError("robot_name parameter is required")

        self._offset_x = float(self.get_parameter("offset_x").value)
        self._offset_y = float(self.get_parameter("offset_y").value)
        self._offset_yaw = float(self.get_parameter("offset_yaw").value)
        publish_rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._team_map_stale_threshold_s = max(
            0.0, float(self.get_parameter("team_map_stale_threshold_s").value))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._local_map: OccupancyGrid | None = None
        self._team_map: OccupancyGrid | None = None

        self._local_sub = self.create_subscription(
            OccupancyGrid,
            f"/{robot_name}/map",
            self._local_callback,
            qos,
        )
        self._team_sub = self.create_subscription(
            OccupancyGrid,
            f"/{robot_name}/team_map_ddil",
            self._team_callback,
            qos,
        )
        self._pub = self.create_publisher(OccupancyGrid, f"/{robot_name}/nav_map", qos)
        # Visualization only -- never subscribed to by Nav2 or frontier_explorer.
        # Same merge as nav_map, except team-sourced cells are suppressed (left
        # unknown) once the team map hasn't been refreshed in
        # team_map_stale_threshold_s, so a demo operator can see "this part of
        # the map is stale team data" instead of it silently blending in as if
        # fresh. Under a heavily bandwidth-throttled DDIL link this gap is
        # expected, not a bug -- see nav_map for the real, unsuppressed map.
        self._debug_pub = self.create_publisher(
            OccupancyGrid, f"/{robot_name}/nav_map_debug", qos)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f"PerRobotMapCompositor for {robot_name}: "
            f"offset=({self._offset_x:.2f}, {self._offset_y:.2f}, {self._offset_yaw:.3f} rad)"
        )

    def _local_callback(self, msg: OccupancyGrid):
        self._local_map = msg

    def _team_callback(self, msg: OccupancyGrid):
        self._team_map = msg

    def _publish(self):
        local = self._local_map
        team = self._team_map

        if local is None:
            return  # no local map yet — nothing to publish

        have_team = team is not None and team.info.width > 0 and team.info.height > 0
        if have_team:
            out = self._merge_local_onto_team(local, team)
        else:
            out = self._local_to_global(local)

        if out is not None:
            out.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(out)

        # Debug-only view: same merge, but team cells are suppressed once stale.
        # Skipped entirely (no second merge computed) unless something is
        # actually subscribed -- e.g. RViz has the topic open for a demo --
        # so this costs nothing on the already CPU-tight robots the rest of
        # the time.
        if self._debug_pub.get_subscription_count() == 0:
            return
        if have_team:
            debug = self._merge_local_onto_team(
                local, team, suppress_team=self._team_map_is_stale(team))
        else:
            debug = out
        if debug is not None:
            debug.header.stamp = self.get_clock().now().to_msg()
            self._debug_pub.publish(debug)

    def _team_map_is_stale(self, team: OccupancyGrid) -> bool:
        if self._team_map_stale_threshold_s <= 0.0:
            return False
        age_s = (self.get_clock().now() - Time.from_msg(team.header.stamp)).nanoseconds / 1e9
        return age_s > self._team_map_stale_threshold_s

    # ── helpers ────────────────────────────────────────────────────────────────

    def _merge_local_onto_team(
        self, local: OccupancyGrid, team: OccupancyGrid, suppress_team: bool = False
    ) -> OccupancyGrid | None:
        """Canvas is the union of team's bounds and this robot's own (offset-
        transformed) local bounds -- not just team's bounds alone. team_map_ddil
        only carries *other* robots' areas (this robot is deliberately excluded
        from its own team fusion), so if this robot's own explored area doesn't
        spatially overlap a peer's, using team's bounds as the whole canvas would
        silently drop this robot's own surroundings entirely (every local cell
        falls outside team's bounds and _stamp_local_cells discards it) -- nav2
        then has no real local costmap data to plan against.
        """
        res = min(team.info.resolution, local.info.resolution)
        if res <= 0.0:
            res = team.info.resolution or local.info.resolution
            if res <= 0.0:
                return None

        team_min_x = team.info.origin.position.x
        team_min_y = team.info.origin.position.y
        team_max_x = team_min_x + team.info.width * team.info.resolution
        team_max_y = team_min_y + team.info.height * team.info.resolution

        local_bounds = self._local_global_bounds(local)
        if local_bounds is not None:
            local_min_x, local_min_y, local_max_x, local_max_y = local_bounds
            min_x = min(team_min_x, local_min_x)
            min_y = min(team_min_y, local_min_y)
            max_x = max(team_max_x, local_max_x)
            max_y = max(team_max_y, local_max_y)
        else:
            min_x, min_y, max_x, max_y = team_min_x, team_min_y, team_max_x, team_max_y

        origin_x = math.floor(min_x / res) * res
        origin_y = math.floor(min_y / res) * res
        width = max(1, int(math.ceil((max_x - origin_x) / res)) + 1)
        height = max(1, int(math.ceil((max_y - origin_y) / res)) + 1)

        out = OccupancyGrid()
        out.header.frame_id = team.header.frame_id
        out.info.resolution = res
        out.info.width = width
        out.info.height = height
        out.info.origin.position.x = origin_x
        out.info.origin.position.y = origin_y
        out.info.origin.orientation.w = 1.0

        canvas = np.full((height, width), -1, dtype=np.int8)
        if not suppress_team:
            self._copy_team_cells(team, canvas, origin_x, origin_y, res)
        self._stamp_local_cells(local, canvas, origin_x, origin_y, res)
        out.data = canvas.reshape(-1).tolist()
        return out

    def _local_global_bounds(self, local: OccupancyGrid):
        """This robot's own local map bounds, transformed into the shared map frame."""
        map_info = local.info
        if map_info.width == 0 or map_info.height == 0 or map_info.resolution <= 0.0:
            return None

        map_yaw = _yaw_from_quaternion(map_info.origin.orientation)
        map_w = map_info.width * map_info.resolution
        map_h = map_info.height * map_info.resolution
        corners_local = [(0.0, 0.0), (map_w, 0.0), (map_w, map_h), (0.0, map_h)]

        min_x = min_y = math.inf
        max_x = max_y = -math.inf
        for lx, ly in corners_local:
            mx, my = _apply_pose(map_info.origin.position.x, map_info.origin.position.y, map_yaw, lx, ly)
            gx, gy = _apply_pose(self._offset_x, self._offset_y, self._offset_yaw, mx, my)
            min_x, min_y = min(min_x, gx), min(min_y, gy)
            max_x, max_y = max(max_x, gx), max(max_y, gy)
        return min_x, min_y, max_x, max_y

    def _copy_team_cells(
        self, team: OccupancyGrid, canvas: np.ndarray, canvas_origin_x: float,
        canvas_origin_y: float, res_dst: float,
    ):
        """Copy team's cells into canvas at team's (axis-aligned) position within it."""
        col_offset = round((team.info.origin.position.x - canvas_origin_x) / res_dst)
        row_offset = round((team.info.origin.position.y - canvas_origin_y) / res_dst)
        out_h, out_w = canvas.shape
        t_h, t_w = team.info.height, team.info.width
        if t_h == 0 or t_w == 0:
            return

        src_row0, src_row1 = max(0, -row_offset), min(t_h, out_h - row_offset)
        src_col0, src_col1 = max(0, -col_offset), min(t_w, out_w - col_offset)
        if src_row0 >= src_row1 or src_col0 >= src_col1:
            return  # team map falls entirely outside canvas

        dst_row0, dst_row1 = src_row0 + row_offset, src_row1 + row_offset
        dst_col0, dst_col1 = src_col0 + col_offset, src_col1 + col_offset

        team_arr = np.asarray(team.data, dtype=np.int8).reshape(t_h, t_w)
        canvas[dst_row0:dst_row1, dst_col0:dst_col1] = team_arr[src_row0:src_row1, src_col0:src_col1]

    def _local_to_global(self, local: OccupancyGrid) -> OccupancyGrid | None:
        """When no team map is available, transform the local SLAM map to global frame."""
        if local.info.width == 0 or local.info.height == 0:
            return None

        res = local.info.resolution
        if res <= 0.0:
            return None

        # Compute global-frame bounding box for this robot's map
        map_info = local.info
        map_yaw = _yaw_from_quaternion(map_info.origin.orientation)
        map_w = map_info.width * res
        map_h = map_info.height * res
        corners_local = [(0.0, 0.0), (map_w, 0.0), (map_w, map_h), (0.0, map_h)]

        min_x = min_y = math.inf
        max_x = max_y = -math.inf
        for lx, ly in corners_local:
            mx, my = _apply_pose(map_info.origin.position.x, map_info.origin.position.y, map_yaw, lx, ly)
            gx, gy = _apply_pose(self._offset_x, self._offset_y, self._offset_yaw, mx, my)
            min_x, min_y = min(min_x, gx), min(min_y, gy)
            max_x, max_y = max(max_x, gx), max(max_y, gy)

        origin_x = math.floor(min_x / res) * res
        origin_y = math.floor(min_y / res) * res
        width = max(1, int(math.ceil((max_x - origin_x) / res)) + 1)
        height = max(1, int(math.ceil((max_y - origin_y) / res)) + 1)

        out = OccupancyGrid()
        out.header.frame_id = "map"
        out.info.resolution = res
        out.info.width = width
        out.info.height = height
        out.info.origin.position.x = origin_x
        out.info.origin.position.y = origin_y
        out.info.origin.orientation.w = 1.0

        canvas = np.full((height, width), -1, dtype=np.int8)
        self._stamp_local_cells(local, canvas, origin_x, origin_y, res)
        out.data = canvas.reshape(-1).tolist()
        return out

    def _stamp_local_cells(
        self, local: OccupancyGrid, canvas: np.ndarray, origin_x: float,
        origin_y: float, res_dst: float,
    ):
        """Transform non-unknown local SLAM cells into canvas (in-place)."""
        map_info = local.info
        w, h = map_info.width, map_info.height
        if w == 0 or h == 0:
            return

        res_src = map_info.resolution
        out_h, out_w = canvas.shape

        data = np.asarray(local.data, dtype=np.int8).reshape(h, w)
        rows, cols = np.nonzero(data >= 0)  # skip unknown cells — don't clear team knowledge
        if rows.size == 0:
            return
        values = data[rows, cols]

        src_x = (cols.astype(np.float64) + 0.5) * res_src
        src_y = (rows.astype(np.float64) + 0.5) * res_src

        map_yaw = _yaw_from_quaternion(map_info.origin.orientation)
        map_cos, map_sin = math.cos(map_yaw), math.sin(map_yaw)
        off_cos, off_sin = math.cos(self._offset_yaw), math.sin(self._offset_yaw)

        # local cell → map frame (apply map origin)
        mx = map_info.origin.position.x + map_cos * src_x - map_sin * src_y
        my = map_info.origin.position.y + map_sin * src_x + map_cos * src_y
        # map frame → global frame (apply spawn offset)
        gx = self._offset_x + off_cos * mx - off_sin * my
        gy = self._offset_y + off_sin * mx + off_cos * my

        out_col = ((gx - origin_x) / res_dst).astype(np.int64)
        out_row = ((gy - origin_y) / res_dst).astype(np.int64)

        mask = (out_col >= 0) & (out_col < out_w) & (out_row >= 0) & (out_row < out_h)
        canvas[out_row[mask], out_col[mask]] = values[mask]


def main():
    rclpy.init()
    node = PerRobotMapCompositor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
