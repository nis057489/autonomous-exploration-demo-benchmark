#!/usr/bin/env python3
import math
from dataclasses import dataclass

import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class RobotMapState:
    name: str
    offset_x: float
    offset_y: float
    offset_yaw: float
    map_msg: OccupancyGrid | None = None


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class TeamMapFusion(Node):
    def __init__(self):
        super().__init__("team_map_fusion")

        self.declare_parameter("robot_names", ["robot1"])
        self.declare_parameter("offsets_x", [0.0])
        self.declare_parameter("offsets_y", [0.0])
        self.declare_parameter("offsets_yaw", [0.0])
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("max_cells", 12000000)
        # vxch_mode: suppress per-robot global_map publishing (VXCH decoder takes over)
        self.declare_parameter("vxch_mode", False)
        # global_map_suffix: rename per-robot topic, e.g. "_raw" → /{robot}/global_map_raw
        # Used in baseline+DDIL mode so DdilProxy can intercept before Nav2/frontier.
        self.declare_parameter("global_map_suffix", "")
        # map_topic_template: where to subscribe for each robot_names entry. Defaults to
        # the centralized-fusion layout (subscribe directly to each robot's own local
        # map); the distributed per-robot role overrides this to read the already-local,
        # already-DDIL'd copies of *other* robots' maps instead (e.g.
        # "/{namespace}/incoming/{name}/map"), since a single machine can't see every
        # robot's raw /{name}/map directly.
        self.declare_parameter("map_topic_template", "/{name}/map")
        self.declare_parameter("output_topic", "/map")
        self.declare_parameter("output_metadata_topic", "/map_metadata")
        # publish_per_robot_maps: the centralized-fusion role also republishes the same
        # merged result to /{robot}/global_map(_metadata) per robot, which the DDIL
        # proxies then intercept. The distributed per-robot role only wants a single
        # output_topic and doesn't need this.
        self.declare_parameter("publish_per_robot_maps", True)

        robot_names = list(self.get_parameter("robot_names").value)
        offsets_x = list(self.get_parameter("offsets_x").value)
        offsets_y = list(self.get_parameter("offsets_y").value)
        offsets_yaw = list(self.get_parameter("offsets_yaw").value)

        if not (
            len(robot_names)
            == len(offsets_x)
            == len(offsets_y)
            == len(offsets_yaw)
        ):
            raise ValueError(
                "robot_names, offsets_x, offsets_y, and offsets_yaw must have equal lengths"
            )

        self.global_frame = str(self.get_parameter("global_frame").value)
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.max_cells = int(self.get_parameter("max_cells").value)
        self.last_warn_time = 0.0
        self.vxch_mode = bool(self.get_parameter("vxch_mode").value)
        global_map_suffix = str(self.get_parameter("global_map_suffix").value)
        map_topic_template = str(self.get_parameter("map_topic_template").value)
        output_topic = str(self.get_parameter("output_topic").value)
        output_metadata_topic = str(self.get_parameter("output_metadata_topic").value)
        self.publish_per_robot_maps = bool(self.get_parameter("publish_per_robot_maps").value)

        self.map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.robot_states = {
            name: RobotMapState(name, float(offset_x), float(offset_y), float(offset_yaw))
            for name, offset_x, offset_y, offset_yaw in zip(
                robot_names, offsets_x, offsets_y, offsets_yaw
            )
        }

        self.team_map_pub = self.create_publisher(OccupancyGrid, output_topic, self.map_qos)
        self.team_metadata_pub = self.create_publisher(
            MapMetaData, output_metadata_topic, self.map_qos
        )
        if self.publish_per_robot_maps:
            self.robot_map_pubs = {
                name: self.create_publisher(
                    OccupancyGrid, f"/{name}/global_map{global_map_suffix}", self.map_qos
                )
                for name in robot_names
            }
            self.robot_metadata_pubs = {
                name: self.create_publisher(
                    MapMetaData, f"/{name}/global_map_metadata", self.map_qos
                )
                for name in robot_names
            }
        else:
            self.robot_map_pubs = {}
            self.robot_metadata_pubs = {}

        self.map_subscriptions = [
            self.create_subscription(
                OccupancyGrid,
                map_topic_template.format(name=name),
                lambda msg, robot_name=name: self._map_callback(robot_name, msg),
                self.map_qos,
            )
            for name in robot_names
        ]

        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_merged_map)
        self.get_logger().info(
            f"team_map_fusion publishing {output_topic} from: " + ", ".join(robot_names)
        )

    def _map_callback(self, robot_name, msg):
        self.robot_states[robot_name].map_msg = msg

    def _warn_throttled(self, message):
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        if now_seconds - self.last_warn_time >= 5.0:
            self.get_logger().warn(message)
            self.last_warn_time = now_seconds

    def _publish_merged_map(self):
        merged_map = self._build_merged_map()
        if merged_map is None:
            return

        self.team_map_pub.publish(merged_map)
        self.team_metadata_pub.publish(merged_map.info)
        if not self.vxch_mode:
            for name, publisher in self.robot_map_pubs.items():
                publisher.publish(merged_map)
                self.robot_metadata_pubs[name].publish(merged_map.info)

    def _build_merged_map(self):
        available_states = [
            state
            for state in self.robot_states.values()
            if state.map_msg is not None
            and state.map_msg.info.width > 0
            and state.map_msg.info.height > 0
        ]
        if not available_states:
            return None

        resolution = min(
            state.map_msg.info.resolution
            for state in available_states
            if state.map_msg.info.resolution > 0.0
        )

        bounds = self._compute_bounds(available_states)
        if bounds is None:
            return None

        min_x, min_y, max_x, max_y = bounds
        origin_x = math.floor(min_x / resolution) * resolution
        origin_y = math.floor(min_y / resolution) * resolution
        width = max(1, int(math.ceil((max_x - origin_x) / resolution)) + 1)
        height = max(1, int(math.ceil((max_y - origin_y) / resolution)) + 1)

        if width * height > self.max_cells:
            self._warn_throttled(
                f"merged map too large ({width}x{height}); waiting for saner bounds"
            )
            return None

        # int16 headroom: cell values only ever hold [-1, 100], but the
        # intermediate max/min priority combination in _merge_one_map briefly
        # compares against occupied-threshold constants -- plain int8 risks
        # silent wraparound if that ever changes, int16 costs nothing here.
        merged = np.full((height, width), -1, dtype=np.int16)
        for state in available_states:
            self._merge_one_map(state, resolution, origin_x, origin_y, width, height, merged)

        merged_map = OccupancyGrid()
        merged_map.header.stamp = self.get_clock().now().to_msg()
        merged_map.header.frame_id = self.global_frame
        merged_map.info.map_load_time = merged_map.header.stamp
        merged_map.info.resolution = resolution
        merged_map.info.width = width
        merged_map.info.height = height
        merged_map.info.origin.position.x = origin_x
        merged_map.info.origin.position.y = origin_y
        merged_map.info.origin.orientation.w = 1.0
        merged_map.data = merged.astype(np.int8).reshape(-1).tolist()
        return merged_map

    def _compute_bounds(self, states):
        min_x = math.inf
        min_y = math.inf
        max_x = -math.inf
        max_y = -math.inf

        for state in states:
            map_msg = state.map_msg
            map_info = map_msg.info
            map_width = map_info.width * map_info.resolution
            map_height = map_info.height * map_info.resolution
            corners = (
                (0.0, 0.0),
                (map_width, 0.0),
                (map_width, map_height),
                (0.0, map_height),
            )
            for local_x, local_y in corners:
                map_x, map_y = self._apply_pose(
                    map_info.origin.position.x,
                    map_info.origin.position.y,
                    yaw_from_quaternion(map_info.origin.orientation),
                    local_x,
                    local_y,
                )
                global_x, global_y = self._apply_pose(
                    state.offset_x,
                    state.offset_y,
                    state.offset_yaw,
                    map_x,
                    map_y,
                )
                min_x = min(min_x, global_x)
                min_y = min(min_y, global_y)
                max_x = max(max_x, global_x)
                max_y = max(max_y, global_y)

        if not all(math.isfinite(value) for value in (min_x, min_y, max_x, max_y)):
            return None
        return min_x, min_y, max_x, max_y

    def _merge_one_map(self, state, resolution, origin_x, origin_y, width, height, merged):
        # Backward/gather resampling: for every DESTINATION cell, inverse-
        # transform its world position into this robot's own map frame and
        # sample the nearest source cell there, then write into `merged` (a
        # numpy array, mutated in place). The previous version scattered
        # forward per SOURCE cell (source -> rounded destination index) --
        # structurally prone to leaving holes wherever the source/destination
        # resolution ratio isn't an exact integer, which in practice it never
        # is (two SLAM instances' float resolutions are essentially never
        # bit-identical, and `resolution` here is a min() over all robots').
        # Gather can't leave holes: every destination cell samples exactly
        # one source cell (or none, if outside that map's bounds), full stop.
        # It's also the geometrically correct direction anyway -- resolution
        # is a min() over all robots, so the destination is always at least
        # as fine as any single source, meaning multiple destination cells
        # legitimately share one source cell (upsampling), not the reverse.
        map_msg = state.map_msg
        map_info = map_msg.info
        map_origin_yaw = yaw_from_quaternion(map_info.origin.orientation)
        map_origin_cos = math.cos(map_origin_yaw)
        map_origin_sin = math.sin(map_origin_yaw)
        offset_cos = math.cos(state.offset_yaw)
        offset_sin = math.sin(state.offset_yaw)
        source_resolution = map_info.resolution

        source_data = np.array(map_msg.data, dtype=np.int16).reshape(
            map_info.height, map_info.width
        )

        # Destination cell centers, in the shared team/global frame.
        dest_row = np.arange(height).reshape(height, 1)
        dest_col = np.arange(width).reshape(1, width)
        global_x = origin_x + (dest_col + 0.5) * resolution
        global_y = origin_y + (dest_row + 0.5) * resolution

        # Undo this robot's offset transform (global -> its map's local frame).
        dx = global_x - state.offset_x
        dy = global_y - state.offset_y
        local_x = offset_cos * dx + offset_sin * dy
        local_y = -offset_sin * dx + offset_cos * dy

        # Undo the map's own origin transform (local frame -> source pixel space).
        ddx = local_x - map_info.origin.position.x
        ddy = local_y - map_info.origin.position.y
        source_x = map_origin_cos * ddx + map_origin_sin * ddy
        source_y = -map_origin_sin * ddx + map_origin_cos * ddy

        source_col = np.floor(source_x / source_resolution).astype(np.int64)
        source_row = np.floor(source_y / source_resolution).astype(np.int64)

        valid = (
            (source_row >= 0) & (source_row < map_info.height) &
            (source_col >= 0) & (source_col < map_info.width)
        )
        clipped_row = np.clip(source_row, 0, map_info.height - 1)
        clipped_col = np.clip(source_col, 0, map_info.width - 1)
        value = np.where(valid, source_data[clipped_row, clipped_col], -1)

        has_value = value >= 0
        existing_unknown = merged < 0
        either_occupied = (value >= 50) | (merged >= 50)
        combined = np.where(
            existing_unknown, value,
            np.where(either_occupied, np.maximum(merged, value), np.minimum(merged, value)),
        )
        merged[:] = np.where(has_value, combined, merged)

    @staticmethod
    def _apply_pose(origin_x, origin_y, yaw, point_x, point_y):
        yaw_cos = math.cos(yaw)
        yaw_sin = math.sin(yaw)
        return (
            origin_x + yaw_cos * point_x - yaw_sin * point_y,
            origin_y + yaw_sin * point_x + yaw_cos * point_y,
        )


def main():
    rclpy.init()
    node = TeamMapFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
