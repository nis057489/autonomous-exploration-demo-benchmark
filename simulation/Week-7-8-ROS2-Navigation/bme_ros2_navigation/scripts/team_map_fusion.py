#!/usr/bin/env python3
import math
from dataclasses import dataclass

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

        self.team_map_pub = self.create_publisher(OccupancyGrid, "/map", self.map_qos)
        self.team_metadata_pub = self.create_publisher(
            MapMetaData, "/map_metadata", self.map_qos
        )
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

        self.map_subscriptions = [
            self.create_subscription(
                OccupancyGrid,
                f"/{name}/map",
                lambda msg, robot_name=name: self._map_callback(robot_name, msg),
                self.map_qos,
            )
            for name in robot_names
        ]

        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_merged_map)
        self.get_logger().info(
            "team_map_fusion publishing /map and per-robot /global_map topics from: "
            + ", ".join(robot_names)
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

        merged_data = [-1] * (width * height)
        for state in available_states:
            self._merge_one_map(
                state,
                resolution,
                origin_x,
                origin_y,
                width,
                height,
                merged_data,
            )

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
        merged_map.data = merged_data
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

    def _merge_one_map(
        self,
        state,
        resolution,
        origin_x,
        origin_y,
        width,
        height,
        merged_data,
    ):
        map_msg = state.map_msg
        map_info = map_msg.info
        map_origin_yaw = yaw_from_quaternion(map_info.origin.orientation)
        map_origin_cos = math.cos(map_origin_yaw)
        map_origin_sin = math.sin(map_origin_yaw)
        offset_cos = math.cos(state.offset_yaw)
        offset_sin = math.sin(state.offset_yaw)
        source_resolution = map_info.resolution

        for row in range(map_info.height):
            source_y = (row + 0.5) * source_resolution
            for column in range(map_info.width):
                source_index = row * map_info.width + column
                value = map_msg.data[source_index]
                if value < 0:
                    continue

                source_x = (column + 0.5) * source_resolution
                local_x = (
                    map_info.origin.position.x
                    + map_origin_cos * source_x
                    - map_origin_sin * source_y
                )
                local_y = (
                    map_info.origin.position.y
                    + map_origin_sin * source_x
                    + map_origin_cos * source_y
                )
                global_x = state.offset_x + offset_cos * local_x - offset_sin * local_y
                global_y = state.offset_y + offset_sin * local_x + offset_cos * local_y

                output_column = int((global_x - origin_x) / resolution)
                output_row = int((global_y - origin_y) / resolution)
                if not (0 <= output_column < width and 0 <= output_row < height):
                    continue

                output_index = output_row * width + output_column
                existing_value = merged_data[output_index]
                if existing_value < 0:
                    merged_data[output_index] = int(value)
                elif value >= 50 or existing_value >= 50:
                    merged_data[output_index] = max(int(existing_value), int(value))
                else:
                    merged_data[output_index] = min(int(existing_value), int(value))

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
