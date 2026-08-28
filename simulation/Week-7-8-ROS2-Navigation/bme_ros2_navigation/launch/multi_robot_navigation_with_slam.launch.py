import copy
import json
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, PushROSNamespace
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def _bool_value(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _write_yaml(directory, name, data):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, name)
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
    return path


def _iter_ros_parameters(value):
    if isinstance(value, dict):
        params = value.get("ros__parameters")
        if isinstance(params, dict):
            yield params
        for child in value.values():
            yield from _iter_ros_parameters(child)


def _node_params(data, node_name):
    return data.setdefault(node_name, {}).setdefault("ros__parameters", {})


def _costmap_params(data, costmap_name):
    return data.setdefault(costmap_name, {}).setdefault(costmap_name, {}).setdefault(
        "ros__parameters", {}
    )


# Mirrors multi_robot_vxch_experiment.launch.py's own `_netns_ip` exactly --
# both files must derive the same per-robot netns IP from the same robot
# index for rviz2's static-peer discovery (below) to actually reach the
# netns ddil_proxy_node/occupancy_grid_vxch_node instances live in.
def _netns_ip(robot_index):
    return f"10.77.0.{10 + robot_index + 1}"


def _robot_pose(index, count, x, y, z, yaw, spacing, world):
    if count == 1:
        return x, y, z, yaw

    if world == "corridor":
        local_x = (index - ((count - 1) / 2.0)) * spacing
        local_y = 0.0
    else:
        columns = max(1, math.ceil(math.sqrt(count)))
        rows = max(1, math.ceil(count / columns))
        row = index // columns
        column = index % columns
        local_x = (row - ((rows - 1) / 2.0)) * spacing
        local_y = (column - ((columns - 1) / 2.0)) * spacing

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x + (cos_yaw * local_x) - (sin_yaw * local_y),
        y + (sin_yaw * local_x) + (cos_yaw * local_y),
        z,
        yaw,
    )


# navigation.yaml's footprint/inflation_radius values are URDF-derived for one
# specific robot (mogi_bot: 0.4x0.2 collision box -- see the footprint comment in
# navigation.yaml itself) and were, until now, applied unchanged to every model.
# turtlebot3_waffle's actual collision geometry (turtlebot3_waffle.urdf: 0.266 x
# 0.266 box) is meaningfully smaller, so reusing mogi_bot's larger footprint made
# Nav2 believe the waffle was ~50% bigger than it really is in both dimensions --
# planner treats passable gaps as too narrow (windy, indirect paths hugging open
# space) and lite_frontier_explorer's reachability check (which just consults
# this same costmap, it has no footprint notion of its own) rejects/avoids
# frontiers a real-sized waffle could reach, leaving only nearby ones actually
# selectable.
#
# Originally used robot_radius (a circular approximation, matching ROBOTIS's own
# turtlebot3_navigation2 reference config for waffle) instead of an explicit
# footprint polygon -- but MPPI's ConstraintCritic/CostCritic (consider_footprint:
# true, see navigation.yaml) throw "Considering footprint in collision checking
# but no robot footprint provided in the costmap" against this installed nav2
# build when the costmap is configured via robot_radius alone, aborting Nav2
# bringup entirely (robots never move). An explicit polygon is the same
# mechanism already proven working for mogi_bot in this exact setup, so use that
# instead of chasing the robot_radius/critic interaction further. Derived
# directly from turtlebot3_waffle.urdf's actual collision box (0.266 x 0.266,
# centered at x=-0.064 relative to base_link, which coincides with
# base_footprint in x/y -- see base_joint's xyz offset, z-only).
_MOGI_BOT_FOOTPRINT = {
    "footprint": "[[0.22, 0.19], [0.22, -0.19], [-0.22, -0.19], [-0.22, 0.19]]",
    "footprint_padding": 0.01,
}
_TURTLEBOT3_WAFFLE_FOOTPRINT = {
    "footprint": "[[0.069, 0.133], [0.069, -0.133], [-0.197, -0.133], [-0.197, 0.133]]",
    "footprint_padding": 0.01,
}
# Scaled down from mogi_bot's local/global inflation_radius by the ratio of the
# two robots' effective radii, with local raised just enough (0.243 -> 0.26,
# not the 0.30 tried first) to clear the footprint's own circumscribed radius
# (~0.2516m, the max footprint_padding-padded vertex distance from base_link)
# and silence nav2's "inflation radius is smaller than the circumscribed
# radius" warning. 0.30 was tried first but proved too aggressive in practice:
# in the warehouse world specifically, it added enough extra obstacle buffer to
# push some genuinely passable (if narrow) aisles past the planner's tolerance,
# causing GridBased to fail outright ("Failed to create plan with tolerance of:
# 0.5") and the robot to retry/clear-costmap/fail in a loop -- a real planning
# failure, not just a smoothness issue, and worse than the warning it was meant
# to fix. Keep this as small above 0.2516 as clears the warning, not the
# largest value that still clears it.
_TURTLEBOT3_WAFFLE_INFLATION = {"local": 0.26, "global": 0.5}

# navigation.yaml's active general_goal_checker tolerance (xy 0.15m, yaw 0.12rad)
# is tight relative to turtlebot3_waffle's actual footprint/turning behavior --
# RegulatedPurePursuitController's rotate-to-heading finishing move can overshoot
# and re-correct repeatedly trying to land within that tight a window, wasting
# real time per frontier goal rather than accepting "close enough" and moving on
# to the next one. Reuses the file's own already-sketched (but previously
# inactive) "precise_goal_checker" values -- a misleading name, since 0.25/0.25
# is actually looser than the active 0.15/0.12, not stricter.
_TURTLEBOT3_WAFFLE_GOAL_TOLERANCE = {"xy_goal_tolerance": 0.25, "yaw_goal_tolerance": 0.25}


def _apply_goal_tolerance(controller_server_params: dict, model: str) -> None:
    if "turtlebot3_waffle" not in model:
        return
    controller_server_params.setdefault("general_goal_checker", {}).update(
        _TURTLEBOT3_WAFFLE_GOAL_TOLERANCE
    )


def _apply_footprint(costmap_params: dict, model: str, costmap_kind: str) -> None:
    """costmap_kind: "local" or "global" -- selects which of navigation.yaml's two
    (differently-tuned) inflation_radius values to scale for a non-default model.
    navigation.yaml's own existing value is left untouched for mogi_bot."""
    for key in ("footprint", "footprint_padding", "robot_radius"):
        costmap_params.pop(key, None)
    if "turtlebot3_waffle" in model:
        costmap_params.update(_TURTLEBOT3_WAFFLE_FOOTPRINT)
        costmap_params.setdefault("inflation_layer", {})["inflation_radius"] = (
            _TURTLEBOT3_WAFFLE_INFLATION[costmap_kind]
        )
    else:
        costmap_params.update(_MOGI_BOT_FOOTPRINT)


# "mppi" | "pure_pursuit" -- navigation.yaml's "FollowPath" key (what
# controller_plugins actually references) is already the pure_pursuit config, so
# a static/single-robot consumer of that file with zero further processing gets
# pure_pursuit by default, matching navigation_hw.yaml's real-hardware
# controller. Multi-robot runs can instead select "mppi", which overwrites
# "FollowPath" with navigation.yaml's FollowPathMPPI block, for A/B comparison.
_VALID_CONTROLLER_TYPES = ("mppi", "pure_pursuit")


def _apply_controller_type(controller_server_params: dict, controller_type: str) -> None:
    if controller_type == "mppi":
        controller_server_params["FollowPath"] = controller_server_params.pop("FollowPathMPPI")
    else:
        controller_server_params.pop("FollowPathMPPI", None)


def _navigation_params(base_path, output_dir, namespace, use_sim_time, model, controller_type):
    if controller_type not in _VALID_CONTROLLER_TYPES:
        raise ValueError(
            f"controller_type must be one of {_VALID_CONTROLLER_TYPES}, got '{controller_type}'")

    data = copy.deepcopy(_load_yaml(base_path))
    base_link_frame = f"{namespace}/base_link"
    base_footprint_frame = f"{namespace}/base_footprint"
    odom_frame = f"{namespace}/odom"

    for params in _iter_ros_parameters(data):
        params["use_sim_time"] = use_sim_time

    bt_navigator = _node_params(data, "bt_navigator")
    bt_navigator["global_frame"] = "map"
    bt_navigator["robot_base_frame"] = base_link_frame
    bt_navigator["odom_topic"] = f"/{namespace}/odom"

    controller_server_params = _node_params(data, "controller_server")
    _apply_controller_type(controller_server_params, controller_type)
    _apply_goal_tolerance(controller_server_params, model)

    local_costmap = _costmap_params(data, "local_costmap")
    local_costmap["global_frame"] = odom_frame
    local_costmap["robot_base_frame"] = base_link_frame
    local_costmap.setdefault("voxel_layer", {}).setdefault("scan", {})["topic"] = (
        f"/{namespace}/scan"
    )
    _apply_footprint(local_costmap, model, "local")

    global_costmap = _costmap_params(data, "global_costmap")
    global_costmap["global_frame"] = "map"
    global_costmap["robot_base_frame"] = base_link_frame
    global_costmap.setdefault("obstacle_layer", {}).setdefault("scan", {})["topic"] = (
        f"/{namespace}/scan"
    )
    global_costmap.setdefault("static_layer", {})["map_topic"] = f"/{namespace}/nav_map"
    _apply_footprint(global_costmap, model, "global")

    behavior_server = _node_params(data, "behavior_server")
    behavior_server["local_frame"] = odom_frame
    behavior_server["global_frame"] = "map"
    behavior_server["robot_base_frame"] = base_link_frame

    velocity_smoother = _node_params(data, "velocity_smoother")
    velocity_smoother["odom_topic"] = f"/{namespace}/odom"

    collision_monitor = _node_params(data, "collision_monitor")
    collision_monitor["base_frame_id"] = base_footprint_frame
    collision_monitor["odom_frame_id"] = odom_frame
    collision_monitor.setdefault("FootprintApproach", {})["footprint_topic"] = (
        f"/{namespace}/local_costmap/published_footprint"
    )
    collision_monitor.setdefault("scan", {})["topic"] = f"/{namespace}/scan"

    docking_server = _node_params(data, "docking_server")
    docking_server["base_frame"] = base_link_frame
    docking_server["fixed_frame"] = odom_frame

    return _write_yaml(output_dir, f"{namespace}_navigation.yaml", {namespace: data})


def _slam_params(base_path, output_dir, namespace, use_sim_time):
    data = copy.deepcopy(_load_yaml(base_path))
    params = _node_params(data, "slam_toolbox")
    params["use_sim_time"] = use_sim_time
    params["odom_frame"] = f"{namespace}/odom"
    params["map_frame"] = f"{namespace}/map"
    params["base_frame"] = f"{namespace}/base_footprint"
    params["scan_topic"] = f"/{namespace}/scan"
    # 0.0 does NOT mean "publish as fast as possible" -- slam_toolbox's own reference
    # config is explicit that 0 means the periodic map->odom broadcast thread never
    # starts at all ("if 0 never publishes odometry"). That left map->odom
    # permanently unbroadcast: slam_toolbox still built /{namespace}/map correctly
    # (scan-matching is a separate code path from tf broadcasting), but "map" and
    # "{namespace}/odom" were two disconnected TF trees the whole time -- see
    # nav2's "Tf has two or more unconnected trees" errors.
    params["transform_publish_period"] = 0.02
    return params


def _ekf_params(base_path, output_dir, namespace, use_sim_time):
    data = copy.deepcopy(_load_yaml(base_path))
    params = _node_params(data, "ekf_filter_node")
    params["use_sim_time"] = use_sim_time
    params["map_frame"] = "map"
    params["odom_frame"] = f"{namespace}/odom"
    params["base_link_frame"] = f"{namespace}/base_footprint"
    params["world_frame"] = f"{namespace}/odom"
    params["odom0"] = f"/{namespace}/odom"
    params["imu0"] = f"/{namespace}/imu"
    return params


def _bridge_config(output_dir, namespace):
    data = [
        {
            "ros_topic_name": f"/{namespace}/joint_states",
            "gz_topic_name": f"/{namespace}/joint_states",
            "ros_type_name": "sensor_msgs/msg/JointState",
            "gz_type_name": "gz.msgs.Model",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": f"/{namespace}/odom",
            "gz_topic_name": f"/{namespace}/odom",
            "ros_type_name": "nav_msgs/msg/Odometry",
            "gz_type_name": "gz.msgs.Odometry",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": f"/{namespace}/cmd_vel",
            "gz_topic_name": f"/{namespace}/cmd_vel",
            "ros_type_name": "geometry_msgs/msg/Twist",
            "gz_type_name": "gz.msgs.Twist",
            "direction": "ROS_TO_GZ",
        },
        {
            "ros_topic_name": f"/{namespace}/camera/camera_info",
            "gz_topic_name": f"/{namespace}/camera/camera_info",
            "ros_type_name": "sensor_msgs/msg/CameraInfo",
            "gz_type_name": "gz.msgs.CameraInfo",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": f"/{namespace}/scan",
            "gz_topic_name": f"/{namespace}/scan",
            "ros_type_name": "sensor_msgs/msg/LaserScan",
            "gz_type_name": "gz.msgs.LaserScan",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": f"/{namespace}/scan/points",
            "gz_topic_name": f"/{namespace}/scan/points",
            "ros_type_name": "sensor_msgs/msg/PointCloud2",
            "gz_type_name": "gz.msgs.PointCloudPacked",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": f"/{namespace}/imu",
            "gz_topic_name": f"/{namespace}/imu",
            "ros_type_name": "sensor_msgs/msg/Imu",
            "gz_type_name": "gz.msgs.IMU",
            "direction": "GZ_TO_ROS",
        },
    ]
    return _write_yaml(output_dir, f"{namespace}_gz_bridge.yaml", data)


def _clock_bridge_config(output_dir):
    data = [
        {
            "ros_topic_name": "clock",
            "gz_topic_name": "clock",
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS",
        }
    ]
    return _write_yaml(output_dir, "clock_gz_bridge.yaml", data)


def _autostart_lifecycle_node(node):
    return [
        node,
        TimerAction(
            period=1.0,
            actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
    ]


def _static_map_transform(namespace, name, child_frame, x, y, z, yaw, use_sim_time):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        namespace=namespace,
        arguments=[
            "--x",
            f"{x:.3f}",
            "--y",
            f"{y:.3f}",
            "--z",
            f"{z:.3f}",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            f"{yaw:.4f}",
            "--frame-id",
            "map",
            "--child-frame-id",
            child_frame,
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/tf_static", "tf_static")],
    )


def _topic_qos(topic):
    return {
        "Depth": 5,
        "Durability Policy": "Volatile",
        "History Policy": "Keep Last",
        "Reliability Policy": "Reliable",
        "Value": topic,
    }


_PATH_COLORS = [
    "255; 85; 0",    # orange-red  (robot1)
    "0; 100; 255",   # blue        (robot2)
    "0; 200; 50",    # green       (robot3)
    "200; 0; 200",   # purple      (robot4+)
]


def _path_display(name, topic, color):
    return {
        "Alpha": 1.0,
        "Buffer Length": 1,
        "Class": "rviz_default_plugins/Path",
        "Color": color,
        "Enabled": True,
        "Head Diameter": 0.03,
        "Head Length": 0.03,
        "Length": 0.3,
        "Line Style": "Lines",
        "Line Width": 0.03,
        "Name": name,
        "Offset": {"X": 0, "Y": 0, "Z": 0},
        "Pose Color": "255; 85; 255",
        "Pose Style": "None",
        "Radius": 0.03,
        "Shaft Diameter": 0.01,
        "Shaft Length": 0.23,
        "Topic": {
            "Depth": 5,
            "Durability Policy": "Transient Local",
            "History Policy": "Keep Last",
            "Reliability Policy": "Reliable",
            "Value": topic,
        },
        "Value": True,
    }


def _map_display(name, topic, alpha=0.55):
    return {
        "Alpha": alpha,
        "Class": "rviz_default_plugins/Map",
        "Color Scheme": "map",
        "Draw Behind": False,
        "Enabled": True,
        "Name": name,
        "Topic": _topic_qos(topic),
        "Update Topic": f"{topic}_updates",
        "Use Timestamp": False,
        "Value": True,
    }


def _rviz_config(output_dir, namespaces):
    displays = [
        {
            "Alpha": 0.5,
            "Cell Size": 1,
            "Class": "rviz_default_plugins/Grid",
            "Color": "160; 160; 164",
            "Enabled": True,
            "Name": "Grid",
            "Plane": "XY",
            "Plane Cell Count": 30,
            "Value": True,
        },
        {
            "Class": "rviz_default_plugins/TF",
            "Enabled": True,
            "Frame Timeout": 15,
            "Name": "TF",
            "Show Arrows": False,
            "Show Axes": False,
            "Show Names": False,
            "Update Interval": 0,
            "Value": True,
        },
    ]
    for idx, namespace in enumerate(namespaces):
        color = _PATH_COLORS[min(idx, len(_PATH_COLORS) - 1)]
        displays.extend(
            [
                {
                    "Alpha": 1,
                    "Class": "rviz_default_plugins/RobotModel",
                    "Collision Enabled": False,
                    "Description Source": "Topic",
                    "Description Topic": _topic_qos(f"/{namespace}/robot_description"),
                    "Enabled": True,
                    "Name": f"{namespace} RobotModel",
                    "TF Prefix": namespace,
                    "Update Interval": 0,
                    "Value": True,
                    "Visual Enabled": True,
                },
                {
                    "Alpha": 1,
                    "Class": "rviz_default_plugins/LaserScan",
                    "Color Transformer": "Intensity",
                    "Decay Time": 0,
                    "Enabled": True,
                    "Name": f"{namespace} LaserScan",
                    "Position Transformer": "XYZ",
                    "Size (Pixels)": 3,
                    "Size (m)": 0.01,
                    "Style": "Flat Squares",
                    "Topic": {
                        "Depth": 5,
                        "Durability Policy": "Volatile",
                        "Filter size": 10,
                        "History Policy": "Keep Last",
                        "Reliability Policy": "Best Effort",
                        "Value": f"/{namespace}/scan",
                    },
                    "Use Fixed Frame": True,
                    "Use rainbow": True,
                    "Value": True,
                },
                {**_map_display(f"{namespace} Local SLAM Map", f"/{namespace}/map", 0.35), "Enabled": False},
                _map_display(f"{namespace} Nav Map (composite)", f"/{namespace}/nav_map", 0.6),
                {
                    **_map_display(f"{namespace} Team Map DDIL", f"/{namespace}/team_map_ddil", 0.4),
                    "Enabled": False,
                },
                {
                    "Class": "rviz_default_plugins/MarkerArray",
                    "Enabled": True,
                    "Name": f"{namespace} Frontiers",
                    "Topic": _topic_qos(f"/{namespace}/explore/frontiers"),
                    "Value": True,
                },
                _path_display(
                    f"{namespace} Traversed Path",
                    f"/{namespace}/explore/traversed_path",
                    color,
                ),
            ]
        )

    config = {
        "Panels": [
            {"Class": "rviz_common/Displays", "Name": "Displays"},
            {"Class": "rviz_common/Time", "Name": "Time"},
            {
                "Class": "rviz_autonomous_exploration_benchmark/ExplorationControlPanel",
                "Name": "Exploration Control",
            },
            {
                "Class": "voxelcodec_ros/NetworkStatsPanel",
                "Name": "VXCH Network Stats",
            },
        ],
        "Visualization Manager": {
            "Class": "",
            "Displays": displays,
            "Enabled": True,
            "Global Options": {
                "Background Color": "48; 48; 48",
                "Fixed Frame": "map",
                "Frame Rate": 30,
            },
            "Name": "root",
            "Tools": [
                {"Class": "rviz_default_plugins/Interact", "Hide Inactive Objects": True},
                {"Class": "rviz_default_plugins/MoveCamera"},
                {"Class": "rviz_default_plugins/Select"},
                {"Class": "rviz_default_plugins/FocusCamera"},
                {"Class": "rviz_default_plugins/Measure"},
                {
                    "Class": "rviz_default_plugins/SetGoal",
                    "Topic": _topic_qos(f"/{namespaces[0]}/goal_pose"),
                },
            ],
            "Transformation": {"Current": {"Class": "rviz_default_plugins/TF"}},
            "Value": True,
            "Views": {
                "Current": {
                    "Class": "rviz_default_plugins/Orbit",
                    "Distance": 15,
                    "Focal Point": {"X": 0, "Y": 0, "Z": 0},
                    "Name": "Current View",
                    "Near Clip Distance": 0.01,
                    "Pitch": 1.0,
                    "Target Frame": "<Fixed Frame>",
                    "Value": "Orbit (rviz)",
                    "Yaw": 0.8,
                }
            },
        },
        "Window Geometry": {
            "Displays": {"collapsed": False},
            "Height": 1000,
            "Hide Left Dock": False,
            "Hide Right Dock": True,
            "Width": 1600,
        },
    }
    return _write_yaml(output_dir, "multi_robot_navigation.rviz", config)


def _spawn_robot_actions(namespace, robot_x, robot_y, robot_z, robot_yaw, use_sim_time, attempts_left=6):
    """ros_gz_sim's `create` is a one-shot process that calls the
    `/world/<world>/create` service and exits -- it does not itself wait for
    that service (or for robot_description to be published on its `-topic`)
    to become available, and nothing here previously retried it. Under
    load, with world.launch.py's gz sim server and this robot's
    robot_state_publisher starting concurrently with `create` rather than
    strictly before it, `create` can lose that race, fail once, and exit --
    silently leaving this robot un-spawned for the rest of the run, which is
    what "gazebo sometimes doesn't spawn one of the robots" actually was.
    Retries the same spawn (fresh `create` process, 2s apart) up to
    attempts_left times on nonzero exit, and logs loudly -- instead of
    silently -- if every attempt fails.

    A plain exit-code retry isn't enough on its own, though: `create -topic
    robot_description` blocks forever waiting for a message on that ROS
    topic (see its own "Waiting messages on topic [robot_description]" log
    line), and if it loses the DDS discovery race against this robot's own
    robot_state_publisher (started in the same batch of actions, no
    ordering between them), it just hangs -- no nonzero exit ever happens,
    so the exit-triggered retry above never fires either, and this robot
    silently never joins for the rest of the run. `prefix="timeout 12"`
    forces that hang itself to become a nonzero exit after 12s, which is
    what actually feeds the retry loop below in that case."""
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"{namespace}_create",
        namespace=namespace,
        prefix="timeout 12",
        arguments=[
            "-name",
            namespace,
            "-topic",
            "robot_description",
            "-x",
            f"{robot_x:.3f}",
            "-y",
            f"{robot_y:.3f}",
            "-z",
            f"{robot_z:.3f}",
            "-Y",
            f"{robot_yaw:.4f}",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    def _on_exit(event, _context):
        if event.returncode == 0:
            return None
        if attempts_left <= 1:
            return [
                LogInfo(
                    msg=f"{namespace}_create: giving up spawning '{namespace}' after "
                        f"repeated failures (exit code {event.returncode}) -- the gz "
                        "world service likely never became available in time."
                )
            ]
        return [
            LogInfo(
                msg=f"{namespace}_create: spawn failed (exit code {event.returncode}), "
                    f"retrying in 2s ({attempts_left - 1} attempt(s) left)..."
            ),
            TimerAction(
                period=2.0,
                actions=_spawn_robot_actions(
                    namespace, robot_x, robot_y, robot_z, robot_yaw, use_sim_time, attempts_left - 1
                ),
            ),
        ]

    return [
        spawn_node,
        RegisterEventHandler(OnProcessExit(target_action=spawn_node, on_exit=_on_exit)),
    ]


def _create_multi_robot_actions(context):
    pkg_bme_ros2_navigation = get_package_share_directory("bme_ros2_navigation")
    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )

    model = LaunchConfiguration("model").perform(context)
    world = LaunchConfiguration("world").perform(context)
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    use_sim_time_text = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = _bool_value(use_sim_time_text)
    is_tc = LaunchConfiguration("impairment_mode").perform(context) == "tc"
    controller_type = LaunchConfiguration("controller_type").perform(context)
    x = float(LaunchConfiguration("x").perform(context))
    y = float(LaunchConfiguration("y").perform(context))
    z = float(LaunchConfiguration("z").perform(context))
    yaw = float(LaunchConfiguration("yaw").perform(context))
    spacing = float(LaunchConfiguration("spacing").perform(context))
    spawn_positions_raw = LaunchConfiguration("spawn_positions_json").perform(context)
    spawn_positions = json.loads(spawn_positions_raw) if spawn_positions_raw else []

    if num_robots < 1:
        raise RuntimeError("num_robots must be at least 1")

    output_dir = tempfile.mkdtemp(prefix="bme_multi_robot_")
    namespaces = [f"robot{index + 1}" for index in range(num_robots)]
    robot_offsets_x = []
    robot_offsets_y = []
    robot_offsets_yaw = []
    urdf_file_path = os.path.join(pkg_bme_ros2_navigation, "urdf", model)
    navigation_params_path = os.path.join(pkg_bme_ros2_navigation, "config", "navigation.yaml")
    slam_params_path = os.path.join(pkg_bme_ros2_navigation, "config", "slam_toolbox_mapping.yaml")
    ekf_params_path = os.path.join(pkg_bme_ros2_navigation, "config", "ekf.yaml")

    actions = []

    # Every robot's spawn/bridges/EKF used to start at t=0 for all robots at
    # once, then every robot's SLAM at a single shared t=5s, then every
    # robot's nav2 at a single shared t=12s -- three simultaneous herds of
    # N robots' worth of process launches, DDS discovery, and CPU load
    # landing in the same instant. On a loaded machine that's exactly the
    # kind of pile-up that starves a control loop's real-time deadline
    # (ekf_filter_node "Failed to meet update rate", controller_server
    # "Controller patience exceeded") for whichever robot's threads happen
    # to lose the scheduling race, intermittently and differently each run.
    # ROBOT_STAGGER_S offsets each robot's whole startup sequence (spawn ->
    # SLAM -> nav2) by index, so the 3 robots start in a deterministic,
    # staged sequence instead of 3-wide simultaneous bursts -- same total
    # work, spread out instead of piled up.
    ROBOT_STAGGER_S = 4.0
    SLAM_BASE_S = 5.0
    NAV2_BASE_S = 12.0

    clock_bridge_path = _clock_bridge_config(output_dir)
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_gz_bridge",
            arguments=["--ros-args", "-p", f"config_file:={clock_bridge_path}"],
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    for index, namespace in enumerate(namespaces):
        if spawn_positions:
            pos = spawn_positions[index % len(spawn_positions)]
            robot_x = float(pos["x"])
            robot_y = float(pos["y"])
            robot_z = z
            robot_yaw = float(pos.get("yaw", yaw))
        else:
            robot_x, robot_y, robot_z, robot_yaw = _robot_pose(
                index, num_robots, x, y, z, yaw, spacing, world
            )
        robot_offsets_x.append(robot_x)
        robot_offsets_y.append(robot_y)
        robot_offsets_yaw.append(robot_yaw)

        actions.append(
            Node(
                package="bme_ros2_navigation",
                executable="per_robot_map_compositor.py",
                name=f"per_robot_map_compositor_{namespace}",
                output="screen",
                parameters=[{
                    "robot_name": namespace,
                    "offset_x": robot_x,
                    "offset_y": robot_y,
                    "offset_yaw": robot_yaw,
                    "publish_rate_hz": 2.0,
                    "use_sim_time": use_sim_time,
                }],
            )
        )

        navigation_params = _navigation_params(
            navigation_params_path, output_dir, namespace, use_sim_time, model, controller_type
        )
        slam_params = _slam_params(slam_params_path, output_dir, namespace, use_sim_time)
        ekf_params = _ekf_params(ekf_params_path, output_dir, namespace, use_sim_time)
        bridge_params = _bridge_config(output_dir, namespace)

        robot_description = Command(
            [
                "xacro",
                " ",
                urdf_file_path,
                f" cmd_vel_topic:=/{namespace}/cmd_vel",
                f" odom_topic:=/{namespace}/odom",
                f" joint_states_topic:=/{namespace}/joint_states",
                f" camera_image_topic:=/{namespace}/camera/image",
                f" camera_info_topic:=/{namespace}/camera/camera_info",
                f" imu_topic:=/{namespace}/imu",
                f" scan_topic:=/{namespace}/scan",
                f" navsat_topic:=/{namespace}/navsat",
                f" frame_prefix:={namespace}/",
            ]
        )

        actions.append(TimerAction(period=index * ROBOT_STAGGER_S, actions=[
                Node(
                    package="topic_tools",
                    executable="relay",
                    name=f"{namespace}_tf_relay",
                    arguments=[f"/{namespace}/tf", "/tf"],
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                Node(
                    package="topic_tools",
                    executable="relay",
                    name=f"{namespace}_tf_static_relay",
                    arguments=[f"/{namespace}/tf_static", "/tf_static"],
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                # Only bridge shared "map" -> "{namespace}/map"; slam_toolbox already
                # publishes "{namespace}/map" -> "{namespace}/odom" continuously
                # (map_frame/odom_frame in slam_toolbox_mapping.yaml), so a second,
                # separate "map" -> "{namespace}/odom" static transform here would give
                # "{namespace}/odom" two competing parents in the TF tree.
                _static_map_transform(
                    namespace,
                    f"{namespace}_map_to_slam_map",
                    f"{namespace}/map",
                    robot_x,
                    robot_y,
                    0.0,
                    robot_yaw,
                    use_sim_time,
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    namespace=namespace,
                    output="screen",
                    parameters=[
                        {
                            "robot_description": ParameterValue(robot_description, value_type=str),
                            "frame_prefix": f"{namespace}/",
                            "use_sim_time": use_sim_time,
                        }
                    ],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
                *_spawn_robot_actions(namespace, robot_x, robot_y, robot_z, robot_yaw, use_sim_time),
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=f"{namespace}_gz_bridge",
                    arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                Node(
                    package="ros_gz_image",
                    executable="image_bridge",
                    name=f"{namespace}_image_bridge",
                    arguments=[f"/{namespace}/camera/image"],
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "camera.image.compressed.jpeg_quality": 75,
                        }
                    ],
                ),
                Node(
                    package="topic_tools",
                    executable="relay",
                    name="relay_camera_info",
                    namespace=namespace,
                    arguments=["camera/camera_info", "camera/image/camera_info"],
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                Node(
                    package="robot_localization",
                    executable="ekf_node",
                    name="ekf_filter_node",
                    namespace=namespace,
                    output="screen",
                    parameters=[ekf_params, {"use_sim_time": use_sim_time}],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
        ]))

        slam_node = LifecycleNode(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace=namespace,
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("/map", "map"),
                ("/map_metadata", "map_metadata"),
                ("/map_updates", "map_updates"),
            ],
        )
        actions.append(TimerAction(
            period=SLAM_BASE_S + index * ROBOT_STAGGER_S,
            actions=_autostart_lifecycle_node(slam_node),
        ))
        actions.append(TimerAction(
            period=NAV2_BASE_S + index * ROBOT_STAGGER_S,
            actions=[
                GroupAction(
                    actions=[
                        PushROSNamespace(namespace),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(nav2_navigation_launch_path),
                            launch_arguments={
                                "namespace": "",
                                "use_namespace": "False",
                                "use_sim_time": use_sim_time_text,
                                "params_file": navigation_params,
                                "autostart": "True",
                            }.items(),
                        ),
                    ]
                )
            ],
        ))

    for namespace in namespaces:
        actions.append(
            Node(
                package="rviz_autonomous_exploration_benchmark",
                executable="frontier_path_tracker.py",
                name=f"frontier_path_tracker_{namespace}",
                output="screen",
                parameters=[{
                    "global_frame": "map",
                    "robot_base_frame": f"{namespace}/base_footprint",
                    "path_topic": f"/{namespace}/explore/traversed_path",
                    "package_topics": [
                        f"lite_frontier_explorer:/{namespace}/explore/traversed_path",
                    ],
                    "default_package": "lite_frontier_explorer",
                    "active_package_topic": f"/{namespace}/explore/path_tracker/active_package",
                    "initial_pose_topic": f"/{namespace}/explore/path_tracker/initial_pose",
                    "reset_topic": "/explore/reset_traveled_path",
                    "use_sim_time": use_sim_time,
                }],
            )
        )

    # No centralized team_map_fusion here -- matches hw_namespaced_stack.launch.py,
    # where each robot fuses its own view of the team locally from whatever peer
    # maps its own DDIL links delivered. See multi_robot_vxch_experiment.launch.py's
    # per-robot, per-peer relay + team_map_fusion_{robot} instances.
    # (SLAM/nav2 startup per robot is scheduled inline above, staggered by
    # ROBOT_STAGGER_S per robot instead of one shared TimerAction firing for
    # every robot at once -- see the comment on ROBOT_STAGGER_S.)

    rviz_config_path = _rviz_config(output_dir, namespaces)

    # In tc mode, ddil_proxy_node/occupancy_grid_vxch_node run confined to a
    # robot's own netns with ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -- they
    # only become visible to a participant that they themselves manage to
    # discover via their own (necessarily limited-range, since the main
    # netns can easily have 50+ other participants) static-peer scan of
    # MAIN_NETNS_IP. rviz2 is started last of everything (this 14s
    # TimerAction), so it reliably lands outside whatever scan window those
    # already-running confined participants catch, and its own default
    # (SUBNET) automatic discovery never reaches into their LOCALHOST-only
    # netns either -- net result, RViz panels/displays reading
    # netns-confined topics stay empty. Fixed by having rviz2 itself
    # actively probe each (small, 2-3 participant) robot netns as an
    # additional static peer -- that scan is cheap enough to actually
    # succeed, unlike the reverse direction. ROS_AUTOMATIC_DISCOVERY_RANGE
    # is deliberately left at its default here so rviz2 still discovers
    # every ordinary main-netns node (nav2, SLAM, etc.) exactly as before.
    #
    # Must stay after every robot's staged nav2 start (see ROBOT_STAGGER_S
    # above) to keep the "started last of everything" invariant this relies
    # on -- with per-robot staggering, the last robot's nav2 now starts at
    # NAV2_BASE_S + (num_robots - 1) * ROBOT_STAGGER_S, later than the old
    # fixed 14s once there are more than ~2 robots.
    rviz_period = NAV2_BASE_S + num_robots * ROBOT_STAGGER_S + 2.0
    rviz_additional_env = {}
    if is_tc:
        rviz_additional_env["ROS_STATIC_PEERS"] = ";".join(
            _netns_ip(i) for i in range(num_robots)
        )

    actions.append(
        TimerAction(
            period=rviz_period,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2_multi_robot",
                    output="screen",
                    arguments=["-d", rviz_config_path],
                    condition=IfCondition(LaunchConfiguration("rviz")),
                    parameters=[{"use_sim_time": use_sim_time}],
                    additional_env=rviz_additional_env,
                )
            ],
        )
    )

    return actions


def generate_launch_description():
    pkg_bme_ros2_navigation = get_package_share_directory("bme_ros2_navigation")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_ros2_navigation, "launch", "world.launch.py"),
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "seed": LaunchConfiguration("seed"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="bookstore"),
            DeclareLaunchArgument("num_robots", default_value="2"),
            DeclareLaunchArgument("model", default_value="mogi_bot.urdf"),
            DeclareLaunchArgument(
                "controller_type", default_value="pure_pursuit",
                description="'pure_pursuit' (default) = nav2_regulated_pure_pursuit_"
                            "controller, mirroring navigation_hw.yaml's real-hardware "
                            "tuning exactly, so sim and hardware behavior are comparable. "
                            "'mppi' = nav2_mppi_controller, sim's original controller -- "
                            "kept available for A/B comparison against pure_pursuit."),
            DeclareLaunchArgument("x", default_value="2.5"),
            DeclareLaunchArgument("y", default_value="1.5"),
            DeclareLaunchArgument("z", default_value="0.05"),
            DeclareLaunchArgument("yaw", default_value="-1.5707"),
            DeclareLaunchArgument("spacing", default_value="0.8"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "impairment_mode", default_value="sim",
                description="'sim' | 'tc' -- must match multi_robot_vxch_experiment."
                            "launch.py's own impairment_mode so rviz2's static-peer "
                            "discovery (see rviz2 Node below) is only added when it's "
                            "actually needed and points at the right netns IPs."),
            DeclareLaunchArgument("seed", default_value="-1",
                description="Gazebo physics RNG seed (-1 = non-deterministic)"),
            DeclareLaunchArgument(
                "spawn_positions_json", default_value="[]",
                description="JSON array of {x,y,yaw} dicts, one per robot. "
                            "Empty array uses automatic grid/line offset."),
            world_launch,
            OpaqueFunction(function=_create_multi_robot_actions),
        ]
    )
