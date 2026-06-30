"""
Per-robot namespaced hardware stack.

Called by launch_real_hardware.sh when ROBOT_ID is set.
Handles SLAM, Nav2, per_robot_map_compositor, frontier_path_tracker,
and frontier_explorer — all under /{namespace}/.

Assumes turtlebot3_bringup is already running on this Pi (started by
launch_real_hardware.sh before this file is invoked, or --local-bringup
is set and it is started here) with ROS_NAMESPACE already exported.

TF frame IDs on the bringup are patched via a generated params file so
that odom/base frames carry the robot's namespace prefix.
"""

import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,

    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── YAML helpers ──────────────────────────────────────────────────────────────

def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _write_yaml(directory, name, data):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, name)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
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
    return (
        data.setdefault(costmap_name, {})
            .setdefault(costmap_name, {})
            .setdefault("ros__parameters", {})
    )


# ── Per-component config patchers ─────────────────────────────────────────────

def _patch_tb3_params(tb3_model, namespace, output_dir):
    """Merge with default bringup params, override frame IDs for this namespace."""
    try:
        default_path = os.path.join(
            get_package_share_directory("turtlebot3_bringup"), "param", f"{tb3_model}.yaml"
        )
        data = copy.deepcopy(_load_yaml(default_path)) if os.path.isfile(default_path) else {}
    except Exception:
        data = {}

    p = _node_params(data, "turtlebot3_node")
    p["odom_frame"] = f"{namespace}/odom"
    p["base_frame"] = f"{namespace}/base_footprint"

    rsp = _node_params(data, "robot_state_publisher")
    rsp["frame_prefix"] = f"{namespace}/"

    return _write_yaml(output_dir, f"{namespace}_tb3.yaml", data)


def _patch_slam_params(base_path, namespace, output_dir):
    data = copy.deepcopy(_load_yaml(base_path))
    p = _node_params(data, "slam_toolbox")
    p["odom_frame"] = f"{namespace}/odom"
    p["map_frame"] = f"{namespace}/map"
    p["base_frame"] = f"{namespace}/base_footprint"
    p["scan_topic"] = f"/{namespace}/scan"
    p["transform_publish_period"] = 0.0
    p["use_sim_time"] = False
    return _write_yaml(output_dir, f"{namespace}_slam.yaml", data)


def _patch_nav_params(base_path, namespace, output_dir):
    data = copy.deepcopy(_load_yaml(base_path))
    base_link = f"{namespace}/base_link"
    base_fp = f"{namespace}/base_footprint"
    odom = f"{namespace}/odom"

    for params in _iter_ros_parameters(data):
        params["use_sim_time"] = False

    bt = _node_params(data, "bt_navigator")
    bt["global_frame"] = "map"
    bt["robot_base_frame"] = base_link
    bt["odom_topic"] = f"/{namespace}/odom"

    lc = _costmap_params(data, "local_costmap")
    lc["global_frame"] = odom
    lc["robot_base_frame"] = base_link
    lc.setdefault("obstacle_layer", {}).setdefault("scan", {})["topic"] = f"/{namespace}/scan"

    gc = _costmap_params(data, "global_costmap")
    gc["global_frame"] = "map"
    gc["robot_base_frame"] = base_link
    gc.setdefault("obstacle_layer", {}).setdefault("scan", {})["topic"] = f"/{namespace}/scan"
    gc.setdefault("static_layer", {})["map_topic"] = f"/{namespace}/nav_map"

    bs = _node_params(data, "behavior_server")
    bs["local_frame"] = odom
    bs["global_frame"] = "map"
    bs["robot_base_frame"] = base_link

    _node_params(data, "velocity_smoother")["odom_topic"] = f"/{namespace}/odom"

    cm = _node_params(data, "collision_monitor")
    cm["base_frame_id"] = base_fp
    cm["odom_frame_id"] = odom
    cm.setdefault("FootprintApproach", {})["footprint_topic"] = (
        f"/{namespace}/local_costmap/published_footprint"
    )
    cm.setdefault("scan", {})["topic"] = f"/{namespace}/scan"

    ds = _node_params(data, "docking_server")
    ds["base_frame"] = base_link
    ds["fixed_frame"] = odom

    return _write_yaml(output_dir, f"{namespace}_navigation.yaml", data)


def _patch_explore_params(base_path, namespace, output_dir):
    data = copy.deepcopy(_load_yaml(base_path))
    p = data.setdefault("frontier_explorer", {}).setdefault("ros__parameters", {})
    p["use_sim_time"] = False
    p["map_topic"] = f"/{namespace}/nav_map"
    p["costmap_topic"] = f"/{namespace}/global_costmap/costmap"
    p["local_costmap_topic"] = f"/{namespace}/local_costmap/costmap"
    p["global_frame"] = "map"
    p["robot_base_frame"] = f"{namespace}/base_footprint"
    p["frontier_marker_topic"] = f"/{namespace}/explore/frontiers"
    p["selected_frontier_topic"] = f"/{namespace}/explore/selected_frontier"
    p["optimized_map_topic"] = f"/{namespace}/explore/optimized_map"
    return _write_yaml(output_dir, f"{namespace}_explore.yaml", data)


# ── Main action builder ───────────────────────────────────────────────────────

def _create_actions(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    nav_params_file = LaunchConfiguration("nav_params_file").perform(context)
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    explore_config = LaunchConfiguration("explore_config").perform(context)
    local_bringup = LaunchConfiguration("local_bringup").perform(context).lower() in (
        "1", "true", "yes", "on"
    )
    tb3_model = LaunchConfiguration("tb3_model").perform(context)
    spawn_x = float(LaunchConfiguration("spawn_x").perform(context))
    spawn_y = float(LaunchConfiguration("spawn_y").perform(context))
    spawn_yaw = float(LaunchConfiguration("spawn_yaw").perform(context))

    output_dir = tempfile.mkdtemp(prefix=f"bme_hw_{namespace}_")

    slam_cfg = _patch_slam_params(slam_params_file, namespace, output_dir)
    nav_cfg = _patch_nav_params(nav_params_file, namespace, output_dir)
    explore_cfg = _patch_explore_params(explore_config, namespace, output_dir)

    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"
    )
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"
    )
    tb3_launch = os.path.join(
        get_package_share_directory("turtlebot3_bringup"), "launch", "robot.launch.py"
    )

    # Delay offsets depend on whether bringup starts here or was already running.
    bringup_delay = 5.0 if local_bringup else 0.0
    slam_delay = bringup_delay + 3.0
    nav2_delay = slam_delay + 10.0
    # frontier_explorer uses startup_delay_s internally, no extra timer needed.

    actions = []

    # ── 1. turtlebot3_bringup (optional, namespaced so its TF goes to /{ns}/tf)
    if local_bringup:
        tb3_cfg = _patch_tb3_params(tb3_model, namespace, output_dir)
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(tb3_launch),
                launch_arguments={"param_file": tb3_cfg}.items(),
            )
        )

    # ── 2. TF frame renamer: /{namespace}/tf → /tf with namespaced frame IDs ─
    # Replaces simple topic relay — also renames bare frames (odom, base_*)
    # to {namespace}/odom, {namespace}/base_* so multiple robots can share /tf.
    actions.append(
        Node(
            package="bme_ros2_navigation_py",
            executable="tf_frame_renamer",
            name=f"tf_frame_renamer_{namespace}",
            output="screen",
            parameters=[{"namespace": namespace, "use_sim_time": False}],
        )
    )

    # ── 3. Static transforms: map → {namespace}/odom and map → {namespace}/map
    for child_frame, tf_name in [
        (f"{namespace}/odom", f"{namespace}_map_to_odom"),
        (f"{namespace}/map", f"{namespace}_map_to_slam_map"),
    ]:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=tf_name,
                arguments=[
                    "--x", str(spawn_x),
                    "--y", str(spawn_y),
                    "--z", "0.0",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", str(spawn_yaw),
                    "--frame-id", "map",
                    "--child-frame-id", child_frame,
                ],
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        )

    # ── 4. Per-robot map compositor ───────────────────────────────────────────
    actions.append(
        Node(
            package="bme_ros2_navigation",
            executable="per_robot_map_compositor.py",
            name=f"per_robot_map_compositor_{namespace}",
            output="screen",
            parameters=[{
                "robot_name": namespace,
                "offset_x": spawn_x,
                "offset_y": spawn_y,
                "offset_yaw": spawn_yaw,
                "publish_rate_hz": 2.0,
                "use_sim_time": False,
            }],
        )
    )

    # ── 5. Frontier path tracker ──────────────────────────────────────────────
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
                    f"frontier_exploration_ros2:/{namespace}/explore/traversed_path",
                ],
                "default_package": "frontier_exploration_ros2",
                "active_package_topic": f"/{namespace}/explore/path_tracker/active_package",
                "initial_pose_topic": f"/{namespace}/explore/path_tracker/initial_pose",
                "reset_topic": "/explore/reset_traveled_path",
                "use_sim_time": False,
            }],
        )
    )

    # ── 6. SLAM ───────────────────────────────────────────────────────────────
    actions.append(
        TimerAction(
            period=slam_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch),
                    launch_arguments={
                        "use_sim_time": "false",
                        "slam_params_file": slam_cfg,
                    }.items(),
                )
            ],
        )
    )

    # ── 7. Nav2 ───────────────────────────────────────────────────────────────
    actions.append(
        TimerAction(
            period=nav2_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        "namespace": namespace,
                        "use_namespace": "True",
                        "use_sim_time": "false",
                        "params_file": nav_cfg,
                        "autostart": "True",
                    }.items(),
                )
            ],
        )
    )

    # ── 8. Frontier explorer (startup_delay_s inside config handles Nav2 wait)
    actions.append(
        Node(
            package="frontier_exploration_ros2",
            executable="frontier_explorer",
            name="frontier_explorer",
            output="screen",
            parameters=[explore_cfg, {"use_sim_time": False}],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace",
            description="ROS namespace for this robot, e.g. robot1"),
        DeclareLaunchArgument("nav_params_file",
            description="Path to navigation_hw.yaml"),
        DeclareLaunchArgument("slam_params_file",
            description="Path to slam_toolbox_mapping_hw.yaml"),
        DeclareLaunchArgument("explore_config",
            description="Path to frontier_exploration_ros2 config.yaml"),
        DeclareLaunchArgument("local_bringup", default_value="false",
            description="true = also launch turtlebot3_bringup here"),
        DeclareLaunchArgument("tb3_model", default_value="waffle_pi"),
        DeclareLaunchArgument("spawn_x", default_value="0.0",
            description="Robot starting X in the shared map frame"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        OpaqueFunction(function=_create_actions),
    ])
