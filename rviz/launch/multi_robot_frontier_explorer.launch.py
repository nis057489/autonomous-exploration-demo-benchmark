import copy
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_value(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _workspace_root(package_share):
    path = Path(package_share)
    for _ in range(4):
        path = path.parent
    return path


def _resolve_params_file(raw_value, package_share):
    candidate = Path(raw_value)
    if candidate.is_absolute() and candidate.is_file():
        return str(candidate)

    cwd_candidate = (Path.cwd() / candidate).resolve()
    if cwd_candidate.is_file():
        return str(cwd_candidate)

    root_candidate = (_workspace_root(package_share) / candidate).resolve()
    if root_candidate.is_file():
        return str(root_candidate)

    share_candidate = (Path(package_share) / "config" / candidate.name).resolve()
    if share_candidate.is_file():
        return str(share_candidate)

    raise RuntimeError(f"frontier params file does not exist: {raw_value}")


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


# Matches _PATH_COLORS in multi_robot_navigation_with_slam.launch.py (values in 0-255).
_PATH_COLORS_255 = [
    (255, 85, 0),    # orange-red  (robot1)
    (0, 100, 255),   # blue        (robot2)
    (0, 200, 50),    # green       (robot3)
    (200, 0, 200),   # purple      (robot4+)
]


def _frontier_params(base_path, namespace, use_sim_time, autostart, control_service, color_255, startup_delay_s):
    data = copy.deepcopy(_load_yaml(base_path))
    params = data.setdefault("frontier_explorer", {}).setdefault("ros__parameters", {})
    params["use_sim_time"] = use_sim_time
    params["autostart"] = autostart
    params["startup_delay_s"] = startup_delay_s
    params["control_service_enabled"] = control_service
    params["map_topic"] = f"/{namespace}/nav_map"
    params["costmap_topic"] = f"/{namespace}/global_costmap/costmap"
    params["local_costmap_topic"] = f"/{namespace}/local_costmap/costmap"
    params["navigate_to_pose_action_name"] = "navigate_to_pose"
    params["global_frame"] = "map"
    params["robot_base_frame"] = f"{namespace}/base_footprint"
    params["frontier_marker_topic"] = f"/{namespace}/explore/frontiers"
    params["selected_frontier_topic"] = f"/{namespace}/explore/selected_frontier"
    params["optimized_map_topic"] = f"/{namespace}/explore/optimized_map"
    params["frontier_marker_color_r"] = color_255[0] / 255.0
    params["frontier_marker_color_g"] = color_255[1] / 255.0
    params["frontier_marker_color_b"] = color_255[2] / 255.0
    return params


def _create_explorer_actions(context):
    package_share = get_package_share_directory("rviz_autonomous_exploration_benchmark")
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    params_file = _resolve_params_file(LaunchConfiguration("params_file").perform(context), package_share)
    use_sim_time = _bool_value(LaunchConfiguration("use_sim_time").perform(context))
    autostart = _bool_value(LaunchConfiguration("autostart").perform(context))
    control_service = _bool_value(LaunchConfiguration("control_service_enabled").perform(context))
    log_level = LaunchConfiguration("log_level").perform(context)
    robot_startup_delay_s = float(LaunchConfiguration("robot_startup_delay_s").perform(context))

    if num_robots < 1:
        raise RuntimeError("num_robots must be at least 1")

    actions = []
    for index in range(num_robots):
        namespace = f"robot{index + 1}"
        color_255 = _PATH_COLORS_255[min(index, len(_PATH_COLORS_255) - 1)]
        generated_params = _frontier_params(
            params_file,
            namespace,
            use_sim_time,
            autostart,
            control_service,
            color_255,
            startup_delay_s=index * robot_startup_delay_s,
        )
        actions.append(
            Node(
                package="frontier_exploration_ros2",
                executable="frontier_explorer",
                name="frontier_explorer",
                namespace=namespace,
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[generated_params, {"use_sim_time": use_sim_time}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_robots", default_value="2"),
            DeclareLaunchArgument("params_file", default_value="config/frontier_exploration_ros2/config.yaml"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("control_service_enabled", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "robot_startup_delay_s", default_value="0.0",
                description="Per-robot startup stagger: robot N waits N * robot_startup_delay_s seconds before starting exploration"),
            OpaqueFunction(function=_create_explorer_actions),
        ]
    )
