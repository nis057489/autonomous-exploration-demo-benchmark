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


# The parameter keys lite_frontier_explorer's node actually declares -- it's a
# nearest-frontier-only node with no marker topics/autostart/control_service/
# team-awareness, and no peer coordination of its own.
_LITE_PARAM_KEYS = (
    "costmap_topic",
    "global_frame",
    "robot_base_frame",
    "min_frontier_size_cells",
    "min_frontier_distance_m",
    "occ_threshold",
    "replan_period_s",
    "navigate_to_pose_action_name",
)


def _frontier_params(base_path, namespace, use_sim_time):
    data = _load_yaml(base_path)
    source = data.get("frontier_explorer", {}).get("ros__parameters", {})
    params = {k: source[k] for k in _LITE_PARAM_KEYS if k in source}
    params["use_sim_time"] = use_sim_time
    params["costmap_topic"] = f"/{namespace}/global_costmap/costmap"
    params["global_frame"] = "map"
    params["robot_base_frame"] = f"{namespace}/base_footprint"
    return params


def _create_explorer_actions(context):
    package_share = get_package_share_directory("rviz_autonomous_exploration_benchmark")
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    params_file = _resolve_params_file(LaunchConfiguration("params_file").perform(context), package_share)
    use_sim_time = _bool_value(LaunchConfiguration("use_sim_time").perform(context))
    log_level = LaunchConfiguration("log_level").perform(context)

    if num_robots < 1:
        raise RuntimeError("num_robots must be at least 1")

    actions = []
    for index in range(num_robots):
        namespace = f"robot{index + 1}"
        generated_params = _frontier_params(params_file, namespace, use_sim_time)
        actions.append(
            Node(
                package="lite_frontier_explorer",
                executable="lite_frontier_explorer_node",
                name="lite_frontier_explorer",
                namespace=namespace,
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[generated_params],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_robots", default_value="2"),
            # Only read for the handful of keys lite_frontier_explorer actually
            # understands (see _LITE_PARAM_KEYS above).
            DeclareLaunchArgument(
                "params_file",
                default_value="config/lite_frontier_explorer/config_visit_once.yaml"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=_create_explorer_actions),
        ]
    )
