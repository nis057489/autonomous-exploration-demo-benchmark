import math
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
# frontier-selection node with optional map-only robot-rank allocation.
_LITE_PARAM_KEYS = (
    "costmap_topic",
    "information_occ_threshold",
    "reservation_ttl_s",
    "reservation_radius_m",
    "min_gain_m2",
    "global_frame",
    "robot_base_frame",
    "min_frontier_size_cells",
    "min_frontier_distance_m",
    "occ_threshold",
    "path_occ_threshold",
    "selection_strategy",
    "branch_priority",
    "branch_left_region",
    "branch_right_region",
    "branch_split_x",
    "branch_unknown_tolerance",
    "frontier_assignment",
    "sensor_range_m",
    "gain_max_viewpoints",
    "gain_distance_weight",
    "gain_threshold_ratio",
    "gain_region_cap",
    "turn_penalty_m",
    "hysteresis_bonus_m",
    "replan_period_s",
    # Without these two in the allowlist the node falls back to its declared
    # defaults and the values in config.yaml are silently ignored -- this tuple
    # is a filter, not documentation.
    "goal_stuck_timeout_s",
    "goal_blacklist_duration_s",
    "goal_stuck_epsilon_m",
    # Preempt gating. Without these in the allowlist a config.yaml that tunes
    # them is silently ignored and the node defaults stand -- which is how the
    # 33-of-34-goals-preempted oscillation stayed invisible.
    "goal_preempt_distance_m",
    "goal_preempt_improvement_m",
    "goal_preempt_utility_ratio",
    "max_consecutive_preemptions",
    "wedge_detect_radius_m",
    "max_wedge_cycles",
    "goal_blacklist_radius_m",
    "navigate_to_pose_action_name",
    "frontier_marker_scale",
)

# Matches _PATH_COLORS in multi_robot_navigation_with_slam.launch.py (values in 0-255)
# so a robot's frontier candidates/selected-goal markers match its traveled-path color.
_MARKER_COLORS_255 = [
    (255, 85, 0),    # orange-red  (robot1)
    (0, 100, 255),   # blue        (robot2)
    (0, 200, 50),    # green       (robot3)
    (200, 0, 200),   # purple      (robot4+)
]


def _frontier_params(base_path, namespace, use_sim_time, color_255, robot_index=0, team_size=1,
                     overrides=None):
    data = _load_yaml(base_path)
    source = data.get("frontier_explorer", {}).get("ros__parameters", {})
    source = {**source, **(overrides or {})}
    params = {k: source[k] for k in _LITE_PARAM_KEYS if k in source}
    params["use_sim_time"] = use_sim_time
    params["robot_index"] = robot_index
    params["team_size"] = team_size
    params["costmap_topic"] = f"/{namespace}/global_costmap/costmap"
    # Occupancy-only experiment: no peer goal coordination channel. An empty
    # reservation_topic disables the whole path in the node, so the peer-topic
    # list is left unset -- passing [] here would be normalized to an untyped
    # empty tuple and the launch would be rejected before any node started.
    params["reservation_topic"] = ""
    params["information_map_topic"] = f"/{namespace}/nav_map"
    params["global_frame"] = "map"
    params["robot_base_frame"] = f"{namespace}/base_footprint"
    params["frontier_marker_topic"] = f"/{namespace}/explore/frontiers"
    params["frontier_marker_color_r"] = color_255[0] / 255.0
    params["frontier_marker_color_g"] = color_255[1] / 255.0
    params["frontier_marker_color_b"] = color_255[2] / 255.0
    return params


def _create_explorer_actions(context):
    package_share = get_package_share_directory("rviz_autonomous_exploration_benchmark")
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    params_file = _resolve_params_file(LaunchConfiguration("params_file").perform(context), package_share)
    override_file = LaunchConfiguration("params_override_file").perform(context)
    overrides = (_load_yaml(_resolve_params_file(override_file, package_share))
                 .get("frontier_explorer", {}).get("ros__parameters", {})
                 if override_file else {})
    use_sim_time = _bool_value(LaunchConfiguration("use_sim_time").perform(context))
    log_level = LaunchConfiguration("log_level").perform(context)
    # Seconds between one robot starting to explore and the next. robot1 always
    # starts immediately; robotN waits (N-1) * stagger. 0 disables it.
    start_stagger_s = float(LaunchConfiguration("start_stagger_s").perform(context))
    if not math.isfinite(start_stagger_s) or start_stagger_s < 0.0:
        raise RuntimeError("start_stagger_s must be >= 0")

    if num_robots < 1:
        raise RuntimeError("num_robots must be at least 1")

    actions = []
    for index in range(num_robots):
        namespace = f"robot{index + 1}"
        color_255 = _MARKER_COLORS_255[min(index, len(_MARKER_COLORS_255) - 1)]
        generated_params = _frontier_params(params_file, namespace, use_sim_time, color_255,
                                            robot_index=index, team_size=num_robots,
                                            overrides=overrides)
        generated_params["explore_start_delay_s"] = index * start_stagger_s
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
            DeclareLaunchArgument("params_override_file", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("start_stagger_s", default_value="0.0"),
            OpaqueFunction(function=_create_explorer_actions),
        ]
    )
