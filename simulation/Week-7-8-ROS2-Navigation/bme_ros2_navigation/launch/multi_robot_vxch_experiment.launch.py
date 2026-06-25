"""
VXCH progressive streaming experiment launcher.

map_transport:=baseline   — team_map_fusion publishes /{robot}/global_map_raw,
                            DdilProxy relays it to /{robot}/global_map with DDIL params.

map_transport:=vxch       — team_map_fusion publishes /map only (vxch_mode=true),
                            OccupancyGridVxchNode encodes it into per-band VXCH topics,
                            DdilProxy applies DDIL to the bands,
                            VxchOccupancyGridNode decodes and publishes /{robot}/global_map.

Nav2 and frontier exploration always consume /{robot}/global_map — unchanged in both modes.

DDIL args (default 0 = no degradation, useful for smoke-testing):
  bandwidth_kbps   token-bucket rate limit on the map transport link
  loss_pct         per-message drop probability (0–100)
  delay_ms         additional forwarding latency
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_value(s):
    return str(s).strip().lower() in ("1", "true", "yes", "on")


def _create_experiment_actions(context):
    map_transport = LaunchConfiguration("map_transport").perform(context)
    haar_levels = int(LaunchConfiguration("haar_levels").perform(context))
    bandwidth_kbps = float(LaunchConfiguration("bandwidth_kbps").perform(context))
    loss_pct = float(LaunchConfiguration("loss_pct").perform(context))
    delay_ms = float(LaunchConfiguration("delay_ms").perform(context))
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    use_sim_time = _bool_value(LaunchConfiguration("use_sim_time").perform(context))

    if map_transport not in ("baseline", "vxch"):
        raise ValueError(f"map_transport must be 'baseline' or 'vxch', got '{map_transport}'")

    is_vxch = map_transport == "vxch"
    robot_names = [f"robot{i + 1}" for i in range(num_robots)]
    vxch_base = "/vxch/map"
    vxch_ddil_base = "/vxch/map_ddil"
    total_bands = haar_levels + 1

    ddil_params = {
        "bandwidth_kbps": bandwidth_kbps,
        "loss_pct": loss_pct,
        "delay_ms": delay_ms,
        "use_sim_time": use_sim_time,
    }

    actions = []

    # ── Baseline mode ──────────────────────────────────────────────────────────
    if not is_vxch:
        # team_map_fusion publishes /{robot}/global_map_raw (via global_map_suffix)
        # DdilProxy relays each to /{robot}/global_map
        relay_topics = [
            f"/{name}/global_map_raw /{name}/global_map nav_msgs/msg/OccupancyGrid"
            for name in robot_names
        ]
        actions.append(
            Node(
                package="voxelcodec_ros",
                executable="ddil_proxy_node",
                name="ddil_proxy_baseline",
                output="screen",
                parameters=[{
                    **ddil_params,
                    "relay_topics": relay_topics,
                    "bypass_topics": [],
                }],
            )
        )

    # ── VXCH mode ──────────────────────────────────────────────────────────────
    else:
        # Encoder: /map → VXCH bands
        actions.append(
            Node(
                package="voxelcodec_ros",
                executable="occupancy_grid_vxch_node",
                name="occupancy_grid_vxch_encoder",
                output="screen",
                parameters=[{
                    "input_topic": "/map",
                    "output_base_topic": vxch_base,
                    "haar_levels": haar_levels,
                    "compression": "zstd",
                    "use_sim_time": use_sim_time,
                }],
            )
        )

        # DdilProxy: throttle bands, bypass manifest
        relay_topics = [
            f"{vxch_base}/band_{k} {vxch_ddil_base}/band_{k} voxelcodec_msgs/msg/VoxelChannel"
            for k in range(total_bands)
        ]
        bypass_topics = [
            f"{vxch_base}/manifest {vxch_ddil_base}/manifest voxelcodec_msgs/msg/VoxelManifest"
        ]
        actions.append(
            Node(
                package="voxelcodec_ros",
                executable="ddil_proxy_node",
                name="ddil_proxy_vxch",
                output="screen",
                parameters=[{
                    **ddil_params,
                    "relay_topics": relay_topics,
                    "bypass_topics": bypass_topics,
                }],
            )
        )

        # One decoder per robot → publishes /{robot}/global_map
        for name in robot_names:
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="vxch_occupancy_grid_node",
                    name=f"vxch_decoder_{name}",
                    output="screen",
                    parameters=[{
                        "input_base_topic": vxch_ddil_base,
                        "output_topic": f"/{name}/global_map",
                        "haar_levels": haar_levels,
                        "publish_rate_hz": 1.0,
                        "use_sim_time": use_sim_time,
                    }],
                )
            )

    return actions


def generate_launch_description():
    pkg_nav = get_package_share_directory("bme_ros2_navigation")
    pkg_rviz = get_package_share_directory("rviz_autonomous_exploration_benchmark")

    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "multi_robot_navigation_with_slam.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "num_robots": LaunchConfiguration("num_robots"),
            "model": LaunchConfiguration("model"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
            "spacing": LaunchConfiguration("spacing"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz": LaunchConfiguration("rviz"),
            # VXCH-specific params forwarded to team_map_fusion
            "vxch_mode": LaunchConfiguration("vxch_mode_inner"),
            "global_map_suffix": LaunchConfiguration("global_map_suffix_inner"),
        }.items(),
    )

    frontier_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rviz, "launch", "multi_robot_frontier_explorer.launch.py")
        ),
        launch_arguments={
            "num_robots": LaunchConfiguration("num_robots"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription(
        [
            # ── Experiment args ────────────────────────────────────────────
            DeclareLaunchArgument(
                "map_transport", default_value="baseline",
                description="'baseline' or 'vxch'"),
            DeclareLaunchArgument(
                "haar_levels", default_value="4",
                description="Haar wavelet levels (total bands = levels+1)"),
            DeclareLaunchArgument(
                "bandwidth_kbps", default_value="0",
                description="Token-bucket bandwidth limit for map transport (0 = unlimited)"),
            DeclareLaunchArgument(
                "loss_pct", default_value="0.0",
                description="Per-message drop probability 0–100"),
            DeclareLaunchArgument(
                "delay_ms", default_value="0",
                description="Additional forwarding delay in ms"),

            # ── Forwarded simulation args ──────────────────────────────────
            DeclareLaunchArgument("world", default_value="bookstore"),
            DeclareLaunchArgument("num_robots", default_value="2"),
            DeclareLaunchArgument("model", default_value="mogi_bot.urdf"),
            DeclareLaunchArgument("x", default_value="2.5"),
            DeclareLaunchArgument("y", default_value="1.5"),
            DeclareLaunchArgument("z", default_value="0.05"),
            DeclareLaunchArgument("yaw", default_value="-1.5707"),
            DeclareLaunchArgument("spacing", default_value="0.8"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("rviz", default_value="true"),

            # Internal substitutions resolved in OpaqueFunction
            OpaqueFunction(
                function=_resolve_inner_args,
            ),

            multi_robot_launch,
            frontier_launch,
            OpaqueFunction(function=_create_experiment_actions),
        ]
    )


def _resolve_inner_args(context):
    """Compute vxch_mode_inner and global_map_suffix_inner from map_transport."""
    map_transport = LaunchConfiguration("map_transport").perform(context)
    is_vxch = map_transport == "vxch"

    from launch.actions import SetLaunchConfiguration
    return [
        SetLaunchConfiguration("vxch_mode_inner", "true" if is_vxch else "false"),
        SetLaunchConfiguration(
            "global_map_suffix_inner", "" if is_vxch else "_raw"),
    ]
