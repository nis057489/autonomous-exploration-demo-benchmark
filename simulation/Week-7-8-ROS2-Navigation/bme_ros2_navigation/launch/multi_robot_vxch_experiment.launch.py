"""
VXCH progressive streaming experiment launcher.

map_transport:=baseline
  team_map_fusion → /{robot}/global_map → DdilProxy → /{robot}/team_map_ddil
  PerRobotMapCompositor merges /{robot}/map (local) + /{robot}/team_map_ddil → /{robot}/nav_map

map_transport:=vxch
  team_map_fusion → /map → encoder → bands → DdilProxy → decoder → /{robot}/team_map_ddil
  PerRobotMapCompositor merges /{robot}/map (local) + /{robot}/team_map_ddil → /{robot}/nav_map

Nav2 and frontier exploration subscribe to /{robot}/nav_map in both modes.
team_map_fusion always publishes /{robot}/global_map with its default empty suffix —
the DdilProxy intercepts it so bypass is structurally impossible.

DDIL args (default 0 = no degradation):
  bandwidth_kbps   token-bucket rate limit on the map transport link
  loss_pct         per-message drop probability (0–100)
  delay_ms         additional forwarding latency
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_value(s):
    return str(s).strip().lower() in ("1", "true", "yes", "on")


def _create_all_actions(context):
    """
    Single OpaqueFunction that builds every action for the experiment.

    By resolving map_transport here and constructing IncludeLaunchDescription
    with plain string values (not LaunchConfiguration substitutions), we avoid
    the SetLaunchConfiguration timing issue where inner args might not be set
    before IncludeLaunchDescription evaluates its launch_arguments.
    """
    pkg_nav = get_package_share_directory("bme_ros2_navigation")
    pkg_rviz = get_package_share_directory("rviz_autonomous_exploration_benchmark")

    map_transport = LaunchConfiguration("map_transport").perform(context)
    haar_levels = int(LaunchConfiguration("haar_levels").perform(context))
    bandwidth_kbps = float(LaunchConfiguration("bandwidth_kbps").perform(context))
    loss_pct = float(LaunchConfiguration("loss_pct").perform(context))
    delay_ms = float(LaunchConfiguration("delay_ms").perform(context))
    rng_seed = int(LaunchConfiguration("rng_seed").perform(context))
    robot_startup_delay_s = float(LaunchConfiguration("robot_startup_delay_s").perform(context))
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = _bool_value(use_sim_time_str)

    world = LaunchConfiguration("world").perform(context)
    model = LaunchConfiguration("model").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    spacing = LaunchConfiguration("spacing").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)

    if map_transport not in ("baseline", "vxch"):
        raise ValueError(f"map_transport must be 'baseline' or 'vxch', got '{map_transport}'")

    is_vxch = map_transport == "vxch"
    vxch_mode_str = "true" if is_vxch else "false"

    robot_names = [f"robot{i + 1}" for i in range(num_robots)]
    vxch_base = "/vxch/map"
    vxch_ddil_base = "/vxch/map_ddil"
    total_bands = haar_levels + 1

    ddil_params = {
        "bandwidth_kbps": bandwidth_kbps,
        "loss_pct": loss_pct,
        "delay_ms": delay_ms,
        "rng_seed": rng_seed,
        "use_sim_time": use_sim_time,
    }

    actions = []

    # ── Navigation + SLAM + world ──────────────────────────────────────────────
    # global_map_suffix is intentionally omitted: team_map_fusion always publishes
    # /{robot}/global_map (default suffix ""). The DdilProxy intercepts that topic
    # and produces /{robot}/team_map_ddil, making bypass structurally impossible.
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, "launch", "multi_robot_navigation_with_slam.launch.py")
            ),
            launch_arguments={
                "world": world,
                "num_robots": str(num_robots),
                "model": model,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "spacing": spacing,
                "use_sim_time": use_sim_time_str,
                "rviz": rviz,
                "vxch_mode": vxch_mode_str,
                "seed": str(rng_seed),
            }.items(),
        )
    )

    # ── Frontier exploration ───────────────────────────────────────────────────
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_rviz, "launch", "multi_robot_frontier_explorer.launch.py")
            ),
            launch_arguments={
                "num_robots": str(num_robots),
                "use_sim_time": use_sim_time_str,
                "robot_startup_delay_s": str(robot_startup_delay_s),
            }.items(),
        )
    )

    # ── Baseline mode ──────────────────────────────────────────────────────────
    if not is_vxch:
        # team_map_fusion publishes /{robot}/global_map (default, no suffix).
        # DdilProxy intercepts it and produces /{robot}/team_map_ddil.
        # The compositor merges /{robot}/map (local) + /{robot}/team_map_ddil → /{robot}/nav_map.
        relay_topics = [
            f"/{name}/global_map /{name}/team_map_ddil nav_msgs/msg/OccupancyGrid reliable"
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
                    # bypass_topics omitted — C++ default is empty vector
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

        # One decoder per robot → publishes /{robot}/team_map_ddil
        # (the compositor merges this with /{robot}/map to produce /{robot}/nav_map)
        for name in robot_names:
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="vxch_occupancy_grid_node",
                    name=f"vxch_decoder_{name}",
                    output="screen",
                    parameters=[{
                        "input_base_topic": vxch_ddil_base,
                        "output_topic": f"/{name}/team_map_ddil",
                        "haar_levels": haar_levels,
                        "publish_rate_hz": 1.0,
                        "use_sim_time": use_sim_time,
                    }],
                )
            )

    return actions


def generate_launch_description():
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
            DeclareLaunchArgument(
                "rng_seed", default_value="-1",
                description="RNG seed for DDIL packet-loss reproducibility (-1 = random)"),
            DeclareLaunchArgument(
                "robot_startup_delay_s", default_value="0.0",
                description="Per-robot exploration startup stagger in seconds (robot N waits N * delay)"),

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

            OpaqueFunction(function=_create_all_actions),
        ]
    )
