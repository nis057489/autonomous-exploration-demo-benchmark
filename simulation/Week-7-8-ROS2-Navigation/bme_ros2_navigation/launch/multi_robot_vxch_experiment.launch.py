"""
VXCH progressive streaming experiment launcher.

Network model: each robot has an independent point-to-point downlink from the base station
at bandwidth_kbps.  One ddil_proxy instance per robot enforces this — robots cannot starve
each other and there is no implicit multicast advantage for either method.

map_transport:=baseline
  base station: team_map_fusion → /{robot}/global_map
  per-robot link (ddil_proxy_{robot}): /{robot}/global_map → /{robot}/team_map_ddil
  robot: PerRobotMapCompositor merges /{robot}/map (local) + /{robot}/team_map_ddil → /{robot}/nav_map

map_transport:=vxch
  base station: team_map_fusion → /map → encoder → /vxch/map/band_0..N + manifest
  per-robot link (ddil_proxy_{robot}): bands + manifest → /vxch/map_ddil_{robot}/band_0..N + manifest
  robot: vxch_decoder_{robot} → /{robot}/team_map_ddil
         PerRobotMapCompositor merges /{robot}/map (local) + /{robot}/team_map_ddil → /{robot}/nav_map

Nav2 and frontier exploration subscribe to /{robot}/nav_map in both modes.

DDIL args (default 0 = no degradation):
  bandwidth_kbps   token-bucket rate limit per robot downlink
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
    tile_size_m = float(LaunchConfiguration("tile_size_m").perform(context))
    max_tiles_per_update = int(LaunchConfiguration("max_tiles_per_update").perform(context))
    min_resend_interval_s = float(LaunchConfiguration("min_resend_interval_s").perform(context))
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

    total_bands = haar_levels + 1

    # Base DDIL params shared across per-robot proxies.
    # rng_seed is offset per robot so packet-loss patterns are independent.
    ddil_params_base = {
        "bandwidth_kbps": bandwidth_kbps,
        "loss_pct": loss_pct,
        "delay_ms": delay_ms,
        "use_sim_time": use_sim_time,
    }

    def ddil_params_for(robot_index: int) -> dict:
        seed = (rng_seed + robot_index) if rng_seed >= 0 else -1
        return {**ddil_params_base, "rng_seed": seed}

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
                "spawn_positions_json": LaunchConfiguration("spawn_positions_json"),
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
        # One proxy per robot — each robot has its own independent downlink.
        # team_map_fusion publishes /{robot}/global_map; the proxy throttles it
        # to /{robot}/team_map_ddil; the compositor merges that with the local
        # /{robot}/map to produce /{robot}/nav_map.
        for i, name in enumerate(robot_names):
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="ddil_proxy_node",
                    name=f"ddil_proxy_{name}",
                    output="screen",
                    parameters=[{
                        **ddil_params_for(i),
                        "relay_topics": [
                            f"/{name}/global_map /{name}/team_map_ddil"
                            " nav_msgs/msg/OccupancyGrid reliable"
                        ],
                    }],
                )
            )

    # ── VXCH mode ──────────────────────────────────────────────────────────────
    else:
        # Encoder runs once at the base station: /map → /vxch/map/band_0..N + manifest.
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
                    "tile_size_m": tile_size_m,
                    "max_tiles_per_update": max_tiles_per_update,
                    "min_resend_interval_s": min_resend_interval_s,
                    "compression": "zstd",
                    "use_sim_time": use_sim_time,
                }],
            )
        )

        # One proxy + decoder per robot — each robot's downlink is independent.
        # The manifest travels through the same token bucket as the bands so it
        # counts against the bandwidth budget; it uses RELIABLE QoS so late-joining
        # decoders still receive it on the next encoder cycle.
        for i, name in enumerate(robot_names):
            robot_ddil_base = f"/vxch/map_ddil_{name}"
            relay_topics = [
                f"{vxch_base}/band_{k} {robot_ddil_base}/band_{k}"
                " voxelcodec_msgs/msg/VoxelChannel"
                for k in range(total_bands)
            ]
            relay_topics.append(
                f"{vxch_base}/manifest {robot_ddil_base}/manifest"
                " voxelcodec_msgs/msg/VoxelManifest reliable"
            )
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="ddil_proxy_node",
                    name=f"ddil_proxy_{name}",
                    output="screen",
                    parameters=[{
                        **ddil_params_for(i),
                        "relay_topics": relay_topics,
                    }],
                )
            )
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="vxch_occupancy_grid_node",
                    name=f"vxch_decoder_{name}",
                    output="screen",
                    parameters=[{
                        "input_base_topic": robot_ddil_base,
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
                "tile_size_m", default_value="2.0",
                description="Encode the map as independent tile_size_m x tile_size_m Haar "
                            "pyramids instead of one pyramid for the whole grid, so a still-"
                            "changing area (wherever a robot currently is) can't starve "
                            "another, already-settled area's detail bands"),
            DeclareLaunchArgument(
                "max_tiles_per_update", default_value="2",
                description="Max distinct tiles the encoder services per send tick (-1 = "
                            "uncapped). Capping this gives busy tiles more time to sit queued, "
                            "so repeated redirties coalesce into one send instead of each "
                            "triggering its own"),
            DeclareLaunchArgument(
                "min_resend_interval_s", default_value="2.0",
                description="Minimum seconds between two actual sends of the same tile, "
                            "regardless of how often it re-dirties (0 = no debounce). Was "
                            "8.0 originally; that made the DDIL map look stalled/slow to "
                            "converge on real hardware, so keep this well under it unless "
                            "you've confirmed the link -- not just this cadence -- is the "
                            "actual bottleneck"),
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
            DeclareLaunchArgument(
                "spawn_positions_json", default_value="[]",
                description="JSON array of {x,y,yaw} dicts, one per robot. "
                            "Empty array uses automatic grid/line offset."),

            OpaqueFunction(function=_create_all_actions),
        ]
    )
