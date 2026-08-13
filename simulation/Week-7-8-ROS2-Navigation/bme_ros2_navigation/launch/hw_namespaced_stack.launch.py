"""
Per-robot namespaced hardware stack.

Called by launch_real_hardware.sh when ROBOT_ID is set.
Handles SLAM, Nav2, per_robot_map_compositor, frontier_path_tracker,
and frontier_explorer — all under /{namespace}/.

Assumes turtlebot3_bringup is already running on this Pi (started by
launch_real_hardware.sh before this file is invoked, or --local-bringup
is set and it is started here) with ROS_NAMESPACE already exported.

TF frame IDs are prefixed natively by the vendor bringup: turtlebot3_ros
takes a "namespace" parameter (odometry.cpp, sensors/imu.cpp,
sensors/joint_state.cpp) and turtlebot3_state_publisher.launch.py bakes
the namespace into the URDF via xacro, so odom/base/sensor frames already
carry the robot's namespace prefix by the time they reach /tf and
/tf_static. What still needs handling is that tf2_ros::TransformBroadcaster
and StaticTransformBroadcaster always publish to the absolute, unnamespaced
/tf and /tf_static topics regardless of node namespace -- so every node in
this stack that needs its own robot's transforms (Nav2, slam_toolbox,
frontier_path_tracker, frontier_explorer) gets /tf and /tf_static remapped
to /{namespace}/tf(_static) via SetRemap/remappings=. A pair of one-way
topic_tools relay nodes additionally republishes each robot's namespaced
/tf(_static) outward onto the shared global /tf(_static), purely so a
base-station RViz can view all robots merged in one window and so bagged
runs capture a single merged TF stream -- nothing in this stack reads that
merged topic back in.
"""

import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


# ── YAML helpers ──────────────────────────────────────────────────────────────

def _deep_merge(base, override):
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value


def _load_yaml(path):
    """Load YAML, merging duplicate top-level keys (turtlebot3's param files have two /**:)."""
    class MergingLoader(yaml.SafeLoader):
        pass

    def construct_mapping(loader, node, deep=False):
        loader.flatten_mapping(node)
        pairs = loader.construct_pairs(node, deep=deep)
        result = {}
        for key, value in pairs:
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                _deep_merge(result[key], value)
            else:
                result[key] = value
        return result

    MergingLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping,
    )
    with open(path, "r", encoding="utf-8") as f:
        return yaml.load(f, Loader=MergingLoader)  # noqa: S506


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


# ── Per-component config patchers ──────────────────────────────────────────────

def _patch_slam_params(base_path, namespace, output_dir):
    data = copy.deepcopy(_load_yaml(base_path))
    p = _node_params(data, "slam_toolbox")
    p["odom_frame"] = f"{namespace}/odom"
    p["map_frame"] = f"{namespace}/map"
    p["base_frame"] = f"{namespace}/base_footprint"
    p["scan_topic"] = f"/{namespace}/scan"
    # 0.0 doesn't mean "publish every update" -- slam_toolbox's transform
    # publish thread returns immediately and never runs at all when this is 0,
    # so map -> odom is never published. See slam_toolbox_localization.yaml's
    # own "#if 0 never publishes odometry" note.
    p["transform_publish_period"] = 0.02
    p["use_sim_time"] = False
    # slam_toolbox's online_async_launch.py passes this file straight through
    # as a --params-file with no namespace-aware rewriting (unlike
    # nav2_bringup's navigation_launch.py, which wraps params via
    # RewrittenYaml). A bare "slam_toolbox:" top-level key does not bind to
    # the node once it's pushed into the /{namespace} namespace -- ROS 2
    # silently ignores the whole file and slam_toolbox falls back to its
    # hardcoded defaults (confirmed via a live get_parameters query: every
    # override here was being dropped). The key must be the node's
    # fully-qualified name instead.
    data[f"/{namespace}/slam_toolbox"] = data.pop("slam_toolbox")
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
    lc.setdefault("obstacle_layer", {}).setdefault(
        "scan", {})["topic"] = f"/{namespace}/scan"

    gc = _costmap_params(data, "global_costmap")
    gc["global_frame"] = "map"
    gc["robot_base_frame"] = base_link
    gc.setdefault("obstacle_layer", {}).setdefault(
        "scan", {})["topic"] = f"/{namespace}/scan"
    gc.setdefault("static_layer", {})["map_topic"] = f"/{namespace}/nav_map"

    bs = _node_params(data, "behavior_server")
    bs["local_frame"] = odom
    bs["global_frame"] = "map"
    bs["robot_base_frame"] = base_link

    _node_params(data, "velocity_smoother")[
        "odom_topic"] = f"/{namespace}/odom"

    cm = _node_params(data, "collision_monitor")
    cm["base_frame_id"] = base_fp
    cm["odom_frame_id"] = odom
    cm.setdefault("FootprintApproach", {})["footprint_topic"] = (
        f"/{namespace}/local_costmap/published_footprint"
    )
    cm.setdefault("scan", {})["topic"] = f"/{namespace}/scan"

    return _write_yaml(output_dir, f"{namespace}_navigation.yaml", data)


#  Matches _PATH_COLORS in multi_robot_navigation_with_slam.launch.py and
#  _PATH_COLORS_255 in rviz/launch/multi_robot_frontier_explorer.launch.py
#  (values in 0-255).
_PATH_COLORS_255 = [
    (255, 85, 0),    # orange-red  (robot1)
    (0, 100, 255),   # blue        (robot2)
    (0, 200, 50),    # green       (robot3)
    (200, 0, 200),   # purple      (robot4+)
]


def _robot_index(namespace):
    digits = "".join(ch for ch in namespace if ch.isdigit())
    return (int(digits) - 1) if digits else 0


def _patch_explore_params(base_path, namespace, output_dir, peers=()):
    data = copy.deepcopy(_load_yaml(base_path))
    p = data.setdefault("frontier_explorer", {}).setdefault(
        "ros__parameters", {})
    p["use_sim_time"] = False
    p["map_topic"] = f"/{namespace}/nav_map"
    p["costmap_topic"] = f"/{namespace}/global_costmap/costmap"
    p["local_costmap_topic"] = f"/{namespace}/local_costmap/costmap"
    p["global_frame"] = "map"
    p["robot_base_frame"] = f"{namespace}/base_footprint"
    p["frontier_marker_topic"] = f"/{namespace}/explore/frontiers"
    p["selected_frontier_topic"] = f"/{namespace}/explore/selected_frontier"
    p["optimized_map_topic"] = f"/{namespace}/explore/optimized_map"
    color_255 = _PATH_COLORS_255[min(_robot_index(namespace), len(_PATH_COLORS_255) - 1)]
    p["frontier_marker_color_r"] = color_255[0] / 255.0
    p["frontier_marker_color_g"] = color_255[1] / 255.0
    p["frontier_marker_color_b"] = color_255[2] / 255.0
    # team_map_topic and own_pose_topic are left as their config defaults
    # ("team_map_ddil" / "explore/pose") -- both relative, so the frontier_explorer
    # Node's own namespace push below resolves them to /{namespace}/... automatically,
    # matching team_map_fusion's own_pose_topic output_topic and this robot's own publish
    # topic respectively. peer_pose_topics has no such default to fall back on: it names
    # *other* robots' namespaces, which this robot's own namespace push cannot derive, so
    # without this it silently stays empty and peer_avoidance_radius_m never filters
    # anything (team_awareness_enabled's "no peers configured" no-op path, permanently).
    p["peer_pose_topics"] = [f"/{peer['name']}/explore/pose" for peer in peers]
    # The frontier_explorer Node() below passes this file straight through as
    # a --params-file with no namespace-aware rewriting (same issue fixed for
    # slam_toolbox above). A bare "frontier_explorer:" key does not bind to
    # the node once it's namespaced as /{namespace}/frontier_explorer, so
    # every override here was being silently dropped -- confirmed live via
    # "Could not transform base_footprint -> map" (the bare, un-namespaced
    # default) instead of using robot_base_frame's override.
    data[f"/{namespace}/frontier_explorer"] = data.pop("frontier_explorer")
    return _write_yaml(output_dir, f"{namespace}_explore.yaml", data)


# ── Distributed team map sharing ───────────────────────────────────────────────

def _parse_peers(peers_str):
    """Parse "name@ip@x@y@yaw,name2@ip2@x2@y2@yaw2" into a list of dicts.

    x/y/yaw are the peer's spawn offset in the shared map frame (same
    convention as this robot's own --robot-offset-x/y/yaw), needed to place
    the peer's map correctly since this robot can't see the peer's TF at all
    (each robot's discovery is intentionally isolated -- see the
    ROS_STATIC_PEERS notes in generate_launch_description below).
    """
    peers = []
    for entry in peers_str.split(","):
        entry = entry.strip()
        if not entry:
            continue
        parts = entry.split("@")
        if len(parts) != 5:
            raise ValueError(f"Invalid peer entry '{entry}', expected name@ip@x@y@yaw")
        name, ip, x, y, yaw = parts
        peers.append({"name": name, "ip": ip, "x": float(x), "y": float(y), "yaw": float(yaw)})
    return peers


def _team_map_share_actions(
    namespace, peers, map_transport, bandwidth_kbps, loss_pct, delay_ms, haar_levels, laptop_ip
):
    """Per-peer DDIL relay (+ vxch encode/decode) feeding this robot's own
    team_map_fusion instance, entirely local except for the relay nodes
    themselves, which are the only processes given broadened discovery
    (this robot's ROS_STATIC_PEERS + that one peer) via additional_env --
    not a global SetEnvironmentVariable, since ros2 launch runs actions
    concurrently/async and a global env mutation could easily leak into
    the main stack depending on scheduling, needlessly widening every other
    node's DDS visibility to that peer as well.
    """
    if not peers:
        return []

    actions = []
    total_bands = haar_levels + 1

    if map_transport == "vxch":
        # Encode our own local map once, for any peer to pull. Purely local
        # (reads our own /{namespace}/map), no broadened visibility needed.
        actions.append(
            Node(
                package="voxelcodec_ros",
                executable="occupancy_grid_vxch_node",
                name=f"vxch_encoder_{namespace}",
                output="screen",
                parameters=[{
                    "input_topic": f"/{namespace}/map",
                    "output_base_topic": f"/{namespace}/vxch/map",
                    "haar_levels": haar_levels,
                    "compression": "zstd",
                    "stream_id": namespace,
                    "use_sim_time": False,
                }],
            )
        )

    for i, peer in enumerate(peers):
        peer_name = peer["name"]
        # Only these relay nodes get broadened peer visibility; everything
        # else in this launch keeps the restrictive default set in
        # generate_launch_description.
        peer_env = {
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "ROS_STATIC_PEERS": f"{laptop_ip};{peer['ip']}",
        }
        seed = 42 + i

        if map_transport == "vxch":
            robot_ddil_base = f"/{namespace}/incoming/{peer_name}"
            # "reliable" here matches the encoder/decoder's band_qos change
            # (occupancy_grid_vxch_node.cpp / vxch_occupancy_grid_node.cpp) --
            # without it, this relay's own republish would still be volatile
            # even though its upstream source is now transient-local, leaving
            # a late-joining on-robot decoder stuck the same way the laptop
            # decoder was.
            relay_topics = [
                f"/{peer_name}/vxch/map/band_{k} {robot_ddil_base}/band_{k}"
                " voxelcodec_msgs/msg/VoxelChannel reliable"
                for k in range(total_bands)
            ]
            relay_topics.append(
                f"/{peer_name}/vxch/map/manifest {robot_ddil_base}/manifest"
                " voxelcodec_msgs/msg/VoxelManifest reliable"
            )
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="ddil_proxy_node",
                    name=f"ddil_proxy_{namespace}_from_{peer_name}",
                    output="screen",
                    additional_env=peer_env,
                    parameters=[{
                        "bandwidth_kbps": bandwidth_kbps,
                        "loss_pct": loss_pct,
                        "delay_ms": delay_ms,
                        "rng_seed": seed,
                        "relay_topics": relay_topics,
                        "use_sim_time": False,
                    }],
                )
            )
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="vxch_occupancy_grid_node",
                    name=f"vxch_decoder_{namespace}_from_{peer_name}",
                    output="screen",
                    parameters=[{
                        "input_base_topic": robot_ddil_base,
                        "output_topic": f"{robot_ddil_base}/map",
                        "haar_levels": haar_levels,
                        # Robots move slowly enough that a fresh reconstruction
                        # every second is wasted CPU (full Haar inverse +
                        # upsample per tick regardless of whether new bands
                        # arrived) -- 0.1 Hz still means every peer's map is at
                        # most 10s stale, which the map merge/costmap can absorb.
                        "publish_rate_hz": 0.1,
                        "use_sim_time": False,
                    }],
                )
            )
        else:
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="ddil_proxy_node",
                    name=f"ddil_proxy_{namespace}_from_{peer_name}",
                    output="screen",
                    additional_env=peer_env,
                    parameters=[{
                        "bandwidth_kbps": bandwidth_kbps,
                        "loss_pct": loss_pct,
                        "delay_ms": delay_ms,
                        "rng_seed": seed,
                        "relay_topics": [
                            f"/{peer_name}/map /{namespace}/incoming/{peer_name}/map"
                            " nav_msgs/msg/OccupancyGrid reliable"
                        ],
                        "use_sim_time": False,
                    }],
                )
            )

    actions.append(
        Node(
            package="bme_ros2_navigation",
            executable="team_map_fusion.py",
            name=f"team_map_fusion_{namespace}",
            output="screen",
            parameters=[{
                "robot_names": [p["name"] for p in peers],
                "offsets_x": [p["x"] for p in peers],
                "offsets_y": [p["y"] for p in peers],
                "offsets_yaw": [p["yaw"] for p in peers],
                "global_frame": "map",
                # Same reasoning as the vxch decoder above -- the numpy-vectorized
                # merge is now cheap, but there's still no reason to redo it 10x
                # more often than the slow-moving robots actually need.
                "publish_rate_hz": 0.1,
                "map_topic_template": f"/{namespace}/incoming/{{name}}/map",
                "output_topic": f"/{namespace}/team_map_ddil",
                "output_metadata_topic": f"/{namespace}/team_map_ddil_metadata",
                "publish_per_robot_maps": False,
                "use_sim_time": False,
            }],
        )
    )

    return actions


# nav2_bringup's navigation_launch.py unconditionally spawns 11 separate
# processes (controller_server, smoother_server, planner_server, route_server,
# behavior_server, bt_navigator, waypoint_follower, velocity_smoother,
# collision_monitor, docking_server, lifecycle_manager). This experiment uses
# bt_navigator's default BT XML (no default_nav_to_pose_bt_xml override),
# which never calls SmoothPath, route_server, waypoint-follow, or docking
# actions -- on a CPU-constrained robot Pi those 4 idle processes are pure
# overhead competing for the same cores as the servers actually driving
# motion, and were observed correlating with "Transform data too old"
# failures in controller_server under load. Everything below mirrors
# navigation_launch.py's non-composed load_nodes path, minus those 4 nodes.
def _nav2_actions(namespace, nav_cfg, log_level="info"):
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav_cfg,
            root_key=namespace,
            param_rewrites={"autostart": "True"},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
    ]

    def _server(package, executable, name, extra_remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            respawn=False,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + list(extra_remappings),
        )

    return [
        _server("nav2_controller", "controller_server", "controller_server",
                [("cmd_vel", "cmd_vel_nav")]),
        _server("nav2_planner", "planner_server", "planner_server"),
        _server("nav2_behaviors", "behavior_server", "behavior_server",
                [("cmd_vel", "cmd_vel_nav")]),
        _server("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        _server("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother",
                [("cmd_vel", "cmd_vel_nav")]),
        _server("nav2_collision_monitor", "collision_monitor", "collision_monitor"),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{"autostart": True}, {"node_names": lifecycle_nodes}],
        ),
    ]


# ── Main action builder ──────────────────────────────────────────────────────────────

def _create_actions(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    abs_namespace = f"/{namespace}"
    nav_params_file = LaunchConfiguration("nav_params_file").perform(context)
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    explore_config = LaunchConfiguration("explore_config").perform(context)
    local_bringup = LaunchConfiguration("local_bringup").perform(context).lower() in (
        "1", "true", "yes", "on"
    )
    spawn_x = float(LaunchConfiguration("spawn_x").perform(context))
    spawn_y = float(LaunchConfiguration("spawn_y").perform(context))
    spawn_yaw = float(LaunchConfiguration("spawn_yaw").perform(context))
    peers = _parse_peers(LaunchConfiguration("peers").perform(context))
    map_transport = LaunchConfiguration("map_transport").perform(context)
    bandwidth_kbps = float(LaunchConfiguration("bandwidth_kbps").perform(context))
    loss_pct = float(LaunchConfiguration("loss_pct").perform(context))
    delay_ms = float(LaunchConfiguration("delay_ms").perform(context))
    haar_levels = int(LaunchConfiguration("haar_levels").perform(context))
    laptop_ip = os.environ.get("VIZ_LAPTOP_IP", "192.168.100.20")

    output_dir = tempfile.mkdtemp(prefix=f"bme_hw_{namespace}_")

    slam_cfg = _patch_slam_params(slam_params_file, namespace, output_dir)
    nav_cfg = _patch_nav_params(nav_params_file, namespace, output_dir)
    explore_cfg = _patch_explore_params(explore_config, namespace, output_dir, peers)

    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py"
    )
    tb3_launch = os.path.join(
        get_package_share_directory("turtlebot3_bringup"),
        "launch",
        "robot.launch.py"
    )

    bringup_delay = 5.0 if local_bringup else 0.0
    slam_delay = bringup_delay + 3.0
    nav2_delay = slam_delay + 10.0

    actions = []

    if local_bringup:
        # robot.launch.py already does its own internal PushRosNamespace(namespace)
        # for topics (odom, scan, joint_states, ...), so no PushRosNamespace is added
        # here -- only /tf and /tf_static need redirecting, since
        # tf2_ros::TransformBroadcaster / StaticTransformBroadcaster (used by
        # turtlebot3_ros and robot_state_publisher) always publish to the
        # absolute, unnamespaced /tf(_static) regardless of node namespace.
        actions.append(
            GroupAction([
                SetRemap(src="/tf", dst=f"{abs_namespace}/tf"),
                SetRemap(src="/tf_static", dst=f"{abs_namespace}/tf_static"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(tb3_launch),
                    launch_arguments={"namespace": namespace}.items(),
                ),
            ])
        )

    # One-way relay of this robot's own namespaced TF outward onto the shared
    # global /tf(_static), purely for a base-station RViz to view all robots
    # merged in one window and for bagged runs to capture a single merged TF
    # stream. Nothing in this stack reads the global topic back in, so this
    # can't reopen the old tf_frame_renamer cross-talk bug.
    actions.append(
        Node(
            package="topic_tools",
            executable="relay",
            name=f"{namespace}_tf_relay",
            output="screen",
            arguments=[f"/{namespace}/tf", "/tf"],
            parameters=[{"use_sim_time": False}],
        )
    )
    actions.append(
        Node(
            package="topic_tools",
            executable="relay",
            name=f"{namespace}_tf_static_relay",
            output="screen",
            arguments=[f"/{namespace}/tf_static", "/tf_static"],
            parameters=[{"use_sim_time": False}],
        )
    )

    # Only bridge map -> {namespace}/map here. slam_toolbox itself publishes
    # {namespace}/map -> {namespace}/odom once active; also statically publishing
    # map -> {namespace}/odom would give that frame two competing parents,
    # leaving tf2 with disconnected/unstable trees (costmap "out of map bounds"
    # and "no map received" warnings trace back to this).
    for child_frame, tf_name in [
        (f"{namespace}/map", f"{namespace}_map_to_slam_map"),
    ]:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=tf_name,
                namespace=abs_namespace,
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
                remappings=[("/tf_static", "tf_static")],
            )
        )

    actions.append(
        Node(
            package="bme_ros2_navigation",
            executable="per_robot_map_compositor.py",
            name=f"per_robot_map_compositor_{namespace}",
            namespace=abs_namespace,
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

    actions.append(
        Node(
            package="rviz_autonomous_exploration_benchmark",
            executable="frontier_path_tracker.py",
            name=f"frontier_path_tracker_{namespace}",
            namespace=abs_namespace,
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
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )
    )

    actions.append(
        TimerAction(
            period=slam_delay,
            actions=[
                GroupAction([
                    PushRosNamespace(abs_namespace),
                    # slam_toolbox publishes "map"/"map_metadata" as absolute
                    # topics ("/map", "/map_metadata") in its own source, so
                    # node namespacing alone never prefixes them -- without
                    # this, the real map data goes to the bare global /map
                    # and per_robot_map_compositor (subscribed to
                    # {namespace}/map) never sees it.
                    SetRemap(src="/map", dst=f"{abs_namespace}/map"),
                    SetRemap(src="/map_metadata", dst=f"{abs_namespace}/map_metadata"),
                    # slam_toolbox's tf2_ros::Buffer/TransformListener also default to
                    # the absolute /tf(_static) regardless of node namespace -- redirect
                    # to this robot's own namespaced topics (see turtlebot3_bringup
                    # GroupAction above, which is what now publishes there).
                    SetRemap(src="/tf", dst=f"{abs_namespace}/tf"),
                    SetRemap(src="/tf_static", dst=f"{abs_namespace}/tf_static"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(slam_launch),
                        launch_arguments={
                            "use_sim_time": "false",
                            "slam_params_file": slam_cfg,
                        }.items(),
                    ),
                ])
            ],
        )
    )

    actions.append(
        TimerAction(
            period=nav2_delay,
            actions=[
                GroupAction([
                    PushRosNamespace(abs_namespace),
                    *_nav2_actions(namespace, nav_cfg),
                ])
            ],
        )
    )

    actions.append(
        Node(
            package="frontier_exploration_ros2",
            executable="frontier_explorer",
            name="frontier_explorer",
            namespace=abs_namespace,
            output="screen",
            parameters=[explore_cfg, {"use_sim_time": False}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )
    )

    actions.extend(
        _team_map_share_actions(
            namespace, peers, map_transport, bandwidth_kbps, loss_pct, delay_ms,
            haar_levels, laptop_ip,
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Set here (not just in the wrapper shell script) so it's guaranteed to
        # reach every process this launch tree spawns, regardless of whether the
        # invoking shell's exported env actually survives to this point. General
        # DDS hygiene for a multi-robot fleet on a shared ROS_DOMAIN_ID: without
        # this, every robot's nodes discover every other robot's nodes/topics by
        # default, which is unnecessary traffic/overhead even though /tf(_static)
        # itself is now isolated per robot via the SetRemap calls above.
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        SetEnvironmentVariable("ROS_STATIC_PEERS", os.environ.get("VIZ_LAPTOP_IP", "192.168.100.20")),
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
        DeclareLaunchArgument(
            "peers", default_value="",
            description="Comma-separated name@ip@x@y@yaw for every other robot in the "
                        "team, for distributed map sharing. Empty = no map sharing."),
        DeclareLaunchArgument("map_transport", default_value="baseline",
                              description="'baseline' or 'vxch' -- how peer maps are relayed"),
        DeclareLaunchArgument("bandwidth_kbps", default_value="0"),
        DeclareLaunchArgument("loss_pct", default_value="0.0"),
        DeclareLaunchArgument("delay_ms", default_value="0"),
        DeclareLaunchArgument("haar_levels", default_value="4"),
        OpaqueFunction(function=_create_actions),
    ])
