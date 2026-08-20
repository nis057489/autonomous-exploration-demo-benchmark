"""
VXCH progressive streaming experiment launcher.

Peer-to-peer network model, matching hw_namespaced_stack.launch.py (the real-robot
launch path) rather than a simulation-only centralized "base station" fusing every
robot's map before any DDIL link is applied. There is no base station on the real
robots -- every robot only ever sees its own local map plus whatever its own DDIL
links deliver from each peer directly -- so the sim topology mirrors that: one
independent per-peer downlink per robot, throttled/lossy independently, fused
locally on each robot via its own team_map_fusion instance.

map_transport:=baseline
  per robot X, per peer Y: ddil_proxy_{X}_from_{Y} relays /{Y}/map (peer's own local
    map) → /{X}/incoming/{Y}/map
  per robot X: team_map_fusion_{X} fuses /{X}/incoming/{peer}/map for every peer →
    /{X}/team_map_ddil
  robot: PerRobotMapCompositor merges /{X}/map (local) + /{X}/team_map_ddil →
    /{X}/nav_map

map_transport:=vxch
  per robot X: occupancy_grid_vxch_encoder_{X} encodes its own /{X}/map →
    /{X}/vxch/map/band_0..N + manifest
  per robot X, per peer Y: ddil_proxy_{X}_from_{Y} relays peer Y's bands + manifest →
    /{X}/incoming/{Y}/band_0..N + manifest; vxch_decoder_{X}_from_{Y} decodes that →
    /{X}/incoming/{Y}/map
  per robot X: team_map_fusion_{X} fuses /{X}/incoming/{peer}/map for every peer →
    /{X}/team_map_ddil
  robot: PerRobotMapCompositor merges /{X}/map (local) + /{X}/team_map_ddil →
    /{X}/nav_map

Nav2 and frontier exploration subscribe to /{robot}/nav_map in both modes.

Each of a robot's per-peer downlinks is independent -- one ddil_proxy instance per
(robot, peer) pair, so peers cannot starve each other and there is no implicit
multicast advantage for either transport.

DDIL args (default 0 = no degradation), applied identically to every (robot, peer)
downlink:
  bandwidth_kbps   token-bucket rate limit
  loss_pct         per-message drop probability (0–100)
  delay_ms         additional forwarding latency
"""

import json
import math
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


def _robot_pose(index, count, x, y, z, yaw, spacing, world):
    """Mirrors multi_robot_navigation_with_slam.launch.py's _robot_pose exactly --
    duplicated (not imported) so this file stays a self-contained launch file, same
    as hw_namespaced_stack.launch.py's own peer-position handling. Needed here only
    to compute each robot's (offset_x, offset_y, offset_yaw) for team_map_fusion_{X};
    the actual spawn itself still happens inside the included navigation launch."""
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
    compression = LaunchConfiguration("compression").perform(context)
    varint_encoding = _bool_value(LaunchConfiguration("varint_encoding").perform(context))
    tile_size_m = float(LaunchConfiguration("tile_size_m").perform(context))
    schedule_mode = LaunchConfiguration("schedule_mode").perform(context)
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

    if map_transport not in ("baseline", "vxch", "zstd"):
        raise ValueError(
            f"map_transport must be 'baseline', 'vxch', or 'zstd', got '{map_transport}'"
        )

    impairment_mode = LaunchConfiguration("impairment_mode").perform(context)
    if impairment_mode not in ("sim", "tc"):
        raise ValueError(f"impairment_mode must be 'sim' or 'tc', got '{impairment_mode}'")
    is_tc = impairment_mode == "tc"

    is_vxch = map_transport == "vxch"
    # zstd: same relayed OccupancyGrid as baseline, but the serialized message
    # is zstd-compressed before publishing and decompressed on the receiving
    # end -- isolates what generic compression alone buys vs. baseline, with
    # no wavelet/tiling/scheduling from vxch involved.
    is_zstd = map_transport == "zstd"

    robot_names = [f"robot{i + 1}" for i in range(num_robots)]

    total_bands = haar_levels + 1

    # ── tc impairment mode: netns/IP scheme (must match setup_ddil_netns.sh) ───
    # Each robot gets its own netns (its "radio"), joined to a shared bridge (the
    # "router") that also carries the main netns's own address -- see that
    # script's own topology comment. Only the nodes that actually carry
    # inter-robot DDIL traffic run inside a robot's netns; everything else
    # (nav2, slam, physics, team_map_fusion, decoders) stays in the main netns,
    # since real hardware doesn't throttle a robot's own onboard buses either.
    MAIN_NETNS_IP = "10.77.0.1"

    def _netns_name(robot_index: int) -> str:
        return f"ns-robot{robot_index + 1}"

    def _netns_ip(robot_index: int) -> str:
        return f"10.77.0.{10 + robot_index + 1}"

    def _netns_kwargs(robot_index: int, static_peers: list) -> dict:
        """Node kwargs that run a node inside robot_index's own netns.
        ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET (not LOCALHOST) -- verified
        against real run logs that LOCALHOST + static-peers-only discovery is
        NOT reliable under load: one robot's encoder->main-netns discovery
        (needed just to subscribe to its own /{robot}/map) would consistently
        never complete, correctly configured ROS_STATIC_PEERS notwithstanding,
        with zero self-healing even after 60s+ of grace before tc shaping.
        SUBNET is safe here specifically because a robot's netns has exactly
        one external interface (its veth into br-ddil, a bridge with no route
        to the container's own outward-facing network) -- multicast discovery
        physically cannot escape the isolated 10.77.0.0/24 subnet, so this
        doesn't reopen the earlier cross-container multicast-pollution bug
        (that was about the container's own eth0/docker-bridge subnet, a
        completely different network this netns has no route to at all).
        ROS_STATIC_PEERS is kept alongside SUBNET as a redundant accelerant,
        not the sole mechanism. Empty dict (no namespacing) in sim mode."""
        if not is_tc:
            return {}
        return {
            "prefix": f"ip netns exec {_netns_name(robot_index)}",
            "additional_env": {
                "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
                "ROS_STATIC_PEERS": ";".join(static_peers),
            },
        }

    # Base DDIL params shared across every per-(robot, peer) proxy.
    # rng_seed is offset per (robot, peer) pair so packet-loss patterns are
    # independent -- otherwise every downlink into the same robot, or every
    # robot's downlink from the same peer, would drop in lockstep.
    ddil_params_base = {
        "bandwidth_kbps": bandwidth_kbps,
        "loss_pct": loss_pct,
        "delay_ms": delay_ms,
        "use_sim_time": use_sim_time,
    }

    def ddil_params_for(robot_index: int, peer_index: int) -> dict:
        seed = (rng_seed + robot_index * num_robots + peer_index) if rng_seed >= 0 else -1
        if is_tc:
            # Real tc netem on the netns links is doing the shaping; the
            # software token bucket/loss/delay would double-impair on top of it.
            # display_bandwidth_kbps is UI-only (see ddil_proxy_node.cpp) --
            # reports the real configured tc rate on the NetworkStatsPanel
            # ("Capacity") without feeding the token bucket, so the panel
            # doesn't read "Unlimited" for a link that's actually shaped.
            return {"bandwidth_kbps": 0.0, "loss_pct": 0.0, "delay_ms": 0.0,
                    "display_bandwidth_kbps": bandwidth_kbps,
                    "use_sim_time": use_sim_time, "rng_seed": seed}
        return {**ddil_params_base, "rng_seed": seed}

    # Each robot's (offset_x, offset_y, offset_yaw) in the shared "map" frame --
    # needed here (not just inside the included navigation launch) so each robot's
    # own team_map_fusion_{X} can place incoming peer maps correctly. Duplicates
    # multi_robot_navigation_with_slam.launch.py's own per-index pose computation;
    # both must derive the same values from the same (world, x, y, z, yaw, spacing,
    # spawn_positions_json) inputs for a robot's own idea of where its peers are to
    # agree with where those peers actually spawned.
    spawn_positions_raw = LaunchConfiguration("spawn_positions_json").perform(context)
    spawn_positions = json.loads(spawn_positions_raw) if spawn_positions_raw else []
    x_f, y_f, z_f, yaw_f, spacing_f = (float(x), float(y), float(z), float(yaw), float(spacing))
    robot_poses = []
    for i in range(num_robots):
        if spawn_positions:
            pos = spawn_positions[i % len(spawn_positions)]
            robot_poses.append(
                (float(pos["x"]), float(pos["y"]), z_f, float(pos.get("yaw", yaw_f)))
            )
        else:
            robot_poses.append(_robot_pose(i, num_robots, x_f, y_f, z_f, yaw_f, spacing_f, world))

    actions = []

    # ── Navigation + SLAM + world ──────────────────────────────────────────────
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
                "seed": str(rng_seed),
                "spawn_positions_json": LaunchConfiguration("spawn_positions_json"),
                "impairment_mode": impairment_mode,
                "controller_type": LaunchConfiguration("controller_type"),
            }.items(),
        )
    )

    # ── Frontier exploration ───────────────────────────────────────────────────
    # config_visit_once.yaml is tuned specifically against real turtlebot3_waffle
    # hardware (0.22 m/s, ~3.5m real LDS-01 lidar range) -- applying it blanket to
    # mogi_bot (0.8 m/s max, 10m sim sensor) would be the same class of mismatch
    # the footprint fix above exists to avoid, just for frontier scoring instead
    # of costmap geometry. mogi_bot keeps config.yaml (its long-standing default,
    # no reported issues); only turtlebot3_waffle -- which is meant to mirror
    # hardware -- gets the hardware-tuned config.
    frontier_params_file = (
        "config/frontier_exploration_ros2/config_visit_once.yaml"
        if "turtlebot3_waffle" in model
        else "config/frontier_exploration_ros2/config.yaml"
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_rviz, "launch", "multi_robot_frontier_explorer.launch.py")
            ),
            launch_arguments={
                "num_robots": str(num_robots),
                "use_sim_time": use_sim_time_str,
                "robot_startup_delay_s": str(robot_startup_delay_s),
                "params_file": frontier_params_file,
            }.items(),
        )
    )

    # ── VXCH mode: one encoder per robot, encoding that robot's own local map ──
    # Matches hw_namespaced_stack.launch.py's _team_map_share_actions: every robot
    # encodes its own /{robot}/map once, for any peer to pull -- purely local, no
    # DDIL involved yet.
    if is_vxch:
        for i, name in enumerate(robot_names):
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="occupancy_grid_vxch_node",
                    name=f"vxch_encoder_{name}",
                    output="screen",
                    parameters=[{
                        "input_topic": f"/{name}/map",
                        "output_base_topic": f"/{name}/vxch/map",
                        "haar_levels": haar_levels,
                        "tile_size_m": tile_size_m,
                        "compression": compression,
                        "varint_encoding": varint_encoding,
                        "schedule_mode": schedule_mode,
                        "use_sim_time": use_sim_time,
                    }],
                    # Deliberately NOT netns'd, even in tc mode: this used to run
                    # inside the robot's own netns on the theory that its own
                    # uplink should cross that robot's tc-shaped veth too (a
                    # "two-hop" -- source uplink + receiver downlink -- emulation
                    # of a real wireless link). In practice that made this node's
                    # subscription to /{name}/map (published by slam_toolbox,
                    # which stays in the main netns) cross the SAME shaped,
                    # bandwidth-limited link its own encoded output was competing
                    # for -- so once real throttling kicked in, the /map
                    # subscription's own RELIABLE QoS liveliness traffic starved
                    # right alongside the deliberately-throttled band data, the
                    # subscription silently died, and (being on the same
                    # congested link) never rediscovered. That's not modeling
                    # anything real: /map never crosses an actual wireless link
                    # on hardware, only the *encoded* band output does, and only
                    # ddil_proxy (below) needs to sit on the genuinely shaped
                    # link for that. Running the encoder here in the main netns
                    # keeps it off the shaped link entirely, at the cost of no
                    # longer emulating the source robot's own uplink constraint --
                    # acceptable given this is a ROS-overhead-inclusive
                    # integration test, not a bit-exact physical-layer model.
                )
            )
    elif is_zstd:
        # Same reasoning as the vxch encoder above for staying out of the
        # robot's own netns: this node's /{name}/map subscription must not
        # cross the same shaped link its own compressed output competes for.
        for i, name in enumerate(robot_names):
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="occupancy_grid_zstd_compress_node",
                    name=f"zstd_encoder_{name}",
                    output="screen",
                    parameters=[{
                        "input_topic": f"/{name}/map",
                        "output_topic": f"/{name}/zstd/map",
                        "use_sim_time": use_sim_time,
                    }],
                )
            )
    elif is_tc:
        # Baseline mode has no encode step, just a rename to /{name}/map_uplink
        # so ddil_proxy below has a stable topic name to pull regardless of
        # mode. See the vxch encoder's comment above for why this deliberately
        # stays OUT of the robot's own netns.
        for i, name in enumerate(robot_names):
            actions.append(
                Node(
                    package="topic_tools",
                    executable="relay",
                    name=f"map_uplink_relay_{name}",
                    output="screen",
                    arguments=[f"/{name}/map", f"/{name}/map_uplink"],
                    parameters=[{"use_sim_time": use_sim_time}],
                )
            )

    # ── Per-robot, per-peer DDIL relay ──────────────────────────────────────────
    # Every robot X pulls each peer Y's map directly over its own independent
    # (X, Y) downlink -- no shared/centralized fusion node in between, matching
    # hw_namespaced_stack.launch.py exactly (which runs the equivalent of this
    # loop once per physical robot, on that robot's own compute). Here all robots
    # run in one process, so the loop just does it N times instead of N robots
    # each doing it once.
    #
    # Pair creation order is round-robin across robots (round 1: each robot's
    # "+1" peer; round 2: each robot's "+2" peer; ...), NOT grouped by robot
    # (all of robot1's pairs, then all of robot2's, ...). Grouped-by-robot order
    # was a real bug: in tc mode, each Node action's underlying process spawn
    # (particularly with the `ip netns exec` prefix) has enough overhead that
    # earlier-listed actions get a measurable head start on DDS discovery over
    # later-listed ones. With every one of robot1's pairs listed before any of
    # robot2's, before any of robot3's, robot1's outbound discovery consistently
    # won races against robot3's, producing an exact, reproducible pattern where
    # every (i, j) pair with i < j succeeded and every pair with i > j failed --
    # not the random per-run flakiness it first looked like, but a deterministic
    # bandwidth/discovery advantage for lower-indexed robots every single time.
    # Round-robin order spreads that head-start evenly across every robot instead
    # of concentrating it on whichever one happens to be listed first.
    pair_order = [
        (i, (i + offset) % num_robots)
        for offset in range(1, num_robots)
        for i in range(num_robots)
    ]
    for i, peer_index in pair_order:
        name = robot_names[i]
        peer_name = robot_names[peer_index]
        robot_ddil_base = f"/{name}/incoming/{peer_name}"

        if is_vxch:
            relay_topics = [
                f"/{peer_name}/vxch/map/band_{k} {robot_ddil_base}/band_{k}"
                " voxelcodec_msgs/msg/VoxelChannel"
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
                    name=f"ddil_proxy_{name}_from_{peer_name}",
                    output="screen",
                    parameters=[{
                        **ddil_params_for(i, peer_index),
                        "relay_topics": relay_topics,
                    }],
                    # Runs inside X's own netns (this is X's own onboard relay
                    # process): this is the one hop that's actually meant to be
                    # tc-shaped, so X's incoming pull of Y's data genuinely
                    # crosses X's shaped veth. Y's encoder now runs in the main
                    # netns (see vxch_encoder_{name} above), not Y's own netns,
                    # so MAIN_NETNS_IP is the only static peer needed to reach
                    # it; it also covers this node's own output being
                    # discoverable by vxch_decoder_{X}_from_{Y}, which stays in
                    # the main netns.
                    **_netns_kwargs(i, [MAIN_NETNS_IP]),
                )
            )
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="vxch_occupancy_grid_node",
                    name=f"vxch_decoder_{name}_from_{peer_name}",
                    output="screen",
                    parameters=[{
                        "input_base_topic": robot_ddil_base,
                        "output_topic": f"{robot_ddil_base}/map",
                        "haar_levels": haar_levels,
                        "publish_rate_hz": 1.0,
                        "use_sim_time": use_sim_time,
                    }],
                )
            )
        elif is_zstd:
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="ddil_proxy_node",
                    name=f"ddil_proxy_{name}_from_{peer_name}",
                    output="screen",
                    parameters=[{
                        **ddil_params_for(i, peer_index),
                        "relay_topics": [
                            f"/{peer_name}/zstd/map {robot_ddil_base}/zstd_map"
                            " std_msgs/msg/UInt8MultiArray reliable"
                        ],
                    }],
                    **_netns_kwargs(i, [MAIN_NETNS_IP]),
                )
            )
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="occupancy_grid_zstd_decompress_node",
                    name=f"zstd_decoder_{name}_from_{peer_name}",
                    output="screen",
                    parameters=[{
                        "input_topic": f"{robot_ddil_base}/zstd_map",
                        "output_topic": f"{robot_ddil_base}/map",
                        "use_sim_time": use_sim_time,
                    }],
                )
            )
        else:
            # In tc mode, pull from the peer's map_uplink relay (a stable name
            # regardless of mode) rather than /{peer}/map directly -- both now
            # live in the main netns (see map_uplink_relay_{name} above), so
            # this is purely a naming convenience at this point, not a second
            # shaped hop.
            source_topic = f"/{peer_name}/map_uplink" if is_tc else f"/{peer_name}/map"
            actions.append(
                Node(
                    package="voxelcodec_ros",
                    executable="ddil_proxy_node",
                    name=f"ddil_proxy_{name}_from_{peer_name}",
                    output="screen",
                    parameters=[{
                        **ddil_params_for(i, peer_index),
                        "relay_topics": [
                            f"{source_topic} {robot_ddil_base}/map"
                            " nav_msgs/msg/OccupancyGrid reliable"
                        ],
                    }],
                    **_netns_kwargs(i, [MAIN_NETNS_IP]),
                )
            )

    # ── Per-robot local fusion ──────────────────────────────────────────────────
    # Separate from the interleaved relay loop above (team_map_fusion runs in the
    # main netns and isn't part of the discovery race that loop's ordering
    # exists to defuse, so its own creation order doesn't matter).
    for i, name in enumerate(robot_names):
        peer_indices = [p for p in range(num_robots) if p != i]
        # Placed at each peer's spawn offset so a peer's map lands in the right
        # place in this robot's "map" frame regardless of which robot is doing
        # the fusing -- same team_map_fusion.py node hw_namespaced_stack.launch.py
        # uses for its own per-robot role.
        peer_names = [robot_names[p] for p in peer_indices]
        peer_offsets = [robot_poses[p] for p in peer_indices]
        actions.append(
            Node(
                package="bme_ros2_navigation",
                executable="team_map_fusion.py",
                name=f"team_map_fusion_{name}",
                output="screen",
                parameters=[{
                    "robot_names": peer_names,
                    "offsets_x": [pose[0] for pose in peer_offsets],
                    "offsets_y": [pose[1] for pose in peer_offsets],
                    "offsets_yaw": [pose[3] for pose in peer_offsets],
                    "global_frame": "map",
                    "publish_rate_hz": 1.0,
                    "map_topic_template": f"/{name}/incoming/{{name}}/map",
                    "output_topic": f"/{name}/team_map_ddil",
                    "output_metadata_topic": f"/{name}/team_map_ddil_metadata",
                    "publish_per_robot_maps": False,
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
                description="'baseline', 'vxch', or 'zstd'"),
            DeclareLaunchArgument(
                "impairment_mode", default_value="sim",
                description="'sim' (default) = ddil_proxy_node's in-process token-bucket/"
                            "loss/delay simulation. 'tc' = real per-robot network "
                            "namespaces joined by veth pairs, shaped with actual `tc "
                            "netem` (see setup_ddil_netns.sh) -- the same tool used to "
                            "throttle the wireless interface in the hardware test. "
                            "Requires the container to have CAP_NET_ADMIN and "
                            "setup_ddil_netns.sh to have already been run for this "
                            "num_robots (launch.sh does this automatically)."),
            DeclareLaunchArgument(
                "controller_type", default_value="pure_pursuit",
                description="'pure_pursuit' (default) = nav2_regulated_pure_pursuit_"
                            "controller, mirroring navigation_hw.yaml's real-hardware "
                            "tuning. 'mppi' = nav2_mppi_controller, sim's original "
                            "controller -- kept for A/B comparison against pure_pursuit."),
            DeclareLaunchArgument(
                "haar_levels", default_value="4",
                description="Haar wavelet levels (total bands = levels+1)"),
            DeclareLaunchArgument(
                "compression", default_value="zstd",
                description="vxch only. 'zstd' or 'none' -- zstd-compresses each band's "
                            "zigzag-varint payload before it's sent; 'none' sends the raw "
                            "varint bytes, useful for isolating how much of vxch's "
                            "bandwidth win is the wavelet/tiling encoding itself vs. "
                            "compression on top"),
            DeclareLaunchArgument(
                "varint_encoding", default_value="true",
                description="vxch only. true (default) zigzag-varint packs each band's "
                            "Haar coefficients before compression; false packs them as "
                            "fixed-width int32 instead. Independent of compression -- with "
                            "compression=none this isolates varint packing's own bandwidth "
                            "contribution, since compression=none alone still leaves "
                            "varint packing in place"),
            DeclareLaunchArgument(
                "tile_size_m", default_value="2.0",
                description="Encode the map as independent tile_size_m x tile_size_m Haar "
                            "pyramids instead of one pyramid for the whole grid, so a still-"
                            "changing area (wherever a robot currently is) can't starve "
                            "another, already-settled area's detail bands"),
            DeclareLaunchArgument(
                "schedule_mode", default_value="smart",
                description="vxch only. 'smart' = only resend a band when it actually "
                            "changed, least-recently-sent-first within a tile. 'simple' = "
                            "always resend every band every cycle, strict coarsest-first, "
                            "no change detection -- matches how the baseline OccupancyGrid "
                            "relay behaves (it has no dedup either)"),
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
