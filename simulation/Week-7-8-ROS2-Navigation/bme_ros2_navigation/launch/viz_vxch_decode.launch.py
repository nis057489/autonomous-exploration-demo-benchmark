"""
Base-station-side VXCH decode, for watching each robot's own progressive codec
feed as it arrives -- not the already-reconstructed map some other node
publishes, the actual band-by-band VXCH stream, reconstructed here.

Decodes each robot's own local encode (/{robot}/vxch/map/manifest + band_0..N,
published straight from that robot's occupancy_grid_vxch_node) directly on the
base station. This is deliberately the SOURCE feed, not the DDIL-relayed
/{robot}/incoming/{peer}/... data used elsewhere for inter-robot bandwidth
comparisons -- the point here is just to see each robot's own VXCH stream
sharpen band-by-band as it's produced, one decoder per robot, no fusion.

Every robot's stack already lists this base station as a static peer (see
SetEnvironmentVariable("ROS_STATIC_PEERS", ...) in hw_namespaced_stack.launch.py's
generate_launch_description), so /{robot}/vxch/map/* is already reachable here
with no relay changes needed on the robots.

Usage:
  ros2 launch bme_ros2_navigation viz_vxch_decode.launch.py \
      robot_names:="robot1,robot2,robot3" haar_levels:=5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_actions(context, *args, **kwargs):
    robot_names = [
        name.strip()
        for name in LaunchConfiguration("robot_names").perform(context).split(",")
        if name.strip()
    ]
    haar_levels = int(LaunchConfiguration("haar_levels").perform(context))

    actions = []
    for name in robot_names:
        actions.append(
            Node(
                package="voxelcodec_ros",
                executable="vxch_occupancy_grid_node",
                name=f"laptop_decoder_{name}_own",
                output="screen",
                parameters=[{
                    "input_base_topic": f"/{name}/vxch/map",
                    "output_topic": f"/laptop/{name}/vxch_map",
                    "haar_levels": haar_levels,
                    "publish_rate_hz": 1.0,
                    "use_sim_time": False,
                }],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_names",
            description="Comma-separated robot namespaces, e.g. robot1,robot2,robot3 "
                        "(names only -- unlike ROBOT_HOSTS, no ip/offset needed since "
                        "each robot's own feed is decoded standalone, not fused)."),
        DeclareLaunchArgument("haar_levels", default_value="4",
                              description="Must match HAAR_LEVELS used on the robots."),
        OpaqueFunction(function=_create_actions),
    ])
