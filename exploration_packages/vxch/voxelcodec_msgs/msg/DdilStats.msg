# Periodic network-stats snapshot published by one ddil_proxy_node instance
# (one per (robot, peer) DDIL downlink -- see multi_robot_vxch_experiment.launch.py).
# Intended for monitoring/UI (e.g. voxelcodec_ros/NetworkStatsPanel in RViz),
# not for anything decode-path-critical.

std_msgs/Header header

string link_name        # this proxy node's own name, e.g. "ddil_proxy_robot2_from_robot1"
float64 bandwidth_kbps  # configured token-bucket rate limit (0 = unlimited)
float64 send_rate_bps   # measured, windowed instantaneous send rate, all bands combined
uint64 queued_bytes     # total bytes currently queued (all bands + manifest)
uint64 sent_bytes       # cumulative bytes actually sent since node start
uint64 msgs_received
uint64 msgs_dropped
uint64 msgs_deduped
uint64 msgs_stale_dropped

voxelcodec_msgs/DdilBandStatus[] bands
