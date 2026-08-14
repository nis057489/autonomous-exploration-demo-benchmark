std_msgs/Header header

# Current token-bucket / injected-degradation settings (mirrors ddil_proxy_node's params).
float64 bandwidth_kbps
float64 loss_pct
float64 delay_ms

# Cumulative since this ddil_proxy_node process started.
uint64 msgs_received
uint64 msgs_sent
uint64 msgs_dropped
uint64 msgs_deduped
uint64 bytes_sent

# Instantaneous.
uint32 queue_depth
