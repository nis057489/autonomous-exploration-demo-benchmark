# Aggregated stats for one Haar band across all tiles currently tracked by a
# single ddil_proxy_node link (part of DdilStats.msg -- see there for the
# per-link fields this sits alongside).

uint8 band_index      # 0 = coarsest (sent first), higher = finer detail
uint32 pending_count   # (tile, band) messages queued, not yet sent, for this band
uint64 pending_bytes   # payload bytes queued for this band, waiting to be sent
uint64 sent_bytes      # cumulative bytes of this band actually sent since node start
uint64 sent_count      # cumulative messages of this band actually sent
bool active            # true if this band's message was the most recently sent
float64 eta_sec        # approx. seconds until this band's backlog (and every
                        # higher-priority band's backlog still ahead of it) drains
                        # at the link's current rate. 0 = nothing queued (idle).
                        # -1 = unknown (link idle/unmeasured and unlimited bandwidth).
