# Every tile scheduled for ONE Haar band in ONE send tick, as a single message.
#
# Replaces publishing one VoxelChannel per (tile, band). A send tick routinely
# services ~48 tiles, and each separate message re-paid a full envelope --
# header + stamp + frame_id + stream_id + the descriptor's fixed strings
# ('occupancy_haar_band', 'haar-wavelet', ...) + CDR overhead -- roughly 550
# bytes to carry ~26 bytes of coefficients. Batching amortises all of that
# across the tick's tiles.
#
# Still one message PER BAND, not one per tick: ddil_proxy_node prioritises and
# rate-limits by band index (coarse bands first, so a bandwidth-constrained
# link still gets a usable low-resolution map before any detail), and it does
# that by topic. Collapsing all bands into one message would destroy that
# progressive behaviour, which is where vxch's coverage advantage comes from.

std_msgs/Header header
string stream_id

# Which band these tiles carry. Redundant with the topic name (band_<k>), but
# carried so a decoder reading a bag doesn't have to parse topic strings.
uint8 band_index

# Stream-level codec constants. These are identical for every tile and every
# tick, so they sit here once per batch rather than in each tile's entry (they
# used to be repeated per message as ASCII metadata: haar_levels, haar_varint,
# haar_total_bands, tile_size_cells).
uint8 haar_levels
uint8 haar_total_bands
bool varint_encoding
string compression      # 'zstd' or 'none'
int32 tile_size_cells

VoxelTilePayload[] tiles
