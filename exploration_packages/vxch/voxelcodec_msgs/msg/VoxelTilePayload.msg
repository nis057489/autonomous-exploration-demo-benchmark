# One tile's coded coefficients for a single Haar band, inside a VoxelTileBatch.
#
# Everything here is a fixed-width field rather than an ASCII key/value entry in
# VoxelChannelDescriptor.metadata. Measured on a long_t run, that metadata map
# cost 321 bytes per message to carry ~14 small integers, against an average
# Haar payload of 26 bytes -- the map was 55% of all bandwidth the encoder
# spent. These same values as typed fields cost 28 bytes.

# Tile position in the world-anchored tile lattice (can be negative).
int32 tile_row
int32 tile_col

# This tile's cell extent. Smaller than tile_size_cells for a tile clipped by
# the sender's array edge.
int32 tile_width
int32 tile_height

# Where this tile's data starts inside its own lattice tile -- nonzero only for
# a clipped tile whose array edge isn't tile-aligned. See tile_scheduler.hpp.
int32 tile_offset_row
int32 tile_offset_col

# Coefficient count this band decodes to (the decoder needs it to size the
# varint/fixed-width decode; it is not derivable from payload length).
uint32 element_count

uint8[] payload
