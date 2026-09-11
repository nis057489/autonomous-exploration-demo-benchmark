#include <wavestream/codec.hpp>

// Linking two users of the complete header catches accidental non-inline
// definitions. Neither translation unit links codec.cpp.
std::vector<std::int8_t> decode_from_second_translation_unit(const wavestream::EncodedTile & tile)
{
  return wavestream::decode_tile(tile, tile.bands.size());
}
