#include <wavestream/codec.hpp>
#include <iostream>

int main()
{
  constexpr int HAAR_LEVELS = 2;
  const std::vector<std::int8_t> tile{
    -1, -1, 100, 100,
    -1, -1, 100, 100,
     0,  0,  50,  50,
     0,  0,  50,  50};

  // Algorithm: Tile encoding (phi, L Haar steps, per-group packing + Zstd).
  const auto encoded = wavestream::encode_tile(tile, 4, 4, HAAR_LEVELS);

  // Algorithm: Progressive tile reconstruction. Simulate groups arriving
  // coarse-first; a network receiver would buffer out-of-order groups until
  // a contiguous prefix is available, and manage tile geometry separately.
  wavestream::CoefficientGroups beta;
  for (const auto & band : encoded.bands) {
    beta.push_back(wavestream::decode_band(band));
    const auto reconstructed = wavestream::reconstruct_tile(beta, 4, 4, HAAR_LEVELS);
    std::cout << beta.size() << " groups: ";
    for (const auto value : reconstructed) {std::cout << static_cast<int>(value) << ' ';}
    std::cout << '\n';
  }
  return wavestream::decode_tile(encoded, HAAR_LEVELS + 1) == tile ? 0 : 1;
}
