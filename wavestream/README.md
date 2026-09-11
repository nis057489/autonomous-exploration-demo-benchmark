# WaveStream: the paper's tile codec

The core codec is a single C++17 header:
[`wavestream/codec.hpp`](../exploration_packages/vxch/voxelcodec_ros/include/wavestream/codec.hpp).
It implements the paper's **Tile encoding** and **Progressive tile reconstruction**
algorithms. The production occupancy-grid adapters use this same implementation.
All WaveStream functions are inline. Zstd is the only external library required;
ROS, JSON and `codec.cpp` are not required to use the standalone codec.

## Paper to code

| Paper operation / notation | Function or data |
| --- | --- |
| Occupancy tile $u$ | Row-major `std::vector<std::int8_t>`, values −1 (unknown) and 0…100 |
| Integer embedding $\phi(u)=u+1$ | `occupancy_to_embedded` |
| $L=\texttt{HAAR\_LEVELS}$ | `haar_levels` argument; counts the initial decomposition |
| $d=b-a$, $s=a+\lfloor d/2\rfloor$ | `haar_forward_level` |
| Row pass, then column pass | `haar_forward_2d_level` |
| Recurse on the approximation for $L$ levels | `forward_transform` |
| $\beta^{(0)}$ | Final LL approximation |
| $\beta^{(j)}$, $j=1,\ldots,L$ | HL, LH, HH concatenated at level $L-j+1$ |
| $p^{(j)}=\mathrm{Zstd}(\mathrm{Varint}(\mathrm{ZigZag}(\beta^{(j)})))$ | `encode_band` |
| Complete tile encoding | `encode_tile` |
| Decode one arriving group | `decode_band` |
| Invert the available prefix $\beta^{(0)},\ldots,\beta^{(k-1)}$ | `reconstruct_haar_from_bands` |
| Nearest-neighbor resampling, then $\phi^{-1}$ | `upsample_occupancy` |
| Reconstruct from a decoded prefix | `reconstruct_tile` |
| Decode and reconstruct the first $k$ encoded groups | `decode_tile` |

For $L=3$, the groups are `[LL_3, details_3, details_2, details_1]`.
One group yields the coarsest approximation; four yield full resolution.
Each decomposition changes dimensions to `(ceil(w/2), ceil(h/2))`.
Unpaired samples survive in the approximation, so odd-sized boundary tiles and
one-cell axes are supported. Deep decompositions can produce empty detail groups.
Full reconstruction is exact for valid occupancy input when all groups describe
the same tile state. Partial reconstruction approximates the embedded values;
it does not preserve an independent unknown-cell mask.

## Minimal use

```cpp
#include <wavestream/codec.hpp>

constexpr int HAAR_LEVELS = 2;
std::vector<std::int8_t> u{
    -1, -1, 100, 100,
    -1, -1, 100, 100,
     0,  0,  50,  50,
     0,  0,  50,  50};

const auto encoded = wavestream::encode_tile(u, 4, 4, HAAR_LEVELS);
const auto coarse = wavestream::decode_tile(encoded, 1);
const auto exact = wavestream::decode_tile(encoded, HAAR_LEVELS + 1);
```

Both results contain `width * height` occupancy cells: the coarse result is
upsampled to the original tile extent. `k=0` is rejected rather than interpreted
as full reconstruction. Invalid geometry, incomplete prefixes, and malformed
coefficient payloads throw exceptions.

For streaming, decode each group with `decode_band`, cache it by tile and group
index, then pass the longest contiguous prefix to `reconstruct_tile`. The
[executable example](examples/paper_pipeline.cpp) follows those steps explicitly.
`EncodedBand` carries payload bytes, coefficient count and uncompressed byte count;
`EncodedTile` adds dimensions and depth. These are in-memory structures, not a new
wire envelope. A transport must carry that metadata plus tile identity and placement.
The convenience API uses Zig-zag/varint and Zstd level 9, matching production defaults.

## Relationship to the running system

The existing ROS interfaces remain source-compatible adapters:

- `haar_forward.hpp::make_haar_bands` calls the extracted transform and packing
  functions, then attaches VXCH channel metadata. The existing fixed-width and
  uncompressed ablation modes remain available through this adapter.
- `codec.cpp` uses the shared inverse for archive decoding and delegates payload
  compression/decompression to the header.
- `TileReconstructor` uses the shared coefficient decoding, inverse transform and
  occupancy upsampling, while retaining geometry and cache management.
- `TileScheduler` retains fixed-lattice tiling, change detection and scheduling.

Tiling, scheduling, manifests, transport, and old-state fallback are outside the
per-tile codec. The receiver may combine groups from different updates; the codec
cannot establish snapshot consistency or acknowledged delivery. Tile placement is
handled by the caller. The default `smart` scheduling policy is unchanged.

The extraction also rejects truncated/overflowing varints and short fixed-width
payloads instead of accepting incomplete coefficients or reading beyond a payload.

## Build and verify

From the repository root, with a C++17 compiler, CMake and Zstd installed:

```sh
cmake -S wavestream -B /tmp/wavestream-build
cmake --build /tmp/wavestream-build
ctest --test-dir /tmp/wavestream-build --output-on-failure
/tmp/wavestream-build/wavestream_example
```

On Homebrew installations, add `-DCMAKE_PREFIX_PATH=/opt/homebrew` if necessary.
The `wavestream::wavestream` CMake target is an `INTERFACE` library that propagates
C++17, includes and Zstd linkage. Disable `WAVESTREAM_BUILD_TESTS` and
`WAVESTREAM_BUILD_EXAMPLE` when embedding only the library. The existing ROS package
installation also installs the header. Alternatively, copy the single header into
your include path and link `-lzstd`.

Without CMake, the example can be compiled directly (add your Zstd include/library
paths if they are not in the compiler's search path):

```sh
c++ -std=c++17 -Iexploration_packages/vxch/voxelcodec_ros/include \
  wavestream/examples/paper_pipeline.cpp -lzstd -o /tmp/wavestream-example
```

Tests cover a hand-calculated Haar example, odd and one-cell dimensions, every
progressive prefix, unknown/free/occupied constant tiles, exact round trips,
malformed payloads, and a two-translation-unit link without `codec.cpp`.
Adapter tests check payload parity and tiled sender/receiver reconstruction across
both compression and packing modes, including negative origins and clipped tiles.
Only the adapter test links the legacy archive codec and uses the repository's
vendored JSON header. Neither test requires ROS.
