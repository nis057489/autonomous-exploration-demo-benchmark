#pragma once

// 3D (sparse voxel) mode for the visual test, alongside the existing 2D dense
// occupancy-grid mode.
//
// The two modes are genuinely different representations, not one algorithm with
// an extra axis, and the demo exists largely to keep that visible:
//
//   2D  a dense occupancy grid, split into tiles, each tile carrying its own 2D
//       Haar pyramid (row pass + column pass), streamed band by band.
//   3D  a SPARSE voxel list split into per-axis coordinate channels (x, y, z),
//       each channel Haar-transformed independently along its own sequence.
//       Progressive decoding takes a prefix of a channel's bands: the first k
//       bands are a lossless representation of the coordinate sequence
//       downsampled by 2^(levels-k), so fewer bands means fewer, coarser
//       voxels -- not a blurrier dense field.
//
// The run-length structure a 3D map compresses on lives in the coordinate
// SEQUENCES (column sorting makes a floor-to-ceiling obstacle emit z, z+1,
// z+2, ... within one column), not in the voxel field, which is why reasoning
// about this mode as though it were a dense volume gives the wrong answer.

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/haar_forward.hpp"
#include "voxelcodec_ros/types.hpp"

namespace vxch_test
{

struct VoxelCloud
{
  std::vector<std::uint32_t> x, y, z;
  std::size_t count() const {return x.size();}
};

inline std::vector<std::uint8_t> read_binary_file(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open: " + path);
  }
  return std::vector<std::uint8_t>(
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

// Pulls one uint32 coordinate channel out of a decoded channel, whatever
// integer width the encoder narrowed it to (best-fit picks uint8/uint16/uint32
// by magnitude, so a small map's channel legitimately arrives narrower).
inline std::vector<std::uint32_t> to_u32(const voxelcodec_ros::DecodedChannel & channel)
{
  return std::visit(
    [](const auto & values) {
      std::vector<std::uint32_t> out;
      out.reserve(values.size());
      for (const auto v : values) {out.push_back(static_cast<std::uint32_t>(v));}
      return out;
    }, channel.values);
}

// Decodes the x/y/z channels of a .vxch archive at a given level of detail.
// max_bands <= 0 means "every band" (full resolution); 1 is the coarsest.
// Channels that are not haar-wavelet encoded (e.g. column-run or delta-varint,
// which the Go encoder selects for z after column sorting) have no band ladder,
// so they always decode in full.
inline VoxelCloud decode_voxels(
  const voxelcodec_ros::Archive & archive, int max_bands,
  std::size_t * out_compressed_bytes = nullptr)
{
  VoxelCloud cloud;
  std::size_t bytes = 0;
  const char * names[3] = {"x", "y", "z"};
  std::vector<std::uint32_t> * targets[3] = {&cloud.x, &cloud.y, &cloud.z};

  for (int i = 0; i < 3; ++i) {
    const voxelcodec_ros::ChannelDescriptor * descriptor = nullptr;
    for (const auto & candidate : archive.manifest.channels) {
      if (candidate.name == names[i]) {descriptor = &candidate;}
    }
    if (descriptor == nullptr) {
      throw std::runtime_error(std::string("archive has no '") + names[i] + "' channel");
    }
    const auto & payload = archive.payloads.at(descriptor->name);
    bytes += payload.size();

    const int available = voxelcodec_ros::haar_max_bands(*descriptor);
    if (available > 0 && max_bands > 0 && max_bands < available) {
      *targets[i] = to_u32(
        voxelcodec_ros::decode_haar_progressive(*descriptor, payload, max_bands));
    } else {
      *targets[i] = to_u32(voxelcodec_ros::decode_channel(*descriptor, payload));
    }
  }

  // A partial Haar prefix yields a shorter (downsampled) coordinate sequence,
  // and the three channels need not shorten identically, so pair only as many
  // voxels as all three axes actually supply.
  const std::size_t n = std::min(cloud.x.size(), std::min(cloud.y.size(), cloud.z.size()));
  cloud.x.resize(n);
  cloud.y.resize(n);
  cloud.z.resize(n);
  if (out_compressed_bytes != nullptr) {*out_compressed_bytes = bytes;}
  return cloud;
}


// Bytes a progressive sender would actually transmit to reach a given level of
// detail on one channel.
//
// A 1D Haar channel stores its coefficients coarsest-first in a single
// zigzag-varint stream, so "the first k bands" is literally a PREFIX of that
// stream -- there is no per-band framing to add up. Walking the varints and
// stopping after the prefix's coefficient count therefore gives the real
// uncompressed size; re-compressing just that prefix gives the on-wire size.
//
// Returns {0, 0} for a channel with no band ladder (column-run, delta-varint,
// raw-le): those have no partial form, so any level of detail costs the whole
// payload, which the caller accounts for separately.
struct BandPrefixBytes
{
  std::size_t varint_bytes{0};
  std::size_t compressed_bytes{0};
  std::size_t coefficients{0};
};

inline BandPrefixBytes haar_prefix_bytes(
  const voxelcodec_ros::ChannelDescriptor & descriptor,
  const std::vector<std::uint8_t> & compressed_payload,
  int max_bands)
{
  BandPrefixBytes out;
  if (descriptor.encoding != voxelcodec_ros::kEncodingHaarWavelet) {
    return out;
  }
  const auto levels_it = descriptor.metadata.find(voxelcodec_ros::kHaarLevelsKey);
  const int levels = (levels_it != descriptor.metadata.end())
    ? std::stoi(levels_it->second) : 0;
  if (levels <= 0) {
    return out;
  }
  const auto len_it = descriptor.metadata.find(voxelcodec_ros::kHaarOriginalLengthKey);
  const std::size_t original_len = (len_it != descriptor.metadata.end())
    ? static_cast<std::size_t>(std::stoul(len_it->second))
    : descriptor.element_count;

  // Smooth-length chain, matching decode_haar_progressive_1d: sl[0] is the full
  // sequence and sl[levels] the coarsest LL. Band 1 (the coarsest) is the first
  // sl[levels] coefficients; each further band appends one detail level.
  std::vector<std::size_t> sl(static_cast<std::size_t>(levels) + 1);
  sl[0] = original_len;
  for (int i = 1; i <= levels; ++i) {
    sl[static_cast<std::size_t>(i)] = (sl[static_cast<std::size_t>(i - 1)] + 1) / 2;
  }
  const int bands = (max_bands <= 0 || max_bands > levels + 1) ? levels + 1 : max_bands;
  const std::size_t prefix = (bands >= levels + 1)
    ? original_len
    : sl[static_cast<std::size_t>(levels - bands + 1)];

  const auto raw = voxelcodec_ros::decompress_payload(descriptor, compressed_payload);
  std::size_t offset = 0;
  for (std::size_t i = 0; i < prefix && offset < raw.size(); ++i) {
    while (offset < raw.size() && (raw[offset] & 0x80U) != 0) {++offset;}
    if (offset < raw.size()) {++offset;}
  }
  out.coefficients = prefix;
  out.varint_bytes = offset;
  out.compressed_bytes = voxelcodec_ros::compress_payload(
    descriptor.compression,
    std::vector<std::uint8_t>(raw.begin(), raw.begin() + static_cast<std::ptrdiff_t>(offset))
  ).size();
  return out;
}


// --- channel-separation ablation --------------------------------------------
// Does splitting the coordinates into per-axis channels actually buy anything,
// or would one blob of interleaved triples compress just as well?
//
// Three arrangements of the SAME coordinate values, each varint-packed and
// zstd-compressed, so the only variable is how the bytes are grouped:
//
//   interleaved   x0,y0,z0, x1,y1,z1, ...   one stream, one compressor
//   concatenated  x0..xn, y0..yn, z0..zn    one stream, one compressor
//   split         x0..xn | y0..yn | z0..zn  three streams, three compressors
//
// interleaved vs concatenated isolates GROUPING: whether merely keeping an
// axis's values adjacent (so its values sit in one run rather than every third
// position) is what helps. concatenated vs split then isolates SEPARATION
// proper: whether giving each axis its own compression context adds anything on
// top of grouping. Reporting both matters -- if concatenated ~= split, the win
// is ordering and a single-stream encoder would do just as well.
struct SeparationAblation
{
  std::size_t interleaved{0};
  std::size_t concatenated{0};
  std::size_t split{0};
  std::size_t split_x{0};
  std::size_t split_y{0};
  std::size_t split_z{0};
  std::size_t voxels{0};
};

inline SeparationAblation separation_ablation(
  const VoxelCloud & cloud, const std::string & compression)
{
  SeparationAblation out;
  out.voxels = cloud.count();
  const std::size_t n = cloud.count();
  if (n == 0) {
    return out;
  }

  const auto pack = [](const std::vector<std::int64_t> & values) {
      return voxelcodec_ros::zigzag_varint_encode(values);
    };
  const auto sized = [&compression](const std::vector<std::uint8_t> & bytes) {
      return voxelcodec_ros::compress_payload(compression, bytes).size();
    };

  std::vector<std::int64_t> xs(n), ys(n), zs(n), inter(n * 3), concat(n * 3);
  for (std::size_t i = 0; i < n; ++i) {
    xs[i] = static_cast<std::int64_t>(cloud.x[i]);
    ys[i] = static_cast<std::int64_t>(cloud.y[i]);
    zs[i] = static_cast<std::int64_t>(cloud.z[i]);
    inter[i * 3 + 0] = xs[i];
    inter[i * 3 + 1] = ys[i];
    inter[i * 3 + 2] = zs[i];
    concat[i] = xs[i];
    concat[n + i] = ys[i];
    concat[2 * n + i] = zs[i];
  }

  out.interleaved = sized(pack(inter));
  out.concatenated = sized(pack(concat));
  out.split_x = sized(pack(xs));
  out.split_y = sized(pack(ys));
  out.split_z = sized(pack(zs));
  out.split = out.split_x + out.split_y + out.split_z;
  return out;
}

// Plain-text voxel dump the GUI reads: one header line then "x y z" per voxel.
inline void write_voxels(const std::string & path, const VoxelCloud & cloud)
{
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot write: " + path);
  }
  stream << "voxels " << cloud.count() << "\n";
  for (std::size_t i = 0; i < cloud.count(); ++i) {
    stream << cloud.x[i] << ' ' << cloud.y[i] << ' ' << cloud.z[i] << '\n';
  }
}

}  // namespace vxch_test
