#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace voxelcodec_ros
{

  inline constexpr char kArchiveMagic[] = "VXCH";
  inline constexpr std::uint16_t kArchiveVersion = 1;
  inline constexpr std::size_t kHeaderSize = 16;

  inline constexpr char kFormat[] = "voxel-channel-archive";
  inline constexpr char kEncodingRawLE[] = "raw-le";
  inline constexpr char kEncodingConstant[] = "constant";
  inline constexpr char kEncodingPalette[] = "palette";
  inline constexpr char kEncodingDeltaVarint[] = "delta-varint";
  inline constexpr char kEncodingHaarWavelet[] = "haar-wavelet";
  inline constexpr char kEncodingByteShuffle[] = "byte-shuffle";

  inline constexpr char kCompressionNone[] = "none";
  inline constexpr char kCompressionZstd[] = "zstd";

  inline constexpr char kDataTypeUint8[] = "uint8";
  inline constexpr char kDataTypeUint16[] = "uint16";
  inline constexpr char kDataTypeUint32[] = "uint32";
  inline constexpr char kDataTypeUint64[] = "uint64";
  inline constexpr char kDataTypeFloat32[] = "float32";
  inline constexpr char kDataTypeFloat64[] = "float64";

  inline constexpr char kConstantValueKey[] = "constant_value";
  inline constexpr char kPaletteValuesKey[] = "palette_values";
  inline constexpr char kPaletteIndexTypeKey[] = "palette_index_type";
  inline constexpr char kHaarLevelsKey[] = "haar_levels";
  inline constexpr char kHaarOriginalLengthKey[] = "haar_original_length";

  using Metadata = std::unordered_map<std::string, std::string>;
  using ScalarBuffer = std::variant<
      std::vector<std::uint8_t>,
      std::vector<std::uint16_t>,
      std::vector<std::uint32_t>,
      std::vector<std::uint64_t>,
      std::vector<float>,
      std::vector<double> >;

  struct ChannelDescriptor
  {
    std::string name;
    std::string semantic;
    std::string data_type;
    std::string encoding;
    std::string compression{kCompressionNone};
    std::uint32_t element_count{};
    std::uint64_t payload_offset{};
    std::uint64_t compressed_size{};
    std::uint64_t uncompressed_size{};
    Metadata metadata;
  };

  struct Manifest
  {
    std::string format{kFormat};
    std::uint16_t version{kArchiveVersion};
    std::uint32_t voxel_count{};
    std::vector<ChannelDescriptor> channels;
    Metadata metadata;
  };

  struct EncodedChannel
  {
    ChannelDescriptor descriptor;
    std::vector<std::uint8_t> payload;
  };

  struct Archive
  {
    Manifest manifest;
    std::unordered_map<std::string, std::vector<std::uint8_t> > payloads;
  };

  struct DecodedChannel
  {
    ChannelDescriptor descriptor;
    ScalarBuffer values;
  };

} // namespace voxelcodec_ros