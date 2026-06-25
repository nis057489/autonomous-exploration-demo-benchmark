#include "voxelcodec_ros/codec.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <unordered_set>

#include <nlohmann/json.hpp>
#include <zstd.h>

namespace voxelcodec_ros
{
  namespace
  {

    using json = nlohmann::json;

    void validate_data_type(const std::string &data_type)
    {
      if (
          data_type != kDataTypeUint8 && data_type != kDataTypeUint16 &&
          data_type != kDataTypeUint32 && data_type != kDataTypeUint64 &&
          data_type != kDataTypeFloat32 && data_type != kDataTypeFloat64)
      {
        throw std::runtime_error("unsupported data type: " + data_type);
      }
    }

    std::size_t data_type_size(const std::string &data_type)
    {
      if (data_type == kDataTypeUint8)
      {
        return 1;
      }
      if (data_type == kDataTypeUint16)
      {
        return 2;
      }
      if (data_type == kDataTypeUint32 || data_type == kDataTypeFloat32)
      {
        return 4;
      }
      if (data_type == kDataTypeUint64 || data_type == kDataTypeFloat64)
      {
        return 8;
      }
      throw std::runtime_error("unsupported data type: " + data_type);
    }

    void validate_encoding(const std::string &encoding)
    {
      if (
          encoding != kEncodingRawLE && encoding != kEncodingConstant &&
          encoding != kEncodingPalette && encoding != kEncodingDeltaVarint &&
          encoding != kEncodingHaarWavelet && encoding != kEncodingByteShuffle)
      {
        throw std::runtime_error("unsupported encoding: " + encoding);
      }
    }

    void validate_compression(const std::string &compression)
    {
      if (compression != kCompressionNone && compression != kCompressionZstd)
      {
        throw std::runtime_error("unsupported compression: " + compression);
      }
    }

    json metadata_to_json(const Metadata &metadata)
    {
      json object = json::object();
      for (const auto &item : metadata)
      {
        object[item.first] = item.second;
      }
      return object;
    }

    Metadata metadata_from_json(const json &object)
    {
      Metadata metadata;
      if (!object.is_object())
      {
        return metadata;
      }
      for (auto it = object.begin(); it != object.end(); ++it)
      {
        metadata.emplace(it.key(), it.value().get<std::string>());
      }
      return metadata;
    }

    json descriptor_to_json(const ChannelDescriptor &descriptor)
    {
      json item = json{{"name", descriptor.name},
                       {"semantic", descriptor.semantic},
                       {"data_type", descriptor.data_type},
                       {"encoding", descriptor.encoding},
                       {"compression", descriptor.compression},
                       {"element_count", descriptor.element_count},
                       {"payload_offset", descriptor.payload_offset},
                       {"compressed_size", descriptor.compressed_size},
                       {"uncompressed_size", descriptor.uncompressed_size}};
      if (!descriptor.metadata.empty())
      {
        item["metadata"] = metadata_to_json(descriptor.metadata);
      }
      return item;
    }

    ChannelDescriptor descriptor_from_json(const json &item)
    {
      ChannelDescriptor descriptor;
      descriptor.name = item.at("name").get<std::string>();
      descriptor.semantic = item.value("semantic", std::string{});
      descriptor.data_type = item.at("data_type").get<std::string>();
      descriptor.encoding = item.at("encoding").get<std::string>();
      descriptor.compression = item.at("compression").get<std::string>();
      descriptor.element_count = item.at("element_count").get<std::uint32_t>();
      descriptor.payload_offset = item.at("payload_offset").get<std::uint64_t>();
      descriptor.compressed_size = item.at("compressed_size").get<std::uint64_t>();
      descriptor.uncompressed_size = item.at("uncompressed_size").get<std::uint64_t>();
      if (item.contains("metadata"))
      {
        descriptor.metadata = metadata_from_json(item.at("metadata"));
      }
      else
      {
        descriptor.metadata.clear();
      }
      return descriptor;
    }

    json manifest_to_json(const Manifest &manifest)
    {
      json channels = json::array();
      for (const auto &descriptor : manifest.channels)
      {
        channels.push_back(descriptor_to_json(descriptor));
      }

      json item = json{{"format", manifest.format},
                       {"version", manifest.version},
                       {"voxel_count", manifest.voxel_count},
                       {"channels", std::move(channels)}};
      if (!manifest.metadata.empty())
      {
        item["metadata"] = metadata_to_json(manifest.metadata);
      }
      return item;
    }

    Manifest manifest_from_json(const json &item)
    {
      Manifest manifest;
      manifest.format = item.at("format").get<std::string>();
      manifest.version = item.at("version").get<std::uint16_t>();
      manifest.voxel_count = item.at("voxel_count").get<std::uint32_t>();
      manifest.channels.clear();
      for (const auto &channel : item.at("channels"))
      {
        manifest.channels.push_back(descriptor_from_json(channel));
      }
      if (item.contains("metadata"))
      {
        manifest.metadata = metadata_from_json(item.at("metadata"));
      }
      else
      {
        manifest.metadata.clear();
      }
      return manifest;
    }

    void store_u16(std::vector<std::uint8_t> &buffer, std::size_t offset, std::uint16_t value)
    {
      buffer[offset] = static_cast<std::uint8_t>(value & 0xffU);
      buffer[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
    }

    void store_u64(std::vector<std::uint8_t> &buffer, std::size_t offset, std::uint64_t value)
    {
      for (std::size_t index = 0; index < 8; ++index)
      {
        buffer[offset + index] = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
      }
    }

    std::uint16_t load_u16(const std::uint8_t *data)
    {
      return static_cast<std::uint16_t>(data[0]) |
             (static_cast<std::uint16_t>(data[1]) << 8U);
    }

    std::uint32_t load_u32(const std::uint8_t *data)
    {
      return static_cast<std::uint32_t>(data[0]) |
             (static_cast<std::uint32_t>(data[1]) << 8U) |
             (static_cast<std::uint32_t>(data[2]) << 16U) |
             (static_cast<std::uint32_t>(data[3]) << 24U);
    }

    std::uint64_t load_u64(const std::uint8_t *data)
    {
      std::uint64_t value = 0;
      for (std::size_t index = 0; index < 8; ++index)
      {
        value |= static_cast<std::uint64_t>(data[index]) << (index * 8U);
      }
      return value;
    }

    std::uint64_t zig_zag_decode(std::uint64_t value)
    {
      return (value >> 1U) ^ (~(value & 1U) + 1U);
    }

    std::uint64_t read_uvarint(const std::vector<std::uint8_t> &buffer, std::size_t &offset)
    {
      std::uint64_t value = 0;
      int shift = 0;
      while (offset < buffer.size())
      {
        const std::uint8_t byte = buffer[offset++];
        value |= static_cast<std::uint64_t>(byte & 0x7fU) << shift;
        if ((byte & 0x80U) == 0)
        {
          return value;
        }
        shift += 7;
        if (shift >= 64)
        {
          throw std::runtime_error("uvarint overflow");
        }
      }
      throw std::runtime_error("truncated uvarint payload");
    }

    template <typename T>
    std::vector<T> decode_integer_values(const std::vector<std::uint8_t> &raw)
    {
      if (raw.size() % sizeof(T) != 0)
      {
        throw std::runtime_error("payload alignment mismatch");
      }
      std::vector<T> values(raw.size() / sizeof(T));
      for (std::size_t index = 0; index < values.size(); ++index)
      {
        if constexpr (sizeof(T) == 1)
        {
          values[index] = raw[index];
        }
        else if constexpr (sizeof(T) == 2)
        {
          values[index] = static_cast<T>(load_u16(raw.data() + index * sizeof(T)));
        }
        else if constexpr (sizeof(T) == 4)
        {
          values[index] = static_cast<T>(load_u32(raw.data() + index * sizeof(T)));
        }
        else
        {
          values[index] = static_cast<T>(load_u64(raw.data() + index * sizeof(T)));
        }
      }
      return values;
    }

    std::vector<float> decode_float32_values(const std::vector<std::uint8_t> &raw)
    {
      if (raw.size() % sizeof(float) != 0)
      {
        throw std::runtime_error("payload alignment mismatch");
      }
      std::vector<float> values(raw.size() / sizeof(float));
      for (std::size_t index = 0; index < values.size(); ++index)
      {
        const std::uint32_t bits = load_u32(raw.data() + index * sizeof(float));
        std::memcpy(&values[index], &bits, sizeof(float));
      }
      return values;
    }

    std::vector<double> decode_float64_values(const std::vector<std::uint8_t> &raw)
    {
      if (raw.size() % sizeof(double) != 0)
      {
        throw std::runtime_error("payload alignment mismatch");
      }
      std::vector<double> values(raw.size() / sizeof(double));
      for (std::size_t index = 0; index < values.size(); ++index)
      {
        const std::uint64_t bits = load_u64(raw.data() + index * sizeof(double));
        std::memcpy(&values[index], &bits, sizeof(double));
      }
      return values;
    }

    ScalarBuffer decode_raw_values(const std::string &data_type, const std::vector<std::uint8_t> &raw)
    {
      if (data_type == kDataTypeUint8)
      {
        return decode_integer_values<std::uint8_t>(raw);
      }
      if (data_type == kDataTypeUint16)
      {
        return decode_integer_values<std::uint16_t>(raw);
      }
      if (data_type == kDataTypeUint32)
      {
        return decode_integer_values<std::uint32_t>(raw);
      }
      if (data_type == kDataTypeUint64)
      {
        return decode_integer_values<std::uint64_t>(raw);
      }
      if (data_type == kDataTypeFloat32)
      {
        return decode_float32_values(raw);
      }
      if (data_type == kDataTypeFloat64)
      {
        return decode_float64_values(raw);
      }
      throw std::runtime_error("unsupported data type: " + data_type);
    }

    template <typename T>
    ScalarBuffer make_constant_values(std::size_t count, T value)
    {
      return ScalarBuffer(std::vector<T>(count, value));
    }

    ScalarBuffer decode_constant_channel(const ChannelDescriptor &descriptor)
    {
      const auto it = descriptor.metadata.find(kConstantValueKey);
      if (it == descriptor.metadata.end())
      {
        throw std::runtime_error("missing constant_value metadata for channel " + descriptor.name);
      }
      const std::size_t count = descriptor.element_count;
      if (descriptor.data_type == kDataTypeUint8)
      {
        return make_constant_values<std::uint8_t>(count, static_cast<std::uint8_t>(std::stoul(it->second)));
      }
      if (descriptor.data_type == kDataTypeUint16)
      {
        return make_constant_values<std::uint16_t>(count, static_cast<std::uint16_t>(std::stoul(it->second)));
      }
      if (descriptor.data_type == kDataTypeUint32)
      {
        return make_constant_values<std::uint32_t>(count, static_cast<std::uint32_t>(std::stoul(it->second)));
      }
      if (descriptor.data_type == kDataTypeUint64)
      {
        return make_constant_values<std::uint64_t>(count, static_cast<std::uint64_t>(std::stoull(it->second)));
      }
      if (descriptor.data_type == kDataTypeFloat32)
      {
        return make_constant_values<float>(count, std::stof(it->second));
      }
      if (descriptor.data_type == kDataTypeFloat64)
      {
        return make_constant_values<double>(count, std::stod(it->second));
      }
      throw std::runtime_error("unsupported constant data type: " + descriptor.data_type);
    }

    std::vector<std::uint32_t> decode_palette_indices(
        const std::string &index_type,
        const std::vector<std::uint8_t> &raw)
    {
      if (index_type == kDataTypeUint8)
      {
        const auto values = decode_integer_values<std::uint8_t>(raw);
        return std::vector<std::uint32_t>(values.begin(), values.end());
      }
      if (index_type == kDataTypeUint16)
      {
        const auto values = decode_integer_values<std::uint16_t>(raw);
        return std::vector<std::uint32_t>(values.begin(), values.end());
      }
      if (index_type == kDataTypeUint32)
      {
        return decode_integer_values<std::uint32_t>(raw);
      }
      throw std::runtime_error("unsupported palette index type: " + index_type);
    }

    ScalarBuffer decode_palette_channel(
        const ChannelDescriptor &descriptor,
        const std::vector<std::uint8_t> &raw)
    {
      if (descriptor.data_type != kDataTypeUint32)
      {
        throw std::runtime_error("palette encoding supports uint32 logical values only");
      }
      const auto palette_it = descriptor.metadata.find(kPaletteValuesKey);
      if (palette_it == descriptor.metadata.end())
      {
        throw std::runtime_error("missing palette_values metadata for channel " + descriptor.name);
      }
      const auto type_it = descriptor.metadata.find(kPaletteIndexTypeKey);
      if (type_it == descriptor.metadata.end())
      {
        throw std::runtime_error("missing palette_index_type metadata for channel " + descriptor.name);
      }

      const auto palette_json = json::parse(palette_it->second);
      const auto palette = palette_json.get<std::vector<std::uint32_t> >();
      const auto indices = decode_palette_indices(type_it->second, raw);
      std::vector<std::uint32_t> values;
      values.reserve(indices.size());
      for (const auto index : indices)
      {
        if (index >= palette.size())
        {
          throw std::runtime_error("palette index out of range for channel " + descriptor.name);
        }
        values.push_back(palette[index]);
      }
      return values;
    }

    ScalarBuffer decode_delta_varint_channel(
        const ChannelDescriptor &descriptor,
        const std::vector<std::uint8_t> &raw)
    {
      if (descriptor.data_type != kDataTypeUint32)
      {
        throw std::runtime_error("delta-varint supports uint32 logical values only");
      }
      std::vector<std::uint32_t> values;
      values.reserve(descriptor.element_count);
      std::size_t offset = 0;
      std::int64_t previous = 0;
      while (values.size() < descriptor.element_count)
      {
        const auto encoded = read_uvarint(raw, offset);
        previous += static_cast<std::int64_t>(zig_zag_decode(encoded));
        if (previous < 0)
        {
          throw std::runtime_error("delta-varint decoded negative value");
        }
        values.push_back(static_cast<std::uint32_t>(previous));
      }
      if (offset != raw.size())
      {
        throw std::runtime_error("delta-varint payload has trailing bytes");
      }
      return values;
    }

    ScalarBuffer decode_byte_shuffle_channel(
        const ChannelDescriptor &descriptor,
        const std::vector<std::uint8_t> &raw)
    {
      if (descriptor.data_type != kDataTypeUint32)
      {
        throw std::runtime_error("byte-shuffle supports uint32 logical values only");
      }
      if (raw.size() != static_cast<std::size_t>(descriptor.element_count) * sizeof(std::uint32_t))
      {
        throw std::runtime_error("byte-shuffle payload size mismatch for channel: " + descriptor.name);
      }

      const std::size_t n = descriptor.element_count;
      std::vector<std::uint32_t> values(n);
      for (std::size_t i = 0; i < n; ++i)
      {
        values[i] =
            static_cast<std::uint32_t>(raw[i]) |
            (static_cast<std::uint32_t>(raw[n + i]) << 8U) |
            (static_cast<std::uint32_t>(raw[2 * n + i]) << 16U) |
            (static_cast<std::uint32_t>(raw[3 * n + i]) << 24U);
      }
      return values;
    }

    void haar_inverse_level(std::vector<std::int64_t> &coeffs, std::size_t full_len, std::size_t smooth_len)
    {
      if (full_len < 2)
      {
        return;
      }
      const std::size_t num_pairs = full_len / 2;
      std::vector<std::int64_t> smooth(coeffs.begin(), coeffs.begin() + static_cast<std::ptrdiff_t>(smooth_len));
      std::vector<std::int64_t> detail(
          coeffs.begin() + static_cast<std::ptrdiff_t>(smooth_len),
          coeffs.begin() + static_cast<std::ptrdiff_t>(smooth_len + num_pairs));
      for (std::size_t i = 0; i < num_pairs; ++i)
      {
        const std::int64_t a = smooth[i] - (detail[i] >> 1);
        const std::int64_t b = a + detail[i];
        coeffs[2 * i] = a;
        coeffs[2 * i + 1] = b;
      }
      if (full_len % 2 == 1)
      {
        coeffs[full_len - 1] = smooth[smooth_len - 1];
      }
    }

    ScalarBuffer decode_haar_wavelet_channel(
        const ChannelDescriptor &descriptor,
        const std::vector<std::uint8_t> &raw)
    {
      if (descriptor.data_type != kDataTypeUint32)
      {
        throw std::runtime_error("haar-wavelet supports uint32 logical values only");
      }

      const auto levels_it = descriptor.metadata.find(kHaarLevelsKey);
      const std::size_t levels = (levels_it != descriptor.metadata.end())
                                     ? std::stoul(levels_it->second)
                                     : 0;

      const auto orig_it = descriptor.metadata.find(kHaarOriginalLengthKey);
      const std::size_t original_len = (orig_it != descriptor.metadata.end())
                                           ? std::stoul(orig_it->second)
                                           : descriptor.element_count;

      // Read zigzag-varint encoded coefficients
      std::vector<std::int64_t> coeffs(original_len, 0);
      std::size_t offset = 0;
      for (std::size_t i = 0; i < original_len; ++i)
      {
        const auto encoded = read_uvarint(raw, offset);
        coeffs[i] = static_cast<std::int64_t>(zig_zag_decode(encoded));
      }

      if (levels == 0)
      {
        std::vector<std::uint32_t> values(original_len);
        for (std::size_t i = 0; i < original_len; ++i)
        {
          values[i] = static_cast<std::uint32_t>(coeffs[i] & 0xFFFFFFFF);
        }
        return values;
      }

      // Compute smooth lengths for each level
      std::vector<std::size_t> smooth_lens(levels + 1);
      smooth_lens[0] = original_len;
      for (std::size_t i = 1; i <= levels; ++i)
      {
        smooth_lens[i] = (smooth_lens[i - 1] + 1) / 2;
      }

      // Inverse wavelet transform: from coarsest to finest
      for (std::size_t level = levels; level-- > 0;)
      {
        const std::size_t full_len = smooth_lens[level];
        const std::size_t sl = smooth_lens[level + 1];
        haar_inverse_level(coeffs, full_len, sl);
      }

      std::vector<std::uint32_t> values(original_len);
      for (std::size_t i = 0; i < original_len; ++i)
      {
        values[i] = static_cast<std::uint32_t>(coeffs[i] & 0xFFFFFFFF);
      }
      return values;
    }

  } // namespace

  void validate_descriptor(const ChannelDescriptor &descriptor)
  {
    if (descriptor.name.empty())
    {
      throw std::runtime_error("channel name is required");
    }
    validate_data_type(descriptor.data_type);
    validate_encoding(descriptor.encoding);
    validate_compression(descriptor.compression);
  }

  void validate_manifest(const Manifest &manifest)
  {
    if (manifest.format.empty())
    {
      throw std::runtime_error("manifest format is required");
    }
    std::unordered_set<std::string> seen;
    for (const auto &descriptor : manifest.channels)
    {
      validate_descriptor(descriptor);
      if (!seen.insert(descriptor.name).second)
      {
        throw std::runtime_error("duplicate channel name: " + descriptor.name);
      }
    }
  }

  std::vector<std::uint8_t> compress_payload(
      const std::string &compression,
      const std::vector<std::uint8_t> &raw_payload)
  {
    validate_compression(compression);
    if (compression == kCompressionNone)
    {
      return raw_payload;
    }

    std::vector<std::uint8_t> compressed(ZSTD_compressBound(raw_payload.size()));
    const auto result = ZSTD_compress(
        compressed.data(), compressed.size(), raw_payload.data(), raw_payload.size(), 9);
    if (ZSTD_isError(result))
    {
      throw std::runtime_error(ZSTD_getErrorName(result));
    }
    compressed.resize(result);
    return compressed;
  }

  std::vector<std::uint8_t> decompress_payload(
      const ChannelDescriptor &descriptor,
      const std::vector<std::uint8_t> &compressed_payload)
  {
    if (descriptor.compression == kCompressionNone)
    {
      return compressed_payload;
    }
    if (descriptor.compression != kCompressionZstd)
    {
      throw std::runtime_error("unsupported compression: " + descriptor.compression);
    }

    std::vector<std::uint8_t> raw(descriptor.uncompressed_size);
    const auto result = ZSTD_decompress(
        raw.data(), raw.size(), compressed_payload.data(), compressed_payload.size());
    if (ZSTD_isError(result))
    {
      throw std::runtime_error(ZSTD_getErrorName(result));
    }
    if (result != descriptor.uncompressed_size)
    {
      throw std::runtime_error("decompressed payload size mismatch");
    }
    return raw;
  }

  std::vector<std::uint8_t> encode_archive(
      const Manifest &input_manifest,
      const std::vector<EncodedChannel> &channels)
  {
    if (input_manifest.channels.size() != channels.size())
    {
      throw std::runtime_error("manifest channel count does not match payload count");
    }

    Manifest manifest = input_manifest;
    validate_manifest(manifest);

    std::vector<std::vector<std::uint8_t> > compressed_payloads;
    compressed_payloads.reserve(channels.size());
    std::uint64_t offset = 0;
    for (std::size_t index = 0; index < channels.size(); ++index)
    {
      const auto &channel = channels[index];
      if (channel.descriptor.name != manifest.channels[index].name)
      {
        throw std::runtime_error("channel ordering does not match manifest");
      }
      auto compressed = compress_payload(channel.descriptor.compression, channel.payload);
      auto &descriptor = manifest.channels[index];
      descriptor.payload_offset = offset;
      descriptor.uncompressed_size = channel.payload.size();
      descriptor.compressed_size = compressed.size();
      offset += descriptor.compressed_size;
      compressed_payloads.push_back(std::move(compressed));
    }

    const auto manifest_bytes = manifest_to_json(manifest).dump();
    std::vector<std::uint8_t> archive(kHeaderSize + manifest_bytes.size());
    std::copy_n(kArchiveMagic, 4, archive.begin());
    store_u16(archive, 4, kArchiveVersion);
    store_u16(archive, 6, 0);
    store_u64(archive, 8, manifest_bytes.size());
    std::copy(manifest_bytes.begin(), manifest_bytes.end(), archive.begin() + static_cast<std::ptrdiff_t>(kHeaderSize));

    for (const auto &payload : compressed_payloads)
    {
      archive.insert(archive.end(), payload.begin(), payload.end());
    }
    return archive;
  }

  Archive read_archive(const std::vector<std::uint8_t> &archive_bytes)
  {
    if (archive_bytes.size() < kHeaderSize)
    {
      throw std::runtime_error("archive too small");
    }
    if (!std::equal(kArchiveMagic, kArchiveMagic + 4, archive_bytes.begin()))
    {
      throw std::runtime_error("invalid archive magic");
    }
    const auto version = load_u16(archive_bytes.data() + 4);
    if (version != kArchiveVersion)
    {
      throw std::runtime_error("unsupported archive version");
    }
    const auto manifest_length = load_u64(archive_bytes.data() + 8);
    const auto payload_base = kHeaderSize + manifest_length;
    if (payload_base > archive_bytes.size())
    {
      throw std::runtime_error("manifest length points outside archive");
    }

    const auto manifest_json = std::string(
        archive_bytes.begin() + static_cast<std::ptrdiff_t>(kHeaderSize),
        archive_bytes.begin() + static_cast<std::ptrdiff_t>(payload_base));
    const auto manifest = manifest_from_json(json::parse(manifest_json));
    validate_manifest(manifest);

    Archive archive;
    archive.manifest = manifest;
    for (const auto &descriptor : manifest.channels)
    {
      const auto start = payload_base + descriptor.payload_offset;
      const auto end = start + descriptor.compressed_size;
      if (end > archive_bytes.size())
      {
        throw std::runtime_error("channel payload points outside archive: " + descriptor.name);
      }
      archive.payloads.emplace(
          descriptor.name,
          std::vector<std::uint8_t>(
              archive_bytes.begin() + static_cast<std::ptrdiff_t>(start),
              archive_bytes.begin() + static_cast<std::ptrdiff_t>(end)));
    }
    return archive;
  }

  DecodedChannel decode_channel(
      const ChannelDescriptor &descriptor,
      const std::vector<std::uint8_t> &compressed_payload)
  {
    validate_descriptor(descriptor);
    const auto raw = decompress_payload(descriptor, compressed_payload);
    if (raw.size() != descriptor.uncompressed_size)
    {
      throw std::runtime_error("uncompressed size mismatch for channel: " + descriptor.name);
    }

    DecodedChannel decoded;
    decoded.descriptor = descriptor;
    if (descriptor.encoding == kEncodingRawLE)
    {
      if (raw.size() % data_type_size(descriptor.data_type) != 0)
      {
        throw std::runtime_error("raw payload alignment mismatch for channel: " + descriptor.name);
      }
      decoded.values = decode_raw_values(descriptor.data_type, raw);
      return decoded;
    }
    if (descriptor.encoding == kEncodingConstant)
    {
      decoded.values = decode_constant_channel(descriptor);
      return decoded;
    }
    if (descriptor.encoding == kEncodingPalette)
    {
      decoded.values = decode_palette_channel(descriptor, raw);
      return decoded;
    }
    if (descriptor.encoding == kEncodingDeltaVarint)
    {
      decoded.values = decode_delta_varint_channel(descriptor, raw);
      return decoded;
    }
    if (descriptor.encoding == kEncodingHaarWavelet)
    {
      decoded.values = decode_haar_wavelet_channel(descriptor, raw);
      return decoded;
    }
    if (descriptor.encoding == kEncodingByteShuffle)
    {
      decoded.values = decode_byte_shuffle_channel(descriptor, raw);
      return decoded;
    }
    throw std::runtime_error("unsupported encoding: " + descriptor.encoding);
  }

  std::unordered_map<std::string, DecodedChannel> decode_selected(
      const Archive &archive,
      const std::vector<std::string> &channel_names)
  {
    std::unordered_set<std::string> requested(channel_names.begin(), channel_names.end());
    std::unordered_map<std::string, DecodedChannel> decoded;
    for (const auto &descriptor : archive.manifest.channels)
    {
      if (requested.find(descriptor.name) == requested.end())
      {
        continue;
      }
      const auto payload_it = archive.payloads.find(descriptor.name);
      if (payload_it == archive.payloads.end())
      {
        throw std::runtime_error("missing payload for channel: " + descriptor.name);
      }
      decoded.emplace(descriptor.name, decode_channel(descriptor, payload_it->second));
    }
    for (const auto &name : channel_names)
    {
      if (decoded.find(name) == decoded.end())
      {
        throw std::runtime_error("requested channel not found: " + name);
      }
    }
    return decoded;
  }

  int haar_max_bands(const ChannelDescriptor &descriptor)
  {
    if (descriptor.encoding != kEncodingHaarWavelet)
    {
      return 0;
    }
    auto it = descriptor.metadata.find(kHaarLevelsKey);
    int levels = (it != descriptor.metadata.end()) ? std::stoi(it->second) : 0;
    return levels + 1;
  }

  DecodedChannel decode_haar_progressive(
      const ChannelDescriptor &descriptor,
      const std::vector<std::uint8_t> &compressed_payload,
      int max_bands)
  {
    if (descriptor.encoding != kEncodingHaarWavelet)
    {
      return decode_channel(descriptor, compressed_payload);
    }
    if (descriptor.data_type != kDataTypeUint32)
    {
      throw std::runtime_error("haar-wavelet supports uint32 logical values only");
    }

    const auto raw = decompress_payload(descriptor, compressed_payload);

    auto levels_it = descriptor.metadata.find(kHaarLevelsKey);
    const int levels = (levels_it != descriptor.metadata.end())
                           ? std::stoi(levels_it->second)
                           : 0;

    auto orig_it = descriptor.metadata.find(kHaarOriginalLengthKey);
    const std::size_t original_len = (orig_it != descriptor.metadata.end())
                                         ? std::stoul(orig_it->second)
                                         : descriptor.element_count;

    // Read all zigzag-varint coefficients
    std::vector<std::int64_t> coeffs(original_len, 0);
    std::size_t offset = 0;
    for (std::size_t i = 0; i < original_len; ++i)
    {
      const auto encoded = read_uvarint(raw, offset);
      coeffs[i] = static_cast<std::int64_t>(zig_zag_decode(encoded));
    }

    if (levels == 0 || max_bands <= 0 || max_bands > levels + 1)
    {
      // Full decode
      for (std::size_t level = static_cast<std::size_t>(levels); level-- > 0;)
      {
        std::vector<std::size_t> sl(levels + 1);
        sl[0] = original_len;
        for (int i = 1; i <= levels; ++i)
          sl[i] = (sl[i - 1] + 1) / 2;
        haar_inverse_level(coeffs, sl[level], sl[level + 1]);
      }
      std::vector<std::uint32_t> values(original_len);
      for (std::size_t i = 0; i < original_len; ++i)
      {
        values[i] = static_cast<std::uint32_t>(coeffs[i] & 0xFFFFFFFF);
      }
      DecodedChannel decoded;
      decoded.descriptor = descriptor;
      decoded.values = std::move(values);
      return decoded;
    }

    // Compute smooth lengths
    std::vector<std::size_t> smooth_lens(levels + 1);
    smooth_lens[0] = original_len;
    for (int i = 1; i <= levels; ++i)
    {
      smooth_lens[i] = (smooth_lens[i - 1] + 1) / 2;
    }

    // Band boundaries: band_boundaries[k] = smooth_lens[levels - k + 1] for k>=1
    // band_boundaries[0] = 0, band_boundaries[1] = smooth_lens[levels], ...
    // band_boundaries[levels+1] = original_len
    std::vector<std::size_t> band_boundaries(levels + 2, 0);
    for (int i = 0; i <= levels; ++i)
    {
      band_boundaries[i + 1] = smooth_lens[levels - i];
    }

    int bands = std::max(1, std::min(max_bands, levels + 1));
    std::size_t output_len = band_boundaries[bands];

    // Work on coefficients up to output_len
    std::vector<std::int64_t> work(coeffs.begin(), coeffs.begin() + static_cast<std::ptrdiff_t>(output_len));

    for (int i = 0; i < bands - 1; ++i)
    {
      int level = levels - 1 - i;
      haar_inverse_level(work, smooth_lens[level], smooth_lens[level + 1]);
    }

    std::vector<std::uint32_t> values(output_len);
    for (std::size_t i = 0; i < output_len; ++i)
    {
      values[i] = static_cast<std::uint32_t>(work[i] & 0xFFFFFFFF);
    }

    DecodedChannel decoded;
    decoded.descriptor = descriptor;
    decoded.descriptor.element_count = static_cast<std::uint32_t>(output_len);
    decoded.values = std::move(values);
    return decoded;
  }

} // namespace voxelcodec_ros