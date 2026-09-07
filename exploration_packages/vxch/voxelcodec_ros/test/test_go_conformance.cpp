#include <gtest/gtest.h>

#include <cstdint>
#include <variant>
#include <string>
#include <vector>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/types.hpp"

// Cross-implementation conformance vectors. The byte arrays below are the
// VERBATIM output of the Go encoder in voxel_channels
// (voxelcodec.ColumnRunChannel / XORByteShuffleChannel MarshalBinary), so
// these tests fail if the C++ decoders drift from the format the Go side
// actually writes -- which is what the ROS nodes have to read.
//
// column-run is the encoding the Go side selects for z after column sorting,
// and until now C++ rejected it outright ("unsupported encoding: column-run"),
// so any 3D archive using it was undecodable here.

namespace
{
struct Vector
{
  const char * name;
  std::vector<std::uint32_t> values;
  std::vector<std::uint8_t> column_run;
  std::vector<std::uint8_t> xor_shuffle;
};

const std::vector<Vector> & vectors()
{
  static const std::vector<Vector> v{
  // Go encoder output for large_values
  {"large_values", {0u, 1u, 2u, 4294967295u}, {0u, 3u, 255u, 255u, 255u, 255u, 15u, 1u}, {0u, 1u, 3u, 253u, 0u, 0u, 0u, 255u, 0u, 0u, 0u, 255u, 0u, 0u, 0u, 255u}},
  // Go encoder output for long_column
  {"long_column", {100u, 101u, 102u, 103u, 104u, 105u, 106u, 107u, 108u, 109u, 110u, 111u, 112u, 113u, 114u, 115u, 116u, 117u, 118u, 119u, 120u, 121u, 122u, 123u, 124u, 125u, 126u, 127u, 128u, 129u, 130u, 131u, 132u, 133u, 134u, 135u, 136u, 137u, 138u, 139u, 140u, 141u, 142u, 143u, 144u, 145u, 146u, 147u, 148u, 149u, 150u, 151u, 152u, 153u, 154u, 155u, 156u, 157u, 158u, 159u, 160u, 161u, 162u, 163u, 164u, 165u, 166u, 167u, 168u, 169u, 170u, 171u, 172u, 173u, 174u, 175u, 176u, 177u, 178u, 179u, 180u, 181u, 182u, 183u, 184u, 185u, 186u, 187u, 188u, 189u, 190u, 191u, 192u, 193u, 194u, 195u, 196u, 197u, 198u, 199u, 200u, 201u, 202u, 203u, 204u, 205u, 206u, 207u, 208u, 209u, 210u, 211u, 212u, 213u, 214u, 215u, 216u, 217u, 218u, 219u, 220u, 221u, 222u, 223u, 224u, 225u, 226u, 227u, 228u, 229u, 230u, 231u, 232u, 233u, 234u, 235u, 236u, 237u, 238u, 239u, 240u, 241u, 242u, 243u, 244u, 245u, 246u, 247u, 248u, 249u, 250u, 251u, 252u, 253u, 254u, 255u, 256u, 257u, 258u, 259u, 260u, 261u, 262u, 263u, 264u, 265u, 266u, 267u, 268u, 269u, 270u, 271u, 272u, 273u, 274u, 275u, 276u, 277u, 278u, 279u, 280u, 281u, 282u, 283u, 284u, 285u, 286u, 287u, 288u, 289u, 290u, 291u, 292u, 293u, 294u, 295u, 296u, 297u, 298u, 299u, 300u, 301u, 302u, 303u, 304u, 305u, 306u, 307u, 308u, 309u, 310u, 311u, 312u, 313u, 314u, 315u, 316u, 317u, 318u, 319u, 320u, 321u, 322u, 323u, 324u, 325u, 326u, 327u, 328u, 329u, 330u, 331u, 332u, 333u, 334u, 335u, 336u, 337u, 338u, 339u, 340u, 341u, 342u, 343u, 344u, 345u, 346u, 347u, 348u, 349u, 350u, 351u, 352u, 353u, 354u, 355u, 356u, 357u, 358u, 359u, 360u, 361u, 362u, 363u, 364u, 365u, 366u, 367u, 368u, 369u, 370u, 371u, 372u, 373u, 374u, 375u, 376u, 377u, 378u, 379u, 380u, 381u, 382u, 383u, 384u, 385u, 386u, 387u, 388u, 389u, 390u, 391u, 392u, 393u, 394u, 395u, 396u, 397u, 398u, 399u}, {100u, 172u, 2u}, {100u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 255u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 63u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 127u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 63u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 255u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 63u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 127u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 63u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 31u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 255u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 15u, 1u, 3u, 1u, 7u, 1u, 3u, 1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u}},
  // Go encoder output for one
  {"one", {42u}, {42u, 1u}, {42u, 0u, 0u, 0u}},
  // Go encoder output for restart
  {"restart", {3u, 4u, 5u, 3u, 4u, 5u}, {3u, 3u, 3u, 3u}, {3u, 7u, 1u, 6u, 7u, 1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u}},
  // Go encoder output for single_run
  {"single_run", {5u, 6u, 7u, 8u}, {5u, 4u}, {5u, 3u, 1u, 15u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u}},
  // Go encoder output for singletons
  {"singletons", {1u, 5u, 9u}, {1u, 1u, 5u, 1u, 9u, 1u}, {1u, 4u, 12u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u}},
  // Go encoder output for two_runs
  {"two_runs", {5u, 6u, 7u, 8u, 12u, 13u}, {5u, 4u, 12u, 2u}, {5u, 3u, 1u, 15u, 4u, 1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u}},
  };
  return v;
}

voxelcodec_ros::ChannelDescriptor descriptor_for(
  const std::string & encoding, std::size_t count, std::size_t payload_bytes)
{
  voxelcodec_ros::ChannelDescriptor d;
  d.name = "z";
  d.semantic = "z";
  d.data_type = voxelcodec_ros::kDataTypeUint32;
  d.encoding = encoding;
  d.compression = voxelcodec_ros::kCompressionNone;
  d.element_count = static_cast<std::uint32_t>(count);
  d.uncompressed_size = payload_bytes;
  d.compressed_size = payload_bytes;
  return d;
}
}  // namespace

TEST(GoConformance, ColumnRunDecodesGoEncoderBytes)
{
  for (const auto & v : vectors()) {
    const auto d = descriptor_for(voxelcodec_ros::kEncodingColumnRun, v.values.size(), v.column_run.size());
    const auto out = voxelcodec_ros::decode_channel(d, v.column_run);
    ASSERT_TRUE(std::holds_alternative<std::vector<std::uint32_t>>(out.values)) << v.name;
    EXPECT_EQ(std::get<std::vector<std::uint32_t>>(out.values), v.values) << "case " << v.name;
  }
}

TEST(GoConformance, XorByteShuffleDecodesGoEncoderBytes)
{
  for (const auto & v : vectors()) {
    const auto d = descriptor_for(voxelcodec_ros::kEncodingXORByteShuffle, v.values.size(), v.xor_shuffle.size());
    const auto out = voxelcodec_ros::decode_channel(d, v.xor_shuffle);
    ASSERT_TRUE(std::holds_alternative<std::vector<std::uint32_t>>(out.values)) << v.name;
    EXPECT_EQ(std::get<std::vector<std::uint32_t>>(out.values), v.values) << "case " << v.name;
  }
}

TEST(GoConformance, ColumnRunRejectsMalformedPayloads)
{
  // A truncated run, a zero length, and a run overflowing element_count must
  // all raise rather than silently produce a short or oversized channel.
  const auto check = [](const std::vector<std::uint8_t> & payload) {
      const auto d = descriptor_for(voxelcodec_ros::kEncodingColumnRun, 4, payload.size());
      EXPECT_THROW(voxelcodec_ros::decode_channel(d, payload), std::runtime_error);
    };
  check({5u});                  // truncated: start with no length
  check({5u, 0u});              // zero-length run
  check({5u, 99u});             // run overflows element_count
  check({5u, 4u, 9u, 1u});      // trailing bytes after element_count satisfied
}

TEST(GoConformance, ColumnRunIsFarSmallerThanRawForAVerticalColumn)
{
  // The property that motivates the encoding: one floor-to-ceiling column of
  // 300 voxels costs two varints instead of 300 values.
  for (const auto & v : vectors()) {
    if (std::string(v.name) != "long_column") {continue;}
    EXPECT_EQ(v.values.size(), 300U);
    EXPECT_LE(v.column_run.size(), 6U) << "300 consecutive z values should cost a couple of varints";
  }
}
