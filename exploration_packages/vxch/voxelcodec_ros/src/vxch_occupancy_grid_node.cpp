#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/ros_messages.hpp"
#include "voxelcodec_ros/types.hpp"
#include "voxelcodec_msgs/msg/voxel_channel.hpp"
#include "voxelcodec_msgs/msg/voxel_manifest.hpp"

namespace voxelcodec_ros
{
namespace
{

// Reverse the uint32 shift applied by the encoder: 0→-1, 1→0, 101→100, etc.
inline std::int8_t unshift_from_uint32(std::uint32_t v)
{
  const int shifted = static_cast<int>(v) - 1;
  return static_cast<std::int8_t>(std::max(-1, std::min(100, shifted)));
}

using TileKey = std::pair<int, int>;

// A tiled encoder can publish several different tiles' messages back-to-back
// on the SAME fixed band_k topic within one send tick (tile identity travels
// in the payload, not the topic). With a depth-1 queue, only the single latest
// sample published before this subscription's executor drains it would
// survive -- everything else is silently dropped at the DDS layer. Must match
// (or exceed) occupancy_grid_vxch_node's kBandQueueDepth.
constexpr std::size_t kBandQueueDepth = 64;

// Per-tile band state: stores decoded int64 coefficients per received band,
// scoped to that tile's own (possibly smaller-than-nominal, at grid edges)
// cell extent.
struct TileBandState
{
  int width{0};
  int height{0};
  int levels{0};
  int total_bands{0};
  std::vector<std::vector<std::int64_t>> band_coeffs;  // [band_index] → coefficients
  std::vector<bool> received;
};

// Zigzag-varint decode (mirrors zigzag_varint_encode in haar_forward.hpp).
inline std::vector<std::int64_t> zigzag_varint_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  std::vector<std::int64_t> out;
  out.reserve(count);
  std::size_t offset = 0;
  while (out.size() < count) {
    std::uint64_t value = 0;
    int shift = 0;
    while (offset < raw.size()) {
      const std::uint8_t byte = raw[offset++];
      value |= static_cast<std::uint64_t>(byte & 0x7FU) << shift;
      if ((byte & 0x80U) == 0) {break;}
      shift += 7;
      if (shift >= 64) {throw std::runtime_error("varint overflow");}
    }
    // Zigzag decode
    const std::int64_t decoded = (value & 1U)
      ? -static_cast<std::int64_t>((value >> 1U) + 1U)
      : static_cast<std::int64_t>(value >> 1U);
    out.push_back(decoded);
  }
  return out;
}

// Mirrors fixed_width_encode in haar_forward.hpp -- the ablation-mode counterpart to
// zigzag_varint_decode above, selected per band via the descriptor's kHaarVarintKey.
inline std::vector<std::int64_t> fixed_width_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  std::vector<std::int64_t> out(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::int32_t v;
    std::memcpy(&v, &raw[i * 4], 4);
    out[i] = v;
  }
  return out;
}

class VxchOccupancyGridNode : public rclcpp::Node
{
public:
  VxchOccupancyGridNode()
  : Node("vxch_occupancy_grid_node")
  {
    input_base_topic_ = declare_parameter<std::string>("input_base_topic", "/vxch/map");
    output_topic_ = declare_parameter<std::string>("output_topic", "");
    haar_levels_ = declare_parameter<int>("haar_levels", 4);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);

    if (output_topic_.empty()) {
      throw std::runtime_error("output_topic parameter is required");
    }
    if (haar_levels_ < 1 || haar_levels_ > 12) {
      throw std::runtime_error("haar_levels must be between 1 and 12");
    }

    const auto map_qos = rclcpp::QoS(1)
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
    const auto band_qos = rclcpp::QoS(kBandQueueDepth).best_effort();

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, map_qos);

    manifest_sub_ = create_subscription<voxelcodec_msgs::msg::VoxelManifest>(
      input_base_topic_ + "/manifest", map_qos,
      [this](voxelcodec_msgs::msg::VoxelManifest::ConstSharedPtr msg) {
        on_manifest(msg);
      });

    const int total_bands = haar_levels_ + 1;
    band_subs_.resize(static_cast<std::size_t>(total_bands));
    for (int k = 0; k < total_bands; ++k) {
      const std::string topic = input_base_topic_ + "/band_" + std::to_string(k);
      const int band_index = k;
      band_subs_[static_cast<std::size_t>(k)] =
        create_subscription<voxelcodec_msgs::msg::VoxelChannel>(
          topic, band_qos,
          [this, band_index](voxelcodec_msgs::msg::VoxelChannel::ConstSharedPtr msg) {
            on_band(band_index, msg);
          });
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      [this]() {publish_map();});

    RCLCPP_INFO(
      get_logger(),
      "Decoding %s/band_0..band_%d → %s (haar_levels=%d, rate=%.1f Hz)",
      input_base_topic_.c_str(), haar_levels_, output_topic_.c_str(),
      haar_levels_, publish_rate_hz_);
  }

private:
  void on_manifest(const voxelcodec_msgs::msg::VoxelManifest::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const Manifest manifest = manifest_from_msg(*msg);

    // Parse grid metadata
    const auto get_meta = [&](const std::string & key) -> std::string {
        auto it = manifest.metadata.find(key);
        return (it != manifest.metadata.end()) ? it->second : "";
      };

    const std::string w_str = get_meta("grid_width");
    const std::string h_str = get_meta("grid_height");
    if (w_str.empty() || h_str.empty()) {
      RCLCPP_WARN(get_logger(), "Manifest missing grid_width/grid_height metadata");
      return;
    }

    const std::uint32_t new_width = static_cast<std::uint32_t>(std::stoul(w_str));
    const std::uint32_t new_height = static_cast<std::uint32_t>(std::stoul(h_str));
    const std::string tile_size_str = get_meta("tile_size_cells");
    const int new_tile_size_cells = tile_size_str.empty() ? 0 : std::stoi(tile_size_str);

    // Bands are sent progressively (possibly one per second, coarsest first, only when their
    // content actually changed -- see occupancy_grid_vxch_node's send_pending_bands()), so a
    // fresher manifest stamp than the last one does NOT mean every tile/band needs to arrive
    // again under it. A tile's band we already decoded is still valid until the grid geometry
    // or tile partition itself changes -- wiping all tile state on every manifest stamp change
    // would discard bands that would never be resent (their content hadn't changed), permanently
    // stalling reconstruction of whatever tiles were already fully caught up.
    if (new_width != grid_width_ || new_height != grid_height_ ||
      (new_tile_size_cells != 0 && new_tile_size_cells != tile_size_cells_))
    {
      tiles_.clear();
      grid_width_ = new_width;
      grid_height_ = new_height;
    }
    if (new_tile_size_cells != 0) {
      tile_size_cells_ = new_tile_size_cells;
    }

    const std::string res_str = get_meta("resolution");
    grid_resolution_ = res_str.empty() ? 0.05f : std::stof(res_str);
    origin_x_ = std::stod(get_meta("origin_x").empty() ? "0" : get_meta("origin_x"));
    origin_y_ = std::stod(get_meta("origin_y").empty() ? "0" : get_meta("origin_y"));
    frame_id_ = get_meta("frame_id");
    if (frame_id_.empty()) {frame_id_ = "map";}
    manifest_stamp_ = msg->header.stamp;
    have_manifest_ = true;
  }

  void on_band(int band_index, const voxelcodec_msgs::msg::VoxelChannel::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // No manifest/geometry established yet -- nothing to place this tile's decode into.
    if (!have_manifest_) {
      return;
    }

    const ChannelDescriptor desc = descriptor_from_msg(msg->descriptor);

    const auto get_desc_meta = [&](const std::string & key, int fallback) -> int {
        auto it = desc.metadata.find(key);
        return (it != desc.metadata.end()) ? std::stoi(it->second) : fallback;
      };
    // Untagged (non-tiled) publishers are treated as a single tile (0,0) covering
    // the whole grid, for backward compatibility with an untiled encoder.
    const TileKey key{get_desc_meta("tile_row", 0), get_desc_meta("tile_col", 0)};
    const int tile_w = get_desc_meta("tile_width", static_cast<int>(grid_width_));
    const int tile_h = get_desc_meta("tile_height", static_cast<int>(grid_height_));

    auto & tile = tiles_[key];
    if (tile.width != tile_w || tile.height != tile_h || tile.levels != haar_levels_) {
      tile = TileBandState{};
      tile.width = tile_w;
      tile.height = tile_h;
      tile.levels = haar_levels_;
      tile.total_bands = haar_levels_ + 1;
      tile.band_coeffs.assign(static_cast<std::size_t>(tile.total_bands), {});
      tile.received.assign(static_cast<std::size_t>(tile.total_bands), false);
    }

    const std::size_t idx = static_cast<std::size_t>(band_index);
    if (idx >= tile.band_coeffs.size()) {
      return;
    }

    // Decompress, then decode with whichever coefficient packing this band's sender used
    // (kHaarVarintKey, always set by make_haar_bands). Always overwrites -- bands are
    // latest-wins (the encoder only resends a band when its content actually changed),
    // and a band, once received, must stay refreshable for as long as this tile's
    // geometry is valid; it doesn't have a one-shot "already received, never touch
    // again" lifetime tied to a manifest stamp.
    try {
      const auto raw = decompress_payload(desc, msg->payload);
      const bool use_varint = desc.metadata.at(kHaarVarintKey) == "1";
      tile.band_coeffs[idx] = use_varint ?
        zigzag_varint_decode(raw, desc.element_count) :
        fixed_width_decode(raw, desc.element_count);
      tile.received[idx] = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "Failed to decode tile (%d,%d) band %d: %s",
        key.first, key.second, band_index, e.what());
    }
  }

  void publish_map()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!have_manifest_ || grid_width_ == 0 || grid_height_ == 0 || tile_size_cells_ <= 0) {
      return;
    }

    std::vector<std::int8_t> grid_data(
      static_cast<std::size_t>(grid_width_) * static_cast<std::size_t>(grid_height_), -1);
    bool any_tile_rendered = false;

    for (auto & [key, tile] : tiles_) {
      if (tile.width <= 0 || tile.height <= 0) {continue;}

      // Count consecutive received bands from 0 -- same rule as the pre-tiling
      // version, just scoped to this tile's own pyramid.
      int bands_received = 0;
      for (int k = 0; k < tile.total_bands; ++k) {
        if (tile.received[static_cast<std::size_t>(k)]) {
          bands_received = k + 1;
        } else {
          break;
        }
      }
      if (bands_received == 0) {continue;}

      HaarReconstruction recon;
      try {
        recon = reconstruct_haar_from_bands(
          tile.band_coeffs, static_cast<std::size_t>(tile.width),
          static_cast<std::size_t>(tile.height), tile.levels, bands_received);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(), "reconstruct_haar_from_bands failed for tile (%d,%d): %s",
          key.first, key.second, e.what());
        continue;
      }

      const std::size_t tw = static_cast<std::size_t>(tile.width);
      const std::size_t th = static_cast<std::size_t>(tile.height);
      const std::size_t w_prime = recon.width;
      const std::size_t h_prime = recon.height;

      const std::size_t row0 = static_cast<std::size_t>(key.first) *
        static_cast<std::size_t>(tile_size_cells_);
      const std::size_t col0 = static_cast<std::size_t>(key.second) *
        static_cast<std::size_t>(tile_size_cells_);

      // 2D nearest-neighbour upsample if we only have a coarse reconstruction of
      // this tile -- a real (blurry but spatially faithful) downsampled patch,
      // pasted at this tile's place in the full-resolution output grid.
      for (std::size_t r = 0; r < th; ++r) {
        const std::size_t r_src = (w_prime == tw && h_prime == th) ? r : (r * h_prime / th);
        const std::size_t dst_row = row0 + r;
        if (dst_row >= grid_height_) {break;}
        for (std::size_t c = 0; c < tw; ++c) {
          const std::size_t c_src = (w_prime == tw && h_prime == th) ? c : (c * w_prime / tw);
          const std::size_t dst_col = col0 + c;
          if (dst_col >= grid_width_) {continue;}
          grid_data[dst_row * grid_width_ + dst_col] =
            unshift_from_uint32(recon.values[r_src * w_prime + c_src]);
        }
      }
      any_tile_rendered = true;
    }

    if (!any_tile_rendered) {return;}

    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = manifest_stamp_;
    out.header.frame_id = frame_id_;
    out.info.map_load_time = manifest_stamp_;
    out.info.resolution = grid_resolution_;
    out.info.width = grid_width_;
    out.info.height = grid_height_;
    out.info.origin.position.x = origin_x_;
    out.info.origin.position.y = origin_y_;
    out.info.origin.orientation.w = 1.0;
    out.data = std::move(grid_data);

    map_pub_->publish(out);
  }

  std::string input_base_topic_;
  std::string output_topic_;
  int haar_levels_;
  double publish_rate_hz_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<voxelcodec_msgs::msg::VoxelManifest>::SharedPtr manifest_sub_;
  std::vector<rclcpp::Subscription<voxelcodec_msgs::msg::VoxelChannel>::SharedPtr> band_subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::map<TileKey, TileBandState> tiles_;

  // Grid geometry from latest manifest
  bool have_manifest_{false};
  std::uint32_t grid_width_{0};
  std::uint32_t grid_height_{0};
  int tile_size_cells_{0};
  float grid_resolution_{0.05f};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::string frame_id_{"map"};
  builtin_interfaces::msg::Time manifest_stamp_{};
};

}  // namespace
}  // namespace voxelcodec_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<voxelcodec_ros::VxchOccupancyGridNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("vxch_occupancy_grid_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
