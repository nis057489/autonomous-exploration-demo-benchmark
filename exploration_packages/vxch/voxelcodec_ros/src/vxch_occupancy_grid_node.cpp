#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
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

// Per-update band state: stores decoded int64 coefficients per received band.
struct BandState
{
  std::uint32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::size_t original_len{0};
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
    const auto band_qos = rclcpp::QoS(1).best_effort();

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

    // Only reset state if this is a new map version (different stamp)
    const std::uint32_t sec = msg->header.stamp.sec;
    const std::uint32_t nsec = msg->header.stamp.nanosec;
    if (sec == pending_.stamp_sec && nsec == pending_.stamp_nanosec &&
      pending_.original_len > 0)
    {
      return;  // same update, already have it
    }

    pending_ = BandState{};
    pending_.stamp_sec = sec;
    pending_.stamp_nanosec = nsec;
    pending_.original_len = static_cast<std::size_t>(std::stoul(w_str)) *
      static_cast<std::size_t>(std::stoul(h_str));
    pending_.levels = haar_levels_;
    pending_.total_bands = haar_levels_ + 1;
    pending_.band_coeffs.assign(static_cast<std::size_t>(pending_.total_bands), {});
    pending_.received.assign(static_cast<std::size_t>(pending_.total_bands), false);

    // Cache grid info for OccupancyGrid reconstruction
    grid_width_ = static_cast<std::uint32_t>(std::stoul(w_str));
    grid_height_ = static_cast<std::uint32_t>(std::stoul(h_str));
    const std::string res_str = get_meta("resolution");
    grid_resolution_ = res_str.empty() ? 0.05f : std::stof(res_str);
    origin_x_ = std::stod(get_meta("origin_x").empty() ? "0" : get_meta("origin_x"));
    origin_y_ = std::stod(get_meta("origin_y").empty() ? "0" : get_meta("origin_y"));
    frame_id_ = get_meta("frame_id");
    if (frame_id_.empty()) {frame_id_ = "map";}
    manifest_stamp_ = msg->header.stamp;
  }

  void on_band(int band_index, const voxelcodec_msgs::msg::VoxelChannel::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Ignore bands that don't match the current manifest stamp
    if (static_cast<std::uint32_t>(msg->header.stamp.sec) != pending_.stamp_sec ||
      msg->header.stamp.nanosec != pending_.stamp_nanosec ||
      pending_.original_len == 0)
    {
      return;
    }

    const std::size_t idx = static_cast<std::size_t>(band_index);
    if (idx >= pending_.band_coeffs.size() || pending_.received[idx]) {
      return;
    }

    const ChannelDescriptor desc = descriptor_from_msg(msg->descriptor);

    // Decompress and zigzag-varint decode
    try {
      const auto raw = decompress_payload(desc, msg->payload);
      pending_.band_coeffs[idx] = zigzag_varint_decode(raw, desc.element_count);
      pending_.received[idx] = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to decode band %d: %s", band_index, e.what());
    }
  }

  void publish_map()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pending_.original_len == 0) {
      return;
    }

    // Count consecutive received bands from 0
    int bands_received = 0;
    for (int k = 0; k < pending_.total_bands; ++k) {
      if (pending_.received[static_cast<std::size_t>(k)]) {
        bands_received = k + 1;
      } else {
        break;
      }
    }
    if (bands_received == 0) {
      return;
    }

    // Reconstruct the (possibly coarser) 2D grid directly from the per-band
    // coefficient vectors -- no flat-buffer assembly needed, band_coeffs is
    // already in the right shape.
    const int L = pending_.levels;
    HaarReconstruction recon;
    try {
      recon = reconstruct_haar_from_bands(
        pending_.band_coeffs, grid_width_, grid_height_, L, bands_received);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "reconstruct_haar_from_bands failed: %s", e.what());
      return;
    }

    const std::size_t W = grid_width_;
    const std::size_t H = grid_height_;
    const std::size_t w_prime = recon.width;
    const std::size_t h_prime = recon.height;

    // 2D nearest-neighbour upsample if we only have a coarse reconstruction --
    // a real (blurry but spatially faithful) downsampled map, not a 1D-flat
    // streak, since reconstruct_haar_from_bands did a proper 2D pyramid inverse.
    std::vector<std::int8_t> grid_data(W * H, -1);
    if (w_prime == W && h_prime == H) {
      for (std::size_t i = 0; i < recon.values.size(); ++i) {
        grid_data[i] = unshift_from_uint32(recon.values[i]);
      }
    } else {
      for (std::size_t r = 0; r < H; ++r) {
        const std::size_t r_src = r * h_prime / H;
        for (std::size_t c = 0; c < W; ++c) {
          const std::size_t c_src = c * w_prime / W;
          grid_data[r * W + c] = unshift_from_uint32(recon.values[r_src * w_prime + c_src]);
        }
      }
    }

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
  BandState pending_;

  // Grid geometry from latest manifest
  std::uint32_t grid_width_{0};
  std::uint32_t grid_height_{0};
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
