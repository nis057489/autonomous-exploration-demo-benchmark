#pragma once

// Pure decision logic behind ddil_proxy_node.cpp's Node -- topic/relay-entry
// parsing, the token-bucket rate limiter, and the priority/dedup band queue
// -- pulled into a header so it's testable without an rclcpp::Node fixture,
// same reasoning as ddil_stale_epoch.hpp. Node-specific glue (parameter
// declaration, generic pub/sub wiring, the worker thread) stays in the .cpp.
//
// Still depends on rclcpp's message-serialization primitives (SerializedMessage,
// Serialization<T>, GenericPublisher) and the voxelcodec_msgs message types --
// those don't require a live Node or rclcpp::init to use, just like
// ros_messages.cpp's struct<->msg conversions.

#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <voxelcodec_msgs/msg/voxel_channel.hpp>
#include <voxelcodec_msgs/msg/voxel_manifest.hpp>

namespace voxelcodec_ros
{

// Whether a relayed channel's wire type is one we know how to deserialize
// just enough to read header.stamp from, for stale-epoch detection. Gated on
// the actual wire type string (authoritative -- it's what
// create_generic_subscription/publisher were built with), never on topic
// name: a misconfigured deployment could name an unrelated type ".../band_3",
// and attempting to deserialize arbitrary bytes as a specific ROS type would
// be a real crash/UB risk. Topic-name parsing (band_index_from_topic /
// is_manifest_topic below) is a separate concern -- it only ever drives
// queue priority/dedup, unchanged by this.
enum class EpochRole
{
  kNone,      // not a VXCH manifest/band type -- relayed exactly as before
  kManifest,
  kBand,
};

inline EpochRole epoch_role_from_msg_type(const std::string & msg_type)
{
  if (msg_type == "voxelcodec_msgs/msg/VoxelManifest") {
    return EpochRole::kManifest;
  }
  if (msg_type == "voxelcodec_msgs/msg/VoxelChannel") {
    return EpochRole::kBand;
  }
  return EpochRole::kNone;
}

struct RelayConfig
{
  std::string input_topic;
  std::string output_topic;
  std::string msg_type;
  bool bypass{false};    // bypass throttle (manifest topics etc.)
  bool reliable{false};  // use RELIABLE + TRANSIENT_LOCAL (for Nav2 consumers)
};

inline RelayConfig parse_relay_entry(const std::string & entry)
{
  // Format: "input_topic output_topic msg_type [reliable]"
  // Add the literal word "reliable" as a 4th token to request RELIABLE+TRANSIENT_LOCAL QoS.
  // Without it, BEST_EFFORT is used (appropriate for VXCH bands).
  RelayConfig cfg;
  std::istringstream ss(entry);
  std::string flag;
  ss >> cfg.input_topic >> cfg.output_topic >> cfg.msg_type >> flag;
  if (cfg.input_topic.empty() || cfg.output_topic.empty() || cfg.msg_type.empty()) {
    throw std::runtime_error(
            "relay entry '" + entry +
            "' must have format 'input_topic output_topic msg_type [reliable]'");
  }
  cfg.reliable = (flag == "reliable");
  return cfg;
}

// Extract band index from a topic name ending in "/band_N", or -1 if not a band topic.
inline int band_index_from_topic(const std::string & topic)
{
  const std::string prefix = "/band_";
  const auto pos = topic.rfind(prefix);
  if (pos == std::string::npos) {
    return -1;
  }
  const std::string suffix = topic.substr(pos + prefix.size());
  if (suffix.empty()) {
    return -1;
  }
  for (char c : suffix) {
    if (c < '0' || c > '9') {
      return -1;
    }
  }
  return std::stoi(suffix);
}

// True if the topic ends in "/manifest" — these get dedup treatment just like bands.
inline bool is_manifest_topic(const std::string & topic)
{
  const std::string suffix = "/manifest";
  return topic.size() >= suffix.size() &&
         topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// A tiled occupancy_grid_vxch_node encoder multiplexes every tile's band_k
// onto the SAME fixed /band_k topic (tile identity travels in the message's
// descriptor metadata, not the topic name -- see occupancy_grid_vxch_node's
// tile_row/tile_col tagging). band_index_from_topic alone can no longer
// identify a dedup slot: two different tiles' band_2 updates queued in the
// same tick would collide on one dedup_key and one would silently clobber
// the other. Peeking at tile_row/tile_col here (a full deserialize, same
// cost extract_stamp already pays at pop-time) keeps each tile's backlog
// independent. Untagged (untiled) publishers fall back to (0,0), which is
// exactly the single-tile-covering-the-whole-grid case, so this is a no-op
// for anyone not running a tiled encoder.
inline std::pair<int, int> tile_id_from_channel_msg(const rclcpp::SerializedMessage & serialized)
{
  static rclcpp::Serialization<voxelcodec_msgs::msg::VoxelChannel> ser;
  voxelcodec_msgs::msg::VoxelChannel msg;
  ser.deserialize_message(&serialized, &msg);
  int tile_row = 0;
  int tile_col = 0;
  for (const auto & entry : msg.descriptor.metadata) {
    if (entry.key == "tile_row") {
      tile_row = std::stoi(entry.value);
    } else if (entry.key == "tile_col") {
      tile_col = std::stoi(entry.value);
    }
  }
  return {tile_row, tile_col};
}

// Token bucket — thread-safe, shared across all relay channels (simulates one shared link).
class TokenBucket
{
public:
  explicit TokenBucket(double bandwidth_kbps)
  : bandwidth_bytes_per_sec_(bandwidth_kbps * 125.0),
    tokens_(bandwidth_kbps > 0.0 ? bandwidth_kbps * 125.0 : 0.0),
    last_refill_(std::chrono::steady_clock::now())
  {}

  // Block until `bytes` tokens are available, then consume them.
  // Returns immediately if bandwidth is unlimited (bandwidth_bytes_per_sec_ == 0).
  void consume(std::size_t bytes)
  {
    if (bandwidth_bytes_per_sec_ <= 0.0) {
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    refill_locked();
    tokens_ -= static_cast<double>(bytes);
    while (tokens_ < 0.0) {
      // Sleep for the time needed to earn back the deficit
      const double wait_sec = -tokens_ / bandwidth_bytes_per_sec_;
      lock.unlock();
      std::this_thread::sleep_for(
        std::chrono::duration<double>(std::min(wait_sec, 0.1)));
      lock.lock();
      refill_locked();
    }
  }

  // Test-only accessor: current token balance without consuming any.
  double tokens_for_testing()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    refill_locked();
    return tokens_;
  }

private:
  void refill_locked()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - last_refill_).count();
    tokens_ = std::min(
      bandwidth_bytes_per_sec_,  // cap at 1-second bucket
      tokens_ + elapsed * bandwidth_bytes_per_sec_);
    last_refill_ = now;
  }

  const double bandwidth_bytes_per_sec_;
  double tokens_;
  std::chrono::steady_clock::time_point last_refill_;
  std::mutex mutex_;
};

struct QueuedMessage
{
  std::shared_ptr<rclcpp::SerializedMessage> serialized;
  std::shared_ptr<rclcpp::GenericPublisher> publisher;
  // band_priority: 0 = coarsest (sent first), INT_MAX for non-band topics.
  // Lower value = higher priority.
  int band_priority{std::numeric_limits<int>::max()};
  // Dedup key: non-empty for band topics, identifies the (channel, band_index) slot.
  // Latest-wins: a new message with the same key replaces the queued one.
  std::string dedup_key;
  // Wire-type classification for stale-epoch detection (see EpochRole above).
  EpochRole epoch_role{EpochRole::kNone};
};

// Priority queue with latest-wins deduplication for band messages.
// Internally a sorted std::map<(priority, insertion_seq), QueuedMessage>.
// Dedup map tracks which insertion_seq holds the live entry for each dedup_key.
class BandQueue
{
public:
  bool empty() const {return ordered_.empty();}
  std::size_t size() const {return ordered_.size();}

  // Returns true if this push replaced an already-queued entry (dedup fired).
  bool push(QueuedMessage msg)
  {
    if (!msg.dedup_key.empty()) {
      auto it = dedup_index_.find(msg.dedup_key);
      if (it != dedup_index_.end()) {
        // Replace payload of existing entry in-place (same priority slot).
        it->second->second.serialized = msg.serialized;
        return true;
      }
    }

    const uint64_t seq = next_seq_++;
    auto [ins, _] = ordered_.emplace(
      std::make_pair(msg.band_priority, seq),
      std::move(msg));

    if (!ins->second.dedup_key.empty()) {
      dedup_index_[ins->second.dedup_key] = ins;
    }
    return false;
  }

  QueuedMessage pop()
  {
    auto it = ordered_.begin();
    QueuedMessage msg = std::move(it->second);
    if (!msg.dedup_key.empty()) {
      dedup_index_.erase(msg.dedup_key);
    }
    ordered_.erase(it);
    return msg;
  }

private:
  using Key = std::pair<int, uint64_t>;
  std::map<Key, QueuedMessage> ordered_;
  std::map<std::string, std::map<Key, QueuedMessage>::iterator> dedup_index_;
  uint64_t next_seq_{0};
};

}  // namespace voxelcodec_ros
