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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <list>
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

// Priority queue with latest-wins deduplication for band messages, and wait-time
// aging so a sustained stream of fresh low-index (coarse) arrivals can't starve
// out higher-index (fine detail) messages indefinitely.
//
// A pure strict-priority ordering (pop always returns the lowest band_priority
// queued, full stop) looks right in isolation, but under real bandwidth pressure
// it's a bug: if content anywhere in the map keeps nudging some tile's band_0,
// every fresh band_0 arrival jumps the (insertion_seq-ordered) queue ahead of
// whatever higher-index band has been sitting there waiting -- and there is no
// mechanism that ever lets the waiting one catch up. Observed for real: one bag
// capture recorded 6925 band messages sent over a whole run, every single one
// band_0, zero of band_1..5 ever delivered -- not a fluke, a guaranteed outcome
// of unbounded strict priority whenever coarse content keeps regenerating faster
// than the link drains the queue.
//
// Aging fixes this by improving a message's *effective* priority the longer it
// waits: every aging_interval_ms of wait time knocks 1 off its band_priority,
// floored at 0 (a band can catch up to band_0's priority but never leapfrog the
// manifest, which is exempt from aging -- see effective_priority below). This
// bounds worst-case wait instead of allowing indefinite starvation, at the cost
// of pop() being O(n) in queue depth (an explicit scan, since the effective
// order changes continuously with wall-clock time and can't be precomputed into
// a static sorted key at push time). Fine at this scale: queue depth is bounded
// by distinct (tile, band) slots currently dirty, not by message rate.
class BandQueue
{
public:
  // aging_interval_ms: see class comment. 250ms default means a tile's finest
  // band (haar_levels=5 -> band index 5) needs ~1.25s of waiting to fully catch
  // up to a never-before-waited band_0 -- long enough that a genuinely transient
  // burst of coarse traffic still gets to go first, short enough that starvation
  // is bounded to about a second on top of whatever the token bucket already
  // imposes, not "for the rest of the run."
  explicit BandQueue(double aging_interval_ms = 250.0)
  : aging_interval_ms_(aging_interval_ms)
  {}

  bool empty() const {return entries_.empty();}
  std::size_t size() const {return entries_.size();}

  void set_aging_interval_ms(double aging_interval_ms) {aging_interval_ms_ = aging_interval_ms;}

  // Aggregate currently-queued (not-yet-sent) count/bytes per band index,
  // across all tiles. Manifest (band_priority -1) and non-band relay traffic
  // (band_priority INT_MAX) are excluded -- this is for band-level UI/stats
  // reporting only (e.g. NetworkStatsPanel), never on the push/pop hot path,
  // so an O(n) scan over the queue here is fine.
  std::map<int, std::pair<std::size_t, std::uint64_t>> pending_by_band() const
  {
    std::map<int, std::pair<std::size_t, std::uint64_t>> result;
    for (const auto & entry : entries_) {
      const int bp = entry.msg.band_priority;
      if (bp < 0 || bp == std::numeric_limits<int>::max()) {
        continue;
      }
      auto & agg = result[bp];
      agg.first += 1;
      agg.second += entry.msg.serialized->size();
    }
    return result;
  }

  // Returns true if this push replaced an already-queued entry (dedup fired).
  bool push(QueuedMessage msg)
  {
    if (!msg.dedup_key.empty()) {
      auto it = dedup_index_.find(msg.dedup_key);
      if (it != dedup_index_.end()) {
        // Replace payload of existing entry in-place (same priority slot) --
        // deliberately NOT resetting enqueued_at: the receiver still doesn't have
        // any version of this slot, so its wait-time (and thus its aging credit)
        // has to keep accruing from the original enqueue, or a slot whose content
        // keeps getting refreshed before its turn comes up would dodge aging
        // forever, reintroducing the exact starvation this class exists to avoid.
        it->second->msg.serialized = std::move(msg.serialized);
        return true;
      }
    }

    entries_.push_back(Entry{std::move(msg), std::chrono::steady_clock::now()});
    auto it = std::prev(entries_.end());
    if (!it->msg.dedup_key.empty()) {
      dedup_index_[it->msg.dedup_key] = it;
    }
    return false;
  }

  QueuedMessage pop()
  {
    const auto now = std::chrono::steady_clock::now();
    auto best = entries_.begin();
    double best_priority = effective_priority(*best, now);
    for (auto it = std::next(entries_.begin()); it != entries_.end(); ++it) {
      const double priority = effective_priority(*it, now);
      if (priority < best_priority) {
        best_priority = priority;
        best = it;
      }
    }

    QueuedMessage msg = std::move(best->msg);
    if (!msg.dedup_key.empty()) {
      dedup_index_.erase(msg.dedup_key);
    }
    entries_.erase(best);
    return msg;
  }

private:
  struct Entry
  {
    QueuedMessage msg;
    std::chrono::steady_clock::time_point enqueued_at;
  };

  double effective_priority(const Entry & entry, std::chrono::steady_clock::time_point now) const
  {
    // Manifest (band_priority -1) and any non-band traffic through this same
    // relay (band_priority INT_MAX) sit at fixed priority tiers that aging must
    // never touch: the manifest has to stay ahead of every band regardless of
    // how long bands have been waiting (the decoder can't parse coefficients
    // without it), and non-band traffic has no coarse/fine notion to age toward.
    if (entry.msg.band_priority < 0 ||
      entry.msg.band_priority == std::numeric_limits<int>::max())
    {
      return static_cast<double>(entry.msg.band_priority);
    }
    const double wait_ms =
      std::chrono::duration<double, std::milli>(now - entry.enqueued_at).count();
    const double aged = static_cast<double>(entry.msg.band_priority) - wait_ms / aging_interval_ms_;
    return std::max(aged, 0.0);
  }

  std::list<Entry> entries_;
  std::map<std::string, std::list<Entry>::iterator> dedup_index_;
  double aging_interval_ms_;
};

}  // namespace voxelcodec_ros
