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
#include <voxelcodec_msgs/msg/voxel_tile_batch.hpp>

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
  // VoxelChannel is the pre-batching per-(tile,band) message; VoxelTileBatch
  // carries a whole tick's tiles for one band. Both are band traffic as far as
  // staleness/priority is concerned.
  if (msg_type == "voxelcodec_msgs/msg/VoxelChannel" ||
    msg_type == "voxelcodec_msgs/msg/VoxelTileBatch")
  {
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

// Legacy per-(tile,band) VoxelChannel identity. VoxelTileBatch uses typed tile
// coordinates in BandQueue::push_tile_batch instead.
//
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
  // Set only from the subscription's actual wire type, never its topic name.
  bool is_tile_batch{false};
  // Internal one-tile representation while queued. serialized holds its CDR
  // size for accounting; pop() repacks compatible tiles to amortize envelopes.
  std::shared_ptr<voxelcodec_msgs::msg::VoxelTileBatch> tile_batch;
  std::size_t batch_byte_limit{0};
};

// VXCH batches are retained as latest-wins (publisher, stream, band, tile)
// slots, then repacked for transmission. Their ordering is strictly by band:
// all pending coarse coverage precedes refinement, even after a long outage.
// Retaining every level also lets a static map finish refining after capacity
// returns; the encoder's change detection will not resend a dropped update.
// Memory scales with pending map content, not the number of encoder ticks.
//
// Other message types keep the legacy byte-budget shedding and aging policy.
// The budget is deliberately soft for retained tile slots: a few seconds of
// a slow link's capacity cannot necessarily hold even one coarse map pass.
class BandQueue
{
public:
  // Legacy non-batched bands only: promote after interval * band_index.
  // Typed tile batches always keep strict band priority.
  explicit BandQueue(double aging_interval_ms = 250.0)
  : aging_interval_ms_(aging_interval_ms)
  {}

  bool empty() const {return entries_.empty();}
  std::size_t size() const {return entries_.size();}

  void set_aging_interval_ms(double aging_interval_ms) {aging_interval_ms_ = aging_interval_ms;}

  // Soft byte budget; retained tile slots are exempt. 0 = unbounded
  // (the pre-bounding behavior). Shrinking the budget sheds immediately rather
  // than waiting for the next push, so lowering the link's bandwidth from the
  // RViz panel takes effect on the existing backlog too.
  void set_byte_budget(std::uint64_t bytes)
  {
    byte_budget_ = bytes;
    shed_to_budget_locked();
  }

  std::uint64_t queued_bytes() const {return total_bytes_;}
  std::uint64_t byte_budget() const {return byte_budget_;}

  // Cumulative messages/bytes dropped by the byte budget since construction.
  // Reported by ddil_proxy_node so a run's logs/stats say plainly how much map
  // detail the link couldn't afford, instead of it silently piling up in an
  // ever-growing backlog nobody will ever receive.
  std::uint64_t shed_count() const {return shed_count_;}
  std::uint64_t shed_bytes() const {return shed_bytes_;}

  // Aggregate currently-queued (not-yet-sent) count/bytes per band index,
  // across all tiles. Manifest (band_priority -1) and non-band relay traffic
  // (band_priority INT_MAX) are excluded -- this is for band-level UI/stats
  // reporting only (e.g. NetworkStatsPanel), never on the push/pop hot path,
  // so an O(n) scan over the queue here is fine. For typed batches these are
  // tile-slot counts and conservative bytes (one envelope per tile); pop()
  // repacks them and the actual token bucket charges the resulting wire size.
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
    if (msg.is_tile_batch) {
      return push_tile_batch(std::move(msg));
    }
    return push_one(std::move(msg));
  }

private:
  bool push_one(QueuedMessage msg, bool apply_budget = true)
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
        total_bytes_ -= entry_bytes(it->second->msg);
        it->second->msg = std::move(msg);
        total_bytes_ += entry_bytes(it->second->msg);
        if (apply_budget) {shed_to_budget_locked();}
        return true;
      }
    }

    total_bytes_ += entry_bytes(msg);
    entries_.push_back(Entry{std::move(msg), std::chrono::steady_clock::now()});
    auto it = std::prev(entries_.end());
    if (!it->msg.dedup_key.empty()) {
      dedup_index_[it->msg.dedup_key] = it;
    }
    if (apply_budget) {shed_to_budget_locked();}
    return false;
  }

public:
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
    total_bytes_ -= std::min<std::uint64_t>(total_bytes_, entry_bytes(msg));
    if (!msg.dedup_key.empty()) {
      dedup_index_.erase(msg.dedup_key);
    }
    entries_.erase(best);
    if (msg.tile_batch) {
      auto batch = *msg.tile_batch;
      auto spec = batch;
      spec.tiles.clear();
      for (auto it = entries_.begin(); it != entries_.end();) {
        const auto & pending = it->msg;
        if (!pending.tile_batch || pending.publisher != msg.publisher) {
          ++it;
          continue;
        }
        auto other_spec = *pending.tile_batch;
        other_spec.tiles.clear();
        // Keep timestamps and codec metadata exact, including across streams.
        if (other_spec != spec) {
          ++it;
          continue;
        }
        batch.tiles.push_back(pending.tile_batch->tiles.front());
        auto packed = serialize_batch(batch);
        if (packed->size() > msg.batch_byte_limit) {
          batch.tiles.pop_back();
          break;
        }
        msg.serialized = std::move(packed);
        total_bytes_ -= entry_bytes(pending);
        dedup_index_.erase(pending.dedup_key);
        it = entries_.erase(it);
      }
      msg.tile_batch = std::make_shared<voxelcodec_msgs::msg::VoxelTileBatch>(std::move(batch));
    }
    return msg;
  }

private:
  static std::shared_ptr<rclcpp::SerializedMessage> serialize_batch(
    const voxelcodec_msgs::msg::VoxelTileBatch & batch)
  {
    rclcpp::Serialization<voxelcodec_msgs::msg::VoxelTileBatch> ser;
    auto serialized = std::make_shared<rclcpp::SerializedMessage>();
    ser.serialize_message(&batch, serialized.get());
    return serialized;
  }

  bool push_tile_batch(QueuedMessage msg)
  {
    voxelcodec_msgs::msg::VoxelTileBatch batch;
    rclcpp::Serialization<voxelcodec_msgs::msg::VoxelTileBatch> ser;
    ser.deserialize_message(msg.serialized.get(), &batch);
    const auto byte_limit = msg.serialized->size();
    auto tiles = std::move(batch.tiles);
    batch.tiles.clear();
    bool replaced = false;
    for (auto & tile : tiles) {
      auto slot = msg;
      slot.band_priority = batch.band_index;
      slot.batch_byte_limit = byte_limit;
      // Length-prefix the stream id so punctuation cannot alias another key.
      slot.dedup_key = std::to_string(reinterpret_cast<std::uintptr_t>(msg.publisher.get())) +
        ":" + std::to_string(batch.stream_id.size()) + ":" + batch.stream_id + ":" +
        std::to_string(batch.band_index) + ":" + std::to_string(tile.tile_row) + ":" +
        std::to_string(tile.tile_col);
      slot.tile_batch = std::make_shared<voxelcodec_msgs::msg::VoxelTileBatch>(batch);
      slot.tile_batch->tiles.push_back(std::move(tile));
      slot.serialized = serialize_batch(*slot.tile_batch);
      // Replacement preserves FIFO position: changing tiles cannot keep
      // jumping ahead of other tiles still waiting for their first coverage.
      replaced = push_one(std::move(slot), false) || replaced;
    }
    shed_to_budget_locked();
    return replaced;
  }

  struct Entry
  {
    QueuedMessage msg;
    std::chrono::steady_clock::time_point enqueued_at;
  };

  double effective_priority(const Entry & entry, std::chrono::steady_clock::time_point now) const
  {
    // Manifest (band_priority -1) and any non-band traffic through this same
    // relay (band_priority INT_MAX) sit at fixed priority tiers that promotion
    // must never touch: the manifest has to stay ahead of every band regardless
    // of how long bands have been waiting (the decoder can't parse coefficients
    // without it), and non-band traffic has no coarse/fine notion to promote
    // toward.
    if (entry.msg.tile_batch || entry.msg.band_priority < 0 ||
      entry.msg.band_priority == std::numeric_limits<int>::max())
    {
      return static_cast<double>(entry.msg.band_priority);
    }
    // Absolute starvation deadline, not a continuous decay -- see the class
    // comment for why the decay had to go. Below the deadline a band keeps its
    // exact band_priority, so coarsest-first is strict and a 4 MB backlog can
    // no longer flatten every band to one tier. At the deadline the band jumps
    // in ONE step to kPromotedPriority: ahead of band_0 (0), still behind the
    // manifest (-1), so a promoted band can never delay the geometry its own
    // decode depends on.
    const double wait_ms =
      std::chrono::duration<double, std::milli>(now - entry.enqueued_at).count();
    const double deadline_ms =
      aging_interval_ms_ * static_cast<double>(entry.msg.band_priority);
    if (entry.msg.band_priority > 0 && wait_ms >= deadline_ms) {
      return kPromotedPriority;
    }
    return static_cast<double>(entry.msg.band_priority);
  }

  // Legacy traffic only: finest-first shedding. Retained tile slots cannot
  // be dropped because the encoder does not know whether a peer received
  // them. A newer update replaces a slot without losing unrelated coverage.
  void shed_to_budget_locked()
  {
    if (byte_budget_ == 0) {
      return;
    }
    while (total_bytes_ > byte_budget_) {
      auto victim = entries_.end();
      int worst_priority = -1;
      for (auto it = entries_.begin(); it != entries_.end(); ++it) {
        const int bp = it->msg.band_priority;
        if (it->msg.tile_batch || bp < 0 || bp == std::numeric_limits<int>::max()) {
          continue;  // retained map slots / manifest / non-band traffic is never shed
        }
        // Strictly-greater keeps the FIRST (oldest, since entries_ is in
        // insertion order) entry at the worst band index.
        if (bp > worst_priority) {
          worst_priority = bp;
          victim = it;
        }
      }
      if (victim == entries_.end()) {
        return;  // nothing sheddable left -- budget can't be met, stop trying
      }
      const std::uint64_t bytes = entry_bytes(victim->msg);
      total_bytes_ -= std::min(total_bytes_, bytes);
      shed_count_ += 1;
      shed_bytes_ += bytes;
      if (!victim->msg.dedup_key.empty()) {
        dedup_index_.erase(victim->msg.dedup_key);
      }
      entries_.erase(victim);
    }
  }

  static std::uint64_t entry_bytes(const QueuedMessage & msg)
  {
    // Tests construct QueuedMessages with no payload attached at all; a null
    // serialized just contributes nothing to the budget.
    return msg.serialized ? static_cast<std::uint64_t>(msg.serialized->size()) : 0U;
  }

  // Tier a starvation-promoted band lands on: ahead of band_0, behind the
  // manifest's -1.
  static constexpr double kPromotedPriority = -0.5;

  std::list<Entry> entries_;
  std::map<std::string, std::list<Entry>::iterator> dedup_index_;
  double aging_interval_ms_;
  std::uint64_t total_bytes_{0};
  std::uint64_t byte_budget_{0};
  std::uint64_t shed_count_{0};
  std::uint64_t shed_bytes_{0};
};

}  // namespace voxelcodec_ros
