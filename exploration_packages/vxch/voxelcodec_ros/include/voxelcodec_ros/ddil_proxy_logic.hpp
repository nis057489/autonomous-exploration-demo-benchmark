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

// NO LONGER USED IN PRODUCTION. Band traffic is now VoxelTileBatch (one
// message per band per send tick, carrying every changed tile), so there is no
// single tile id to key a dedup slot on, and ddil_proxy_node deliberately
// leaves band messages un-deduped -- successive batches hold DIFFERENT tile
// sets, so replacing one with another would drop map updates. Kept, with its
// tests, for the legacy per-(tile,band) VoxelChannel format.
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
};

// Priority queue with latest-wins deduplication for band messages, a bounded
// backlog that sheds finest-band content first when the link can't keep up,
// and an absolute starvation deadline so a waiting fine band can't be held
// behind fresh coarse arrivals forever.
//
// A pure strict-priority ordering (pop always returns the lowest band_priority
// queued, full stop) looks right in isolation, but with an UNBOUNDED queue it's
// a bug: if content anywhere in the map keeps nudging some tile's band_0, every
// fresh band_0 arrival jumps ahead of whatever higher-index band has been
// sitting there waiting, and nothing ever lets the waiting one catch up.
// Observed for real: one bag capture recorded 6925 band messages sent over a
// whole run, every single one band_0, zero of band_1..5 ever delivered.
//
// The first fix for that was *continuous* aging: every aging_interval_ms of
// wait knocked 1 off a message's effective band_priority, floored at 0. That
// cured the starvation and introduced a worse bug, because it made the
// priority ordering itself evaporate under exactly the conditions it existed
// for. With haar_levels=2 (bands 0..2) and the 250ms default, any message that
// had waited 750ms was floored to effective priority 0 -- so once the backlog
// was deeper than 750ms of drain time, EVERY queued band tied at 0 and pop()'s
// tie-break (insertion order) turned the whole queue into plain FIFO.
// Measured on a live 5 kbps link with a 4 MB backlog: bands 0/1/2 sent
// 73/91/107 messages respectively -- i.e. the scarce link spent MORE on fine
// detail than on coarse, the exact inverse of progressive transmission, and no
// receiver ever saw a coarse-then-refine progression. (The bias toward fine
// bands is not chance: fine bands carry larger payloads, so MTU-splitting in
// occupancy_grid_vxch_node emits more sub-MTU messages per band per tick for
// them, and FIFO hands out link share per message.)
//
// So the ordering and the backlog have to be fixed together, and in this order:
//
//   1. Bound the backlog (set_byte_budget). A queue capped at a few seconds of
//      the link's own capacity is what actually prevents starvation now --
//      nothing can wait longer than the budget takes to drain. Over budget,
//      shed_to_budget_locked() drops the FINEST pending band first (highest
//      band_priority, oldest among equals), never the manifest and never
//      non-band relay traffic. That is the load-shedding policy that matches
//      what the scheme is for: keep coarse coverage of the whole map rather
//      than full detail of whatever happened to arrive first. Unbounded (the
//      old behavior) is still available with a budget of 0.
//   2. Keep band ordering STRICT (effective_priority returns band_priority
//      unchanged), so coarsest-first genuinely holds, with an absolute
//      starvation deadline instead of a continuous decay: a band that has
//      waited aging_interval_ms * band_priority is promoted, in one step, to a
//      tier ahead of every band but still behind the manifest. Set the
//      deadline above the byte budget's drain time and it never fires in
//      normal operation (the budget bounds waits first); it stays as the
//      backstop for an unbounded/unlimited-bandwidth link, which is the case
//      the two starvation regression tests cover.
//
// pop() is O(n) in queue depth (an explicit scan, since promotion depends on
// wall-clock time and can't be precomputed into a static sorted key at push
// time). Fine at this scale, and the byte budget now bounds n directly.
class BandQueue
{
public:
  // aging_interval_ms: the per-band-index step of the absolute starvation
  // deadline (see class comment) -- a queued band_k is promoted ahead of every
  // other band once it has waited aging_interval_ms * k. It is NOT a
  // continuous decay rate any more; between enqueue and that deadline a band
  // keeps its exact band_priority, which is what makes coarsest-first hold.
  //
  // Pick it comfortably ABOVE how long the byte budget takes to drain, or the
  // deadline fires routinely and reintroduces the FIFO flattening it replaced.
  // ddil_proxy_node's default does exactly that; the 250ms kept here is only
  // the header's own standalone default.
  explicit BandQueue(double aging_interval_ms = 250.0)
  : aging_interval_ms_(aging_interval_ms)
  {}

  bool empty() const {return entries_.empty();}
  std::size_t size() const {return entries_.size();}

  void set_aging_interval_ms(double aging_interval_ms) {aging_interval_ms_ = aging_interval_ms;}

  // Maximum bytes of queued band traffic to hold before shedding. 0 = unbounded
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
        total_bytes_ -= entry_bytes(it->second->msg);
        it->second->msg.serialized = std::move(msg.serialized);
        total_bytes_ += entry_bytes(it->second->msg);
        shed_to_budget_locked();
        return true;
      }
    }

    total_bytes_ += entry_bytes(msg);
    entries_.push_back(Entry{std::move(msg), std::chrono::steady_clock::now()});
    auto it = std::prev(entries_.end());
    if (!it->msg.dedup_key.empty()) {
      dedup_index_[it->msg.dedup_key] = it;
    }
    shed_to_budget_locked();
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
    total_bytes_ -= std::min<std::uint64_t>(total_bytes_, entry_bytes(msg));
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
    // relay (band_priority INT_MAX) sit at fixed priority tiers that promotion
    // must never touch: the manifest has to stay ahead of every band regardless
    // of how long bands have been waiting (the decoder can't parse coefficients
    // without it), and non-band traffic has no coarse/fine notion to promote
    // toward.
    if (entry.msg.band_priority < 0 ||
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

  // Drops the least valuable queued band traffic until the backlog is inside
  // byte_budget_: finest band first (highest band_priority), oldest first
  // among equals. Manifest (band_priority < 0) and non-band relay traffic
  // (INT_MAX) are never shed -- the manifest because a decoder cannot place
  // any tile without it, non-band traffic because this budget is a statement
  // about map-detail value and says nothing about whatever else shares the
  // relay.
  //
  // Dropping the finest band is what preserves the property the whole scheme
  // exists for. The alternative (drop oldest regardless of band, i.e. plain
  // tail-drop) throws away coarse coverage of newly explored ground to keep
  // fine detail of ground the peer already has a picture of -- which is how a
  // peer ends up, as measured, holding 64 of a robot's 468 explored tiles
  // while the link burned its capacity refining those same 64.
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
        if (bp < 0 || bp == std::numeric_limits<int>::max()) {
          continue;  // manifest / non-band traffic is never shed
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
