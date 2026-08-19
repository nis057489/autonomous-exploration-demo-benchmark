#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <voxelcodec_msgs/msg/voxel_channel.hpp>
#include <voxelcodec_msgs/msg/voxel_manifest.hpp>

#include "voxelcodec_ros/ddil_proxy_logic.hpp"
#include "voxelcodec_ros/ddil_stale_epoch.hpp"

namespace
{

using voxelcodec_ros::BandQueue;
using voxelcodec_ros::EpochRole;
using voxelcodec_ros::QueuedMessage;
using voxelcodec_ros::TokenBucket;
using voxelcodec_ros::band_index_from_topic;
using voxelcodec_ros::epoch_role_from_msg_type;
using voxelcodec_ros::is_manifest_topic;
using voxelcodec_ros::parse_relay_entry;
using voxelcodec_ros::RelayConfig;
using voxelcodec_ros::tile_id_from_channel_msg;

class DdilProxy : public rclcpp::Node
{
public:
  DdilProxy()
  : Node("ddil_proxy"),
    rng_(0)  // seeded below after parameter declaration
  {
    bandwidth_kbps_ = declare_parameter<double>("bandwidth_kbps", 0.0);
    loss_pct_ = declare_parameter<double>("loss_pct", 0.0);
    delay_ms_ = declare_parameter<double>("delay_ms", 0.0);
    // See BandQueue's class comment in ddil_proxy_logic.hpp: without aging, a
    // sustained stream of fresh band_0 (coarse) traffic anywhere in the map can
    // starve every tile's higher-index (fine detail) bands indefinitely once the
    // link is actually bandwidth-constrained -- confirmed in a real run where
    // 6925/6925 band messages sent were band_0. This is the knob for how many ms
    // of wait time a queued band needs before its effective priority improves by
    // one step.
    priority_aging_ms_ = declare_parameter<double>("priority_aging_ms", 250.0);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.set_aging_interval_ms(priority_aging_ms_);
    }
    const int64_t rng_seed = declare_parameter<int64_t>("rng_seed", -1);
    rng_.seed(rng_seed >= 0 ? static_cast<uint32_t>(rng_seed) : std::random_device{}());
    const auto relay_entries =
      declare_parameter<std::vector<std::string>>("relay_topics", std::vector<std::string>{});
    const auto bypass_entries =
      declare_parameter<std::vector<std::string>>("bypass_topics", std::vector<std::string>{});

    token_bucket_ = std::make_shared<TokenBucket>(bandwidth_kbps_);

    // Register parameter change handler
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_param_change(params);
      });

    // Set up relay channels (throttled)
    for (const auto & entry : relay_entries) {
      try {
        auto cfg = parse_relay_entry(entry);
        cfg.bypass = false;
        setup_relay(cfg);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Bad relay entry '%s': %s", entry.c_str(), e.what());
      }
    }

    // Set up bypass channels (forwarded immediately, no throttle)
    for (const auto & entry : bypass_entries) {
      try {
        auto cfg = parse_relay_entry(entry);
        cfg.bypass = true;
        setup_relay(cfg);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Bad bypass entry '%s': %s", entry.c_str(), e.what());
      }
    }

    // Periodic stats log
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(10),
      [this]() {log_stats();});

    // Worker thread processes the throttled queue
    worker_ = std::thread([this]() {worker_loop();});

    RCLCPP_INFO(
      get_logger(),
      "DdilProxy: %.0f kbps, %.1f%% loss, %.0f ms delay | %zu relay + %zu bypass topics",
      bandwidth_kbps_, loss_pct_, delay_ms_,
      relay_entries.size(), bypass_entries.size());
  }

  ~DdilProxy() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      shutdown_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void setup_relay(const RelayConfig & cfg)
  {
    // bypass or reliable → RELIABLE + TRANSIENT_LOCAL (for manifest topics and Nav2 consumers)
    // plain relay         → BEST_EFFORT volatile (for VXCH bands between encoder and decoder)
    //
    // A tiled occupancy_grid_vxch_node can publish several different tiles'
    // messages back-to-back on the same fixed band_k topic within one send
    // tick (tile identity travels in the payload, not the topic). This
    // subscription's own DDS history depth needs to be able to hold all of
    // them, or the ones after the first are silently dropped before
    // on_message() ever runs -- BandQueue's own dedup/priority downstream
    // never gets a chance to see them. kBandQueueDepth must be at least as
    // large as occupancy_grid_vxch_node's own constant of the same name.
    constexpr int kBandQueueDepth = 64;
    rclcpp::QoS qos(cfg.bypass || cfg.reliable ? 1 : kBandQueueDepth);
    if (cfg.bypass || cfg.reliable) {
      qos.reliable().durability(rclcpp::DurabilityPolicy::TransientLocal);
    } else {
      qos.best_effort();
    }

    auto pub = create_generic_publisher(cfg.output_topic, cfg.msg_type, qos);
    publishers_.push_back(pub);

    const bool is_bypass = cfg.bypass;
    const std::string input_topic = cfg.input_topic;
    const EpochRole epoch_role = epoch_role_from_msg_type(cfg.msg_type);
    auto sub = create_generic_subscription(
      cfg.input_topic, cfg.msg_type, qos,
      [this, pub, is_bypass, input_topic, epoch_role](
        std::shared_ptr<rclcpp::SerializedMessage> serialized) {
        on_message(serialized, pub, is_bypass, input_topic, epoch_role);
      });
    subscriptions_.push_back(sub);

    RCLCPP_INFO(
      get_logger(), "%s %s → %s (%s)",
      is_bypass ? "bypass" : "relay",
      cfg.input_topic.c_str(), cfg.output_topic.c_str(), cfg.msg_type.c_str());
  }

  void on_message(
    std::shared_ptr<rclcpp::SerializedMessage> serialized,
    std::shared_ptr<rclcpp::GenericPublisher> pub,
    bool bypass,
    const std::string & input_topic,
    EpochRole epoch_role)
  {
    if (bypass) {
      pub->publish(*serialized);
      RCLCPP_DEBUG(get_logger(), "bypass  %s  %zu B", input_topic.c_str(), serialized->size());
      return;
    }

    msgs_received_.fetch_add(1, std::memory_order_relaxed);

    // Random drop
    {
      std::lock_guard<std::mutex> lock(rng_mutex_);
      const double roll = std::uniform_real_distribution<double>(0.0, 100.0)(rng_);
      if (roll < loss_pct_) {
        msgs_dropped_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_DEBUG(
          get_logger(), "DROP  %s  %zu B  (roll=%.1f < %.1f%%)",
          input_topic.c_str(), serialized->size(), roll, loss_pct_);
        return;
      }
    }

    // Build queued message with priority and dedup key for band topics.
    QueuedMessage item;
    item.serialized = serialized;
    item.publisher = pub;
    item.epoch_role = epoch_role;

    const int band_idx = band_index_from_topic(input_topic);
    if (band_idx >= 0) {
      item.band_priority = band_idx;
      const auto [tile_row, tile_col] = tile_id_from_channel_msg(*serialized);
      item.dedup_key =
        std::to_string(reinterpret_cast<std::uintptr_t>(pub.get())) +
        ":band_" + std::to_string(band_idx) +
        ":tile_" + std::to_string(tile_row) + "_" + std::to_string(tile_col);
    } else if (is_manifest_topic(input_topic)) {
      // Manifest must arrive before any band (decoder needs it to parse coefficients).
      // Priority -1 puts it ahead of band_0 (priority 0). Deduplicated so only the
      // latest manifest is ever queued; stale ones are replaced in-place.
      item.band_priority = -1;
      item.dedup_key =
        std::to_string(reinterpret_cast<std::uintptr_t>(pub.get())) + ":manifest";
    }

    bool deduped;
    std::size_t queue_depth;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      deduped = queue_.push(std::move(item));
      queue_depth = queue_.size();
      if (!deduped) {
        queue_cv_.notify_one();
      }
    }

    if (deduped) {
      msgs_deduped_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_DEBUG(
        get_logger(), "DEDUP %s  %zu B  (replaced stale queued entry, queue=%zu)",
        input_topic.c_str(), serialized->size(), queue_depth);
    } else {
      RCLCPP_DEBUG(
        get_logger(), "QUEUE %s  %zu B  prio=%d  queue=%zu",
        input_topic.c_str(), serialized->size(), band_idx >= 0 ? band_idx : -1, queue_depth);
    }
  }

  // Deserialize just enough to read header.stamp, for stale-epoch tracking.
  // Only called for items already gated by EpochRole (kManifest/kBand), i.e.
  // whose msg_type string is confirmed to be VoxelManifest/VoxelChannel --
  // never attempted for arbitrary/generic relayed types.
  static voxelcodec_ros::Stamp extract_stamp(const QueuedMessage & item)
  {
    if (item.epoch_role == EpochRole::kManifest) {
      static rclcpp::Serialization<voxelcodec_msgs::msg::VoxelManifest> ser;
      voxelcodec_msgs::msg::VoxelManifest msg;
      ser.deserialize_message(item.serialized.get(), &msg);
      return {msg.header.stamp.sec, static_cast<std::uint32_t>(msg.header.stamp.nanosec)};
    }
    static rclcpp::Serialization<voxelcodec_msgs::msg::VoxelChannel> ser;
    voxelcodec_msgs::msg::VoxelChannel msg;
    ser.deserialize_message(item.serialized.get(), &msg);
    return {msg.header.stamp.sec, static_cast<std::uint32_t>(msg.header.stamp.nanosec)};
  }

  void worker_loop()
  {
    while (true) {
      QueuedMessage item;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {return !queue_.empty() || shutdown_;});
        if (shutdown_ && queue_.empty()) {break;}
        item = queue_.pop();
      }

      if (item.epoch_role == EpochRole::kBand) {
        const auto band_stamp = extract_stamp(item);
        if (voxelcodec_ros::should_drop_as_stale(band_stamp, latest_manifest_stamp_)) {
          const std::size_t stale_bytes = item.serialized->size();
          msgs_stale_dropped_.fetch_add(1, std::memory_order_relaxed);
          bytes_stale_dropped_.fetch_add(stale_bytes, std::memory_order_relaxed);
          RCLCPP_DEBUG(
            get_logger(), "STALE prio=%d  %zu B  epoch=%d.%09u  latest_manifest=%d.%09u",
            item.band_priority, stale_bytes, band_stamp.sec, band_stamp.nanosec,
            latest_manifest_stamp_->sec, latest_manifest_stamp_->nanosec);
          continue;
        }
      }

      const std::size_t nbytes = item.serialized->size();

      // Token bucket: blocks until enough tokens available
      token_bucket_->consume(nbytes);

      // Optional delay
      const double delay_ms = delay_ms_;
      if (delay_ms > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(delay_ms * 1e-3));
      }

      item.publisher->publish(*item.serialized);

      if (item.epoch_role == EpochRole::kManifest) {
        latest_manifest_stamp_ = extract_stamp(item);
      }

      msgs_sent_.fetch_add(1, std::memory_order_relaxed);
      bytes_sent_.fetch_add(nbytes, std::memory_order_relaxed);

      RCLCPP_DEBUG(
        get_logger(), "SEND  prio=%d  %zu B  (total sent: %zu msgs / %zu KB)",
        item.band_priority == std::numeric_limits<int>::max() ? -1 : item.band_priority,
        nbytes,
        msgs_sent_.load(std::memory_order_relaxed),
        bytes_sent_.load(std::memory_order_relaxed) / 1024);
    }
  }

  void log_stats()
  {
    const uint64_t received = msgs_received_.load(std::memory_order_relaxed);
    const uint64_t dropped  = msgs_dropped_.load(std::memory_order_relaxed);
    const uint64_t deduped  = msgs_deduped_.load(std::memory_order_relaxed);
    const uint64_t sent     = msgs_sent_.load(std::memory_order_relaxed);
    const uint64_t kb_sent  = bytes_sent_.load(std::memory_order_relaxed) / 1024;
    const uint64_t stale    = msgs_stale_dropped_.load(std::memory_order_relaxed);
    const uint64_t kb_stale = bytes_stale_dropped_.load(std::memory_order_relaxed) / 1024;
    std::size_t queue_depth;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_depth = queue_.size();
    }
    RCLCPP_INFO(
      get_logger(),
      "stats | rcvd=%lu  sent=%lu (%lu KB)  dropped=%lu  deduped=%lu  stale=%lu (%lu KB)  queued=%zu",
      received, sent, kb_sent, dropped, deduped, stale, kb_stale, queue_depth);
  }

  rcl_interfaces::msg::SetParametersResult on_param_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      if (p.get_name() == "bandwidth_kbps") {
        bandwidth_kbps_ = p.as_double();
        token_bucket_ = std::make_shared<TokenBucket>(bandwidth_kbps_);
        RCLCPP_INFO(get_logger(), "bandwidth_kbps updated to %.0f", bandwidth_kbps_);
      } else if (p.get_name() == "loss_pct") {
        loss_pct_ = p.as_double();
        RCLCPP_INFO(get_logger(), "loss_pct updated to %.1f%%", loss_pct_);
      } else if (p.get_name() == "delay_ms") {
        delay_ms_ = p.as_double();
        RCLCPP_INFO(get_logger(), "delay_ms updated to %.0f", delay_ms_);
      } else if (p.get_name() == "priority_aging_ms") {
        priority_aging_ms_ = p.as_double();
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          queue_.set_aging_interval_ms(priority_aging_ms_);
        }
        RCLCPP_INFO(get_logger(), "priority_aging_ms updated to %.0f", priority_aging_ms_);
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  double bandwidth_kbps_;
  double loss_pct_;
  double delay_ms_;
  double priority_aging_ms_;

  std::shared_ptr<TokenBucket> token_bucket_;
  std::vector<std::shared_ptr<rclcpp::GenericPublisher>> publishers_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  BandQueue queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool shutdown_{false};
  std::thread worker_;

  std::mt19937 rng_;
  std::mutex rng_mutex_;

  // Newest manifest stamp actually sent so far. Written and read only on the
  // worker thread (worker_loop) -- on_message() never touches it -- so it
  // needs no mutex despite being a member shared with the rest of the class.
  std::optional<voxelcodec_ros::Stamp> latest_manifest_stamp_;

  // Cumulative counters (relaxed atomics — only read in stats timer and worker).
  std::atomic<uint64_t> msgs_received_{0};
  std::atomic<uint64_t> msgs_dropped_{0};
  std::atomic<uint64_t> msgs_deduped_{0};
  std::atomic<uint64_t> msgs_sent_{0};
  std::atomic<uint64_t> bytes_sent_{0};
  std::atomic<uint64_t> msgs_stale_dropped_{0};
  std::atomic<uint64_t> bytes_stale_dropped_{0};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DdilProxy>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ddil_proxy"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
