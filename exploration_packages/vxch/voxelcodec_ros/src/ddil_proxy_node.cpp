#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace
{

struct RelayConfig
{
  std::string input_topic;
  std::string output_topic;
  std::string msg_type;
  bool bypass{false};    // bypass throttle (manifest topics etc.)
  bool reliable{false};  // use RELIABLE + TRANSIENT_LOCAL (for Nav2 consumers)
};

RelayConfig parse_relay_entry(const std::string & entry)
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
int band_index_from_topic(const std::string & topic)
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
    rclcpp::QoS qos(1);
    if (cfg.bypass || cfg.reliable) {
      qos.reliable().durability(rclcpp::DurabilityPolicy::TransientLocal);
    } else {
      qos.best_effort();
    }

    auto pub = create_generic_publisher(cfg.output_topic, cfg.msg_type, qos);
    publishers_.push_back(pub);

    const bool is_bypass = cfg.bypass;
    const std::string input_topic = cfg.input_topic;
    auto sub = create_generic_subscription(
      cfg.input_topic, cfg.msg_type, qos,
      [this, pub, is_bypass, input_topic](
        std::shared_ptr<rclcpp::SerializedMessage> serialized) {
        on_message(serialized, pub, is_bypass, input_topic);
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
    const std::string & input_topic)
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

    const int band_idx = band_index_from_topic(input_topic);
    if (band_idx >= 0) {
      item.band_priority = band_idx;
      // Dedup key: publisher address + band index so each (channel, band) slot is independent.
      item.dedup_key =
        std::to_string(reinterpret_cast<std::uintptr_t>(pub.get())) +
        ":band_" + std::to_string(band_idx);
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

      const std::size_t nbytes = item.serialized->size();

      // Token bucket: blocks until enough tokens available
      token_bucket_->consume(nbytes);

      // Optional delay
      const double delay_ms = delay_ms_;
      if (delay_ms > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(delay_ms * 1e-3));
      }

      item.publisher->publish(*item.serialized);

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
    std::size_t queue_depth;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_depth = queue_.size();
    }
    RCLCPP_INFO(
      get_logger(),
      "stats | rcvd=%lu  sent=%lu (%lu KB)  dropped=%lu  deduped=%lu  queued=%zu",
      received, sent, kb_sent, dropped, deduped, queue_depth);
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
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  double bandwidth_kbps_;
  double loss_pct_;
  double delay_ms_;

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

  // Cumulative counters (relaxed atomics — only read in stats timer and worker).
  std::atomic<uint64_t> msgs_received_{0};
  std::atomic<uint64_t> msgs_dropped_{0};
  std::atomic<uint64_t> msgs_deduped_{0};
  std::atomic<uint64_t> msgs_sent_{0};
  std::atomic<uint64_t> bytes_sent_{0};
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
