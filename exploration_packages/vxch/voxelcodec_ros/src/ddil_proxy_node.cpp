#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
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
    auto sub = create_generic_subscription(
      cfg.input_topic, cfg.msg_type, qos,
      [this, pub, is_bypass](std::shared_ptr<rclcpp::SerializedMessage> serialized) {
        on_message(serialized, pub, is_bypass);
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
    bool bypass)
  {
    if (bypass) {
      pub->publish(*serialized);
      return;
    }

    // Random drop
    {
      std::lock_guard<std::mutex> lock(rng_mutex_);
      const double roll = std::uniform_real_distribution<double>(0.0, 100.0)(rng_);
      if (roll < loss_pct_) {
        return;
      }
    }

    // Queue for worker thread (token bucket + delay)
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back({serialized, pub});
    queue_cv_.notify_one();
  }

  void worker_loop()
  {
    while (true) {
      QueuedMessage item;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {return !queue_.empty() || shutdown_;});
        if (shutdown_ && queue_.empty()) {break;}
        item = queue_.front();
        queue_.pop_front();
      }

      // Token bucket: blocks until enough tokens available
      token_bucket_->consume(item.serialized->size());

      // Optional delay
      const double delay_ms = delay_ms_;
      if (delay_ms > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(delay_ms * 1e-3));
      }

      item.publisher->publish(*item.serialized);
    }
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

  std::deque<QueuedMessage> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool shutdown_{false};
  std::thread worker_;

  std::mt19937 rng_;
  std::mutex rng_mutex_;
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
