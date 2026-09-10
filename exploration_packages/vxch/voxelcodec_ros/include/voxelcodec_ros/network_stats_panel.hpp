#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>

#include <rviz_common/panel.hpp>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <voxelcodec_msgs/msg/ddil_stats.hpp>

namespace voxelcodec_ros
{

// "Download manager" style view of every active VXCH DDIL link (one card per
// ddil_proxy_node instance, auto-discovered by scanning for DdilStats topics
// -- no manual topic entry needed, matching the multi-link/multi-robot
// topology where each (robot, peer) pair gets its own proxy). Each card shows
// the link's capacity/current throughput and one row per Haar band: how much
// data is queued for it, whether it's the one currently being sent, and an
// ETA for its backlog to drain -- the same at-a-glance shape as a browser or
// torrent download list, just per-band instead of per-file.
class NetworkStatsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NetworkStatsPanel(QWidget * parent = nullptr);
  ~NetworkStatsPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onRescan();
  void onRefresh();
  // Pushes the "all links" spinbox value to every discovered link at once --
  // the common case, since a run's links are configured identically and a
  // sweep is about the whole team's radio conditions, not one pair's.
  void onApplyAll();

private:
  // One band's row of widgets inside a LinkCard, plus enough client-side
  // state to render a meaningful progress bar out of a continuously
  // regenerating queue backlog (see updateBandRow()).
  struct BandRow
  {
    QWidget * row{nullptr};
    QLabel * band_label{nullptr};
    QLabel * status_badge{nullptr};
    QLabel * size_label{nullptr};
    QProgressBar * progress{nullptr};
    QLabel * eta_label{nullptr};
    uint64_t baseline_pending_bytes{0};
  };

  // One card per discovered DDIL link (one ddil_proxy_node instance).
  struct LinkCard
  {
    QWidget * box{nullptr};
    QLabel * capacity_label{nullptr};
    QLabel * throughput_label{nullptr};
    QLabel * totals_label{nullptr};
    QVBoxLayout * rows_layout{nullptr};
    std::map<int, BandRow> band_rows;  // keyed by band_index, Qt-thread only

    // Live bandwidth control for this link. bandwidth_kbps is a dynamically
    // reconfigurable parameter on ddil_proxy_node, so setting it re-creates
    // that node's token bucket (and resizes its queue budget) in place -- no
    // relaunch, and the effect on per-band queue drain shows up in this same
    // card within a second or two.
    QDoubleSpinBox * bandwidth_spin{nullptr};
    QPushButton * bandwidth_apply{nullptr};
    QLabel * bandwidth_note{nullptr};
    // Suppresses overwriting the spinbox out from under a user who is mid-edit:
    // the value is seeded from the first stats message and from then on only
    // the user (or an Apply-all) changes it.
    bool bandwidth_seeded{false};
    bool externally_shaped{false};
    // Async parameter client for this link's proxy node, created lazily on
    // first Apply. Held so its pending futures outlive the call.
    std::shared_ptr<rclcpp::AsyncParametersClient> param_client;
  };

  // Latest message per topic, filled in on the ROS executor thread; onRefresh()
  // (Qt main thread, via refresh_timer_) is the only reader -- same
  // snapshot-then-poll pattern BandwidthPanel uses to keep widget mutation on
  // the Qt thread.
  struct LinkSnapshot
  {
    voxelcodec_msgs::msg::DdilStats msg;
    bool dirty{false};
  };

  void statsCb(const std::string & topic, voxelcodec_msgs::msg::DdilStats::ConstSharedPtr msg);
  void ensureLinkCard(const std::string & topic, const std::string & link_name);
  void updateLinkCard(LinkCard & card, const voxelcodec_msgs::msg::DdilStats & msg);
  void updateBandRow(BandRow & row, const voxelcodec_msgs::msg::DdilBandStatus & bs);
  // Sets bandwidth_kbps on one link's proxy node. `topic` is that link's
  // DdilStats topic, which is "<fully-qualified node name>/ddil_stats" (the
  // publisher is created as "~/ddil_stats"), so the node name to address the
  // parameter service at is derivable from it -- no extra discovery needed.
  void applyBandwidth(const std::string & topic, LinkCard & card, double kbps);
  static std::string nodeNameFromStatsTopic(const std::string & topic);

  static QString fmtBytes(uint64_t b);
  static QString fmtRate(double bytes_per_sec);
  static QString fmtEta(double sec);
  static QString linkLabel(const std::string & node_name);

  QWidget * cards_container_{nullptr};
  QVBoxLayout * cards_layout_{nullptr};
  QLabel * empty_label_{nullptr};
  QDoubleSpinBox * all_bandwidth_spin_{nullptr};
  QPushButton * all_bandwidth_apply_{nullptr};
  QLabel * apply_status_{nullptr};

  QTimer * rescan_timer_{nullptr};
  QTimer * refresh_timer_{nullptr};

  std::map<std::string, rclcpp::Subscription<voxelcodec_msgs::msg::DdilStats>::SharedPtr> subs_;
  std::map<std::string, LinkSnapshot> snapshots_;
  std::mutex snapshots_mutex_;

  std::map<std::string, LinkCard> cards_;  // keyed by topic, Qt-thread only
};

}  // namespace voxelcodec_ros
