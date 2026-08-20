#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>

#include <rviz_common/panel.hpp>

#include <QLabel>
#include <QProgressBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
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

  static QString fmtBytes(uint64_t b);
  static QString fmtRate(double bytes_per_sec);
  static QString fmtEta(double sec);
  static QString linkLabel(const std::string & node_name);

  QWidget * cards_container_{nullptr};
  QVBoxLayout * cards_layout_{nullptr};
  QLabel * empty_label_{nullptr};

  QTimer * rescan_timer_{nullptr};
  QTimer * refresh_timer_{nullptr};

  std::map<std::string, rclcpp::Subscription<voxelcodec_msgs::msg::DdilStats>::SharedPtr> subs_;
  std::map<std::string, LinkSnapshot> snapshots_;
  std::mutex snapshots_mutex_;

  std::map<std::string, LinkCard> cards_;  // keyed by topic, Qt-thread only
};

}  // namespace voxelcodec_ros
