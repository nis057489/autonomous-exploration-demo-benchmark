#pragma once

#include <mutex>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <voxelcodec_msgs/msg/ddil_stats.hpp>

namespace voxelcodec_ros
{

class DdilStatsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DdilStatsPanel(QWidget * parent = nullptr);
  ~DdilStatsPanel() override = default;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onSubscribe();
  void onScan();
  void onRefresh();

private:
  void statsCb(const voxelcodec_msgs::msg::DdilStats::ConstSharedPtr & msg);

  QLineEdit * topic_edit_;
  QPushButton * subscribe_btn_;
  QPushButton * scan_btn_;
  QLabel * config_label_;
  QTableWidget * table_;
  QTimer * refresh_timer_;

  rclcpp::Subscription<voxelcodec_msgs::msg::DdilStats>::SharedPtr stats_sub_;

  std::mutex snapshot_mutex_;
  voxelcodec_msgs::msg::DdilStats snapshot_;
  bool has_data_{false};
};

}  // namespace voxelcodec_ros
