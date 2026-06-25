#pragma once

#include <atomic>
#include <mutex>

#include <rviz_common/panel.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace voxelcodec_ros
{

class BandwidthPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BandwidthPanel(QWidget * parent = nullptr);
  ~BandwidthPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onSubscribe();
  void onRefresh();

private:
  struct TopicStats {
    std::mutex mu;
    uint64_t msg_count{0};
    uint64_t total_bytes{0};    // cumulative payload bytes
    uint64_t last_msg_bytes{0};
    double first_time{0.0};
    double last_time{0.0};
  };

  void nativeCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void vxchCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  static QString fmtBytes(uint64_t b);
  static QString fmtRate(const TopicStats & s);

  QLineEdit * native_topic_edit_;
  QLineEdit * vxch_topic_edit_;
  QPushButton * subscribe_btn_;

  QLabel * native_count_label_;
  QLabel * native_total_label_;
  QLabel * native_rate_label_;
  QLabel * native_last_label_;

  QLabel * vxch_count_label_;
  QLabel * vxch_total_label_;
  QLabel * vxch_rate_label_;
  QLabel * vxch_last_label_;

  QLabel * ratio_label_;

  QTimer * refresh_timer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr native_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr vxch_sub_;

  TopicStats native_stats_;
  TopicStats vxch_stats_;
};

}  // namespace voxelcodec_ros
