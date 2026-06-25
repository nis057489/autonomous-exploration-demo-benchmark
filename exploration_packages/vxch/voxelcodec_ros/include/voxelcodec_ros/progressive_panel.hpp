#pragma once

#include <memory>

#include <rviz_common/panel.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <voxelcodec_msgs/srv/set_band.hpp>
#include <voxelcodec_msgs/srv/set_level.hpp>

namespace voxelcodec_ros
{

class ProgressivePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ProgressivePanel(QWidget * parent = nullptr);
  ~ProgressivePanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onConnect();
  void onBandUp();
  void onBandDown();
  void onBandMax();
  void onBandMin();
  void onLevelUp();
  void onLevelDown();
  void onLevelMax();
  void onLevelMin();

private:
  void callSetBand(int band);
  void callSetLevel(int wvmp_min_height);
  void updateBandLabels(
    int current, int max_bands, int voxels, int unique,
    int64_t cum_bytes, int64_t total_bytes);
  void updateLevelLabels(
    int current_min_height, int height_max, int height_min,
    int voxels, int active_levels);

  QLineEdit * service_edit_;
  QPushButton * connect_btn_;

  // Band controls
  QPushButton * btn_band_up_;
  QPushButton * btn_band_down_;
  QPushButton * btn_band_max_;
  QPushButton * btn_band_min_;
  QLabel * band_label_;
  QLabel * voxel_label_;
  QLabel * bandwidth_label_;

  // Level controls
  QPushButton * btn_level_up_;
  QPushButton * btn_level_down_;
  QPushButton * btn_level_max_;
  QPushButton * btn_level_min_;
  QLabel * level_label_;
  QLabel * level_voxel_label_;

  rclcpp::Client<voxelcodec_msgs::srv::SetBand>::SharedPtr band_client_;
  rclcpp::Client<voxelcodec_msgs::srv::SetLevel>::SharedPtr level_client_;
  int current_band_{1};
  int max_bands_{1};
  int current_wvmp_min_height_{0};
  int wvmp_height_max_{0};
  int wvmp_height_min_{0};
};

}  // namespace voxelcodec_ros
