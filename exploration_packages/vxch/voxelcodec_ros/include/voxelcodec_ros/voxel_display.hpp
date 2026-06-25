#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>

#include <rclcpp/rclcpp.hpp>
#include <voxelcodec_msgs/msg/voxel_channel.hpp>
#include <voxelcodec_msgs/msg/voxel_manifest.hpp>

#include "voxelcodec_ros/ros_messages.hpp"
#include "voxelcodec_ros/codec.hpp"

namespace voxelcodec_ros
{

class VoxelDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  VoxelDisplay();
  ~VoxelDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private Q_SLOTS:
  void updateTopic();
  void updateFile();
  void updateColorMode();
  void updateAlpha();
  void updateBandLevel();

private:
  void subscribe();
  void unsubscribe();
  void clearDisplay();

  void manifestCallback(const voxelcodec_msgs::msg::VoxelManifest::ConstSharedPtr & msg);
  void channelCallback(const voxelcodec_msgs::msg::VoxelChannel::ConstSharedPtr & msg);

  void rebuildVisuals();
  void loadFile(const std::string & path);
  void rebuildFromArchive();

  Ogre::ColourValue heightToColor(float z) const;
  Ogre::ColourValue occupancyToColor(float occ) const;

  // Properties
  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::StringProperty * file_property_;
  rviz_common::properties::EnumProperty * color_mode_property_;
  rviz_common::properties::ColorProperty * flat_color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::BoolProperty * show_coarse_property_;
  rviz_common::properties::IntProperty * band_property_;
  rviz_common::properties::IntProperty * level_property_;

  // Subscriptions
  rclcpp::Subscription<voxelcodec_msgs::msg::VoxelManifest>::SharedPtr manifest_sub_;
  std::unordered_map<std::string, rclcpp::Subscription<voxelcodec_msgs::msg::VoxelChannel>::SharedPtr> channel_subs_;

  // Buffered channel data (guarded by mutex_)
  std::mutex mutex_;
  Manifest current_manifest_;
  std::unordered_map<std::string, std::vector<uint8_t>> channel_payloads_;
  bool needs_rebuild_{false};

  // Cached archive for re-decoding with different band/level settings
  std::unique_ptr<Archive> cached_archive_;
  int max_bands_{1};
  int max_level_{0};  // highest wvmp height

  // Visual layers: one PointCloud per unique scale value, each on its own child scene node
  // Key = scale (1, 2, 4, 8, 16, 32)
  struct ScaleLayer {
    std::unique_ptr<rviz_rendering::PointCloud> cloud;
    Ogre::SceneNode * node{nullptr};
  };
  std::map<uint32_t, ScaleLayer> scale_layers_;
};

}  // namespace voxelcodec_ros
