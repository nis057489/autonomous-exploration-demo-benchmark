#include "voxelcodec_ros/voxel_display.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

namespace voxelcodec_ros
{

  namespace
  {

    // Channel names we look for
    constexpr char kChannelX[] = "x";
    constexpr char kChannelY[] = "y";
    constexpr char kChannelZ[] = "z";
    constexpr char kChannelScale[] = "scale";
    constexpr char kChannelOccupancy[] = "occupancy";

    enum ColorMode
    {
      kHeight = 0,
      kOccupancy,
      kFlat,
    };

    std::vector<float> scalar_to_float(const ScalarBuffer &buf)
    {
      return std::visit([](const auto &vec) -> std::vector<float>
                        {
    std::vector<float> result;
    result.reserve(vec.size());
    for (const auto & v : vec) {
      result.push_back(static_cast<float>(v));
    }
    return result; }, buf);
    }

  } // namespace

  VoxelDisplay::VoxelDisplay()
  {
    topic_property_ = new rviz_common::properties::StringProperty(
        "Base Topic", "/voxelcodec",
        "Base topic for voxel channel messages. Leave empty when using File.",
        this, SLOT(updateTopic()));

    file_property_ = new rviz_common::properties::StringProperty(
        "VXCH File", "",
        "Path to a .vxch file to load directly. Overrides Base Topic when set.",
        this, SLOT(updateFile()));

    color_mode_property_ = new rviz_common::properties::EnumProperty(
        "Color Mode", "Height",
        "How to color the voxels.",
        this, SLOT(updateColorMode()));
    color_mode_property_->addOption("Height", kHeight);
    color_mode_property_->addOption("Occupancy", kOccupancy);
    color_mode_property_->addOption("Flat", kFlat);

    flat_color_property_ = new rviz_common::properties::ColorProperty(
        "Flat Color", QColor(70, 130, 230),
        "Color to use when Color Mode is Flat.",
        this, SLOT(updateColorMode()));

    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0f,
        "Opacity of the voxels (0 = transparent, 1 = opaque).",
        this, SLOT(updateAlpha()));
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    show_coarse_property_ = new rviz_common::properties::BoolProperty(
        "Stretch Coarse Voxels", true,
        "When enabled, coarse-level voxels are rendered as larger boxes matching "
        "their actual octree cell size. When disabled, all voxels are the same size.",
        this);

    band_property_ = new rviz_common::properties::IntProperty(
        "Band", 0,
        "Haar wavelet band count (1 = coarsest, 0 or max = full resolution). "
        "Only applies when loading a .vxch file with haar-wavelet channels.",
        this, SLOT(updateBandLevel()));
    band_property_->setMin(0);
    band_property_->setMax(1);

    level_property_ = new rviz_common::properties::IntProperty(
        "Min Wavemap Height", 0,
        "Minimum wavemap height to include (0 = all levels, higher = coarser only). "
        "Only applies when loading a wvmp .vxch file.",
        this, SLOT(updateBandLevel()));
    level_property_->setMin(0);
    level_property_->setMax(0);
  }

  VoxelDisplay::~VoxelDisplay()
  {
    unsubscribe();
    clearDisplay();
  }

  void VoxelDisplay::onInitialize()
  {
    Display::onInitialize();
    const std::string file_path = file_property_->getStdString();
    if (!file_path.empty())
    {
      updateFile();
    }
    else
    {
      updateTopic();
    }
  }

  void VoxelDisplay::onEnable()
  {
    if (!file_property_->getStdString().empty())
    {
      updateFile();
    }
    else
    {
      subscribe();
    }
  }

  void VoxelDisplay::onDisable()
  {
    unsubscribe();
    clearDisplay();
  }

  void VoxelDisplay::reset()
  {
    Display::reset();
    clearDisplay();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      channel_payloads_.clear();
      needs_rebuild_ = false;
    }
  }

  void VoxelDisplay::updateTopic()
  {
    unsubscribe();
    clearDisplay();
    // Only subscribe if no file is set
    if (file_property_->getStdString().empty())
    {
      subscribe();
    }
  }

  void VoxelDisplay::updateFile()
  {
    unsubscribe();
    clearDisplay();
    const std::string path = file_property_->getStdString();
    if (!path.empty())
    {
      loadFile(path);
    }
    else
    {
      // Fall back to topic mode
      subscribe();
    }
  }

  void VoxelDisplay::updateBandLevel()
  {
    if (!cached_archive_)
      return;
    rebuildFromArchive();
  }

  void VoxelDisplay::updateColorMode()
  {
    flat_color_property_->setHidden(
        color_mode_property_->getOptionInt() != kFlat);
    std::lock_guard<std::mutex> lock(mutex_);
    needs_rebuild_ = true;
  }

  void VoxelDisplay::updateAlpha()
  {
    const float alpha = alpha_property_->getFloat();
    for (auto &[scale, layer] : scale_layers_)
    {
      layer.cloud->setAlpha(alpha, false);
    }
  }

  void VoxelDisplay::subscribe()
  {
    if (!isEnabled())
    {
      return;
    }

    const std::string base_topic = topic_property_->getStdString();
    if (base_topic.empty())
    {
      return;
    }

    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

    manifest_sub_ = node->create_subscription<voxelcodec_msgs::msg::VoxelManifest>(
        base_topic + "/manifest", rclcpp::SensorDataQoS(),
        std::bind(&VoxelDisplay::manifestCallback, this, std::placeholders::_1));
  }

  void VoxelDisplay::unsubscribe()
  {
    manifest_sub_.reset();
    channel_subs_.clear();
  }

  void VoxelDisplay::clearDisplay()
  {
    for (auto &[scale, layer] : scale_layers_)
    {
      if (layer.cloud)
      {
        layer.cloud->clear();
      }
      if (layer.node)
      {
        layer.node->detachAllObjects();
        scene_manager_->destroySceneNode(layer.node);
      }
    }
    scale_layers_.clear();
  }

  void VoxelDisplay::manifestCallback(
      const voxelcodec_msgs::msg::VoxelManifest::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_manifest_ = manifest_from_msg(*msg);
    channel_payloads_.clear();

    // Set fixed frame from message header if available
    if (!msg->header.frame_id.empty())
    {
      setStatus(rviz_common::properties::StatusProperty::Ok, "Frame",
                QString::fromStdString(msg->header.frame_id));
      fixed_frame_ = QString::fromStdString(msg->header.frame_id);
    }

    // Subscribe to channels we need
    const std::string base_topic = topic_property_->getStdString();
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

    // Determine which channels to subscribe to
    std::vector<std::string> wanted = {kChannelX, kChannelY, kChannelZ};
    for (const auto &desc : current_manifest_.channels)
    {
      if (desc.name == kChannelScale || desc.name == kChannelOccupancy)
      {
        wanted.push_back(desc.name);
      }
    }

    // Create subscriptions for missing channels
    for (const auto &name : wanted)
    {
      if (channel_subs_.count(name))
      {
        continue;
      }
      const std::string topic = channel_topic(base_topic, name);
      channel_subs_[name] = node->create_subscription<voxelcodec_msgs::msg::VoxelChannel>(
          topic, rclcpp::SensorDataQoS(),
          std::bind(&VoxelDisplay::channelCallback, this, std::placeholders::_1));
    }

    // Remove subscriptions for channels no longer needed
    for (auto it = channel_subs_.begin(); it != channel_subs_.end();)
    {
      if (std::find(wanted.begin(), wanted.end(), it->first) == wanted.end())
      {
        it = channel_subs_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  void VoxelDisplay::channelCallback(
      const voxelcodec_msgs::msg::VoxelChannel::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    channel_payloads_[msg->descriptor.name] = msg->payload;

    // Check if we have all required channels
    if (channel_payloads_.count(kChannelX) &&
        channel_payloads_.count(kChannelY) &&
        channel_payloads_.count(kChannelZ))
    {
      needs_rebuild_ = true;
    }
  }

  void VoxelDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!needs_rebuild_)
    {
      return;
    }
    needs_rebuild_ = false;
    rebuildVisuals();
  }

  void VoxelDisplay::loadFile(const std::string &path)
  {
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "File", QString("Cannot open: %1").arg(QString::fromStdString(path)));
      return;
    }
    std::vector<uint8_t> bytes(
        (std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());

    try
    {
      auto archive = std::make_unique<Archive>(read_archive(bytes));

      // Discover max bands and max wvmp height
      int discovered_max_bands = 1;
      int discovered_max_height = -1;
      for (const auto &desc : archive->manifest.channels)
      {
        int mb = haar_max_bands(desc);
        if (mb > discovered_max_bands)
          discovered_max_bands = mb;
        auto h_it = desc.metadata.find("wvmp_height");
        if (h_it != desc.metadata.end())
        {
          int h = std::stoi(h_it->second);
          if (h > discovered_max_height)
            discovered_max_height = h;
        }
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        cached_archive_ = std::move(archive);
        max_bands_ = discovered_max_bands;
        max_level_ = std::max(0, discovered_max_height);
      }

      // Update property ranges
      band_property_->setMax(discovered_max_bands);
      if (band_property_->getInt() == 0 || band_property_->getInt() > discovered_max_bands)
      {
        band_property_->setInt(0); // 0 = full resolution
      }
      level_property_->setMax(std::max(0, discovered_max_height));
      if (level_property_->getInt() > std::max(0, discovered_max_height))
      {
        level_property_->setInt(0);
      }

      rebuildFromArchive();
    }
    catch (const std::exception &e)
    {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "File", QString::fromStdString(e.what()));
    }
  }

  void VoxelDisplay::rebuildFromArchive()
  {
    if (!cached_archive_)
      return;

    const int band = band_property_->getInt(); // 0 = full
    const int min_height = level_property_->getInt();

    try
    {
      const auto &archive = *cached_archive_;

      // Detect wvmp mode
      struct LevelInfo
      {
        uint32_t scale;
        std::map<std::string, const ChannelDescriptor *> channels;
      };
      std::map<int, LevelInfo> wvmp_levels;

      for (const auto &desc : archive.manifest.channels)
      {
        auto it = desc.metadata.find("wvmp_height");
        if (it == desc.metadata.end())
          continue;
        int height = std::stoi(it->second);
        auto scale_it = desc.metadata.find("wvmp_scale");
        uint32_t scale = scale_it != desc.metadata.end()
                             ? static_cast<uint32_t>(std::stoul(scale_it->second))
                             : (1u << height);
        std::string axis = desc.name.substr(0, desc.name.find('.'));
        if (axis == "occ")
          axis = "occupancy";
        wvmp_levels[height].scale = scale;
        wvmp_levels[height].channels[axis] = &desc;
      }

      auto to_raw_bytes = [](const std::vector<uint32_t> &vals) -> std::vector<uint8_t>
      {
        std::vector<uint8_t> raw(vals.size() * 4);
        for (size_t i = 0; i < vals.size(); ++i)
        {
          uint32_t v = vals[i];
          raw[i * 4 + 0] = static_cast<uint8_t>(v & 0xff);
          raw[i * 4 + 1] = static_cast<uint8_t>((v >> 8) & 0xff);
          raw[i * 4 + 2] = static_cast<uint8_t>((v >> 16) & 0xff);
          raw[i * 4 + 3] = static_cast<uint8_t>((v >> 24) & 0xff);
        }
        return raw;
      };

      std::lock_guard<std::mutex> lock(mutex_);

      if (!wvmp_levels.empty())
      {
        std::vector<uint32_t> combined_x, combined_y, combined_z, combined_scale, combined_occ;

        for (auto rit = wvmp_levels.rbegin(); rit != wvmp_levels.rend(); ++rit)
        {
          int height = rit->first;
          if (height < min_height)
            continue; // Level filter

          auto &level = rit->second;
          auto x_it = level.channels.find("x");
          auto y_it = level.channels.find("y");
          auto z_it = level.channels.find("z");
          if (x_it == level.channels.end() || y_it == level.channels.end() ||
              z_it == level.channels.end())
          {
            continue;
          }

          auto decode_u32 = [&](const ChannelDescriptor *desc) -> std::vector<uint32_t>
          {
            auto payload_it = archive.payloads.find(desc->name);
            if (payload_it == archive.payloads.end())
              return {};
            auto decoded = decode_haar_progressive(*desc, payload_it->second, band);
            return std::get<std::vector<uint32_t> >(decoded.values);
          };

          auto xs = decode_u32(x_it->second);
          auto ys = decode_u32(y_it->second);
          auto zs = decode_u32(z_it->second);
          size_t count = std::min({xs.size(), ys.size(), zs.size()});

          combined_x.insert(combined_x.end(), xs.begin(), xs.begin() + count);
          combined_y.insert(combined_y.end(), ys.begin(), ys.begin() + count);
          combined_z.insert(combined_z.end(), zs.begin(), zs.begin() + count);
          combined_scale.insert(combined_scale.end(), count, level.scale);

          auto occ_it = level.channels.find("occupancy");
          if (occ_it != level.channels.end())
          {
            auto occs = decode_u32(occ_it->second);
            if (occs.size() >= count)
            {
              combined_occ.insert(combined_occ.end(), occs.begin(), occs.begin() + count);
            }
            else
            {
              combined_occ.insert(combined_occ.end(), count, 1000000u);
            }
          }
          else
          {
            combined_occ.insert(combined_occ.end(), count, 1000000u);
          }
        }

        current_manifest_ = archive.manifest;
        current_manifest_.voxel_count = static_cast<uint32_t>(combined_x.size());
        current_manifest_.channels.clear();

        auto make_desc = [](const std::string &name, const std::string &semantic,
                            size_t count) -> ChannelDescriptor
        {
          ChannelDescriptor d;
          d.name = name;
          d.semantic = semantic;
          d.data_type = kDataTypeUint32;
          d.encoding = kEncodingRawLE;
          d.compression = kCompressionNone;
          d.element_count = static_cast<uint32_t>(count);
          d.uncompressed_size = count * 4;
          d.compressed_size = count * 4;
          return d;
        };

        size_t n = combined_x.size();
        current_manifest_.channels.push_back(make_desc("x", "position/x", n));
        current_manifest_.channels.push_back(make_desc("y", "position/y", n));
        current_manifest_.channels.push_back(make_desc("z", "position/z", n));
        current_manifest_.channels.push_back(make_desc("scale", "voxel.scale", n));
        current_manifest_.channels.push_back(make_desc("occupancy", "occupancy.probability", n));

        channel_payloads_.clear();
        channel_payloads_["x"] = to_raw_bytes(combined_x);
        channel_payloads_["y"] = to_raw_bytes(combined_y);
        channel_payloads_["z"] = to_raw_bytes(combined_z);
        channel_payloads_["scale"] = to_raw_bytes(combined_scale);
        channel_payloads_["occupancy"] = to_raw_bytes(combined_occ);
      }
      else
      {
        const auto source_format_it = archive.manifest.metadata.find("source_format");
        const bool is_wvmp_coeff_archive =
            source_format_it != archive.manifest.metadata.end() &&
            source_format_it->second == "wvmp_coefficients";

        if (is_wvmp_coeff_archive)
        {
          std::string available;
          for (size_t i = 0; i < archive.manifest.channels.size(); ++i)
          {
            if (i > 0)
            {
              available += ", ";
            }
            available += archive.manifest.channels[i].name;
          }

          setStatus(rviz_common::properties::StatusProperty::Warn,
                    "Channels",
                    QString("This .vxch stores wavemap coefficients (source_format=wvmp_coefficients), not direct x/y/z voxel channels. Available: %1")
                        .arg(QString::fromStdString(available)));
          channel_payloads_.clear();
          needs_rebuild_ = false;
          return;
        }

        // Standard mode (non-wvmp): decode all channels, applying progressive haar if needed
        current_manifest_ = archive.manifest;
        channel_payloads_.clear();

        for (auto &desc : current_manifest_.channels)
        {
          auto payload_it = archive.payloads.find(desc.name);
          if (payload_it == archive.payloads.end())
            continue;

          DecodedChannel decoded;
          if (desc.encoding == kEncodingHaarWavelet)
          {
            decoded = decode_haar_progressive(desc, payload_it->second, band);
          }
          else
          {
            decoded = decode_channel(desc, payload_it->second);
          }

          // Convert decoded values to raw-le bytes
          auto raw = std::visit([](const auto &vec) -> std::vector<uint8_t>
                                {
          std::vector<uint8_t> raw(vec.size() * sizeof(typename std::remove_reference_t<decltype(vec)>::value_type));
          std::memcpy(raw.data(), vec.data(), raw.size());
          return raw; }, decoded.values);
          channel_payloads_[desc.name] = std::move(raw);

          // Update descriptor so rebuildVisuals sees raw-le
          std::size_t elem_count = std::visit([](const auto &v)
                                              { return v.size(); }, decoded.values);
          desc.encoding = kEncodingRawLE;
          desc.compression = kCompressionNone;
          desc.element_count = static_cast<uint32_t>(elem_count);
          desc.uncompressed_size = channel_payloads_[desc.name].size();
          desc.compressed_size = channel_payloads_[desc.name].size();
        }
        current_manifest_.voxel_count = current_manifest_.channels.empty()
                                            ? 0
                                            : current_manifest_.channels[0].element_count;
      }

      needs_rebuild_ = true;

      int active_levels = 0;
      for (auto rit = wvmp_levels.rbegin(); rit != wvmp_levels.rend(); ++rit)
      {
        if (rit->first >= min_height)
          active_levels++;
      }
      size_t total = 0;
      if (channel_payloads_.count("x"))
        total = channel_payloads_["x"].size() / 4;
      setStatus(rviz_common::properties::StatusProperty::Ok,
                "File",
                QString("%1 voxels, band=%2/%3, levels=%4")
                    .arg(total)
                    .arg(band == 0 ? max_bands_ : band)
                    .arg(max_bands_)
                    .arg(active_levels > 0 ? active_levels : static_cast<int>(archive.manifest.channels.size())));
    }
    catch (const std::exception &e)
    {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Decode", QString::fromStdString(e.what()));
    }
  }

  void VoxelDisplay::rebuildVisuals()
  {
    // Clear existing visuals
    clearDisplay();

    // Find channel descriptors
    const ChannelDescriptor *x_desc = nullptr;
    const ChannelDescriptor *y_desc = nullptr;
    const ChannelDescriptor *z_desc = nullptr;
    const ChannelDescriptor *scale_desc = nullptr;
    const ChannelDescriptor *occ_desc = nullptr;
    for (const auto &desc : current_manifest_.channels)
    {
      if (desc.name == kChannelX)
      {
        x_desc = &desc;
      }
      else if (desc.name == kChannelY)
      {
        y_desc = &desc;
      }
      else if (desc.name == kChannelZ)
      {
        z_desc = &desc;
      }
      else if (desc.name == kChannelScale)
      {
        scale_desc = &desc;
      }
      else if (desc.name == kChannelOccupancy)
      {
        occ_desc = &desc;
      }
    }

    if (!x_desc || !y_desc || !z_desc)
    {
      std::string available;
      for (size_t i = 0; i < current_manifest_.channels.size(); ++i)
      {
        if (i > 0)
        {
          available += ", ";
        }
        available += current_manifest_.channels[i].name;
      }
      setStatus(rviz_common::properties::StatusProperty::Warn,
                "Channels",
                QString("Missing x, y, or z channel. Available: %1")
                    .arg(QString::fromStdString(available)));
      return;
    }

    // Decode channels
    std::vector<float> xs, ys, zs, scales, occs;
    try
    {
      auto x_payload_it = channel_payloads_.find(kChannelX);
      auto y_payload_it = channel_payloads_.find(kChannelY);
      auto z_payload_it = channel_payloads_.find(kChannelZ);
      if (x_payload_it == channel_payloads_.end() ||
          y_payload_it == channel_payloads_.end() ||
          z_payload_it == channel_payloads_.end())
      {
        return;
      }

      xs = scalar_to_float(decode_channel(*x_desc, x_payload_it->second).values);
      ys = scalar_to_float(decode_channel(*y_desc, y_payload_it->second).values);
      zs = scalar_to_float(decode_channel(*z_desc, z_payload_it->second).values);

      if (scale_desc && channel_payloads_.count(kChannelScale))
      {
        scales = scalar_to_float(
            decode_channel(*scale_desc, channel_payloads_[kChannelScale]).values);
      }
      if (occ_desc && channel_payloads_.count(kChannelOccupancy))
      {
        occs = scalar_to_float(
            decode_channel(*occ_desc, channel_payloads_[kChannelOccupancy]).values);
      }
    }
    catch (const std::exception &e)
    {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Decode", QString::fromStdString(e.what()));
      return;
    }

    const size_t count = std::min({xs.size(), ys.size(), zs.size()});
    if (count == 0)
    {
      return;
    }

    // Get resolution from manifest metadata
    float resolution = 1.0f;
    auto res_it = current_manifest_.metadata.find("resolution");
    if (res_it != current_manifest_.metadata.end())
    {
      resolution = std::stof(res_it->second);
    }

    // Get grid-index origin offset (grid_origin_ix/iy/iz × resolution → world)
    float origin_x = 0.0f, origin_y = 0.0f, origin_z = 0.0f;
    auto gx_it = current_manifest_.metadata.find("grid_origin_ix");
    auto gy_it = current_manifest_.metadata.find("grid_origin_iy");
    auto gz_it = current_manifest_.metadata.find("grid_origin_iz");
    if (gx_it != current_manifest_.metadata.end())
    {
      origin_x = std::stof(gx_it->second) * resolution;
    }
    if (gy_it != current_manifest_.metadata.end())
    {
      origin_y = std::stof(gy_it->second) * resolution;
    }
    if (gz_it != current_manifest_.metadata.end())
    {
      origin_z = std::stof(gz_it->second) * resolution;
    }

    const bool stretch = show_coarse_property_->getBool() && !scales.empty();
    const int color_mode = color_mode_property_->getOptionInt();
    const float alpha = alpha_property_->getFloat();
    const QColor flat_qcolor = flat_color_property_->getColor();
    const Ogre::ColourValue flat_color(
        flat_qcolor.redF(), flat_qcolor.greenF(), flat_qcolor.blueF(), alpha);

    // Compute z range for height coloring
    float z_min = *std::min_element(zs.begin(), zs.begin() + count);
    float z_max = *std::max_element(zs.begin(), zs.begin() + count);
    if (z_max <= z_min)
    {
      z_max = z_min + 1.0f;
    }

    // Group points by scale
    std::map<uint32_t, std::vector<rviz_rendering::PointCloud::Point> > groups;
    for (size_t i = 0; i < count; ++i)
    {
      uint32_t scale_val = 1;
      if (stretch && i < scales.size())
      {
        scale_val = std::max(1u, static_cast<uint32_t>(scales[i]));
      }

      rviz_rendering::PointCloud::Point pt;
      pt.position.x = xs[i] * resolution + origin_x;
      pt.position.y = ys[i] * resolution + origin_y;
      pt.position.z = zs[i] * resolution + origin_z;

      // Offset coarse voxels to center of their cell
      if (scale_val > 1)
      {
        float half = (scale_val - 1) * resolution * 0.5f;
        pt.position.x += half;
        pt.position.y += half;
        pt.position.z += half;
      }

      switch (color_mode)
      {
      case kHeight:
      {
        float t = (zs[i] - z_min) / (z_max - z_min);
        pt.color = heightToColor(t);
        pt.color.a = alpha;
        break;
      }
      case kOccupancy:
      {
        float occ = 1.0f;
        if (i < occs.size())
        {
          occ = occs[i] / 1000000.0f; // q1e6 to [0,1]
        }
        pt.color = occupancyToColor(occ);
        pt.color.a = alpha;
        break;
      }
      case kFlat:
      default:
        pt.color = flat_color;
        break;
      }

      groups[scale_val].push_back(pt);
    }

    // Create one PointCloud per scale level, each on its own child scene node
    for (auto &[scale_val, points] : groups)
    {
      auto cloud = std::make_unique<rviz_rendering::PointCloud>();
      cloud->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);

      const float dim = static_cast<float>(scale_val) * resolution;
      cloud->setDimensions(dim, dim, dim);
      cloud->setAlpha(alpha, false);

      // Create a child scene node for this scale layer (isolates Ogre objects)
      Ogre::SceneNode *child = scene_node_->createChildSceneNode();
      child->attachObject(cloud.get());

      // Add points AFTER attaching so sub-renderables attach to the child node
      cloud->addPoints(points.begin(), points.end());

      ScaleLayer layer;
      layer.cloud = std::move(cloud);
      layer.node = child;
      scale_layers_[scale_val] = std::move(layer);
    }

    setStatus(rviz_common::properties::StatusProperty::Ok,
              "Voxels",
              QString::number(count) + " voxels in " +
                  QString::number(groups.size()) + " scale layers");
  }

  Ogre::ColourValue VoxelDisplay::heightToColor(float t) const
  {
    // Rainbow colormap: blue → cyan → green → yellow → red
    t = std::clamp(t, 0.0f, 1.0f);
    float r, g, b;
    if (t < 0.25f)
    {
      float s = t / 0.25f;
      r = 0.0f;
      g = s;
      b = 1.0f;
    }
    else if (t < 0.5f)
    {
      float s = (t - 0.25f) / 0.25f;
      r = 0.0f;
      g = 1.0f;
      b = 1.0f - s;
    }
    else if (t < 0.75f)
    {
      float s = (t - 0.5f) / 0.25f;
      r = s;
      g = 1.0f;
      b = 0.0f;
    }
    else
    {
      float s = (t - 0.75f) / 0.25f;
      r = 1.0f;
      g = 1.0f - s;
      b = 0.0f;
    }
    return Ogre::ColourValue(r, g, b, 1.0f);
  }

  Ogre::ColourValue VoxelDisplay::occupancyToColor(float occ) const
  {
    occ = std::clamp(occ, 0.0f, 1.0f);
    // Gray scale: darker = more occupied
    float v = 1.0f - occ * 0.8f;
    return Ogre::ColourValue(v, v, v, 1.0f);
  }

} // namespace voxelcodec_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxelcodec_ros::VoxelDisplay, rviz_common::Display)
