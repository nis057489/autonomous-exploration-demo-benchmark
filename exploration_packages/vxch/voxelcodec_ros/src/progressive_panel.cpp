#include "voxelcodec_ros/progressive_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace voxelcodec_ros
{

ProgressivePanel::ProgressivePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  // Service topic
  auto * svc_row = new QHBoxLayout();
  svc_row->addWidget(new QLabel("Service:"));
  service_edit_ = new QLineEdit("/voxelcodec/set_band");
  svc_row->addWidget(service_edit_);
  connect_btn_ = new QPushButton("Connect");
  svc_row->addWidget(connect_btn_);
  layout->addLayout(svc_row);

  // --- Band controls (VXCH Haar LOD) ---
  layout->addWidget(new QLabel("VXCH Band (Haar LOD):"));
  auto * band_row = new QHBoxLayout();
  btn_band_min_ = new QPushButton("|<");
  btn_band_down_ = new QPushButton("-");
  btn_band_up_ = new QPushButton("+");
  btn_band_max_ = new QPushButton(">|");
  btn_band_min_->setToolTip("Coarsest (band 1)");
  btn_band_down_->setToolTip("Decrease band");
  btn_band_up_->setToolTip("Increase band");
  btn_band_max_->setToolTip("Full resolution");
  band_row->addWidget(btn_band_min_);
  band_row->addWidget(btn_band_down_);
  band_row->addWidget(btn_band_up_);
  band_row->addWidget(btn_band_max_);
  layout->addLayout(band_row);

  band_label_ = new QLabel("Band: ?/?");
  voxel_label_ = new QLabel("Voxels: ?");
  bandwidth_label_ = new QLabel("Bandwidth: ?");
  layout->addWidget(band_label_);
  layout->addWidget(voxel_label_);
  layout->addWidget(bandwidth_label_);

  // --- Level controls (Wavemap LOD) ---
  layout->addWidget(new QLabel("Wavemap Level (Tree LOD):"));
  auto * level_row = new QHBoxLayout();
  btn_level_min_ = new QPushButton("|<");
  btn_level_down_ = new QPushButton("-");
  btn_level_up_ = new QPushButton("+");
  btn_level_max_ = new QPushButton(">|");
  btn_level_min_->setToolTip("Finest (include all levels)");
  btn_level_down_->setToolTip("Include finer level");
  btn_level_up_->setToolTip("Exclude finest level");
  btn_level_max_->setToolTip("Coarsest only");
  level_row->addWidget(btn_level_min_);
  level_row->addWidget(btn_level_down_);
  level_row->addWidget(btn_level_up_);
  level_row->addWidget(btn_level_max_);
  layout->addLayout(level_row);

  level_label_ = new QLabel("Level: ?");
  level_voxel_label_ = new QLabel("Active levels: ?");
  layout->addWidget(level_label_);
  layout->addWidget(level_voxel_label_);

  // Band connections
  connect(btn_band_up_, &QPushButton::clicked, this, &ProgressivePanel::onBandUp);
  connect(btn_band_down_, &QPushButton::clicked, this, &ProgressivePanel::onBandDown);
  connect(btn_band_max_, &QPushButton::clicked, this, &ProgressivePanel::onBandMax);
  connect(btn_band_min_, &QPushButton::clicked, this, &ProgressivePanel::onBandMin);
  // Level connections
  connect(btn_level_up_, &QPushButton::clicked, this, &ProgressivePanel::onLevelUp);
  connect(btn_level_down_, &QPushButton::clicked, this, &ProgressivePanel::onLevelDown);
  connect(btn_level_max_, &QPushButton::clicked, this, &ProgressivePanel::onLevelMax);
  connect(btn_level_min_, &QPushButton::clicked, this, &ProgressivePanel::onLevelMin);
  connect(connect_btn_, &QPushButton::clicked, this, &ProgressivePanel::onConnect);

  setLayout(layout);
}

void ProgressivePanel::onInitialize()
{
  onConnect();
}

void ProgressivePanel::onConnect()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  auto svc_name = service_edit_->text().toStdString();

  // Derive the level service name from the band service name:
  // "/voxelcodec/set_band" -> "/voxelcodec/set_level"
  std::string level_svc = svc_name;
  auto pos = level_svc.rfind("set_band");
  if (pos != std::string::npos) {
    level_svc.replace(pos, 8, "set_level");
  } else {
    level_svc += "_level";
  }

  band_client_ = node->create_client<voxelcodec_msgs::srv::SetBand>(svc_name);
  level_client_ = node->create_client<voxelcodec_msgs::srv::SetLevel>(level_svc);
  band_label_->setText(QString("Connecting to %1...").arg(service_edit_->text()));
  level_label_->setText(QString("Connecting to %1...").arg(QString::fromStdString(level_svc)));

  // Query current state
  callSetBand(0);
  callSetLevel(-1);
}

// --- Band callbacks ---

void ProgressivePanel::onBandUp()
{
  if (current_band_ < max_bands_) {
    callSetBand(current_band_ + 1);
  }
}

void ProgressivePanel::onBandDown()
{
  if (current_band_ > 1) {
    callSetBand(current_band_ - 1);
  }
}

void ProgressivePanel::onBandMax()
{
  callSetBand(0);  // 0 = max
}

void ProgressivePanel::onBandMin()
{
  callSetBand(1);
}

// --- Level callbacks ---

void ProgressivePanel::onLevelUp()
{
  // Increase min_height = exclude finer levels = coarser
  if (current_wvmp_min_height_ < wvmp_height_max_) {
    callSetLevel(current_wvmp_min_height_ + 1);
  }
}

void ProgressivePanel::onLevelDown()
{
  // Decrease min_height = include finer levels
  if (current_wvmp_min_height_ > wvmp_height_min_) {
    callSetLevel(current_wvmp_min_height_ - 1);
  }
}

void ProgressivePanel::onLevelMax()
{
  // Coarsest only
  callSetLevel(wvmp_height_max_);
}

void ProgressivePanel::onLevelMin()
{
  // Include all (finest)
  callSetLevel(-1);
}

void ProgressivePanel::callSetBand(int band)
{
  if (!band_client_ || !band_client_->service_is_ready()) {
    band_label_->setText("Band service not available");
    return;
  }

  auto req = std::make_shared<voxelcodec_msgs::srv::SetBand::Request>();
  req->band = band;

  band_client_->async_send_request(
    req,
    [this](rclcpp::Client<voxelcodec_msgs::srv::SetBand>::SharedFuture future) {
      auto resp = future.get();
      current_band_ = resp->current_band;
      max_bands_ = resp->max_bands;
      QMetaObject::invokeMethod(
        this, [this, resp]() {
          updateBandLabels(
            resp->current_band, resp->max_bands,
            resp->voxel_count, resp->unique_voxels,
            resp->cumulative_bytes, resp->total_bytes);
        }, Qt::QueuedConnection);
    });
}

void ProgressivePanel::callSetLevel(int wvmp_min_height)
{
  if (!level_client_ || !level_client_->service_is_ready()) {
    level_label_->setText("Level service not available");
    return;
  }

  auto req = std::make_shared<voxelcodec_msgs::srv::SetLevel::Request>();
  req->wvmp_min_height = wvmp_min_height;

  level_client_->async_send_request(
    req,
    [this](rclcpp::Client<voxelcodec_msgs::srv::SetLevel>::SharedFuture future) {
      auto resp = future.get();
      current_wvmp_min_height_ = resp->current_wvmp_min_height;
      wvmp_height_max_ = resp->wvmp_height_max;
      wvmp_height_min_ = resp->wvmp_height_min;
      QMetaObject::invokeMethod(
        this, [this, resp]() {
          updateLevelLabels(
            resp->current_wvmp_min_height, resp->wvmp_height_max,
            resp->wvmp_height_min, resp->voxel_count,
            resp->active_levels);
        }, Qt::QueuedConnection);
    });
}

void ProgressivePanel::updateBandLabels(
  int current, int max_bands, int voxels, int unique,
  int64_t cum_bytes, int64_t total_bytes)
{
  band_label_->setText(
    QString("Band: %1 / %2").arg(current).arg(max_bands));
  voxel_label_->setText(
    QString("Voxels: %1 (unique: %2)").arg(voxels).arg(unique));

  auto fmt = [](int64_t b) -> QString {
      if (b < 1024) return QString("%1 B").arg(b);
      if (b < 1024 * 1024) return QString("%1 KB").arg(b / 1024.0, 0, 'f', 1);
      return QString("%1 MB").arg(b / (1024.0 * 1024.0), 0, 'f', 2);
    };
  double pct = total_bytes > 0
    ? 100.0 * cum_bytes / total_bytes : 0.0;
  bandwidth_label_->setText(
    QString("Bandwidth: %1 / %2 (%3%)")
    .arg(fmt(cum_bytes))
    .arg(fmt(total_bytes))
    .arg(pct, 0, 'f', 1));
}

void ProgressivePanel::updateLevelLabels(
  int current_min_height, int height_max, int height_min,
  int voxels, int active_levels)
{
  level_label_->setText(
    QString("Min height: %1 (range: %2..%3)")
    .arg(current_min_height).arg(height_min).arg(height_max));
  level_voxel_label_->setText(
    QString("Active levels: %1, Voxels: %2")
    .arg(active_levels).arg(voxels));
}

}  // namespace voxelcodec_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxelcodec_ros::ProgressivePanel, rviz_common::Panel)
