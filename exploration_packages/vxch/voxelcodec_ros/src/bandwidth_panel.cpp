#include "voxelcodec_ros/bandwidth_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace voxelcodec_ros
{

BandwidthPanel::BandwidthPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);

  // --- Topic inputs ---
  auto * topic_box = new QGroupBox("Topics");
  auto * topic_grid = new QGridLayout();
  topic_grid->addWidget(new QLabel("Baseline:"), 0, 0);
  native_topic_edit_ = new QLineEdit("/vxch/voxelized/points");
  topic_grid->addWidget(native_topic_edit_, 0, 1);
  topic_grid->addWidget(new QLabel("VXCH:"), 1, 0);
  vxch_topic_edit_ = new QLineEdit("/vxch/progressive/points");
  topic_grid->addWidget(vxch_topic_edit_, 1, 1);
  subscribe_btn_ = new QPushButton("Subscribe");
  topic_grid->addWidget(subscribe_btn_, 2, 0, 1, 2);
  topic_box->setLayout(topic_grid);
  root->addWidget(topic_box);

  // --- Native stats ---
  auto * native_box = new QGroupBox("Voxelized baseline");
  auto * ng = new QGridLayout();
  native_count_label_ = new QLabel("Msgs: 0");
  native_last_label_  = new QLabel("Last: —");
  native_total_label_ = new QLabel("Total: 0 B");
  native_rate_label_  = new QLabel("Rate: —");
  ng->addWidget(native_count_label_, 0, 0);
  ng->addWidget(native_last_label_,  0, 1);
  ng->addWidget(native_total_label_, 1, 0);
  ng->addWidget(native_rate_label_,  1, 1);
  native_box->setLayout(ng);
  root->addWidget(native_box);

  // --- VXCH stats ---
  auto * vxch_box = new QGroupBox("VXCH progressive");
  auto * vg = new QGridLayout();
  vxch_count_label_ = new QLabel("Msgs: 0");
  vxch_last_label_  = new QLabel("Last: —");
  vxch_total_label_ = new QLabel("Total: 0 B");
  vxch_rate_label_  = new QLabel("Rate: —");
  vg->addWidget(vxch_count_label_, 0, 0);
  vg->addWidget(vxch_last_label_,  0, 1);
  vg->addWidget(vxch_total_label_, 1, 0);
  vg->addWidget(vxch_rate_label_,  1, 1);
  vxch_box->setLayout(vg);
  root->addWidget(vxch_box);

  // --- Ratio ---
  ratio_label_ = new QLabel("Compression: —");
  ratio_label_->setStyleSheet("font-weight: bold; font-size: 14px;");
  root->addWidget(ratio_label_);

  root->addStretch();

  connect(subscribe_btn_, &QPushButton::clicked, this, &BandwidthPanel::onSubscribe);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &BandwidthPanel::onRefresh);

  setLayout(root);
}

void BandwidthPanel::onInitialize()
{
  // Auto-subscribe on panel open
  onSubscribe();
  refresh_timer_->start(500);  // 2 Hz refresh
}

void BandwidthPanel::onSubscribe()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Reset stats
  {
    std::lock_guard<std::mutex> lk(native_stats_.mu);
    native_stats_.msg_count = 0;
    native_stats_.total_bytes = 0;
    native_stats_.last_msg_bytes = 0;
    native_stats_.first_time = 0.0;
    native_stats_.last_time = 0.0;
  }
  {
    std::lock_guard<std::mutex> lk(vxch_stats_.mu);
    vxch_stats_.msg_count = 0;
    vxch_stats_.total_bytes = 0;
    vxch_stats_.last_msg_bytes = 0;
    vxch_stats_.first_time = 0.0;
    vxch_stats_.last_time = 0.0;
  }

  // Experimental fairness: force identical subscriber QoS for both streams.
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();

  auto native_topic = native_topic_edit_->text().toStdString();
  auto vxch_topic = vxch_topic_edit_->text().toStdString();

  native_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    native_topic, qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      nativeCb(msg);
    });

  vxch_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    vxch_topic, qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      vxchCb(msg);
    });
}

void BandwidthPanel::nativeCb(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lk(native_stats_.mu);
  // Use local receive time so both topics are measured in the same clock domain.
  double now = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->now().seconds();
  if (native_stats_.msg_count == 0) {
    native_stats_.first_time = now;
  }
  native_stats_.last_time = now;
  native_stats_.msg_count++;
  native_stats_.last_msg_bytes = msg->data.size();
  native_stats_.total_bytes += msg->data.size();
}

void BandwidthPanel::vxchCb(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lk(vxch_stats_.mu);
  // Use local receive time so both topics are measured in the same clock domain.
  double now = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->now().seconds();
  if (vxch_stats_.msg_count == 0) {
    vxch_stats_.first_time = now;
  }
  vxch_stats_.last_time = now;
  vxch_stats_.msg_count++;
  vxch_stats_.last_msg_bytes = msg->data.size();
  vxch_stats_.total_bytes += msg->data.size();
}

void BandwidthPanel::onRefresh()
{
  uint64_t nt, nc, nlast, vt, vc, vlast;
  double n_dur, v_dur;
  {
    std::lock_guard<std::mutex> lk(native_stats_.mu);
    nt = native_stats_.total_bytes;
    nc = native_stats_.msg_count;
    nlast = native_stats_.last_msg_bytes;
    n_dur = native_stats_.last_time - native_stats_.first_time;
  }
  {
    std::lock_guard<std::mutex> lk(vxch_stats_.mu);
    vt = vxch_stats_.total_bytes;
    vc = vxch_stats_.msg_count;
    vlast = vxch_stats_.last_msg_bytes;
    v_dur = vxch_stats_.last_time - vxch_stats_.first_time;
  }

  native_count_label_->setText(QString("Msgs: %1").arg(nc));
  native_last_label_->setText(QString("Last: %1").arg(fmtBytes(nlast)));
  native_total_label_->setText(QString("Total: %1").arg(fmtBytes(nt)));
  if (nc > 1 && n_dur > 0) {
    native_rate_label_->setText(
      QString("Rate: %1 Hz  %2/s")
      .arg(nc / n_dur, 0, 'f', 1)
      .arg(fmtBytes(static_cast<uint64_t>(nt / n_dur))));
  }

  vxch_count_label_->setText(QString("Msgs: %1").arg(vc));
  vxch_last_label_->setText(QString("Last: %1").arg(fmtBytes(vlast)));
  vxch_total_label_->setText(QString("Total: %1").arg(fmtBytes(vt)));
  if (vc > 1 && v_dur > 0) {
    vxch_rate_label_->setText(
      QString("Rate: %1 Hz  %2/s")
      .arg(vc / v_dur, 0, 'f', 1)
      .arg(fmtBytes(static_cast<uint64_t>(vt / v_dur))));
  }

  // Compression ratio
  if (nt > 0 && vt > 0) {
    double ratio = static_cast<double>(vt) / static_cast<double>(nt);
    ratio_label_->setText(
      QString("Compression: %1x  (%2%  of baseline)")
      .arg(1.0 / ratio, 0, 'f', 1)
      .arg(ratio * 100.0, 0, 'f', 1));
  } else {
    ratio_label_->setText("Compression: —");
  }
}

QString BandwidthPanel::fmtBytes(uint64_t b)
{
  if (b < 1024) return QString("%1 B").arg(b);
  if (b < 1024 * 1024) return QString("%1 KB").arg(b / 1024.0, 0, 'f', 1);
  return QString("%1 MB").arg(b / (1024.0 * 1024.0), 0, 'f', 2);
}

}  // namespace voxelcodec_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxelcodec_ros::BandwidthPanel, rviz_common::Panel)
