#include "voxelcodec_ros/ddil_stats_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace voxelcodec_ros
{

DdilStatsPanel::DdilStatsPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * topic_row = new QHBoxLayout;
  topic_edit_ = new QLineEdit;
  topic_edit_->setPlaceholderText("e.g. /ddil_proxy_robot1/stats");
  subscribe_btn_ = new QPushButton("Subscribe");
  scan_btn_ = new QPushButton("Scan");
  scan_btn_->setToolTip("Find active DdilStats topics");
  topic_row->addWidget(topic_edit_);
  topic_row->addWidget(scan_btn_);
  topic_row->addWidget(subscribe_btn_);

  config_label_ = new QLabel("(not subscribed — enter topic and click Subscribe, or Scan)");
  config_label_->setWordWrap(true);

  table_ = new QTableWidget(0, 7);
  table_->setHorizontalHeaderLabels(
    {"Topic", "Rcvd", "Sent", "Drop", "Dedup", "msgs/s", "KB/s"});
  table_->horizontalHeader()->setStretchLastSection(false);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  table_->verticalHeader()->setVisible(false);

  auto * layout = new QVBoxLayout;
  layout->addLayout(topic_row);
  layout->addWidget(config_label_);
  layout->addWidget(table_);
  setLayout(layout);

  connect(subscribe_btn_, &QPushButton::clicked, this, &DdilStatsPanel::onSubscribe);
  connect(scan_btn_, &QPushButton::clicked, this, &DdilStatsPanel::onScan);
}

void DdilStatsPanel::onInitialize()
{
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &DdilStatsPanel::onRefresh);
  refresh_timer_->start(500);
  // Don't auto-subscribe here — load() sets the topic first; user can also click Scan/Subscribe.
}

void DdilStatsPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_edit_->text());
}

void DdilStatsPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic) && !topic.isEmpty()) {
    topic_edit_->setText(topic);
    onSubscribe();
  }
}

void DdilStatsPanel::onScan()
{
  auto node = getDisplayContext()
    ->getRosNodeAbstraction().lock()->get_raw_node();

  const auto topic_map = node->get_topic_names_and_types();
  const std::string target_type = "voxelcodec_msgs/msg/DdilStats";

  QStringList found;
  for (const auto & [topic, types] : topic_map) {
    for (const auto & t : types) {
      if (t == target_type) {
        found << QString::fromStdString(topic);
        break;
      }
    }
  }
  found.sort();

  if (found.isEmpty()) {
    config_label_->setText("Scan: no DdilStats topics found — is ddil_proxy_node running?");
    return;
  }

  // If only one match, subscribe immediately.
  if (found.size() == 1) {
    topic_edit_->setText(found[0]);
    onSubscribe();
    return;
  }

  // Multiple matches: populate the text field with the first and show all in the label.
  topic_edit_->setText(found[0]);
  config_label_->setText(
    QString("Found %1 topics: %2  (edit field to switch)").arg(found.size()).arg(found.join(", ")));
  onSubscribe();
}

void DdilStatsPanel::onSubscribe()
{
  const std::string topic = topic_edit_->text().toStdString();
  if (topic.empty()) {
    config_label_->setText("Enter a topic name first.");
    return;
  }

  stats_sub_.reset();
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    has_data_ = false;
  }
  table_->setRowCount(0);

  auto node = getDisplayContext()
    ->getRosNodeAbstraction().lock()->get_raw_node();

  stats_sub_ = node->create_subscription<voxelcodec_msgs::msg::DdilStats>(
    topic,
    rclcpp::QoS(1).best_effort(),
    [this](const voxelcodec_msgs::msg::DdilStats::ConstSharedPtr & msg) {
      statsCb(msg);
    });

  config_label_->setText(QString("Subscribed to %1").arg(QString::fromStdString(topic)));
}

void DdilStatsPanel::statsCb(const voxelcodec_msgs::msg::DdilStats::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lk(snapshot_mutex_);
  snapshot_ = *msg;
  has_data_ = true;
}

void DdilStatsPanel::onRefresh()
{
  voxelcodec_msgs::msg::DdilStats data;
  bool has_data;
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    data = snapshot_;
    has_data = has_data_;
  }
  if (!has_data) {
    return;
  }

  config_label_->setText(
    QString("BW: %1 kbps  Loss: %2%  Delay: %3 ms  Queue: %4")
      .arg(data.bandwidth_kbps, 0, 'f', 0)
      .arg(data.loss_pct, 0, 'f', 1)
      .arg(data.delay_ms, 0, 'f', 0)
      .arg(data.queue_depth));

  const int n = static_cast<int>(data.topics.size());
  table_->setRowCount(n);

  for (int i = 0; i < n; ++i) {
    const auto & t = data.topics[i];

    // Show last two path components of input_topic to keep rows compact.
    QString full = QString::fromStdString(t.input_topic);
    QStringList parts = full.split('/');
    QString label = parts.size() >= 2
      ? parts[parts.size() - 2] + "/" + parts[parts.size() - 1]
      : full;

    auto set = [&](int col, const QString & text) {
        auto * item = table_->item(i, col);
        if (!item) {
          item = new QTableWidgetItem;
          item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
          table_->setItem(i, col, item);
        }
        item->setText(text);
      };

    auto * topic_item = table_->item(i, 0);
    if (!topic_item) {
      topic_item = new QTableWidgetItem;
      table_->setItem(i, 0, topic_item);
    }
    topic_item->setText(label);

    set(1, QString::number(t.msgs_received));
    set(2, QString::number(t.msgs_sent));
    set(3, QString::number(t.msgs_dropped));
    set(4, QString::number(t.msgs_deduped));
    set(5, QString("%1").arg(static_cast<double>(t.msgs_per_sec), 0, 'f', 1));
    set(6, QString("%1").arg(static_cast<double>(t.kbytes_per_sec), 0, 'f', 1));
  }

  table_->resizeColumnsToContents();
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
}

}  // namespace voxelcodec_ros

PLUGINLIB_EXPORT_CLASS(voxelcodec_ros::DdilStatsPanel, rviz_common::Panel)
