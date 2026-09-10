#include "voxelcodec_ros/network_stats_panel.hpp"

#include <algorithm>
#include <cmath>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSizePolicy>

namespace voxelcodec_ros
{

namespace
{
constexpr const char * kDdilStatsType = "voxelcodec_msgs/msg/DdilStats";

QString badgeStyle(const QString & bg, const QString & fg)
{
  return QString(
    "QLabel { background-color: %1; color: %2; border-radius: 4px; "
    "padding: 1px 6px; font-weight: bold; }").arg(bg, fg);
}
}  // namespace

NetworkStatsPanel::NetworkStatsPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);

  auto * header = new QLabel("VXCH DDIL Links — Network Stats");
  header->setStyleSheet("font-weight: bold; font-size: 13px;");
  root->addWidget(header);

  // Global bandwidth control. Sits above the per-link cards because changing
  // every link at once is the usual intent -- a run's links are configured
  // identically and what's being explored is the team's radio conditions, not
  // one pair's.
  auto * all_row = new QHBoxLayout();
  all_row->addWidget(new QLabel("Bandwidth, all links:"));
  all_bandwidth_spin_ = new QDoubleSpinBox();
  // 0 means unlimited (ddil_proxy_node's own convention for bandwidth_kbps),
  // so it has to be reachable from the UI too. Upper bound is well past any
  // DDIL-interesting rate -- the point of the control is exploring the
  // constrained end, and "effectively unthrottled" is what 0 is for.
  all_bandwidth_spin_->setRange(0.0, 100000.0);
  all_bandwidth_spin_->setDecimals(0);
  all_bandwidth_spin_->setSingleStep(5.0);
  all_bandwidth_spin_->setValue(50.0);
  all_bandwidth_spin_->setSuffix(" kbps");
  all_bandwidth_spin_->setSpecialValueText("unlimited");
  all_row->addWidget(all_bandwidth_spin_);
  all_bandwidth_apply_ = new QPushButton("Apply to all");
  all_row->addWidget(all_bandwidth_apply_);
  apply_status_ = new QLabel();
  apply_status_->setStyleSheet("color: gray;");
  all_row->addWidget(apply_status_, /*stretch=*/1);
  root->addLayout(all_row);

  connect(all_bandwidth_apply_, &QPushButton::clicked, this, &NetworkStatsPanel::onApplyAll);

  empty_label_ = new QLabel(
    "No active DDIL links found yet. Waiting for ddil_proxy_node "
    "instances to appear (scanning every few seconds)...");
  empty_label_->setWordWrap(true);
  empty_label_->setStyleSheet("color: gray; font-style: italic;");
  root->addWidget(empty_label_);

  auto * scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);

  cards_container_ = new QWidget();
  cards_layout_ = new QVBoxLayout(cards_container_);
  cards_layout_->addStretch();
  scroll->setWidget(cards_container_);

  root->addWidget(scroll, /*stretch=*/1);
  setLayout(root);

  rescan_timer_ = new QTimer(this);
  connect(rescan_timer_, &QTimer::timeout, this, &NetworkStatsPanel::onRescan);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &NetworkStatsPanel::onRefresh);
}

void NetworkStatsPanel::onInitialize()
{
  onRescan();
  rescan_timer_->start(3000);   // pick up newly-launched links every few seconds
  refresh_timer_->start(150);   // ~6-7 Hz UI refresh, smooth enough for progress bars
}

void NetworkStatsPanel::onRescan()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  rclcpp::QoS qos(5);
  qos.best_effort();

  for (const auto & [topic, types] : node->get_topic_names_and_types()) {
    if (subs_.count(topic)) {
      continue;
    }
    const bool is_ddil_stats =
      std::find(types.begin(), types.end(), kDdilStatsType) != types.end();
    if (!is_ddil_stats) {
      continue;
    }

    subs_[topic] = node->create_subscription<voxelcodec_msgs::msg::DdilStats>(
      topic, qos,
      [this, topic](voxelcodec_msgs::msg::DdilStats::ConstSharedPtr msg) {
        statsCb(topic, msg);
      });
  }
}

void NetworkStatsPanel::statsCb(
  const std::string & topic, voxelcodec_msgs::msg::DdilStats::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(snapshots_mutex_);
  snapshots_[topic] = LinkSnapshot{*msg, true};
}

void NetworkStatsPanel::onRefresh()
{
  std::map<std::string, LinkSnapshot> local;
  {
    std::lock_guard<std::mutex> lock(snapshots_mutex_);
    local = snapshots_;
  }

  for (auto & [topic, snapshot] : local) {
    if (!snapshot.dirty) {
      continue;
    }
    if (!cards_.count(topic)) {
      ensureLinkCard(topic, snapshot.msg.link_name);
    }
    updateLinkCard(cards_.at(topic), snapshot.msg);
  }

  empty_label_->setVisible(cards_.empty());
}

void NetworkStatsPanel::ensureLinkCard(const std::string & topic, const std::string & link_name)
{
  auto * box = new QGroupBox(linkLabel(link_name));
  auto * box_layout = new QVBoxLayout(box);

  auto * info_row = new QHBoxLayout();
  auto * capacity_label = new QLabel("Capacity: —");
  auto * throughput_label = new QLabel("Current: —");
  throughput_label->setStyleSheet("font-weight: bold;");
  auto * totals_label = new QLabel("Queued: — · Sent: —");
  info_row->addWidget(capacity_label);
  info_row->addWidget(throughput_label);
  info_row->addWidget(totals_label);
  info_row->addStretch();
  box_layout->addLayout(info_row);

  // Per-link bandwidth override, for the asymmetric cases the global control
  // can't express (e.g. degrading one robot's uplink while the rest stay put).
  auto * bw_row = new QHBoxLayout();
  bw_row->addWidget(new QLabel("Bandwidth:"));
  auto * bandwidth_spin = new QDoubleSpinBox();
  bandwidth_spin->setRange(0.0, 100000.0);
  bandwidth_spin->setDecimals(0);
  bandwidth_spin->setSingleStep(5.0);
  bandwidth_spin->setSuffix(" kbps");
  bandwidth_spin->setSpecialValueText("unlimited");
  bw_row->addWidget(bandwidth_spin);
  auto * bandwidth_apply = new QPushButton("Set");
  bw_row->addWidget(bandwidth_apply);
  auto * bandwidth_note = new QLabel();
  bandwidth_note->setStyleSheet("color: gray; font-style: italic;");
  bw_row->addWidget(bandwidth_note, /*stretch=*/1);
  box_layout->addLayout(bw_row);

  auto * rows_layout = new QVBoxLayout();
  rows_layout->setSpacing(2);
  box_layout->addLayout(rows_layout);

  // Insert before the trailing stretch so new cards stack from the top.
  cards_layout_->insertWidget(cards_layout_->count() - 1, box);

  LinkCard card;
  card.box = box;
  card.capacity_label = capacity_label;
  card.throughput_label = throughput_label;
  card.totals_label = totals_label;
  card.rows_layout = rows_layout;
  card.bandwidth_spin = bandwidth_spin;
  card.bandwidth_apply = bandwidth_apply;
  card.bandwidth_note = bandwidth_note;
  cards_[topic] = card;

  connect(
    bandwidth_apply, &QPushButton::clicked, this,
    [this, topic]() {
      auto it = cards_.find(topic);
      if (it == cards_.end()) {return;}
      applyBandwidth(topic, it->second, it->second.bandwidth_spin->value());
    });
}

void NetworkStatsPanel::updateLinkCard(LinkCard & card, const voxelcodec_msgs::msg::DdilStats & msg)
{
  if (msg.bandwidth_kbps > 0.0) {
    card.capacity_label->setText(
      QString("Capacity: %1 kbps (%2)")
      .arg(msg.bandwidth_kbps, 0, 'f', 0)
      .arg(fmtRate(msg.bandwidth_kbps * 125.0)));
  } else {
    card.capacity_label->setText("Capacity: Unlimited");
  }

  card.throughput_label->setText(QString("Current: %1").arg(fmtRate(msg.send_rate_bps)));

  // Queued now carries its budget, because "4.0 MB queued" alone reads as a
  // busy link when it actually means the backlog is unbounded and everything
  // in it is minutes-to-hours stale. Shed is the honest counterpart: how much
  // map detail the link could not afford and the queue therefore dropped
  // (finest band first) rather than accumulating forever.
  QString totals = QString("Queued: %1").arg(fmtBytes(msg.queued_bytes));
  if (msg.queue_budget_bytes > 0) {
    totals += QString(" / %1 budget").arg(fmtBytes(msg.queue_budget_bytes));
  } else {
    totals += " (unbounded)";
  }
  totals += QString(" · Sent: %1").arg(fmtBytes(msg.sent_bytes));
  if (msg.msgs_shed > 0) {
    totals += QString(" · Shed: %1 (%2)")
      .arg(msg.msgs_shed)
      .arg(fmtBytes(msg.shed_bytes));
  }
  card.totals_label->setText(totals);

  // Seed the spinbox from the link's real configured rate once, then leave it
  // alone -- otherwise a 5 Hz stats stream would fight the user's typing.
  if (!card.bandwidth_seeded) {
    card.bandwidth_seeded = true;
    card.externally_shaped = msg.externally_shaped;
    card.bandwidth_spin->setValue(msg.bandwidth_kbps);
    if (msg.externally_shaped) {
      // tc netem in a network namespace is doing the real shaping; setting
      // bandwidth_kbps here would engage this node's token bucket ON TOP of
      // the kernel's, which is exactly the double-impairment the tc path sets
      // bandwidth_kbps=0 to avoid. Offering a knob that makes things quietly
      // wrong is worse than not offering one.
      card.bandwidth_spin->setEnabled(false);
      card.bandwidth_apply->setEnabled(false);
      card.bandwidth_note->setText("shaped by tc netem — change via tc, not RViz");
    }
  }

  bool added_row = false;
  for (const auto & bs : msg.bands) {
    const int band_index = static_cast<int>(bs.band_index);
    if (!card.band_rows.count(band_index)) {
      BandRow row;
      row.row = new QWidget();
      auto * hl = new QHBoxLayout(row.row);
      hl->setContentsMargins(4, 2, 4, 2);

      row.band_label = new QLabel();
      row.band_label->setMinimumWidth(110);
      row.status_badge = new QLabel();
      row.status_badge->setFixedWidth(80);
      row.status_badge->setAlignment(Qt::AlignCenter);
      row.size_label = new QLabel();
      row.size_label->setMinimumWidth(100);
      row.progress = new QProgressBar();
      row.progress->setRange(0, 100);
      row.progress->setTextVisible(true);
      row.eta_label = new QLabel();
      row.eta_label->setMinimumWidth(70);
      row.eta_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

      hl->addWidget(row.band_label);
      hl->addWidget(row.status_badge);
      hl->addWidget(row.size_label);
      hl->addWidget(row.progress, /*stretch=*/1);
      hl->addWidget(row.eta_label);

      card.band_rows[band_index] = row;
      added_row = true;
    }
    updateBandRow(card.band_rows.at(band_index), bs);
  }

  if (added_row) {
    // Bands rarely appear after startup, so re-laying-out in sorted key
    // order on the (uncommon) occasions a new one shows up is cheap enough.
    while (card.rows_layout->count() > 0) {
      auto * item = card.rows_layout->takeAt(0);
      delete item;
    }
    for (auto & [band_index, row] : card.band_rows) {
      (void)band_index;
      card.rows_layout->addWidget(row.row);
    }
  }
}

void NetworkStatsPanel::updateBandRow(BandRow & row, const voxelcodec_msgs::msg::DdilBandStatus & bs)
{
  const QString band_name = bs.band_index == 0
    ? QString("Band 0 (coarsest)")
    : QString("Band %1").arg(bs.band_index);
  row.band_label->setText(band_name);

  if (bs.active) {
    row.status_badge->setText("Sending");
    row.status_badge->setStyleSheet(badgeStyle("#2e7d32", "white"));
    row.row->setStyleSheet("background-color: rgba(46, 125, 50, 30);");
  } else if (bs.pending_bytes > 0) {
    row.status_badge->setText("Queued");
    row.status_badge->setStyleSheet(badgeStyle("#ef6c00", "white"));
    row.row->setStyleSheet("");
  } else {
    row.status_badge->setText("Idle");
    row.status_badge->setStyleSheet(badgeStyle("#757575", "white"));
    row.row->setStyleSheet("");
  }

  row.size_label->setText(QString("%1 to send").arg(fmtBytes(bs.pending_bytes)));

  if (bs.pending_bytes == 0) {
    row.baseline_pending_bytes = 0;
    row.progress->setValue(100);
  } else {
    if (bs.pending_bytes > row.baseline_pending_bytes) {
      row.baseline_pending_bytes = bs.pending_bytes;
    }
    const double drained = row.baseline_pending_bytes > 0
      ? 1.0 - static_cast<double>(bs.pending_bytes) / static_cast<double>(row.baseline_pending_bytes)
      : 0.0;
    row.progress->setValue(std::clamp(static_cast<int>(std::lround(drained * 100.0)), 0, 100));
  }

  row.eta_label->setText(fmtEta(bs.eta_sec));
}

std::string NetworkStatsPanel::nodeNameFromStatsTopic(const std::string & topic)
{
  // ddil_proxy_node publishes "~/ddil_stats", which resolves to
  // "<fully-qualified node name>/ddil_stats" -- so stripping the suffix yields
  // exactly the name the parameter services live under, namespace included.
  const std::string suffix = "/ddil_stats";
  if (topic.size() > suffix.size() &&
    topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    return topic.substr(0, topic.size() - suffix.size());
  }
  return topic;
}

void NetworkStatsPanel::applyBandwidth(
  const std::string & topic, LinkCard & card, double kbps)
{
  if (card.externally_shaped) {
    return;  // see updateLinkCard -- the knob is disabled for tc-shaped links
  }
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  if (!card.param_client) {
    card.param_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node, nodeNameFromStatsTopic(topic));
  }
  if (!card.param_client->service_is_ready()) {
    // Don't block RViz's UI thread waiting on a service that may never come
    // up (a proxy that has since exited still leaves its topic in the graph
    // for a while). The user can just press Set again.
    card.bandwidth_note->setText("parameter service not ready — try again");
    return;
  }

  // Fire-and-forget: the result shows up as the link's own reported Capacity
  // changing on the next stats message, which is a better confirmation than a
  // dialog would be. The future is discarded deliberately -- blocking on it
  // here would stall the Qt event loop.
  card.param_client->set_parameters({rclcpp::Parameter("bandwidth_kbps", kbps)});
  card.bandwidth_note->setText(
    kbps > 0.0 ? QString("set → %1 kbps").arg(kbps, 0, 'f', 0) : QString("set → unlimited"));
}

void NetworkStatsPanel::onApplyAll()
{
  const double kbps = all_bandwidth_spin_->value();
  int applied = 0;
  int skipped = 0;
  for (auto & [topic, card] : cards_) {
    if (card.externally_shaped) {
      ++skipped;
      continue;
    }
    card.bandwidth_spin->setValue(kbps);
    applyBandwidth(topic, card, kbps);
    ++applied;
  }
  QString status = QString("applied to %1 link(s)").arg(applied);
  if (skipped > 0) {
    status += QString(", %1 tc-shaped skipped").arg(skipped);
  }
  apply_status_->setText(status);
}

QString NetworkStatsPanel::fmtBytes(uint64_t b)
{
  if (b < 1024) {return QString("%1 B").arg(b);}
  if (b < 1024 * 1024) {return QString("%1 KB").arg(b / 1024.0, 0, 'f', 1);}
  return QString("%1 MB").arg(b / (1024.0 * 1024.0), 0, 'f', 2);
}

QString NetworkStatsPanel::fmtRate(double bytes_per_sec)
{
  if (bytes_per_sec <= 0.0) {return "0 B/s";}
  return fmtBytes(static_cast<uint64_t>(bytes_per_sec)) + "/s";
}

QString NetworkStatsPanel::fmtEta(double sec)
{
  if (sec < 0.0) {return "—";}
  if (sec < 0.05) {return "—";}
  if (sec < 60.0) {return QString("%1s").arg(sec, 0, 'f', 1);}
  const int total = static_cast<int>(std::lround(sec));
  const int m = total / 60;
  const int s = total % 60;
  return QString("%1m %2s").arg(m).arg(s, 2, 10, QChar('0'));
}

QString NetworkStatsPanel::linkLabel(const std::string & node_name)
{
  // "ddil_proxy_{to}_from_{from}" -> "{from} → {to}" (falls back to the raw
  // node name if it doesn't match, e.g. a custom node name override).
  const std::string prefix = "ddil_proxy_";
  const std::string sep = "_from_";
  if (node_name.rfind(prefix, 0) != 0) {
    return QString::fromStdString(node_name);
  }
  const auto rest = node_name.substr(prefix.size());
  const auto sep_pos = rest.find(sep);
  if (sep_pos == std::string::npos) {
    return QString::fromStdString(node_name);
  }
  const auto to = rest.substr(0, sep_pos);
  const auto from = rest.substr(sep_pos + sep.size());
  return QString("%1 → %2").arg(QString::fromStdString(from), QString::fromStdString(to));
}

}  // namespace voxelcodec_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxelcodec_ros::NetworkStatsPanel, rviz_common::Panel)
