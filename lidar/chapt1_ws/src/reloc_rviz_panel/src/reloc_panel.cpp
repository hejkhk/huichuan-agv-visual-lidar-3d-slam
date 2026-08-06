#include "reloc_rviz_panel/reloc_panel.hpp"

#include <functional>
#include <string>

#include <QFont>
#include <QMetaObject>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace reloc_rviz_panel
{

RelocPanel::RelocPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);

  status_label_ = new QLabel("Localization: waiting");
  QFont status_font = status_label_->font();
  status_font.setBold(true);
  status_font.setPointSize(11);
  status_label_->setFont(status_font);
  layout->addWidget(status_label_);

  detail_label_ = new QLabel("Keep the vehicle stationary during matching.");
  detail_label_->setWordWrap(true);
  layout->addWidget(detail_label_);

  trigger_button_ = new QPushButton("Relocalize");
  trigger_button_->setMinimumHeight(42);
  trigger_button_->setToolTip(
    "Pause Nav2 and run a new verified global scan-to-map match");
  trigger_button_->setStyleSheet(
    "QPushButton { background: #1976d2; color: white; border: 0; "
    "padding: 8px; font-weight: 600; }"
    "QPushButton:hover { background: #1565c0; }"
    "QPushButton:disabled { background: #777777; }");
  layout->addWidget(trigger_button_);

  node_ = std::make_shared<rclcpp::Node>("rviz_reloc_panel");
  trigger_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/cartographer_reloc/trigger", rclcpp::QoS(1));
  state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/cartographer_reloc/state", rclcpp::QoS(5),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto state = msg->data;
      QMetaObject::invokeMethod(
        this, [this, state]() {updateState(state);}, Qt::QueuedConnection);
    });

  connect(
    trigger_button_, &QPushButton::clicked,
    this, &RelocPanel::requestRelocalization);
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &RelocPanel::spinRos);
  spin_timer_->start(100);
}

void RelocPanel::requestRelocalization()
{
  std_msgs::msg::Bool request;
  request.data = true;
  trigger_pub_->publish(request);
  trigger_button_->setEnabled(false);
  status_label_->setText("Localization: requested");
  detail_label_->setText("Nav2 is pausing. Keep the vehicle stationary.");
}

void RelocPanel::spinRos()
{
  if (node_) {
    rclcpp::spin_some(node_);
  }
}

void RelocPanel::updateState(const std::string & state)
{
  const auto separator = state.find('|');
  const auto phase = state.substr(0, separator);
  status_label_->setText(QString::fromStdString("Localization: " + phase));
  detail_label_->setText(QString::fromStdString(state));

  const bool busy = phase == "searching" || phase == "verifying" ||
    phase == "restarting_trajectory" || phase == "waiting_stop";
  trigger_button_->setEnabled(!busy);
  if (phase == "localized") {
    status_label_->setStyleSheet("color: #2e7d32;");
  } else if (phase == "failed") {
    status_label_->setStyleSheet("color: #c62828;");
  } else {
    status_label_->setStyleSheet("color: #ef6c00;");
  }
}

}  // namespace reloc_rviz_panel

PLUGINLIB_EXPORT_CLASS(reloc_rviz_panel::RelocPanel, rviz_common::Panel)
