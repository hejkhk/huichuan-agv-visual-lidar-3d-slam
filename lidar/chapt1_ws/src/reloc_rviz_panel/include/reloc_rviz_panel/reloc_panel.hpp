#ifndef RELOC_RVIZ_PANEL__RELOC_PANEL_HPP_
#define RELOC_RVIZ_PANEL__RELOC_PANEL_HPP_

#include <memory>

#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace reloc_rviz_panel
{

class RelocPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RelocPanel(QWidget * parent = nullptr);
  ~RelocPanel() override = default;

private Q_SLOTS:
  void requestRelocalization();
  void spinRos();

private:
  void updateState(const std::string & state);

  QLabel * status_label_;
  QLabel * detail_label_;
  QPushButton * trigger_button_;
  QTimer * spin_timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
};

}  // namespace reloc_rviz_panel
#endif  // RELOC_RVIZ_PANEL__RELOC_PANEL_HPP_
