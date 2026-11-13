// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "perception_msgs/msg/object_list.hpp"
#include "perception_msgs/rendering/traffic_sign/traffic_sign.hpp"
#include "perception_msgs_utils/object_access.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common {
namespace properties {
class BoolProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace perception_msgs {
namespace displays {

/**
 * \class TrafficSignDisplay
 * \brief Displays traffic signs from a perception_msgs::ObjectList message
 */
class TrafficSignDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

 public:
  TrafficSignDisplay();
  ~TrafficSignDisplay() override;

  void onInitialize() override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;

 protected:
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;
  bool validateFloats(perception_msgs::msg::ObjectList::ConstSharedPtr msg);
  void timeoutTimerCallback();

 protected:

  rviz_common::properties::BoolProperty* enable_type_property_;
  rviz_common::properties::BoolProperty* enable_timeout_property_;
  rviz_common::properties::FloatProperty* timeout_property_;

  std::atomic<bool> is_reset{true};
  std::vector<std::unique_ptr<perception_msgs::rendering::TrafficSign>> viz_object_states_;

  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace displays
}  // namespace perception_msgs
