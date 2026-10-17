// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <chrono>
#include <memory>

#include "perception_msgs/msg/object_list.hpp"
#include "perception_msgs/rendering/traffic_light/traffic_light.hpp"
#include "perception_msgs_utils/object_access.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre {
class ManualObject;
}

namespace rviz_common {
namespace properties {
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace perception_msgs {
namespace displays {

/**
 * \class TrafficLightDisplay
 * \brief Displays a perception_msgs::ObjectList message
 */
class TrafficLightDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

 public:
  TrafficLightDisplay();
  ~TrafficLightDisplay() override;

  void onInitialize() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;
  void onEnable() override;
  void onDisable() override;

 protected:
  void processTypeErasedMessage(std::shared_ptr<const void> msg) override;
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;
  bool validateFloats(perception_msgs::msg::ObjectList::ConstSharedPtr msg);

 protected:

  rviz_common::properties::BoolProperty* enable_type_property_;
  rviz_common::properties::BoolProperty* enable_timeout_property_;
  rviz_common::properties::FloatProperty* timeout_property_;

  std::atomic<bool> is_reset{true};
  std::vector<std::unique_ptr<perception_msgs::rendering::TrafficLight>> viz_object_states_;

  std::shared_ptr<const void> pending_message_;
  std::chrono::steady_clock::time_point last_message_time_;
  bool has_visualization_ = false;
};

}  // namespace displays
}  // namespace perception_msgs
