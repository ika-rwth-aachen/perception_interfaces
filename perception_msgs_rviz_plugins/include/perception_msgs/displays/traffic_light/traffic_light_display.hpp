/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

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
  std::vector<std::unique_ptr<perception_msgs::rendering::TrafficLight>> viz_object_states_;

  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace displays
}  // namespace perception_msgs