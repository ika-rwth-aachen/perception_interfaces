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

#include <atomic>
#include "perception_msgs/msg/object_list.hpp"
#include "perception_msgs/rendering/object_state/object_state.hpp"
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
class BoolProperty;
class EnumProperty;
class IntProperty;
}  // namespace properties
}  // namespace rviz_common

namespace perception_msgs {
namespace displays {

/**
 * \class ObjectListDisplay
 * \brief Displays a perception_msgs::ObjectList message
 */
class ObjectListDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  ObjectListDisplay();
  ~ObjectListDisplay() override;

  void onInitialize() override;

  void reset() override;

  void onEnable() override;
  void onDisable() override;

  void timeoutTimerCallback();

protected:
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;

  // properties
  // general
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;

  // object appearance properties
  rviz_common::properties::Property *appearance_properties_;
  rviz_common::properties::BoolProperty *viz_bounding_box_, *viz_direction_ind_, *viz_mesh_;
  // hoverboard appearance
  rviz_common::properties::BoolProperty *viz_hoverboard_;
  rviz_common::properties::FloatProperty *hoverboard_thickness_;
  rviz_common::properties::FloatProperty *hoverboard_corner_radius_;
  rviz_common::properties::EnumProperty *hoverboard_cap_style_;
  rviz_common::properties::IntProperty *hoverboard_corner_segments_;
  rviz_common::properties::BoolProperty *hoverboard_glow_;
  rviz_common::properties::FloatProperty *hoverboard_glow_height_;
  rviz_common::properties::FloatProperty *hoverboard_glow_intensity_;

  // Uncertainty visualization
  rviz_common::properties::BoolProperty *viz_xy_uncertainty_;
  rviz_common::properties::ColorProperty *xy_uncertainty_color_;
  rviz_common::properties::FloatProperty *xy_uncertainty_alpha_;
  rviz_common::properties::FloatProperty *xy_uncertainty_scale_;
  rviz_common::properties::IntProperty *xy_uncertainty_segments_;
  rviz_common::properties::BoolProperty *viz_yaw_uncertainty_;
  rviz_common::properties::ColorProperty *yaw_uncertainty_color_;
  rviz_common::properties::FloatProperty *yaw_uncertainty_alpha_;
  rviz_common::properties::FloatProperty *yaw_uncertainty_cone_length_;

  // Safety margin visualization
  rviz_common::properties::BoolProperty *viz_safety_margins_;
  rviz_common::properties::ColorProperty *safety_margin_color_2sigma_;
  rviz_common::properties::ColorProperty *safety_margin_color_3sigma_;
  rviz_common::properties::FloatProperty *safety_margin_alpha_;

  // Color by Classification (applies to multiple modes)
  rviz_common::properties::BoolProperty *color_property_group_;
  rviz_common::properties::ColorProperty *color_property_pedestrian_;
  rviz_common::properties::ColorProperty *color_property_bicycle_;
  rviz_common::properties::ColorProperty *color_property_motorcycle_;
  rviz_common::properties::ColorProperty *color_property_car_;
  rviz_common::properties::ColorProperty *color_property_utility_;
  rviz_common::properties::ColorProperty *color_property_bus_;
  rviz_common::properties::ColorProperty *color_property_animal_;
  rviz_common::properties::ColorProperty *color_property_vru_;
  rviz_common::properties::ColorProperty *color_property_micro_;
  rviz_common::properties::ColorProperty *color_property_unknown_;

  // velocity properties
  rviz_common::properties::BoolProperty *viz_velocity_;
  rviz_common::properties::FloatProperty *velocity_scale_;
  rviz_common::properties::BoolProperty *use_velocity_color_;
  rviz_common::properties::ColorProperty *velocity_color_property_;

  // acceleration properties
  rviz_common::properties::BoolProperty *viz_acceleration_;
  rviz_common::properties::FloatProperty *acceleration_scale_;
  rviz_common::properties::BoolProperty *use_acceleration_color_;
  rviz_common::properties::ColorProperty *acceleration_color_property_;

  // text properties
  rviz_common::properties::BoolProperty *viz_text_;
  rviz_common::properties::FloatProperty *char_height_;
  rviz_common::properties::BoolProperty *use_text_color_class_;
  rviz_common::properties::BoolProperty *print_id_;
  rviz_common::properties::BoolProperty *print_exist_prob_;
  rviz_common::properties::BoolProperty *print_class_;
  rviz_common::properties::BoolProperty *print_vel_;

  // prediction properties
  rviz_common::properties::BoolProperty *viz_predictions_;
  rviz_common::properties::BoolProperty *viz_prediction_points_;
  rviz_common::properties::ColorProperty *color_property_prediction_line_;
  rviz_common::properties::ColorProperty *color_property_prediction_points_;
  rviz_common::properties::FloatProperty *width_property_prediction_;
  rviz_common::properties::FloatProperty *width_property_prediction_points_;
  rviz_common::properties::BoolProperty *viz_prediction_probabilities_;
  rviz_common::properties::FloatProperty *char_height_prediction_probs_;

  // timeout
  rviz_common::properties::BoolProperty* enable_timeout_property_;
  rviz_common::properties::FloatProperty* timeout_property_;

  std::unordered_map<unsigned int, Ogre::ColourValue> classification_color_map_;
  std::vector<std::unique_ptr<perception_msgs::rendering::ObjectState>> viz_object_states_;
  std::atomic<bool> is_reset{true};

  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace displays
}  // namespace perception_msgs
