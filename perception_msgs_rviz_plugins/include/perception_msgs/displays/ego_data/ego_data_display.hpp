// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

#include "perception_msgs/msg/ego_data.hpp"
#include "perception_msgs/rendering/object_state/object_state.hpp"
#include "perception_msgs_utils/object_access.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/int_property.hpp"

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/material_manager.hpp"

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
 * \class EgoDataDisplay
 * \brief Displays a perception_msgs::EgoData message
 */
class EgoDataDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::EgoData> {
  Q_OBJECT

 public:
  EgoDataDisplay();
  ~EgoDataDisplay() override;

  void onInitialize() override;

  void reset() override;

  void timeoutTimerCallback();

 protected:
  void processMessage(perception_msgs::msg::EgoData::ConstSharedPtr msg) override;

  Ogre::ManualObject *manual_object_ = nullptr;
  std::string trajectory_material_name_ = "EgoTrajectory/ThickLine";

  // properties
  // general
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
  rviz_common::properties::BoolProperty *viz_z_dim_, *viz_bounding_box_, *viz_direction_ind_, *viz_text_,
      *viz_velocity_, *viz_acceleration_;

  // velocity properties
  rviz_common::properties::FloatProperty *velocity_scale_;
  rviz_common::properties::BoolProperty *use_velocity_color_, *velocity_height_;
  rviz_common::properties::ColorProperty *velocity_color_property_;

  // acceleration properties
  rviz_common::properties::FloatProperty *acceleration_scale_;
  rviz_common::properties::BoolProperty *use_acceleration_color_;
  rviz_common::properties::ColorProperty *acceleration_color_property_;

  // text properties
  rviz_common::properties::FloatProperty *char_height_, *text_offset_;
  rviz_common::properties::BoolProperty *print_vel_;

  // timeout
  rviz_common::properties::BoolProperty* enable_timeout_property_;
  rviz_common::properties::FloatProperty* timeout_property_;

  // trajectory properties
  rviz_common::properties::Property *color_options_, *parameter_options_;
  rviz_common::properties::BoolProperty *viz_trajectory_;
  rviz_common::properties::ColorProperty *color_property_base_, *color_negative_dynamics_, *color_positive_dynamics_;
  rviz_common::properties::FloatProperty *trajectory_alpha_property_, *v_max_property_, *a_max_property_;
  rviz_common::properties::EnumProperty *drop_down_;
  // trajectory ending customization
  rviz_common::properties::EnumProperty *trajectory_end_cap_;
  rviz_common::properties::BoolProperty *trajectory_fade_out_;
  rviz_common::properties::FloatProperty *trajectory_fade_length_;
  rviz_common::properties::IntProperty *trajectory_round_segments_;

  std::unordered_map<unsigned int, Ogre::ColourValue> classification_color_map_;
  std::shared_ptr<perception_msgs::rendering::ObjectState> viz_ego_state_;

  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // parameters
  float v_max_ = 50.0;
  float a_max_ = 5.0;
  std::string default_ = "Static Color";
  std::string option_vel_ = "Velocity-dependent Coloring";
  std::string option_accel_ = "Acceleration-dependent Coloring";
};

}  // namespace displays
}  // namespace perception_msgs
