// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include "perception_msgs/displays/traffic_light/traffic_light_display.hpp"

#include <OgreBillboardSet.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <utility>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"

namespace perception_msgs {
namespace displays {

TrafficLightDisplay::TrafficLightDisplay() {

  // plugin properties
  enable_type_property_ = new rviz_common::properties::BoolProperty("Type",
                                                                    false,
                                                                    "Show traffic light type",
                                                                    this);
  enable_timeout_property_ = new rviz_common::properties::BoolProperty("Timeout",
                                                                       true,
                                                                       "Remove traffic lights after timeout if no new ones have been received",
                                                                       this);
  timeout_property_ = new rviz_common::properties::FloatProperty("Duration",
                                                                 1.0,
                                                                 "Timeout duration in seconds (wall time)",
                                                                 enable_timeout_property_);
  timeout_property_->setMin(0.001);
}

TrafficLightDisplay::~TrafficLightDisplay() {
  if (initialized()) {
    viz_object_states_.clear();
  }
}

void TrafficLightDisplay::onInitialize() {
  MFDClass::onInitialize();
  is_reset.store(false);
}

void TrafficLightDisplay::reset() {
  pending_message_.reset();
  has_visualization_ = false;
  MFDClass::reset();
  viz_object_states_.clear();
}

void TrafficLightDisplay::processTypeErasedMessage(std::shared_ptr<const void> msg) {
  if (isEnabled() && !is_reset.load()) {
    pending_message_ = std::move(msg);
  }
}

void TrafficLightDisplay::update(float wall_dt, float ros_dt) {
  MFDClass::update(wall_dt, ros_dt);
  if (pending_message_) {
    auto msg = std::move(pending_message_);
    pending_message_.reset();
    MFDClass::processTypeErasedMessage(std::move(msg));
  }
  if (has_visualization_ && enable_timeout_property_->getBool() &&
      std::chrono::steady_clock::now() - last_message_time_ >=
          std::chrono::duration<float>(timeout_property_->getFloat())) {
    reset();
  }
}

void TrafficLightDisplay::onEnable() {
  MFDClass::onEnable();
  is_reset.store(false);
}

void TrafficLightDisplay::onDisable() {
  is_reset.store(true);
  MFDClass::onDisable();
}

bool TrafficLightDisplay::validateFloats(perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  bool valid = true;
  for (int i = 0; i < int(msg->objects.size()); i++) {
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getX(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getY(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getZ(msg->objects[i]));
  }
  return valid;
}

void TrafficLightDisplay::processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) {

  // check for supported object model id
  for (size_t i = 0; i < msg->objects.size(); ++i) {
    const auto& obj = msg->objects[i];
    if (obj.state.model_id != perception_msgs::msg::TRAFFICLIGHT::MODEL_ID) {
      const std::string error_msg = "Model ID " + std::to_string(obj.state.model_id) +
                                    " of traffic light " + std::to_string(i) + " is not supported";
      this->setStatus(rviz_common::properties::StatusProperty::Error, "Model ID", QString::fromStdString(error_msg));
      return;
    }
  }

  if (is_reset.load()) {
    return;
  }
  if (!validateFloats(msg)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  viz_object_states_.clear();

  if (msg->objects.size()) {
    for (int i = 0; i < int(msg->objects.size()); i++) {
      // render object state
      std::unique_ptr<perception_msgs::rendering::TrafficLight> state_ptr =
          std::make_unique<perception_msgs::rendering::TrafficLight>(scene_manager_, scene_node_);

      state_ptr->setVisualizeType(enable_type_property_->getBool());

      // render
      state_ptr->setObjectState(msg->objects[i].state);
      viz_object_states_.push_back(std::move(state_ptr));
    }
  }

  has_visualization_ = true;
  last_message_time_ = std::chrono::steady_clock::now();
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::TrafficLightDisplay, rviz_common::Display)
