// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include "perception_msgs/displays/traffic_sign/traffic_sign_display.hpp"

#include <chrono>
#include <functional>
#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace perception_msgs {
namespace displays {

TrafficSignDisplay::TrafficSignDisplay() {
  enable_type_property_ = new rviz_common::properties::BoolProperty(
      "Type", false, "Show traffic sign type", this);
  enable_timeout_property_ = new rviz_common::properties::BoolProperty(
      "Timeout", true, "Remove traffic signs after timeout if no new ones have been received", this);
  timeout_property_ = new rviz_common::properties::FloatProperty(
      "Duration", 1.0, "Timeout duration in seconds (wall time)", enable_timeout_property_);
  timeout_property_->setMin(0.001);
}

TrafficSignDisplay::~TrafficSignDisplay() {
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  timeout_timer_.reset();

  if (initialized()) {
    viz_object_states_.clear();
  }
}

void TrafficSignDisplay::onInitialize() {
  MFDClass::onInitialize();
  is_reset.store(false);
}

void TrafficSignDisplay::reset() {
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  MFDClass::reset();
  viz_object_states_.clear();
}

void TrafficSignDisplay::onEnable() {
  MFDClass::onEnable();
  is_reset.store(false);
}

void TrafficSignDisplay::onDisable() {
  is_reset.store(true);
  MFDClass::onDisable();
}

bool TrafficSignDisplay::validateFloats(perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  bool valid = true;
  for (int i = 0; i < int(msg->objects.size()); i++) {
    if (msg->objects[i].state.model_id != perception_msgs::msg::TRAFFICSIGN::MODEL_ID ||
        !perception_msgs::object_access::hasValidPosition(msg->objects[i])) continue;
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getX(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getY(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getZ(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(msg->objects[i].state.reference_point.translation_to_geometric_center);
  }
  return valid;
}

void TrafficSignDisplay::processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) {

  if (is_reset.load()) {
    return;
  }
  try {
    for (const auto& object : msg->objects) {
      if (object.state.model_id == perception_msgs::msg::TRAFFICSIGN::MODEL_ID)
        perception_msgs::object_access::sanityCheckFullState(object);
    }
    if (!validateFloats(msg)) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Invalid traffic sign geometry");
      return;
    }
  } catch (const std::exception& error) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", QString::fromUtf8(error.what()));
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  if (timeout_timer_) {
    timeout_timer_->cancel();
    timeout_timer_.reset();
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  viz_object_states_.clear();

  if (msg->objects.size()) {
    for (int i = 0; i < int(msg->objects.size()); i++) {
      if (msg->objects[i].state.model_id != perception_msgs::msg::TRAFFICSIGN::MODEL_ID ||
        !perception_msgs::object_access::hasValidPosition(msg->objects[i])) continue;
      // render object state
      std::unique_ptr<perception_msgs::rendering::TrafficSign> state_ptr =
          std::make_unique<perception_msgs::rendering::TrafficSign>(scene_manager_, scene_node_);

      state_ptr->setVisualizeType(enable_type_property_->getBool());

      // render
      state_ptr->setObjectState(msg->objects[i].state);
      viz_object_states_.push_back(std::move(state_ptr));
    }
  }

  // reset scene after timeout, if enabled
  if (enable_timeout_property_->getBool()) {
    timeout_timer_ = rviz_ros_node_.lock()->get_raw_node()->create_wall_timer(
      std::chrono::duration<float>(timeout_property_->getFloat()),
      std::bind(&TrafficSignDisplay::timeoutTimerCallback, this)
    );
  }
}

void TrafficSignDisplay::timeoutTimerCallback() {
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  this->reset();
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::TrafficSignDisplay, rviz_common::Display)
