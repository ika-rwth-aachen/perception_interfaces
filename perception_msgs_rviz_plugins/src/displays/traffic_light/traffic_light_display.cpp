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

#include "perception_msgs/displays/traffic_light/traffic_light_display.hpp"

#include <OgreBillboardSet.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

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
}

TrafficLightDisplay::~TrafficLightDisplay() {
  if (initialized()) viz_object_states_.clear();
  delete enable_type_property_;
  delete enable_timeout_property_;
  delete timeout_property_;
}

void TrafficLightDisplay::onInitialize() {
  MFDClass::onInitialize();
  is_reset.store(false);
}

void TrafficLightDisplay::reset() {
  MFDClass::reset();
  viz_object_states_.clear();
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
  for (const auto& obj : msg->objects) {
    if (obj.state.model_id != perception_msgs::msg::TRAFFICLIGHT::MODEL_ID) {
      std::string error_msg = "Model ID not supported"; // TODO: add model_id value and object number (didn't know how to convert int to string offline)
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
      // Render Object State
      std::unique_ptr<perception_msgs::rendering::TrafficLight> state_ptr =
          std::make_unique<perception_msgs::rendering::TrafficLight>(scene_manager_, scene_node_);

      state_ptr->setVisualizeType(enable_type_property_->getBool());

      // Render
      state_ptr->setObjectState(msg->objects[i].state);
      viz_object_states_.push_back(std::move(state_ptr));
    }
  }

  // reset scene after timeout, if enabled
  if (enable_timeout_property_->getBool()) {
    timeout_timer_ = rviz_ros_node_.lock()->get_raw_node()->create_wall_timer(
      std::chrono::duration<float>(timeout_property_->getFloat()),
      std::bind(&TrafficLightDisplay::timeoutTimerCallback, this)
    );
  }
}

void TrafficLightDisplay::timeoutTimerCallback() {

  timeout_timer_->cancel();
  this->reset();
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::TrafficLightDisplay, rviz_common::Display)
