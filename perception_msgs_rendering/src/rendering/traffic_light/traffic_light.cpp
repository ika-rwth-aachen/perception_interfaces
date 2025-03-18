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

#include "perception_msgs/rendering/traffic_light/traffic_light.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "perception_msgs/msg/object_classification.hpp"

#include <OgreMovableObject.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>

#include <cmath>

#include "rviz_rendering/objects/shape.hpp"

#include <iomanip>

namespace perception_msgs {

namespace rendering {

TrafficLight::TrafficLight(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  if (!parent_node) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
}

TrafficLight::~TrafficLight() { scene_manager_->destroySceneNode(scene_node_); }

void TrafficLight::setObjectState(const perception_msgs::msg::ObjectState& state) {
  object_state_ = state;

  // Set the color based on the traffic light state
  switch (perception_msgs::object_access::getTrafficLightState(state)) {
    case 0:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = 1.0;
      break;
    case 1:
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;
      break;
    case 2:
      color.r = 1.0;
      color.g = 0.3;
      color.b = 0.0;
      color.a = 1.0;
      break;
    case 3:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.0;
      color.a = 1.0;
      break;
    case 4:
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      color.a = 1.0;
      break;
    case 255:
      color.r = 0.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;
      break;
    default:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = 1.0;
      break;
  }

  setObjectStateVizDefault(state);

  // Set Position of Scene Node
  setSceneNodePose(state);
}  // namespace rendering

void TrafficLight::setObjectStateVizDefault(const perception_msgs::msg::ObjectState& state) {
  Ogre::ColourValue sphere_color = color;
  if (color.a > 1.0) {
    sphere_color.a -= 0.2;
  }
  sphere_ = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
  sphere_->setColor(sphere_color);
  Ogre::Vector3 scale(0.5, 0.5, 0.5);
  sphere_->setScale(scale);

  if (visualize_type_) {
    setTrafficLightTypeDefault(state);
  }
}

void TrafficLight::setSceneNodePose(const perception_msgs::msg::ObjectState& state) {
  // Set position of scene node
  geometry_msgs::msg::Pose gm_pose = perception_msgs::object_access::getPose(state);
  // state.reference_point.translation_to_geometric_center
  // Define transform from our fixed frame to the reference point frame
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.translation.x = gm_pose.position.x;
  tf.transform.translation.y = gm_pose.position.y;
  tf.transform.translation.z = gm_pose.position.z;
  tf.transform.rotation = gm_pose.orientation;
  geometry_msgs::msg::Vector3 translation_map;
  tf2::doTransform(state.reference_point.translation_to_geometric_center, translation_map, tf);
  Ogre::Vector3 position(gm_pose.position.x + translation_map.x, gm_pose.position.y + translation_map.y,
                         gm_pose.position.z + translation_map.z);
  Ogre::Quaternion orientation(gm_pose.orientation.w, gm_pose.orientation.x, gm_pose.orientation.y,
                               gm_pose.orientation.z);
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void TrafficLight::setTrafficLightTypeDefault(const perception_msgs::msg::ObjectState& state) {
  std::string text;

  typeToText(state, text);
  text += "\n";

  if (!text.size()) return;
  text_ = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", 0.3);
  Ogre::Vector3 offs(0.3, 0.3, 1.0);
  // Maybe there is a bug in rviz_rendering::MovableText::setGlobalTranslation
  // Currently only the given y-Position is set
  // https://github.com/ros2/rviz/blob/1ac419472ed06cdd52842a8f964f953a75395245/rviz_rendering/src/rviz_rendering/objects/movable_text.cpp#L520
  // Shows that the global_translation-vector is mutliplied with Ogre::Vector3::UNIT_Y is this intended?
  // In the ROS1 implementation the translation-vector is added without any multiplication
  // See: https://github.com/ros-visualization/rviz/blob/ec7ab1b0183244c05fbd2d0d1b8d8f53d8f42f2b/src/rviz/ogre_helpers/movable_text.cpp#L506
  // I've opened an Issue here: https://github.com/ros2/rviz/issues/974

  text_->setGlobalTranslation(offs);
  scene_node_->attachObject(text_.get());
}

void TrafficLight::typeToText(const perception_msgs::msg::ObjectState& state, std::string& text) {
  switch (perception_msgs::object_access::getTrafficLightType(state)) {
    case 0:
      text += "DEFAULT";
      break;
    case 1:
      text += "STRAIGHT";
      break;
    case 2:
      text += "LEFT";
      break;
    case 3:
      text += "RIGHT";
      break;
    case 255:
      text += "UNKNOWN";
      break;
  }
  return;
}


void TrafficLight::setVisualizeType(const bool& val) { visualize_type_ = val; }

}  // namespace rendering

}  // namespace perception_msgs
