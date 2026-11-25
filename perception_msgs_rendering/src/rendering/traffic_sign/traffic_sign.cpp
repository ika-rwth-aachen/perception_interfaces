// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include "perception_msgs/rendering/traffic_sign/traffic_sign.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreQuaternion.h>
#include "perception_msgs_utils/object_access.hpp"

namespace perception_msgs {
namespace rendering {

std::string trafficSignTypeName(uint8_t type) { return object_access::taxonomyLabel(type); }

TrafficSign::TrafficSign(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager),
      scene_node_((parent_node ? parent_node : scene_manager->getRootSceneNode())->createChildSceneNode()) {}

TrafficSign::~TrafficSign() {
  if (text_ && text_->isAttached()) text_->getParentSceneNode()->detachObject(text_.get());
  text_.reset();
  face_.reset();
  marker_.reset();
  scene_manager_->destroySceneNode(scene_node_);
}

void TrafficSign::setVisualizeType(const bool& value) { visualize_type_ = value; }

void TrafficSign::setObjectState(const perception_msgs::msg::ObjectState& state) {
  using namespace object_access;
  using Sign = perception_msgs::msg::HEXAMOTION;
  if (state.model_id != Sign::MODEL_ID || !isTrafficSignClass(bestClass(state))) throw std::invalid_argument("Expected a traffic-sign state");
  sanityCheckFullState(state);

  if (text_ && text_->isAttached()) text_->getParentSceneNode()->detachObject(text_.get());
  text_.reset();
  face_.reset();
  marker_.reset();
  if (!hasValidPosition(state)) return;

  const bool oriented = isComponentValid(state, Sign::ROLL) && isComponentValid(state, Sign::PITCH) && isComponentValid(state, Sign::YAW);
  const auto pose = getPose(state);
  Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  Ogre::Quaternion orientation = Ogre::Quaternion::IDENTITY;
  if (oriented) {
    orientation = Ogre::Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    const auto& offset = state.reference_point.translation_to_geometric_center;
    if (!std::isfinite(offset.x) || !std::isfinite(offset.y) || !std::isfinite(offset.z))
      throw std::invalid_argument("Invalid traffic-sign reference point");
    position += orientation * Ogre::Vector3(offset.x, offset.y, offset.z);
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  const bool face_geometry = oriented && isComponentValid(state, Sign::WIDTH) && isComponentValid(state, Sign::HEIGHT) &&
                             getWidth(state) > 0. && getHeight(state) > 0.;
  const Ogre::ColourValue color(1., 1., 1., 1.);
  double label_height = 0.3;
  if (face_geometry) {
    const float half_width = getWidth(state) / 2.;
    const float half_height = getHeight(state) / 2.;
    face_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
    face_->setColor(color.r, color.g, color.b, color.a);
    face_->setLineWidth(0.02);
    face_->setMaxPointsPerLine(5);
    face_->addPoint(Ogre::Vector3(0., -half_width, -half_height));
    face_->addPoint(Ogre::Vector3(0., half_width, -half_height));
    face_->addPoint(Ogre::Vector3(0., half_width, half_height));
    face_->addPoint(Ogre::Vector3(0., -half_width, half_height));
    face_->addPoint(Ogre::Vector3(0., -half_width, -half_height));
    label_height = half_height + 0.2;
  } else {
    marker_ = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
    marker_->setColor(color);
    marker_->setScale(Ogre::Vector3(0.2, 0.2, 0.2));
  }

  if (visualize_type_) {
    text_ = std::make_unique<rviz_rendering::MovableText>(trafficSignTypeName(bestClass(state)), "Liberation Sans", 0.2);
    text_->setColor(color);
    text_->setLocalTranslation(Ogre::Vector3(0., 0., label_height));
    scene_node_->attachObject(text_.get());
  }
}

}  // namespace rendering
}  // namespace perception_msgs
