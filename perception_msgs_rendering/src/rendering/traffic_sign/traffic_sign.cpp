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
namespace {

bool measured(const perception_msgs::msg::ObjectState& state, int index) {
  const double variance = object_access::getContinuousStateCovarianceAt(state, index, index);
  return std::isfinite(state.continuous_state[index]) && std::isfinite(variance) && variance >= 0.;
}

std::string typeToText(uint8_t type) {
  using Sign = perception_msgs::msg::TRAFFICSIGN;
  switch (type) {
    case Sign::TYPE_UNKNOWN: return "UNKNOWN";
    case Sign::TYPE_STOP: return "STOP";
    case Sign::TYPE_GIVE_WAY: return "GIVE WAY";
    case Sign::TYPE_PRIORITY_ROAD: return "PRIORITY ROAD";
    case Sign::TYPE_PRIORITY_AT_NEXT_INTERSECTION: return "PRIORITY AT NEXT INTERSECTION";
    case Sign::TYPE_NO_ENTRY: return "NO ENTRY";
    case Sign::TYPE_NO_VEHICLES: return "NO VEHICLES";
    case Sign::TYPE_NO_GOODS_VEHICLES_OVER_3_5_T: return "NO GOODS VEHICLES OVER 3.5 T";
    case Sign::TYPE_NO_OVERTAKING: return "NO OVERTAKING";
    case Sign::TYPE_NO_OVERTAKING_GOODS_VEHICLES: return "NO OVERTAKING GOODS VEHICLES";
    case Sign::TYPE_END_NO_OVERTAKING: return "END NO OVERTAKING";
    case Sign::TYPE_END_NO_OVERTAKING_GOODS_VEHICLES: return "END NO OVERTAKING GOODS VEHICLES";
    case Sign::TYPE_END_ALL_SPEED_AND_OVERTAKING_RESTRICTIONS: return "END ALL SPEED AND OVERTAKING RESTRICTIONS";
    case Sign::TYPE_SPEED_LIMIT_20_KMH: return "SPEED LIMIT 20 KMH";
    case Sign::TYPE_SPEED_LIMIT_30_KMH: return "SPEED LIMIT 30 KMH";
    case Sign::TYPE_SPEED_LIMIT_50_KMH: return "SPEED LIMIT 50 KMH";
    case Sign::TYPE_SPEED_LIMIT_60_KMH: return "SPEED LIMIT 60 KMH";
    case Sign::TYPE_SPEED_LIMIT_70_KMH: return "SPEED LIMIT 70 KMH";
    case Sign::TYPE_SPEED_LIMIT_80_KMH: return "SPEED LIMIT 80 KMH";
    case Sign::TYPE_SPEED_LIMIT_100_KMH: return "SPEED LIMIT 100 KMH";
    case Sign::TYPE_SPEED_LIMIT_120_KMH: return "SPEED LIMIT 120 KMH";
    case Sign::TYPE_END_SPEED_LIMIT_80_KMH: return "END SPEED LIMIT 80 KMH";
    case Sign::TYPE_DANGER: return "DANGER";
    case Sign::TYPE_CURVE_LEFT: return "CURVE LEFT";
    case Sign::TYPE_CURVE_RIGHT: return "CURVE RIGHT";
    case Sign::TYPE_DOUBLE_CURVE_FIRST_LEFT: return "DOUBLE CURVE FIRST LEFT";
    case Sign::TYPE_UNEVEN_ROAD: return "UNEVEN ROAD";
    case Sign::TYPE_SLIPPERY_ROAD: return "SLIPPERY ROAD";
    case Sign::TYPE_ROAD_NARROWS_RIGHT: return "ROAD NARROWS RIGHT";
    case Sign::TYPE_ROAD_WORKS: return "ROAD WORKS";
    case Sign::TYPE_TRAFFIC_SIGNALS: return "TRAFFIC SIGNALS";
    case Sign::TYPE_PEDESTRIANS: return "PEDESTRIANS";
    case Sign::TYPE_CHILDREN: return "CHILDREN";
    case Sign::TYPE_CYCLISTS: return "CYCLISTS";
    case Sign::TYPE_ICE_OR_SNOW: return "ICE OR SNOW";
    case Sign::TYPE_WILD_ANIMALS: return "WILD ANIMALS";
    case Sign::TYPE_TURN_RIGHT: return "TURN RIGHT";
    case Sign::TYPE_TURN_LEFT: return "TURN LEFT";
    case Sign::TYPE_STRAIGHT_AHEAD: return "STRAIGHT AHEAD";
    case Sign::TYPE_STRAIGHT_OR_RIGHT: return "STRAIGHT OR RIGHT";
    case Sign::TYPE_STRAIGHT_OR_LEFT: return "STRAIGHT OR LEFT";
    case Sign::TYPE_KEEP_RIGHT: return "KEEP RIGHT";
    case Sign::TYPE_KEEP_LEFT: return "KEEP LEFT";
    case Sign::TYPE_ROUNDABOUT: return "ROUNDABOUT";
    default: return "Type " + std::to_string(type);
  }
}

}  // namespace

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
  using Sign = perception_msgs::msg::TRAFFICSIGN;
  if (state.model_id != Sign::MODEL_ID) throw std::invalid_argument("Expected a traffic-sign state");
  sanityCheckFullState(state);

  if (text_ && text_->isAttached()) text_->getParentSceneNode()->detachObject(text_.get());
  text_.reset();
  face_.reset();
  marker_.reset();
  if (!hasValidPosition(state)) return;

  const bool oriented = measured(state, Sign::ROLL) && measured(state, Sign::PITCH) && measured(state, Sign::YAW);
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

  const bool face_geometry = oriented && measured(state, Sign::WIDTH) && measured(state, Sign::HEIGHT) &&
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
    text_ = std::make_unique<rviz_rendering::MovableText>(typeToText(getTrafficSignType(state)), "Liberation Sans", 0.2);
    text_->setColor(color);
    text_->setLocalTranslation(Ogre::Vector3(0., 0., label_height));
    scene_node_->attachObject(text_.get());
  }
}

}  // namespace rendering
}  // namespace perception_msgs
