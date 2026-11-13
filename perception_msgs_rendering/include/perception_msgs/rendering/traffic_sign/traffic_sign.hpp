// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>

#include <OgreColourValue.h>
#include "perception_msgs/msg/object_state.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace Ogre {
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace perception_msgs {
namespace rendering {

/** @brief Renders a traffic-sign face or a position marker, with an optional type label. */
class TrafficSign {
 public:
  TrafficSign(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr);
  ~TrafficSign();

  TrafficSign(const TrafficSign&) = delete;
  TrafficSign& operator=(const TrafficSign&) = delete;

  /** @brief Render the measured geometry of a traffic-sign state. */
  void setObjectState(const perception_msgs::msg::ObjectState& state);
  /** @brief Enable or disable the semantic type label. */
  void setVisualizeType(const bool& value);

 private:
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_node_;
  std::unique_ptr<rviz_rendering::BillboardLine> face_;
  std::unique_ptr<rviz_rendering::Shape> marker_;
  std::unique_ptr<rviz_rendering::MovableText> text_;
  bool visualize_type_ = false;
};

}  // namespace rendering
}  // namespace perception_msgs
