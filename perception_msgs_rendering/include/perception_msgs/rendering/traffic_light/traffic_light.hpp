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

#include <string>

#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/visibility_control.hpp"

#include "perception_msgs/msg/object_state.hpp"
#include "perception_msgs_utils/object_access.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace Ogre {
class Any;
class ColourValue;
class Quaternion;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace perception_msgs {

namespace rendering {

class TrafficLight {
 public:
  /**
   * @brief Construct a new Object State object
   * 
   * @param classification_color_map mapping of object classification to Ogre::ColourValue to set different colours with reference to objec classification
   * @param text_color color to visualize the text
   * @param scene_manager 
   * @param parent_node 
   */
  TrafficLight(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = 0);

  ~TrafficLight();

  /**
   * @brief Set the parameters for this ObjectState
   * 
   * @param state the ObjectState to set
   */
  void setObjectState(const perception_msgs::msg::ObjectState& state);

  /**
   * @brief Set the parameters for the Visualization of the type
   * 
   * @param val the bool to set
   */
  void setVisualizeType(const bool& val);

  /**
   * @brief Set type of traffic light
   * 
   * @param state the ObjectState to set
   */
  void setTrafficLightTypeDefault(const perception_msgs::msg::ObjectState& state);

 private:
  /**
   * @brief Set the Object State of an default object
   * 
   * @param state 
   * @param color 
   */
  void setObjectStateVizDefault(const perception_msgs::msg::ObjectState& state);

  /**
   * @brief Generate std::string from type
   * 
   * @param text 
   */
  void typeToText(const perception_msgs::msg::ObjectState& state, std::string& text);

  /**
   * @brief Set the Scene Node Pose of state-object
   * 
   * @param state 
   */
  void setSceneNodePose(const perception_msgs::msg::ObjectState& state);

  Ogre::SceneNode* scene_node_;
  Ogre::SceneManager* scene_manager_;

  std::shared_ptr<rviz_rendering::Shape> sphere_;
  std::shared_ptr<rviz_rendering::MovableText> text_;

  perception_msgs::msg::ObjectState object_state_;
  Ogre::ColourValue state_color_;
  Ogre::ColourValue color;
  bool visualize_type_ = true;
};

}  // namespace rendering

}  // namespace perception_msgs