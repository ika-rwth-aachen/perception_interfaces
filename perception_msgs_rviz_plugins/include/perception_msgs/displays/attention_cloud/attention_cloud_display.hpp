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

#include <memory>
#include <vector>
#include <chrono>

#include "perception_msgs/msg/object_list.hpp"
#include "perception_msgs_utils/object_access.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <QColor>

#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreManualObject.h>
#include <OgreMeshManager.h>
#include <OgreResourceGroupManager.h>

namespace perception_msgs {
namespace displays {

/**
 * \class AttentionCloudDisplay
 * \brief XAI thinking cloud visualization that appears above vehicle when pedestrians detected
 * 
 * This plugin creates a 3D thinking bubble/cloud above the ego vehicle when pedestrians
 * are detected in front of the vehicle (positive X direction within specified range).
 * Features:
 * - 3D cloud geometry that's always camera-facing (billboard)
 * - White fluffy cloud appearance with smooth transitions
 * - Placeholder area for pedestrian icon (future enhancement)
 * - Only shows when pedestrians detected in front of vehicle
 * - Clean, user-friendly XAI visualization for automated vehicles
 */
class AttentionCloudDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  AttentionCloudDisplay();
  ~AttentionCloudDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;

private slots:
  void updateCloudProperties();

private:
  // Core cloud functionality
  void createThinkingCloud();
  void updateCloudVisibility();
  void destroyCloud();
  
  // Detection logic
  bool hasPedestriansInFront(const perception_msgs::msg::ObjectList& objects);
  bool isInsideDetectionRoi(float x, float y) const;
  
  // Cloud mesh loading
  void loadCloudMesh();
  void updateCloudMaterial();
  
  // Cloud animation
  void animateCloud(float dt);
  void applyCloudTransform();

  // ROI debug visualization
  void createRoiDebugPlane();
  void updateRoiVisualization();
  void destroyRoi();

  // Properties
  rviz_common::properties::FloatProperty* detection_range_property_;
  rviz_common::properties::FloatProperty* position_z_offset_property_;
  rviz_common::properties::FloatProperty* cloud_size_property_;
  rviz_common::properties::FloatProperty* persistence_time_property_;
  rviz_common::properties::FloatProperty* position_x_offset_property_;
  rviz_common::properties::FloatProperty* position_y_offset_property_;
  rviz_common::properties::BoolProperty* force_visible_property_;
  rviz_common::properties::FloatProperty* roi_near_width_property_;
  rviz_common::properties::FloatProperty* roi_far_width_property_;
  rviz_common::properties::BoolProperty* roi_show_property_;
  rviz_common::properties::ColorProperty* roi_color_property_;
  rviz_common::properties::FloatProperty* roi_alpha_property_;

  // Cloud state
  bool has_pedestrians_detected_;
  bool cloud_visible_;
  float current_opacity_;
  float target_opacity_;
  float animation_time_;
  float persistence_timer_; // Seconds since last detection (for logging/UI)
  
  // 3D Objects
  Ogre::SceneNode* cloud_node_;
  Ogre::Entity* cloud_entity_;
  Ogre::MaterialPtr cloud_material_;
  Ogre::Vector3 cloud_anchor_position_;
  Ogre::Quaternion cloud_anchor_orientation_;
  Ogre::SceneNode* roi_node_;
  Ogre::ManualObject* roi_manual_object_;
  Ogre::MaterialPtr roi_material_;
  std::chrono::steady_clock::time_point last_detection_time_;
  bool has_recent_detection_timestamp_;
  
  // Constants
  static constexpr float DEFAULT_DETECTION_RANGE = 40.0f; // 40 meters in front
  static constexpr float DEFAULT_CLOUD_HEIGHT = 4.0f;     // 4 meters above vehicle
  static constexpr float DEFAULT_CLOUD_SIZE = 1.0f;       // 1 meter diameter
  static constexpr float DEFAULT_ROI_NEAR_HALF_WIDTH = 5.0f;   // +/- 5 m close to ego
  static constexpr float DEFAULT_ROI_FAR_HALF_WIDTH = 15.0f;   // +/- 15 m at max range
  static constexpr float ROI_DEBUG_HEIGHT_OFFSET = 0.05f;     // Slight lift to avoid z-fighting
};

}  // namespace displays
}  // namespace perception_msgs
