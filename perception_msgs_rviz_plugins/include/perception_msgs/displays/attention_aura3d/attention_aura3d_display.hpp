#ifndef PERCEPTION_MSGS__DISPLAYS__ATTENTION_AURA3D__ATTENTION_AURA3D_DISPLAY_HPP_
#define PERCEPTION_MSGS__DISPLAYS__ATTENTION_AURA3D__ATTENTION_AURA3D_DISPLAY_HPP_

#include <memory>
#include <vector>
#include <string>

#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <chrono>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>

#include <perception_msgs/msg/object_list.hpp>
#include "perception_msgs_utils/object_access.hpp"

namespace perception_msgs
{
namespace displays
{

struct DirectionSector3D {
  float attention_level;    // 0.0 to 1.0 - normalized attention strength
  int object_count;        // Number of objects in this direction
  float target_opacity;    // Target opacity for smooth transitions
  float current_opacity;   // Current opacity for smooth animations
  bool has_objects;        // Whether this direction has any objects
  float min_distance;      // Closest object distance (meters)
  float weighted_distance; // Threat-weighted closest distance
  float threat_level;      // 0.0 (safe) .. 1.0 (critical)
  bool is_dangerous;       // Flag when sector is in dangerous range
  bool should_blink;       // Whether the sector should trigger blinking
};

class AttentionAura3DDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList>
{
  Q_OBJECT

public:
  AttentionAura3DDisplay();
  ~AttentionAura3DDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;

private:
  void processMessage(const perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;
  void updateAura();
  void createAuraElements();
  void destroyAuraElements();
  void updateRenderSettings();
  
  // Helper functions
  int getSectorIndex(float angle_rad) const;
  void analyzeSectors(const perception_msgs::msg::ObjectList::ConstSharedPtr msg);
  void createTrapezoidalElement(int sector_index, float opacity);
  void updateElementOpacity(int sector_index, float opacity);
  
  // Properties
  rviz_common::properties::FloatProperty* radius_property_;
  // Height property removed - elements are always flat
  rviz_common::properties::FloatProperty* thickness_property_;
  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::ColorProperty* border_color_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::StringProperty* frame_property_;
  rviz_common::properties::BoolProperty* draw_in_background_property_;
  rviz_common::properties::EnumProperty* class_filter_property_;
  rviz_common::properties::FloatProperty* position_x_offset_property_;
  rviz_common::properties::FloatProperty* position_y_offset_property_;
  rviz_common::properties::FloatProperty* position_z_offset_property_;
  rviz_common::properties::FloatProperty* blink_distance_property_;
  
  // 3D Objects
  std::vector<Ogre::SceneNode*> sector_nodes_;
  std::vector<Ogre::ManualObject*> sector_objects_;
  std::vector<Ogre::MaterialPtr> sector_materials_;
  
  // Data
  std::vector<DirectionSector3D> sectors_;
  static constexpr int NUM_SECTORS = 8;
  static constexpr float SECTOR_ANGLE = 2.0f * M_PI / NUM_SECTORS;
  
  // Animation
  void updateAnimations();
  static int material_counter_;
  std::chrono::steady_clock::time_point last_animation_time_ {};
  bool animation_time_initialized_ {false};
  float blink_time_accumulator_ {0.0f};

  // Transform helpers
  void applySceneNodeTransform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
  Ogre::Vector3 last_position_ {Ogre::Vector3::ZERO};
  Ogre::Quaternion last_orientation_ {Ogre::Quaternion::IDENTITY};
  bool has_last_pose_ {false};

private Q_SLOTS:
  void updateAuraProperties();
};

} // namespace displays
} // namespace perception_msgs

#endif // PERCEPTION_MSGS__DISPLAYS__ATTENTION_AURA3D__ATTENTION_AURA3D_DISPLAY_HPP_
