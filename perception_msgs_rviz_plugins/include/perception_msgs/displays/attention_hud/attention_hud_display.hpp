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
#include <array>

#include "perception_msgs/msg/object_list.hpp"
#include "perception_msgs_utils/object_access.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <QColor>
#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QFontMetrics>

#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgrePanelOverlayElement.h>

// Forward declarations for Ogre
namespace Ogre {
class SceneManager;
}

namespace perception_msgs {
namespace displays {

// Structure to hold attention data for each direction
struct DirectionSector {
  float attention_level;    // 0.0 to 1.0 - normalized attention strength
  int object_count;        // Number of objects in this direction
  float target_opacity;    // Target opacity for smooth transitions
  float current_opacity;   // Current opacity for smooth animations
  bool has_objects;        // Whether this direction has any objects
};

/**
 * \\class AttentionHUDDisplay
 * \\brief Large futuristic HUD overlay with trapezoidal directional indicators
 * 
 * This plugin creates a large, abstract overlay that shows where the AI model
 * is focusing its attention using futuristic trapezoidal elements with rounded edges.
 * Features:
 * - Large, abstract design covering significant screen area
 * - 8 trapezoidal elements for directions (N, NE, E, SE, S, SW, W, NW)
 * - Smooth opacity transitions based on object presence
 * - Futuristic styling with rounded corners
 * - No text labels or object counts - pure visual indication
 */
class AttentionHUDDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  AttentionHUDDisplay();
  ~AttentionHUDDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;

private slots:
  void updateHUDProperties();
  void updatePosition();
  void updateSize();
  void updateColors();
  void updateTransparency();
  void updateAnimationSpeed();

private:
  // Core HUD functionality
  void createHUDOverlay();
  void updateHUD();
  
  // Attention analysis
  void analyzeAttention(const perception_msgs::msg::ObjectList& objects);
  int getSectorIndex(float angle_rad) const;
  void updateOpacities(float dt);
  
  // Drawing functions
  void drawTrapezoidalElements(QPainter& painter, int width, int height);
  void drawTrapezoid(QPainter& painter, int center_x, int center_y, 
                     float angle, float opacity, int width, int height);

  // Properties
  rviz_common::properties::IntProperty* hud_size_property_;
  rviz_common::properties::IntProperty* hud_left_property_;
  rviz_common::properties::IntProperty* hud_top_property_;
  rviz_common::properties::ColorProperty* hud_color_property_;
  rviz_common::properties::FloatProperty* hud_alpha_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;
  rviz_common::properties::FloatProperty* max_range_property_;
  rviz_common::properties::FloatProperty* animation_speed_property_;
  rviz_common::properties::StringProperty* ego_frame_property_;

  // HUD state
  std::array<DirectionSector, 8> sector_data_;
  bool update_required_;
  bool first_time_;
  
  // Display properties
  int hud_size_;
  int hud_left_;
  int hud_top_;
  QColor hud_color_;
  float hud_alpha_;
  float bg_alpha_;
  float max_range_;
  float animation_speed_;
  
  // Ogre resources
  Ogre::SceneManager* scene_manager_;
  Ogre::Overlay* overlay_;
  Ogre::PanelOverlayElement* panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
  
  // Constants
  static constexpr int DEFAULT_HUD_SIZE = 800; // Much larger than compass
  static constexpr float SECTOR_ANGLE = M_PI / 4.0f;  // 45 degrees per sector
  static constexpr float DEFAULT_ANIMATION_SPEED = 2.0f; // Opacity change speed
  
  // Mutex for thread safety
  std::mutex hud_mutex_;
};

}  // namespace displays
}  // namespace perception_msgs