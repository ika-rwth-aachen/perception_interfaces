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

// Structure to hold attention data for each sector
struct AttentionSector {
  float attention_level;    // 0.0 to 1.0 - normalized attention strength
  int object_count;        // Number of objects in this sector
  float closest_distance;  // Distance to closest object in sector
  bool has_objects;        // Whether this sector has any objects
};

/**
 * \class AttentionCompassDisplay
 * \brief Futuristic HUD overlay showing AI attention in 8 directional sectors
 * 
 * This plugin creates a transparent cyan HUD overlay that shows where the AI model
 * is focusing its attention in 8 circular sectors around the ego vehicle.
 * Features:
 * - Transparent cyan futuristic styling
 * - 8 directional sectors (N, NE, E, SE, S, SW, W, NW)
 * - Attention intensity visualization
 * - Object count indicators
 * - Configurable appearance and positioning
 */
class AttentionCompassDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  AttentionCompassDisplay();
  ~AttentionCompassDisplay() override;

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

private:
  // Core HUD functionality
  void createHUDOverlay();
  void updateHUD();
  
  // Attention analysis
  void analyzeAttention(const perception_msgs::msg::ObjectList& objects);
  int getSectorIndex(float angle_rad) const;
  
  // Drawing functions
  void drawHUDBackground(QPainter& painter, int width, int height);
  void drawSectorIndicators(QPainter& painter, int width, int height);
  void drawCentralCrosshairs(QPainter& painter, int width, int height);
  void drawAttentionBars(QPainter& painter, int width, int height);
  void drawObjectCounts(QPainter& painter, int width, int height);
  void drawSectorLabels(QPainter& painter, int width, int height);

  // Properties
  rviz_common::properties::IntProperty* hud_size_property_;
  rviz_common::properties::IntProperty* hud_left_property_;
  rviz_common::properties::IntProperty* hud_top_property_;
  rviz_common::properties::ColorProperty* hud_color_property_;
  rviz_common::properties::FloatProperty* hud_alpha_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;
  rviz_common::properties::FloatProperty* max_range_property_;
  rviz_common::properties::BoolProperty* show_labels_property_;
  rviz_common::properties::BoolProperty* show_object_counts_property_;
  rviz_common::properties::StringProperty* ego_frame_property_;

  // HUD state
  std::array<AttentionSector, 8> sector_data_;
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
  bool show_labels_;
  bool show_object_counts_;
  
  // Ogre resources
  Ogre::SceneManager* scene_manager_;
  Ogre::Overlay* overlay_;
  Ogre::PanelOverlayElement* panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
  
  // Constants
  static constexpr int DEFAULT_HUD_SIZE = 300;
  static constexpr float SECTOR_ANGLE = M_PI / 4.0f;  // 45 degrees per sector
  static const char* SECTOR_LABELS[8];
  
  // Mutex for thread safety
  std::mutex hud_mutex_;
};

}  // namespace displays
}  // namespace perception_msgs