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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_rendering/objects/grid.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre {
class SceneNode;
}

namespace perception_msgs {
namespace displays {

class AttentionSwatch;

/**
 * \class AttentionGridDisplay
 * \brief Enhanced occupancy grid display with advanced colormaps
 * 
 * This plugin provides improved visualization of occupancy grids with:
 * - Multiple colormap options (jet, hot, magma, etc.)
 * - Better highlighting of high-attention areas
 */
class AttentionGridDisplay : public rviz_common::MessageFilterDisplay<nav_msgs::msg::OccupancyGrid> {
  Q_OBJECT

public:
  AttentionGridDisplay();
  ~AttentionGridDisplay() override;

  void onInitialize() override;
  void reset() override;
  void onEnable() override;

  void fixedFrameChanged() override;
  void update(float wall_dt, float ros_dt) override;

Q_SIGNALS:
  void mapUpdated();

protected:
  void processMessage(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) override;

private Q_SLOTS:
  void showMap();
  void updateDrawUnder();
  void updatePalette();

private:
  void clear();
  void createSwatches();
  void showValidMap();
  void updateSwatches() const;
  void transformMap();
  void resetSwatchesIfNecessary(size_t width, size_t height, float resolution);
  
  std::vector<unsigned char> createAdvancedPalette(const std::string& colormap_name);
  std::vector<unsigned char> createJetPalette();
  std::vector<unsigned char> createHotPalette();
  std::vector<unsigned char> createMagmaPalette();
  std::vector<unsigned char> createViridsPalette();
  std::vector<unsigned char> createPlasmaPalette();

  // Properties
  rviz_common::properties::EnumProperty *colormap_property_;
  rviz_common::properties::BoolProperty *draw_under_property_;
  
  // Read-only properties
  rviz_common::properties::FloatProperty *resolution_property_;
  rviz_common::properties::IntProperty *width_property_;
  rviz_common::properties::IntProperty *height_property_;
  rviz_common::properties::VectorProperty *position_property_;
  rviz_common::properties::QuaternionProperty *orientation_property_;

  // Map data
  nav_msgs::msg::OccupancyGrid current_map_;
  std::string frame_;
  bool loaded_;
  float resolution_;
  size_t width_;
  size_t height_;

  // Visualization
  std::vector<std::shared_ptr<AttentionSwatch>> swatches_;
  std::vector<Ogre::TexturePtr> palette_textures_;
};

}  // namespace displays
}  // namespace perception_msgs