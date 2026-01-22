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
#include <map>
#include "perception_msgs/msg/object_list.hpp"
#include "perception_msgs_utils/object_access.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre {
class ManualObject;
class SceneNode;
}

namespace rviz_common {
namespace properties {
class ColorProperty;
class FloatProperty;
class StringProperty;
class EnumProperty;
}  // namespace properties
}  // namespace rviz_common

namespace perception_msgs {
namespace displays {

// Aggregated range data for simplified visualization
struct AggregatedRange {
  float angle;          // Direction angle in radians
  float min_distance;   // Minimum distance to objects in this direction
  float field_of_view;  // Angular width of this aggregate
  int object_count;     // Number of objects aggregated
};

/**
 * \class AttentionVectorDisplay
 * \brief Displays attention vectors as red arrows from ego position to detected objects
 * 
 * This plugin visualizes where an AI model is "looking" by drawing either:
 * 1. Individual arrows to each detected object (detailed mode)
 * 2. Aggregated range-based visualization (simplified mode for reduced visual clutter)
 */
class AttentionVectorDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  AttentionVectorDisplay();
  ~AttentionVectorDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updateStyle();

private:
  void updateArrows();
  void createArrow(
    const Ogre::Vector3& start, 
    const Ogre::Vector3& end, 
    const Ogre::ColourValue& color,
    float shaft_radius,
    float head_radius,
    float head_length
  );
  
  void createRangeVisualization(const AggregatedRange& range, const Ogre::ColourValue& color, float length_percentage);
  std::vector<AggregatedRange> aggregateObjectsByDirection(const perception_msgs::msg::ObjectList& msg);
  void clearArrows();

  // Properties
  rviz_common::properties::EnumProperty *visualization_mode_property_;
  rviz_common::properties::ColorProperty *arrow_color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
  rviz_common::properties::FloatProperty *arrow_shaft_radius_property_;
  rviz_common::properties::FloatProperty *arrow_head_radius_property_;  
  rviz_common::properties::FloatProperty *arrow_head_length_property_;
  rviz_common::properties::FloatProperty *arrow_length_percentage_property_;
  rviz_common::properties::FloatProperty *max_range_property_;
  rviz_common::properties::FloatProperty *angular_resolution_property_;
  
  
  rviz_common::properties::StringProperty *ego_frame_property_;

  // Visualization objects
  std::vector<std::unique_ptr<Ogre::ManualObject>> arrow_objects_;
  std::vector<std::unique_ptr<Ogre::SceneNode>> arrow_nodes_;
  
  // Store last message for property updates
  perception_msgs::msg::ObjectList::ConstSharedPtr last_message_;
};

}  // namespace displays
}  // namespace perception_msgs