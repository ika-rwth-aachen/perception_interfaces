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

#include "perception_msgs/displays/attention_vector/attention_vector_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"

#include <cmath>

namespace perception_msgs {
namespace displays {

// Forward declaration for existing validateFloats function
bool validateFloats(perception_msgs::msg::ObjectList::ConstSharedPtr msg);

AttentionVectorDisplay::AttentionVectorDisplay()
{
  visualization_mode_property_ = new rviz_common::properties::EnumProperty(
    "Visualization Mode", "Individual Arrows",
    "Choose between detailed individual arrows or simplified aggregated ranges.", this, SLOT(updateStyle()));
  visualization_mode_property_->addOption("Arrows", 0);
  visualization_mode_property_->addOption("Ranges", 1);

  arrow_color_property_ = new rviz_common::properties::ColorProperty(
    "Arrow Color", QColor(255, 0, 0),
    "Color of the attention vector arrows.", this, SLOT(updateStyle()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.8f,
    "Transparency of the visualization (0.0 = transparent, 1.0 = opaque).", this, SLOT(updateStyle()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);
    
  arrow_shaft_radius_property_ = new rviz_common::properties::FloatProperty(
    "Arrow Shaft Radius", 0.2f,
    "Radius of the arrow shaft.", this, SLOT(updateStyle()));
  arrow_shaft_radius_property_->setMin(0.01f);
  arrow_shaft_radius_property_->setMax(1.0f);
  
  arrow_head_radius_property_ = new rviz_common::properties::FloatProperty(
    "Arrow Head Radius", 1.0f,
    "Radius of the arrow head.", this, SLOT(updateStyle()));
  arrow_head_radius_property_->setMin(0.01f);
  arrow_head_radius_property_->setMax(5.0f);
  
  arrow_head_length_property_ = new rviz_common::properties::FloatProperty(
    "Arrow Head Length", 2.0f,
    "Length of the arrow head.", this, SLOT(updateStyle()));
  arrow_head_length_property_->setMin(0.01f);
  arrow_head_length_property_->setMax(10.0f);
  
  arrow_length_percentage_property_ = new rviz_common::properties::FloatProperty(
    "Length Percentage", 50.0f,
    "Percentage of full distance to display (0-100%). Works for both Arrows and Ranges modes.", this, SLOT(updateStyle()));
  arrow_length_percentage_property_->setMin(1.0f);
  arrow_length_percentage_property_->setMax(100.0f);

  max_range_property_ = new rviz_common::properties::FloatProperty(
    "Max Range", 30.0f,
    "Maximum range for aggregated visualization (meters).", this, SLOT(updateStyle()));
  max_range_property_->setMin(1.0f);
  max_range_property_->setMax(100.0f);

  angular_resolution_property_ = new rviz_common::properties::FloatProperty(
    "Angular Resolution", 15.0f,
    "Angular resolution for aggregation (degrees).", this, SLOT(updateStyle()));
  angular_resolution_property_->setMin(1.0f);
  angular_resolution_property_->setMax(90.0f);


  ego_frame_property_ = new rviz_common::properties::StringProperty(
    "Ego Frame", "base_link",
    "Frame ID representing the ego vehicle position (origin for arrows).", this, SLOT(updateStyle()));
}

AttentionVectorDisplay::~AttentionVectorDisplay()
{
  clearArrows();
  delete visualization_mode_property_;
  delete arrow_color_property_;
  delete alpha_property_;
  delete arrow_shaft_radius_property_;
  delete arrow_head_radius_property_;
  delete arrow_head_length_property_;
  delete arrow_length_percentage_property_;
  delete max_range_property_;
  delete angular_resolution_property_;
  delete ego_frame_property_;
}

void AttentionVectorDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void AttentionVectorDisplay::reset()
{
  MFDClass::reset();
  clearArrows();
}

void AttentionVectorDisplay::updateStyle()
{
  // Update the display when properties change
  if (last_message_) {
    updateArrows();
  }
}

void AttentionVectorDisplay::processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  // Store the message for property updates
  last_message_ = msg;
  
  if (!validateFloats(msg)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  updateArrows();
}

void AttentionVectorDisplay::updateArrows()
{
  clearArrows();
  
  if (!last_message_) {
    return;
  }

  // Get properties
  int visualization_mode = visualization_mode_property_->getOptionInt();
  Ogre::ColourValue arrow_color = rviz_common::properties::qtToOgre(arrow_color_property_->getColor());
  float alpha = alpha_property_->getFloat();
  arrow_color.a = alpha;

  if (visualization_mode == 0) {
    // Individual arrows mode
    float shaft_radius = arrow_shaft_radius_property_->getFloat();
    float head_radius = arrow_head_radius_property_->getFloat();
    float head_length = arrow_head_length_property_->getFloat();
    float length_percentage = arrow_length_percentage_property_->getFloat() / 100.0f;
    float max_range = max_range_property_->getFloat();

    // Ego position is at the origin of the coordinate frame (0,0,0)
    Ogre::Vector3 ego_pos(0.0f, 0.0f, 0.0f);

    // Create arrows for each object
    for (const auto& object : last_message_->objects) {
      // Get object center position
      Ogre::Vector3 object_pos(
        static_cast<float>(perception_msgs::object_access::getX(object)),
        static_cast<float>(perception_msgs::object_access::getY(object)),
        static_cast<float>(perception_msgs::object_access::getZ(object))
      );

      // Check max range for arrows mode
      float distance = object_pos.length();
      if (distance > max_range) {
        continue; // Skip objects beyond max range
      }

      // Apply length percentage to shorten the arrow
      Ogre::Vector3 direction = object_pos - ego_pos;
      Ogre::Vector3 shortened_end = ego_pos + direction * length_percentage;

      // Create arrow from ego to shortened position
      createArrow(ego_pos, shortened_end, arrow_color, shaft_radius, head_radius, head_length);
    }

    setStatus(rviz_common::properties::StatusProperty::Ok, "Objects", 
      QString("Displaying %1 individual attention vectors").arg(last_message_->objects.size()));
  } else {
    // Aggregated ranges mode
    std::vector<AggregatedRange> ranges = aggregateObjectsByDirection(*last_message_);
    float length_percentage = arrow_length_percentage_property_->getFloat() / 100.0f;
    
    for (const auto& range : ranges) {
      createRangeVisualization(range, arrow_color, length_percentage);
    }

    setStatus(rviz_common::properties::StatusProperty::Ok, "Ranges", 
      QString("Displaying %1 aggregated attention ranges").arg(ranges.size()));
  }
}

void AttentionVectorDisplay::createArrow(
  const Ogre::Vector3& start, 
  const Ogre::Vector3& end, 
  const Ogre::ColourValue& color,
  float shaft_radius,
  float head_radius,
  float head_length)
{
  // Calculate arrow direction and length
  Ogre::Vector3 direction = end - start;
  float total_length = direction.length();
  
  if (total_length < 0.001f) {
    return; // Skip arrows that are too short
  }
  
  direction.normalise();
  
  // Create scene node for this arrow
  auto arrow_node = std::unique_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
  arrow_node->setPosition(start);
  
  // Orient the node to point toward the target
  Ogre::Vector3 default_direction(0, 0, 1); // Default arrow direction
  Ogre::Quaternion rotation = default_direction.getRotationTo(direction);
  arrow_node->setOrientation(rotation);
  
  // Create manual object for the arrow
  auto arrow_object = std::unique_ptr<Ogre::ManualObject>(
    scene_manager_->createManualObject());
  
  // Create a material that supports alpha blending
  static int material_counter = 0;
  std::string material_name = "AttentionArrowMaterial_" + std::to_string(material_counter++);
  
  // Check if material already exists and remove it to avoid conflicts
  if (Ogre::MaterialManager::getSingleton().resourceExists(material_name)) {
    Ogre::MaterialManager::getSingleton().remove(material_name);
  }
  
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setReceiveShadows(false);
  material->getTechnique(0)->setLightingEnabled(false);
  material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
  material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
  material->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
  
  // Enable alpha blending if alpha < 1.0
  if (color.a < 1.0f) {
    material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  }
  
  material->getTechnique(0)->getPass(0)->setAmbient(color.r, color.g, color.b);
  material->getTechnique(0)->getPass(0)->setDiffuse(color.r, color.g, color.b, color.a);
  
  arrow_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  // Calculate shaft and head lengths
  float shaft_length = std::max(0.0f, total_length - head_length);
  if (shaft_length < 0.001f) {
    shaft_length = total_length * 0.7f;
    head_length = total_length * 0.3f;
  }
  
  // Create arrow shaft (cylinder)
  const int segments = 12;
  const float angle_step = 2.0f * M_PI / segments;
  
  // Shaft vertices
  for (int i = 0; i <= segments; ++i) {
    float angle = i * angle_step;
    float x = shaft_radius * cos(angle);
    float y = shaft_radius * sin(angle);
    
    // Bottom circle
    arrow_object->position(x, y, 0.0f);
    arrow_object->colour(color);
    
    // Top circle
    arrow_object->position(x, y, shaft_length);
    arrow_object->colour(color);
  }
  
  // Shaft triangles
  for (int i = 0; i < segments; ++i) {
    int bottom1 = i * 2;
    int top1 = i * 2 + 1;
    int bottom2 = ((i + 1) % segments) * 2;
    int top2 = ((i + 1) % segments) * 2 + 1;
    
    // Two triangles per segment
    arrow_object->triangle(bottom1, bottom2, top1);
    arrow_object->triangle(top1, bottom2, top2);
  }
  
  // Create arrow head (cone)
  int base_vertex_start = (segments + 1) * 2;
  
  // Head base vertices
  for (int i = 0; i <= segments; ++i) {
    float angle = i * angle_step;
    float x = head_radius * cos(angle);
    float y = head_radius * sin(angle);
    
    arrow_object->position(x, y, shaft_length);
    arrow_object->colour(color);
  }
  
  // Head tip vertex
  int tip_vertex = base_vertex_start + segments + 1;
  arrow_object->position(0.0f, 0.0f, total_length);
  arrow_object->colour(color);
  
  // Head triangles
  for (int i = 0; i < segments; ++i) {
    int base1 = base_vertex_start + i;
    int base2 = base_vertex_start + (i + 1) % segments;
    arrow_object->triangle(base1, base2, tip_vertex);
  }
  
  arrow_object->end();
  
  // Attach to scene node
  arrow_node->attachObject(arrow_object.get());
  
  // Store for cleanup
  arrow_objects_.push_back(std::move(arrow_object));
  arrow_nodes_.push_back(std::move(arrow_node));
}

void AttentionVectorDisplay::clearArrows()
{
  // Clean up all arrow objects and nodes
  for (auto& arrow_object : arrow_objects_) {
    scene_manager_->destroyManualObject(arrow_object.release());
  }
  arrow_objects_.clear();
  
  for (auto& arrow_node : arrow_nodes_) {
    scene_node_->removeAndDestroyChild(arrow_node.release());
  }
  arrow_nodes_.clear();
}

std::vector<AggregatedRange> AttentionVectorDisplay::aggregateObjectsByDirection(const perception_msgs::msg::ObjectList& msg)
{
  std::vector<AggregatedRange> ranges;
  std::map<int, std::vector<std::pair<float, float>>> sector_objects; // sector_id -> [(distance, angle)]
  
  float max_range = max_range_property_->getFloat();
  float angular_resolution_rad = angular_resolution_property_->getFloat() * M_PI / 180.0f;
  
  // Group objects by angular sectors
  for (const auto& object : msg.objects) {
    float x = static_cast<float>(perception_msgs::object_access::getX(object));
    float y = static_cast<float>(perception_msgs::object_access::getY(object));
    float z = static_cast<float>(perception_msgs::object_access::getZ(object));
    
    float distance = std::sqrt(x*x + y*y + z*z);
    float angle = std::atan2(y, x);
    
    // Skip objects beyond max range
    if (distance > max_range) {
      continue;
    }
    
    // Determine angular sector
    int sector = static_cast<int>(std::floor((angle + M_PI) / angular_resolution_rad));
    
    sector_objects[sector].emplace_back(distance, angle);
  }
  
  // Create aggregated ranges for each sector with objects
  for (const auto& sector_pair : sector_objects) {
    int sector = sector_pair.first;
    const auto& objects = sector_pair.second;
    
    if (objects.empty()) continue;
    
    AggregatedRange range;
    range.angle = sector * angular_resolution_rad - M_PI + angular_resolution_rad / 2.0f;
    range.field_of_view = angular_resolution_rad;
    range.object_count = objects.size();
    
    // Find minimum distance in this sector
    range.min_distance = max_range;
    for (const auto& obj : objects) {
      range.min_distance = std::min(range.min_distance, obj.first);
    }
    
    ranges.push_back(range);
  }
  
  return ranges;
}

void AttentionVectorDisplay::createRangeVisualization(const AggregatedRange& range, const Ogre::ColourValue& color, float length_percentage)
{
  // Create scene node for this range visualization
  auto range_node = std::unique_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
  range_node->setPosition(Ogre::Vector3::ZERO);
  
  // Create manual object for the range visualization (sector/wedge shape)
  auto range_object = std::unique_ptr<Ogre::ManualObject>(
    scene_manager_->createManualObject());
  
  // Create a material that supports alpha blending
  static int range_material_counter = 0;
  std::string material_name = "AttentionRangeMaterial_" + std::to_string(range_material_counter++);
  
  // Check if material already exists and remove it to avoid conflicts
  if (Ogre::MaterialManager::getSingleton().resourceExists(material_name)) {
    Ogre::MaterialManager::getSingleton().remove(material_name);
  }
  
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setReceiveShadows(false);
  material->getTechnique(0)->setLightingEnabled(false);
  material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
  material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
  material->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
  
  // Enable alpha blending if alpha < 1.0
  if (color.a < 1.0f) {
    material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  }
  
  material->getTechnique(0)->getPass(0)->setAmbient(color.r, color.g, color.b);
  material->getTechnique(0)->getPass(0)->setDiffuse(color.r, color.g, color.b, color.a);
  
  range_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  // Create a sector (wedge) from origin to the minimum distance (with length percentage applied)
  float start_angle = range.angle - range.field_of_view / 2.0f;
  float end_angle = range.angle + range.field_of_view / 2.0f;
  int segments = std::max(3, static_cast<int>(range.field_of_view / (5.0f * M_PI / 180.0f))); // At least 3 segments, more for wider angles
  
  // Apply length percentage to the range distance
  float display_distance = range.min_distance * length_percentage;
  
  // Center vertex at origin
  range_object->position(0.0f, 0.0f, 0.1f); // Slightly above ground
  range_object->colour(color);
  int center_vertex = 0;
  
  // Arc vertices
  for (int i = 0; i <= segments; ++i) {
    float angle = start_angle + (end_angle - start_angle) * i / segments;
    float x = display_distance * std::cos(angle);
    float y = display_distance * std::sin(angle);
    
    range_object->position(x, y, 0.1f);
    range_object->colour(color);
  }
  
  // Create triangles for the sector
  for (int i = 0; i < segments; ++i) {
    int vertex1 = i + 1;
    int vertex2 = i + 2;
    range_object->triangle(center_vertex, vertex1, vertex2);
  }
  
  range_object->end();
  
  // Attach to scene node
  range_node->attachObject(range_object.get());
  
  // Store for cleanup
  arrow_objects_.push_back(std::move(range_object));
  arrow_nodes_.push_back(std::move(range_node));
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionVectorDisplay, rviz_common::Display)