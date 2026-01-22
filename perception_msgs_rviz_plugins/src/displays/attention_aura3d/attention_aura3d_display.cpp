#include <perception_msgs/displays/attention_aura3d/attention_aura3d_display.hpp>

#include <cmath>
#include <algorithm>
#include <sstream>
#include <limits>
#include <chrono>

#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreColourValue.h>
#include <OgreRenderQueue.h>

#include <perception_msgs/msg/object_classification.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>

namespace perception_msgs
{
namespace displays
{

namespace
{
struct ClassOption
{
  const char* label;
  int value;
};

constexpr ClassOption kClassOptions[] = {
  {"All Classes", -1},
  {"Pedestrian", perception_msgs::msg::ObjectClassification::PEDESTRIAN},
  {"Bicycle", perception_msgs::msg::ObjectClassification::BICYCLE},
  {"Motorbike", perception_msgs::msg::ObjectClassification::MOTORBIKE},
  {"Car", perception_msgs::msg::ObjectClassification::CAR},
  {"Truck", perception_msgs::msg::ObjectClassification::TRUCK},
  {"Van", perception_msgs::msg::ObjectClassification::VAN},
  {"Bus", perception_msgs::msg::ObjectClassification::BUS},
  {"Animal", perception_msgs::msg::ObjectClassification::ANIMAL},
  {"Road Obstacle", perception_msgs::msg::ObjectClassification::ROAD_OBSTACLE},
  {"Train", perception_msgs::msg::ObjectClassification::TRAIN},
  {"Trailer", perception_msgs::msg::ObjectClassification::TRAILER},
  {"Car Union", perception_msgs::msg::ObjectClassification::CAR_UNION},
  {"Truck Union", perception_msgs::msg::ObjectClassification::TRUCK_UNION},
  {"Bike Union", perception_msgs::msg::ObjectClassification::BIKE_UNION},
  {"Unknown", perception_msgs::msg::ObjectClassification::UNKNOWN}
};

constexpr float kMinColorDistanceMeters = 6.0f;
constexpr float kMaxColorDistanceMeters = 40.0f;
constexpr float kBaseCautionDistanceMeters = 15.0f;
constexpr float kBaseDangerDistanceMeters = 7.5f;
constexpr float kBaseBlinkDistanceMeters = 4.0f;
constexpr float kBlinkFrequencyHz = 2.5f;

float classificationRiskMultiplier(int class_type)
{
  using perception_msgs::msg::ObjectClassification;
  switch (class_type) {
    case ObjectClassification::PEDESTRIAN:
      return 0.5f;
    case ObjectClassification::BICYCLE:
    case ObjectClassification::BIKE_UNION:
      return 0.65f;
    case ObjectClassification::MOTORBIKE:
      return 0.75f;
    case ObjectClassification::CAR:
    case ObjectClassification::CAR_UNION:
      return 1.0f;
    case ObjectClassification::VAN:
      return 1.05f;
    case ObjectClassification::BUS:
    case ObjectClassification::TRUCK:
    case ObjectClassification::TRUCK_UNION:
    case ObjectClassification::TRAILER:
      return 1.15f;
    case ObjectClassification::TRAIN:
      return 1.2f;
    default:
      return 1.0f;
  }
}

QColor lerpColor(const QColor& from, const QColor& to, float t)
{
  t = std::clamp(t, 0.0f, 1.0f);
  QColor result;
  result.setRedF(from.redF() + (to.redF() - from.redF()) * t);
  result.setGreenF(from.greenF() + (to.greenF() - from.greenF()) * t);
  result.setBlueF(from.blueF() + (to.blueF() - from.blueF()) * t);
  result.setAlphaF(from.alphaF() + (to.alphaF() - from.alphaF()) * t);
  return result;
}
}  // namespace

int AttentionAura3DDisplay::material_counter_ = 0;

AttentionAura3DDisplay::AttentionAura3DDisplay()
: MessageFilterDisplay<perception_msgs::msg::ObjectList>(),
  sectors_(NUM_SECTORS)
{
  // Initialize properties
  radius_property_ = new rviz_common::properties::FloatProperty(
    "Radius", 10.0f,
    "Radius of the flat aura elements from vehicle center",
    this, SLOT(updateAuraProperties()));
  radius_property_->setMin(2.0f);
  radius_property_->setMax(50.0f);

  // Height property removed - elements are always flat on ground

  thickness_property_ = new rviz_common::properties::FloatProperty(
    "Thickness", 10.0f,
    "Radial thickness of the flat trapezoidal elements",
    this, SLOT(updateAuraProperties()));
  thickness_property_->setMin(0.5f);
  thickness_property_->setMax(15.0f);

  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(0, 255, 255),
    "Base color of the aura elements",
    this, SLOT(updateAuraProperties()));

  border_color_property_ = new rviz_common::properties::ColorProperty(
    "Border Color", QColor(255, 255, 255),
    "Color of the element borders",
    this, SLOT(updateAuraProperties()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f,
    "Base transparency of the aura elements",
    this, SLOT(updateAuraProperties()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  frame_property_ = new rviz_common::properties::StringProperty(
    "Reference Frame", "base_link",
    "Reference frame for the aura visualization",
    this, SLOT(updateAuraProperties()));

  class_filter_property_ = new rviz_common::properties::EnumProperty(
    "Focus Class", "All Classes",
    "Only count objects whose highest-probability classification matches this selection.",
    this, SLOT(updateAuraProperties()));
  for (const auto& option : kClassOptions) {
    class_filter_property_->addOption(QString::fromUtf8(option.label), option.value);
  }

  draw_in_background_property_ = new rviz_common::properties::BoolProperty(
    "Draw Behind Objects", false,
    "Render the aura before other scene geometry so only its transparency shines through.",
    this, SLOT(updateAuraProperties()));

  position_x_offset_property_ = new rviz_common::properties::FloatProperty(
    "Position X Offset", 0.0f,
    "Manual X offset for the aura center (meters)",
    this, SLOT(updateAuraProperties()));
  position_x_offset_property_->setMin(-10.0f);
  position_x_offset_property_->setMax(10.0f);

  position_y_offset_property_ = new rviz_common::properties::FloatProperty(
    "Position Y Offset", 0.0f,
    "Manual Y offset for the aura center (meters)",
    this, SLOT(updateAuraProperties()));
  position_y_offset_property_->setMin(-10.0f);
  position_y_offset_property_->setMax(10.0f);

  position_z_offset_property_ = new rviz_common::properties::FloatProperty(
    "Position Z Offset", 0.0f,
    "Manual Z offset for the aura center (meters)",
    this, SLOT(updateAuraProperties()));
  position_z_offset_property_->setMin(-5.0f);
  position_z_offset_property_->setMax(5.0f);

  blink_distance_property_ = new rviz_common::properties::FloatProperty(
    "Blink Threshold", kBaseBlinkDistanceMeters,
    "Blinking activates when the threat-weighted object distance drops below this value (meters)",
    this, SLOT(updateAuraProperties()));
  blink_distance_property_->setMin(0.5f);
  blink_distance_property_->setMax(100.0f);

  // Initialize sectors with base visibility for debugging
  for (auto& sector : sectors_) {
    sector.attention_level = 0.0f;
    sector.object_count = 0;
    sector.target_opacity = 0.2f; // Start dim - will brighten with objects
    sector.current_opacity = 0.2f;
    sector.has_objects = false;
    sector.min_distance = std::numeric_limits<float>::infinity();
    sector.weighted_distance = std::numeric_limits<float>::infinity();
    sector.threat_level = 0.0f;
    sector.is_dangerous = false;
    sector.should_blink = false;
  }
}

AttentionAura3DDisplay::~AttentionAura3DDisplay()
{
  destroyAuraElements();

  delete radius_property_;
  delete thickness_property_;
  delete color_property_;
  delete border_color_property_;
  delete alpha_property_;
  delete frame_property_;
  delete class_filter_property_;
  delete draw_in_background_property_;
  delete position_x_offset_property_;
  delete position_y_offset_property_;
  delete position_z_offset_property_;
  delete blink_distance_property_;
}

void AttentionAura3DDisplay::onInitialize()
{
  MessageFilterDisplay::onInitialize();
  createAuraElements();
}

void AttentionAura3DDisplay::reset()
{
  MessageFilterDisplay::reset();
  destroyAuraElements();
  createAuraElements();
  has_last_pose_ = false;
  animation_time_initialized_ = false;
  blink_time_accumulator_ = 0.0f;
}

void AttentionAura3DDisplay::processMessage(const perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  // Handle frame transforms like attention_vector does
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  last_position_ = position;
  last_orientation_ = orientation;
  has_last_pose_ = true;

  applySceneNodeTransform(position, orientation);

  analyzeSectors(msg);
  updateAura();
}

void AttentionAura3DDisplay::analyzeSectors(const perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  // Reset sectors
  for (auto& sector : sectors_) {
    sector.attention_level = 0.0f;
    sector.object_count = 0;
    sector.has_objects = false;
    sector.min_distance = std::numeric_limits<float>::infinity();
    sector.weighted_distance = std::numeric_limits<float>::infinity();
    sector.threat_level = 0.0f;
    sector.is_dangerous = false;
    sector.should_blink = false;
  }

  const int selected_class = class_filter_property_ ? class_filter_property_->getOptionInt() : -1;

  const float x_offset = position_x_offset_property_ ? position_x_offset_property_->getFloat() : 0.0f;
  const float y_offset = position_y_offset_property_ ? position_y_offset_property_->getFloat() : 0.0f;
  const float z_offset = position_z_offset_property_ ? position_z_offset_property_->getFloat() : 0.0f;

  const float radius = radius_property_ ? radius_property_->getValue().toFloat() : 10.0f;
  const float thickness = thickness_property_ ? thickness_property_->getValue().toFloat() : 10.0f;
  const float outer_radius = std::max(radius + 0.5f * thickness, 1.0f);
  const float color_range = std::clamp(std::max(outer_radius, kBaseCautionDistanceMeters), kMinColorDistanceMeters, kMaxColorDistanceMeters);
  const float base_danger_distance = std::min(kBaseDangerDistanceMeters, color_range * 0.7f);
  float blink_threshold = blink_distance_property_ ? blink_distance_property_->getFloat() : kBaseBlinkDistanceMeters;
  float blink_distance = std::clamp(blink_threshold, 0.1f, color_range);
  const float danger_distance = std::max(base_danger_distance, blink_distance);

  if (msg->objects.empty()) {
    // If no objects, set all sectors to low visibility
    for (auto& sector : sectors_) {
      sector.target_opacity = 0.2f; // Low baseline when no objects
    }
    return;
  }

  // Analyze objects and distribute attention
  float total_attention = 0.0f;
  std::vector<float> sector_attention(NUM_SECTORS, 0.0f);

  for (const auto& object : msg->objects) {
    auto classification = perception_msgs::object_access::getClassWithHighestProbability(object);
    if (selected_class >= 0 && static_cast<int>(classification.type) != selected_class) {
      continue;
    }

    // Calculate angle from ego to object
    float dx = static_cast<float>(perception_msgs::object_access::getX(object)) - x_offset;
    float dy = static_cast<float>(perception_msgs::object_access::getY(object)) - y_offset;
    float dz = static_cast<float>(perception_msgs::object_access::getZ(object)) - z_offset;
    float angle = std::atan2(dy, dx);

    int sector_idx = getSectorIndex(angle);

    // Calculate attention based on distance (closer objects get more attention)
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    float risk_multiplier = classificationRiskMultiplier(static_cast<int>(classification.type));
    float weighted_distance = distance * risk_multiplier;

    float attention_weight = 1.0f / (1.0f + distance * 0.1f); // Inverse distance weighting

    sector_attention[sector_idx] += attention_weight;
    sectors_[sector_idx].object_count++;
    sectors_[sector_idx].has_objects = true;
    sectors_[sector_idx].min_distance = std::min(sectors_[sector_idx].min_distance, distance);
    sectors_[sector_idx].weighted_distance = std::min(sectors_[sector_idx].weighted_distance, weighted_distance);

    total_attention += attention_weight;

    float normalized_threat = 1.0f - std::clamp(weighted_distance / color_range, 0.0f, 1.0f);
    normalized_threat = std::pow(normalized_threat, 0.75f);
    sectors_[sector_idx].threat_level = std::max(sectors_[sector_idx].threat_level, normalized_threat);
    if (weighted_distance <= danger_distance) {
      sectors_[sector_idx].is_dangerous = true;
    }
    if (weighted_distance <= blink_distance) {
      sectors_[sector_idx].should_blink = true;
    }
  }

  // Normalize attention levels and set dynamic target opacities
  if (total_attention > 0.0f) {
    for (int i = 0; i < NUM_SECTORS; ++i) {
      sectors_[i].attention_level = sector_attention[i] / total_attention;

      if (sectors_[i].has_objects) {
        // Sectors with objects: brighten based on attention and threat (0.8 to 1.0+)
        float base_opacity = std::max(0.8f, std::min(1.0f, sectors_[i].attention_level * 3.0f));
        float threat_boost = 0.2f * sectors_[i].threat_level;
        sectors_[i].target_opacity = std::clamp(base_opacity + threat_boost, 0.0f, 1.0f);
      } else {
        // Sectors without objects: dim baseline
        sectors_[i].target_opacity = 0.2f;
      }
    }
  } else {
    // Fallback when no attention calculated
    for (int i = 0; i < NUM_SECTORS; ++i) {
      sectors_[i].target_opacity = sectors_[i].has_objects ? 0.7f : 0.2f;
    }
  }
}

int AttentionAura3DDisplay::getSectorIndex(float angle_rad) const
{
  // Rotate by π/2 (90 degrees) so that sector 0 = North (forward direction)
  // Then rotate 45 degrees clockwise (subtract π/4) to align directions properly
  float adjusted_angle = angle_rad + (M_PI / 2.0f) + (M_PI / 8.0f) - (M_PI / 4.0f);
  
  // Normalize to [0, 2π)
  while (adjusted_angle < 0) adjusted_angle += 2.0f * M_PI;
  while (adjusted_angle >= 2.0f * M_PI) adjusted_angle -= 2.0f * M_PI;
  
  int sector = static_cast<int>(adjusted_angle / SECTOR_ANGLE);
  return std::clamp(sector, 0, NUM_SECTORS - 1);
}

void AttentionAura3DDisplay::createAuraElements()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  destroyAuraElements();

  Ogre::SceneManager* scene_manager = context_->getSceneManager();
  
  for (int i = 0; i < NUM_SECTORS; ++i) {
    // Create scene node
    std::string node_name = "attention_aura3d_node_" + std::to_string(material_counter_) + "_" + std::to_string(i);
    Ogre::SceneNode* parent = scene_node_ ? scene_node_ : scene_manager->getRootSceneNode();
    Ogre::SceneNode* node = parent->createChildSceneNode(node_name);
    sector_nodes_.push_back(node);

    // Create manual object for trapezoidal geometry
    std::string object_name = "attention_aura3d_object_" + std::to_string(material_counter_) + "_" + std::to_string(i);
    Ogre::ManualObject* manual_object = scene_manager->createManualObject(object_name);
    sector_objects_.push_back(manual_object);

    // Create material for this sector with vertex colors enabled
    std::string material_name = "AttentionAura3DMaterial_" + std::to_string(material_counter_++) + "_" + std::to_string(i);
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setDepthCheckEnabled(true);
    pass->setDepthBias(1.0f, 0.0f); // push aura slightly down so closer geometry renders in front
    pass->setLightingEnabled(false);
    pass->setCullingMode(Ogre::CULL_NONE);

    // Enable vertex colors - this is crucial!
    pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    
    // Set base material to white so vertex colors show through properly
    pass->setDiffuse(1.0f, 1.0f, 1.0f, 1.0f);
    pass->setAmbient(0.8f, 0.8f, 0.8f); // Slightly less ambient for better contrast
    
    sector_materials_.push_back(material);

    node->attachObject(manual_object);
  }

  updateRenderSettings();
  updateAura();
}

void AttentionAura3DDisplay::destroyAuraElements()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  Ogre::SceneManager* scene_manager = context_->getSceneManager();

  // Destroy manual objects
  for (auto* obj : sector_objects_) {
    if (obj) {
      try {
        scene_manager->destroyManualObject(obj);
      } catch (...) {
        // Ignore destruction errors
      }
    }
  }
  sector_objects_.clear();

  // Destroy scene nodes
  for (auto* node : sector_nodes_) {
    if (node) {
      try {
        if (node->getParentSceneNode()) {
          node->getParentSceneNode()->removeChild(node);
        }
        scene_manager->destroySceneNode(node);
      } catch (...) {
        // Ignore destruction errors
      }
    }
  }
  sector_nodes_.clear();

  // Clear materials
  sector_materials_.clear();
}

void AttentionAura3DDisplay::createTrapezoidalElement(int sector_index, float /*opacity*/)
{
  if (sector_index >= static_cast<int>(sector_objects_.size()) || !sector_objects_[sector_index]) {
    return;
  }

  Ogre::ManualObject* manual_object = sector_objects_[sector_index];
  manual_object->clear();

  // Calculate color and opacity for this element
  float radius = radius_property_->getValue().toFloat();
  float thickness = thickness_property_->getValue().toFloat();
  
  // Calculate sector angle and rotate 45 degrees clockwise
  float sector_angle = sector_index * SECTOR_ANGLE - (M_PI / 4.0f);
  
  // Get the pre-created material
  if (sector_index >= static_cast<int>(sector_materials_.size()) || !sector_materials_[sector_index]) {
    return; // No valid material
  }
  
  std::string material_name = sector_materials_[sector_index]->getName();
  
  // Calculate color based on current sector state
  QColor safe_color = color_property_->getColor();
  float base_alpha = alpha_property_->getValue().toFloat();

  const DirectionSector3D* sector_state = (sector_index < static_cast<int>(sectors_.size())) ?
    &sectors_[sector_index] : nullptr;

  float threat_level = 0.0f;
  bool should_blink = false;
  bool is_dangerous = false;
  if (sector_state) {
    threat_level = std::clamp(sector_state->threat_level, 0.0f, 1.0f);
    should_blink = sector_state->should_blink;
    is_dangerous = sector_state->is_dangerous;
  }

  QColor caution_color(255, 180, 0); // bright amber
  QColor danger_color(255, 0, 0);
  QColor gradient_color = safe_color;

  if (threat_level > 0.0f) {
    if (threat_level < 0.6f) {
      gradient_color = lerpColor(safe_color, caution_color, threat_level / 0.6f);
    } else {
      gradient_color = lerpColor(caution_color, danger_color, (threat_level - 0.6f) / 0.4f);
    }
  }

  if (is_dangerous) {
    // Shift slightly more towards danger even without blinking
    gradient_color = lerpColor(gradient_color, danger_color, 0.3f * threat_level);
  }

  QColor base_gradient_color = gradient_color;
  bool blink_on = false;
  if (should_blink) {
    float blink_phase = std::fmod(blink_time_accumulator_ * kBlinkFrequencyHz, 1.0f);
    blink_on = blink_phase < 0.5f;
    if (blink_on) {
      gradient_color = danger_color;
    } else {
      gradient_color = base_gradient_color;
    }
  }

  // Get current opacity from sector state
  float current_opacity = sector_state ? sector_state->current_opacity : 0.3f;
  float severity_multiplier = 1.0f + 0.4f * threat_level;
  float boosted_opacity = std::clamp(current_opacity * severity_multiplier, 0.0f, 1.0f);
  if (blink_on) {
    boosted_opacity = std::max(boosted_opacity, 0.85f);
  }

  Ogre::ColourValue vertex_color(
    gradient_color.redF(),
    gradient_color.greenF(),
    gradient_color.blueF(),
    base_alpha * boosted_opacity
  );
  
  // Create flat trapezoidal sector shape (like HUD but on ground)
  manual_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // Use HUD's exact trapezoid geometry
  float inner_radius = radius - thickness * 0.5f;
  float outer_radius = radius + thickness * 0.5f;
  
  // Ensure valid radii  
  if (inner_radius <= 0) inner_radius = 1.0f;
  if (outer_radius <= inner_radius) outer_radius = inner_radius + 1.0f;
  
  // HUD's exact angular widths - narrow at inner edge, wider at outer edge
  float inner_half_angle = M_PI / 16.0f; // ±π/16 at inner edge (narrow)
  float outer_half_angle = M_PI / 8.0f;  // ±π/8 at outer edge (wide)
  
  // Create 4 vertices of the trapezoid (exactly like HUD)
  // Inner edge (narrow end)
  float inner_angle1 = sector_angle - inner_half_angle;
  float inner_angle2 = sector_angle + inner_half_angle;
  // Outer edge (wide end)
  float outer_angle1 = sector_angle - outer_half_angle;
  float outer_angle2 = sector_angle + outer_half_angle;
  
  // Add vertices in correct winding order (counter-clockwise) with colors
  // Vertex 0: Inner left
  manual_object->position(inner_radius * cos(inner_angle1), inner_radius * sin(inner_angle1), 0.1f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(vertex_color);
  
  // Vertex 1: Inner right  
  manual_object->position(inner_radius * cos(inner_angle2), inner_radius * sin(inner_angle2), 0.1f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(vertex_color);
  
  // Vertex 2: Outer right
  manual_object->position(outer_radius * cos(outer_angle2), outer_radius * sin(outer_angle2), 0.1f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(vertex_color);
  
  // Vertex 3: Outer left
  manual_object->position(outer_radius * cos(outer_angle1), outer_radius * sin(outer_angle1), 0.1f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(vertex_color);
  
  // Create two triangles to form the trapezoid (counter-clockwise winding)
  manual_object->triangle(0, 1, 2); // First triangle
  manual_object->triangle(0, 2, 3); // Second triangle

  manual_object->end();

  // Add border rendering
  QColor border_base = border_color_property_->getColor();
  QColor border_color = lerpColor(border_base, gradient_color, 0.6f);
  float border_opacity = base_alpha * std::clamp(boosted_opacity * (0.85f + 0.15f * threat_level), 0.0f, 1.0f);
  Ogre::ColourValue border_vertex_color(
    border_color.redF(),
    border_color.greenF(),
    border_color.blueF(),
    border_opacity
  );

  // Create border as line strips - slightly elevated to avoid z-fighting
  manual_object->begin(material_name, Ogre::RenderOperation::OT_LINE_STRIP);
  
  // Border vertices (same positions but as line strip)
  manual_object->position(inner_radius * cos(inner_angle1), inner_radius * sin(inner_angle1), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);
  
  manual_object->position(inner_radius * cos(inner_angle2), inner_radius * sin(inner_angle2), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);
  
  manual_object->position(outer_radius * cos(outer_angle2), outer_radius * sin(outer_angle2), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);
  
  manual_object->position(outer_radius * cos(outer_angle1), outer_radius * sin(outer_angle1), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);
  
  // Close the border loop
  manual_object->position(inner_radius * cos(inner_angle1), inner_radius * sin(inner_angle1), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);

  manual_object->end();
}
void AttentionAura3DDisplay::updateElementOpacity(int /*sector_index*/, float /*opacity*/)
{
  // Opacity is now handled directly in createTrapezoidalElement via vertex colors
  // This function is kept for interface compatibility
}

void AttentionAura3DDisplay::updateAura()
{
  updateAnimations();
  
  for (int i = 0; i < NUM_SECTORS; ++i) {
    createTrapezoidalElement(i, sectors_[i].current_opacity);
    updateElementOpacity(i, sectors_[i].current_opacity);
  }
}

void AttentionAura3DDisplay::updateAnimations()
{
  const auto now = std::chrono::steady_clock::now();
  float dt = 1.0f / 60.0f;
  if (animation_time_initialized_) {
    dt = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_animation_time_).count();
    dt = std::clamp(dt, 1.0f / 240.0f, 0.1f); // Avoid huge or tiny jumps
  } else {
    animation_time_initialized_ = true;
  }
  last_animation_time_ = now;

  blink_time_accumulator_ += dt;
  if (blink_time_accumulator_ > 1000.0f) {
    blink_time_accumulator_ = std::fmod(blink_time_accumulator_, 1000.0f);
  }

  // Fast animation like HUD (2.0f per second)
  const float animation_speed = 5.0f; // Even faster for more responsive feel

  for (auto& sector : sectors_) {
    float diff = sector.target_opacity - sector.current_opacity;
    float step = animation_speed * dt;
    
    if (std::abs(diff) < step) {
      sector.current_opacity = sector.target_opacity;
    } else {
      sector.current_opacity += (diff > 0 ? step : -step);
    }
    
    // Clamp to valid range
    sector.current_opacity = std::clamp(sector.current_opacity, 0.0f, 1.0f);
  }
}

void AttentionAura3DDisplay::updateAuraProperties()
{
  if (has_last_pose_) {
    applySceneNodeTransform(last_position_, last_orientation_);
  }
  updateRenderSettings();
  updateAura();
}

void AttentionAura3DDisplay::applySceneNodeTransform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  if (!scene_node_) {
    return;
  }

  float x_offset = position_x_offset_property_->getFloat();
  float y_offset = position_y_offset_property_->getFloat();
  float z_offset = position_z_offset_property_->getFloat();

  Ogre::Vector3 offset(x_offset, y_offset, z_offset);
  Ogre::Vector3 world_offset = orientation * offset;

  scene_node_->setPosition(position + world_offset);
  scene_node_->setOrientation(orientation);
}

void AttentionAura3DDisplay::updateRenderSettings()
{
  uint8_t queue = draw_in_background_property_ && draw_in_background_property_->getBool() ?
    Ogre::RENDER_QUEUE_BACKGROUND : Ogre::RENDER_QUEUE_MAIN;

  for (auto* manual_object : sector_objects_) {
    if (manual_object) {
      manual_object->setRenderQueueGroup(queue);
    }
  }
}

} // namespace displays
} // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionAura3DDisplay, rviz_common::Display)
