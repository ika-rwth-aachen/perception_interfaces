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

#include "perception_msgs/displays/attention_cloud/attention_cloud_display.hpp"

#include <Ogre.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/mesh_loader.hpp"

#include <chrono>

#include <cmath>
#include <algorithm>
#include <iostream>
#include <string>

namespace perception_msgs {
namespace displays {

AttentionCloudDisplay::AttentionCloudDisplay()
: MessageFilterDisplay<perception_msgs::msg::ObjectList>(),
  has_pedestrians_detected_(false),
  cloud_visible_(false),
  current_opacity_(0.0f),
  target_opacity_(0.0f),
  animation_time_(0.0f),
  persistence_timer_(0.0f),
  cloud_node_(nullptr),
  cloud_entity_(nullptr),
  cloud_anchor_position_(Ogre::Vector3::ZERO),
  cloud_anchor_orientation_(Ogre::Quaternion::IDENTITY),
  roi_node_(nullptr),
  roi_manual_object_(nullptr),
  has_recent_detection_timestamp_(false)
{
  // Initialize properties
  detection_range_property_ = new rviz_common::properties::FloatProperty(
    "Detection Range", DEFAULT_DETECTION_RANGE,
    "Range in front of vehicle to detect pedestrians (meters)",
    this, SLOT(updateCloudProperties()));
  detection_range_property_->setMin(5.0f);
  detection_range_property_->setMax(60.0f);

  cloud_size_property_ = new rviz_common::properties::FloatProperty(
    "Cloud Size", DEFAULT_CLOUD_SIZE,
    "Diameter of the thinking cloud (meters)",
    this, SLOT(updateCloudProperties()));
  cloud_size_property_->setMin(0.5f);
  cloud_size_property_->setMax(5.0f);

  persistence_time_property_ = new rviz_common::properties::FloatProperty(
    "Persistence Time", 3.0f,
    "Time cloud stays visible after pedestrians disappear (seconds)",
    this, SLOT(updateCloudProperties()));
  persistence_time_property_->setMin(0.5f);
  persistence_time_property_->setMax(10.0f);

  position_x_offset_property_ = new rviz_common::properties::FloatProperty(
    "Position X Offset", 0.0f,
    "Manual X position offset for cloud positioning (meters)",
    this, SLOT(updateCloudProperties()));
  position_x_offset_property_->setMin(-5.0f);
  position_x_offset_property_->setMax(5.0f);

  position_y_offset_property_ = new rviz_common::properties::FloatProperty(
    "Position Y Offset", 0.0f,
    "Manual Y position offset for cloud positioning (meters)",
    this, SLOT(updateCloudProperties()));
  position_y_offset_property_->setMin(-5.0f);
  position_y_offset_property_->setMax(5.0f);

  position_z_offset_property_ = new rviz_common::properties::FloatProperty(
    "Position Z Offset", DEFAULT_CLOUD_HEIGHT,
    "Vertical offset of the thinking cloud above vehicle (meters)",
    this, SLOT(updateCloudProperties()));
  position_z_offset_property_->setMin(1.0f);
  position_z_offset_property_->setMax(10.0f);

  force_visible_property_ = new rviz_common::properties::BoolProperty(
    "Force Cloud Visible", false,
    "Keep the thinking cloud visible regardless of detections (debugging)",
    this, SLOT(updateCloudProperties()));

  roi_near_width_property_ = new rviz_common::properties::FloatProperty(
    "ROI Near Width", DEFAULT_ROI_NEAR_HALF_WIDTH,
    "Half-width (per side) of detection trapezoid at the ego vehicle (meters)",
    this, SLOT(updateCloudProperties()));
  roi_near_width_property_->setMin(0.0f);
  roi_near_width_property_->setMax(20.0f);

  roi_far_width_property_ = new rviz_common::properties::FloatProperty(
    "ROI Far Width", DEFAULT_ROI_FAR_HALF_WIDTH,
    "Half-width (per side) of detection trapezoid at the far end (meters)",
    this, SLOT(updateCloudProperties()));
  roi_far_width_property_->setMin(0.0f);
  roi_far_width_property_->setMax(40.0f);

  roi_show_property_ = new rviz_common::properties::BoolProperty(
    "ROI Show", false,
    "Display the pedestrian detection trapezoid as a transparent plane for debugging",
    this, SLOT(updateCloudProperties()));

  roi_color_property_ = new rviz_common::properties::ColorProperty(
    "ROI Color", QColor(255, 153, 0),
    "Color used for the ROI debug plane",
    this, SLOT(updateCloudProperties()));

  roi_alpha_property_ = new rviz_common::properties::FloatProperty(
    "ROI Alpha", 0.9f,
    "Transparency of the ROI debug plane",
    this, SLOT(updateCloudProperties()));
  roi_alpha_property_->setMin(0.0f);
  roi_alpha_property_->setMax(1.0f);
}

AttentionCloudDisplay::~AttentionCloudDisplay()
{
  destroyCloud();
  
  delete detection_range_property_;
  delete position_z_offset_property_;
  delete cloud_size_property_;
  delete persistence_time_property_;
  delete position_x_offset_property_;
  delete position_y_offset_property_;
  delete force_visible_property_;
  delete roi_near_width_property_;
  delete roi_far_width_property_;
  delete roi_show_property_;
  delete roi_color_property_;
  delete roi_alpha_property_;
}

void AttentionCloudDisplay::onInitialize()
{
  MessageFilterDisplay::onInitialize();
  createThinkingCloud();
  createRoiDebugPlane();
}

void AttentionCloudDisplay::onEnable()
{
  MessageFilterDisplay::onEnable();
  updateCloudVisibility();
  if (roi_node_) {
    roi_node_->setVisible(roi_show_property_->getBool());
  }
}

void AttentionCloudDisplay::onDisable()
{
  if (cloud_node_) {
    cloud_node_->setVisible(false);
  }
  if (roi_node_) {
    roi_node_->setVisible(false);
  }
  MessageFilterDisplay::onDisable();
}

void AttentionCloudDisplay::update(float wall_dt, float /*ros_dt*/)
{
  animateCloud(wall_dt);
}

void AttentionCloudDisplay::processMessage(const perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  // Handle frame transforms like other plugins
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  cloud_anchor_position_ = position;
  cloud_anchor_orientation_ = orientation;
  applyCloudTransform();

  if (roi_node_) {
    roi_node_->setPosition(position);
    roi_node_->setOrientation(orientation);
  }
  
  bool pedestrians_detected = hasPedestriansInFront(*msg);
  bool force_visible = force_visible_property_->getBool();

  if (pedestrians_detected) {
    has_pedestrians_detected_ = true;
    target_opacity_ = 1.0f;
    persistence_timer_ = 0.0f;
    last_detection_time_ = std::chrono::steady_clock::now();
    has_recent_detection_timestamp_ = true;
  } else if (!force_visible) {
    has_pedestrians_detected_ = false;
  }

  if (force_visible) {
    target_opacity_ = 1.0f;
  }

  if (!pedestrians_detected && has_recent_detection_timestamp_) {
    persistence_timer_ = std::chrono::duration<float>(
      std::chrono::steady_clock::now() - last_detection_time_).count();
  } else if (!pedestrians_detected && !has_recent_detection_timestamp_) {
    float fallback_timer = persistence_time_property_->getValue().toFloat();
    if (fallback_timer < 0.0f) {
      fallback_timer = 0.0f;
    }
    persistence_timer_ = fallback_timer;
  }

  std::cout << "[AttentionCloud] pedestrians_detected=" << (pedestrians_detected ? "true" : "false")
            << ", force_visible=" << (force_visible ? "true" : "false")
            << ", persistence_timer=" << persistence_timer_
            << ", target_opacity=" << target_opacity_
            << std::endl;
}

bool AttentionCloudDisplay::hasPedestriansInFront(const perception_msgs::msg::ObjectList& objects)
{
  for (const auto& object : objects.objects) {
    // Get the classification with highest probability
    auto classification = perception_msgs::object_access::getClassWithHighestProbability(object);
    
    // Check if object is a pedestrian (PEDESTRIAN = 1 from ObjectClassification.msg)
    if (classification.type == perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
      // Get position using utility functions
      float x = perception_msgs::object_access::getX(object);
      float y = perception_msgs::object_access::getY(object);
      
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }

      if (isInsideDetectionRoi(x, y)) {
        return true;
      }
    }
  }
  return false;
}

bool AttentionCloudDisplay::isInsideDetectionRoi(float x, float y) const
{
  float roi_length = detection_range_property_->getValue().toFloat();
  float near_half_width = roi_near_width_property_->getValue().toFloat();
  float far_half_width = roi_far_width_property_->getValue().toFloat();

  roi_length = std::max(0.1f, roi_length);
  near_half_width = std::max(0.0f, near_half_width);
  far_half_width = std::max(near_half_width, far_half_width);

  if (x < 0.0f || x > roi_length) {
    return false;
  }

  float interpolation = x / roi_length;
  float half_width = near_half_width + (far_half_width - near_half_width) * interpolation;
  return std::abs(y) <= half_width;
}

void AttentionCloudDisplay::createThinkingCloud()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  Ogre::SceneManager* scene_manager = context_->getSceneManager();
  
  // Create main cloud scene node
  cloud_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();
  
  // Load the cloud mesh
  loadCloudMesh();
  
  // Initialize visibility according to the debug override property
  bool force_visible = force_visible_property_->getBool();
  current_opacity_ = force_visible ? 1.0f : 0.0f;
  target_opacity_ = current_opacity_;

  // Position cloud above vehicle
  updateCloudProperties();

  if (cloud_node_) {
    cloud_node_->setVisible(force_visible);
    cloud_visible_ = force_visible;
  }
}

void AttentionCloudDisplay::loadCloudMesh()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  Ogre::SceneManager* scene_manager = context_->getSceneManager();
  
  try {
    std::string package = "package://perception_msgs_rviz_plugins/meshes/bubble.obj";
    Ogre::MeshPtr mesh = rviz_rendering::loadMeshFromResource(package);  
    
    if (mesh) {
      std::string mesh_name = "cloud_mesh_" + std::to_string(reinterpret_cast<uintptr_t>(this));
      cloud_entity_ = scene_manager->createEntity(mesh_name, mesh);

      cloud_node_->attachObject(cloud_entity_);
      cloud_node_->setVisible(false);

      updateCloudMaterial();
      std::cout << "[AttentionCloud] Cloud mesh loaded. Sub-entities: "
                << cloud_entity_->getNumSubEntities() << std::endl;
      
      RCLCPP_INFO(rclcpp::get_logger("AttentionCloudDisplay"), "Successfully loaded cloud mesh using its default material.");
      
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AttentionCloudDisplay"), "Failed to load cloud mesh: mesh is null");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("AttentionCloudDisplay"), "Failed to load cloud mesh: %s", e.what());
  }
}


void AttentionCloudDisplay::updateCloudProperties()
{
  if (!cloud_node_ || !cloud_entity_) {
    return;
  }
  
  float cloud_size = cloud_size_property_->getValue().toFloat();
  
  // Set mesh scale based on cloud size
  cloud_node_->setScale(cloud_size, cloud_size, cloud_size);

  updateRoiVisualization();
  applyCloudTransform();

  if (force_visible_property_ && force_visible_property_->getBool()) {
    current_opacity_ = 1.0f;
    target_opacity_ = 1.0f;
    persistence_timer_ = 0.0f;
    if (cloud_node_) {
      cloud_node_->setVisible(true);
      cloud_visible_ = true;
    }
  }

  updateCloudVisibility();
}


void AttentionCloudDisplay::animateCloud(float dt)
{
  animation_time_ += dt;
  bool force_visible = force_visible_property_->getBool();
  float persistence_time = persistence_time_property_->getValue().toFloat();
  if (persistence_time < 0.0f) {
    persistence_time = 0.0f;
  }

  const auto now = std::chrono::steady_clock::now();

  if (has_pedestrians_detected_) {
    persistence_timer_ = 0.0f;
    if (!has_recent_detection_timestamp_) {
      last_detection_time_ = now;
      has_recent_detection_timestamp_ = true;
    }
  }

  bool persistence_active = false;
  if (has_recent_detection_timestamp_) {
    float elapsed = std::chrono::duration<float>(now - last_detection_time_).count();
    if (!has_pedestrians_detected_) {
      persistence_timer_ = elapsed;
    }
    persistence_active = elapsed <= persistence_time;
    if (!has_pedestrians_detected_ && !persistence_active) {
      has_recent_detection_timestamp_ = false;
    }
  } else if (!has_pedestrians_detected_) {
    persistence_timer_ = persistence_time;
  }

  float desired_opacity = 0.0f;
  if (force_visible || has_pedestrians_detected_ || persistence_active) {
    desired_opacity = 1.0f;
  }

  target_opacity_ = desired_opacity;

  float fade_in_speed = 2.0f;
  float fade_out_speed = 0.33f;
  if (std::abs(current_opacity_ - target_opacity_) > 0.01f) {
    if (current_opacity_ < target_opacity_) {
      current_opacity_ = std::min(target_opacity_, current_opacity_ + fade_in_speed * dt);
    } else {
      current_opacity_ = std::max(target_opacity_, current_opacity_ - fade_out_speed * dt);
    }
  }

  bool should_be_visible = (current_opacity_ > 0.01f) || force_visible;
  bool visible_state = should_be_visible && isEnabled();
  if (cloud_visible_ != visible_state) {
    cloud_visible_ = visible_state;
    if (cloud_node_) {
      cloud_node_->setVisible(cloud_visible_);
    }
  }

  applyCloudTransform();
}

void AttentionCloudDisplay::applyCloudTransform()
{
  if (!cloud_node_) {
    return;
  }

  float base_height = position_z_offset_property_->getValue().toFloat();
  float x_offset = position_x_offset_property_->getValue().toFloat();
  float y_offset = position_y_offset_property_->getValue().toFloat();

  Ogre::Vector3 local_offset(x_offset, y_offset, base_height);
  Ogre::Vector3 world_offset = cloud_anchor_orientation_ * local_offset;
  cloud_node_->setPosition(cloud_anchor_position_ + world_offset);
  cloud_node_->setOrientation(cloud_anchor_orientation_);
}

void AttentionCloudDisplay::updateCloudVisibility()
{
  if (cloud_node_) {
    bool force_visible = force_visible_property_ && force_visible_property_->getBool();
    bool show = isEnabled() && (cloud_visible_ || force_visible);
    cloud_node_->setVisible(show);
  }
}

void AttentionCloudDisplay::destroyCloud()
{
  if (cloud_entity_) {
    if (cloud_node_) {
      cloud_node_->detachObject(cloud_entity_);
    }
    if (context_ && context_->getSceneManager()) {
      context_->getSceneManager()->destroyEntity(cloud_entity_);
    }
    cloud_entity_ = nullptr;
  }
  
  if (cloud_node_) {
    if (context_ && context_->getSceneManager()) {
      context_->getSceneManager()->destroySceneNode(cloud_node_);
    }
    cloud_node_ = nullptr;
  }
  
  if (cloud_material_) {
    cloud_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(cloud_material_->getName());
    cloud_material_.reset();
  }

  destroyRoi();
}

void AttentionCloudDisplay::updateCloudMaterial()
{
  // Keep the original mesh materials untouched during debugging so textures and
  // shading appear exactly as authored. We rely on toggling node visibility
  // rather than per-material alpha, therefore no cloned material is required.
  cloud_material_.reset();
}

void AttentionCloudDisplay::createRoiDebugPlane()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  Ogre::SceneManager* scene_manager = context_->getSceneManager();

  roi_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();

  std::string manual_name = "attention_cloud_roi_" + std::to_string(reinterpret_cast<uintptr_t>(this));
  roi_manual_object_ = scene_manager->createManualObject(manual_name);
  roi_manual_object_->setDynamic(true);
  roi_node_->attachObject(roi_manual_object_);

  std::string material_name = "attention_cloud_roi_material_" + std::to_string(reinterpret_cast<uintptr_t>(this));
  roi_material_ = Ogre::MaterialManager::getSingleton().create(
    material_name,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  roi_material_->setReceiveShadows(false);
  Ogre::Technique* technique = roi_material_->getTechnique(0);
  Ogre::Pass* pass = technique->getPass(0);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthWriteEnabled(false);
  pass->setLightingEnabled(false);
  pass->setCullingMode(Ogre::CULL_NONE);

  updateRoiVisualization();
  roi_node_->setVisible(roi_show_property_->getBool());
}

void AttentionCloudDisplay::updateRoiVisualization()
{
  if (!roi_manual_object_ || !roi_material_) {
    return;
  }

  bool show_roi = roi_show_property_->getBool();
  if (roi_node_) {
    roi_node_->setVisible(show_roi);
  }

  roi_manual_object_->clear();

  if (!show_roi) {
    return;
  }

  float roi_length = detection_range_property_->getValue().toFloat();
  float near_half_width = roi_near_width_property_->getValue().toFloat();
  float far_half_width = roi_far_width_property_->getValue().toFloat();
  roi_length = std::max(0.1f, roi_length);
  near_half_width = std::max(0.0f, near_half_width);
  far_half_width = std::max(near_half_width, far_half_width);

  QColor roi_color = roi_color_property_->getColor();
  float roi_alpha = roi_alpha_property_->getValue().toFloat();
  roi_alpha = std::max(0.0f, std::min(1.0f, roi_alpha));

  Ogre::Pass* pass = roi_material_->getTechnique(0)->getPass(0);
  pass->setDiffuse(roi_color.redF(), roi_color.greenF(), roi_color.blueF(), roi_alpha);
  pass->setAmbient(roi_color.redF() * 0.5f, roi_color.greenF() * 0.5f, roi_color.blueF() * 0.5f);

  Ogre::ColourValue colour(roi_color.redF(), roi_color.greenF(), roi_color.blueF(), roi_alpha);

  roi_manual_object_->begin(roi_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  Ogre::Vector3 near_left(0.0f, -near_half_width, ROI_DEBUG_HEIGHT_OFFSET);
  Ogre::Vector3 near_right(0.0f, near_half_width, ROI_DEBUG_HEIGHT_OFFSET);
  Ogre::Vector3 far_right(roi_length, far_half_width, ROI_DEBUG_HEIGHT_OFFSET);
  Ogre::Vector3 far_left(roi_length, -far_half_width, ROI_DEBUG_HEIGHT_OFFSET);

  roi_manual_object_->position(near_left);
  roi_manual_object_->normal(0, 0, 1);
  roi_manual_object_->colour(colour);

  roi_manual_object_->position(near_right);
  roi_manual_object_->normal(0, 0, 1);
  roi_manual_object_->colour(colour);

  roi_manual_object_->position(far_right);
  roi_manual_object_->normal(0, 0, 1);
  roi_manual_object_->colour(colour);

  roi_manual_object_->position(far_left);
  roi_manual_object_->normal(0, 0, 1);
  roi_manual_object_->colour(colour);

  // Triangles (counter-clockwise winding order)
  roi_manual_object_->triangle(0, 1, 2);
  roi_manual_object_->triangle(0, 2, 3);

  roi_manual_object_->end();
}

void AttentionCloudDisplay::destroyRoi()
{
  if (roi_manual_object_) {
    roi_manual_object_->clear();
    if (roi_node_) {
      roi_node_->detachObject(roi_manual_object_);
    }
    if (context_ && context_->getSceneManager()) {
      context_->getSceneManager()->destroyManualObject(roi_manual_object_);
    }
    roi_manual_object_ = nullptr;
  }

  if (roi_node_) {
    if (context_ && context_->getSceneManager()) {
      context_->getSceneManager()->destroySceneNode(roi_node_);
    }
    roi_node_ = nullptr;
  }

  if (roi_material_) {
    roi_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(roi_material_->getName());
    roi_material_.reset();
  }
}


}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionCloudDisplay, rviz_common::Display)
