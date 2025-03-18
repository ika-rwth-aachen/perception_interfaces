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

#include "perception_msgs/displays/ego_data/ego_data_display.hpp"

#include <OgreBillboardSet.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"

namespace perception_msgs {
namespace displays {

EgoDataDisplay::EgoDataDisplay() {
  // General Properties
  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(255, 0, 25),
                                                               "Color to visualize the Ego-Vehicle.", this);
  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 0.5f, "Amount of transparency to apply.", this);
  viz_bounding_box_ = new rviz_common::properties::BoolProperty("Bounding box", true,
                                                                "Visualize the bounding box of the Ego-Vehicle.", this);
  viz_direction_ind_ = new rviz_common::properties::BoolProperty(
      "Orientation", true, "Visualize the direction indicator of the Ego-Vehicle.", this);
  viz_velocity_ = new rviz_common::properties::BoolProperty(
      "Velocity arrow", true, "Add an arrow visualizing the EgoVehicles's velocity", this);
  viz_acceleration_ = new rviz_common::properties::BoolProperty(
      "Acceleration arrow", false, "Add an arrow visualizing the EgoVehicles's acceleration", this);
  viz_text_ = new rviz_common::properties::BoolProperty("Text information", false,
                                                        "Visualize informing text about the Ego-Vehicle.", this);
  viz_z_dim_ = new rviz_common::properties::BoolProperty("Visualize Z dimension", true,
                                                         "Visualize the Z component of the Ego-Vehicle.", this);

  // Velocity options
  velocity_scale_ = new rviz_common::properties::FloatProperty(
      "Velocity scale", 1.0, "Scale the length of the velocity arrows", viz_velocity_);
  velocity_height_ = new rviz_common::properties::BoolProperty(
      "Set height with Velocity", false, "Set the height of the arrow according to the EgoVehicle's velocity", viz_velocity_);
  use_velocity_color_ = new rviz_common::properties::BoolProperty(
      "Use velocity color", true,
      "Visualize the velocity arrow in the bbox color. If not set, use specific color instead.", viz_velocity_);
  velocity_color_property_ = new rviz_common::properties::ColorProperty(
      "Velocity Color", QColor(255, 0, 255), "Color to visualize velocity arrow", viz_velocity_);

  // Acceleration options
  acceleration_scale_ = new rviz_common::properties::FloatProperty(
      "Acceleration scale", 10.0, "Scale the length of the acceleration arrows", viz_acceleration_);
  use_acceleration_color_ = new rviz_common::properties::BoolProperty(
      "Use acceleration color", true,
      "Visualize the acceleration arrow in the bbox color. If not set, use specific color instead.", viz_acceleration_);
  acceleration_color_property_ = new rviz_common::properties::ColorProperty(
      "Acceleration Color", QColor(255, 0, 0), "Color to visualize acceleration arrow", viz_acceleration_);

  // Text printing options
  char_height_ =
      new rviz_common::properties::FloatProperty("Char height", 4.0, "Height of characters, ~ Font size", viz_text_);
  print_vel_ = new rviz_common::properties::BoolProperty("Velocity", true,
                                                         "Print the speed of the Ego-Vehicle within text.", viz_text_);

  // timeout properties
  enable_timeout_property_ = new rviz_common::properties::BoolProperty("Timeout",
                                                                       true,
                                                                       "Remove traffic lights after timeout if no new ones have been received",
                                                                       this);
  timeout_property_ = new rviz_common::properties::FloatProperty("Duration",
                                                                 1.0,
                                                                 "Timeout duration in seconds (wall time)",
                                                                 enable_timeout_property_);

  // trajectory properties
  viz_trajectory_ = new rviz_common::properties::BoolProperty("Planned Trajectory", true,
    "Visualize the trajectory as vehicle outlines on road surface.", this);
  trajectory_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.1f,
    "Amount of transparency to apply.", viz_trajectory_, SLOT(queueRender()));
  drop_down_ = new rviz_common::properties::EnumProperty("Color Coding", QString::fromStdString(option_vel_),
    "Visualization properties for dynamic behavior.", viz_trajectory_);
  drop_down_->addOptionStd(default_, 1);
  drop_down_->addOptionStd(option_vel_, 2);
  drop_down_->addOptionStd(option_accel_, 3);
  color_options_ = new rviz_common::properties::Property("Color Options", " ", "Customize colors of the trajectory display", viz_trajectory_, SLOT(queueRender()));
  parameter_options_ = new rviz_common::properties::Property("Parameter Options", " ", "Customize parameterization of the trajectory's color coding", viz_trajectory_, SLOT(queueRender()));
  color_property_base_ = new rviz_common::properties::ColorProperty(
    "Base Color", QColor(0, 170, 0),
    "Color to draw the vehicle outlines.", color_options_, SLOT(queueRender()));
  color_negative_dynamics_ = new rviz_common::properties::ColorProperty(
    "Negative dynamics", QColor(0, 0, 0),
    "Color to draw vehicle outlines corresponding to states with negative acceleration", color_options_, SLOT(queueRender()));
  color_positive_dynamics_ = new rviz_common::properties::ColorProperty(
    "Positive dynamics", QColor(255, 0, 0),
    "Color to draw vehicle outlines corresponding to states with positive acceleration", color_options_, SLOT(queueRender()));
  v_max_property_ = new rviz_common::properties::FloatProperty("max. velocity  [km/h]", v_max_, "Velocity limit for color coding", parameter_options_, SLOT(queueRender()));
  a_max_property_ = new rviz_common::properties::FloatProperty("max. acceleration [m/s²]", a_max_, "acceleration limit for color coding", parameter_options_, SLOT(queueRender()));

  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  trajectory_alpha_property_->setMin(0);
  trajectory_alpha_property_->setMax(1);
}

EgoDataDisplay::~EgoDataDisplay() {
  if (initialized()) scene_manager_->destroyManualObject(manual_object_);
  delete color_property_;
  delete alpha_property_;
  delete viz_z_dim_;
  delete viz_bounding_box_;
  delete viz_direction_ind_;
  delete viz_text_;
  delete viz_velocity_;
  delete viz_acceleration_;
  delete velocity_scale_;
  delete velocity_height_;
  delete use_velocity_color_;
  delete velocity_color_property_;
  delete acceleration_scale_;
  delete use_acceleration_color_;
  delete acceleration_color_property_;
  delete char_height_;
  delete print_vel_;
  delete enable_timeout_property_;
  delete timeout_property_;
  delete viz_trajectory_;
  delete trajectory_alpha_property_;
  delete drop_down_;
  delete color_options_;
  delete parameter_options_;
  delete color_property_base_;
  delete color_negative_dynamics_;
  delete color_positive_dynamics_;
  delete v_max_property_;
  delete a_max_property_;
}

void EgoDataDisplay::onInitialize() {
  MFDClass::onInitialize();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void EgoDataDisplay::reset() {
  MFDClass::reset();
  manual_object_->clear();
  flat_areas_.clear();
}

bool validateFloats(perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  bool valid = true;
  valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getX(msg->state));
  valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getY(msg->state));
  valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getZ(msg->state));
  valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getYaw(msg->state));
  valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getVelocityMagnitude(msg->state));
  return valid;
}

void EgoDataDisplay::processMessage(perception_msgs::msg::EgoData::ConstSharedPtr msg) {

  // check for supported object model id
  if (msg->state.model_id != perception_msgs::msg::EGO::MODEL_ID && msg->state.model_id != perception_msgs::msg::EGORWS::MODEL_ID) {
    std::string error_msg = "Model ID" + std::to_string(msg->state.model_id) + "not supported";
    this->setStatus(rviz_common::properties::StatusProperty::Error, "Model ID", QString::fromStdString(error_msg));
    return;
  }

  if (!validateFloats(msg)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
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

  // Set Colors
  Ogre::ColourValue color_general = rviz_common::properties::qtToOgre(color_property_->getColor());
  Ogre::ColourValue color_text = rviz_common::properties::qtToOgre(color_property_->getColor());

  color_general.a = alpha_property_->getFloat();
  color_text.a = alpha_property_->getFloat();

  // To-Do: find a clever way so that we don't need this map for EgoData
  classification_color_map_ = {{perception_msgs::msg::ObjectClassification::UNCLASSIFIED, color_general},
                               {perception_msgs::msg::ObjectClassification::PEDESTRIAN, color_general},
                               {perception_msgs::msg::ObjectClassification::BICYCLE, color_general},
                               {perception_msgs::msg::ObjectClassification::MOTORBIKE, color_general},
                               {perception_msgs::msg::ObjectClassification::CAR, color_general},
                               {perception_msgs::msg::ObjectClassification::TRUCK, color_general},
                               {perception_msgs::msg::ObjectClassification::VAN, color_general},
                               {perception_msgs::msg::ObjectClassification::BUS, color_general},
                               {perception_msgs::msg::ObjectClassification::ANIMAL, color_general},
                               {perception_msgs::msg::ObjectClassification::ROAD_OBSTACLE, color_general},
                               {perception_msgs::msg::ObjectClassification::TRAIN, color_general},
                               {perception_msgs::msg::ObjectClassification::TRAILER, color_general},
                               {perception_msgs::msg::ObjectClassification::CAR_UNION, color_general},
                               {perception_msgs::msg::ObjectClassification::TRUCK_UNION, color_general},
                               {perception_msgs::msg::ObjectClassification::BIKE_UNION, color_general},
                               {perception_msgs::msg::ObjectClassification::UNKNOWN, color_general}};

  bool visualize_bounding_box = viz_bounding_box_->getBool();
  bool visualize_direction_indicator = viz_direction_ind_->getBool();
  bool visualize_velocity = viz_velocity_->getBool();
  float velocity_scale;
  bool use_velocity_color;
  bool velocity_height = velocity_height_->getBool();
  Ogre::ColourValue velocity_color;
  if (visualize_velocity) {
    velocity_scale = velocity_scale_->getFloat();
    use_velocity_color = use_velocity_color_->getBool();
    velocity_color = rviz_common::properties::qtToOgre(velocity_color_property_->getColor());
    velocity_color.a = alpha_property_->getFloat();
  }

  bool visualize_acceleration = viz_acceleration_->getBool();
  float acceleration_scale;
  bool use_acceleration_color;
  Ogre::ColourValue acceleration_color;
  if (visualize_acceleration) {
    acceleration_scale = acceleration_scale_->getFloat();
    use_acceleration_color = use_acceleration_color_->getBool();
    acceleration_color = rviz_common::properties::qtToOgre(acceleration_color_property_->getColor());
    acceleration_color.a = alpha_property_->getFloat();
  }

  bool visualize_z_dimension = viz_z_dim_->getBool();
  if (!visualize_z_dimension) {
    viz_ego_state_->setZComponent(msg->height / 2.0);
  }

  bool visualize_text = viz_text_->getBool();
  float char_height = char_height_->getFloat();
  bool print_vel = false;
  if (visualize_text) {
    print_vel = print_vel_->getBool();
  }

  // set trajectory variables
  Ogre::Vector3 flat_dims(msg->length, msg->width, 0);
  Ogre::ColourValue color_trajectory = rviz_common::properties::qtToOgre(color_property_base_->getColor());
  color_trajectory.a = trajectory_alpha_property_->getFloat();
  
  manual_object_->clear();

  // Render Object State
  viz_ego_state_ = std::make_shared<perception_msgs::rendering::ObjectState>(classification_color_map_, color_text,
                                                                             scene_manager_, scene_node_);
  // Settings
  viz_ego_state_->setVisualizeDirectionIndicator(visualize_direction_indicator);
  viz_ego_state_->setVisualizeBoundingBox(visualize_bounding_box);
  viz_ego_state_->setVisualizeVelocity(visualize_velocity);
  viz_ego_state_->setVelocityHeight(velocity_height);
  if (visualize_velocity) {
    viz_ego_state_->setVelocityScale(velocity_scale);
    viz_ego_state_->setUseVelocityColor(use_velocity_color);
    viz_ego_state_->setVelocityColor(velocity_color);
  }
  viz_ego_state_->setVisualizeAcceleration(visualize_acceleration);
  if (visualize_acceleration) {
    viz_ego_state_->setAccelerationScale(acceleration_scale);
    viz_ego_state_->setUseAccelerationColor(use_acceleration_color);
    viz_ego_state_->setAccelerationColor(acceleration_color);
  }
  viz_ego_state_->setVisualizeText(visualize_text);
  if (visualize_text) {
    viz_ego_state_->setCharHeight(char_height);
    viz_ego_state_->printVelocity(print_vel);
  }
  // Render
  Ogre::Vector3 bb_dims(msg->length, msg->width, msg->height);
  viz_ego_state_->setBoundingBoxDimensions(bb_dims);
  viz_ego_state_->setObjectState(msg->state);

  // Display trajectory
  flat_areas_.clear();
  size_t num_points = msg->trajectory_planned.size();
  if (num_points > 0 and viz_trajectory_->getBool()){
    // initialize translation into the geometric center
    geometry_msgs::msg::Pose gm_pose;
    geometry_msgs::msg::TransformStamped tf;
    geometry_msgs::msg::Vector3 translation_map;
    
    // create flat geometric shape for each state in the planned trajectory
    for (size_t i = 0; i < num_points; ++i) {
      // update translation into the geometric center
      gm_pose = perception_msgs::object_access::getPose(msg->trajectory_planned[i]);
      tf.transform.translation.x = gm_pose.position.x;
      tf.transform.translation.y = gm_pose.position.y;
      tf.transform.translation.z = gm_pose.position.z;
      tf.transform.rotation = gm_pose.orientation;
      tf2::doTransform(msg->state.reference_point.translation_to_geometric_center, translation_map, tf);

      // create shape of projected bounding box with translation to geometric center
      std::shared_ptr<rviz_rendering::Shape> bb_area = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, scene_node_);
      Ogre::Vector3 flat_pos(perception_msgs::object_access::getX(msg->trajectory_planned[i]) + translation_map.x, perception_msgs::object_access::getY(msg->trajectory_planned[i]) + translation_map.y, 0);
      Ogre::Quaternion flat_orientation(Ogre::Radian(perception_msgs::object_access::getYaw(msg->trajectory_planned[i])), Ogre::Vector3::UNIT_Z);
      bb_area->setPosition(flat_pos);
      bb_area->setOrientation(flat_orientation);
      bb_area->setScale(flat_dims);
      Ogre::ColourValue color_pos = rviz_common::properties::qtToOgre(color_positive_dynamics_->getColor());;
      Ogre::ColourValue color_neg = rviz_common::properties::qtToOgre(color_negative_dynamics_->getColor());;
      Ogre::ColourValue dynamic_color = color_trajectory;
      float v = (float) 3.6 * perception_msgs::object_access::getVelocityMagnitude(msg->trajectory_planned[i]);
      float a = perception_msgs::object_access::getAccelerationMagnitude(msg->trajectory_planned[i]);
      float f = 0;
      if (drop_down_->getOptionInt() == 2){
        f = std::min((float) 1.0, v/v_max_property_->getFloat());
        if (v > 0){
          dynamic_color.r = (1-f)*color_trajectory.r + f*color_pos.r;
          dynamic_color.g = (1-f)*color_trajectory.g + f*color_pos.g;
          dynamic_color.b = (1-f)*color_trajectory.b + f*color_pos.b;
        } else {
          dynamic_color.r = (1-f)*color_trajectory.r + f*color_neg.r;
          dynamic_color.g = (1-f)*color_trajectory.g + f*color_neg.g;
          dynamic_color.b = (1-f)*color_trajectory.b + f*color_neg.b;
        }
      } else if (drop_down_->getOptionInt() == 3){
        f = std::min((float) 1.0, std::abs(a)/a_max_property_->getFloat());
        if (a > 0){
          dynamic_color.r = (1-f)*color_trajectory.r + f*color_pos.r;
          dynamic_color.g = (1-f)*color_trajectory.g + f*color_pos.g;
          dynamic_color.b = (1-f)*color_trajectory.b + f*color_pos.b;
        } else {
          dynamic_color.r = (1-f)*color_trajectory.r + f*color_neg.r;
          dynamic_color.g = (1-f)*color_trajectory.g + f*color_neg.g;
          dynamic_color.b = (1-f)*color_trajectory.b + f*color_neg.b;
        }
      }
      // modify material
      bb_area->setColor(dynamic_color);
      // Get the Ogre::Entity from the shape
      Ogre::Entity* entity = bb_area->getEntity();
      // Get the material name used by the entity
      Ogre::String material_name = entity->getSubEntity(0)->getMaterialName();
      // Get the material from the material manager
      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(material_name);
      if (material) {
        for (unsigned int i = 0; i < material->getNumTechniques(); ++i) {
            Ogre::Technique* technique = material->getTechnique(i);
            for (unsigned int j = 0; j < technique->getNumPasses(); ++j) {
                Ogre::Pass* pass = technique->getPass(j);
                pass->setAmbient(dynamic_color);
                pass->setDiffuse(dynamic_color);
                pass->setEmissive(dynamic_color);
                pass->setSpecular(dynamic_color);
                pass->setSelfIllumination(dynamic_color);
            }
        }
      }
      flat_areas_.push_back(bb_area);
    }
  }

  // reset scene after timeout, if enabled
  if (enable_timeout_property_->getBool()) {
    timeout_timer_ = rviz_ros_node_.lock()->get_raw_node()->create_wall_timer(
      std::chrono::duration<float>(timeout_property_->getFloat()),
      std::bind(&EgoDataDisplay::timeoutTimerCallback, this)
    );
  }
}

void EgoDataDisplay::timeoutTimerCallback() {

  timeout_timer_->cancel();
  this->reset();
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::EgoDataDisplay, rviz_common::Display)