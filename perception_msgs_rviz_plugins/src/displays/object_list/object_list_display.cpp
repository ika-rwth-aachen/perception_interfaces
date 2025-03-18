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

#include "perception_msgs/displays/object_list/object_list_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"

namespace perception_msgs
{
namespace displays
{

ObjectListDisplay::ObjectListDisplay()
{
  // General Properties
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(0, 255, 25),
    "Color to visualize objects if no specific class color is defined.", this);
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.5f,
    "Amount of transparency to apply.", this);

  // Object Appearance Properties
  appearance_properties_ = new rviz_common::properties::Property("Appearance Properties", " ", "Different properties to modify the appearance of objects", this);
  viz_mesh_ = new rviz_common::properties::BoolProperty("Mesh", true, 
    "Visualize the object as a mesh.", appearance_properties_);
  viz_bounding_box_ = new rviz_common::properties::BoolProperty("Bounding box", false,
    "Visualize the bounding box of an object.", appearance_properties_);
  viz_direction_ind_ = new rviz_common::properties::BoolProperty("Orientation Indication", false,
    "Visualize a cone indicating the direction of an object.", viz_bounding_box_);
  color_property_group_ = new rviz_common::properties::BoolProperty("Classification coloring", true,
    "Use the object classification to set the color.", viz_bounding_box_);
  // Classification Color Properties
  color_property_pedestrian_ = new rviz_common::properties::ColorProperty(
    "PEDESTRIAN", QColor(25, 255, 255),
    "Color to visualize objects with classification PEDESTRIAN.", color_property_group_);
  color_property_bicycle_ = new rviz_common::properties::ColorProperty(
    "BICYCLE", QColor(255, 255, 25),
    "Color to visualize objects with classification BICYCLE.", color_property_group_);
  color_property_motorbike_ = new rviz_common::properties::ColorProperty(
    "MOTORBIKE", QColor(255, 255, 25),
    "Color to visualize objects with classification MOTORBIKE.", color_property_group_);
  color_property_car_ = new rviz_common::properties::ColorProperty(
    "CAR", QColor(25, 25, 255),
    "Color to visualize objects with classification CAR.", color_property_group_);
  color_property_truck_ = new rviz_common::properties::ColorProperty(
    "TRUCK", QColor(25, 25, 255),
    "Color to visualize objects with classification TRUCK.", color_property_group_);
  color_property_van_ = new rviz_common::properties::ColorProperty(
    "VAN", QColor(25, 25, 255),
    "Color to visualize objects with classification VAN.", color_property_group_);
  color_property_bus_ = new rviz_common::properties::ColorProperty(
    "BUS", QColor(25, 25, 255),
    "Color to visualize objects with classification BUS.", color_property_group_);
  color_property_animal_ = new rviz_common::properties::ColorProperty(
    "ANIMAL", QColor(0, 128, 128),
    "Color to visualize objects with classification ANIMAL.", color_property_group_);
  color_property_road_obstacle_ = new rviz_common::properties::ColorProperty(
    "ROAD-OBSTACLE", QColor(0, 128, 128),
    "Color to visualize objects with classification ROAD-OBSTACLE.", color_property_group_);
  color_property_train_ = new rviz_common::properties::ColorProperty(
    "TRAIN", QColor(0, 128, 128),
    "Color to visualize objects with classification TRAIN.", color_property_group_);
  color_property_trailer_ = new rviz_common::properties::ColorProperty(
    "TRAILER", QColor(25, 25, 255),
    "Color to visualize objects with classification TRAILER.", color_property_group_);
  color_property_unknown_ = new rviz_common::properties::ColorProperty(
    "UNKNOWN", QColor(128, 128, 128),
    "Color to visualize objects with classification UNKNOWN.", color_property_group_);

  // Velocity options
  viz_velocity_ = new rviz_common::properties::BoolProperty("Velocity arrow", false,
    "Add an arrow visualizing the object's velocity", this);
  velocity_scale_ = new rviz_common::properties::FloatProperty("Velocity scale", 1.0, "Scale the length of the velocity arrows", viz_velocity_);
  use_velocity_color_ = new rviz_common::properties::BoolProperty("Use velocity color", true,
    "Visualize the velocity arrow in the bbox color. If not set, use specific color instead.", viz_velocity_);
  velocity_color_property_ = new rviz_common::properties::ColorProperty(
    "Velocity Color", QColor(255, 0, 255),
    "Color to visualize velocity arrow", viz_velocity_);

  // Acceleration options
  viz_acceleration_ = new rviz_common::properties::BoolProperty("Acceleration arrow", false,
    "Add an arrow visualizing the object's acceleration", this);
  acceleration_scale_ = new rviz_common::properties::FloatProperty("Acceleration scale", 10.0, "Scale the length of the acceleration arrows", viz_acceleration_);
  use_acceleration_color_ = new rviz_common::properties::BoolProperty("Use acceleration color", true,
    "Visualize the acceleration arrow in the bbox color. If not set, use specific color instead.", viz_acceleration_);
  acceleration_color_property_ = new rviz_common::properties::ColorProperty(
    "Acceleration Color", QColor(255, 0, 0),
    "Color to visualize acceleration arrow", viz_acceleration_);

  // Prediction options
  viz_predictions_ = new rviz_common::properties::BoolProperty("Predictions", true, 
    "Add trajectories of the predictions", this);
  color_property_prediction_line_ = new rviz_common::properties::ColorProperty(
    "Line Color", QColor(0, 255, 0),
    "Color to visualize prediction lines", viz_predictions_);
  width_property_prediction_ = new rviz_common::properties::FloatProperty("Line Width", 1.0, 
    "Width of the prediction lines", viz_predictions_);
  viz_prediction_probabilities_ = new rviz_common::properties::BoolProperty("Probabilities", false, 
    "Visualize the probabilities of the predictions", viz_predictions_);
  char_height_prediction_probs_ = new rviz_common::properties::FloatProperty("Char height", 4.0, "Height of characters, ~ Font size", viz_prediction_probabilities_);
  viz_prediction_points_ = new rviz_common::properties::BoolProperty("Prediction Points", true, 
    "Visualize the points of the predictions", viz_predictions_);
  color_property_prediction_points_ = new rviz_common::properties::ColorProperty(
    "Point Color", QColor(255, 170, 0),
    "Color to visualize prediction points", viz_prediction_points_);
  width_property_prediction_points_ = new rviz_common::properties::FloatProperty("Point Width", 0.5, 
    "Width of the prediction points", viz_prediction_points_);

  // Text printing options
  viz_text_ = new rviz_common::properties::BoolProperty("Text information", false,
    "Visualize informing text about an object.", this);
  char_height_ = new rviz_common::properties::FloatProperty("Char height", 4.0, "Height of characters, ~ Font size", viz_text_);
  use_text_color_class_ = new rviz_common::properties::BoolProperty("Classification coloring", true,
    "Visualize the text info with respect to the classification coloring. If not set, use general color instead.", viz_text_);
  print_id_ = new rviz_common::properties::BoolProperty("Object ID", false,
    "Print the ID of the current object within text.", viz_text_);
  print_exist_prob_ = new rviz_common::properties::BoolProperty("Existence probability", false,
    "Print the existence probability of the current object within text.", viz_text_);
  print_class_ = new rviz_common::properties::BoolProperty("Classification", true,
    "Print the classification of the current object within text.", viz_text_);
  print_vel_ = new rviz_common::properties::BoolProperty("Velocity", true,
    "Print the speed of the current object within text.", viz_text_);

  // timeout properties
  enable_timeout_property_ = new rviz_common::properties::BoolProperty("Timeout",
                                                                       true,
                                                                       "Remove traffic lights after timeout if no new ones have been received",
                                                                       this);
  timeout_property_ = new rviz_common::properties::FloatProperty("Duration",
                                                                 1.0,
                                                                 "Timeout duration in seconds (wall time)",
                                                                 enable_timeout_property_);

  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
}

ObjectListDisplay::~ObjectListDisplay()
{
  if (initialized() ) viz_object_states_.clear();
  delete appearance_properties_;
  delete color_property_;
  delete alpha_property_;
  delete color_property_group_;
  delete viz_bounding_box_;
  delete viz_direction_ind_;
  delete viz_text_;
  delete viz_velocity_;
  delete viz_acceleration_;
  delete velocity_scale_;
  delete use_velocity_color_;
  delete velocity_color_property_;
  delete acceleration_scale_;
  delete use_acceleration_color_;
  delete acceleration_color_property_;
  delete char_height_;
  delete use_text_color_class_;
  delete print_id_;
  delete print_exist_prob_;
  delete print_class_;
  delete print_vel_;
  delete color_property_pedestrian_;
  delete color_property_bicycle_;
  delete color_property_motorbike_;
  delete color_property_car_;
  delete color_property_truck_;
  delete color_property_van_;
  delete color_property_bus_;
  delete color_property_animal_;
  delete color_property_road_obstacle_;
  delete color_property_train_;
  delete color_property_trailer_;
  delete color_property_unknown_;
  delete enable_timeout_property_;
  delete timeout_property_;
}

void ObjectListDisplay::onInitialize()
{
  MFDClass::onInitialize();
  is_reset.store(false);
}

void ObjectListDisplay::reset()
{
  MFDClass::reset();
  viz_object_states_.clear();
}

void ObjectListDisplay::onEnable(){
  MFDClass::onEnable();
  is_reset.store(false);
}

void ObjectListDisplay::onDisable(){
  is_reset.store(true);
  MFDClass::onDisable();
}

bool validateFloats(perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  bool valid = true;
  for (int i=0; i<int(msg->objects.size()); i++)
  {
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getX(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getY(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getZ(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getWidth(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getLength(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getHeight(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getYaw(msg->objects[i]));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getVelocityMagnitude(msg->objects[i]));
  }
  return valid;
}

void ObjectListDisplay::processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{

  // check for supported object model id
  for (const auto& obj : msg->objects) {
    if (obj.state.model_id != perception_msgs::msg::ISCACTR::MODEL_ID && obj.state.model_id != perception_msgs::msg::HEXAMOTION::MODEL_ID) {
      std::string error_msg = "Model ID not supported"; // TODO: add model_id value and object number (didn't know how to convert int to string offline)
      this->setStatus(rviz_common::properties::StatusProperty::Error, "Model ID", QString::fromStdString(error_msg));
      return;
    }
  }

  if(is_reset.load())
  {
    return;
  }
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

  // Set Colors
  Ogre::ColourValue color_general = rviz_common::properties::qtToOgre(color_property_->getColor());
  Ogre::ColourValue color_text = rviz_common::properties::qtToOgre(color_property_->getColor());

  Ogre::ColourValue color_pedestrian = color_general;
  Ogre::ColourValue color_bicycle = color_general;
  Ogre::ColourValue color_motorbike = color_general;
  Ogre::ColourValue color_car = color_general;
  Ogre::ColourValue color_truck = color_general;
  Ogre::ColourValue color_van = color_general;
  Ogre::ColourValue color_bus = color_general;
  Ogre::ColourValue color_animal = color_general;
  Ogre::ColourValue color_road_obstacle = color_general;
  Ogre::ColourValue color_train = color_general;
  Ogre::ColourValue color_trailer = color_general;
  Ogre::ColourValue color_unknown = color_general;

  // Colors for Classes
  if(color_property_group_->getBool()) {
    color_pedestrian = rviz_common::properties::qtToOgre(color_property_pedestrian_->getColor());
    color_bicycle = rviz_common::properties::qtToOgre(color_property_bicycle_->getColor());
    color_motorbike = rviz_common::properties::qtToOgre(color_property_motorbike_->getColor());
    color_car = rviz_common::properties::qtToOgre(color_property_car_->getColor());
    color_truck = rviz_common::properties::qtToOgre(color_property_truck_->getColor());
    color_van = rviz_common::properties::qtToOgre(color_property_van_->getColor());
    color_bus = rviz_common::properties::qtToOgre(color_property_bus_->getColor());
    color_animal = rviz_common::properties::qtToOgre(color_property_animal_->getColor());
    color_road_obstacle = rviz_common::properties::qtToOgre(color_property_road_obstacle_->getColor());
    color_train = rviz_common::properties::qtToOgre(color_property_train_->getColor());
    color_trailer = rviz_common::properties::qtToOgre(color_property_trailer_->getColor());
    color_unknown = rviz_common::properties::qtToOgre(color_property_unknown_->getColor());
  }

  color_general.a = alpha_property_->getFloat();
  color_text.a = alpha_property_->getFloat();

  color_pedestrian.a = alpha_property_->getFloat();
  color_bicycle.a = alpha_property_->getFloat();
  color_motorbike.a = alpha_property_->getFloat();
  color_car.a = alpha_property_->getFloat();
  color_truck.a = alpha_property_->getFloat();
  color_van.a = alpha_property_->getFloat();
  color_bus.a = alpha_property_->getFloat();
  color_animal.a = alpha_property_->getFloat();
  color_road_obstacle.a = alpha_property_->getFloat();
  color_trailer.a = alpha_property_->getFloat();
  color_unknown.a = alpha_property_->getFloat();

  classification_color_map_ =
  {
    {perception_msgs::msg::ObjectClassification::UNCLASSIFIED, color_unknown},
    {perception_msgs::msg::ObjectClassification::PEDESTRIAN, color_pedestrian},
    {perception_msgs::msg::ObjectClassification::BICYCLE, color_bicycle},
    {perception_msgs::msg::ObjectClassification::MOTORBIKE, color_motorbike},
    {perception_msgs::msg::ObjectClassification::CAR, color_car},
    {perception_msgs::msg::ObjectClassification::TRUCK, color_truck},
    {perception_msgs::msg::ObjectClassification::VAN, color_van},
    {perception_msgs::msg::ObjectClassification::BUS, color_bus},
    {perception_msgs::msg::ObjectClassification::ANIMAL, color_animal},
    {perception_msgs::msg::ObjectClassification::ROAD_OBSTACLE, color_road_obstacle},
    {perception_msgs::msg::ObjectClassification::TRAIN, color_train},
    {perception_msgs::msg::ObjectClassification::TRAILER, color_trailer},
    {perception_msgs::msg::ObjectClassification::CAR_UNION, color_car},
    {perception_msgs::msg::ObjectClassification::TRUCK_UNION, color_truck},
    {perception_msgs::msg::ObjectClassification::BIKE_UNION, color_motorbike},
    {perception_msgs::msg::ObjectClassification::UNKNOWN, color_unknown}
  };

  bool visualize_bounding_box = viz_bounding_box_->getBool();
  bool visualize_mesh = viz_mesh_->getBool();
  bool visualize_direction_indicator = viz_direction_ind_->getBool();
  bool visualize_velocity = viz_velocity_->getBool();
  bool visualize_predictions = viz_predictions_->getBool();
  bool visualize_prediction_points = viz_prediction_points_->getBool();
  bool visualize_prediction_probabilities = viz_prediction_probabilities_->getBool();
  
  float velocity_scale;
  bool use_velocity_color;
  Ogre::ColourValue velocity_color;
  if(visualize_velocity)
  {
    velocity_scale = velocity_scale_->getFloat();
    use_velocity_color = use_velocity_color_->getBool();
    velocity_color = rviz_common::properties::qtToOgre(velocity_color_property_->getColor());
    velocity_color.a = alpha_property_->getFloat();
  }
  bool visualize_acceleration = viz_acceleration_->getBool();
  float acceleration_scale;
  bool use_acceleration_color;
  Ogre::ColourValue acceleration_color;
  if(visualize_acceleration)
  {
    acceleration_scale = acceleration_scale_->getFloat();
    use_acceleration_color = use_acceleration_color_->getBool();
    acceleration_color = rviz_common::properties::qtToOgre(acceleration_color_property_->getColor());
    acceleration_color.a = alpha_property_->getFloat();
  }
  bool visualize_text = viz_text_->getBool();
  float char_height = char_height_->getFloat();
  bool print_id = false;
  bool print_ex_prob = false;
  bool print_class = false;
  bool print_vel = false;
  bool use_text_color_class = false;
  if(visualize_text) {
    use_text_color_class = use_text_color_class_->getBool();
    print_id = print_id_->getBool();
    print_ex_prob = print_exist_prob_->getBool();
    print_class = print_class_->getBool();
    print_vel = print_vel_->getBool();
  }

  viz_object_states_.clear();
  if(msg->objects.size()) {
    for (int i = 0; i<int(msg->objects.size()); i++)
    {
      // Render Object State
      std::unique_ptr<perception_msgs::rendering::ObjectState> state_ptr = std::make_unique<perception_msgs::rendering::ObjectState>(classification_color_map_, color_text, scene_manager_, scene_node_);
      // Settings
      state_ptr->setVisualizeDirectionIndicator(visualize_direction_indicator);
      state_ptr->setVisualizeBoundingBox(visualize_bounding_box);
      state_ptr->setVisualizeMesh(visualize_mesh);
      state_ptr->setVisualizeVelocity(visualize_velocity);
      if(visualize_velocity)
      {
        state_ptr->setVelocityScale(velocity_scale);
        state_ptr->setUseVelocityColor(use_velocity_color);
        state_ptr->setVelocityColor(velocity_color);
      }
      state_ptr->setVisualizeAcceleration(visualize_acceleration);
      if(visualize_acceleration)
      {
        state_ptr->setAccelerationScale(acceleration_scale);
        state_ptr->setUseAccelerationColor(use_acceleration_color);
        state_ptr->setAccelerationColor(acceleration_color);
      }
      state_ptr->setVisualizeText(visualize_text);
      if(visualize_text)
      {
        state_ptr->setCharHeight(char_height);
        state_ptr->setColorTextWithClass(use_text_color_class);
        if(print_id) {
          state_ptr->setObjectId(msg->objects[i].id);
          state_ptr->printObjectId(print_id);
        }
        if(print_ex_prob) {
          state_ptr->setExistanceProb(msg->objects[i].existence_probability);
          state_ptr->printExistanceProb(print_ex_prob);
        }
        state_ptr->printVelocity(print_vel);
        state_ptr->printClass(print_class);
      }
      state_ptr->setVisualizePredictions(visualize_predictions);
      if(visualize_predictions) {
        state_ptr->setPredictionLineColor(rviz_common::properties::qtToOgre(color_property_prediction_line_->getColor()));
        state_ptr->setPredictionLineWidth(width_property_prediction_->getFloat());

        state_ptr->setVisualizePredictionProbabilities(viz_prediction_probabilities_->getBool());
        if(visualize_prediction_probabilities) {
          state_ptr->setPredictionProbCharHeight(char_height_prediction_probs_->getFloat());
        }

        state_ptr->setVisualizePredictionPoints(visualize_prediction_points);
        if(visualize_prediction_points) {
          state_ptr->setPredictionPointColor(rviz_common::properties::qtToOgre(color_property_prediction_points_->getColor()));
          state_ptr->setPredictionPointWidth(width_property_prediction_points_->getFloat());
        }
      }
      // Render
      state_ptr->setObjectState(msg->objects[i].state);
      state_ptr->setObjectStatePredictions(msg->objects[i].state_predictions);
      viz_object_states_.push_back(std::move(state_ptr));
    }
  }

  // reset scene after timeout, if enabled
  if (enable_timeout_property_->getBool()) {
    timeout_timer_ = rviz_ros_node_.lock()->get_raw_node()->create_wall_timer(
      std::chrono::duration<float>(timeout_property_->getFloat()),
      std::bind(&ObjectListDisplay::timeoutTimerCallback, this)
    );
  }
}

void ObjectListDisplay::timeoutTimerCallback() {

  timeout_timer_->cancel();
  this->reset();
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::ObjectListDisplay, rviz_common::Display)