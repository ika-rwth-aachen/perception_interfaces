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
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreTechnique.h>
#include <algorithm>
#include <cmath>
#include <exception>

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
  // general properties
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

  // velocity options
  velocity_scale_ = new rviz_common::properties::FloatProperty(
      "Velocity scale", 1.0, "Scale the length of the velocity arrows", viz_velocity_);
  velocity_scale_->setMin(0.0);
  velocity_height_ = new rviz_common::properties::BoolProperty(
      "Set height with Velocity", false, "Set the height of the arrow according to the EgoVehicle's velocity", viz_velocity_);
  use_velocity_color_ = new rviz_common::properties::BoolProperty(
      "Use velocity color", true,
      "Use the custom velocity color instead of the bounding-box color.", viz_velocity_);
  velocity_color_property_ = new rviz_common::properties::ColorProperty(
      "Velocity Color", QColor(255, 0, 255), "Color to visualize velocity arrow", viz_velocity_);

  // acceleration options
  acceleration_scale_ = new rviz_common::properties::FloatProperty(
      "Acceleration scale", 10.0, "Scale the length of the acceleration arrows", viz_acceleration_);
  acceleration_scale_->setMin(0.0);
  use_acceleration_color_ = new rviz_common::properties::BoolProperty(
      "Use acceleration color", true,
      "Use the custom acceleration color instead of the bounding-box color.", viz_acceleration_);
  acceleration_color_property_ = new rviz_common::properties::ColorProperty(
      "Acceleration Color", QColor(255, 0, 0), "Color to visualize acceleration arrow", viz_acceleration_);

  // text printing options
  char_height_ =
      new rviz_common::properties::FloatProperty("Char height", 0.5, "Height of characters in metres.", viz_text_);
  char_height_->setMin(0.01);
  text_offset_ = new rviz_common::properties::FloatProperty(
      "Vertical offset", 1.0, "Clearance between the vehicle roof and the text in metres.", viz_text_);
  text_offset_->setMin(0.0);
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
  timeout_property_->setMin(0.001);

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
  v_max_property_->setMin(0.001);
  a_max_property_->setMin(0.001);

  // trajectory ending customization
  trajectory_end_cap_ = new rviz_common::properties::EnumProperty(
    "End Cap", "Straight",
    "End shape of the trajectory ribbon.", viz_trajectory_);
  trajectory_end_cap_->addOption("Straight", 0);
  trajectory_end_cap_->addOption("Round", 1);
  trajectory_round_segments_ = new rviz_common::properties::IntProperty(
    "Round Segments", 16,
    "Segments used for round end cap.", trajectory_end_cap_);
  trajectory_round_segments_->setMin(4);
  trajectory_round_segments_->setMax(64);
  trajectory_fade_out_ = new rviz_common::properties::BoolProperty(
    "Fade Out", false,
    "Fade alpha to zero at the end of the trajectory.", viz_trajectory_);
  trajectory_fade_length_ = new rviz_common::properties::FloatProperty(
    "Fade Length [m]", 5.0,
    "Length over which to fade the end.", trajectory_fade_out_);

  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  trajectory_alpha_property_->setMin(0);
  trajectory_alpha_property_->setMax(1);
}

EgoDataDisplay::~EgoDataDisplay() {
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  timeout_timer_.reset();

  if (initialized()) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void EgoDataDisplay::onInitialize() {
  MFDClass::onInitialize();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);

  // create or fetch material for thick trajectory line with vertex colors and transparency
  if (!Ogre::MaterialManager::getSingleton().resourceExists(trajectory_material_name_)) {
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
        trajectory_material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    if (mat) {
      Ogre::Technique *tech = mat->getTechnique(0);
      if (!tech) tech = mat->createTechnique();
      Ogre::Pass *pass = tech->getPass(0);
      if (!pass) pass = tech->createPass();
      pass->setLightingEnabled(false);
      pass->setDepthCheckEnabled(true);
      pass->setDepthWriteEnabled(false);  // better blending for translucent ribbons
      pass->setCullingMode(Ogre::CULL_NONE);
      pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    }
  }
}

void EgoDataDisplay::reset() {
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  MFDClass::reset();
  manual_object_->clear();
  viz_ego_state_.reset();
}

bool validateFloats(perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  try {
    bool valid = rviz_common::validateFloats(perception_msgs::object_access::getPose(msg->state));
    valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getVelocityMagnitude(msg->state));
    for (const auto& state : msg->trajectory_planned) {
      valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getPose(state));
      valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getVelocityMagnitude(state));
      valid = valid && rviz_common::validateFloats(perception_msgs::object_access::getAccelerationMagnitude(state));
    }
    return valid;
  } catch (const std::exception&) {
    return false;
  }
}

void EgoDataDisplay::processMessage(perception_msgs::msg::EgoData::ConstSharedPtr msg) {

  // check for supported object model id
  if (msg->state.model_id != perception_msgs::msg::EGO::MODEL_ID && msg->state.model_id != perception_msgs::msg::EGORWS::MODEL_ID) {
    std::string error_msg = "Model ID " + std::to_string(msg->state.model_id) + " is not supported";
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

  if (timeout_timer_) {
    timeout_timer_->cancel();
    timeout_timer_.reset();
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  // set colors
  Ogre::ColourValue color_general = rviz_common::properties::qtToOgre(color_property_->getColor());
  Ogre::ColourValue color_text = rviz_common::properties::qtToOgre(color_property_->getColor());

  color_general.a = alpha_property_->getFloat();
  color_text.a = alpha_property_->getFloat();

  // To-Do: find a clever way so that we don't need this map for EgoData
  classification_color_map_ = {{perception_msgs::msg::ObjectClassification::UNCLASSIFIED, color_general},
                               {perception_msgs::msg::ObjectClassification::PEDESTRIAN, color_general},
                               {perception_msgs::msg::ObjectClassification::BICYCLE, color_general},
                               {perception_msgs::msg::ObjectClassification::MOTORCYCLE, color_general},
                               {perception_msgs::msg::ObjectClassification::CAR, color_general},
                               {perception_msgs::msg::ObjectClassification::UTILITY, color_general},
                               {perception_msgs::msg::ObjectClassification::BUS, color_general},
                               {perception_msgs::msg::ObjectClassification::ANIMAL, color_general},
                               {perception_msgs::msg::ObjectClassification::VRU, color_general},
                               {perception_msgs::msg::ObjectClassification::MICRO, color_general},
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

  bool visualize_text = viz_text_->getBool();
  float char_height = char_height_->getFloat();
  bool print_vel = false;
  if (visualize_text) {
    print_vel = print_vel_->getBool();
  }

  // set trajectory variables
  Ogre::ColourValue color_trajectory = rviz_common::properties::qtToOgre(color_property_base_->getColor());
  color_trajectory.a = trajectory_alpha_property_->getFloat();
  
  manual_object_->clear();

  // render object state
  viz_ego_state_ = std::make_shared<perception_msgs::rendering::ObjectState>(classification_color_map_, color_text,
                                                                             scene_manager_, scene_node_);
  // settings
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
    viz_ego_state_->setTextOffset(text_offset_->getFloat());
    viz_ego_state_->printVelocity(print_vel);
  }
  // render
  Ogre::Vector3 bb_dims(msg->length, msg->width, msg->height);
  viz_ego_state_->setBoundingBoxDimensions(bb_dims);
  viz_ego_state_->setObjectState(msg->state);
  if (!visualize_z_dimension) {
    viz_ego_state_->setZComponent(msg->height / 2.0);
  }

  // display trajectory
  if (viz_trajectory_->getBool() && !msg->trajectory_planned.empty()) {
    auto compute_dynamic_color = [&](size_t source_index) -> Ogre::ColourValue {
      Ogre::ColourValue color_pos = rviz_common::properties::qtToOgre(color_positive_dynamics_->getColor());
      Ogre::ColourValue color_neg = rviz_common::properties::qtToOgre(color_negative_dynamics_->getColor());
      Ogre::ColourValue dyn = color_trajectory;
      const float v = static_cast<float>(
          3.6 * perception_msgs::object_access::getVelocityMagnitude(msg->trajectory_planned[source_index]));
      const float a = static_cast<float>(
          perception_msgs::object_access::getAccelerationMagnitude(msg->trajectory_planned[source_index]));
      float f = 0.0f;
      if (drop_down_->getOptionInt() == 2) {
        f = std::min(1.0f, std::abs(v) / std::max(0.001f, v_max_property_->getFloat()));
        if (v > 0) {
          dyn.r = (1 - f) * color_trajectory.r + f * color_pos.r;
          dyn.g = (1 - f) * color_trajectory.g + f * color_pos.g;
          dyn.b = (1 - f) * color_trajectory.b + f * color_pos.b;
        } else {
          dyn.r = (1 - f) * color_trajectory.r + f * color_neg.r;
          dyn.g = (1 - f) * color_trajectory.g + f * color_neg.g;
          dyn.b = (1 - f) * color_trajectory.b + f * color_neg.b;
        }
      } else if (drop_down_->getOptionInt() == 3) {
        f = std::min(1.0f, std::abs(a) / std::max(0.001f, a_max_property_->getFloat()));
        if (a > 0) {
          dyn.r = (1 - f) * color_trajectory.r + f * color_pos.r;
          dyn.g = (1 - f) * color_trajectory.g + f * color_pos.g;
          dyn.b = (1 - f) * color_trajectory.b + f * color_pos.b;
        } else {
          dyn.r = (1 - f) * color_trajectory.r + f * color_neg.r;
          dyn.g = (1 - f) * color_trajectory.g + f * color_neg.g;
          dyn.b = (1 - f) * color_trajectory.b + f * color_neg.b;
        }
      }
      return dyn;
    };

    // Transform points to the geometric centre and discard consecutive
    // duplicates. Zero-length segments used to inject arbitrary UNIT_X
    // normals into the strip and were a common source of diamond-shaped
    // self-intersections on short trajectories.
    std::vector<Ogre::Vector3> pts;
    std::vector<size_t> source_indices;
    pts.reserve(msg->trajectory_planned.size());
    source_indices.reserve(msg->trajectory_planned.size());
    for (size_t i = 0; i < msg->trajectory_planned.size(); ++i) {
      geometry_msgs::msg::Pose gm_pose = perception_msgs::object_access::getPose(msg->trajectory_planned[i]);
      geometry_msgs::msg::TransformStamped tf;
      geometry_msgs::msg::Vector3 translation_map;
      tf.transform.translation.x = gm_pose.position.x;
      tf.transform.translation.y = gm_pose.position.y;
      tf.transform.translation.z = gm_pose.position.z;
      tf.transform.rotation = gm_pose.orientation;
      tf2::doTransform(msg->state.reference_point.translation_to_geometric_center, translation_map, tf);
      Ogre::Vector3 point(
          perception_msgs::object_access::getX(msg->trajectory_planned[i]) + translation_map.x,
          perception_msgs::object_access::getY(msg->trajectory_planned[i]) + translation_map.y,
          0.0f);
      if (pts.empty() || (point - pts.back()).squaredLength() > 1e-6f) {
        pts.push_back(point);
        source_indices.push_back(i);
      }
    }

    const float half_width = std::max(0.0f, static_cast<float>(0.5 * msg->width));
    const int round_segments = std::max(8, trajectory_round_segments_->getInt());
    auto emit_disk = [&](const Ogre::Vector3& centre, const Ogre::ColourValue& color) {
      manual_object_->begin(trajectory_material_name_, Ogre::RenderOperation::OT_TRIANGLE_FAN);
      manual_object_->position(centre);
      manual_object_->colour(color);
      for (int i = 0; i <= round_segments; ++i) {
        const float angle = static_cast<float>(i) / static_cast<float>(round_segments) * Ogre::Math::TWO_PI;
        manual_object_->position(centre + Ogre::Vector3(std::cos(angle), std::sin(angle), 0.0f) * half_width);
        manual_object_->colour(color);
      }
      manual_object_->end();
    };

    if (pts.size() == 1 && half_width > 0.0f) {
      emit_disk(pts.front(), compute_dynamic_color(source_indices.front()));
    } else if (pts.size() > 1 && half_width > 0.0f) {
      const size_t num_points = pts.size();

      std::vector<float> cumlen(num_points, 0.0f);
      float total_len = 0.0f;
      for (size_t i = 1; i < num_points; ++i) {
        total_len += (pts[i] - pts[i - 1]).length();
        cumlen[i] = total_len;
      }

      std::vector<Ogre::Vector3> dirs(num_points - 1);
      std::vector<Ogre::Vector3> norms(num_points - 1);
      std::vector<Ogre::Vector3> left_start(num_points - 1), right_start(num_points - 1);
      std::vector<Ogre::Vector3> left_end(num_points - 1), right_end(num_points - 1);
      for (size_t i = 0; i + 1 < num_points; ++i) {
        dirs[i] = pts[i + 1] - pts[i];
        dirs[i].normalise();
        norms[i] = Ogre::Vector3(-dirs[i].y, dirs[i].x, 0.0f);
        left_start[i] = pts[i] + norms[i] * half_width;
        right_start[i] = pts[i] - norms[i] * half_width;
        left_end[i] = pts[i + 1] + norms[i] * half_width;
        right_end[i] = pts[i + 1] - norms[i] * half_width;
      }

      // Share a miter vertex only for well-conditioned joins. Sharp turns keep
      // the two segment end edges separate; every emitted quad then remains
      // non-self-intersecting instead of collapsing into a diamond.
      for (size_t i = 1; i + 1 < num_points; ++i) {
        Ogre::Vector3 join_normal = norms[i - 1] + norms[i];
        if (dirs[i - 1].dotProduct(dirs[i]) > -0.5f && join_normal.squaredLength() > 1e-8f) {
          join_normal.normalise();
          const float denominator = join_normal.dotProduct(norms[i]);
          if (denominator > 1e-3f) {
            const float miter = half_width / denominator;
            if (miter <= 2.0f * half_width) {
              const Ogre::Vector3 left = pts[i] + join_normal * miter;
              const Ogre::Vector3 right = pts[i] - join_normal * miter;
              left_end[i - 1] = left;
              right_end[i - 1] = right;
              left_start[i] = left;
              right_start[i] = right;
            }
          }
        }
      }

      manual_object_->begin(trajectory_material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
      for (size_t i = 0; i + 1 < num_points; ++i) {
        Ogre::ColourValue c0 = compute_dynamic_color(source_indices[i]);
        Ogre::ColourValue c1 = compute_dynamic_color(source_indices[i + 1]);
        if (trajectory_fade_out_->getBool()) {
          const float fade_len = std::max(0.001f, trajectory_fade_length_->getFloat());
          c0.a *= std::min(1.0f, (total_len - cumlen[i]) / fade_len);
          c1.a *= std::min(1.0f, (total_len - cumlen[i + 1]) / fade_len);
        }

        manual_object_->position(left_start[i]); manual_object_->colour(c0);
        manual_object_->position(right_start[i]); manual_object_->colour(c0);
        manual_object_->position(right_end[i]); manual_object_->colour(c1);

        manual_object_->position(left_start[i]); manual_object_->colour(c0);
        manual_object_->position(right_end[i]); manual_object_->colour(c1);
        manual_object_->position(left_end[i]); manual_object_->colour(c1);
      }
      manual_object_->end();

      if (trajectory_end_cap_->getOptionInt() == 1) {
        const Ogre::Vector3& direction = dirs.back();
        const Ogre::Vector3& normal = norms.back();
        Ogre::ColourValue end_color = compute_dynamic_color(source_indices.back());
        if (trajectory_fade_out_->getBool()) {
          end_color.a = 0.0f;
        }
        manual_object_->begin(trajectory_material_name_, Ogre::RenderOperation::OT_TRIANGLE_FAN);
        manual_object_->position(pts.back());
        manual_object_->colour(end_color);
        for (int i = 0; i <= round_segments; ++i) {
          const float angle = static_cast<float>(i) / static_cast<float>(round_segments) * Ogre::Math::PI;
          manual_object_->position(
              pts.back() + (normal * std::cos(angle) + direction * std::sin(angle)) * half_width);
          manual_object_->colour(end_color);
        }
        manual_object_->end();
      }
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
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  this->reset();
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::EgoDataDisplay, rviz_common::Display)
