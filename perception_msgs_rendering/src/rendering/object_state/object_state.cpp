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

#include "perception_msgs/rendering/object_state/object_state.hpp"
#include "perception_msgs/msg/object_classification.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreManualObject.h>
#include <OgreMovableObject.h>
#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubMesh.h>
#include <OgreTechnique.h>
#include <OgreVector.h>

#include <cmath>
#include <algorithm>

#include <rviz_rendering/mesh_loader.hpp>
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/logging.hpp"

#include <iomanip>

namespace perception_msgs {

namespace rendering {

ObjectState::ObjectState(const std::unordered_map<unsigned int, Ogre::ColourValue>& classification_color_map,
                         const Ogre::ColourValue& text_color, Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  if (!parent_node) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  classification_color_map_ = classification_color_map;
  text_color_ = text_color;
}

ObjectState::~ObjectState() {
  if (hoverboard_mo_) {
    scene_manager_->destroyManualObject(hoverboard_mo_);
    hoverboard_mo_ = nullptr;
  }
  if (hoverboard_glow_mo_) {
    scene_manager_->destroyManualObject(hoverboard_glow_mo_);
    hoverboard_glow_mo_ = nullptr;
  }
  if (xy_uncertainty_mo_) {
    scene_manager_->destroyManualObject(xy_uncertainty_mo_);
    xy_uncertainty_mo_ = nullptr;
  }
  if (yaw_uncertainty_mo_) {
    scene_manager_->destroyManualObject(yaw_uncertainty_mo_);
    yaw_uncertainty_mo_ = nullptr;
  }
  if (safety_margin_mo_) {
    scene_manager_->destroyManualObject(safety_margin_mo_);
    safety_margin_mo_ = nullptr;
  }
  scene_manager_->destroySceneNode(scene_node_);
}

void ObjectState::setObjectState(const perception_msgs::msg::ObjectState& state) {
  object_state_ = state;
  // Get Classification
  classification_ = perception_msgs::object_access::getClassWithHighestProbability(object_state_);

  // Set color
  Ogre::ColourValue color;
  if (b_colour_override_)
    color = color_override_;  // Check if there is an override of the colour
  else {                      // Set colour with respect to classification_color_map_
    if (classification_color_map_.count(classification_.type))
      color = classification_color_map_[classification_.type];
    else {  // Set default colour (grey)
      color.r = 128.0;
      color.g = 128.0;
      color.b = 128.0;
      color.a = 1.0;
    }
  }

  // Check if there is an override of alpha
  if (alpha_override_ >= 0.0) color.a = alpha_override_;

  switch (classification_.type) {
    case perception_msgs::msg::ObjectClassification::CAR:
      //setObjectStateCar(object_state_, color);
      setObjectStateVizDefault(object_state_, color, visualize_bounding_box_, indicate_direction_);
      break;
    default:
      setObjectStateVizDefault(object_state_, color, visualize_bounding_box_, indicate_direction_);
      break;
  }

  // Set Text
  if (visualize_text_) {
    if (use_class_color_for_text_)
      setObjectStateTextDefault(object_state_, color);
    else
      setObjectStateTextDefault(object_state_, text_color_);
  }

  // Set Position of Scene Node
  setSceneNodePose(object_state_);
}

void ObjectState::setZComponent(const double& val) {
  perception_msgs::object_access::setZ(object_state_, val);
  setSceneNodePose(object_state_);
}

void ObjectState::setObjectStatePredictions(
    const std::vector<perception_msgs::msg::ObjectStatePrediction>& predictions) {
  predictions_ = predictions;
  //Fill billboard_line_predictions_ with empty vectors
  billboard_line_predictions_.clear();
  text_prob_vector_.clear();
  bbox_predictions_.clear();
  for (size_t i = 0; i < predictions.size(); i++) {
    billboard_line_predictions_.push_back(std::shared_ptr<rviz_rendering::BillboardLine>());
    text_prob_vector_.push_back(std::shared_ptr<rviz_rendering::MovableText>());
    bbox_predictions_.push_back(std::vector<std::shared_ptr<rviz_rendering::Shape>>(predictions[i].states.size()));
  }

  for (size_t i = 0; i < predictions.size(); i++) {
    Ogre::ColourValue line_color = prediction_line_color_;
    Ogre::ColourValue point_color = prediction_point_color_;
    if (predictions[i].probability >= 0.0) {
      line_color.a = predictions[i].probability;
      point_color.a = predictions[i].probability;
    } else {
      line_color.a = 0.0;
      point_color.a = 0.0;
    }  //TODO: Set to what value for the else case?

    setObjectPredictionsVizDefault(predictions[i].states, billboard_line_predictions_[i], bbox_predictions_[i],
                                   line_color, point_color);

    if (visualize_predictions_ && visualize_prediction_probabilities_ && !predictions[i].states.empty()) {
      setObjectPredictionProbabilityText(predictions[i].probability, predictions[i].states[0], text_prob_vector_[i]);
    }
  }
}

void ObjectState::overrideAlpha(const float& alpha) { alpha_override_ = alpha; }

void ObjectState::overrideColour(const Ogre::ColourValue& colour) {
  color_override_ = colour;
  b_colour_override_ = true;
}

void ObjectState::setBoundingBoxDimensions(const Ogre::Vector3& dims) {
  bbox_dims_ = dims;
  b_bbox_dims_set_ = true;
}

void ObjectState::setVisualizeDirectionIndicator(const bool& val) { indicate_direction_ = val; }

void ObjectState::setVisualizeBoundingBox(const bool& val) { visualize_bounding_box_ = val; }

void ObjectState::setVisualizeMesh(const bool& val) { visualize_mesh_ = val; }

void ObjectState::setVisualizeHoverboard(const bool& val) { visualize_hoverboard_ = val; }
void ObjectState::setHoverboardThickness(const float& val) { hoverboard_thickness_ = std::max(0.0f, val); }
void ObjectState::setHoverboardCornerRadius(const float& val) { hoverboard_corner_radius_ = std::max(0.0f, val); }
void ObjectState::setHoverboardGlow(const bool& val) { hoverboard_glow_ = val; }
void ObjectState::setHoverboardGlowParams(const float& height, const float& intensity) {
  hoverboard_glow_height_ = std::max(0.0f, height);
  hoverboard_glow_intensity_ = std::max(0.0f, std::min(1.0f, intensity));
}

void ObjectState::setHoverboardCapStyle(int style) {
  hoverboard_cap_style_ = std::max(0, std::min(2, style));
}

void ObjectState::setHoverboardCornerSegments(int segs) {
  hoverboard_corner_segments_ = std::max(3, std::min(64, segs));
}

void ObjectState::setVisualizeXYUncertainty(const bool& val) { visualize_xy_uncertainty_ = val; }
void ObjectState::setXYUncertaintyColor(const Ogre::ColourValue& color) { xy_uncertainty_color_ = color; }
void ObjectState::setXYUncertaintyAlpha(const float& alpha) { xy_uncertainty_alpha_ = std::max(0.0f, std::min(1.0f, alpha)); }
void ObjectState::setXYUncertaintyScale(const float& scale) { xy_uncertainty_scale_ = std::max(0.1f, scale); }
void ObjectState::setXYUncertaintySegments(int segs) { xy_uncertainty_segments_ = std::max(8, std::min(128, segs)); }
void ObjectState::setVisualizeYawUncertainty(const bool& val) { visualize_yaw_uncertainty_ = val; }
void ObjectState::setYawUncertaintyColor(const Ogre::ColourValue& color) { yaw_uncertainty_color_ = color; }
void ObjectState::setYawUncertaintyAlpha(const float& alpha) { yaw_uncertainty_alpha_ = std::max(0.0f, std::min(1.0f, alpha)); }
void ObjectState::setYawUncertaintyConeLength(const float& length) { yaw_uncertainty_cone_length_ = std::max(0.1f, length); }

void ObjectState::setVisualizeSafetyMargins(const bool& val) { visualize_safety_margins_ = val; }
void ObjectState::setSafetyMarginColor(const Ogre::ColourValue& color) { safety_margin_color_ = color; }
void ObjectState::setSafetyMarginColor2Sigma(const Ogre::ColourValue& color) { safety_margin_color_2sigma_ = color; }
void ObjectState::setSafetyMarginColor3Sigma(const Ogre::ColourValue& color) { safety_margin_color_3sigma_ = color; }
void ObjectState::setSafetyMarginAlpha(const float& alpha) { safety_margin_alpha_ = std::max(0.0f, std::min(1.0f, alpha)); }
void ObjectState::setSafetyMargin2Sigma(const bool& val) { safety_margin_2sigma_ = val; }
void ObjectState::setSafetyMargin3Sigma(const bool& val) { safety_margin_3sigma_ = val; }

void ObjectState::setVisualizeVelocity(const bool& val) { visualize_velocity_ = val; }

void ObjectState::setVelocityScale(const float& val) { velocity_scale_ = val; }

void ObjectState::setVelocityHeight(const bool& val) { velocity_height_ = val; }

void ObjectState::setUseVelocityColor(const bool& val) { use_velocity_color_ = val; }

void ObjectState::setVelocityColor(const Ogre::ColourValue& colour) { velocity_color_ = colour; }

void ObjectState::setVisualizeAcceleration(const bool& val) { visualize_acceleration_ = val; }

void ObjectState::setAccelerationScale(const float& val) { acceleration_scale_ = val; }

void ObjectState::setUseAccelerationColor(const bool& val) { use_acceleration_color_ = val; }

void ObjectState::setAccelerationColor(const Ogre::ColourValue& colour) { acceleration_color_ = colour; }

void ObjectState::setVisualizeText(const bool& val) { visualize_text_ = val; }

void ObjectState::setCharHeight(const float& val) { char_height_ = val; }

void ObjectState::setColorTextWithClass(const bool& val) { use_class_color_for_text_ = val; }

void ObjectState::setObjectId(const unsigned int& val) {
  id_set_ = true;
  id_ = val;
}

void ObjectState::setExistanceProb(const double& val) { existence_probability_ = val; }

void ObjectState::printObjectId(const bool& val) { print_id_ = val; }

void ObjectState::printExistanceProb(const bool& val) { print_existance_prob_ = val; }

void ObjectState::printVelocity(const bool& val) { print_velocity_ = val; }

void ObjectState::printClass(const bool& val) { print_class_ = val; }

void ObjectState::setVisualizePredictions(const bool& val) { visualize_predictions_ = val; }

void ObjectState::setVisualizePredictionPoints(const bool& val) { visualize_prediction_points_ = val; }

void ObjectState::setPredictionLineColor(const Ogre::ColourValue& color) {
  prediction_line_color_ = color;
  prediction_line_color_.a = 1.0;
}

void ObjectState::setPredictionPointColor(const Ogre::ColourValue& color) {
  prediction_point_color_ = color;
  prediction_point_color_.a = 1.0;
}

void ObjectState::setPredictionLineWidth(const float& width) { prediction_line_width_ = width; }

void ObjectState::setPredictionPointWidth(const float& width) { prediction_point_width_ = width; }

void ObjectState::setVisualizePredictionProbabilities(const bool& val) { visualize_prediction_probabilities_ = val; }

void ObjectState::setPredictionProbCharHeight(const float& val) { char_height_prediction_probs_ = val; }

void ObjectState::setObjectStateVizDefault(const perception_msgs::msg::ObjectState& state,
                                           const Ogre::ColourValue& color, const bool& viz_bb,
                                           const bool& indicate_direction) {
  if (!b_bbox_dims_set_) {  // Bounding-Box dimensions are not set explicitly (e.g. EgoData), try to get bounding box dimensions from ObjectState vector
    bbox_dims_.x = perception_msgs::object_access::getLength(state);
    bbox_dims_.y = perception_msgs::object_access::getWidth(state);
    bbox_dims_.z = perception_msgs::object_access::getHeight(state);
  }

  if (visualize_mesh_) {
    // Clean up any previously created mesh node and its attached objects to avoid leaks
    if (mesh_node_) {
      // Detach and destroy all attached movable objects (e.g., entities)
      while (mesh_node_->numAttachedObjects() > 0) {
        Ogre::MovableObject* mo = mesh_node_->getAttachedObject(0);
        mesh_node_->detachObject(mo);
        scene_manager_->destroyMovableObject(mo);
      }
      scene_manager_->destroySceneNode(mesh_node_);
      mesh_node_ = nullptr;
    }
    //load mesh to render based on classification
    Ogre::Entity* entity;
    Ogre::MeshPtr mesh;

    std::string package;

    using namespace perception_msgs::msg;
    switch (classification_.type) {
      case ObjectClassification::CAR:
        package = "package://perception_msgs_rendering/meshes/car.stl";
        material = "CarMaterial";
        break;
      case ObjectClassification::UTILITY:
        package = "package://perception_msgs_rendering/meshes/utility.stl";
        material = "UtilityMaterial";
        break;
      case ObjectClassification::BUS:
        package = "package://perception_msgs_rendering/meshes/bus.stl";
        material = "BusMaterial";
        break;
      case ObjectClassification::BICYCLE:
        package = "package://perception_msgs_rendering/meshes/bicycle.stl";
        material = "BicycleMaterial";
        break;
      case ObjectClassification::MOTORCYCLE:
        package = "package://perception_msgs_rendering/meshes/motorcycle.stl";
        material = "MotorcycleMaterial";
        break;
      case ObjectClassification::PEDESTRIAN:
        package = "package://perception_msgs_rendering/meshes/pedestrian.stl";
        material = "PedestrianMaterial";
        break;
      default:
        package = "package://perception_msgs_rendering/meshes/car.stl";
        material = "CarMaterial";
        break;
    }

  #if defined(ROS_DISTRO_noetic) || defined(ROS_DISTRO_humble) || defined(ROS_DISTRO_jazzy)
    mesh = rviz_rendering::loadMeshFromResource(package);
  #else
    resource_retriever::Retriever* retriever;
    mesh = rviz_rendering::loadMeshFromResource(retriever, package);
  #endif  

    // Check if mesh was loaded successfully (nullptr if loading failed)
    if (mesh)
    {
      // compute mesh scaling factors to fixed height
      Ogre::Vector3 mesh_dims = mesh->getBounds().getSize();
      double scaling_factor_z;
      switch (classification_.type) {
      case ObjectClassification::CAR:
        scaling_factor_z = kFixedMeshHeightCar / mesh_dims.z;
        break;
      case ObjectClassification::UTILITY:
        scaling_factor_z = kFixedMeshHeightUtility / mesh_dims.z;
        break;
      case ObjectClassification::BUS:
        scaling_factor_z = kFixedMeshHeightBus / mesh_dims.z;
        break;
      case ObjectClassification::BICYCLE:
        scaling_factor_z = kFixedMeshHeightBicycle / mesh_dims.z;
        break;
      case ObjectClassification::MOTORCYCLE:
        scaling_factor_z = kFixedMeshHeightMotorcycle / mesh_dims.z;
        break;
      case ObjectClassification::PEDESTRIAN:
        scaling_factor_z = kFixedMeshHeightPedestrian / mesh_dims.z;
        break;
      default:
        scaling_factor_z = bbox_dims_.z / mesh_dims.z;
        break;
      }
      double scaling_factor_x = scaling_factor_z;
      double scaling_factor_y = scaling_factor_z;

      entity = scene_manager_->createEntity(mesh);

      // Load material in runtime
      Ogre::ResourceGroupManager::getSingletonPtr()->createResourceGroup("object_list_materials");
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation("package://perception_msgs_rendering/materials",
                                                                     "FileSystem", "UserDefinedMaterials", true);
      Ogre::ResourceGroupManager::getSingletonPtr()->initialiseResourceGroup("object_list_materials");
      Ogre::ResourceGroupManager::getSingletonPtr()->loadResourceGroup("object_list_materials");
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation("package://perception_msgs_rendering/materials",
                                                                     "FileSystem", "General");

      entity->setMaterialName(material);

      mesh_node_ = scene_node_->createChildSceneNode();
      mesh_node_->attachObject(static_cast<Ogre::MovableObject*>(entity));
      // Offset mesh_node
      mesh_node_->setPosition(Ogre::Vector3(0.0, 0.0, -bbox_dims_.z / 2.0));

      // Scale mesh_node so it fits the bounding box
      mesh_node_->setScale(Ogre::Vector3(scaling_factor_x, scaling_factor_y, scaling_factor_z));
      Ogre::ResourceGroupManager::getSingletonPtr()->destroyResourceGroup("object_list_materials");
    }
    else
    {
      std::string class_name;
      classToText(classification_, class_name);
      RVIZ_RENDERING_LOG_ERROR_STREAM("Failed to load mesh for object type [" 
          << class_name << "] (ID: " << static_cast<int>(classification_.type) << ")");
    }
  }
  
  // Visualize Bounding Box
  if (viz_bb) {
    Ogre::ColourValue bb_color = color;
    if (color.a >= 1.0) {
      bb_color.a -= 0.2;
    }
    bbox_ = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, scene_node_);
    bbox_->setColor(bb_color);
    bbox_->setScale(bbox_dims_);
  }

  // Visualize cone that indicates the orientation of the object
  if (indicate_direction) {
    bbox_cone_ = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cone, scene_manager_, scene_node_);
    double cone_length = 0.25 * bbox_dims_.x;
    bbox_cone_->setColor(color);
    Ogre::Vector3 scale(bbox_dims_.y, cone_length, bbox_dims_.z);
    bbox_cone_->setScale(scale);
    Ogre::Vector3 cone_pos(((bbox_dims_.x - cone_length) / 2.0), 0.0, 0.0);
    bbox_cone_->setOrientation(Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Z));
    bbox_cone_->setPosition(cone_pos);
  }

  // Hoverboard tile (rounded rectangle with optional upward glow)
  if (visualize_hoverboard_) {
    // Create tile and glow materials if missing
    if (!Ogre::MaterialManager::getSingleton().resourceExists(hoverboard_material_name_)) {
      Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
          hoverboard_material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      if (!mat.isNull()) {
        Ogre::Technique* tech = mat->getTechnique(0);
        if (!tech) tech = mat->createTechnique();
        Ogre::Pass* pass = tech->getPass(0);
        if (!pass) pass = tech->createPass();
        pass->setLightingEnabled(false);
        pass->setDepthCheckEnabled(true);
        pass->setDepthWriteEnabled(true);
        pass->setCullingMode(Ogre::CULL_NONE);
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
      }
    }
    if (!Ogre::MaterialManager::getSingleton().resourceExists(hoverboard_glow_material_name_)) {
      Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
          hoverboard_glow_material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      if (!mat.isNull()) {
        Ogre::Technique* tech = mat->getTechnique(0);
        if (!tech) tech = mat->createTechnique();
        Ogre::Pass* pass = tech->getPass(0);
        if (!pass) pass = tech->createPass();
        pass->setLightingEnabled(false);
        pass->setDepthCheckEnabled(true);
        pass->setDepthWriteEnabled(false);
        pass->setCullingMode(Ogre::CULL_NONE);
        pass->setSceneBlending(Ogre::SBT_ADD);
        pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
      }
    }

    // Ensure manual objects exist
    if (!hoverboard_mo_) {
      hoverboard_mo_ = scene_manager_->createManualObject();
      hoverboard_mo_->setDynamic(true);
      scene_node_->attachObject(hoverboard_mo_);
    }
    if (!hoverboard_glow_mo_) {
      hoverboard_glow_mo_ = scene_manager_->createManualObject();
      hoverboard_glow_mo_->setDynamic(true);
      scene_node_->attachObject(hoverboard_glow_mo_);
    }

    // Dimensions and corner resolution
    const float L = static_cast<float>(bbox_dims_.x);
    const float W = static_cast<float>(bbox_dims_.y);
    float r = hoverboard_corner_radius_;
    const float rmax = 0.5f * std::min(L, W) - 1e-3f;
    if (r > rmax) r = std::max(0.0f, rmax);
    int seg = (hoverboard_cap_style_ == 2) ? hoverboard_corner_segments_ : 1; // round uses configured segments
    const float zBot = static_cast<float>(-0.5 * bbox_dims_.z);
    const float zTop = zBot + hoverboard_thickness_;

    std::vector<Ogre::Vector3> boundary;
    boundary.reserve(4 * (seg + 1));
    auto arc = [&](float cx, float cy, float a0, float a1) {
      if (hoverboard_cap_style_ == 0) {
        return; // square handled separately
      }
      if (hoverboard_cap_style_ == 1) {
        // bevel: straight cut between the two edge-offset points
        boundary.emplace_back(cx + r * std::cos(a0), cy + r * std::sin(a0), zTop);
        boundary.emplace_back(cx + r * std::cos(a1), cy + r * std::sin(a1), zTop);
        return;
      }
      // round corner
      for (int i = 0; i <= seg; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(seg);
        float a = a0 + t * (a1 - a0);
        boundary.emplace_back(cx + r * std::cos(a), cy + r * std::sin(a), zTop);
      }
    };
    const float hx = 0.5f * L - r;
    const float hy = 0.5f * W - r;
    if (hoverboard_cap_style_ == 0 || r <= 1e-6f) {
      // Square corners: rectangle boundary
      boundary.emplace_back(+0.5f * L, +0.5f * W, zTop);
      boundary.emplace_back(-0.5f * L, +0.5f * W, zTop);
      boundary.emplace_back(-0.5f * L, -0.5f * W, zTop);
      boundary.emplace_back(+0.5f * L, -0.5f * W, zTop);
    } else {
      const float PI = static_cast<float>(Ogre::Math::PI);
      const float HALF_PI = static_cast<float>(Ogre::Math::HALF_PI);
      arc(+hx, +hy, 0.0f, HALF_PI);      // top-right
      arc(-hx, +hy, HALF_PI, PI);        // top-left
      arc(-hx, -hy, PI, 1.5f * PI);      // bottom-left
      arc(+hx, -hy, 1.5f * PI, 2.0f * PI); // bottom-right
    }

    Ogre::ColourValue tile_col = color;

    // Build tile
    hoverboard_mo_->clear();
    hoverboard_mo_->begin(hoverboard_material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    // top face
    Ogre::Vector3 cTop(0, 0, zTop);
    for (size_t i = 0; i < boundary.size(); ++i) {
      const Ogre::Vector3 &p0 = boundary[i];
      const Ogre::Vector3 &p1 = boundary[(i + 1) % boundary.size()];
      hoverboard_mo_->position(cTop); hoverboard_mo_->colour(tile_col);
      hoverboard_mo_->position(p0);   hoverboard_mo_->colour(tile_col);
      hoverboard_mo_->position(p1);   hoverboard_mo_->colour(tile_col);
    }
    // sides
    for (size_t i = 0; i < boundary.size(); ++i) {
      Ogre::Vector3 t0 = boundary[i];
      Ogre::Vector3 t1 = boundary[(i + 1) % boundary.size()];
      Ogre::Vector3 b0(t0.x, t0.y, zBot);
      Ogre::Vector3 b1(t1.x, t1.y, zBot);
      hoverboard_mo_->position(t0); hoverboard_mo_->colour(tile_col);
      hoverboard_mo_->position(b0); hoverboard_mo_->colour(tile_col);
      hoverboard_mo_->position(b1); hoverboard_mo_->colour(tile_col);

      hoverboard_mo_->position(t0); hoverboard_mo_->colour(tile_col);
      hoverboard_mo_->position(b1); hoverboard_mo_->colour(tile_col);
      hoverboard_mo_->position(t1); hoverboard_mo_->colour(tile_col);
    }
    hoverboard_mo_->end();

    // Glow skirt
    hoverboard_glow_mo_->clear();
    if (hoverboard_glow_ && hoverboard_glow_height_ > 1e-4f && hoverboard_glow_intensity_ > 1e-4f) {
      Ogre::ColourValue cBottom = tile_col;
      cBottom.r *= hoverboard_glow_intensity_;
      cBottom.g *= hoverboard_glow_intensity_;
      cBottom.b *= hoverboard_glow_intensity_;
      Ogre::ColourValue cTop = cBottom; cTop.a = 0.0f;
      hoverboard_glow_mo_->begin(hoverboard_glow_material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
      const float zG0 = zTop;
      const float zG1 = zTop + hoverboard_glow_height_;
      for (size_t i = 0; i < boundary.size(); ++i) {
        Ogre::Vector3 e0 = boundary[i];
        Ogre::Vector3 e1 = boundary[(i + 1) % boundary.size()];
        Ogre::Vector3 g0(e0.x, e0.y, zG0);
        Ogre::Vector3 g1(e1.x, e1.y, zG0);
        Ogre::Vector3 g2(e1.x, e1.y, zG1);
        Ogre::Vector3 g3(e0.x, e0.y, zG1);
        hoverboard_glow_mo_->position(g0); hoverboard_glow_mo_->colour(cBottom);
        hoverboard_glow_mo_->position(g1); hoverboard_glow_mo_->colour(cBottom);
        hoverboard_glow_mo_->position(g2); hoverboard_glow_mo_->colour(cTop);

        hoverboard_glow_mo_->position(g0); hoverboard_glow_mo_->colour(cBottom);
        hoverboard_glow_mo_->position(g2); hoverboard_glow_mo_->colour(cTop);
        hoverboard_glow_mo_->position(g3); hoverboard_glow_mo_->colour(cTop);
      }
      hoverboard_glow_mo_->end();
    }
  } else {
    // cleanup if previously created
    if (hoverboard_mo_) { scene_manager_->destroyManualObject(hoverboard_mo_); hoverboard_mo_ = nullptr; }
    if (hoverboard_glow_mo_) { scene_manager_->destroyManualObject(hoverboard_glow_mo_); hoverboard_glow_mo_ = nullptr; }
  }

  // Visualize XY uncertainty as ellipse
  if (visualize_xy_uncertainty_) {
    Ogre::ColourValue ellipse_color = xy_uncertainty_color_;
    ellipse_color.a = xy_uncertainty_alpha_;
    visualizeXYUncertaintyEllipse(state, ellipse_color);
  } else {
    if (xy_uncertainty_mo_) { scene_manager_->destroyManualObject(xy_uncertainty_mo_); xy_uncertainty_mo_ = nullptr; }
  }

  // Visualize yaw uncertainty as cone/wedge
  if (visualize_yaw_uncertainty_) {
    Ogre::ColourValue cone_color = yaw_uncertainty_color_;
    cone_color.a = yaw_uncertainty_alpha_;
    visualizeYawUncertaintyCone(state, cone_color);
  } else {
    if (yaw_uncertainty_mo_) { scene_manager_->destroyManualObject(yaw_uncertainty_mo_); yaw_uncertainty_mo_ = nullptr; }
  }

  // Visualize safety margins as expanded bounding box or circle
  if (visualize_safety_margins_) {
    Ogre::ColourValue color_2sigma = safety_margin_color_2sigma_;
    color_2sigma.a = safety_margin_alpha_;
    Ogre::ColourValue color_3sigma = safety_margin_color_3sigma_;
    color_3sigma.a = safety_margin_alpha_;
    visualizeSafetyMargins(state, color_2sigma, color_3sigma);
  } else {
    if (safety_margin_mo_) { scene_manager_->destroyManualObject(safety_margin_mo_); safety_margin_mo_ = nullptr; }
  }

  if (visualize_velocity_) {
    vel_arrow_ =
        std::make_shared<rviz_rendering::Arrow>(scene_manager_, scene_node_, 1.0f, 0.5f, 0.3f,
                                                1.0f);  // shaft_length, shaft_diameter, head_length, head_diameter
    auto vel = perception_msgs::object_access::getVelocity(
        state);  // As this is in LonLat, the orientation is correctly inherited from scene_node_
    Ogre::Vector3 velocity(vel.x, vel.y, vel.z);
    Ogre::Vector3 scale_arrow_vel(perception_msgs::object_access::getVelocityMagnitude(state) * velocity_scale_, 1,
                                  1);  // Scale only length of the arrow
    double height = 0.0;
    Ogre::Vector3 position(0.0, 0.0, 0.0);
    if (velocity_height_) {
        height = perception_msgs::object_access::getVelocityMagnitude(state);
        position.x = -state.reference_point.translation_to_geometric_center.x;
        position.y = -state.reference_point.translation_to_geometric_center.y;
        position.z = -state.reference_point.translation_to_geometric_center.z + height;
    } else {
        position.x = bbox_dims_.x / 2.0;
    }
    vel_arrow_->setPosition(position);
    vel_arrow_->setDirection(velocity);
    vel_arrow_->setScale(scale_arrow_vel);
    vel_arrow_->setColor(use_velocity_color_ ? velocity_color_ : color);
  }
  if (visualize_acceleration_) {
    acc_arrow_ =
        std::make_shared<rviz_rendering::Arrow>(scene_manager_, scene_node_, 1.0f, 0.5f, 0.3f,
                                                1.0f);  // shaft_length, shaft_diameter, head_length, head_diameter
    auto acc = perception_msgs::object_access::getAcceleration(
        state);  // As this is in LonLat, the orientation is correctly inherited from scene_node_
    Ogre::Vector3 acceleration(acc.x, acc.y, acc.z);
    Ogre::Vector3 scale_arrow_acc(perception_msgs::object_access::getAccelerationMagnitude(state) * acceleration_scale_,
                                  1, 1);  // Scale only length of the arrow
    // Place the arrow on the edge with the largest component
    Ogre::Vector3 arrow_position(std::abs(acc.x) >= std::abs(acc.y) ? bbox_dims_.x / 2.0 * (2 * (acc.x >= 0) - 1) : 0.0,
                                 std::abs(acc.x) >= std::abs(acc.y) ? 0.0 : bbox_dims_.y / 2.0 * (2 * (acc.y >= 0) - 1),
                                 0.0);
    acc_arrow_->setDirection(acceleration);
    acc_arrow_->setScale(scale_arrow_acc);
    acc_arrow_->setPosition(arrow_position);
    acc_arrow_->setColor(use_acceleration_color_ ? acceleration_color_ : color);
  }
}

void ObjectState::setObjectPredictionsVizDefault(
    const std::vector<perception_msgs::msg::ObjectState>& states,
    std::shared_ptr<rviz_rendering::BillboardLine>& billboard_line_prediction,
    std::vector<std::shared_ptr<rviz_rendering::Shape>>& bbox_prediction, const Ogre::ColourValue& line_color,
    const Ogre::ColourValue& point_color) {
  if (visualize_predictions_) {
    billboard_line_prediction = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
    billboard_line_prediction->setColor(line_color.r, line_color.g, line_color.b, line_color.a);
    float line_width = prediction_line_width_;
    billboard_line_prediction->setLineWidth(line_width);
    billboard_line_prediction->setMaxPointsPerLine(states.size());
    auto base_state = perception_msgs::object_access::getPose(object_state_);
    tf2::Transform base_state_tf;
    tf2::fromMsg(base_state, base_state_tf);
    for (const auto& state : states) {
      // Transform position to base_state frame
      tf2::Transform state_tf;
      tf2::fromMsg(perception_msgs::object_access::getPose(state), state_tf);
      auto transformed_pos_tf = base_state_tf.inverse() * state_tf;
      auto transformed_pos = transformed_pos_tf.getOrigin();
      billboard_line_prediction->addPoint(Ogre::Vector3(transformed_pos.x(), transformed_pos.y(), transformed_pos.z()));
    }
    if (visualize_prediction_points_) {
      for (size_t i = 0; i < states.size(); i++) {
        bbox_prediction[i] =
            std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
        bbox_prediction[i]->setColor(point_color.r, point_color.g, point_color.b, point_color.a);
        float point_width = prediction_point_width_;
        bbox_prediction[i]->setScale(Ogre::Vector3(point_width, point_width, point_width));
        // Transform pose to base_state frame
        tf2::Transform state_tf;
        tf2::fromMsg(perception_msgs::object_access::getPose(states[i]), state_tf);
        auto transformed_pos_tf = base_state_tf.inverse() * state_tf;
        auto pos = transformed_pos_tf.getOrigin();
        auto orientation = transformed_pos_tf.getRotation();
        bbox_prediction[i]->setPosition(Ogre::Vector3(pos.x(), pos.y(), pos.z()));
        bbox_prediction[i]->setOrientation(
            Ogre::Quaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z()));
      }
    }
  }
}

void ObjectState::setSceneNodePose(const perception_msgs::msg::ObjectState& state) {
  // Set position of scene node
  geometry_msgs::msg::Pose gm_pose = perception_msgs::object_access::getPose(state);
  // state.reference_point.translation_to_geometric_center
  // Define transform from our fixed frame to the reference point frame
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.translation.x = gm_pose.position.x;
  tf.transform.translation.y = gm_pose.position.y;
  tf.transform.translation.z = gm_pose.position.z;
  tf.transform.rotation = gm_pose.orientation;
  geometry_msgs::msg::Vector3 translation_map;
  tf2::doTransform(object_state_.reference_point.translation_to_geometric_center, translation_map, tf);
  Ogre::Vector3 position(gm_pose.position.x + translation_map.x, gm_pose.position.y + translation_map.y,
                         gm_pose.position.z + translation_map.z);
  Ogre::Quaternion orientation(gm_pose.orientation.w, gm_pose.orientation.x, gm_pose.orientation.y,
                               gm_pose.orientation.z);
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void ObjectState::setObjectStateTextDefault(const perception_msgs::msg::ObjectState& state,
                                            const Ogre::ColourValue& color) {
  std::string text;
  if (print_id_) {
    if (id_set_)
      text += "id = " + std::to_string(id_);
    else
      text += std::string("id = ") + std::string("NOT SET");
    text += "\n";
  }

  if (print_existance_prob_) {
    if (existence_probability_ >= 0.0) {
      std::ostringstream textStream;
      textStream << std::fixed << std::setprecision(1)
                 << std::round(existence_probability_ * 1000.0) / 10.0;  // Generate stream with precision of one digit
      text += textStream.str() + "%";
    } else
      text += std::string("Probability NOT SET");
    text += "\n";
  }

  if (print_class_) {
    classToText(classification_, text);
    text += "\n";
  }

  if (print_velocity_) velocityToText(object_state_, text);
  if (!text.size()) return;
  text_ = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", char_height_);
  if (!b_bbox_dims_set_) bbox_dims_.z = perception_msgs::object_access::getHeight(state);
  double height = bbox_dims_.z;
  height += text_->getBoundingRadius();
  Ogre::Vector3 offs(0.0, 0.0, height);
  // Maybe there is a bug in rviz_rendering::MovableText::setGlobalTranslation
  // Currently only the given y-Position is set
  // https://github.com/ros2/rviz/blob/1ac419472ed06cdd52842a8f964f953a75395245/rviz_rendering/src/rviz_rendering/objects/movable_text.cpp#L520
  // Shows that the global_translation-vector is mutliplied with Ogre::Vector3::UNIT_Y is this intended?
  // In the ROS1 implementation the translation-vector is added without any multiplication
  // See: https://github.com/ros-visualization/rviz/blob/ec7ab1b0183244c05fbd2d0d1b8d8f53d8f42f2b/src/rviz/ogre_helpers/movable_text.cpp#L506
  // I've opened an Issue here: https://github.com/ros2/rviz/issues/974
  text_->setGlobalTranslation(offs);
  text_->setColor(color);
  scene_node_->attachObject(text_.get());
}

void ObjectState::setObjectPredictionProbabilityText(const double& probability,
                                                     const perception_msgs::msg::ObjectState& state,
                                                     std::shared_ptr<rviz_rendering::MovableText>& text_prob) {
  // reset text buffer per prediction label to avoid uncontrolled growth
  text_probabilities_.clear();
  if (probability >= 0.0) {
    std::ostringstream textStream;
    textStream << std::fixed << std::setprecision(1)
               << std::round(probability * 1000.0) / 10.0;  // Generate stream with precision of one digit
    text_probabilities_ += textStream.str() + "%";
  } else
    text_probabilities_ += std::string("Probability NOT SET");

  if (!text_probabilities_.size()) return;

  text_probabilities_ += "\n";

  text_prob = std::make_shared<rviz_rendering::MovableText>(text_probabilities_, "Liberation Sans",
                                                            char_height_prediction_probs_);
  if (!b_bbox_dims_set_) bbox_dims_.z = perception_msgs::object_access::getHeight(state);
  double height = bbox_dims_.z;
  height += text_prob->getBoundingRadius();
  Ogre::Vector3 offs(0.0, 0.0, height);
  text_prob->setGlobalTranslation(offs);
  text_prob->setColor(prediction_line_color_);
  scene_node_->attachObject(text_prob.get());
}

void ObjectState::classToText(const perception_msgs::msg::ObjectClassification& classification, std::string& text) {
  switch (classification.type) {
    case perception_msgs::msg::ObjectClassification::UNCLASSIFIED:
      text += "UNCLASSIFIED";
      break;
    case perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      text += "PEDESTRIAN";
      break;
    case perception_msgs::msg::ObjectClassification::BICYCLE:
      text += "BICYCLE";
      break;
    case perception_msgs::msg::ObjectClassification::MOTORCYCLE:
      text += "MOTORCYCLE";
      break;
    case perception_msgs::msg::ObjectClassification::CAR:
      text += "CAR";
      break;
    case perception_msgs::msg::ObjectClassification::UTILITY:
      text += "UTILITY";
      break;
    case perception_msgs::msg::ObjectClassification::VAN:
      text += "VAN (DEPRECATED)";
      break;
    case perception_msgs::msg::ObjectClassification::BUS:
      text += "BUS";
      break;
    case perception_msgs::msg::ObjectClassification::ANIMAL:
      text += "ANIMAL";
      break;
    case perception_msgs::msg::ObjectClassification::ROAD_OBSTACLE:
      text += "ROAD_OBSTACLE (DEPRECATED)";
      break;
    case perception_msgs::msg::ObjectClassification::TRAIN:
      text += "TRAIN (DEPRECATED)";
      break;
    case perception_msgs::msg::ObjectClassification::TRAILER:
      text += "TRAILER (DEPRECATED)";
      break;
    case perception_msgs::msg::ObjectClassification::VRU:
      text += "VRU";
      break;
    case perception_msgs::msg::ObjectClassification::MICRO:
      text += "MICRO";
      break;
    case perception_msgs::msg::ObjectClassification::CAR_UNION:
      text += "CAR_UNION (DEPRECATED)";
      break;
    case perception_msgs::msg::ObjectClassification::TRUCK_UNION:
      text += "TRUCK_UNION (DEPRECATED)";
      break;
    case perception_msgs::msg::ObjectClassification::BIKE_UNION:
      text += "BIKE_UNION (DEPRECATED)";
      break;
    case perception_msgs::msg::ObjectClassification::UNKNOWN:
      text += "UNKNOWN";
      break;
    default:
      text += "TYPE NOT IMPLEMENTED";
      break;
  }
  return;
}

void ObjectState::velocityToText(const perception_msgs::msg::ObjectState& state, std::string& text) {
  if (true) {  // To-Do: Use covariance to check if velocity is set!
    int vel =
        std::round(perception_msgs::object_access::getVelocityMagnitude(state) * 3.6);  // m/s to km/h and cast to int
    text += std::to_string(vel) + " km/h";
  } else
    text += "Velocity NOT SET";
  return;
}

void ObjectState::visualizeXYUncertaintyEllipse(const perception_msgs::msg::ObjectState& state, const Ogre::ColourValue& color) {
  // Get covariance values for X and Y positions (indices 0 and 1 in ISCACTR)
  // Covariance matrix is 12x12 flattened, so var(X) is at [0*12+0]=0, var(Y) is at [1*12+1]=13
  const int state_size = perception_msgs::object_access::getContinuousStateSize(state);
  const auto& cov = state.continuous_state_covariance;
  
  // Check if covariance data is available and valid
  if (cov.size() < static_cast<size_t>(state_size * state_size)) {
    if (xy_uncertainty_mo_) { scene_manager_->destroyManualObject(xy_uncertainty_mo_); xy_uncertainty_mo_ = nullptr; }
    return;
  }
  
  const double var_x = cov[0 * state_size + 0];  // X variance
  const double var_y = cov[1 * state_size + 1];  // Y variance
  
  // Check for invalid covariance values (-1 means invalid/not set, also check for NaN and very large values)
  const double max_valid_variance = 1e6;
  if (var_x < 0.0 || var_y < 0.0 || 
      std::isnan(var_x) || std::isnan(var_y) || 
      std::isinf(var_x) || std::isinf(var_y) ||
      var_x > max_valid_variance || var_y > max_valid_variance) {
    if (xy_uncertainty_mo_) { scene_manager_->destroyManualObject(xy_uncertainty_mo_); xy_uncertainty_mo_ = nullptr; }
    return;
  }
  
  // Standard deviations (radii of ellipse)
  const float sigma_x = static_cast<float>(std::sqrt(var_x)) * xy_uncertainty_scale_;
  const float sigma_y = static_cast<float>(std::sqrt(var_y)) * xy_uncertainty_scale_;
  
  // Minimum visible size threshold
  const float min_size = 0.05f;
  if (sigma_x < min_size && sigma_y < min_size) {
    if (xy_uncertainty_mo_) { scene_manager_->destroyManualObject(xy_uncertainty_mo_); xy_uncertainty_mo_ = nullptr; }
    return;
  }
  
  // Create material if it doesn't exist
  if (!Ogre::MaterialManager::getSingleton().resourceExists(xy_uncertainty_material_name_)) {
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
        xy_uncertainty_material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    if (!mat.isNull()) {
      Ogre::Technique* tech = mat->getTechnique(0);
      if (!tech) tech = mat->createTechnique();
      Ogre::Pass* pass = tech->getPass(0);
      if (!pass) pass = tech->createPass();
      pass->setLightingEnabled(false);
      pass->setDepthCheckEnabled(true);
      pass->setDepthWriteEnabled(false);
      pass->setCullingMode(Ogre::CULL_NONE);
      pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    }
  }
  
  // Create manual object if needed
  if (!xy_uncertainty_mo_) {
    xy_uncertainty_mo_ = scene_manager_->createManualObject();
    xy_uncertainty_mo_->setDynamic(true);
    scene_node_->attachObject(xy_uncertainty_mo_);
  }
  
  // Build ellipse at the bottom of the bounding box, above the hoverboard if enabled
  const float z_bottom = static_cast<float>(-0.5 * bbox_dims_.z);
  // Place ellipse above hoverboard (hoverboard_thickness_ height) plus small offset
  const float z_pos = z_bottom + (visualize_hoverboard_ ? hoverboard_thickness_ : 0.0f) + 0.02f;
  const int segments = xy_uncertainty_segments_;
  
  xy_uncertainty_mo_->clear();
  xy_uncertainty_mo_->begin(xy_uncertainty_material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  // Create filled ellipse with gradient from center (more opaque) to edge (more transparent)
  Ogre::ColourValue center_color = color;
  center_color.a = color.a * 0.8f;
  Ogre::ColourValue edge_color = color;
  edge_color.a = color.a * 0.2f;
  
  // Build ellipse as triangle fan from center
  const float PI = static_cast<float>(Ogre::Math::PI);
  for (int i = 0; i < segments; ++i) {
    float angle0 = 2.0f * PI * static_cast<float>(i) / static_cast<float>(segments);
    float angle1 = 2.0f * PI * static_cast<float>(i + 1) / static_cast<float>(segments);
    
    float x0 = sigma_x * std::cos(angle0);
    float y0 = sigma_y * std::sin(angle0);
    float x1 = sigma_x * std::cos(angle1);
    float y1 = sigma_y * std::sin(angle1);
    
    // Triangle from center to edge
    xy_uncertainty_mo_->position(0.0f, 0.0f, z_pos); xy_uncertainty_mo_->colour(center_color);
    xy_uncertainty_mo_->position(x0, y0, z_pos); xy_uncertainty_mo_->colour(edge_color);
    xy_uncertainty_mo_->position(x1, y1, z_pos); xy_uncertainty_mo_->colour(edge_color);
  }
  
  // Add a subtle ring outline at the edge for better visibility
  const float ring_inner = 0.92f;
  Ogre::ColourValue ring_color = color;
  ring_color.a = color.a * 1.2f;  // Slightly more opaque ring
  for (int i = 0; i < segments; ++i) {
    float angle0 = 2.0f * PI * static_cast<float>(i) / static_cast<float>(segments);
    float angle1 = 2.0f * PI * static_cast<float>(i + 1) / static_cast<float>(segments);
    
    float x0_out = sigma_x * std::cos(angle0);
    float y0_out = sigma_y * std::sin(angle0);
    float x1_out = sigma_x * std::cos(angle1);
    float y1_out = sigma_y * std::sin(angle1);
    float x0_in = sigma_x * ring_inner * std::cos(angle0);
    float y0_in = sigma_y * ring_inner * std::sin(angle0);
    float x1_in = sigma_x * ring_inner * std::cos(angle1);
    float y1_in = sigma_y * ring_inner * std::sin(angle1);
    
    // Two triangles for ring segment
    xy_uncertainty_mo_->position(x0_in, y0_in, z_pos + 0.001f); xy_uncertainty_mo_->colour(ring_color);
    xy_uncertainty_mo_->position(x0_out, y0_out, z_pos + 0.001f); xy_uncertainty_mo_->colour(ring_color);
    xy_uncertainty_mo_->position(x1_out, y1_out, z_pos + 0.001f); xy_uncertainty_mo_->colour(ring_color);
    
    xy_uncertainty_mo_->position(x0_in, y0_in, z_pos + 0.001f); xy_uncertainty_mo_->colour(ring_color);
    xy_uncertainty_mo_->position(x1_out, y1_out, z_pos + 0.001f); xy_uncertainty_mo_->colour(ring_color);
    xy_uncertainty_mo_->position(x1_in, y1_in, z_pos + 0.001f); xy_uncertainty_mo_->colour(ring_color);
  }
  
  xy_uncertainty_mo_->end();
}

void ObjectState::visualizeYawUncertaintyCone(const perception_msgs::msg::ObjectState& state, const Ogre::ColourValue& color) {
  // Get covariance value for yaw (index 7 in ISCACTR)
  // For von-Mises distribution, the covariance stores kappa (concentration parameter)
  const int state_size = perception_msgs::object_access::getContinuousStateSize(state);
  const auto& cov = state.continuous_state_covariance;
  
  // Check if covariance data is available and valid
  if (cov.size() < static_cast<size_t>(state_size * state_size)) {
    if (yaw_uncertainty_mo_) { scene_manager_->destroyManualObject(yaw_uncertainty_mo_); yaw_uncertainty_mo_ = nullptr; }
    return;
  }
  
  const double kappa = cov[7 * state_size + 7];  // Yaw variance/kappa
  
  // Check for invalid covariance value (-1 means invalid/not set, also check for NaN and invalid values)
  if (kappa < 0.0 || std::isnan(kappa) || std::isinf(kappa)) {
    if (yaw_uncertainty_mo_) { scene_manager_->destroyManualObject(yaw_uncertainty_mo_); yaw_uncertainty_mo_ = nullptr; }
    return;
  }
  
  // Also skip if kappa is exactly 0 (would cause division by zero)
  if (kappa < 1e-9) {
    if (yaw_uncertainty_mo_) { scene_manager_->destroyManualObject(yaw_uncertainty_mo_); yaw_uncertainty_mo_ = nullptr; }
    return;
  }
  
  // Convert kappa to angular uncertainty (approximate standard deviation for von-Mises)
  // For von-Mises: variance ≈ 1 - I1(κ)/I0(κ), for large κ: σ ≈ 1/√κ
  // Higher kappa = lower uncertainty, so we use 1/sqrt(kappa) as the angular spread
  const float angular_spread = static_cast<float>(1.0 / std::sqrt(kappa));
  
  // Clamp angular spread to reasonable range (5 degrees to 90 degrees half-angle)
  const float min_spread = static_cast<float>(5.0 * Ogre::Math::PI / 180.0);
  const float max_spread = static_cast<float>(90.0 * Ogre::Math::PI / 180.0);
  const float half_angle = std::max(min_spread, std::min(max_spread, angular_spread));
  
  // Create material if it doesn't exist
  if (!Ogre::MaterialManager::getSingleton().resourceExists(yaw_uncertainty_material_name_)) {
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
        yaw_uncertainty_material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    if (!mat.isNull()) {
      Ogre::Technique* tech = mat->getTechnique(0);
      if (!tech) tech = mat->createTechnique();
      Ogre::Pass* pass = tech->getPass(0);
      if (!pass) pass = tech->createPass();
      pass->setLightingEnabled(false);
      pass->setDepthCheckEnabled(true);
      pass->setDepthWriteEnabled(false);
      pass->setCullingMode(Ogre::CULL_NONE);
      pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    }
  }
  
  // Create manual object if needed
  if (!yaw_uncertainty_mo_) {
    yaw_uncertainty_mo_ = scene_manager_->createManualObject();
    yaw_uncertainty_mo_->setDynamic(true);
    scene_node_->attachObject(yaw_uncertainty_mo_);
  }
  
  // Build cone/wedge showing yaw uncertainty
  // Cone extends from object center in the forward (X) direction
  const float cone_length = yaw_uncertainty_cone_length_;
  const float z_pos = 0.0f;  // At object center height
  const int segments = 16;  // Number of segments for the arc
  
  yaw_uncertainty_mo_->clear();
  yaw_uncertainty_mo_->begin(yaw_uncertainty_material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  // Colors: center more opaque, edges more transparent
  Ogre::ColourValue center_color = color;
  center_color.a = color.a * 0.6f;
  Ogre::ColourValue tip_color = color;
  tip_color.a = color.a * 0.1f;
  Ogre::ColourValue edge_color = color;
  edge_color.a = color.a * 0.4f;
  
  // Build wedge as triangular sectors from center point
  // The wedge spans from -half_angle to +half_angle around the X-axis (forward direction)
  for (int i = 0; i < segments; ++i) {
    float t0 = static_cast<float>(i) / static_cast<float>(segments);
    float t1 = static_cast<float>(i + 1) / static_cast<float>(segments);
    float angle0 = -half_angle + 2.0f * half_angle * t0;
    float angle1 = -half_angle + 2.0f * half_angle * t1;
    
    // Points on the arc at cone_length distance
    // Since yaw rotates in XY plane, the cone extends along X with spread in Y
    float x0 = cone_length * std::cos(angle0);
    float y0 = cone_length * std::sin(angle0);
    float x1 = cone_length * std::cos(angle1);
    float y1 = cone_length * std::sin(angle1);
    
    // Triangle from center to arc edge
    yaw_uncertainty_mo_->position(0.0f, 0.0f, z_pos); yaw_uncertainty_mo_->colour(center_color);
    yaw_uncertainty_mo_->position(x0, y0, z_pos); yaw_uncertainty_mo_->colour(tip_color);
    yaw_uncertainty_mo_->position(x1, y1, z_pos); yaw_uncertainty_mo_->colour(tip_color);
  }
  
  // Add edge lines for better visibility (left and right edges of the cone)
  const float edge_width = 0.08f;
  
  // Left edge
  float left_x = cone_length * std::cos(-half_angle);
  float left_y = cone_length * std::sin(-half_angle);
  float left_perp_x = -std::sin(-half_angle) * edge_width;
  float left_perp_y = std::cos(-half_angle) * edge_width;
  
  yaw_uncertainty_mo_->position(0.0f, 0.0f, z_pos + 0.01f); yaw_uncertainty_mo_->colour(edge_color);
  yaw_uncertainty_mo_->position(left_x - left_perp_x, left_y - left_perp_y, z_pos + 0.01f); yaw_uncertainty_mo_->colour(edge_color);
  yaw_uncertainty_mo_->position(left_x + left_perp_x, left_y + left_perp_y, z_pos + 0.01f); yaw_uncertainty_mo_->colour(edge_color);
  
  // Right edge
  float right_x = cone_length * std::cos(half_angle);
  float right_y = cone_length * std::sin(half_angle);
  float right_perp_x = -std::sin(half_angle) * edge_width;
  float right_perp_y = std::cos(half_angle) * edge_width;
  
  yaw_uncertainty_mo_->position(0.0f, 0.0f, z_pos + 0.01f); yaw_uncertainty_mo_->colour(edge_color);
  yaw_uncertainty_mo_->position(right_x - right_perp_x, right_y - right_perp_y, z_pos + 0.01f); yaw_uncertainty_mo_->colour(edge_color);
  yaw_uncertainty_mo_->position(right_x + right_perp_x, right_y + right_perp_y, z_pos + 0.01f); yaw_uncertainty_mo_->colour(edge_color);
  
  yaw_uncertainty_mo_->end();
}

void ObjectState::visualizeSafetyMargins(const perception_msgs::msg::ObjectState& state, 
                                          const Ogre::ColourValue& color_2sigma,
                                          const Ogre::ColourValue& color_3sigma) {
  // Get covariance values for X and Y positions
  const int state_size = perception_msgs::object_access::getContinuousStateSize(state);
  const auto& cov = state.continuous_state_covariance;
  
  // Check if covariance data is available and valid
  if (cov.size() < static_cast<size_t>(state_size * state_size)) {
    if (safety_margin_mo_) { scene_manager_->destroyManualObject(safety_margin_mo_); safety_margin_mo_ = nullptr; }
    return;
  }
  
  const double var_x = cov[0 * state_size + 0];  // X variance
  const double var_y = cov[1 * state_size + 1];  // Y variance
  
  // Check for invalid covariance values
  const double max_valid_variance = 1e6;
  if (var_x < 0.0 || var_y < 0.0 || 
      std::isnan(var_x) || std::isnan(var_y) || 
      std::isinf(var_x) || std::isinf(var_y) ||
      var_x > max_valid_variance || var_y > max_valid_variance) {
    if (safety_margin_mo_) { scene_manager_->destroyManualObject(safety_margin_mo_); safety_margin_mo_ = nullptr; }
    return;
  }
  
  // Standard deviations
  const float sigma_x = static_cast<float>(std::sqrt(var_x));
  const float sigma_y = static_cast<float>(std::sqrt(var_y));
  
  // Minimum visible threshold
  const float min_sigma = 0.01f;
  if (sigma_x < min_sigma && sigma_y < min_sigma) {
    if (safety_margin_mo_) { scene_manager_->destroyManualObject(safety_margin_mo_); safety_margin_mo_ = nullptr; }
    return;
  }
  
  // Create material if it doesn't exist
  if (!Ogre::MaterialManager::getSingleton().resourceExists(safety_margin_material_name_)) {
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
        safety_margin_material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    if (!mat.isNull()) {
      Ogre::Technique* tech = mat->getTechnique(0);
      if (!tech) tech = mat->createTechnique();
      Ogre::Pass* pass = tech->getPass(0);
      if (!pass) pass = tech->createPass();
      pass->setLightingEnabled(false);
      pass->setDepthCheckEnabled(true);
      pass->setDepthWriteEnabled(false);
      pass->setCullingMode(Ogre::CULL_NONE);
      pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    }
  }
  
  // Create manual object if needed
  if (!safety_margin_mo_) {
    safety_margin_mo_ = scene_manager_->createManualObject();
    safety_margin_mo_->setDynamic(true);
    scene_node_->attachObject(safety_margin_mo_);
  }
  
  safety_margin_mo_->clear();
  safety_margin_mo_->begin(safety_margin_material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  // Check if this is a pedestrian (use circle) or vehicle (use rectangle)
  const bool is_pedestrian = (classification_.type == perception_msgs::msg::ObjectClassification::PEDESTRIAN);
  const bool is_cyclist = (classification_.type == perception_msgs::msg::ObjectClassification::BICYCLE ||
                           classification_.type == perception_msgs::msg::ObjectClassification::MOTORBIKE);
  
  // Z position - place at the bottom of the bounding box, above hoverboard if present
  const float z_bottom = static_cast<float>(-0.5 * bbox_dims_.z);
  const float z_pos = z_bottom + (visualize_hoverboard_ ? hoverboard_thickness_ : 0.0f) + 0.03f;
  
  // Build sigma levels with their corresponding colors (draw 3σ first, then 2σ on top)
  std::vector<std::tuple<float, Ogre::ColourValue, bool>> sigma_levels;  // (multiplier, color, is_dotted)
  sigma_levels.push_back({3.0f, color_3sigma, true});   // 3-sigma with dotted outline
  sigma_levels.push_back({2.0f, color_2sigma, false});  // 2-sigma with dashed outline
  
  const float PI = static_cast<float>(Ogre::Math::PI);
  
  // Draw safety margins as expanded shapes
  for (const auto& level : sigma_levels) {
    const float sigma_mult = std::get<0>(level);
    const Ogre::ColourValue& margin_color = std::get<1>(level);
    const bool is_dotted = std::get<2>(level);
    
    // Expansion amount in each direction
    const float expand_x = sigma_x * sigma_mult;
    const float expand_y = sigma_y * sigma_mult;
    
    if (is_pedestrian || is_cyclist) {
      // Draw as circle/ellipse for pedestrians and cyclists
      const float base_radius = static_cast<float>(std::max(bbox_dims_.x, bbox_dims_.y) * 0.5);
      const float radius_x = base_radius + expand_x;
      const float radius_y = base_radius + expand_y;
      
      const int segments = 32;
      
      // Draw filled circle with gradient
      Ogre::ColourValue center_color = margin_color;
      center_color.a *= 0.25f;
      Ogre::ColourValue edge_color = margin_color;
      edge_color.a *= 0.5f;
      
      for (int i = 0; i < segments; ++i) {
        float angle0 = 2.0f * PI * static_cast<float>(i) / static_cast<float>(segments);
        float angle1 = 2.0f * PI * static_cast<float>(i + 1) / static_cast<float>(segments);
        
        float x0 = radius_x * std::cos(angle0);
        float y0 = radius_y * std::sin(angle0);
        float x1 = radius_x * std::cos(angle1);
        float y1 = radius_y * std::sin(angle1);
        
        // Triangle from center to edge
        safety_margin_mo_->position(0.0f, 0.0f, z_pos); safety_margin_mo_->colour(center_color);
        safety_margin_mo_->position(x0, y0, z_pos); safety_margin_mo_->colour(edge_color);
        safety_margin_mo_->position(x1, y1, z_pos); safety_margin_mo_->colour(edge_color);
      }
      
      // Draw dashed/dotted ring outline at the edge
      const float ring_width = 0.06f;
      const float inner_ratio = 1.0f - (ring_width / std::min(radius_x, radius_y));
      Ogre::ColourValue ring_color = margin_color;
      ring_color.a = std::min(1.0f, margin_color.a * 1.5f);
      
      // Dotted pattern uses more skips than dashed
      const int skip_pattern = is_dotted ? 3 : 2;
      
      for (int i = 0; i < segments; ++i) {
        // Skip segments for dashed/dotted effect
        if (i % skip_pattern != 1) continue;
        
        float angle0 = 2.0f * PI * static_cast<float>(i) / static_cast<float>(segments);
        float angle1 = 2.0f * PI * static_cast<float>(i + 1) / static_cast<float>(segments);
        
        float x0_out = radius_x * std::cos(angle0);
        float y0_out = radius_y * std::sin(angle0);
        float x1_out = radius_x * std::cos(angle1);
        float y1_out = radius_y * std::sin(angle1);
        float x0_in = radius_x * inner_ratio * std::cos(angle0);
        float y0_in = radius_y * inner_ratio * std::sin(angle0);
        float x1_in = radius_x * inner_ratio * std::cos(angle1);
        float y1_in = radius_y * inner_ratio * std::sin(angle1);
        
        const float z_ring = z_pos + (sigma_mult == 3.0f ? 0.003f : 0.006f);
        
        safety_margin_mo_->position(x0_in, y0_in, z_ring); safety_margin_mo_->colour(ring_color);
        safety_margin_mo_->position(x0_out, y0_out, z_ring); safety_margin_mo_->colour(ring_color);
        safety_margin_mo_->position(x1_out, y1_out, z_ring); safety_margin_mo_->colour(ring_color);
        
        safety_margin_mo_->position(x0_in, y0_in, z_ring); safety_margin_mo_->colour(ring_color);
        safety_margin_mo_->position(x1_out, y1_out, z_ring); safety_margin_mo_->colour(ring_color);
        safety_margin_mo_->position(x1_in, y1_in, z_ring); safety_margin_mo_->colour(ring_color);
      }
    } else {
      // Draw as expanded rectangle for vehicles
      const float half_length = static_cast<float>(bbox_dims_.x * 0.5) + expand_x;
      const float half_width = static_cast<float>(bbox_dims_.y * 0.5) + expand_y;
      
      // Corner radius for rounded rectangle
      const float corner_radius = std::min(0.35f, std::min(half_length, half_width) * 0.25f);
      const int corner_segments = 8;
      
      // Build rounded rectangle boundary
      std::vector<Ogre::Vector2> boundary;
      const float HALF_PI = PI * 0.5f;
      
      auto addArc = [&](float cx, float cy, float start_angle, float end_angle) {
        for (int i = 0; i <= corner_segments; ++i) {
          float t = static_cast<float>(i) / static_cast<float>(corner_segments);
          float angle = start_angle + t * (end_angle - start_angle);
          boundary.emplace_back(cx + corner_radius * std::cos(angle), 
                                cy + corner_radius * std::sin(angle));
        }
      };
      
      const float hx = half_length - corner_radius;
      const float hy = half_width - corner_radius;
      
      addArc(+hx, +hy, 0.0f, HALF_PI);           // front-right
      addArc(-hx, +hy, HALF_PI, PI);             // front-left
      addArc(-hx, -hy, PI, 1.5f * PI);           // rear-left
      addArc(+hx, -hy, 1.5f * PI, 2.0f * PI);    // rear-right
      
      // Draw filled shape with gradient
      Ogre::ColourValue center_color = margin_color;
      center_color.a *= 0.2f;
      Ogre::ColourValue edge_color = margin_color;
      edge_color.a *= 0.45f;
      
      for (size_t i = 0; i < boundary.size(); ++i) {
        const Ogre::Vector2& p0 = boundary[i];
        const Ogre::Vector2& p1 = boundary[(i + 1) % boundary.size()];
        
        safety_margin_mo_->position(0.0f, 0.0f, z_pos); safety_margin_mo_->colour(center_color);
        safety_margin_mo_->position(p0.x, p0.y, z_pos); safety_margin_mo_->colour(edge_color);
        safety_margin_mo_->position(p1.x, p1.y, z_pos); safety_margin_mo_->colour(edge_color);
      }
      
      // Draw dashed/dotted outline
      const float line_width = 0.05f;
      Ogre::ColourValue line_color = margin_color;
      line_color.a = std::min(1.0f, margin_color.a * 1.5f);
      
      // Dotted pattern uses more skips than dashed
      const int skip_pattern = is_dotted ? 3 : 2;
      
      for (size_t i = 0; i < boundary.size(); ++i) {
        // Skip segments for dashed/dotted effect
        if (static_cast<int>(i) % skip_pattern != 1) continue;
        
        const Ogre::Vector2& p0 = boundary[i];
        const Ogre::Vector2& p1 = boundary[(i + 1) % boundary.size()];
        
        // Calculate perpendicular for line width
        Ogre::Vector2 dir = p1 - p0;
        float len = dir.normalise();
        if (len < 0.001f) continue;
        
        Ogre::Vector2 perp(-dir.y * line_width, dir.x * line_width);
        
        Ogre::Vector2 v0 = p0 - perp;
        Ogre::Vector2 v1 = p0 + perp;
        Ogre::Vector2 v2 = p1 + perp;
        Ogre::Vector2 v3 = p1 - perp;
        
        const float z_line = z_pos + (sigma_mult == 3.0f ? 0.003f : 0.006f);
        
        safety_margin_mo_->position(v0.x, v0.y, z_line); safety_margin_mo_->colour(line_color);
        safety_margin_mo_->position(v1.x, v1.y, z_line); safety_margin_mo_->colour(line_color);
        safety_margin_mo_->position(v2.x, v2.y, z_line); safety_margin_mo_->colour(line_color);
        
        safety_margin_mo_->position(v0.x, v0.y, z_line); safety_margin_mo_->colour(line_color);
        safety_margin_mo_->position(v2.x, v2.y, z_line); safety_margin_mo_->colour(line_color);
        safety_margin_mo_->position(v3.x, v3.y, z_line); safety_margin_mo_->colour(line_color);
      }
    }
  }
  
  safety_margin_mo_->end();
}

}  // namespace rendering

}  // namespace perception_msgs
