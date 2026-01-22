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

#include <string>

#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/visibility_control.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

#include "perception_msgs/msg/object_state.hpp"
#include "perception_msgs_utils/object_access.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Ogre {
class Any;
class ColourValue;
class Quaternion;
class SceneManager;
class SceneNode;
class ManualObject;
}  // namespace Ogre

namespace perception_msgs {

namespace rendering {

class ObjectState {
 public:
  /**
   * @brief Construct a new Object State object
   *
   * @param classification_color_map mapping of object classification to Ogre::ColourValue to set different colours with reference to objec classification
   * @param text_color color to visualize the text
   * @param scene_manager
   * @param parent_node
   */
  ObjectState(const std::unordered_map<unsigned int, Ogre::ColourValue>& classification_color_map,
              const Ogre::ColourValue& text_color, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = 0);

  ~ObjectState();

  /**
   * @brief Set the parameters for this ObjectState
   *
   * @param state the ObjectState to set
   */
  void setObjectState(const perception_msgs::msg::ObjectState& state);

  /**
   * @brief Set the parameters for this ObjectStatePredictions
   *
   * @param state the ObjectStatePredictions to set
   */
  void setObjectStatePredictions(const std::vector<perception_msgs::msg::ObjectStatePrediction>& predictions);

  /**
   * @brief Override the alpha value for current ObjectState
   *
   * @param alpha
   */
  void overrideAlpha(const float& alpha);

  /**
   * @brief Override the colour value for current ObjectState
   *
   * @param colour
   */
  void overrideColour(const Ogre::ColourValue& colour);

  /**
   * @brief Set bool to indicate visualization of direction indicator object
   *
   * @param val
   */
  void setVisualizeDirectionIndicator(const bool& val);

  /**
   * @brief Set double to set Z component of the object state
   *
   * @param val
   */
  void setZComponent(const double& val);

  /**
   * @brief Set bool to indicate visualization of the bounding box
   *
   * @param val
   */
  void setVisualizeBoundingBox(const bool& val);

  /**
   * @brief Set bool to indicate visualization of the velocity
   *
   * @param val
   */
  void setVisualizeVelocity(const bool& val);

  /**
   * @brief Set scale for velocity arrow
   *
   * @param val
   */
  void setVelocityScale(const float& val);

  /**
   * @brief Set the height of the velocity arrow according to velocity
   * 
   * @param val 
   */
  void setVelocityHeight(const bool& val);

  /**
   * @brief Set wether to use velocity or box color
   *
   * @param val
   */
  void setUseVelocityColor(const bool& val);

  /**
   * @brief Set the colour for the velocity arrow
   *
   * @param colour
   */
  void setVelocityColor(const Ogre::ColourValue& colour);

  /**
   * @brief Set bool to indicate visualization of the acceleration
   *
   * @param val
   */
  void setVisualizeAcceleration(const bool& val);

  /**
   * @brief Set scale for acceleration arrow
   *
   * @param val
   */
  void setAccelerationScale(const float& val);

  /**
   * @brief Set wether to use acceleration or box color
   *
   * @param val
   */
  void setUseAccelerationColor(const bool& val);

  /**
   * @brief Set the colour for the acceleration arrow
   *
   * @param colour
   */
  void setAccelerationColor(const Ogre::ColourValue& colour);

  /**
   * @brief Set bool to indicate visualization of text
   *
   * @param val
   */
  void setVisualizeText(const bool& val);

  /**
   * @brief Set float to change character height of text
   *
   * @param val
   */
  void setCharHeight(const float& val);

  /**
   * @brief Set the visualization color of text with respect to objects classification color
   *
   * @param val
   */
  void setColorTextWithClass(const bool& val);

  /**
   * @brief Set the Object Id
   *
   * @param val
   */
  void setObjectId(const unsigned int& val);

  /**
   * @brief Set the Existance Prob object
   *
   * @param val
   */
  void setExistanceProb(const double& val);

  /**
   * @brief Set bool to visualize mesh
   *
   * @param val
   */
  void setVisualizeMesh(const bool& val);

  // Hoverboard visualization
  void setVisualizeHoverboard(const bool& val);
  void setHoverboardThickness(const float& val);
  void setHoverboardCornerRadius(const float& val);
  void setHoverboardGlow(const bool& val);
  void setHoverboardGlowParams(const float& height, const float& intensity);
  void setHoverboardCapStyle(int style);
  void setHoverboardCornerSegments(int segs);

  // Uncertainty visualization
  void setVisualizeXYUncertainty(const bool& val);
  void setXYUncertaintyColor(const Ogre::ColourValue& color);
  void setXYUncertaintyAlpha(const float& alpha);
  void setXYUncertaintyScale(const float& scale);
  void setXYUncertaintySegments(int segs);
  void setVisualizeYawUncertainty(const bool& val);
  void setYawUncertaintyColor(const Ogre::ColourValue& color);
  void setYawUncertaintyAlpha(const float& alpha);
  void setYawUncertaintyConeLength(const float& length);

  // Safety margin visualization
  void setVisualizeSafetyMargins(const bool& val);
  void setSafetyMarginColor(const Ogre::ColourValue& color);
  void setSafetyMarginColor2Sigma(const Ogre::ColourValue& color);
  void setSafetyMarginColor3Sigma(const Ogre::ColourValue& color);
  void setSafetyMarginAlpha(const float& alpha);
  void setSafetyMargin2Sigma(const bool& val);
  void setSafetyMargin3Sigma(const bool& val);

  /**
   * @brief Set the Bounding Box Dimensions explicitly (e.g. for EgoData where dimensions are not part of the EGO-State-Model)
   *
   * @param dims
   */
  void setBoundingBoxDimensions(const Ogre::Vector3& dims);

  /**
   * @brief
   *
   * @param val
   */
  void printObjectId(const bool& val);

  /**
   * @brief
   *
   * @param val
   */
  void printExistanceProb(const bool& val);

  /**
   * @brief
   *
   * @param val
   */
  void printVelocity(const bool& val);

  /**
   * @brief
   *
   * @param val
   */
  void printClass(const bool& val);

  /**
   * @brief Set bool to indicate visualization of the predictions
   *
   * @param val
   */
  void setVisualizePredictions(const bool& val);

  /**
   * @brief Set bool to indicate visualization of the prediction points
   *
   * @param val
   */
  void setVisualizePredictionPoints(const bool& val);

  /**
   * @brief Set the colour for the prediction lines
   *
   * @param color
   */
  void setPredictionLineColor(const Ogre::ColourValue& color);

  /**
   * @brief Set the colour for the prediction points
   *
   * @param color
   */
  void setPredictionPointColor(const Ogre::ColourValue& color);

  /**
   * @brief Set the width for the prediction lines
   *
   * @param width
   */
  void setPredictionLineWidth(const float& width);

  /**
   * @brief Set the width for the prediction points
   *
   * @param width
   */
  void setPredictionPointWidth(const float& width);

  /**
   * @brief Set bool to indicate visualization of the prediction probabilities
   *
   * @param val
   */
  void setVisualizePredictionProbabilities(const bool& val);

  /**
   * @brief Set float to change character height of prediction probabilities text
   *
   * @param val
   */
  void setPredictionProbCharHeight(const float& val);

  /**
   * @brief Set the text of the probability of an object state prediction
   *
   * @param probability
   * @param state
   */
  void setObjectPredictionProbabilityText(const double& probability, const perception_msgs::msg::ObjectState& state,  std::shared_ptr<rviz_rendering::MovableText>& text_prob);

 private:
  /**
   * @brief Set the Object State of an default object
   *
   * @param state
   * @param color
   */
  void setObjectStateVizDefault(const perception_msgs::msg::ObjectState& state, const Ogre::ColourValue& color,
                                const bool& viz_bb, const bool& indicate_direction);

  /**
   * @brief Set the Object Predictions of an default object
   *
   * @param state
   * @param color
   */
  void setObjectPredictionsVizDefault(const std::vector<perception_msgs::msg::ObjectState>& states, std::shared_ptr<rviz_rendering::BillboardLine>& billboard_line_prediction, std::vector<std::shared_ptr<rviz_rendering::Shape>>& bbox_prediction, const Ogre::ColourValue& line_color, const Ogre::ColourValue& point_color);

  /**
   * @brief Set the Object State for an object classified as CAR
   *
   * @param state
   * @param color
   */
  void setObjectStateVizCar(const perception_msgs::msg::ObjectState& state, const Ogre::ColourValue& color,
                            const bool& viz_bb, const bool& indicate_direction);

  /**
  * @brief Set the object state default text object
  *
  * @param state
  * @param color
  */
  void setObjectStateTextDefault(const perception_msgs::msg::ObjectState& state, const Ogre::ColourValue& color);

  /**
   * @brief Set the Scene Node Pose of state-object
   *
   * @param state
   */
  void setSceneNodePose(const perception_msgs::msg::ObjectState& state);

  /**
   * @brief Generate std::string from classification
   *
   * @param text
   */
  void classToText(const perception_msgs::msg::ObjectClassification& classification, std::string& text);

  /**
   * @brief Generate std::string from velocity magnitude
   *
   * @param state
   * @param text
   */
  void velocityToText(const perception_msgs::msg::ObjectState& state, std::string& text);

  /**
   * @brief Visualize XY position uncertainty as an ellipse
   *
   * @param state
   * @param color
   */
  void visualizeXYUncertaintyEllipse(const perception_msgs::msg::ObjectState& state, const Ogre::ColourValue& color);

  /**
   * @brief Visualize yaw angle uncertainty as a cone/wedge
   *
   * @param state
   * @param color
   */
  void visualizeYawUncertaintyCone(const perception_msgs::msg::ObjectState& state, const Ogre::ColourValue& color);

  /**
   * @brief Visualize safety margins as expanded bounding box or circle based on uncertainty
   *
   * @param state
   * @param color_2sigma Color for 2-sigma (95% confidence) margin
   * @param color_3sigma Color for 3-sigma (99.7% confidence) margin
   */
  void visualizeSafetyMargins(const perception_msgs::msg::ObjectState& state, 
                              const Ogre::ColourValue& color_2sigma,
                              const Ogre::ColourValue& color_3sigma);

  Ogre::SceneNode* scene_node_;
  Ogre::SceneNode* mesh_node_;
  Ogre::SceneManager* scene_manager_;

  std::shared_ptr<rviz_rendering::Shape> bbox_;
  std::shared_ptr<rviz_rendering::Shape> bbox_cone_;
  std::shared_ptr<rviz_rendering::Shape> bbox_mesh_;
  Ogre::ManualObject* hoverboard_mo_ = nullptr;
  Ogre::ManualObject* hoverboard_glow_mo_ = nullptr;
  std::shared_ptr<rviz_rendering::Arrow> vel_arrow_;
  std::shared_ptr<rviz_rendering::Arrow> acc_arrow_;
  std::shared_ptr<rviz_rendering::MovableText> text_;
  std::vector<std::shared_ptr<rviz_rendering::MovableText>> text_prob_vector_;
  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> billboard_line_predictions_;
  std::vector<std::vector<std::shared_ptr<rviz_rendering::Shape>>> bbox_predictions_;

  perception_msgs::msg::ObjectState object_state_;
  std::vector<perception_msgs::msg::ObjectStatePrediction> predictions_;
  perception_msgs::msg::ObjectClassification classification_;
  float char_height_ = 4.0;
  unsigned int id_;
  bool id_set_ = false;
  double existence_probability_ = -1.0;
  std::unordered_map<unsigned int, Ogre::ColourValue> classification_color_map_;
  Ogre::ColourValue text_color_;
  float alpha_override_ = -1.0;
  Ogre::ColourValue color_override_;
  bool b_colour_override_ = false;
  Ogre::Vector3 bbox_dims_;
  bool b_bbox_dims_set_ = false;
  bool indicate_direction_ = true;
  bool visualize_bounding_box_ = true;
  bool visualize_mesh_ = false;
  bool visualize_hoverboard_ = false;
  bool visualize_velocity_ = true;
  float velocity_scale_ = 1.0;
  bool velocity_height_ = false;
  bool use_velocity_color_ = true;
  Ogre::ColourValue velocity_color_;
  bool visualize_acceleration_ = true;
  float acceleration_scale_ = 1.0;
  bool use_acceleration_color_ = true;
  Ogre::ColourValue acceleration_color_;
  bool visualize_text_ = true;
  bool use_class_color_for_text_ = true;
  bool print_id_ = false;
  bool print_existance_prob_ = false;
  bool print_class_ = true;
  bool print_velocity_ = true;
  bool visualize_predictions_ = true;
  bool visualize_prediction_points_ = true;
  Ogre::ColourValue prediction_line_color_;
  Ogre::ColourValue prediction_point_color_;
  float prediction_line_width_ = 1.0;
  float prediction_point_width_ = 0.5;
  bool visualize_prediction_probabilities_ = true;
  float char_height_prediction_probs_ = 4.0;
  std::string text_probabilities_;
  std::string material;

  // Hoverboard params
  float hoverboard_thickness_ = 0.12f;
  float hoverboard_corner_radius_ = 0.35f;
  bool hoverboard_glow_ = true;
  float hoverboard_glow_height_ = 0.7f;
  float hoverboard_glow_intensity_ = 0.6f;
  int hoverboard_cap_style_ = 2; // 0=square,1=bevel,2=round
  int hoverboard_corner_segments_ = 12;
  std::string hoverboard_material_name_ = "ObjectHoverboard/Tile";
  std::string hoverboard_glow_material_name_ = "ObjectHoverboard/Glow";

  // Uncertainty visualization params
  bool visualize_xy_uncertainty_ = false;
  Ogre::ColourValue xy_uncertainty_color_ = Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f);
  float xy_uncertainty_alpha_ = 1.0f;
  float xy_uncertainty_scale_ = 16.0f;  // Scale factor for standard deviation to ellipse size
  int xy_uncertainty_segments_ = 16;
  bool visualize_yaw_uncertainty_ = false;
  Ogre::ColourValue yaw_uncertainty_color_ = Ogre::ColourValue(1.0f, 0.5f, 0.0f, 1.0f);
  float yaw_uncertainty_alpha_ = 1.0f;
  float yaw_uncertainty_cone_length_ = 3.0f;
  Ogre::ManualObject* xy_uncertainty_mo_ = nullptr;
  Ogre::ManualObject* yaw_uncertainty_mo_ = nullptr;
  Ogre::ManualObject* safety_margin_mo_ = nullptr;
  std::string xy_uncertainty_material_name_ = "ObjectUncertainty/XYEllipse";
  std::string yaw_uncertainty_material_name_ = "ObjectUncertainty/YawCone";
  std::string safety_margin_material_name_ = "ObjectUncertainty/SafetyMargin";

  // Safety margin params
  bool visualize_safety_margins_ = false;
  Ogre::ColourValue safety_margin_color_ = Ogre::ColourValue(1.0f, 0.3f, 0.0f, 0.6f);
  Ogre::ColourValue safety_margin_color_2sigma_ = Ogre::ColourValue(1.0f, 0.78f, 0.0f, 0.8f);  // Yellow-orange
  Ogre::ColourValue safety_margin_color_3sigma_ = Ogre::ColourValue(1.0f, 0.31f, 0.0f, 0.8f);  // Red-orange
  float safety_margin_alpha_ = 0.6f;
  bool safety_margin_2sigma_ = true;
  bool safety_margin_3sigma_ = false;

  const double kFixedMeshHeightCar = 1.5;
  const double kFixedMeshHeightUtility = 4.0;
  const double kFixedMeshHeightBus = 4.0;
  const double kFixedMeshHeightBicycle = 1.1;
  const double kFixedMeshHeightMotorcycle = 1.3;
  const double kFixedMeshHeightPedestrian = 1.8;
};

}  // namespace rendering

}  // namespace perception_msgs
