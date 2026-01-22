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

#include <deque>
#include <mutex>

#include "perception_msgs/msg/object_list.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"

#include <QColor>
#include <QString>

#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgrePanelOverlayElement.h>

namespace perception_msgs {
namespace displays {

/**
 * @brief Futuristic temporal uncertainty graph overlay showing classification and regression
 *        certainty history over time as animated line charts.
 * 
 * Displays a time-series graph with:
 * - Classification certainty (derived from classification probabilities)
 * - Regression certainty (derived from state covariances)
 * - Threshold indicators and grid lines
 * - Futuristic styling with gradient fills and glow effects
 */
class UncertaintyGraphDisplay
  : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  UncertaintyGraphDisplay();
  ~UncertaintyGraphDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updatePosition();
  void updateSize();
  void updateAppearance();

private:
  void createOverlay();
  void destroyOverlay();
  void updateGraph();
  void drawGraph(QPainter& painter);
  double computeClassificationCertainty(const perception_msgs::msg::ObjectList& objects) const;
  double computeRegressionCertainty(const perception_msgs::msg::ObjectList& objects) const;

  // Position and size properties
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::IntProperty* left_property_;
  rviz_common::properties::IntProperty* top_property_;
  
  // Appearance properties
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;
  rviz_common::properties::IntProperty* history_length_property_;
  rviz_common::properties::FloatProperty* smoothing_property_;
  rviz_common::properties::StringProperty* title_property_;
  
  // Line colors
  rviz_common::properties::ColorProperty* class_color_property_;
  rviz_common::properties::ColorProperty* regr_color_property_;
  
  // Threshold properties for regression certainty calculation
  rviz_common::properties::FloatProperty* max_variance_property_;
  
  // Threshold line properties
  rviz_common::properties::BoolProperty* show_thresholds_property_;
  rviz_common::properties::FloatProperty* high_threshold_property_;
  rviz_common::properties::FloatProperty* low_threshold_property_;
  rviz_common::properties::ColorProperty* high_threshold_color_property_;
  rviz_common::properties::ColorProperty* low_threshold_color_property_;

  // Cached values
  int width_;
  int height_;
  int left_;
  int top_;
  float alpha_;
  float bg_alpha_;
  int history_length_;
  float smoothing_;
  QString title_;
  QColor class_color_;
  QColor regr_color_;
  float max_variance_;
  bool show_thresholds_;
  float high_threshold_;
  float low_threshold_;
  QColor high_threshold_color_;
  QColor low_threshold_color_;

  // State
  double smoothed_classification_;
  double smoothed_regression_;
  bool update_required_;
  float animation_time_;

  // History buffers
  std::deque<double> classification_history_;
  std::deque<double> regression_history_;
  std::mutex history_mutex_;

  // Ogre overlay
  Ogre::Overlay* overlay_;
  Ogre::PanelOverlayElement* panel_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  std::string overlay_name_;
  std::string panel_name_;
  std::string texture_name_;
  std::string material_name_;

  static constexpr int kDefaultWidth = 280;
  static constexpr int kDefaultHeight = 120;
  static constexpr int kDefaultHistoryLength = 60;
  static constexpr float kDefaultSmoothing = 0.85f;
};

}  // namespace displays
}  // namespace perception_msgs
