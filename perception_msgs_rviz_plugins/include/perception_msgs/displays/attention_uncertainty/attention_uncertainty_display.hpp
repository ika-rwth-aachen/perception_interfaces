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
 * @brief Futuristic overlay showing mean perception certainty as dual vertical bar charts.
 * 
 * Shows both classification certainty (from classification probabilities) and
 * regression certainty (from state covariances) side by side.
 */
class AttentionUncertaintyDisplay
  : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  AttentionUncertaintyDisplay();
  ~AttentionUncertaintyDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;

private slots:
  void updatePosition();
  void updateSize();
  void updateTransparency();
  void updateThresholds();
  void updateSmoothing();
  void updateTitle();

private:
  void createHUDOverlay();
  void destroyHUDOverlay();
  void updateHUD();
  double computeClassificationCertainty(const perception_msgs::msg::ObjectList& objects) const;
  double computeRegressionCertainty(const perception_msgs::msg::ObjectList& objects) const;
  QColor barColorForCertainty(double certainty, bool blink_on) const;

  rviz_common::properties::IntProperty* hud_width_property_;
  rviz_common::properties::IntProperty* hud_height_property_;
  rviz_common::properties::IntProperty* hud_left_property_;
  rviz_common::properties::IntProperty* hud_top_property_;
  rviz_common::properties::FloatProperty* hud_alpha_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;
  rviz_common::properties::FloatProperty* high_threshold_property_;
  rviz_common::properties::FloatProperty* low_threshold_property_;
  rviz_common::properties::FloatProperty* blink_threshold_property_;
  rviz_common::properties::FloatProperty* blink_frequency_property_;
  rviz_common::properties::FloatProperty* smoothing_alpha_property_;
  rviz_common::properties::ColorProperty* high_color_property_;
  rviz_common::properties::ColorProperty* mid_color_property_;
  rviz_common::properties::ColorProperty* low_color_property_;
  rviz_common::properties::StringProperty* title_text_property_;
  rviz_common::properties::ColorProperty* regression_high_color_property_;
  rviz_common::properties::ColorProperty* regression_mid_color_property_;
  rviz_common::properties::ColorProperty* regression_low_color_property_;
  rviz_common::properties::FloatProperty* max_variance_property_;

  int hud_width_;
  int hud_height_;
  int hud_left_;
  int hud_top_;
  float hud_alpha_;
  float bg_alpha_;
  float high_threshold_;
  float low_threshold_;
  float blink_threshold_;
  float blink_frequency_;
  float smoothing_alpha_;
  QColor high_color_;
  QColor mid_color_;
  QColor low_color_;
  QString title_text_;
  QColor regression_high_color_;
  QColor regression_mid_color_;
  QColor regression_low_color_;
  float max_variance_;

  double smoothed_classification_certainty_;
  double smoothed_regression_certainty_;
  bool have_classification_certainty_;
  bool have_regression_certainty_;
  bool update_required_;
  bool blink_state_;
  double blink_timer_;
  double no_data_timer_;  // Timer for detecting no incoming data
  static constexpr double kNoDataTimeout = 2.0;  // Seconds before showing "no data" warning
  static constexpr double kSuspiciouslyLowThreshold = 0.15;  // 15% - suspiciously low certainty

  Ogre::Overlay* overlay_;
  Ogre::PanelOverlayElement* panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;

  std::mutex hud_mutex_;

  static constexpr int kDefaultWidth = 180;
  static constexpr int kDefaultHeight = 220;
};

}  // namespace displays
}  // namespace perception_msgs
