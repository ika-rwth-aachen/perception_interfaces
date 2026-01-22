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

#include "rviz_common/display.hpp"
#include "rviz_common/properties/bool_property.hpp"
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
 * @brief Model Card overlay displaying essential model information in a futuristic panel.
 * 
 * Shows key model facts like name, version, training dataset, accuracy metrics,
 * model size, and deployment date. Features an expandable design with a neural
 * network icon and smooth animations.
 */
class ModelCardDisplay : public rviz_common::Display {
  Q_OBJECT

public:
  ModelCardDisplay();
  ~ModelCardDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void updatePosition();
  void updateSize();
  void updateAppearance();
  void updateModelInfo();

private:
  void createOverlay();
  void destroyOverlay();
  void updateHUD();
  void drawHUD(class QPainter& painter);
  void drawNeuralNetIcon(class QPainter& painter, int x, int y, int size);

  // Properties - Position & Size
  rviz_common::properties::IntProperty* left_property_;
  rviz_common::properties::IntProperty* top_property_;
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;

  // Properties - Model Information
  rviz_common::properties::StringProperty* model_name_property_;
  rviz_common::properties::StringProperty* model_version_property_;
  rviz_common::properties::StringProperty* model_date_property_;
  rviz_common::properties::StringProperty* training_dataset_property_;
  rviz_common::properties::StringProperty* model_size_property_;
  rviz_common::properties::StringProperty* accuracy_property_;
  rviz_common::properties::StringProperty* inference_time_property_;
  rviz_common::properties::StringProperty* framework_property_;
  rviz_common::properties::StringProperty* input_shape_property_;
  rviz_common::properties::StringProperty* output_classes_property_;

  // Display state
  int left_;
  int top_;
  int width_;
  int height_;
  float alpha_;
  float bg_alpha_;
  
  // Animation
  double animation_phase_;
  double pulse_phase_;
  
  // Model info cache
  QString model_name_;
  QString model_version_;
  QString model_date_;
  QString training_dataset_;
  QString model_size_;
  QString accuracy_;
  QString inference_time_;
  QString framework_;
  QString input_shape_;
  QString output_classes_;

  // Overlay resources
  Ogre::Overlay* overlay_;
  Ogre::PanelOverlayElement* panel_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;

  std::string overlay_name_;
  std::string panel_name_;
  std::string material_name_;
  std::string texture_name_;

  std::mutex hud_mutex_;
  bool update_required_;

  // Constants
  static constexpr int kDefaultHeight = 365;
  static constexpr int kDefaultWidth = 280;
};

}  // namespace displays
}  // namespace perception_msgs
