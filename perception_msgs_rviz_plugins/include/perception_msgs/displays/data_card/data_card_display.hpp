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
 * @brief Data Card overlay displaying dataset information for Trustworthy AI.
 * 
 * Shows training dataset statistics including class distributions,
 * LiDAR points per sample, pretraining information, and GDPR compliance.
 * Features the same futuristic visual style as ModelCard.
 */
class DataCardDisplay : public rviz_common::Display {
  Q_OBJECT

public:
  DataCardDisplay();
  ~DataCardDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void updatePosition();
  void updateSize();
  void updateAppearance();
  void updateDatasetInfo();

private:
  void createOverlay();
  void destroyOverlay();
  void updateHUD();
  void drawHUD(class QPainter& painter);
  void drawDatasetIcon(class QPainter& painter, int x, int y, int size);
  void drawClassBar(class QPainter& painter, int x, int y, int width, int height,
                    const QString& label, int count, int max_count, const QColor& color);

  // Properties - Position & Size
  rviz_common::properties::IntProperty* left_property_;
  rviz_common::properties::IntProperty* top_property_;
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;

  // Properties - Dataset Information
  rviz_common::properties::StringProperty* dataset_name_property_;
  rviz_common::properties::StringProperty* dataset_version_property_;
  rviz_common::properties::StringProperty* pretrain_dataset_property_;
  rviz_common::properties::StringProperty* points_per_sample_property_;
  rviz_common::properties::IntProperty* car_count_property_;
  rviz_common::properties::IntProperty* pedestrian_count_property_;
  rviz_common::properties::IntProperty* truck_count_property_;
  rviz_common::properties::IntProperty* trailer_count_property_;
  rviz_common::properties::IntProperty* bus_count_property_;
  rviz_common::properties::IntProperty* twowheeler_count_property_;

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
  
  // Dataset info cache
  QString dataset_name_;
  QString dataset_version_;
  QString pretrain_dataset_;
  QString points_per_sample_;
  int car_count_;
  int pedestrian_count_;
  int truck_count_;
  int trailer_count_;
  int bus_count_;
  int twowheeler_count_;

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
