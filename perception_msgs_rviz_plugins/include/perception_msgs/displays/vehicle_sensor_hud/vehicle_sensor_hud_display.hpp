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
#include <map>

#include "perception_msgs/msg/object_list.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"

#include <rclcpp/rclcpp.hpp>

#include <QColor>
#include <QString>

#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgrePanelOverlayElement.h>

namespace perception_msgs {
namespace displays {

/**
 * @brief Vehicle Sensor Status HUD with 3D wireframe van and health monitoring.
 * 
 * Features:
 * - Isometric 3D wireframe van visualization
 * - Animated data flow lines from LiDARs to vehicle center
 * - Green flowing lines = healthy, Red/broken = unhealthy
 * - Overall system health status
 * - Classification and regression certainty display
 */
class VehicleSensorHudDisplay
  : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  VehicleSensorHudDisplay();
  ~VehicleSensorHudDisplay() override;

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
  void updateLidarTopics();

private:
  void createOverlay();
  void destroyOverlay();
  void updateHud();
  void drawHud(QPainter& painter);
  void drawWireframeVan(QPainter& painter, const QRectF& bounds, 
                        double cls_certainty, double reg_certainty, bool has_data);
  void drawDataFlowLine(QPainter& painter, const QPointF& from, const QPointF& to,
                        bool healthy, double phase, double pulse);
  void drawHealthIndicator(QPainter& painter, const QPointF& pos, 
                           const QString& label, bool active);
  void drawStatusPanel(QPainter& painter, const QRectF& bounds);
  double computeOverallCertainty() const;
  
  // LiDAR callbacks
  void lidarCallback(const std::string& sensor_id, 
                     sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void createLidarSubscriptions();
  void destroyLidarSubscriptions();

  // Properties
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::IntProperty* left_property_;
  rviz_common::properties::IntProperty* top_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;
  rviz_common::properties::StringProperty* title_property_;
  
  // LiDAR topics
  rviz_common::properties::StringProperty* lidar_fr_topic_property_;
  rviz_common::properties::StringProperty* lidar_fl_topic_property_;
  rviz_common::properties::StringProperty* lidar_rr_topic_property_;
  rviz_common::properties::StringProperty* lidar_rl_topic_property_;
  
  // Colors
  rviz_common::properties::ColorProperty* healthy_color_property_;
  rviz_common::properties::ColorProperty* warning_color_property_;
  rviz_common::properties::ColorProperty* unhealthy_color_property_;
  rviz_common::properties::ColorProperty* vehicle_color_property_;
  
  // Thresholds
  rviz_common::properties::FloatProperty* high_threshold_property_;
  rviz_common::properties::FloatProperty* low_threshold_property_;
  rviz_common::properties::FloatProperty* max_variance_property_;
  rviz_common::properties::FloatProperty* lidar_timeout_property_;

  // State
  int width_, height_, left_, top_;
  float alpha_, bg_alpha_;
  QString title_;
  QColor healthy_color_, warning_color_, unhealthy_color_, vehicle_color_;
  float high_threshold_, low_threshold_, max_variance_, lidar_timeout_;

  // Perception data
  int object_count_;
  double smoothed_classification_, smoothed_regression_;
  bool has_objectlist_data_;
  double objectlist_last_update_;
  
  // LiDAR status
  struct LidarStatus {
    bool active = false;
    double last_msg_time = 0.0;
  };
  std::map<std::string, LidarStatus> lidar_status_;
  
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_fr_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_fl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_rr_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_rl_sub_;
  
  // Animation
  double flow_phase_;
  double pulse_phase_;
  double rotation_phase_;  // For rotating the 3D van mesh
  
  // Overlay
  std::string overlay_name_, panel_name_, material_name_, texture_name_;
  Ogre::Overlay* overlay_;
  Ogre::PanelOverlayElement* panel_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  
  std::mutex data_mutex_;
  bool update_required_;
};

}  // namespace displays
}  // namespace perception_msgs
