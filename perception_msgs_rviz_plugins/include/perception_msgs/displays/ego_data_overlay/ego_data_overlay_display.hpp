#ifndef PERCEPTION_MSGS__DISPLAYS__EGO_DATA_OVERLAY__EGO_DATA_OVERLAY_DISPLAY_HPP_
#define PERCEPTION_MSGS__DISPLAYS__EGO_DATA_OVERLAY__EGO_DATA_OVERLAY_DISPLAY_HPP_

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/ego.hpp>
#include <QImage>
#include <QPainter>
#include <QColor>
#include <QFont>
#include <QPixmap>

namespace perception_msgs
{
namespace displays
{

class EgoDataOverlay 
  : public rviz_common::RosTopicDisplay<perception_msgs::msg::EgoData>
{
  Q_OBJECT

public:
  EgoDataOverlay();
  ~EgoDataOverlay() override = default;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void processMessage(perception_msgs::msg::EgoData::ConstSharedPtr msg) override;
  void update(float wall_dt, float ros_dt) override;
  
  // Custom rendering function with full control over overlay appearance
  void renderOverlay();

protected Q_SLOTS:
  // Slots für Property-Updates
  void updateWidth();
  void updateHeight();
  void updateLeft();
  void updateBottom();
  void updateBackgroundColor();
  void updateBackgroundAlpha();

private:
  // Overlay object for direct rendering
  rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;
  
  // Properties for UI configuration
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::IntProperty* left_property_;
  rviz_common::properties::IntProperty* bottom_property_;
  rviz_common::properties::ColorProperty* bg_color_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;
  
  // Rendering parameters
  int width_;
  int height_;
  int left_;
  int bottom_;
  QColor bg_color_;
  float bg_alpha_;
  
  // Vehicle state data from EgoData topic
  double velocity_;           // km/h
  double steering_angle_;     // degrees
  bool standstill_;
  bool turn_signal_left_;
  bool turn_signal_right_;
  bool update_required_;
  
  // Icon images
  QPixmap icon_steering_wheel_;
  QPixmap icon_turn_signal_left_on_;
  QPixmap icon_turn_signal_left_off_;
  QPixmap icon_turn_signal_right_on_;
  QPixmap icon_turn_signal_right_off_;
};

}  // namespace displays
}  // namespace perception_msgs

#endif  // PERCEPTION_MSGS__DISPLAYS__EGO_DATA_OVERLAY__EGO_DATA_OVERLAY_DISPLAY_HPP_
