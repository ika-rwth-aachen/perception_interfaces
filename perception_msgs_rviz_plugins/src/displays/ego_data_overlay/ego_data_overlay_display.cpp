#include "perception_msgs/displays/ego_data_overlay/ego_data_overlay_display.hpp"
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTexture.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>
#include <rviz_rendering/render_system.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QPainter>
#include <QFontMetrics>

namespace perception_msgs
{
namespace displays
{

EgoDataOverlay::EgoDataOverlay()
  : width_(350)
  , height_(80)
  , left_(10)
  , top_(10)
  , bg_color_(20, 20, 20)
  , bg_alpha_(0.9)
  , velocity_(0.0)
  , steering_angle_(0.0)
  , standstill_(false)
  , turn_signal_left_(false)
  , turn_signal_right_(false)
  , update_required_(false)
{
  // Properties for RViz UI
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", width_, "Width of the overlay", this, SLOT(updateWidth()));
  width_property_->setMin(50);
  
  height_property_ = new rviz_common::properties::IntProperty(
    "Height", height_, "Height of the overlay", this, SLOT(updateHeight()));
  height_property_->setMin(50);
  
  left_property_ = new rviz_common::properties::IntProperty(
    "Left", left_, "Left position of the overlay", this, SLOT(updateLeft()));
  left_property_->setMin(0);
  
  top_property_ = new rviz_common::properties::IntProperty(
    "Top", top_, "Top position of the overlay", this, SLOT(updateTop()));
  top_property_->setMin(0);
  
  bg_color_property_ = new rviz_common::properties::ColorProperty(
    "Background Color", bg_color_, "Background color", this, SLOT(updateBackgroundColor()));
  
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", bg_alpha_, "Background transparency", this, SLOT(updateBackgroundAlpha()));
  bg_alpha_property_->setMin(0.0);
  bg_alpha_property_->setMax(1.0);
}

void EgoDataOverlay::onInitialize()
{
  rviz_common::RosTopicDisplay<perception_msgs::msg::EgoData>::onInitialize();
  
  // Initialize overlay system
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  
  // Create overlay object
  static int count = 0;
  std::string overlay_name = "EgoDataOverlayDisplayOverlay" + std::to_string(count++);
  overlay_ = std::make_shared<rviz_2d_overlay_plugins::OverlayObject>(overlay_name);
  
  overlay_->updateTextureSize(width_, height_);
  overlay_->setDimensions(width_, height_);
  overlay_->setPosition(left_, top_);
  
  // Load icon images (place PNGs in assets/ folder)
  QString package_path = QString::fromStdString(ament_index_cpp::get_package_share_directory("perception_msgs_rviz_plugins"));
  icon_steering_wheel_.load(package_path + "/assets/steering_wheel.png");
  icon_turn_signal_left_on_.load(package_path + "/assets/turn_signal_left_on.png");
  icon_turn_signal_left_off_.load(package_path + "/assets/turn_signal_left_off.png");
  icon_turn_signal_right_on_.load(package_path + "/assets/turn_signal_right_on.png");
  icon_turn_signal_right_off_.load(package_path + "/assets/turn_signal_right_off.png");
  
  // Initial render
  update_required_ = true;
}

void EgoDataOverlay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
  update_required_ = true;
}

void EgoDataOverlay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
}

void EgoDataOverlay::processMessage(perception_msgs::msg::EgoData::ConstSharedPtr msg)
{
  // Extract values from EgoData
  const auto& state = msg->state;
  
  // VEL_LON at index 3, convert m/s to km/h
  if (state.continuous_state.size() > perception_msgs::msg::EGO::VEL_LON) {
    velocity_ = state.continuous_state[perception_msgs::msg::EGO::VEL_LON] * 3.6;
  }
  
  // STEERING_ANGLE_ACK at index 11, convert rad to degrees
  if (state.continuous_state.size() > perception_msgs::msg::EGO::STEERING_ANGLE_ACK) {
    steering_angle_ = state.continuous_state[perception_msgs::msg::EGO::STEERING_ANGLE_ACK] * 180.0 / M_PI;
  }
  
  // STANDSTILL at index 0 in discrete_state
  if (state.discrete_state.size() > perception_msgs::msg::EGO::STANDSTILL) {
    standstill_ = state.discrete_state[perception_msgs::msg::EGO::STANDSTILL] != 0;
  }
  
  // TURN_INDICATOR at index 1 in discrete_state - always reset first
  turn_signal_left_ = false;
  turn_signal_right_ = false;
  if (state.discrete_state.size() > perception_msgs::msg::EGO::TURN_INDICATOR) {
    int turn_indicator = state.discrete_state[perception_msgs::msg::EGO::TURN_INDICATOR];
    turn_signal_left_ = (turn_indicator == perception_msgs::msg::EGO::TURN_INDICATOR_LEFT || 
                         turn_indicator == perception_msgs::msg::EGO::TURN_INDICATOR_HAZARD);
    turn_signal_right_ = (turn_indicator == perception_msgs::msg::EGO::TURN_INDICATOR_RIGHT || 
                          turn_indicator == perception_msgs::msg::EGO::TURN_INDICATOR_HAZARD);
  }

  update_required_ = true;
}

void EgoDataOverlay::update(float wall_dt, float ros_dt)
{
  rviz_common::RosTopicDisplay<perception_msgs::msg::EgoData>::update(wall_dt, ros_dt);
  
  if (update_required_) {
    renderOverlay();
    update_required_ = false;
  }
}

void EgoDataOverlay::renderOverlay()
{
  if (!overlay_ || !overlay_->isVisible()) {
    return;
  }
  
  overlay_->updateTextureSize(width_, height_);
  
  rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage image = buffer.getQImage(width_, height_);
  image.fill(Qt::transparent);
  
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  
  // Background with rounded corners
  QColor bg_with_alpha = bg_color_;
  bg_with_alpha.setAlphaF(bg_alpha_);
  painter.setPen(QPen(QColor(80, 80, 80), 1));
  painter.setBrush(bg_with_alpha);
  painter.drawRoundedRect(0, 0, width_, height_, 40, 40);
  
  // Layout-Parameter
  int circle_size = 50;
  int center_y = height_ / 2;
  int spacing = 12;
  
  // Calculate total content width
  int total_content_width = circle_size + spacing +  // Park
                            circle_size + spacing +  // Left turn_signal
                            50 + spacing +           // Velocity
                            circle_size + spacing +  // Right turn_signal
                            35;                      // Steering
  
  // Center the content horizontally
  int current_x = (width_ - total_content_width) / 2;
  
  QFont speedFont("Arial", 16, QFont::Bold);
  QFont unitFont("Arial", 9);
  QFont smallFont("Arial", 11);
  
  // SECTION 1: Park symbol
  QColor park_color = standstill_ ? QColor(255, 100, 100) : QColor(60, 60, 60);
  painter.setPen(QPen(QColor(150, 150, 150), 2));
  painter.setBrush(park_color);
  painter.drawEllipse(current_x, center_y - circle_size/2, circle_size, circle_size);
  painter.setFont(speedFont);
  painter.setPen(QColor(220, 220, 220));
  painter.drawText(QRect(current_x, center_y - circle_size/2, circle_size, circle_size), 
                   Qt::AlignCenter, "P");
  current_x += circle_size + spacing;
  
  // SECTION 3: Left turn_signal icon
  QPixmap& turn_signal_left_icon = turn_signal_left_ ? icon_turn_signal_left_on_ : icon_turn_signal_left_off_;
  QPixmap scaled_left = turn_signal_left_icon.scaled(circle_size, circle_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(current_x, center_y - circle_size/2, scaled_left);
  current_x += circle_size + spacing;
  
  // Section 4: Velocity
  painter.setFont(speedFont);
  painter.setPen(QColor(220, 220, 220));
  painter.drawText(QRect(current_x, center_y - 22, 50, 24), 
                   Qt::AlignCenter, QString::number(static_cast<int>(velocity_)));
  painter.setFont(unitFont);
  painter.setPen(QColor(150, 150, 150));
  painter.drawText(QRect(current_x, center_y + 4, 50, 18), 
                   Qt::AlignCenter, "km/h");
  current_x += 50 + spacing;
  
  // Section 5: Right turn_signal icon
  QPixmap& turn_signal_right_icon = turn_signal_right_ ? icon_turn_signal_right_on_ : icon_turn_signal_right_off_;
  QPixmap scaled_right = turn_signal_right_icon.scaled(circle_size, circle_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(current_x, center_y - circle_size/2, scaled_right);
  current_x += circle_size + spacing;
  
  // SECTION 6: Steering wheel angle
  int steering_size = 35;
  int steering_start = current_x;
  
  // Draw rotated steering wheel
  QPixmap scaled_steering = icon_steering_wheel_.scaled(steering_size, steering_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  QTransform transform;
  transform.translate(steering_size/2.0, steering_size/2.0);
  transform.rotate(steering_angle_);
  transform.translate(-steering_size/2.0, -steering_size/2.0);
  QPixmap rotated = scaled_steering.transformed(transform, Qt::SmoothTransformation);
  
  int offset_x = (rotated.width() - steering_size) / 2;
  int offset_y = (rotated.height() - steering_size) / 2;
  painter.drawPixmap(steering_start - offset_x, center_y - steering_size/2 - offset_y - 8, rotated);
  
  // Draw angle text below the steering wheel
  painter.setFont(QFont("Arial", 12, QFont::Bold));
  painter.setPen(QColor(220, 220, 220));
  painter.drawText(QRect(steering_start, center_y + steering_size/2 - 6, steering_size, 18), 
                   Qt::AlignCenter, 
                   QString::number(static_cast<int>(steering_angle_)) + "°");
  
  painter.end();
}

// Property Update Slots
void EgoDataOverlay::updateWidth()
{
  width_ = width_property_->getInt();
  if (overlay_) {
    overlay_->setDimensions(width_, height_);
  }
  update_required_ = true;
}

void EgoDataOverlay::updateHeight()
{
  height_ = height_property_->getInt();
  if (overlay_) {
    overlay_->setDimensions(width_, height_);
  }
  update_required_ = true;
}

void EgoDataOverlay::updateLeft()
{
  left_ = left_property_->getInt();
  if (overlay_) {
    overlay_->setPosition(left_, top_);
  }
  update_required_ = true;
}

void EgoDataOverlay::updateTop()
{
  top_ = top_property_->getInt();
  if (overlay_) {
    overlay_->setPosition(left_, top_);
  }
  update_required_ = true;
}

void EgoDataOverlay::updateBackgroundColor()
{
  bg_color_ = bg_color_property_->getColor();
  update_required_ = true;
}

void EgoDataOverlay::updateBackgroundAlpha()
{
  bg_alpha_ = bg_alpha_property_->getFloat();
  update_required_ = true;
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::EgoDataOverlay, rviz_common::Display)
