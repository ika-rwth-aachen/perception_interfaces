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

#include "perception_msgs/displays/attention_hud/attention_hud_display.hpp"

#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreHardwarePixelBuffer.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/render_system.hpp"

#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace perception_msgs {
namespace displays {

AttentionHUDDisplay::AttentionHUDDisplay()
: update_required_(false), first_time_(true), hud_size_(DEFAULT_HUD_SIZE),
  hud_left_(100), hud_top_(100), hud_alpha_(0.7f), bg_alpha_(0.1f), 
  max_range_(50.0f), animation_speed_(DEFAULT_ANIMATION_SPEED),
  scene_manager_(nullptr), overlay_(nullptr), panel_(nullptr)
{
  // Initialize sector data
  for (auto& sector : sector_data_) {
    sector.attention_level = 0.0f;
    sector.object_count = 0;
    sector.target_opacity = 0.0f;
    sector.current_opacity = 0.0f;
    sector.has_objects = false;
  }

  // Create properties
  hud_size_property_ = new rviz_common::properties::IntProperty(
    "HUD Size", DEFAULT_HUD_SIZE,
    "Size of the HUD overlay in pixels", this, SLOT(updateSize()));
  hud_size_property_->setMin(400);
  hud_size_property_->setMax(1200);

  hud_left_property_ = new rviz_common::properties::IntProperty(
    "Left", 100, "Left position of the HUD overlay", this, SLOT(updatePosition()));

  hud_top_property_ = new rviz_common::properties::IntProperty(
    "Top", 100, "Top position of the HUD overlay", this, SLOT(updatePosition()));

  hud_color_property_ = new rviz_common::properties::ColorProperty(
    "HUD Color", QColor(0, 255, 255), // Cyan
    "Color of the HUD elements", this, SLOT(updateColors()));

  hud_alpha_property_ = new rviz_common::properties::FloatProperty(
    "HUD Alpha", 0.7f,
    "Maximum transparency of HUD elements (0.0 = transparent, 1.0 = opaque)", 
    this, SLOT(updateTransparency()));
  hud_alpha_property_->setMin(0.0f);
  hud_alpha_property_->setMax(1.0f);

  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", 0.1f,
    "Transparency of HUD background (0.0 = transparent, 1.0 = opaque)", 
    this, SLOT(updateTransparency()));
  bg_alpha_property_->setMin(0.0f);
  bg_alpha_property_->setMax(1.0f);

  max_range_property_ = new rviz_common::properties::FloatProperty(
    "Max Range", 50.0f,
    "Maximum range for attention analysis (meters)", this, SLOT(updateHUDProperties()));
  max_range_property_->setMin(1.0f);
  max_range_property_->setMax(200.0f);

  animation_speed_property_ = new rviz_common::properties::FloatProperty(
    "Animation Speed", DEFAULT_ANIMATION_SPEED,
    "Speed of opacity animations", this, SLOT(updateAnimationSpeed()));
  animation_speed_property_->setMin(0.1f);
  animation_speed_property_->setMax(10.0f);

  ego_frame_property_ = new rviz_common::properties::StringProperty(
    "Ego Frame", "base_link",
    "Frame ID representing the ego vehicle position", this, SLOT(updateHUDProperties()));

  // Set default HUD color to cyan
  hud_color_ = QColor(0, 255, 255);
}

AttentionHUDDisplay::~AttentionHUDDisplay()
{
  // Clean up overlay resources
  if (overlay_ && overlay_->isVisible()) {
    overlay_->hide();
  }
  
  Ogre::OverlayManager* overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  if (overlay_mgr) {
    if (panel_) {
      overlay_mgr->destroyOverlayElement(panel_);
      panel_ = nullptr;
    }
    if (overlay_) {
      overlay_mgr->destroy(overlay_);
      overlay_ = nullptr;
    }
  }
  
  if (panel_material_) {
    panel_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
  }
  
  delete hud_size_property_;
  delete hud_left_property_;
  delete hud_top_property_;
  delete hud_color_property_;
  delete hud_alpha_property_;
  delete bg_alpha_property_;
  delete max_range_property_;
  delete animation_speed_property_;
  delete ego_frame_property_;
}

void AttentionHUDDisplay::onInitialize()
{
  MFDClass::onInitialize();
  
  scene_manager_ = context_->getSceneManager();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  
  createHUDOverlay();
  
  // Initialize property values
  updateSize();
  updatePosition();
  updateColors();
  updateTransparency();
  updateAnimationSpeed();
}

void AttentionHUDDisplay::onEnable()
{
  MFDClass::onEnable();
  
  if (overlay_) {
    overlay_->show();
  }
  first_time_ = true;
}

void AttentionHUDDisplay::onDisable()
{
  MFDClass::onDisable();
  
  if (overlay_) {
    overlay_->hide();
  }
}

void AttentionHUDDisplay::createHUDOverlay()
{
  // Create unique names
  static int hud_counter = 0;
  std::string overlay_name = "AttentionHUDOverlay" + std::to_string(hud_counter);
  std::string panel_name = "AttentionHUDPanel" + std::to_string(hud_counter);
  std::string material_name = "AttentionHUDMaterial" + std::to_string(hud_counter);
  hud_counter++;
  
  // Create overlay
  Ogre::OverlayManager* overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  overlay_ = overlay_mgr->create(overlay_name);
  
  // Create panel overlay element
  panel_ = static_cast<Ogre::PanelOverlayElement*>(
    overlay_mgr->createOverlayElement("Panel", panel_name));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  
  // Set position and size based on user properties
  panel_->setPosition(hud_left_, hud_top_);
  panel_->setDimensions(hud_size_, hud_size_);
  
  // Create material for the panel
  panel_material_ = Ogre::MaterialManager::getSingleton().create(
    material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_->setMaterialName(panel_material_->getName());
  
  // Add panel to overlay
  overlay_->add2D(panel_);
  
  first_time_ = true;
  update_required_ = true;
}

void AttentionHUDDisplay::processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  
  // Analyze attention from the object list
  analyzeAttention(*msg);
  
  first_time_ = false;
  update_required_ = true;
  
  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic",
    QString("Processed %1 objects").arg(msg->objects.size()));
}

void AttentionHUDDisplay::analyzeAttention(const perception_msgs::msg::ObjectList& objects)
{
  // Reset sector data
  for (auto& sector : sector_data_) {
    sector.attention_level = 0.0f;
    sector.object_count = 0;
    sector.has_objects = false;
  }
  
  // Calculate sector attention based on object positions
  for (const auto& object : objects.objects) {
    float x = static_cast<float>(perception_msgs::object_access::getX(object));
    float y = static_cast<float>(perception_msgs::object_access::getY(object));
    float z = static_cast<float>(perception_msgs::object_access::getZ(object));
    
    float distance = std::sqrt(x*x + y*y + z*z);
    
    // Skip objects beyond max range
    if (distance > max_range_) {
      continue;
    }
    
    // Calculate angle (0 = +X axis, counter-clockwise)
    float angle = std::atan2(y, x);
    
    // Normalize angle to [0, 2π)
    if (angle < 0) {
      angle += 2.0f * M_PI;
    }
    
    // Determine sector (0 = North after rotation)
    int sector_index = getSectorIndex(angle);
    
    auto& sector = sector_data_[sector_index];
    sector.object_count++;
    sector.has_objects = true;
    
    // Calculate attention level based on proximity (closer = more attention)
    float normalized_distance = distance / max_range_;
    float proximity_attention = 1.0f - normalized_distance;
    
    // Accumulate attention
    sector.attention_level += proximity_attention;
  }
  
  // Set target opacities based on attention levels
  float max_attention = 0.0f;
  for (const auto& sector : sector_data_) {
    max_attention = std::max(max_attention, sector.attention_level);
  }
  
  // Normalize and set target opacities
  for (auto& sector : sector_data_) {
    if (sector.has_objects && max_attention > 0.0f) {
      sector.target_opacity = (sector.attention_level / max_attention) * hud_alpha_;
    } else {
      sector.target_opacity = 0.0f;
    }
  }
}

int AttentionHUDDisplay::getSectorIndex(float angle_rad) const
{
  // Rotate by π/2 (90 degrees) so that sector 0 = North (top)
  float adjusted_angle = angle_rad + (M_PI / 2.0f) + (M_PI / 8.0f);
  
  // Normalize to [0, 2π)
  while (adjusted_angle < 0) adjusted_angle += 2.0f * M_PI;
  while (adjusted_angle >= 2.0f * M_PI) adjusted_angle -= 2.0f * M_PI;
  
  int sector = static_cast<int>(adjusted_angle / SECTOR_ANGLE);
  return std::clamp(sector, 0, 7);
}

void AttentionHUDDisplay::updateOpacities(float dt)
{
  // Smooth opacity transitions
  for (auto& sector : sector_data_) {
    float diff = sector.target_opacity - sector.current_opacity;
    float step = animation_speed_ * dt;
    
    if (std::abs(diff) < step) {
      sector.current_opacity = sector.target_opacity;
    } else {
      sector.current_opacity += (diff > 0 ? step : -step);
    }
    
    // Clamp to valid range
    sector.current_opacity = std::clamp(sector.current_opacity, 0.0f, 1.0f);
  }
}

void AttentionHUDDisplay::update(float wall_dt, float /* ros_dt */)
{
  if (update_required_) {
    std::lock_guard<std::mutex> lock(hud_mutex_);
    
    // Update opacity animations
    updateOpacities(wall_dt);
    
    updateHUD();
    update_required_ = true; // Keep updating for animations
  }
}

void AttentionHUDDisplay::updateHUD()
{
  if (!overlay_ || !panel_) {
    return;
  }
  
  // Update panel position and size
  panel_->setPosition(hud_left_, hud_top_);
  panel_->setDimensions(hud_size_, hud_size_);
  
  // Create/update texture to match HUD size
  std::string texture_name = panel_material_->getName() + "Texture";
  
  if (hud_size_ == 0) {
    hud_size_ = 1; // Prevent zero-size textures
  }
  
  if (!texture_ || (texture_->getWidth() != static_cast<unsigned int>(hud_size_)) || 
      (texture_->getHeight() != static_cast<unsigned int>(hud_size_))) {
    
    if (texture_) {
      Ogre::TextureManager::getSingleton().remove(texture_name);
      panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }
    
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      hud_size_, hud_size_,
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_DEFAULT);
      
    panel_material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);
    panel_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }
  
  // Get texture buffer for drawing
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  
  const Ogre::PixelBox& pixel_box = pixel_buffer->getCurrentLock();
  
  // Create QImage from pixel buffer with transparent background
  QImage hud_image(static_cast<uchar*>(pixel_box.data), 
                   hud_size_, hud_size_, QImage::Format_ARGB32);
  
  // Fill with transparent background
  QColor bg_color(0, 0, 0, static_cast<int>(bg_alpha_ * 255));
  for (int i = 0; i < hud_size_; i++) {
    for (int j = 0; j < hud_size_; j++) {
      hud_image.setPixel(i, j, bg_color.rgba());
    }
  }
  
  // Draw HUD elements
  QPainter painter(&hud_image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  
  // Draw the futuristic trapezoidal elements around center
  drawTrapezoidalElements(painter, hud_size_, hud_size_);
  
  painter.end();
  
  pixel_buffer->unlock();
}

void AttentionHUDDisplay::drawTrapezoidalElements(QPainter& painter, int width, int height)
{
  int center_x = width / 2;
  int center_y = height / 2;
  
  // Draw 8 trapezoidal elements radiating from center
  for (int i = 0; i < 8; ++i) {
    const auto& sector = sector_data_[i];
    
    // Skip if no opacity
    if (sector.current_opacity <= 0.0f) {
      continue;
    }
    
    // Calculate angle - rotate so North points up
    float angle = i * SECTOR_ANGLE - (M_PI / 2.0f);
    
    // Draw beautiful trapezoid radiating outward with current opacity
    drawTrapezoid(painter, center_x, center_y, angle, sector.current_opacity, width, height);
  }
}

void AttentionHUDDisplay::drawTrapezoid(QPainter& painter, int center_x, int center_y, 
                                        float angle, float opacity, int width, int height)
{
  // Set up color with opacity
  QColor trap_color = hud_color_;
  trap_color.setAlpha(static_cast<int>(opacity * 255));
  
  // Create trapezoid shape - radiating from center toward edges
  int inner_radius = std::min(width, height) / 8;  // Distance from center to inner edge
  int outer_radius = std::min(width, height) / 2 - 20; // Distance to outer edge
  
  // Calculate trapezoid vertices
  QPointF vertices[4];
  
  // Inner edge (narrow end)
  float inner_angle1 = angle - (M_PI / 16.0f); // Narrower at inner edge
  float inner_angle2 = angle + (M_PI / 16.0f);
  vertices[0] = QPointF(center_x + inner_radius * std::cos(inner_angle1),
                        center_y + inner_radius * std::sin(inner_angle1));
  vertices[1] = QPointF(center_x + inner_radius * std::cos(inner_angle2),
                        center_y + inner_radius * std::sin(inner_angle2));
  
  // Outer edge (wider end)
  float outer_angle1 = angle - (M_PI / 8.0f); // Wider at outer edge
  float outer_angle2 = angle + (M_PI / 8.0f);
  vertices[2] = QPointF(center_x + outer_radius * std::cos(outer_angle2),
                        center_y + outer_radius * std::sin(outer_angle2));
  vertices[3] = QPointF(center_x + outer_radius * std::cos(outer_angle1),
                        center_y + outer_radius * std::sin(outer_angle1));
  
  // Create path for rounded trapezoid
  QPainterPath path;
  path.moveTo(vertices[0]);
  
  // Connect vertices with slight curves for futuristic look
  for (int i = 1; i < 4; ++i) {
    QPointF control = (vertices[i-1] + vertices[i]) / 2;
    path.quadTo(control, vertices[i]);
  }
  
  // Close the shape with curve back to start
  QPointF control = (vertices[3] + vertices[0]) / 2;
  path.quadTo(control, vertices[0]);
  
  // Draw filled trapezoid with gradient effect
  QRadialGradient gradient(center_x, center_y, outer_radius);
  QColor center_color = trap_color;
  center_color.setAlpha(static_cast<int>(opacity * 0.8 * 255));
  QColor edge_color = trap_color;
  edge_color.setAlpha(static_cast<int>(opacity * 0.3 * 255));
  
  gradient.setColorAt(0.0, center_color);
  gradient.setColorAt(1.0, edge_color);
  
  painter.fillPath(path, QBrush(gradient));
  
  // Add subtle glow effect
  QPen glow_pen(trap_color, 2);
  glow_pen.setCapStyle(Qt::RoundCap);
  glow_pen.setJoinStyle(Qt::RoundJoin);
  painter.setPen(glow_pen);
  painter.drawPath(path);
}

// Property update slots
void AttentionHUDDisplay::updateHUDProperties()
{
  max_range_ = max_range_property_->getFloat();
  update_required_ = true;
}

void AttentionHUDDisplay::updatePosition()
{
  hud_left_ = hud_left_property_->getInt();
  hud_top_ = hud_top_property_->getInt();
  update_required_ = true;
}

void AttentionHUDDisplay::updateSize()
{
  hud_size_ = hud_size_property_->getInt();
  update_required_ = true;
}

void AttentionHUDDisplay::updateColors()
{
  hud_color_ = hud_color_property_->getColor();
  update_required_ = true;
}

void AttentionHUDDisplay::updateTransparency()
{
  hud_alpha_ = hud_alpha_property_->getFloat();
  bg_alpha_ = bg_alpha_property_->getFloat();
  update_required_ = true;
}

void AttentionHUDDisplay::updateAnimationSpeed()
{
  animation_speed_ = animation_speed_property_->getFloat();
  update_required_ = true;
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionHUDDisplay, rviz_common::Display)