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

#include "perception_msgs/displays/attention_compass/attention_compass_display.hpp"

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

// Sector label constants
const char* AttentionCompassDisplay::SECTOR_LABELS[8] = {
  "N", "NE", "E", "SE", "S", "SW", "W", "NW"
};

AttentionCompassDisplay::AttentionCompassDisplay()
: update_required_(false), first_time_(true), hud_size_(DEFAULT_HUD_SIZE),
  hud_left_(50), hud_top_(50), hud_alpha_(0.8f), bg_alpha_(0.3f), 
  max_range_(50.0f), show_labels_(true), show_object_counts_(true),
  scene_manager_(nullptr), overlay_(nullptr), panel_(nullptr)
{
  // Initialize sector data
  for (auto& sector : sector_data_) {
    sector.attention_level = 0.0f;
    sector.object_count = 0;
    sector.closest_distance = std::numeric_limits<float>::max();
    sector.has_objects = false;
  }

  // Create properties
  hud_size_property_ = new rviz_common::properties::IntProperty(
    "HUD Size", DEFAULT_HUD_SIZE,
    "Size of the HUD overlay in pixels", this, SLOT(updateSize()));
  hud_size_property_->setMin(100);
  hud_size_property_->setMax(800);

  hud_left_property_ = new rviz_common::properties::IntProperty(
    "Left", 50, "Left position of the HUD overlay", this, SLOT(updatePosition()));

  hud_top_property_ = new rviz_common::properties::IntProperty(
    "Top", 50, "Top position of the HUD overlay", this, SLOT(updatePosition()));

  hud_color_property_ = new rviz_common::properties::ColorProperty(
    "HUD Color", QColor(0, 255, 255), // Cyan
    "Color of the HUD elements", this, SLOT(updateColors()));

  hud_alpha_property_ = new rviz_common::properties::FloatProperty(
    "HUD Alpha", 0.8f,
    "Transparency of HUD elements (0.0 = transparent, 1.0 = opaque)", 
    this, SLOT(updateTransparency()));
  hud_alpha_property_->setMin(0.0f);
  hud_alpha_property_->setMax(1.0f);

  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", 0.3f,
    "Transparency of HUD background (0.0 = transparent, 1.0 = opaque)", 
    this, SLOT(updateTransparency()));
  bg_alpha_property_->setMin(0.0f);
  bg_alpha_property_->setMax(1.0f);

  max_range_property_ = new rviz_common::properties::FloatProperty(
    "Max Range", 50.0f,
    "Maximum range for attention analysis (meters)", this, SLOT(updateHUDProperties()));
  max_range_property_->setMin(1.0f);
  max_range_property_->setMax(200.0f);

  show_labels_property_ = new rviz_common::properties::BoolProperty(
    "Show Labels", true,
    "Show sector direction labels (N, NE, etc.)", this, SLOT(updateHUDProperties()));

  show_object_counts_property_ = new rviz_common::properties::BoolProperty(
    "Show Object Counts", true,
    "Show number of objects in each sector", this, SLOT(updateHUDProperties()));

  ego_frame_property_ = new rviz_common::properties::StringProperty(
    "Ego Frame", "base_link",
    "Frame ID representing the ego vehicle position", this, SLOT(updateHUDProperties()));

  // Set default HUD color to cyan
  hud_color_ = QColor(0, 255, 255);
}

AttentionCompassDisplay::~AttentionCompassDisplay()
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
  delete show_labels_property_;
  delete show_object_counts_property_;
  delete ego_frame_property_;
}

void AttentionCompassDisplay::onInitialize()
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
}

void AttentionCompassDisplay::onEnable()
{
  MFDClass::onEnable();
  
  if (overlay_) {
    overlay_->show();
  }
  first_time_ = true;
}

void AttentionCompassDisplay::onDisable()
{
  MFDClass::onDisable();
  
  if (overlay_) {
    overlay_->hide();
  }
}

void AttentionCompassDisplay::createHUDOverlay()
{
  // Create unique names
  static int hud_counter = 0;
  std::string overlay_name = "AttentionCompassOverlay" + std::to_string(hud_counter);
  std::string panel_name = "AttentionCompassPanel" + std::to_string(hud_counter);
  std::string texture_name = "AttentionCompassTexture" + std::to_string(hud_counter);
  std::string material_name = "AttentionCompassMaterial" + std::to_string(hud_counter);
  hud_counter++;
  
  // Create overlay
  Ogre::OverlayManager* overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  overlay_ = overlay_mgr->create(overlay_name);
  
  // Create panel overlay element
  panel_ = static_cast<Ogre::PanelOverlayElement*>(
    overlay_mgr->createOverlayElement("Panel", panel_name));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  
  // Create material for the panel
  panel_material_ = Ogre::MaterialManager::getSingleton().create(
    material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_->setMaterialName(panel_material_->getName());
  
  // Add panel to overlay
  overlay_->add2D(panel_);
  
  first_time_ = true;
  update_required_ = true;
}

void AttentionCompassDisplay::processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  // Skip validation for ObjectList as it's not supported by rviz_common::validateFloats
  // Objects will be validated individually during processing if needed

  std::lock_guard<std::mutex> lock(hud_mutex_);
  
  // Analyze attention from the object list
  analyzeAttention(*msg);
  
  first_time_ = false;
  update_required_ = true;
  
  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic",
    QString("Processed %1 objects").arg(msg->objects.size()));
}

void AttentionCompassDisplay::analyzeAttention(const perception_msgs::msg::ObjectList& objects)
{
  // Reset sector data
  for (auto& sector : sector_data_) {
    sector.attention_level = 0.0f;
    sector.object_count = 0;
    sector.closest_distance = std::numeric_limits<float>::max();
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
    
    // Determine sector (0 = East, 1 = NorthEast, etc.)
    int sector_index = getSectorIndex(angle);
    
    auto& sector = sector_data_[sector_index];
    sector.object_count++;
    sector.has_objects = true;
    sector.closest_distance = std::min(sector.closest_distance, distance);
    
    // Calculate attention level based on proximity (closer = more attention)
    // Normalize distance to [0,1] and invert so closer objects have higher attention
    float normalized_distance = distance / max_range_;
    float proximity_attention = 1.0f - normalized_distance;
    
    // Accumulate attention (can be refined with more sophisticated weighting)
    sector.attention_level += proximity_attention;
  }
  
  // Normalize attention levels to [0,1] range
  float max_attention = 0.0f;
  for (const auto& sector : sector_data_) {
    max_attention = std::max(max_attention, sector.attention_level);
  }
  
  if (max_attention > 0.0f) {
    for (auto& sector : sector_data_) {
      sector.attention_level = sector.attention_level / max_attention;
    }
  }
}

int AttentionCompassDisplay::getSectorIndex(float angle_rad) const
{
  // Convert to sector index (0-7)
  // Rotate by π/2 (90 degrees) so that:
  // Sector 0 = North (-π/2), Sector 1 = Northeast (-π/4), etc.
  float adjusted_angle = angle_rad + (M_PI / 2.0f) + (M_PI / 8.0f);
  
  // Normalize to [0, 2π)
  while (adjusted_angle < 0) adjusted_angle += 2.0f * M_PI;
  while (adjusted_angle >= 2.0f * M_PI) adjusted_angle -= 2.0f * M_PI;
  
  int sector = static_cast<int>(adjusted_angle / SECTOR_ANGLE);
  return std::clamp(sector, 0, 7);
}

void AttentionCompassDisplay::update(float /* wall_dt */, float /* ros_dt */)
{
  if (update_required_) {
    std::lock_guard<std::mutex> lock(hud_mutex_);
    updateHUD();
    update_required_ = false;
  }
}

void AttentionCompassDisplay::updateHUD()
{
  if (!overlay_ || !panel_) {
    return;
  }
  
  // Update panel properties
  panel_->setPosition(hud_left_, hud_top_);
  panel_->setDimensions(hud_size_, hud_size_);
  
  // Create/update texture if needed
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
  
  // Create QImage from pixel buffer with background color
  QImage hud_image(static_cast<uchar*>(pixel_box.data), 
                   hud_size_, hud_size_, QImage::Format_ARGB32);
  
  QColor bg_color(0, 0, 0, static_cast<int>(bg_alpha_ * 255));
  for (int i = 0; i < hud_size_; i++) {
    for (int j = 0; j < hud_size_; j++) {
      hud_image.setPixel(i, j, bg_color.rgba());
    }
  }
  
  // Draw HUD to the image
  QPainter painter(&hud_image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  
  // Draw HUD elements
  drawHUDBackground(painter, hud_size_, hud_size_);
  drawCentralCrosshairs(painter, hud_size_, hud_size_);
  drawSectorIndicators(painter, hud_size_, hud_size_);
  drawAttentionBars(painter, hud_size_, hud_size_);
  
  if (show_labels_) {
    drawSectorLabels(painter, hud_size_, hud_size_);
  }
  
  if (show_object_counts_) {
    drawObjectCounts(painter, hud_size_, hud_size_);
  }
  
  painter.end();
  
  pixel_buffer->unlock();
}


void AttentionCompassDisplay::drawHUDBackground(QPainter& painter, int width, int height)
{
  // Draw outer ring
  QColor bg_color = hud_color_;
  bg_color.setAlpha(static_cast<int>(bg_alpha_ * 255));
  
  painter.setPen(QPen(bg_color, 2, Qt::SolidLine));
  painter.drawEllipse(10, 10, width - 20, height - 20);
  
  // Draw inner ring
  painter.drawEllipse(width/4, height/4, width/2, height/2);
}

void AttentionCompassDisplay::drawCentralCrosshairs(QPainter& painter, int width, int height)
{
  QColor cross_color = hud_color_;
  cross_color.setAlpha(static_cast<int>(hud_alpha_ * 255));
  
  painter.setPen(QPen(cross_color, 1, Qt::SolidLine));
  
  int center_x = width / 2;
  int center_y = height / 2;
  int cross_size = 20;
  
  // Horizontal line
  painter.drawLine(center_x - cross_size, center_y, center_x + cross_size, center_y);
  // Vertical line
  painter.drawLine(center_x, center_y - cross_size, center_x, center_y + cross_size);
  
  // Central dot
  painter.fillRect(center_x - 2, center_y - 2, 4, 4, cross_color);
}

void AttentionCompassDisplay::drawSectorIndicators(QPainter& painter, int width, int height)
{
  QColor sector_color = hud_color_;
  sector_color.setAlpha(static_cast<int>(hud_alpha_ * 0.6 * 255));
  
  painter.setPen(QPen(sector_color, 1, Qt::DashLine));
  
  int center_x = width / 2;
  int center_y = height / 2;
  int radius = width / 2 - 15;
  
  // Draw sector division lines
  for (int i = 0; i < 8; ++i) {
    float angle = i * SECTOR_ANGLE - (M_PI / 2.0f); // Rotate so N points up
    int end_x = center_x + static_cast<int>(radius * std::cos(angle));
    int end_y = center_y + static_cast<int>(radius * std::sin(angle));
    
    painter.drawLine(center_x, center_y, end_x, end_y);
  }
}

void AttentionCompassDisplay::drawAttentionBars(QPainter& painter, int width, int height)
{
  int center_x = width / 2;
  int center_y = height / 2;
  int inner_radius = width / 4;
  int outer_radius = width / 2 - 30;
  
  for (int i = 0; i < 8; ++i) {
    const auto& sector = sector_data_[i];
    
    if (!sector.has_objects) {
      continue;
    }
    
    // Calculate bar intensity based on attention level
    QColor bar_color = hud_color_;
    int alpha = static_cast<int>(sector.attention_level * hud_alpha_ * 255);
    bar_color.setAlpha(alpha);
    
    // Calculate bar geometry
    float angle = i * SECTOR_ANGLE - (M_PI / 2.0f); // Rotate so N points up
    // float bar_width = SECTOR_ANGLE * 0.8f; // 80% of sector width
    // Note: start_angle and end_angle could be used for more sophisticated arc rendering
    // float start_angle = angle - bar_width / 2.0f;
    // float end_angle = angle + bar_width / 2.0f;
    
    // Draw attention bar as filled arc segment
    painter.setBrush(QBrush(bar_color));
    painter.setPen(Qt::NoPen);
    
    // Create arc path (simplified rectangle for demonstration)
    int bar_length = static_cast<int>((outer_radius - inner_radius) * sector.attention_level);
    int bar_start = inner_radius;
    int bar_end = bar_start + bar_length;
    
    // Draw bar as line from center outward
    int bar_x = center_x + static_cast<int>(bar_end * std::cos(angle));
    int bar_y = center_y + static_cast<int>(bar_end * std::sin(angle));
    
    painter.setPen(QPen(bar_color, 8, Qt::SolidLine));
    painter.drawLine(
      center_x + static_cast<int>(bar_start * std::cos(angle)),
      center_y + static_cast<int>(bar_start * std::sin(angle)),
      bar_x, bar_y
    );
  }
}

void AttentionCompassDisplay::drawSectorLabels(QPainter& painter, int width, int height)
{
  QColor label_color = hud_color_;
  label_color.setAlpha(static_cast<int>(hud_alpha_ * 255));
  
  painter.setPen(QPen(label_color, 1, Qt::SolidLine));
  
  QFont font = painter.font();
  font.setPointSize(12);
  font.setBold(true);
  painter.setFont(font);
  
  int center_x = width / 2;
  int center_y = height / 2;
  int label_radius = width / 2 - 25;
  
  for (int i = 0; i < 8; ++i) {
    float angle = i * SECTOR_ANGLE - (M_PI / 2.0f); // Rotate so N points up
    int label_x = center_x + static_cast<int>(label_radius * std::cos(angle));
    int label_y = center_y + static_cast<int>(label_radius * std::sin(angle));
    
    // Adjust text position to center on point
    QFontMetrics metrics(font);
    QRect text_rect = metrics.boundingRect(SECTOR_LABELS[i]);
    label_x -= text_rect.width() / 2;
    label_y += text_rect.height() / 4;
    
    painter.drawText(label_x, label_y, SECTOR_LABELS[i]);
  }
}

void AttentionCompassDisplay::drawObjectCounts(QPainter& painter, int width, int height)
{
  QColor count_color = hud_color_;
  count_color.setAlpha(static_cast<int>(hud_alpha_ * 0.8 * 255));
  
  painter.setPen(QPen(count_color, 1, Qt::SolidLine));
  
  QFont font = painter.font();
  font.setPointSize(10);
  painter.setFont(font);
  
  int center_x = width / 2;
  int center_y = height / 2;
  int count_radius = width / 3;
  
  for (int i = 0; i < 8; ++i) {
    const auto& sector = sector_data_[i];
    
    if (!sector.has_objects) {
      continue;
    }
    
    float angle = i * SECTOR_ANGLE - (M_PI / 2.0f); // Rotate so N points up
    int count_x = center_x + static_cast<int>(count_radius * std::cos(angle));
    int count_y = center_y + static_cast<int>(count_radius * std::sin(angle));
    
    std::string count_text = std::to_string(sector.object_count);
    
    // Adjust text position
    QFontMetrics metrics(font);
    QRect text_rect = metrics.boundingRect(count_text.c_str());
    count_x -= text_rect.width() / 2;
    count_y += text_rect.height() / 4;
    
    painter.drawText(count_x, count_y, count_text.c_str());
  }
}

// Property update slots
void AttentionCompassDisplay::updateHUDProperties()
{
  update_required_ = true;
}

void AttentionCompassDisplay::updatePosition()
{
  hud_left_ = hud_left_property_->getInt();
  hud_top_ = hud_top_property_->getInt();
  update_required_ = true;
}

void AttentionCompassDisplay::updateSize()
{
  hud_size_ = hud_size_property_->getInt();
  update_required_ = true;
}

void AttentionCompassDisplay::updateColors()
{
  hud_color_ = hud_color_property_->getColor();
  update_required_ = true;
}

void AttentionCompassDisplay::updateTransparency()
{
  hud_alpha_ = hud_alpha_property_->getFloat();
  bg_alpha_ = bg_alpha_property_->getFloat();
  update_required_ = true;
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionCompassDisplay, rviz_common::Display)
