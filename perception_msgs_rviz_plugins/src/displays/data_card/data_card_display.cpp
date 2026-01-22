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

#include "perception_msgs/displays/data_card/data_card_display.hpp"

#include <algorithm>
#include <cmath>

#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <OgreTextureManager.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/render_system.hpp>

namespace perception_msgs {
namespace displays {

static int data_card_counter = 0;

DataCardDisplay::DataCardDisplay()
  : left_(340), top_(40), width_(kDefaultWidth), height_(kDefaultHeight),
    alpha_(0.95f), bg_alpha_(0.6f),
    animation_phase_(0.0), pulse_phase_(0.0),
    dataset_name_("Dataset: KARL"), dataset_version_("v1.0.1"),
    pretrain_dataset_("Waymo"),
    points_per_sample_("56946"),
    car_count_(4946), pedestrian_count_(3641), truck_count_(1606),
    trailer_count_(384), bus_count_(1), twowheeler_count_(160),
    overlay_(nullptr), panel_(nullptr),
    update_required_(true)
{
  const int id = data_card_counter++;
  overlay_name_ = "DataCardOverlay" + std::to_string(id);
  panel_name_ = "DataCardPanel" + std::to_string(id);
  material_name_ = "DataCardMaterial" + std::to_string(id);
  texture_name_ = "DataCardTexture" + std::to_string(id);

  // Position & Appearance properties
  left_property_ = new rviz_common::properties::IntProperty(
    "Left", left_, "X position", this, SLOT(updatePosition()));
  
  top_property_ = new rviz_common::properties::IntProperty(
    "Top", top_, "Y position", this, SLOT(updatePosition()));
  
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", width_, "Panel width in pixels", this, SLOT(updateSize()));
  width_property_->setMin(200);
  width_property_->setMax(500);
  
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Foreground Alpha", alpha_, "Text and icon opacity", this, SLOT(updateAppearance()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);
  
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", bg_alpha_, "Background opacity", this, SLOT(updateAppearance()));
  bg_alpha_property_->setMin(0.0f);
  bg_alpha_property_->setMax(1.0f);

  // Dataset Information properties
  dataset_name_property_ = new rviz_common::properties::StringProperty(
    "Dataset Name", dataset_name_, "Name of the training dataset", this, SLOT(updateDatasetInfo()));
  
  dataset_version_property_ = new rviz_common::properties::StringProperty(
    "Version", dataset_version_, "Dataset version number", this, SLOT(updateDatasetInfo()));
  
  pretrain_dataset_property_ = new rviz_common::properties::StringProperty(
    "Pretrain Dataset", pretrain_dataset_, "Dataset used for pretraining", this, SLOT(updateDatasetInfo()));
  
  points_per_sample_property_ = new rviz_common::properties::StringProperty(
    "Points/Sample", points_per_sample_, "LiDAR points per sample", this, SLOT(updateDatasetInfo()));
  
  car_count_property_ = new rviz_common::properties::IntProperty(
    "Car Count", car_count_, "Number of car samples", this, SLOT(updateDatasetInfo()));
  car_count_property_->setMin(0);
  
  pedestrian_count_property_ = new rviz_common::properties::IntProperty(
    "Pedestrian Count", pedestrian_count_, "Number of pedestrian samples", this, SLOT(updateDatasetInfo()));
  pedestrian_count_property_->setMin(0);
  
  truck_count_property_ = new rviz_common::properties::IntProperty(
    "Truck Count", truck_count_, "Number of truck samples", this, SLOT(updateDatasetInfo()));
  truck_count_property_->setMin(0);
  
  trailer_count_property_ = new rviz_common::properties::IntProperty(
    "Trailer Count", trailer_count_, "Number of trailer samples", this, SLOT(updateDatasetInfo()));
  trailer_count_property_->setMin(0);
  
  bus_count_property_ = new rviz_common::properties::IntProperty(
    "Bus Count", bus_count_, "Number of bus samples", this, SLOT(updateDatasetInfo()));
  bus_count_property_->setMin(0);
  
  twowheeler_count_property_ = new rviz_common::properties::IntProperty(
    "Two-Wheeler Count", twowheeler_count_, "Number of two-wheeler samples", this, SLOT(updateDatasetInfo()));
  twowheeler_count_property_->setMin(0);
}

DataCardDisplay::~DataCardDisplay()
{
  destroyOverlay();
  
  delete left_property_;
  delete top_property_;
  delete width_property_;
  delete alpha_property_;
  delete bg_alpha_property_;
  delete dataset_name_property_;
  delete dataset_version_property_;
  delete pretrain_dataset_property_;
  delete points_per_sample_property_;
  delete car_count_property_;
  delete pedestrian_count_property_;
  delete truck_count_property_;
  delete trailer_count_property_;
  delete bus_count_property_;
  delete twowheeler_count_property_;
}

void DataCardDisplay::onInitialize()
{
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  createOverlay();
}

void DataCardDisplay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
  update_required_ = true;
}

void DataCardDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
}

void DataCardDisplay::createOverlay()
{
  if (overlay_) {
    return;
  }

  auto& overlay_mgr = Ogre::OverlayManager::getSingleton();
  auto& tex_mgr = Ogre::TextureManager::getSingleton();
  auto& mat_mgr = Ogre::MaterialManager::getSingleton();

  texture_ = tex_mgr.createManual(
    texture_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_A8R8G8B8,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  material_ = mat_mgr.create(material_name_,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto* pass = material_->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthCheckEnabled(false);
  pass->setDepthWriteEnabled(false);
  pass->setLightingEnabled(false);
  pass->setCullingMode(Ogre::CULL_NONE);

  auto* tex_state = pass->createTextureUnitState(texture_name_);
  tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  tex_state->setTextureFiltering(Ogre::TFO_BILINEAR);

  panel_ = static_cast<Ogre::PanelOverlayElement*>(
    overlay_mgr.createOverlayElement("Panel", panel_name_));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  panel_->setPosition(static_cast<Ogre::Real>(left_), static_cast<Ogre::Real>(top_));
  panel_->setDimensions(static_cast<Ogre::Real>(width_), static_cast<Ogre::Real>(height_));
  panel_->setMaterialName(material_name_);

  overlay_ = overlay_mgr.create(overlay_name_);
  overlay_->add2D(panel_);
  overlay_->setZOrder(500);
  overlay_->show();
}

void DataCardDisplay::destroyOverlay()
{
  if (!overlay_) {
    return;
  }

  auto& overlay_mgr = Ogre::OverlayManager::getSingleton();
  auto& tex_mgr = Ogre::TextureManager::getSingleton();
  auto& mat_mgr = Ogre::MaterialManager::getSingleton();

  overlay_->hide();
  overlay_->remove2D(panel_);
  overlay_mgr.destroyOverlayElement(panel_);
  overlay_mgr.destroy(overlay_);

  mat_mgr.remove(material_);
  tex_mgr.remove(texture_);

  overlay_ = nullptr;
  panel_ = nullptr;
  material_.reset();
  texture_.reset();
}

void DataCardDisplay::update(float wall_dt, float /*ros_dt*/)
{
  if (!isEnabled() || !overlay_) {
    return;
  }

  animation_phase_ += wall_dt * 0.8;
  if (animation_phase_ > 2.0 * M_PI) {
    animation_phase_ -= 2.0 * M_PI;
  }
  
  pulse_phase_ += wall_dt * 2.0;
  if (pulse_phase_ > 2.0 * M_PI) {
    pulse_phase_ -= 2.0 * M_PI;
  }

  if (update_required_) {
    updateHUD();
    update_required_ = false;
  }
}

void DataCardDisplay::updatePosition()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  left_ = left_property_->getInt();
  top_ = top_property_->getInt();
  if (panel_) {
    panel_->setPosition(static_cast<Ogre::Real>(left_), static_cast<Ogre::Real>(top_));
  }
  update_required_ = true;
}

void DataCardDisplay::updateSize()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  width_ = width_property_->getInt();
  
  destroyOverlay();
  createOverlay();
  update_required_ = true;
}

void DataCardDisplay::updateAppearance()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  alpha_ = alpha_property_->getFloat();
  bg_alpha_ = bg_alpha_property_->getFloat();
  update_required_ = true;
}

void DataCardDisplay::updateDatasetInfo()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  dataset_name_ = dataset_name_property_->getString();
  dataset_version_ = dataset_version_property_->getString();
  pretrain_dataset_ = pretrain_dataset_property_->getString();
  points_per_sample_ = points_per_sample_property_->getString();
  car_count_ = car_count_property_->getInt();
  pedestrian_count_ = pedestrian_count_property_->getInt();
  truck_count_ = truck_count_property_->getInt();
  trailer_count_ = trailer_count_property_->getInt();
  bus_count_ = bus_count_property_->getInt();
  twowheeler_count_ = twowheeler_count_property_->getInt();
  update_required_ = true;
}

void DataCardDisplay::updateHUD()
{
  if (!texture_) {
    return;
  }

  Ogre::HardwarePixelBufferSharedPtr buffer = texture_->getBuffer();
  buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pb = buffer->getCurrentLock();
  
  QImage image(static_cast<uchar*>(pb.data), width_, height_, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  drawHUD(painter);
  painter.end();
  
  buffer->unlock();
}

void DataCardDisplay::drawHUD(QPainter& painter)
{
  const int margin = 12;
  const int alpha_val = static_cast<int>(alpha_ * 255);

  // Background gradient - futuristic dark theme (blue accent like ModelCard)
  QLinearGradient bg(0, 0, 0, height_);
  bg.setColorAt(0.0, QColor(15, 25, 45, static_cast<int>(bg_alpha_ * 255 * 0.95)));
  bg.setColorAt(0.3, QColor(20, 35, 60, static_cast<int>(bg_alpha_ * 255)));
  bg.setColorAt(1.0, QColor(10, 20, 40, static_cast<int>(bg_alpha_ * 255 * 0.9)));
  
  QPainterPath bg_path;
  bg_path.addRoundedRect(QRectF(0, 0, width_, height_), 10, 10);
  painter.fillPath(bg_path, bg);
  
  // Border glow (blue)
  QPen border_pen(QColor(80, 160, 220, static_cast<int>(alpha_ * 100)));
  border_pen.setWidth(1);
  painter.setPen(border_pen);
  painter.drawPath(bg_path);
  
  // Inner border
  QPainterPath inner_path;
  inner_path.addRoundedRect(QRectF(2, 2, width_ - 4, height_ - 4), 9, 9);
  painter.setPen(QPen(QColor(60, 120, 180, static_cast<int>(alpha_ * 60)), 1));
  painter.drawPath(inner_path);

  // Dataset icon
  const int icon_size = 40;
  const int icon_x = margin + 4;
  const int icon_y = margin + 4;
  drawDatasetIcon(painter, icon_x, icon_y, icon_size);

  // Title section - Dataset name
  QFont title_font("Segoe UI", 13, QFont::Bold);
  title_font.setLetterSpacing(QFont::PercentageSpacing, 102);
  painter.setFont(title_font);
  
  QColor title_color(200, 230, 255, alpha_val);
  painter.setPen(title_color);
  
  const int text_x = icon_x + icon_size + 12;
  painter.drawText(QRectF(text_x, margin, width_ - text_x - margin, 22),
                   Qt::AlignLeft | Qt::AlignVCenter, dataset_name_);

  // Version badge
  QFont version_font("Consolas", 9, QFont::Bold);
  painter.setFont(version_font);
  
  QColor version_bg(60, 180, 120, static_cast<int>(alpha_ * 180));
  QRectF version_rect(text_x, margin + 26, 55, 18);
  QPainterPath version_path;
  version_path.addRoundedRect(version_rect, 4, 4);
  painter.fillPath(version_path, version_bg);
  
  painter.setPen(QColor(255, 255, 255, alpha_val));
  painter.drawText(version_rect, Qt::AlignCenter, dataset_version_);

  // Pretrain badge (wider)
  QFont badge_font("Consolas", 8, QFont::Bold);
  painter.setFont(badge_font);
  
  QColor pretrain_bg(100, 80, 200, static_cast<int>(alpha_ * 180));
  QRectF pretrain_rect(text_x + 62, margin + 26, width_ - text_x - 62 - margin, 18);
  QPainterPath pretrain_path;
  pretrain_path.addRoundedRect(pretrain_rect, 4, 4);
  painter.fillPath(pretrain_path, pretrain_bg);
  
  painter.setPen(QColor(255, 255, 255, alpha_val));
  painter.drawText(pretrain_rect, Qt::AlignCenter, "Pretrained: " + pretrain_dataset_);

  // Quick stats row
  const int stats_y = margin + 52;
  QFont stats_font("Segoe UI", 9);
  painter.setFont(stats_font);
  
  // Total samples with pulsing glow
  int total_samples = car_count_ + pedestrian_count_ + truck_count_ + 
                      trailer_count_ + twowheeler_count_;
  double pulse = 0.5 + 0.5 * std::sin(pulse_phase_);
  QColor total_color(0, 220, 120, static_cast<int>(alpha_ * (180 + 75 * pulse)));
  painter.setPen(total_color);
  painter.drawText(QRectF(margin, stats_y, 110, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("⚡ ") + QString::number(total_samples) + " samples");

  // Points per sample
  QColor pts_color(255, 200, 80, static_cast<int>(alpha_ * 220));
  painter.setPen(pts_color);
  painter.drawText(QRectF(margin + 115, stats_y, 130, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("◆ ") + points_per_sample_ + " pts/sample");

  // Section divider
  const int section_y = 85;
  painter.setPen(QPen(QColor(80, 140, 200, alpha_val / 3), 1, Qt::DashLine));
  painter.drawLine(margin, section_y, width_ - margin, section_y);

  // Class distribution section
  QFont label_font("Segoe UI", 9);
  label_font.setBold(true);
  painter.setFont(label_font);
  painter.setPen(QColor(120, 150, 180, alpha_val));
  painter.drawText(QRectF(margin, section_y + 6, 150, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   "Class Distribution:");

  // Find max count for scaling bars
  int max_count = std::max({car_count_, pedestrian_count_, truck_count_, 
                            trailer_count_, twowheeler_count_});
  max_count = std::max(max_count, 1);  // Avoid division by zero

  const int bar_start_y = section_y + 28;
  const int bar_height = 18;
  const int bar_spacing = 24;
  const int bar_width = width_ - 2 * margin - 80;
  
  // Class bars with distinct colors
  struct ClassInfo {
    QString name;
    int count;
    QColor color;
  };
  
  std::vector<ClassInfo> classes = {
    {"Car", car_count_, QColor(100, 180, 255)},
    {"Pedestrian", pedestrian_count_, QColor(255, 150, 100)},
    {"Truck", truck_count_, QColor(150, 100, 255)},
    {"Trailer", trailer_count_, QColor(255, 200, 80)},
    {"Two-Wheeler", twowheeler_count_, QColor(100, 255, 180)}
  };

  int current_y = bar_start_y;
  for (const auto& cls : classes) {
    drawClassBar(painter, margin, current_y, bar_width, bar_height, 
                 cls.name, cls.count, max_count, cls.color);
    current_y += bar_spacing;
  }

  // Section divider before compliance
  int compliance_div_y = current_y + 4;
  painter.setPen(QPen(QColor(80, 140, 200, alpha_val / 3), 1, Qt::DashLine));
  painter.drawLine(margin, compliance_div_y, width_ - margin, compliance_div_y);

  // GDPR Compliance badge (full width) - green
  int badge_y = compliance_div_y + 12;
  QColor gdpr_bg(60, 140, 100, alpha_val);
  QRectF gdpr_rect(margin, badge_y, width_ - 2 * margin, 24);
  QPainterPath gdpr_path;
  gdpr_path.addRoundedRect(gdpr_rect, 6, 6);
  painter.fillPath(gdpr_path, gdpr_bg);
  
  painter.setFont(label_font);
  painter.setPen(QColor(200, 255, 220, alpha_val));
  painter.drawText(gdpr_rect, Qt::AlignCenter, 
                   QStringLiteral("GDPR Compliant ✓"));

  // Proprietary Private Dataset badge
  int prop_badge_y = badge_y + 30;
  QColor prop_bg(100, 80, 160, alpha_val);
  QRectF prop_rect(margin, prop_badge_y, width_ - 2 * margin, 24);
  QPainterPath prop_path;
  prop_path.addRoundedRect(prop_rect, 6, 6);
  painter.fillPath(prop_path, prop_bg);
  
  painter.setPen(QColor(220, 210, 255, alpha_val));
  painter.drawText(prop_rect, Qt::AlignCenter, 
                   QStringLiteral("Proprietary Private Dataset"));

  // Risks and Biases badge
  int risks_badge_y = prop_badge_y + 30;
  QColor risks_bg(180, 100, 60, alpha_val);
  QRectF risks_rect(margin, risks_badge_y, width_ - 2 * margin, 24);
  QPainterPath risks_path;
  risks_path.addRoundedRect(risks_rect, 6, 6);
  painter.fillPath(risks_path, risks_bg);
  
  painter.setPen(QColor(255, 230, 200, alpha_val));
  painter.drawText(risks_rect, Qt::AlignCenter, 
                   QStringLiteral("Risks and Biases ⚠"));
}

void DataCardDisplay::drawClassBar(QPainter& painter, int x, int y, int width, int height,
                                   const QString& label, int count, int max_count, const QColor& color)
{
  const int alpha_val = static_cast<int>(alpha_ * 255);
  const int label_width = 80;
  
  // Label
  QFont label_font("Segoe UI", 8);
  painter.setFont(label_font);
  painter.setPen(QColor(160, 190, 220, alpha_val));
  painter.drawText(QRectF(x, y, label_width - 5, height), 
                   Qt::AlignLeft | Qt::AlignVCenter, label);
  
  // Background bar
  const int bar_x = x + label_width;
  const int bar_w = width - label_width;
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(30, 50, 70, static_cast<int>(alpha_ * 100)));
  painter.drawRoundedRect(bar_x, y + 2, bar_w, height - 4, 3, 3);
  
  // Filled bar
  double fill_ratio = static_cast<double>(count) / static_cast<double>(max_count);
  int fill_width = static_cast<int>(fill_ratio * bar_w);
  if (fill_width > 0) {
    QColor bar_color = color;
    bar_color.setAlpha(alpha_val);
    painter.setBrush(bar_color);
    painter.drawRoundedRect(bar_x, y + 2, fill_width, height - 4, 3, 3);
  }
  
  // Count text
  QFont count_font("Consolas", 8);
  painter.setFont(count_font);
  painter.setPen(QColor(255, 255, 255, alpha_val));
  painter.drawText(QRectF(bar_x + 4, y, bar_w - 8, height), 
                   Qt::AlignRight | Qt::AlignVCenter, QString::number(count));
}

void DataCardDisplay::drawDatasetIcon(QPainter& painter, int x, int y, int size)
{
  // Animated dataset/database icon
  const double pulse = 0.5 + 0.5 * std::sin(animation_phase_);
  const int alpha_base = static_cast<int>(alpha_ * 255);
  
  // Background circle with glow
  QRadialGradient glow(x + size/2, y + size/2, size/2 + 4);
  glow.setColorAt(0.0, QColor(60, 140, 220, static_cast<int>(alpha_base * 0.3 * pulse)));
  glow.setColorAt(1.0, Qt::transparent);
  painter.setBrush(glow);
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(x - 4, y - 4, size + 8, size + 8);
  
  // Main circle background
  QRadialGradient bg(x + size/2, y + size/2, size/2);
  bg.setColorAt(0.0, QColor(40, 80, 140, alpha_base));
  bg.setColorAt(1.0, QColor(20, 50, 100, alpha_base));
  painter.setBrush(bg);
  painter.setPen(QPen(QColor(80, 160, 220, alpha_base), 1.5));
  painter.drawEllipse(x, y, size, size);
  
  // Database cylinder icon
  const int cx = x + size / 2;
  const int cy = y + size / 2;
  const int cyl_w = 18;
  const int cyl_h = 20;
  const int ellipse_h = 5;
  
  // Cylinder body
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(100, 180, 255, alpha_base));
  painter.drawRect(cx - cyl_w/2, cy - cyl_h/2 + ellipse_h/2, cyl_w, cyl_h - ellipse_h);
  
  // Top ellipse
  painter.setBrush(QColor(140, 200, 255, alpha_base));
  painter.drawEllipse(cx - cyl_w/2, cy - cyl_h/2, cyl_w, ellipse_h);
  
  // Bottom ellipse  
  painter.setBrush(QColor(60, 120, 180, alpha_base));
  painter.drawEllipse(cx - cyl_w/2, cy + cyl_h/2 - ellipse_h, cyl_w, ellipse_h);
  
  // Data lines (stacked disks effect)
  painter.setPen(QPen(QColor(40, 80, 140, alpha_base), 1));
  for (int i = 1; i <= 2; ++i) {
    int line_y = cy - cyl_h/2 + ellipse_h/2 + i * (cyl_h - ellipse_h) / 3;
    painter.drawEllipse(cx - cyl_w/2 + 1, line_y - 2, cyl_w - 2, 4);
  }
}

}  // namespace displays
}  // namespace perception_msgs

PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::DataCardDisplay, rviz_common::Display)
