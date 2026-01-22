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

#include "perception_msgs/displays/model_card/model_card_display.hpp"

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

static int model_card_counter = 0;

ModelCardDisplay::ModelCardDisplay()
  : left_(40), top_(40), width_(kDefaultWidth), height_(kDefaultHeight),
    alpha_(0.95f), bg_alpha_(0.6f),
    animation_phase_(0.0), pulse_phase_(0.0),
    model_name_("TPOD-128"), model_version_("v2.1.0"),
    model_date_("2025-11-11"), training_dataset_("KARL + Waymo"),
    model_size_("12.4 MB"), accuracy_("95.6% mAP"),
    inference_time_("40 ms"), framework_("TensorRT FP16"),
    input_shape_("120000×3"), output_classes_("4"),
    overlay_(nullptr), panel_(nullptr),
    update_required_(true)
{
  const int id = model_card_counter++;
  overlay_name_ = "ModelCardOverlay" + std::to_string(id);
  panel_name_ = "ModelCardPanel" + std::to_string(id);
  material_name_ = "ModelCardMaterial" + std::to_string(id);
  texture_name_ = "ModelCardTexture" + std::to_string(id);

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

  // Model Information properties
  model_name_property_ = new rviz_common::properties::StringProperty(
    "Model Name", model_name_, "Name of the neural network model", this, SLOT(updateModelInfo()));
  
  model_version_property_ = new rviz_common::properties::StringProperty(
    "Version", model_version_, "Model version number", this, SLOT(updateModelInfo()));
  
  model_date_property_ = new rviz_common::properties::StringProperty(
    "Release Date", model_date_, "Model release/training date", this, SLOT(updateModelInfo()));
  
  training_dataset_property_ = new rviz_common::properties::StringProperty(
    "Training Dataset", training_dataset_, "Dataset used for training", this, SLOT(updateModelInfo()));
  
  model_size_property_ = new rviz_common::properties::StringProperty(
    "Model Size", model_size_, "Size of the model file", this, SLOT(updateModelInfo()));
  
  accuracy_property_ = new rviz_common::properties::StringProperty(
    "Accuracy", accuracy_, "Model accuracy metric", this, SLOT(updateModelInfo()));
  
  inference_time_property_ = new rviz_common::properties::StringProperty(
    "Inference Time", inference_time_, "Average inference time", this, SLOT(updateModelInfo()));
  
  framework_property_ = new rviz_common::properties::StringProperty(
    "Framework", framework_, "Deep learning framework", this, SLOT(updateModelInfo()));
  
  input_shape_property_ = new rviz_common::properties::StringProperty(
    "Input Shape", input_shape_, "Model input tensor shape", this, SLOT(updateModelInfo()));
  
  output_classes_property_ = new rviz_common::properties::StringProperty(
    "Output Classes", output_classes_, "Number of output classes", this, SLOT(updateModelInfo()));
}

ModelCardDisplay::~ModelCardDisplay()
{
  destroyOverlay();
  
  delete left_property_;
  delete top_property_;
  delete width_property_;
  delete alpha_property_;
  delete bg_alpha_property_;
  delete model_name_property_;
  delete model_version_property_;
  delete model_date_property_;
  delete training_dataset_property_;
  delete model_size_property_;
  delete accuracy_property_;
  delete inference_time_property_;
  delete framework_property_;
  delete input_shape_property_;
  delete output_classes_property_;
}

void ModelCardDisplay::onInitialize()
{
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  createOverlay();
}

void ModelCardDisplay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
  update_required_ = true;
}

void ModelCardDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
}

void ModelCardDisplay::createOverlay()
{
  if (overlay_) {
    return;
  }

  auto& overlay_mgr = Ogre::OverlayManager::getSingleton();
  auto& tex_mgr = Ogre::TextureManager::getSingleton();
  auto& mat_mgr = Ogre::MaterialManager::getSingleton();

  // Create texture with fixed height
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

void ModelCardDisplay::destroyOverlay()
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

void ModelCardDisplay::update(float wall_dt, float /*ros_dt*/)
{
  if (!isEnabled() || !overlay_) {
    return;
  }

  // Animation updates
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

void ModelCardDisplay::updatePosition()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  left_ = left_property_->getInt();
  top_ = top_property_->getInt();
  if (panel_) {
    panel_->setPosition(static_cast<Ogre::Real>(left_), static_cast<Ogre::Real>(top_));
  }
  update_required_ = true;
}

void ModelCardDisplay::updateSize()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  width_ = width_property_->getInt();
  
  // Recreate texture with new width
  destroyOverlay();
  createOverlay();
  update_required_ = true;
}

void ModelCardDisplay::updateAppearance()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  alpha_ = alpha_property_->getFloat();
  bg_alpha_ = bg_alpha_property_->getFloat();
  update_required_ = true;
}

void ModelCardDisplay::updateModelInfo()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  model_name_ = model_name_property_->getString();
  model_version_ = model_version_property_->getString();
  model_date_ = model_date_property_->getString();
  training_dataset_ = training_dataset_property_->getString();
  model_size_ = model_size_property_->getString();
  accuracy_ = accuracy_property_->getString();
  inference_time_ = inference_time_property_->getString();
  framework_ = framework_property_->getString();
  input_shape_ = input_shape_property_->getString();
  output_classes_ = output_classes_property_->getString();
  update_required_ = true;
}

void ModelCardDisplay::updateHUD()
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

void ModelCardDisplay::drawHUD(QPainter& painter)
{
  const int margin = 12;

  // Background gradient - futuristic dark theme
  QLinearGradient bg(0, 0, 0, height_);
  bg.setColorAt(0.0, QColor(15, 25, 45, static_cast<int>(bg_alpha_ * 255 * 0.95)));
  bg.setColorAt(0.3, QColor(20, 35, 60, static_cast<int>(bg_alpha_ * 255)));
  bg.setColorAt(1.0, QColor(10, 20, 40, static_cast<int>(bg_alpha_ * 255 * 0.9)));
  
  QPainterPath bg_path;
  bg_path.addRoundedRect(QRectF(0, 0, width_, height_), 10, 10);
  painter.fillPath(bg_path, bg);
  
  // Border glow
  QPen border_pen(QColor(80, 160, 220, static_cast<int>(alpha_ * 100)));
  border_pen.setWidth(1);
  painter.setPen(border_pen);
  painter.drawPath(bg_path);
  
  // Inner border
  QPainterPath inner_path;
  inner_path.addRoundedRect(QRectF(2, 2, width_ - 4, height_ - 4), 9, 9);
  painter.setPen(QPen(QColor(60, 120, 180, static_cast<int>(alpha_ * 60)), 1));
  painter.drawPath(inner_path);

  // Neural network icon
  const int icon_size = 40;
  const int icon_x = margin + 4;
  const int icon_y = margin + 4;
  drawNeuralNetIcon(painter, icon_x, icon_y, icon_size);

  // Title section - Model name and version
  QFont title_font("Segoe UI", 13, QFont::Bold);
  title_font.setLetterSpacing(QFont::PercentageSpacing, 102);
  painter.setFont(title_font);
  
  QColor title_color(200, 230, 255, static_cast<int>(alpha_ * 255));
  painter.setPen(title_color);
  
  const int text_x = icon_x + icon_size + 12;
  painter.drawText(QRectF(text_x, margin, width_ - text_x - margin, 22),
                   Qt::AlignLeft | Qt::AlignVCenter, model_name_);

  // Version badge
  QFont version_font("Consolas", 9, QFont::Bold);
  painter.setFont(version_font);
  
  QColor version_bg(60, 180, 120, static_cast<int>(alpha_ * 180));
  QRectF version_rect(text_x, margin + 26, 60, 18);
  QPainterPath version_path;
  version_path.addRoundedRect(version_rect, 4, 4);
  painter.fillPath(version_path, version_bg);
  
  painter.setPen(QColor(255, 255, 255, static_cast<int>(alpha_ * 255)));
  painter.drawText(version_rect, Qt::AlignCenter, model_version_);

  // Framework badge next to version
  QColor framework_bg(100, 80, 200, static_cast<int>(alpha_ * 180));
  QRectF framework_rect(text_x + 68, margin + 26, 105, 18);
  QPainterPath framework_path;
  framework_path.addRoundedRect(framework_rect, 4, 4);
  painter.fillPath(framework_path, framework_bg);
  painter.drawText(framework_rect, Qt::AlignCenter, framework_);

  // Quick stats row (always visible)
  const int stats_y = margin + 52;
  QFont stats_font("Segoe UI", 9);
  painter.setFont(stats_font);
  
  // Accuracy indicator with subtle glow
  double pulse = 0.5 + 0.5 * std::sin(pulse_phase_);
  QColor acc_color(0, 220, 120, static_cast<int>(alpha_ * (180 + 75 * pulse)));
  painter.setPen(acc_color);
  painter.drawText(QRectF(margin, stats_y, 90, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("⚡ ") + accuracy_);

  // Inference time
  QColor time_color(255, 200, 80, static_cast<int>(alpha_ * 220));
  painter.setPen(time_color);
  painter.drawText(QRectF(margin + 95, stats_y, 75, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("◷ ") + inference_time_);

  // Model size
  QColor size_color(150, 180, 220, static_cast<int>(alpha_ * 200));
  painter.setPen(size_color);
  painter.drawText(QRectF(margin + 175, stats_y, 85, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("▦ ") + model_size_);

  // Detail section
  const int alpha_val = static_cast<int>(alpha_ * 255);
  const int section_y = 90;
  
  // Section divider
  painter.setPen(QPen(QColor(80, 140, 200, alpha_val / 3), 1, Qt::DashLine));
  painter.drawLine(margin, section_y, width_ - margin, section_y);
  
  // Detail rows
  QFont label_font("Segoe UI", 9);
  label_font.setBold(true);
  QFont value_font("Consolas", 9);
  
  auto drawDetailRow = [&](int y, const QString& label, const QString& value, 
                           const QColor& value_color = QColor(180, 210, 240)) {
    painter.setFont(label_font);
    painter.setPen(QColor(120, 150, 180, alpha_val));
    painter.drawText(QRectF(margin, y, 100, 20), Qt::AlignLeft | Qt::AlignVCenter, label);
    
    painter.setFont(value_font);
    QColor vc = value_color;
    vc.setAlpha(alpha_val);
    painter.setPen(vc);
    painter.drawText(QRectF(margin + 105, y, width_ - margin - 115, 20), 
                     Qt::AlignLeft | Qt::AlignVCenter, value);
  };

  int row_y = section_y + 12;
  const int row_h = 26;
  
  drawDetailRow(row_y, "Release Date:", model_date_);
  row_y += row_h;
  
  drawDetailRow(row_y, "Dataset:", training_dataset_, QColor(100, 200, 255));
  row_y += row_h;
  
  drawDetailRow(row_y, "Input Shape:", input_shape_, QColor(255, 180, 100));
  row_y += row_h;
  
  drawDetailRow(row_y, "Classes:", output_classes_ + " object types", QColor(180, 255, 180));
  row_y += row_h;

  // Architecture diagram hint
  row_y += 8;
  painter.setPen(QPen(QColor(80, 140, 200, alpha_val / 3), 1, Qt::DashLine));
  painter.drawLine(margin, row_y, width_ - margin, row_y);
  row_y += 12;
  
  // Mini architecture visualization
  painter.setFont(label_font);
  painter.setPen(QColor(120, 150, 180, alpha_val));
  painter.drawText(QRectF(margin, row_y, 100, 20), Qt::AlignLeft | Qt::AlignVCenter, 
                   "Architecture:");
  
  // Draw mini network diagram
  const int net_x = margin + 105;
  const int net_y = row_y + 2;
  const int layer_w = 8;
  const int layer_spacing = 18;
  
  // Input layer
  painter.setBrush(QColor(100, 180, 255, alpha_val));
  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(net_x, net_y, layer_w, 16, 2, 2);
  
  // Hidden layers (varying heights)
  int layers[] = {14, 18, 20, 18, 14, 10};
  for (int i = 0; i < 6; ++i) {
    int lx = net_x + (i + 1) * layer_spacing;
    int lh = layers[i];
    int ly = net_y + (20 - lh) / 2;
    
    // Gradient color from blue to purple
    int r = 100 + i * 20;
    int g = 180 - i * 20;
    int b = 255 - i * 10;
    painter.setBrush(QColor(r, g, b, alpha_val));
    painter.drawRoundedRect(lx, ly, layer_w, lh, 2, 2);
  }
  
  // Output layer
  painter.setBrush(QColor(0, 220, 120, alpha_val));
  painter.drawRoundedRect(net_x + 7 * layer_spacing, net_y + 2, layer_w, 12, 2, 2);
  
  // Connection lines (subtle)
  painter.setPen(QPen(QColor(100, 150, 200, alpha_val / 4), 1));
  for (int i = 0; i < 7; ++i) {
    int x1 = net_x + i * layer_spacing + layer_w;
    int x2 = net_x + (i + 1) * layer_spacing;
    painter.drawLine(x1, net_y + 8, x2, net_y + 8);
  }
  
  row_y += 32;
  
  // Certification/Trust badge - Uncertainty Quantification
  QColor badge_bg(60, 100, 160, alpha_val);
  QRectF badge_rect(margin, row_y, width_ - 2 * margin, 24);
  QPainterPath badge_path;
  badge_path.addRoundedRect(badge_rect, 6, 6);
  painter.fillPath(badge_path, badge_bg);
  
  painter.setFont(label_font);
  painter.setPen(QColor(200, 230, 255, alpha_val));
  painter.drawText(badge_rect, Qt::AlignCenter, 
                   QStringLiteral("Uncertainty Quantification ✓"));

  // Robustness badge
  row_y += 30;
  QColor robust_bg(60, 140, 100, alpha_val);
  QRectF robust_rect(margin, row_y, width_ - 2 * margin, 24);
  QPainterPath robust_path;
  robust_path.addRoundedRect(robust_rect, 6, 6);
  painter.fillPath(robust_path, robust_bg);
  
  painter.setPen(QColor(200, 255, 220, alpha_val));
  painter.drawText(robust_rect, Qt::AlignCenter, 
                   QStringLiteral("Robustness ✓"));

  // XAI badge
  row_y += 30;
  QColor xai_bg(100, 80, 160, alpha_val);
  QRectF xai_rect(margin, row_y, width_ - 2 * margin, 24);
  QPainterPath xai_path;
  xai_path.addRoundedRect(xai_rect, 6, 6);
  painter.fillPath(xai_path, xai_bg);
  
  painter.setPen(QColor(220, 210, 255, alpha_val));
  painter.drawText(xai_rect, Qt::AlignCenter, 
                   QStringLiteral("XAI ✓"));
}

void ModelCardDisplay::drawNeuralNetIcon(QPainter& painter, int x, int y, int size)
{
  // Animated neural network icon
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
  
  // Neural network nodes
  const int cx = x + size / 2;
  const int cy = y + size / 2;
  const int node_r = 3;
  
  // Input layer (left)
  QColor node_color(100, 200, 255, alpha_base);
  painter.setBrush(node_color);
  painter.setPen(Qt::NoPen);
  
  const int input_x = cx - 12;
  painter.drawEllipse(input_x - node_r, cy - 8 - node_r, node_r * 2, node_r * 2);
  painter.drawEllipse(input_x - node_r, cy - node_r, node_r * 2, node_r * 2);
  painter.drawEllipse(input_x - node_r, cy + 8 - node_r, node_r * 2, node_r * 2);
  
  // Hidden layer (center) - pulsing
  QColor hidden_color(180, 100, 255, static_cast<int>(alpha_base * (0.7 + 0.3 * pulse)));
  painter.setBrush(hidden_color);
  painter.drawEllipse(cx - node_r, cy - 6 - node_r, node_r * 2, node_r * 2);
  painter.drawEllipse(cx - node_r, cy + 6 - node_r, node_r * 2, node_r * 2);
  
  // Output layer (right)
  QColor output_color(0, 220, 120, alpha_base);
  painter.setBrush(output_color);
  const int output_x = cx + 12;
  painter.drawEllipse(output_x - node_r, cy - node_r, node_r * 2, node_r * 2);
  
  // Connection lines
  painter.setPen(QPen(QColor(150, 180, 220, static_cast<int>(alpha_base * 0.5)), 1));
  
  // Input to hidden
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; j += 2) {
      painter.drawLine(input_x, cy + i * 8, cx, cy + j * 6);
    }
  }
  
  // Hidden to output
  painter.drawLine(cx, cy - 6, output_x, cy);
  painter.drawLine(cx, cy + 6, output_x, cy);
}

}  // namespace displays
}  // namespace perception_msgs

PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::ModelCardDisplay, rviz_common::Display)
