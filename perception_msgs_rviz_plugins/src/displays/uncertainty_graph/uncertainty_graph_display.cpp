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

#include "perception_msgs/displays/uncertainty_graph/uncertainty_graph_display.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <QImage>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QFontMetrics>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_rendering/render_system.hpp"

namespace perception_msgs {
namespace displays {

UncertaintyGraphDisplay::UncertaintyGraphDisplay()
: width_property_(nullptr), height_property_(nullptr),
  left_property_(nullptr), top_property_(nullptr),
  alpha_property_(nullptr), bg_alpha_property_(nullptr),
  history_length_property_(nullptr), smoothing_property_(nullptr),
  title_property_(nullptr), class_color_property_(nullptr),
  regr_color_property_(nullptr), max_variance_property_(nullptr),
  show_thresholds_property_(nullptr), high_threshold_property_(nullptr),
  low_threshold_property_(nullptr), high_threshold_color_property_(nullptr),
  low_threshold_color_property_(nullptr),
  width_(kDefaultWidth), height_(kDefaultHeight),
  left_(40), top_(280),
  alpha_(0.9f), bg_alpha_(0.5f),
  history_length_(kDefaultHistoryLength), smoothing_(kDefaultSmoothing),
  title_(QStringLiteral("Uncertainty Over Time")),
  class_color_(0, 220, 180), regr_color_(180, 120, 255),
  max_variance_(10.0f), show_thresholds_(true),
  high_threshold_(0.75f), low_threshold_(0.45f),
  high_threshold_color_(0, 200, 100), low_threshold_color_(255, 180, 50),
  smoothed_classification_(0.0), smoothed_regression_(0.0),
  update_required_(false), animation_time_(0.0f),
  overlay_(nullptr), panel_(nullptr)
{
  // Position properties
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", width_, "Width of the graph overlay in pixels", this, SLOT(updateSize()));
  width_property_->setMin(200);
  width_property_->setMax(800);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", height_, "Height of the graph overlay in pixels", this, SLOT(updateSize()));
  height_property_->setMin(80);
  height_property_->setMax(400);

  left_property_ = new rviz_common::properties::IntProperty(
    "Left", left_, "X position from left edge", this, SLOT(updatePosition()));
  left_property_->setMin(0);
  left_property_->setMax(2000);

  top_property_ = new rviz_common::properties::IntProperty(
    "Top", top_, "Y position from top edge", this, SLOT(updatePosition()));
  top_property_->setMin(0);
  top_property_->setMax(2000);

  // Appearance properties
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Foreground Alpha", alpha_, "Opacity of lines and text", this, SLOT(updateAppearance()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", bg_alpha_, "Opacity of background", this, SLOT(updateAppearance()));
  bg_alpha_property_->setMin(0.0f);
  bg_alpha_property_->setMax(1.0f);

  history_length_property_ = new rviz_common::properties::IntProperty(
    "History Length", history_length_, "Number of samples to display", this, SLOT(updateAppearance()));
  history_length_property_->setMin(20);
  history_length_property_->setMax(200);

  smoothing_property_ = new rviz_common::properties::FloatProperty(
    "Smoothing", smoothing_, "Exponential smoothing factor (0=no smoothing, 1=max smoothing)", 
    this, SLOT(updateAppearance()));
  smoothing_property_->setMin(0.0f);
  smoothing_property_->setMax(0.99f);

  title_property_ = new rviz_common::properties::StringProperty(
    "Title", title_, "Graph title text", this, SLOT(updateAppearance()));

  // Line color properties
  class_color_property_ = new rviz_common::properties::ColorProperty(
    "Classification Color", class_color_, "Color for classification certainty line", 
    this, SLOT(updateAppearance()));

  regr_color_property_ = new rviz_common::properties::ColorProperty(
    "Regression Color", regr_color_, "Color for regression certainty line", 
    this, SLOT(updateAppearance()));

  // Variance threshold for regression certainty calculation
  max_variance_property_ = new rviz_common::properties::FloatProperty(
    "Max Variance", max_variance_, 
    "Maximum expected variance (higher values = lower certainty threshold)", 
    this, SLOT(updateAppearance()));
  max_variance_property_->setMin(0.1f);
  max_variance_property_->setMax(100.0f);

  // Threshold lines
  show_thresholds_property_ = new rviz_common::properties::BoolProperty(
    "Show Thresholds", show_thresholds_, "Show threshold indicator lines", 
    this, SLOT(updateAppearance()));

  high_threshold_property_ = new rviz_common::properties::FloatProperty(
    "High Threshold", high_threshold_, "High certainty threshold (0-1)", 
    show_thresholds_property_, SLOT(updateAppearance()), this);
  high_threshold_property_->setMin(0.0f);
  high_threshold_property_->setMax(1.0f);

  low_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Low Threshold", low_threshold_, "Low certainty threshold (0-1)", 
    show_thresholds_property_, SLOT(updateAppearance()), this);
  low_threshold_property_->setMin(0.0f);
  low_threshold_property_->setMax(1.0f);

  high_threshold_color_property_ = new rviz_common::properties::ColorProperty(
    "High Threshold Color", high_threshold_color_, "Color for high threshold line", 
    show_thresholds_property_, SLOT(updateAppearance()), this);

  low_threshold_color_property_ = new rviz_common::properties::ColorProperty(
    "Low Threshold Color", low_threshold_color_, "Color for low threshold line", 
    show_thresholds_property_, SLOT(updateAppearance()), this);
}

UncertaintyGraphDisplay::~UncertaintyGraphDisplay() {
  destroyOverlay();
  
  delete width_property_;
  delete height_property_;
  delete left_property_;
  delete top_property_;
  delete alpha_property_;
  delete bg_alpha_property_;
  delete history_length_property_;
  delete smoothing_property_;
  delete title_property_;
  delete class_color_property_;
  delete regr_color_property_;
  delete max_variance_property_;
  delete show_thresholds_property_;
  delete high_threshold_property_;
  delete low_threshold_property_;
  delete high_threshold_color_property_;
  delete low_threshold_color_property_;
}

void UncertaintyGraphDisplay::onInitialize() {
  MFDClass::onInitialize();
  
  // Generate unique names
  std::stringstream ss;
  ss << "UncertaintyGraphOverlay_" << this;
  overlay_name_ = ss.str();
  panel_name_ = overlay_name_ + "_Panel";
  texture_name_ = overlay_name_ + "_Texture";
  material_name_ = overlay_name_ + "_Material";
  
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  createOverlay();
}

void UncertaintyGraphDisplay::onEnable() {
  if (overlay_) overlay_->show();
  update_required_ = true;
}

void UncertaintyGraphDisplay::onDisable() {
  if (overlay_) overlay_->hide();
}

void UncertaintyGraphDisplay::createOverlay() {
  Ogre::OverlayManager& mgr = Ogre::OverlayManager::getSingleton();
  
  overlay_ = mgr.create(overlay_name_);
  
  panel_ = static_cast<Ogre::PanelOverlayElement*>(
    mgr.createOverlayElement("Panel", panel_name_));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  panel_->setPosition(static_cast<float>(left_), static_cast<float>(top_));
  panel_->setDimensions(static_cast<float>(width_), static_cast<float>(height_));
  
  // Create texture
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    texture_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_A8R8G8B8,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  
  // Create material
  material_ = Ogre::MaterialManager::getSingleton().create(
    material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthCheckEnabled(false);
  pass->setDepthWriteEnabled(false);
  pass->createTextureUnitState(texture_name_);
  
  panel_->setMaterialName(material_name_);
  overlay_->add2D(panel_);
  overlay_->setZOrder(501);
  overlay_->show();
  
  update_required_ = true;
}

void UncertaintyGraphDisplay::destroyOverlay() {
  if (overlay_) {
    Ogre::OverlayManager& mgr = Ogre::OverlayManager::getSingleton();
    overlay_->hide();
    if (panel_) {
      overlay_->remove2D(panel_);
      mgr.destroyOverlayElement(panel_);
      panel_ = nullptr;
    }
    mgr.destroy(overlay_);
    overlay_ = nullptr;
  }
  
  if (material_) {
    Ogre::MaterialManager::getSingleton().remove(material_name_);
    material_.reset();
  }
  
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_name_);
    texture_.reset();
  }
}

void UncertaintyGraphDisplay::updatePosition() {
  left_ = left_property_->getInt();
  top_ = top_property_->getInt();
  
  if (panel_) {
    panel_->setPosition(static_cast<float>(left_), static_cast<float>(top_));
  }
}

void UncertaintyGraphDisplay::updateSize() {
  int new_width = width_property_->getInt();
  int new_height = height_property_->getInt();
  
  if (new_width != width_ || new_height != height_) {
    width_ = new_width;
    height_ = new_height;
    
    // Recreate texture with new size
    destroyOverlay();
    createOverlay();
  }
}

void UncertaintyGraphDisplay::updateAppearance() {
  alpha_ = alpha_property_->getFloat();
  bg_alpha_ = bg_alpha_property_->getFloat();
  history_length_ = history_length_property_->getInt();
  smoothing_ = smoothing_property_->getFloat();
  title_ = title_property_->getString();
  class_color_ = class_color_property_->getColor();
  regr_color_ = regr_color_property_->getColor();
  max_variance_ = max_variance_property_->getFloat();
  show_thresholds_ = show_thresholds_property_->getBool();
  high_threshold_ = high_threshold_property_->getFloat();
  low_threshold_ = low_threshold_property_->getFloat();
  high_threshold_color_ = high_threshold_color_property_->getColor();
  low_threshold_color_ = low_threshold_color_property_->getColor();
  
  update_required_ = true;
}

double UncertaintyGraphDisplay::computeClassificationCertainty(
    const perception_msgs::msg::ObjectList& objects) const {
  if (objects.objects.empty()) return 0.0;
  
  double total_certainty = 0.0;
  int valid_count = 0;
  
  for (const auto& object : objects.objects) {
    double max_probability = 0.0;
    bool has_valid_classification = false;
    
    for (const auto& classification : object.state.classifications) {
      if (!std::isfinite(classification.probability)) {
        continue;
      }
      max_probability = std::max(max_probability, static_cast<double>(classification.probability));
      has_valid_classification = true;
    }
    
    if (!has_valid_classification) {
      max_probability = 0.0;
    }
    
    total_certainty += std::clamp(max_probability, 0.0, 1.0);
    ++valid_count;
  }
  
  return valid_count > 0 ? total_certainty / valid_count : 0.0;
}

double UncertaintyGraphDisplay::computeRegressionCertainty(
    const perception_msgs::msg::ObjectList& objects) const {
  if (objects.objects.empty()) return 0.0;
  
  double sum = 0.0;
  std::size_t count = 0;
  
  for (const auto& object : objects.objects) {
    const auto& cov = object.state.continuous_state_covariance;
    const int state_size = object.state.continuous_state.size();
    
    // Check if covariance data is available
    if (cov.size() < static_cast<size_t>(state_size * state_size) || state_size < 12) {
      continue;
    }
    
    // Extract variances for key states: X(0), Y(1), Z(2), YAW(7), WIDTH(9), LENGTH(10), HEIGHT(11)
    // In ISCACTR model: indices are [0]=X, [1]=Y, [2]=Z, [7]=YAW, [9]=WIDTH, [10]=LENGTH, [11]=HEIGHT
    const std::array<int, 7> indices = {0, 1, 2, 7, 9, 10, 11};
    double total_variance = 0.0;
    int valid_variances = 0;
    
    for (int idx : indices) {
      const double var = cov[idx * state_size + idx];
      // Skip invalid variances (-1 means not set, also skip NaN/Inf)
      if (var < 0.0 || !std::isfinite(var)) {
        continue;
      }
      total_variance += var;
      ++valid_variances;
    }
    
    if (valid_variances > 0) {
      // Average variance per state
      const double avg_variance = total_variance / static_cast<double>(valid_variances);
      // Convert variance to certainty: high variance = low certainty
      // Use exponential decay: certainty = exp(-variance / max_variance)
      const double certainty = std::exp(-avg_variance / max_variance_);
      sum += std::clamp(certainty, 0.0, 1.0);
      ++count;
    }
  }
  
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

void UncertaintyGraphDisplay::processMessage(
    perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  double raw_class = computeClassificationCertainty(*msg);
  double raw_regr = computeRegressionCertainty(*msg);
  
  // Exponential smoothing
  smoothed_classification_ = smoothing_ * smoothed_classification_ + (1.0 - smoothing_) * raw_class;
  smoothed_regression_ = smoothing_ * smoothed_regression_ + (1.0 - smoothing_) * raw_regr;
  
  // Update history buffers
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    
    classification_history_.push_back(smoothed_classification_);
    regression_history_.push_back(smoothed_regression_);
    
    while (static_cast<int>(classification_history_.size()) > history_length_) {
      classification_history_.pop_front();
    }
    while (static_cast<int>(regression_history_.size()) > history_length_) {
      regression_history_.pop_front();
    }
  }
  
  update_required_ = true;
}

void UncertaintyGraphDisplay::update(float wall_dt, float /*ros_dt*/) {
  animation_time_ += wall_dt;
  
  if (update_required_ && overlay_ && overlay_->isVisible()) {
    updateGraph();
    update_required_ = false;
  }
}

void UncertaintyGraphDisplay::updateGraph() {
  if (!texture_) return;
  
  QImage image(width_, height_, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  
  drawGraph(painter);
  
  painter.end();
  
  // Upload to texture
  Ogre::HardwarePixelBufferSharedPtr buffer = texture_->getBuffer();
  buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pb = buffer->getCurrentLock();
  
  uint8_t* dest = static_cast<uint8_t*>(pb.data);
  const int dest_pitch = static_cast<int>(pb.rowPitch) * 4;
  
  for (int y = 0; y < height_; ++y) {
    const QRgb* src_line = reinterpret_cast<const QRgb*>(image.scanLine(y));
    uint8_t* dest_line = dest + y * dest_pitch;
    for (int x = 0; x < width_; ++x) {
      QRgb px = src_line[x];
      dest_line[x * 4 + 0] = qBlue(px);
      dest_line[x * 4 + 1] = qGreen(px);
      dest_line[x * 4 + 2] = qRed(px);
      dest_line[x * 4 + 3] = qAlpha(px);
    }
  }
  
  buffer->unlock();
}

void UncertaintyGraphDisplay::drawGraph(QPainter& painter) {
  const int margin = 8;
  const int title_height = 20;
  const int legend_height = 18;
  const int axis_label_width = 28;
  
  // Background with gradient
  QLinearGradient bg_gradient(0, 0, 0, height_);
  bg_gradient.setColorAt(0.0, QColor(15, 25, 40, static_cast<int>(bg_alpha_ * 255 * 0.95)));
  bg_gradient.setColorAt(0.5, QColor(20, 35, 55, static_cast<int>(bg_alpha_ * 255)));
  bg_gradient.setColorAt(1.0, QColor(10, 20, 35, static_cast<int>(bg_alpha_ * 255 * 0.9)));
  
  // Rounded rectangle background
  QPainterPath bg_path;
  bg_path.addRoundedRect(QRectF(0, 0, width_, height_), 8, 8);
  painter.fillPath(bg_path, bg_gradient);
  
  // Subtle border glow
  QPen border_pen(QColor(60, 140, 200, static_cast<int>(alpha_ * 80)));
  border_pen.setWidth(1);
  painter.setPen(border_pen);
  painter.drawPath(bg_path);
  
  // Inner border
  QPainterPath inner_path;
  inner_path.addRoundedRect(QRectF(1, 1, width_ - 2, height_ - 2), 7, 7);
  painter.setPen(QPen(QColor(40, 80, 120, static_cast<int>(alpha_ * 60)), 1));
  painter.drawPath(inner_path);
  
  // Title (centered)
  QFont title_font;
  title_font.setFamily("Segoe UI");
  title_font.setPointSize(9);
  title_font.setBold(true);
  title_font.setLetterSpacing(QFont::PercentageSpacing, 105);
  painter.setFont(title_font);
  painter.setPen(QColor(180, 210, 240, static_cast<int>(alpha_ * 255)));
  painter.drawText(QRectF(margin, margin, width_ - 2 * margin, title_height),
                   Qt::AlignHCenter | Qt::AlignVCenter, title_);
  
  // Plot area
  const int plot_x = margin + axis_label_width;
  const int plot_y = margin + title_height;
  const int plot_width = width_ - 2 * margin - axis_label_width - 4;
  const int plot_height = height_ - 2 * margin - title_height - legend_height;
  
  // Plot background
  QLinearGradient plot_bg(plot_x, plot_y, plot_x, plot_y + plot_height);
  plot_bg.setColorAt(0.0, QColor(10, 20, 35, static_cast<int>(alpha_ * 120)));
  plot_bg.setColorAt(1.0, QColor(5, 15, 25, static_cast<int>(alpha_ * 100)));
  painter.fillRect(plot_x, plot_y, plot_width, plot_height, plot_bg);
  
  // Plot border
  painter.setPen(QPen(QColor(50, 100, 150, static_cast<int>(alpha_ * 100)), 1));
  painter.drawRect(plot_x, plot_y, plot_width, plot_height);
  
  // Grid lines (horizontal)
  painter.setPen(QPen(QColor(60, 100, 140, static_cast<int>(alpha_ * 40)), 1, Qt::DotLine));
  for (int i = 1; i < 4; ++i) {
    int y = plot_y + (plot_height * i) / 4;
    painter.drawLine(plot_x, y, plot_x + plot_width, y);
  }
  
  // Y-axis labels
  QFont axis_font;
  axis_font.setFamily("Consolas");
  axis_font.setPointSize(7);
  painter.setFont(axis_font);
  painter.setPen(QColor(120, 160, 200, static_cast<int>(alpha_ * 180)));
  
  painter.drawText(margin, plot_y + 8, QStringLiteral("1.0"));
  painter.drawText(margin, plot_y + plot_height / 2 + 4, QStringLiteral("0.5"));
  painter.drawText(margin, plot_y + plot_height, QStringLiteral("0.0"));
  
  // Threshold lines
  if (show_thresholds_) {
    // High threshold
    int high_y = plot_y + static_cast<int>((1.0 - high_threshold_) * plot_height);
    QColor high_color = high_threshold_color_;
    high_color.setAlpha(static_cast<int>(alpha_ * 120));
    painter.setPen(QPen(high_color, 1, Qt::DashLine));
    painter.drawLine(plot_x, high_y, plot_x + plot_width, high_y);
    
    // Low threshold
    int low_y = plot_y + static_cast<int>((1.0 - low_threshold_) * plot_height);
    QColor low_color = low_threshold_color_;
    low_color.setAlpha(static_cast<int>(alpha_ * 120));
    painter.setPen(QPen(low_color, 1, Qt::DashLine));
    painter.drawLine(plot_x, low_y, plot_x + plot_width, low_y);
  }
  
  // Helper lambda to draw a line graph with glow
  auto drawLineGraph = [&](const std::deque<double>& history, const QColor& color, int line_width) {
    if (history.size() < 2) return;
    
    const int n = static_cast<int>(history.size());
    const double x_step = static_cast<double>(plot_width) / static_cast<double>(history_length_ - 1);
    const int x_offset = static_cast<int>((history_length_ - n) * x_step);
    
    QPainterPath path;
    bool first = true;
    for (int i = 0; i < n; ++i) {
      const double val = std::clamp(history[i], 0.0, 1.0);
      const int x = plot_x + x_offset + static_cast<int>(i * x_step);
      const int y = plot_y + plot_height - static_cast<int>(val * plot_height);
      
      if (first) {
        path.moveTo(x, y);
        first = false;
      } else {
        path.lineTo(x, y);
      }
    }
    
    // Outer glow
    QColor glow_color = color;
    glow_color.setAlpha(static_cast<int>(alpha_ * 30));
    painter.setPen(QPen(glow_color, line_width + 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawPath(path);
    
    // Middle glow
    glow_color.setAlpha(static_cast<int>(alpha_ * 60));
    painter.setPen(QPen(glow_color, line_width + 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawPath(path);
    
    // Main line
    QColor line_color = color;
    line_color.setAlpha(static_cast<int>(alpha_ * 255));
    painter.setPen(QPen(line_color, line_width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawPath(path);
  };
  
  // Draw the graph lines
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    
    // Draw regression first (behind)
    if (!regression_history_.empty()) {
      drawLineGraph(regression_history_, regr_color_, 2);
    }
    
    // Draw classification on top
    if (!classification_history_.empty()) {
      drawLineGraph(classification_history_, class_color_, 2);
    }
  }
  
  // Scanline effect
  painter.setPen(Qt::NoPen);
  for (int y = plot_y; y < plot_y + plot_height; y += 3) {
    painter.fillRect(plot_x, y, plot_width, 1, QColor(0, 0, 0, static_cast<int>(alpha_ * 15)));
  }
  
  // Legend
  const int legend_y = height_ - margin - legend_height + 4;
  const int legend_box = 10;
  
  QFont legend_font;
  legend_font.setFamily("Segoe UI");
  legend_font.setPointSize(7);
  legend_font.setBold(true);
  painter.setFont(legend_font);
  
  // Classification legend
  QColor class_box = class_color_;
  class_box.setAlpha(static_cast<int>(alpha_ * 220));
  painter.setBrush(class_box);
  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(plot_x, legend_y, legend_box, legend_box, 2, 2);
  
  painter.setPen(QColor(180, 210, 240, static_cast<int>(alpha_ * 200)));
  painter.drawText(plot_x + legend_box + 4, legend_y + legend_box - 1, QStringLiteral("CLS"));
  
  // Regression legend
  const int regr_x = plot_x + 40;
  QColor regr_box = regr_color_;
  regr_box.setAlpha(static_cast<int>(alpha_ * 220));
  painter.setBrush(regr_box);
  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(regr_x, legend_y, legend_box, legend_box, 2, 2);
  
  painter.setPen(QColor(180, 210, 240, static_cast<int>(alpha_ * 200)));
  painter.drawText(regr_x + legend_box + 4, legend_y + legend_box - 1, QStringLiteral("REG"));
  
  // Current values (right side)
  QFont value_font;
  value_font.setFamily("Consolas");
  value_font.setPointSize(7);
  painter.setFont(value_font);
  
  QString class_val = QString::number(smoothed_classification_, 'f', 2);
  QString regr_val = QString::number(smoothed_regression_, 'f', 2);
  
  const int val_x = width_ - margin - 60;
  
  QColor class_text = class_color_;
  class_text.setAlpha(static_cast<int>(alpha_ * 255));
  painter.setPen(class_text);
  painter.drawText(val_x, legend_y + legend_box - 1, QStringLiteral("C:") + class_val);
  
  QColor regr_text = regr_color_;
  regr_text.setAlpha(static_cast<int>(alpha_ * 255));
  painter.setPen(regr_text);
  painter.drawText(val_x + 35, legend_y + legend_box - 1, QStringLiteral("R:") + regr_val);
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::UncertaintyGraphDisplay, rviz_common::Display)
