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

#include "perception_msgs/displays/attention_uncertainty/attention_uncertainty_display.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>

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

namespace {
constexpr float kClampEpsilon = 1e-5f;
}

AttentionUncertaintyDisplay::AttentionUncertaintyDisplay()
: hud_width_property_(nullptr), hud_height_property_(nullptr),
  hud_left_property_(nullptr), hud_top_property_(nullptr),
  hud_alpha_property_(nullptr), bg_alpha_property_(nullptr),
  high_threshold_property_(nullptr), low_threshold_property_(nullptr),
  blink_threshold_property_(nullptr), blink_frequency_property_(nullptr),
  smoothing_alpha_property_(nullptr),
  high_color_property_(nullptr), mid_color_property_(nullptr),
  low_color_property_(nullptr), title_text_property_(nullptr),
  regression_high_color_property_(nullptr), regression_mid_color_property_(nullptr),
  regression_low_color_property_(nullptr), max_variance_property_(nullptr),
  hud_width_(kDefaultWidth), hud_height_(kDefaultHeight),
  hud_left_(40), hud_top_(40), hud_alpha_(0.9f), bg_alpha_(0.35f),
  high_threshold_(0.75f), low_threshold_(0.45f), blink_threshold_(0.25f),
  blink_frequency_(2.0f), smoothing_alpha_(0.6f),
  high_color_(0, 220, 120), mid_color_(255, 200, 40),
  low_color_(255, 70, 70), title_text_(QStringLiteral("Perception Certainty")),
  regression_high_color_(140, 100, 255), regression_mid_color_(180, 120, 255),
  regression_low_color_(220, 80, 180), max_variance_(10.0f),
  smoothed_classification_certainty_(0.0), smoothed_regression_certainty_(0.0),
  have_classification_certainty_(false), have_regression_certainty_(false),
  update_required_(false), blink_state_(false), blink_timer_(0.0), no_data_timer_(0.0),
  overlay_(nullptr), panel_(nullptr)
{
  hud_width_property_ = new rviz_common::properties::IntProperty(
    "HUD Width", hud_width_,
    "Width of the uncertainty overlay in pixels", this, SLOT(updateSize()));
  hud_width_property_->setMin(160);
  hud_width_property_->setMax(640);

  hud_height_property_ = new rviz_common::properties::IntProperty(
    "HUD Height", hud_height_,
    "Height of the uncertainty overlay in pixels", this, SLOT(updateSize()));
  hud_height_property_->setMin(120);
  hud_height_property_->setMax(480);

  hud_left_property_ = new rviz_common::properties::IntProperty(
    "Left", hud_left_,
    "Left screen position of the overlay (in pixels)", this, SLOT(updatePosition()));

  hud_top_property_ = new rviz_common::properties::IntProperty(
    "Top", hud_top_,
    "Top screen position of the overlay (in pixels)", this, SLOT(updatePosition()));

  hud_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Foreground Alpha", hud_alpha_,
    "Opacity applied to bar and frame elements (0 = transparent, 1 = opaque)",
    this, SLOT(updateTransparency()));
  hud_alpha_property_->setMin(0.0f);
  hud_alpha_property_->setMax(1.0f);

  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", bg_alpha_,
    "Opacity of the background glass panel", this, SLOT(updateTransparency()));
  bg_alpha_property_->setMin(0.0f);
  bg_alpha_property_->setMax(1.0f);

  high_threshold_property_ = new rviz_common::properties::FloatProperty(
    "High Certainty Threshold", high_threshold_,
    "Certainty above which the bar turns green", this, SLOT(updateThresholds()));
  high_threshold_property_->setMin(0.0f);
  high_threshold_property_->setMax(1.0f);

  low_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Medium Certainty Threshold", low_threshold_,
    "Certainty above this value is shown as amber", this, SLOT(updateThresholds()));
  low_threshold_property_->setMin(0.0f);
  low_threshold_property_->setMax(1.0f);

  blink_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Blink Threshold", blink_threshold_,
    "Certainty below this value triggers blinking red warning", this, SLOT(updateThresholds()));
  blink_threshold_property_->setMin(0.0f);
  blink_threshold_property_->setMax(1.0f);

  blink_frequency_property_ = new rviz_common::properties::FloatProperty(
    "Blink Frequency", blink_frequency_,
    "Blink frequency in Hz when certainty is critical", this, SLOT(updateThresholds()));
  blink_frequency_property_->setMin(0.2f);
  blink_frequency_property_->setMax(5.0f);

  smoothing_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Smoothing Factor", smoothing_alpha_,
    "Weight of previous certainty (0 = no smoothing, 0.9 = slow response)",
    this, SLOT(updateSmoothing()));
  smoothing_alpha_property_->setMin(0.0f);
  smoothing_alpha_property_->setMax(0.95f);

  high_color_property_ = new rviz_common::properties::ColorProperty(
    "High Certainty Color", high_color_,
    "Bar color when certainty is above the high threshold", this, SLOT(updateThresholds()));

  mid_color_property_ = new rviz_common::properties::ColorProperty(
    "Medium Certainty Color", mid_color_,
    "Bar color when certainty is between medium and high thresholds", this, SLOT(updateThresholds()));

  low_color_property_ = new rviz_common::properties::ColorProperty(
    "Low Certainty Color", low_color_,
    "Bar color when certainty is below the medium threshold", this, SLOT(updateThresholds()));

  regression_high_color_property_ = new rviz_common::properties::ColorProperty(
    "Regression High Color", regression_high_color_,
    "Regression bar color when certainty is above the high threshold", this, SLOT(updateThresholds()));

  regression_mid_color_property_ = new rviz_common::properties::ColorProperty(
    "Regression Mid Color", regression_mid_color_,
    "Regression bar color when certainty is between medium and high thresholds", this, SLOT(updateThresholds()));

  regression_low_color_property_ = new rviz_common::properties::ColorProperty(
    "Regression Low Color", regression_low_color_,
    "Regression bar color when certainty is below the medium threshold", this, SLOT(updateThresholds()));

  max_variance_property_ = new rviz_common::properties::FloatProperty(
    "Max Variance", max_variance_,
    "Maximum expected variance for normalization (higher variance = lower certainty)", this, SLOT(updateThresholds()));
  max_variance_property_->setMin(0.1f);
  max_variance_property_->setMax(100.0f);

  title_text_property_ = new rviz_common::properties::StringProperty(
    "Title", title_text_,
    "Text displayed above the certainty bar (leave empty to hide)", this, SLOT(updateTitle()));

  updateTitle();
}

AttentionUncertaintyDisplay::~AttentionUncertaintyDisplay()
{
  destroyHUDOverlay();

  delete hud_width_property_;
  delete hud_height_property_;
  delete hud_left_property_;
  delete hud_top_property_;
  delete hud_alpha_property_;
  delete bg_alpha_property_;
  delete high_threshold_property_;
  delete low_threshold_property_;
  delete blink_threshold_property_;
  delete blink_frequency_property_;
  delete smoothing_alpha_property_;
  delete high_color_property_;
  delete mid_color_property_;
  delete low_color_property_;
  delete regression_high_color_property_;
  delete regression_mid_color_property_;
  delete regression_low_color_property_;
  delete max_variance_property_;
  delete title_text_property_;
}

void AttentionUncertaintyDisplay::onInitialize()
{
  MFDClass::onInitialize();

  rviz_rendering::RenderSystem::get()->prepareOverlays(context_->getSceneManager());
  createHUDOverlay();
  {
    std::lock_guard<std::mutex> lock(hud_mutex_);
    updateHUD();
    update_required_ = false;
  }
}

void AttentionUncertaintyDisplay::onEnable()
{
  MFDClass::onEnable();
  if (overlay_) {
    overlay_->show();
  }
  update_required_ = true;
}

void AttentionUncertaintyDisplay::onDisable()
{
  MFDClass::onDisable();
  if (overlay_) {
    overlay_->hide();
  }
}

void AttentionUncertaintyDisplay::processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  const double classification_certainty = std::clamp(computeClassificationCertainty(*msg), 0.0, 1.0);
  const double regression_certainty = std::clamp(computeRegressionCertainty(*msg), 0.0, 1.0);

  std::lock_guard<std::mutex> lock(hud_mutex_);

  // Reset no-data timer on message receipt
  no_data_timer_ = 0.0;

  // Smooth classification certainty
  if (!have_classification_certainty_) {
    smoothed_classification_certainty_ = classification_certainty;
    have_classification_certainty_ = true;
  } else {
    const double clamped_factor = std::clamp(static_cast<double>(smoothing_alpha_), 0.0, 0.95);
    const double new_weight = 1.0 - clamped_factor;
    smoothed_classification_certainty_ = clamped_factor * smoothed_classification_certainty_ + new_weight * classification_certainty;
  }

  // Smooth regression certainty
  if (!have_regression_certainty_) {
    smoothed_regression_certainty_ = regression_certainty;
    have_regression_certainty_ = true;
  } else {
    const double clamped_factor = std::clamp(static_cast<double>(smoothing_alpha_), 0.0, 0.95);
    const double new_weight = 1.0 - clamped_factor;
    smoothed_regression_certainty_ = clamped_factor * smoothed_regression_certainty_ + new_weight * regression_certainty;
  }

  std::ostringstream oss;
  oss << "Classification: " << std::fixed << std::setprecision(2) << smoothed_classification_certainty_
      << " | Regression: " << std::fixed << std::setprecision(2) << smoothed_regression_certainty_;
  setStatus(rviz_common::properties::StatusProperty::Ok, "Certainty", oss.str().c_str());

  update_required_ = true;
}

void AttentionUncertaintyDisplay::update(float wall_dt, float /*ros_dt*/)
{
  if (!overlay_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(hud_mutex_);

    // Clamp wall_dt to prevent huge jumps
    const float clamped_dt = std::min(wall_dt, 0.1f);
    
    // Update no-data timer
    no_data_timer_ += clamped_dt;
    const bool no_data = no_data_timer_ >= kNoDataTimeout;
    
    // Check for warning conditions:
    // 1. No data arriving (timeout)
    // 2. Suspiciously low certainty (below 15%)
    // 3. Critical threshold (existing blink_threshold_)
    const bool suspiciously_low = have_classification_certainty_ && 
                                   (smoothed_classification_certainty_ <= kSuspiciouslyLowThreshold);
    const bool critical = have_classification_certainty_ && 
                          (smoothed_classification_certainty_ <= static_cast<double>(blink_threshold_));
    
    // Trigger blinking for any warning condition
    const bool should_blink = no_data || suspiciously_low || critical;

    if (should_blink && blink_frequency_ > kClampEpsilon) {
      blink_timer_ += clamped_dt;
      const double half_period = 0.5 / static_cast<double>(blink_frequency_);
      if (blink_timer_ >= half_period) {
        blink_state_ = !blink_state_;
        blink_timer_ = 0.0;
        update_required_ = true;
      }
    } else {
      if (blink_state_) {
        blink_state_ = false;
        update_required_ = true;
      }
      blink_timer_ = 0.0;
    }

    if (update_required_) {
      updateHUD();
      update_required_ = false;
    }
  }
}

double AttentionUncertaintyDisplay::computeClassificationCertainty(const perception_msgs::msg::ObjectList& objects) const
{
  if (objects.objects.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  std::size_t count = 0;

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
      // Treat missing classifications as uncertain
      max_probability = 0.0;
    }

    sum += std::clamp(max_probability, 0.0, 1.0);
    ++count;
  }

  if (count == 0) {
    return 0.0;
  }

  return sum / static_cast<double>(count);
}

double AttentionUncertaintyDisplay::computeRegressionCertainty(const perception_msgs::msg::ObjectList& objects) const
{
  if (objects.objects.empty()) {
    return 0.0;
  }

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
      const double certainty = std::exp(-avg_variance / static_cast<double>(max_variance_));
      sum += std::clamp(certainty, 0.0, 1.0);
      ++count;
    }
  }

  if (count == 0) {
    return 0.0;
  }

  return sum / static_cast<double>(count);
}

void AttentionUncertaintyDisplay::createHUDOverlay()
{
  static int overlay_counter = 0;
  const std::string overlay_name = "AttentionUncertaintyOverlay" + std::to_string(overlay_counter);
  const std::string panel_name = "AttentionUncertaintyPanel" + std::to_string(overlay_counter);
  const std::string material_name = "AttentionUncertaintyMaterial" + std::to_string(overlay_counter);
  overlay_counter++;

  Ogre::OverlayManager* overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  if (!overlay_mgr) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttentionUncertaintyDisplay"),
                        "Ogre OverlayManager not available");
    return;
  }

  overlay_ = overlay_mgr->create(overlay_name);

  panel_ = static_cast<Ogre::PanelOverlayElement*>(
    overlay_mgr->createOverlayElement("Panel", panel_name));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);

  panel_material_ = Ogre::MaterialManager::getSingleton().create(
    material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_material_->setReceiveShadows(false);
  panel_material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  panel_->setMaterialName(panel_material_->getName());
  overlay_->add2D(panel_);
  overlay_->hide();
}

void AttentionUncertaintyDisplay::destroyHUDOverlay()
{
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

  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_->getName());
    texture_.setNull();
  }

  if (!panel_material_.isNull()) {
    panel_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
    panel_material_.setNull();
  }
}

void AttentionUncertaintyDisplay::updateHUD()
{
  if (!panel_ || panel_material_.isNull()) {
    return;
  }

  hud_width_ = hud_width_property_->getInt();
  hud_height_ = hud_height_property_->getInt();
  hud_left_ = hud_left_property_->getInt();
  hud_top_ = hud_top_property_->getInt();

  hud_alpha_ = hud_alpha_property_->getFloat();
  bg_alpha_ = bg_alpha_property_->getFloat();
  high_threshold_ = std::clamp(high_threshold_property_->getFloat(), 0.0f, 1.0f);
  low_threshold_ = std::clamp(low_threshold_property_->getFloat(), 0.0f, high_threshold_);
  blink_threshold_ = std::clamp(blink_threshold_property_->getFloat(), 0.0f, low_threshold_);
  blink_frequency_ = std::max(blink_frequency_property_->getFloat(), kClampEpsilon);
  smoothing_alpha_ = std::clamp(smoothing_alpha_property_->getFloat(), 0.0f, 0.95f);
  high_color_ = high_color_property_->getColor();
  mid_color_ = mid_color_property_->getColor();
  low_color_ = low_color_property_->getColor();
  regression_high_color_ = regression_high_color_property_->getColor();
  regression_mid_color_ = regression_mid_color_property_->getColor();
  regression_low_color_ = regression_low_color_property_->getColor();
  max_variance_ = std::max(max_variance_property_->getFloat(), 0.1f);

  panel_->setPosition(static_cast<Ogre::Real>(hud_left_), static_cast<Ogre::Real>(hud_top_));
  panel_->setDimensions(static_cast<Ogre::Real>(hud_width_), static_cast<Ogre::Real>(hud_height_));

  if (hud_width_ <= 0) {
    hud_width_ = 1;
  }
  if (hud_height_ <= 0) {
    hud_height_ = 1;
  }

  const std::string texture_name = panel_material_->getName() + "Texture";

  const bool recreate_texture = texture_.isNull() ||
    texture_->getWidth() != static_cast<unsigned int>(hud_width_) ||
    texture_->getHeight() != static_cast<unsigned int>(hud_height_);

  if (recreate_texture) {
    if (!texture_.isNull()) {
      Ogre::TextureManager::getSingleton().remove(texture_->getName());
      panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }

    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      static_cast<Ogre::uint>(hud_width_),
      static_cast<Ogre::uint>(hud_height_),
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_DYNAMIC);

    panel_material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);
    panel_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    panel_material_->setCullingMode(Ogre::CULL_NONE);
  }

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pixel_box = pixel_buffer->getCurrentLock();

  QImage hud_image(static_cast<uchar*>(pixel_box.data), hud_width_, hud_height_, QImage::Format_ARGB32);

  // Clear to transparent first
  hud_image.fill(Qt::transparent);

  QPainter painter(&hud_image);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const int margin = 12;
  const QRectF frame_rect(margin, margin, hud_width_ - 2 * margin, hud_height_ - 2 * margin);

  // Draw gradient background matching graph plot style
  QLinearGradient bg_gradient(0, 0, 0, hud_height_);
  bg_gradient.setColorAt(0.0, QColor(15, 25, 40, static_cast<int>(bg_alpha_ * 255 * 0.95)));
  bg_gradient.setColorAt(0.5, QColor(20, 35, 55, static_cast<int>(bg_alpha_ * 255)));
  bg_gradient.setColorAt(1.0, QColor(10, 20, 35, static_cast<int>(bg_alpha_ * 255 * 0.9)));
  
  QPainterPath bg_path;
  bg_path.addRoundedRect(frame_rect, 8, 8);
  painter.fillPath(bg_path, bg_gradient);
  
  // Subtle border glow (matching graph plot)
  QPen border_pen(QColor(60, 140, 200, static_cast<int>(hud_alpha_ * 80)));
  border_pen.setWidth(1);
  painter.setPen(border_pen);
  painter.drawPath(bg_path);
  
  // Inner border
  QPainterPath inner_path;
  inner_path.addRoundedRect(QRectF(margin + 1, margin + 1, hud_width_ - 2 * margin - 2, hud_height_ - 2 * margin - 2), 7, 7);
  painter.setPen(QPen(QColor(40, 80, 120, static_cast<int>(hud_alpha_ * 60)), 1));
  painter.drawPath(inner_path);

  QColor frame_color(180, 210, 240);
  frame_color.setAlpha(static_cast<int>(hud_alpha_ * 255));

  QFont base_font = painter.font();

  // Title (optional)
  int bar_top = margin + 40;
  if (!title_text_.isEmpty()) {
    QFont title_font = base_font;
    title_font.setPointSize(14);
    title_font.setBold(true);
    painter.setFont(title_font);
    painter.setPen(frame_color);
    painter.drawText(QRectF(margin, margin, hud_width_ - 2 * margin, 26),
                     Qt::AlignHCenter | Qt::AlignVCenter, title_text_);
    bar_top = margin + 34;
    painter.setFont(base_font);
  }

  // Bar geometry for dual bars
  const int bar_spacing = 16;  // Space between bars
  const int bar_width = std::max(28, (hud_width_ - 2 * margin - bar_spacing * 3) / 3);
  const int bar_bottom_margin = 72;  // More space for labels
  const int bar_height = std::max(60, hud_height_ - bar_top - margin - bar_bottom_margin);
  
  // Calculate positions for two centered bars
  const int total_width = 2 * bar_width + bar_spacing;
  const int start_x = (hud_width_ - total_width) / 2;
  const int class_bar_x = start_x;
  const int regr_bar_x = start_x + bar_width + bar_spacing;
  const int bar_y = bar_top + 6;

  // Check warning conditions
  const bool no_data = no_data_timer_ >= kNoDataTimeout;
  const bool suspiciously_low_cls = have_classification_certainty_ && 
                                     (smoothed_classification_certainty_ <= kSuspiciouslyLowThreshold);
  const bool suspiciously_low_reg = have_regression_certainty_ && 
                                     (smoothed_regression_certainty_ <= kSuspiciouslyLowThreshold);

  // Helper lambda to draw a bar with outline and fill
  auto drawBar = [&](int bar_x, double certainty, bool have_data, 
                     const QColor& high_col, const QColor& mid_col, const QColor& low_col,
                     bool is_classification, bool show_warning) {
    // Bar outline - red border when warning
    const QRect bar_outline(bar_x, bar_y, bar_width, bar_height);
    QColor bar_outline_color = frame_color;
    
    // Red pulsing border when warning condition
    if (show_warning && blink_state_) {
      bar_outline_color = low_color_;
      bar_outline_color.setAlpha(255);
    } else if (show_warning) {
      bar_outline_color = low_color_;
      bar_outline_color.setAlpha(static_cast<int>(hud_alpha_ * 150));
    } else {
      bar_outline_color.setAlpha(static_cast<int>(hud_alpha_ * 200));
    }
    
    painter.setPen(QPen(bar_outline_color, show_warning ? 3 : 2));
    painter.setBrush(QColor(255, 255, 255, 25));
    painter.drawRoundedRect(bar_outline, 8, 8);

    const double cert = have_data ? std::clamp(certainty, 0.0, 1.0) : 0.0;
    const int filled_height = static_cast<int>(cert * static_cast<double>(bar_height));
    const int fill_top = bar_y + bar_height - filled_height;
    const QRect filled_rect(bar_x + 3, fill_top + 3, bar_width - 6, filled_height - 6);

    if (filled_rect.height() > 0) {
      const bool blink_on = blink_state_ && (show_warning || cert <= static_cast<double>(blink_threshold_));
      
      // Determine bar color based on certainty
      QColor bar_color;
      if (show_warning) {
        // Force red when warning
        bar_color = low_col;
      } else if (cert >= static_cast<double>(high_threshold_)) {
        bar_color = high_col;
      } else if (cert >= static_cast<double>(low_threshold_)) {
        bar_color = mid_col;
      } else {
        bar_color = low_col;
      }
      
      if (blink_on) {
        bar_color = bar_color.lighter(blink_state_ ? 180 : 60);
      }
      bar_color.setAlpha(static_cast<int>(hud_alpha_ * 255));

      QLinearGradient gradient(filled_rect.left(), filled_rect.bottom(), filled_rect.right(), filled_rect.top());
      gradient.setColorAt(0.0, bar_color.darker(120));
      gradient.setColorAt(1.0, bar_color.lighter(130));

      painter.setBrush(gradient);
      painter.setPen(Qt::NoPen);
      painter.drawRoundedRect(filled_rect, 6, 6);
    }

    // Draw exclamation mark warning when conditions are met
    if (show_warning) {
      const int center_x = bar_x + bar_width / 2;
      const int center_y = bar_y + bar_height / 2;
      
      // Exclamation mark triangle background (blinking)
      if (blink_state_) {
        QColor warning_bg(255, 70, 70, 220);
        painter.setBrush(warning_bg);
        painter.setPen(QPen(QColor(255, 200, 200), 2));
        
        // Draw warning triangle
        QPolygonF triangle;
        const int tri_size = std::min(bar_width - 8, 24);
        triangle << QPointF(center_x, center_y - tri_size / 2)
                 << QPointF(center_x - tri_size / 2, center_y + tri_size / 3)
                 << QPointF(center_x + tri_size / 2, center_y + tri_size / 3);
        painter.drawPolygon(triangle);
        
        // Draw exclamation mark
        QFont warn_font = base_font;
        warn_font.setPointSize(12);
        warn_font.setBold(true);
        painter.setFont(warn_font);
        painter.setPen(QColor(255, 255, 255, 255));
        painter.drawText(QRectF(center_x - 10, center_y - tri_size / 2, 20, tri_size),
                         Qt::AlignHCenter | Qt::AlignVCenter, QStringLiteral("!"));
      }
    }

    // Threshold ticks
    painter.setPen(QPen(QColor(255, 255, 255, 100), 1, Qt::DashLine));
    const auto threshold_to_y = [&](float threshold) {
      return bar_y + bar_height - static_cast<int>(threshold * bar_height);
    };

    const int high_y = threshold_to_y(high_threshold_);
    painter.drawLine(bar_x + 2, high_y, bar_x + bar_width - 2, high_y);

    const int mid_y = threshold_to_y(low_threshold_);
    painter.drawLine(bar_x + 2, mid_y, bar_x + bar_width - 2, mid_y);
  };

  // Draw classification bar (left) - warning if no data or suspiciously low
  const bool cls_warning = no_data || suspiciously_low_cls;
  drawBar(class_bar_x, smoothed_classification_certainty_, have_classification_certainty_,
          high_color_, mid_color_, low_color_, true, cls_warning);

  // Draw regression bar (right) - warning if no data or suspiciously low
  const bool reg_warning = no_data || suspiciously_low_reg;
  drawBar(regr_bar_x, smoothed_regression_certainty_, have_regression_certainty_,
          regression_high_color_, regression_mid_color_, regression_low_color_, false, reg_warning);

  // Bar labels
  QFont label_font = base_font;
  label_font.setPointSize(9);
  label_font.setBold(true);
  painter.setFont(label_font);
  
  QColor label_color = frame_color;
  label_color.setAlpha(200);
  painter.setPen(label_color);

  // Classification label
  painter.drawText(QRectF(class_bar_x - 10, bar_y + bar_height + 4, bar_width + 20, 16),
                   Qt::AlignHCenter | Qt::AlignVCenter, QStringLiteral("CLS"));

  // Regression label
  painter.drawText(QRectF(regr_bar_x - 10, bar_y + bar_height + 4, bar_width + 20, 16),
                   Qt::AlignHCenter | Qt::AlignVCenter, QStringLiteral("REG"));

  // Percentage values
  QFont value_font = painter.font();
  value_font.setPointSize(12);
  value_font.setBold(true);
  painter.setFont(value_font);
  QColor value_color = frame_color;
  value_color.setAlpha(255);
  painter.setPen(value_color);

  const double class_percentage = smoothed_classification_certainty_ * 100.0;
  const double regr_percentage = smoothed_regression_certainty_ * 100.0;

  // Show "NO DATA" in red if no data warning is active
  QString class_text;
  QString regr_text;

  if (no_data) {
    class_text = QStringLiteral("N/A");
    regr_text = QStringLiteral("N/A");
    // Draw in red for no data
    painter.setPen(blink_state_ ? QColor(255, 80, 80) : QColor(180, 60, 60));
  } else {
    class_text = have_classification_certainty_
      ? QString::number(class_percentage, 'f', 0) + QLatin1String("%")
      : QStringLiteral("--%");

    regr_text = have_regression_certainty_
      ? QString::number(regr_percentage, 'f', 0) + QLatin1String("%")
      : QStringLiteral("--%");
    
    // Orange/red if suspiciously low
    if (suspiciously_low_cls) {
      painter.setPen(blink_state_ ? QColor(255, 150, 50) : QColor(180, 100, 40));
    }
  }

  painter.drawText(QRectF(class_bar_x - 10, bar_y + bar_height + 20, bar_width + 20, 20),
                   Qt::AlignHCenter | Qt::AlignVCenter, class_text);

  // Reset pen for regression if not in warning state
  if (no_data) {
    painter.setPen(blink_state_ ? QColor(255, 80, 80) : QColor(180, 60, 60));
  } else if (suspiciously_low_reg) {
    painter.setPen(blink_state_ ? QColor(255, 150, 50) : QColor(180, 100, 40));
  } else {
    painter.setPen(value_color);
  }

  painter.drawText(QRectF(regr_bar_x - 10, bar_y + bar_height + 20, bar_width + 20, 20),
                   Qt::AlignHCenter | Qt::AlignVCenter, regr_text);

  // Warning message at the bottom if any warning is active
  if (no_data || suspiciously_low_cls || suspiciously_low_reg) {
    QFont warning_font = base_font;
    warning_font.setPointSize(9);
    warning_font.setBold(true);
    painter.setFont(warning_font);
    
    QString warning_msg;
    QColor warning_text_color;
    
    if (no_data) {
      warning_msg = QStringLiteral("⚠ NO DATA!");
      warning_text_color = blink_state_ ? QColor(255, 80, 80) : QColor(180, 60, 60);
    } else {
      warning_msg = QStringLiteral("⚠ Uncertain!");
      warning_text_color = blink_state_ ? QColor(255, 150, 50) : QColor(180, 100, 40);
    }
    
    painter.setPen(warning_text_color);
    const int warning_y = bar_y + bar_height + 42;
    painter.drawText(QRectF(margin, warning_y, hud_width_ - 2 * margin, 16),
                     Qt::AlignHCenter | Qt::AlignVCenter, warning_msg);
  }

  painter.end();
  pixel_buffer->unlock();
}

QColor AttentionUncertaintyDisplay::barColorForCertainty(double certainty, bool blink_on) const
{
  QColor color;

  if (certainty >= static_cast<double>(high_threshold_)) {
    color = high_color_;
  } else if (certainty >= static_cast<double>(low_threshold_)) {
    color = mid_color_;
  } else {
    color = low_color_;
  }

  if (blink_on) {
    return color.lighter(blink_state_ ? 180 : 60);
  }

  return color;
}

void AttentionUncertaintyDisplay::updatePosition()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateSize()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateTransparency()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateThresholds()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateSmoothing()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  smoothing_alpha_ = std::clamp(smoothing_alpha_property_->getFloat(), 0.0f, 0.95f);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateTitle()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  title_text_ = QString::fromStdString(title_text_property_->getStdString());
  update_required_ = true;
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionUncertaintyDisplay, rviz_common::Display)
