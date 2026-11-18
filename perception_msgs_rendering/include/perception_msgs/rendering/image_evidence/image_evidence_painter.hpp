// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <QColor>
#include <QImage>
#include <QPainter>
#include <QRectF>
#include <QString>

#include <perception_msgs/msg/evidence.hpp>
#include <perception_msgs/msg/object_state.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace perception_msgs {
namespace rendering {

/** @brief How instance masks are drawn. */
enum class MaskMode : uint8_t { Filled = 0, Contour = 1, FilledAndContour = 2 };

/** @brief Appearance of image evidence annotations. Sizes are in canvas (device) pixels. */
struct ImageEvidenceStyle {
  bool draw_box = true;
  double box_line_width = 2.0;
  double box_fill_alpha = 0.0;

  bool draw_mask = true;
  MaskMode mask_mode = MaskMode::FilledAndContour;
  double mask_alpha = 0.4;
  double contour_line_width = 1.5;

  bool draw_crop = false;  ///< paste ImageEvidence::crop at its ROI
  bool draw_roi = false;   ///< draw the crop/mask source rectangle

  bool draw_label = true;
  int font_pixel_size = 13;
  double label_background_alpha = 0.75;

  double stale_opacity = 0.45;  ///< opacity multiplier for stale annotations
};

/** @brief One image evidence to draw, with its precomputed appearance. */
struct ImageEvidenceAnnotation {
  const perception_msgs::msg::Evidence* evidence = nullptr;
  QColor color;
  QString label;
  bool stale = false;  ///< drawn dashed and faded, e.g. when its stamp does not match the canvas image
};

/**
 * @brief Convert a packed sensor_msgs/Image into a deep-copied QImage.
 *
 * Supported: rgb8, bgr8, rgba8, bgra8, mono8, mono16, 8UC1, 8UC3 (as bgr8), 8UC4 (as bgra8), 16UC1,
 * uyvy/yuv422, yuyv/yuv422_yuy2. Returns a null QImage and fills @p error for anything else.
 */
QImage toQImage(const sensor_msgs::msg::Image& image, std::string* error = nullptr);

/**
 * @brief Paint annotations into @p target.
 *
 * Image evidence geometry is defined in source image pixels (image_width x image_height); each annotation
 * is scaled from its own source size onto @p target, so the canvas may have any resolution.
 * Bounding box centers follow the message convention (pixel centers, x right, y down).
 */
void paintImageEvidence(QPainter& painter, const QRectF& target,
                        const std::vector<ImageEvidenceAnnotation>& annotations, const ImageEvidenceStyle& style);

/** @brief Human-readable name of an ObjectClassification type. */
QString classificationName(uint8_t type);

/** @brief Default class color, identical to the ObjectList display defaults. */
QColor defaultClassificationColor(uint8_t type);

/** @brief Object class label, including traffic-sign type or traffic-light state. */
QString objectClassName(const perception_msgs::msg::ObjectState& state);

/** @brief Class color for road users, traffic signs and traffic lights. */
QColor defaultObjectColor(const perception_msgs::msg::ObjectState& state);

/** @brief Stable, well-separated color for an integer key such as an object or sensor ID. */
QColor colorForId(uint64_t id);

}  // namespace rendering
}  // namespace perception_msgs
