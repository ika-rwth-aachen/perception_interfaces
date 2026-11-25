// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include "perception_msgs/rendering/image_observation/image_observation_painter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <QFont>
#include <QFontMetricsF>
#include <QLineF>
#include <QPen>
#include <QPolygonF>
#include <QSysInfo>
#include <QTransform>
#include <QVector>

#include <perception_msgs/msg/object_classification.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include "perception_msgs/rendering/object_state/object_state.hpp"
#include "perception_msgs/rendering/traffic_sign/traffic_sign.hpp"

namespace perception_msgs {
namespace rendering {

namespace {

using perception_msgs::object_access::ImageObservationView;
using perception_msgs::msg::ObjectClassification;

bool hostIsBigEndian() { return QSysInfo::ByteOrder == QSysInfo::BigEndian; }

QImage wrapAndCopy(const sensor_msgs::msg::Image& image, QImage::Format format) {
  return QImage(image.data.data(), static_cast<int>(image.width), static_cast<int>(image.height),
                static_cast<qsizetype>(image.step), format)
      .copy();
}

inline uint8_t clampByte(int value) { return static_cast<uint8_t>(std::clamp(value, 0, 255)); }

inline QRgb yuvToRgb(int y, int u, int v) {
  // BT.601, limited range
  const int c = y - 16, d = u - 128, e = v - 128;
  return qRgb(clampByte((298 * c + 409 * e + 128) >> 8), clampByte((298 * c - 100 * d - 208 * e + 128) >> 8),
              clampByte((298 * c + 516 * d + 128) >> 8));
}

QImage convertYuv422(const sensor_msgs::msg::Image& image, bool uyvy) {
  QImage out(static_cast<int>(image.width), static_cast<int>(image.height), QImage::Format_RGB32);
  for (uint32_t row = 0; row < image.height; ++row) {
    const uint8_t* src = image.data.data() + static_cast<size_t>(row) * image.step;
    auto* dst = reinterpret_cast<QRgb*>(out.scanLine(static_cast<int>(row)));
    for (uint32_t col = 0; col + 1 < image.width; col += 2, src += 4) {
      const int y0 = uyvy ? src[1] : src[0], y1 = uyvy ? src[3] : src[2];
      const int u = uyvy ? src[0] : src[1], v = uyvy ? src[2] : src[3];
      dst[col] = yuvToRgb(y0, u, v);
      dst[col + 1] = yuvToRgb(y1, u, v);
    }
    // odd widths have no complete macro pixel for the last column
    if (image.width % 2) dst[image.width - 1] = image.width > 1 ? dst[image.width - 2] : qRgb(0, 0, 0);
  }
  return out;
}

QImage convertMono16(const sensor_msgs::msg::Image& image) {
  const bool swap = static_cast<bool>(image.is_bigendian) != hostIsBigEndian();
  if (!swap) return wrapAndCopy(image, QImage::Format_Grayscale16);
  QImage out(static_cast<int>(image.width), static_cast<int>(image.height), QImage::Format_Grayscale16);
  for (uint32_t row = 0; row < image.height; ++row) {
    const uint8_t* src = image.data.data() + static_cast<size_t>(row) * image.step;
    auto* dst = reinterpret_cast<uint16_t*>(out.scanLine(static_cast<int>(row)));
    for (uint32_t col = 0; col < image.width; ++col) {
      dst[col] = image.is_bigendian
                     ? static_cast<uint16_t>((src[2 * col] << 8) | src[2 * col + 1])
                     : static_cast<uint16_t>(src[2 * col] | (src[2 * col + 1] << 8));
    }
  }
  return out;
}

QImage convertBgra(const sensor_msgs::msg::Image& image) {
  // Format_ARGB32 stores B,G,R,A in memory on little-endian hosts
  if (!hostIsBigEndian()) return wrapAndCopy(image, QImage::Format_ARGB32);
  QImage out(static_cast<int>(image.width), static_cast<int>(image.height), QImage::Format_ARGB32);
  for (uint32_t row = 0; row < image.height; ++row) {
    const uint8_t* src = image.data.data() + static_cast<size_t>(row) * image.step;
    auto* dst = reinterpret_cast<QRgb*>(out.scanLine(static_cast<int>(row)));
    for (uint32_t col = 0; col < image.width; ++col, src += 4) dst[col] = qRgba(src[2], src[1], src[0], src[3]);
  }
  return out;
}

/** @brief Maps source-image pixel coordinates (pixel edges, Qt convention) onto the canvas target. */
QTransform sourceToTarget(const ImageObservationView& image, const QRectF& target) {
  QTransform transform;
  transform.translate(target.x(), target.y());
  transform.scale(target.width() / static_cast<double>(image.image_width),
                  target.height() / static_cast<double>(image.image_height));
  return transform;
}

bool hasRoi(const ImageObservationView& image) { return image.roi.width > 0 && image.roi.height > 0; }

QRectF roiRect(const ImageObservationView& image) {
  return QRectF(image.roi.x_offset, image.roi.y_offset, image.roi.width, image.roi.height);
}

QPolygonF boxPolygon(const ImageObservationView& image) {
  const auto& box = image.bounding_box;
  // message uses pixel centers; Qt geometry uses pixel edges
  const double cx = box.center.position.x + 0.5, cy = box.center.position.y + 0.5;
  const double hx = 0.5 * box.size_x, hy = 0.5 * box.size_y;
  const double c = std::cos(box.center.theta), s = std::sin(box.center.theta);
  QPolygonF polygon;
  for (const auto& [dx, dy] : {std::pair{-hx, -hy}, std::pair{hx, -hy}, std::pair{hx, hy}, std::pair{-hx, hy}})
    polygon << QPointF(cx + c * dx - s * dy, cy + s * dx + c * dy);
  return polygon;
}

bool maskOn(const sensor_msgs::msg::Image& mask, uint32_t u, uint32_t v) {
  return mask.data[static_cast<size_t>(v) * mask.step + u] >= 128;
}

void paintMask(QPainter& painter, const ImageObservationView& image, const QTransform& transform, const QColor& color,
               const ImageObservationStyle& style, double opacity, bool stale) {
  const auto& mask = image.mask;
  if (mask.data.empty() || mask.encoding != "mono8" || !hasRoi(image) || mask.width == 0 || mask.height == 0 ||
      mask.step < mask.width || static_cast<size_t>(mask.step) * mask.height > mask.data.size())
    return;

  const QRectF target_roi = transform.mapRect(roiRect(image));

  if (style.mask_mode != MaskMode::Contour && style.mask_alpha > 0.0) {
    QColor fill = color;
    fill.setAlphaF(std::clamp(style.mask_alpha * opacity, 0.0, 1.0));
    const QRgb on = qPremultiply(fill.rgba());
    QImage colored(static_cast<int>(mask.width), static_cast<int>(mask.height), QImage::Format_ARGB32_Premultiplied);
    for (uint32_t v = 0; v < mask.height; ++v) {
      auto* dst = reinterpret_cast<QRgb*>(colored.scanLine(static_cast<int>(v)));
      for (uint32_t u = 0; u < mask.width; ++u) dst[u] = maskOn(mask, u, v) ? on : 0u;
    }
    painter.save();
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter.drawImage(target_roi, colored);
    painter.restore();
  }

  if (style.mask_mode != MaskMode::Filled) {
    // staircase outline along mask pixel edges; stays crisp at any canvas scale
    const double kx = static_cast<double>(image.roi.width) / mask.width;
    const double ky = static_cast<double>(image.roi.height) / mask.height;
    const double ox = image.roi.x_offset, oy = image.roi.y_offset;
    QVector<QLineF> lines;
    const auto edge = [&](double u0, double v0, double u1, double v1) {
      lines.append(transform.map(QLineF(ox + u0 * kx, oy + v0 * ky, ox + u1 * kx, oy + v1 * ky)));
    };
    for (uint32_t v = 0; v < mask.height; ++v) {
      for (uint32_t u = 0; u < mask.width; ++u) {
        if (!maskOn(mask, u, v)) continue;
        if (u == 0 || !maskOn(mask, u - 1, v)) edge(u, v, u, v + 1);
        if (u + 1 == mask.width || !maskOn(mask, u + 1, v)) edge(u + 1, v, u + 1, v + 1);
        if (v == 0 || !maskOn(mask, u, v - 1)) edge(u, v, u + 1, v);
        if (v + 1 == mask.height || !maskOn(mask, u, v + 1)) edge(u, v + 1, u + 1, v + 1);
      }
    }
    QColor line_color = color;
    line_color.setAlphaF(std::clamp(opacity, 0.0, 1.0));
    QPen pen(line_color, style.contour_line_width);
    pen.setCapStyle(Qt::SquareCap);
    if (stale) pen.setStyle(Qt::DotLine);
    painter.setPen(pen);
    painter.drawLines(lines);
  }
}

void paintLabel(QPainter& painter, const QRectF& anchor, const QRectF& target, const QString& text,
                const QColor& color, const ImageObservationStyle& style, double opacity) {
  if (text.isEmpty()) return;
  QFont font = painter.font();
  font.setPixelSize(std::max(6, style.font_pixel_size));
  font.setBold(true);
  painter.setFont(font);
  const QFontMetricsF metrics(font);
  const double pad = 0.25 * metrics.height();
  const QSizeF size(metrics.horizontalAdvance(text) + 2 * pad, metrics.height() + pad);

  // above the box if possible, otherwise inside its top edge; always inside the canvas
  double x = std::clamp(anchor.left(), target.left(), std::max(target.left(), target.right() - size.width()));
  double y = anchor.top() - size.height();
  if (y < target.top()) y = std::max(target.top(), anchor.top());
  const QRectF rect(QPointF(x, y), size);

  QColor background = color;
  background.setAlphaF(std::clamp(style.label_background_alpha * opacity, 0.0, 1.0));
  painter.setPen(Qt::NoPen);
  painter.setBrush(background);
  painter.drawRect(rect);

  // pick black or white text depending on background luminance
  const double luminance = 0.299 * color.redF() + 0.587 * color.greenF() + 0.114 * color.blueF();
  QColor text_color = luminance > 0.6 ? QColor(0, 0, 0) : QColor(255, 255, 255);
  text_color.setAlphaF(std::clamp(opacity, 0.0, 1.0));
  painter.setPen(text_color);
  painter.drawText(rect.adjusted(pad, 0, -pad, 0), Qt::AlignVCenter | Qt::AlignLeft, text);
}

}  // namespace

QImage toQImage(const sensor_msgs::msg::Image& image, std::string* error) {
  const auto fail = [&](const std::string& message) {
    if (error) *error = message;
    return QImage();
  };
  if (image.width == 0 || image.height == 0 || image.data.empty()) return fail("empty image");
  if (static_cast<size_t>(image.step) * image.height > image.data.size()) return fail("image buffer smaller than step * height");

  const std::string& enc = image.encoding;
  const auto require_step = [&](uint32_t bytes_per_pixel) { return image.step >= image.width * bytes_per_pixel; };

  if ((enc == "rgb8") && require_step(3)) return wrapAndCopy(image, QImage::Format_RGB888);
  if ((enc == "bgr8" || enc == "8UC3") && require_step(3)) return wrapAndCopy(image, QImage::Format_BGR888);
  if ((enc == "rgba8") && require_step(4)) return wrapAndCopy(image, QImage::Format_RGBA8888);
  if ((enc == "bgra8" || enc == "8UC4") && require_step(4)) return convertBgra(image);
  if ((enc == "mono8" || enc == "8UC1") && require_step(1)) return wrapAndCopy(image, QImage::Format_Grayscale8);
  if ((enc == "mono16" || enc == "16UC1") && require_step(2)) return convertMono16(image);
  if ((enc == "uyvy" || enc == "yuv422") && require_step(2)) return convertYuv422(image, true);
  if ((enc == "yuyv" || enc == "yuv422_yuy2") && require_step(2)) return convertYuv422(image, false);
  return fail("unsupported encoding or step for '" + enc + "'");
}

void paintImageObservation(QPainter& painter, const QRectF& target,
                        const std::vector<ImageObservationAnnotation>& annotations, const ImageObservationStyle& style) {
  painter.save();
  painter.setClipRect(target);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);

  // layer by layer so that labels are never covered by another object's mask or box
  const auto valid = [](const ImageObservationAnnotation& a) {
    if (!a.observation) return false;
    try { object_access::imageView(*a.observation); return true; }
    catch (const std::exception&) { return false; }
  };
  if (style.draw_crop) {
    for (const auto& a : annotations) {
      if (!valid(a)) continue;
      const auto image = object_access::imageView(*a.observation);
      if (image.crop.data.empty() || !hasRoi(image)) continue;
      const QImage crop = toQImage(image.crop);
      if (crop.isNull()) continue;
      painter.save();
      painter.setOpacity(a.stale ? style.stale_opacity : 1.0);
      painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
      painter.drawImage(sourceToTarget(image, target).mapRect(roiRect(image)), crop);
      painter.restore();
    }
  }

  if (style.draw_mask) {
    for (const auto& a : annotations) {
      if (!valid(a)) continue;
      const auto image = object_access::imageView(*a.observation);
      paintMask(painter, image, sourceToTarget(image, target), a.color, style,
                a.stale ? style.stale_opacity : 1.0, a.stale);
    }
  }

  for (const auto& a : annotations) {
    if (!valid(a)) continue;
    const auto image = object_access::imageView(*a.observation);
    const double opacity = a.stale ? style.stale_opacity : 1.0;
    const QTransform transform = sourceToTarget(image, target);

    if (style.draw_roi && hasRoi(image)) {
      QColor roi_color = a.color;
      roi_color.setAlphaF(0.8 * opacity);
      QPen pen(roi_color, 1.0, Qt::DashLine);
      painter.setPen(pen);
      painter.setBrush(Qt::NoBrush);
      painter.drawRect(transform.mapRect(roiRect(image)));
    }

    if (style.draw_box) {
      QColor line_color = a.color;
      line_color.setAlphaF(opacity);
      QPen pen(line_color, style.box_line_width);
      pen.setJoinStyle(Qt::MiterJoin);
      if (a.stale) pen.setStyle(Qt::DashLine);
      QColor fill = a.color;
      fill.setAlphaF(std::clamp(style.box_fill_alpha * opacity, 0.0, 1.0));
      painter.setPen(pen);
      painter.setBrush(style.box_fill_alpha > 0.0 ? QBrush(fill) : QBrush(Qt::NoBrush));
      painter.drawPolygon(transform.map(boxPolygon(image)));
    }
  }

  if (style.draw_label) {
    for (const auto& a : annotations) {
      if (!valid(a)) continue;
      const auto image = object_access::imageView(*a.observation);
      const QRectF anchor = sourceToTarget(image, target).map(boxPolygon(image)).boundingRect();
      paintLabel(painter, anchor, target, a.label, a.color, style, a.stale ? style.stale_opacity : 1.0);
    }
  }

  painter.restore();
}

QString classificationName(uint8_t type) {
  return QString::fromStdString(classificationTypeName(type));
}

QColor defaultClassificationColor(uint8_t type) {
  switch (type) {
    case ObjectClassification::PEDESTRIAN:
    case ObjectClassification::VRU:
    case ObjectClassification::MICRO: return QColor(25, 255, 255);
    case ObjectClassification::BICYCLE:
    case ObjectClassification::MOTORCYCLE: return QColor(255, 255, 25);
    case ObjectClassification::CAR:
    case ObjectClassification::UTILITY:
    case ObjectClassification::BUS:
      return QColor(25, 25, 255);
    case ObjectClassification::ANIMAL: return QColor(0, 128, 128);
    case ObjectClassification::UNCLASSIFIED:
    case ObjectClassification::UNKNOWN: return QColor(128, 128, 128);
    default: return QColor(128, 128, 128);
  }
}

QString objectClassName(const perception_msgs::msg::ObjectState& state) {
  const auto semantic = object_access::taxonomyLabel(object_access::bestClass(state));
  if (!semantic.empty()) return QString::fromStdString(semantic);
  using Light = perception_msgs::msg::TRAFFICLIGHT;
  if (state.model_id == Light::MODEL_ID) {
    QString label = QStringLiteral("traffic light");
    if (state.discrete_state.size() != Light::DISCRETE_STATE_SIZE) return label;
    switch (perception_msgs::object_access::getTrafficLightState(state)) {
      case Light::STATE_RED: label += " red"; break;
      case Light::STATE_RED_YELLOW: label += " red/yellow"; break;
      case Light::STATE_YELLOW: label += " yellow"; break;
      case Light::STATE_GREEN: label += " green"; break;
      case Light::STATE_OFF: label += " off"; break;
      default: break;
    }
    switch (perception_msgs::object_access::getTrafficLightType(state)) {
      case Light::TYPE_STRAIGHT: label += " straight"; break;
      case Light::TYPE_LEFT: label += " left"; break;
      case Light::TYPE_RIGHT: label += " right"; break;
      default: break;
    }
    return label;
  }
  return classificationName(perception_msgs::object_access::getClassWithHighestProbability(state).type);
}

QColor defaultObjectColor(const perception_msgs::msg::ObjectState& state) {
  if (object_access::isTrafficSignClass(object_access::bestClass(state))) return QColor(255, 80, 80);
  if (state.model_id == perception_msgs::msg::TRAFFICLIGHT::MODEL_ID) return QColor(255, 190, 0);
  return defaultClassificationColor(perception_msgs::object_access::getClassWithHighestProbability(state).type);
}

QColor colorForId(uint64_t id) {
  // golden-ratio hue stepping keeps consecutive IDs visually distinct
  const double hue = std::fmod(static_cast<double>(id % 100000) * 0.618033988749895, 1.0);
  return QColor::fromHsvF(hue, 0.85, 1.0);
}

}  // namespace rendering
}  // namespace perception_msgs
