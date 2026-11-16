// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include "perception_msgs/displays/image_observations/image_observations_widget.hpp"

#include <utility>

#include <QPainter>

namespace perception_msgs {
namespace displays {

ImageObservationsWidget::ImageObservationsWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(160, 90);
  setAttribute(Qt::WA_OpaquePaintEvent);
  frame_.message = QStringLiteral("Waiting for data");
}

void ImageObservationsWidget::setFrame(ImageObservationsFrame frame) {
  frame_ = std::move(frame);
  update();
}

QSize ImageObservationsWidget::sizeHint() const { return QSize(640, 360); }

void ImageObservationsWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.fillRect(rect(), Qt::black);

  if (!frame_.source_size.isValid() || frame_.source_size.isEmpty()) {
    painter.setPen(QColor(160, 160, 160));
    painter.drawText(rect(), Qt::AlignCenter, frame_.message);
    return;
  }

  // letterbox: largest rectangle with the source aspect ratio, centered
  const QSizeF fitted = QSizeF(frame_.source_size).scaled(QSizeF(size()), Qt::KeepAspectRatio);
  const QRectF target(QPointF((width() - fitted.width()) / 2.0, (height() - fitted.height()) / 2.0), fitted);

  if (frame_.image.isNull()) {
    painter.fillRect(target, QColor(30, 30, 30));
  } else {
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.setOpacity(frame_.image_opacity);
    painter.drawImage(target, frame_.image);
    painter.setOpacity(1.0);
  }
  rendering::paintImageObservations(painter, target, frame_.annotations, frame_.style);

  if (!frame_.message.isEmpty()) {
    painter.setPen(QColor(230, 230, 230));
    painter.drawText(target.adjusted(6, 4, -6, -4), Qt::AlignBottom | Qt::AlignLeft, frame_.message);
  }
}

}  // namespace displays
}  // namespace perception_msgs
