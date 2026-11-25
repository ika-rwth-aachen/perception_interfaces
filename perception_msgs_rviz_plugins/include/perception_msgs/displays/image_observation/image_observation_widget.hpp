// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <vector>

#include <QImage>
#include <QSize>
#include <QString>
#include <QWidget>

#include <perception_msgs/msg/object_list_with_observation.hpp>

#include "perception_msgs/rendering/image_observation/image_observation_painter.hpp"

namespace perception_msgs {
namespace displays {

/** @brief Everything needed to draw one frame of the image observation view. */
struct ImageObservationFrame {
  QImage image;       ///< canvas background; null draws a blank canvas
  double image_opacity = 1.0;
  QSize source_size;  ///< canvas size in source pixels; invalid shows @ref message instead
  std::vector<rendering::ImageObservationAnnotation> annotations;
  rendering::ImageObservationStyle style;
  perception_msgs::msg::ObjectListWithObservation::ConstSharedPtr keep_alive;  ///< owns the observation the annotations point to
  QString message;    ///< shown while there is nothing to draw
};

/**
 * \class ImageObservationWidget
 * \brief 2D view of image observation, shown in its own (dockable or floating) RViz panel.
 *
 * Draws at the widget's resolution and keeps the source aspect ratio (letterboxed).
 */
class ImageObservationWidget : public QWidget {
public:
  explicit ImageObservationWidget(QWidget* parent = nullptr);

  /** @brief Replace the shown frame and schedule a repaint. */
  void setFrame(ImageObservationFrame frame);

  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  ImageObservationFrame frame_;
};

}  // namespace displays
}  // namespace perception_msgs
