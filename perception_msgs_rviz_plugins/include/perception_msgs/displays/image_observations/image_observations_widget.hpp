// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <vector>

#include <QImage>
#include <QSize>
#include <QString>
#include <QWidget>

#include <perception_msgs/msg/object_list.hpp>

#include "perception_msgs/rendering/image_observation/image_observation_painter.hpp"

namespace perception_msgs {
namespace displays {

/** @brief Everything needed to draw one frame of the image observations view. */
struct ImageObservationsFrame {
  QImage image;       ///< canvas background; null draws a blank canvas
  double image_opacity = 1.0;
  QSize source_size;  ///< canvas size in source pixels; invalid shows @ref message instead
  std::vector<rendering::ImageObservationAnnotation> annotations;
  rendering::ImageObservationStyle style;
  perception_msgs::msg::ObjectList::ConstSharedPtr keep_alive;  ///< owns the observations the annotations point to
  QString message;    ///< shown while there is nothing to draw
};

/**
 * \class ImageObservationsWidget
 * \brief 2D view of image observations, shown in its own (dockable or floating) RViz panel.
 *
 * Draws at the widget's resolution and keeps the source aspect ratio (letterboxed).
 */
class ImageObservationsWidget : public QWidget {
public:
  explicit ImageObservationsWidget(QWidget* parent = nullptr);

  /** @brief Replace the shown frame and schedule a repaint. */
  void setFrame(ImageObservationsFrame frame);

  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  ImageObservationsFrame frame_;
};

}  // namespace displays
}  // namespace perception_msgs
