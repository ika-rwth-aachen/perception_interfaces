// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <image_transport/subscriber.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "perception_msgs/displays/image_observations/image_observations_widget.hpp"
#include "perception_msgs/rendering/image_observation/image_observation_painter.hpp"

namespace rviz_common {
namespace properties {
class BoolProperty;
class ColorProperty;
class EditableEnumProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class RosTopicProperty;
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace perception_msgs {
namespace displays {

/**
 * \class ImageObservationsDisplay
 * \brief Draws the 2D image observations (boxes, masks, crops, labels) of a perception_msgs/ObjectList
 *        in its own dockable RViz panel, optionally on top of the matching camera image.
 *
 * Needs no TF and no camera_info; works for image-only objects as well.
 */
class ImageObservationsDisplay : public rviz_common::RosTopicDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  ImageObservationsDisplay();
  ~ImageObservationsDisplay() override;

  void onInitialize() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void onEnable() override;
  void onDisable() override;
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updateImageSubscription();
  void updateSourceVisibility();
  void queueRender();

private:
  friend class ImageObservationsDisplayTest;

  enum class ImageSource : int { ImageTopic = 0, CropMosaic = 1, Blank = 2 };
  enum class SyncMode : int { Synchronized = 0, LatestImage = 1 };
  enum class ColorMode : int { ByClass = 0, ByObjectId = 1, BySensor = 2, Fixed = 3 };

  struct Selection {
    std::vector<rendering::ImageObservationAnnotation> annotations;
    std::optional<rclcpp::Time> reference_stamp;  ///< newest selected observation stamp
    uint32_t source_width = 0, source_height = 0;
    size_t invalid = 0;
  };

  void subscribeImage();
  void unsubscribeImage();
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  Selection selectObservations(const std::optional<std::string>& image_frame) const;
  sensor_msgs::msg::Image::ConstSharedPtr findImage(const rclcpp::Time& stamp, double& dt_ms) const;
  QString formatLabel(const perception_msgs::msg::Object& object,
                      const perception_msgs::msg::ImageObservation& observation) const;
  QColor annotationColor(const perception_msgs::msg::Object& object,
                         const perception_msgs::msg::ImageObservation& observation) const;
  rendering::ImageObservationStyle currentStyle() const;
  void updateSensorOptions(const std::set<uint64_t>& sensor_ids);
  /** @brief Compose the next frame for the widget; false while waiting for a matching image. */
  bool render();

  // image source
  rviz_common::properties::EnumProperty* image_source_property_;
  rviz_common::properties::RosTopicProperty* image_topic_property_;
  rviz_common::properties::EditableEnumProperty* image_transport_property_;
  rviz_common::properties::EnumProperty* sync_mode_property_;
  rviz_common::properties::FloatProperty* sync_tolerance_property_;
  rviz_common::properties::IntProperty* image_buffer_size_property_;

  // filter
  rviz_common::properties::Property* filter_property_;
  rviz_common::properties::EditableEnumProperty* sensor_id_property_;
  rviz_common::properties::FloatProperty* min_existence_property_;
  rviz_common::properties::BoolProperty* show_stale_property_;

  // appearance
  rviz_common::properties::Property* appearance_properties_;
  rviz_common::properties::BoolProperty* color_property_group_;
  std::map<uint8_t, rviz_common::properties::ColorProperty*> classification_color_properties_;
  rviz_common::properties::BoolProperty* box_property_;
  rviz_common::properties::FloatProperty* box_line_width_property_;
  rviz_common::properties::FloatProperty* box_fill_alpha_property_;
  rviz_common::properties::BoolProperty* mask_property_;
  rviz_common::properties::EnumProperty* mask_mode_property_;
  rviz_common::properties::FloatProperty* mask_alpha_property_;
  rviz_common::properties::BoolProperty* crop_property_;
  rviz_common::properties::BoolProperty* roi_property_;
  rviz_common::properties::BoolProperty* label_property_;
  rviz_common::properties::StringProperty* label_format_property_;
  rviz_common::properties::IntProperty* font_size_property_;
  rviz_common::properties::EnumProperty* color_mode_property_;
  rviz_common::properties::ColorProperty* fixed_color_property_;

  rviz_common::properties::FloatProperty* image_alpha_property_;

  // owned here; destroyed before rviz_common::Display deletes the dock panel that hosts it
  std::unique_ptr<ImageObservationsWidget> widget_;
  image_transport::Subscriber image_subscriber_;

  perception_msgs::msg::ObjectList::ConstSharedPtr object_list_;
  std::optional<std::chrono::steady_clock::time_point> sync_wait_started_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_;
  std::set<uint64_t> known_sensor_ids_;
  QSize last_source_size_;
  bool dirty_ = false;
  std::optional<rclcpp::Time> sync_reference_stamp_;
  double sync_image_difference_ms_ = std::numeric_limits<double>::infinity();
};

}  // namespace displays
}  // namespace perception_msgs
