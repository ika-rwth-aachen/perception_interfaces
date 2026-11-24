// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT
#include "perception_msgs/displays/image_evidence/image_observation_display.hpp"
#include <perception_msgs_utils/object_access.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>

namespace perception_msgs::displays {
ImageObservationDisplay::ImageObservationDisplay() {
  topic_property_->setMessageType(
      "perception_msgs/msg/ObjectListWithObservation");
  topic_property_->setDescription("2D objects with image observations.");
}
void ImageObservationDisplay::subscribe() {
  if (!isEnabled() || topic_property_->isEmpty())
    return;
  try {
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    subscription_ = node->create_subscription<
        perception_msgs::msg::ObjectListWithObservation>(
        topic_property_->getTopicStd(), qos_profile,
        [this](perception_msgs::msg::ObjectListWithObservation::ConstSharedPtr
                   message) {
          auto canvas =
              std::make_shared<perception_msgs::msg::ObjectListWithEvidence>();
          canvas->header = message->header;
          size_t invalid = 0;
          for (const auto &entry : message->objects) {
            try {
              // A rendering projection only: raw 2D geometry is retained in the
              // canvas evidence.
              canvas->objects.push_back(object_access::toLegacyImage(
                  entry, perception_msgs::msg::HEXAMOTION::MODEL_ID));
            } catch (const std::exception &) {
              ++invalid;
            }
          }
          processMessage(canvas);
          if (invalid)
            setStatus(rviz_common::properties::StatusProperty::Warn,
                      "Observation",
                      QString("%1 invalid/non-image observations skipped")
                          .arg(invalid));
          else
            deleteStatus("Observation");
        });
  } catch (const std::exception &error) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              QString::fromUtf8(error.what()));
  }
}
void ImageObservationDisplay::unsubscribe() {
  subscription_.reset();
  ImageEvidenceDisplay::unsubscribe();
}
} // namespace perception_msgs::displays
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::ImageObservationDisplay,
                       rviz_common::Display)
