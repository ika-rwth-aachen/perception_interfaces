// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT
#pragma once
#include "perception_msgs/displays/image_evidence/image_evidence_display.hpp"
#include <perception_msgs/msg/object_list_with_observation.hpp>

namespace perception_msgs::displays {
/** Reuse the evidence canvas and synchronization UI through a validated
 * rendering projection. */
class ImageObservationDisplay : public ImageEvidenceDisplay {
public:
  ImageObservationDisplay();

protected:
  void subscribe() override;
  void unsubscribe() override;

private:
  rclcpp::Subscription<
      perception_msgs::msg::ObjectListWithObservation>::SharedPtr subscription_;
};
} // namespace perception_msgs::displays
