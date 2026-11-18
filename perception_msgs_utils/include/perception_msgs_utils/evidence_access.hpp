// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

/**
 * @file evidence_access.hpp
 * @brief Evidence access functions
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <sensor_msgs/image_encodings.hpp>


namespace perception_msgs {

namespace object_access {

  /**
   * @brief Perform sanity check on given image evidence.
   *
   * @param image
   * @param header header of the enclosing evidence
   */
  inline void sanityCheckImageEvidence(const ImageEvidence& image, const std_msgs::msg::Header& header) {
    const auto& box = image.bounding_box;
    if (image.image_width == 0 || image.image_height == 0)
      throw std::invalid_argument("Image evidence without image size");
    if (!std::isfinite(box.center.position.x) || !std::isfinite(box.center.position.y) ||
        !std::isfinite(box.center.theta) || !std::isfinite(box.size_x) || !std::isfinite(box.size_y) ||
        box.size_x <= 0.0 || box.size_y <= 0.0)
      throw std::invalid_argument("Invalid image evidence bounding box");
    const auto& roi = image.roi;
    if (static_cast<uint64_t>(roi.x_offset) + roi.width > image.image_width ||
        static_cast<uint64_t>(roi.y_offset) + roi.height > image.image_height)
      throw std::invalid_argument("Image evidence ROI exceeds the source image");
    for (const auto* pixels : {&image.crop, &image.mask}) {
      if (pixels->data.empty()) continue;
      if (roi.width == 0 || roi.height == 0 || pixels->header != header ||
          static_cast<uint64_t>(pixels->step) * pixels->height != pixels->data.size())
        throw std::invalid_argument("Invalid image evidence crop or mask");
    }
    if (!image.mask.data.empty() &&
        (image.mask.encoding != sensor_msgs::image_encodings::MONO8 || image.mask.step < image.mask.width))
      throw std::invalid_argument("Image evidence mask must be mono8");
  }

  /**
   * @brief Perform sanity check on given point cloud evidence.
   *
   * @param point_cloud
   * @param header header of the enclosing evidence
   */
  inline void sanityCheckPointCloudEvidence(const PointCloudEvidence& point_cloud,
                                            const std_msgs::msg::Header& header) {
    const auto& points = point_cloud.points;
    if (points.header != header || static_cast<uint64_t>(points.row_step) * points.height != points.data.size())
      throw std::invalid_argument("Invalid point cloud evidence");
  }

  /**
   * @brief Perform sanity check on given evidence.
   *
   * @param evidence
   */
  inline void sanityCheckEvidence(const Evidence& evidence) {
    if (evidence.header.frame_id.empty() || evidence.image.size() + evidence.point_cloud.size() != 1)
      throw std::invalid_argument("Evidence needs a frame_id and exactly one kind");
    for (const auto& image : evidence.image) sanityCheckImageEvidence(image, evidence.header);
    for (const auto& point_cloud : evidence.point_cloud) sanityCheckPointCloudEvidence(point_cloud, evidence.header);
  }

  /**
   * @brief Perform sanity check on the evidence of a given object.
   * The sensor of each evidence must be contained in the object state.
   *
   * @param obj
   */
  inline void sanityCheckObjectEvidence(const ObjectWithEvidence& obj) {
    const auto& sensor_ids = obj.object.state.sensor_id;
    for (const auto& evidence : obj.evidence) {
      sanityCheckEvidence(evidence);
      if (std::find(sensor_ids.begin(), sensor_ids.end(), evidence.sensor_id) == sensor_ids.end())
        throw std::invalid_argument("Evidence sensor is missing in object.state.sensor_id");
    }
  }

  /**
   * @brief Add evidence to a given object and register its sensor in the object state.
   *
   * @param obj
   * @param evidence
   */
  inline void addEvidence(ObjectWithEvidence& obj, const Evidence& evidence) {
    sanityCheckEvidence(evidence);
    auto& sensor_ids = obj.object.state.sensor_id;
    if (std::find(sensor_ids.begin(), sensor_ids.end(), evidence.sensor_id) == sensor_ids.end())
      sensor_ids.push_back(evidence.sensor_id);
    obj.evidence.push_back(evidence);
  }

} // namespace object_access

} // namespace perception_msgs
