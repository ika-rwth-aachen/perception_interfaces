// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT
#pragma once

#include <perception_msgs_utils/class_taxonomy.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

namespace perception_msgs::object_access {

inline bool isMetricModel(uint8_t id) {
  return id == EGO::MODEL_ID || id == EGORWS::MODEL_ID ||
         id == ISCACTR::MODEL_ID || id == HEXAMOTION::MODEL_ID;
}

inline void sanityCheckBox2D(const ObjectState &state) {
  if (state.model_id != BOX2D::MODEL_ID || state.continuous_state.size() != 5 ||
      state.discrete_state.size() != 6 || state.header.frame_id.empty() ||
      state.header.stamp.sec < 0 || state.header.stamp.nanosec >= 1000000000u ||
      state.sensor_id.size() != 1)
    throw std::invalid_argument("BOX2D requires five coordinates, six image "
                                "dimensions and one acquisition sensor");
  const auto &c = state.continuous_state;
  const auto &d = state.discrete_state;
  if (!std::all_of(c.begin(), c.end(),
                   [](double x) { return std::isfinite(x); }) ||
      c[2] <= 0 || c[3] <= 0 || d[0] <= 0 || d[1] <= 0 || d[0] > UINT32_MAX ||
      d[1] > UINT32_MAX || d[2] < 0 || d[3] < 0 || d[4] < 0 || d[5] < 0 ||
      d[2] > d[0] || d[3] > d[1] || d[4] > d[0] - d[2] || d[5] > d[1] - d[3])
    throw std::invalid_argument("Invalid BOX2D box, source dimensions or ROI");
  if (!state.continuous_state_covariance.empty() &&
      state.continuous_state_covariance.size() != 25)
    throw std::invalid_argument("BOX2D covariance must be empty or 5x5");
}

inline void sanityCheckObservationImage(const sensor_msgs::msg::Image &pixels,
                                        const ObjectState &state, bool mask) {
  if (pixels.data.empty()) {
    if (pixels.width || pixels.height || pixels.step)
      throw std::invalid_argument(
          "Absent image payload must have zero dimensions and stride");
    return;
  }
  if (pixels.header != state.header || !pixels.width || !pixels.height ||
      !state.discrete_state[4] || !state.discrete_state[5] ||
      uint64_t(pixels.step) * pixels.height != pixels.data.size() ||
      pixels.is_bigendian > 1)
    throw std::invalid_argument(
        "Invalid observation image header, ROI or storage");
  const auto channels =
      sensor_msgs::image_encodings::numChannels(pixels.encoding);
  const auto bits = sensor_msgs::image_encodings::bitDepth(pixels.encoding);
  if (channels <= 0 || bits <= 0 ||
      pixels.step < (uint64_t(pixels.width) * channels * bits + 7) / 8 ||
      (mask && pixels.encoding != "mono8"))
    throw std::invalid_argument(
        "Invalid image encoding or stride; mask must be mono8");
}

inline void sanityCheckObjectObservation(const ObjectWithObservation &entry) {
  const auto &observation = entry.observation;
  const auto &state = entry.object.state;
  if (!std::isfinite(entry.object.existence_probability) || entry.object.existence_probability < 0.0 ||
      entry.object.existence_probability > 1.0)
    throw std::invalid_argument("Invalid detector existence probability");
  for (const auto& classification : state.classifications)
    if (!std::isfinite(classification.probability) || classification.probability < 0.0 || classification.probability > 1.0)
      throw std::invalid_argument("Invalid semantic probability");
  if (observation.image.size() + observation.point_cloud.size() != 1)
    throw std::invalid_argument(
        "Exactly one image or point cloud observation is required");
  if (!observation.image.empty()) {
    sanityCheckBox2D(state);
    sanityCheckObservationImage(observation.image.front().crop, state, false);
    sanityCheckObservationImage(observation.image.front().mask, state, true);
    return;
  }
  if (!isMetricModel(state.model_id) ||
      state.continuous_state.size() !=
          size_t(getContinuousStateSize(state.model_id)) ||
      state.discrete_state.size() !=
          size_t(getDiscreteStateSize(state.model_id)) ||
      state.sensor_id.size() != 1 || state.header.frame_id.empty() ||
      state.header.stamp.sec < 0 || state.header.stamp.nanosec >= 1000000000u)
    throw std::invalid_argument(
        "Point observations require a metric state and one acquisition sensor");
  sanityCheckFullState(state);
  const auto &points = observation.point_cloud.front().points;
  if (points.header != state.header ||
      uint64_t(points.row_step) * points.height != points.data.size() ||
      points.row_step < uint64_t(points.point_step) * points.width ||
      (points.width && (!points.point_step || !points.height || points.fields.empty())))
    throw std::invalid_argument(
        "Invalid point cloud layout or acquisition header");
  for (const auto &field : points.fields) {
    static constexpr uint8_t sizes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
    if (field.datatype == 0 || field.datatype > 8 || !field.count ||
        field.name.empty() ||
        uint64_t(field.offset) + uint64_t(field.count) * sizes[field.datatype] >
            points.point_step)
      throw std::invalid_argument("Point field exceeds the point layout");
  }
}


/** Zero-copy view of source geometry and image payloads. Geometry stays in BOX2D. */
struct ImageObservationView {
  const std_msgs::msg::Header& header;
  uint64_t sensor_id;
  uint32_t image_width, image_height;
  vision_msgs::msg::BoundingBox2D bounding_box;
  sensor_msgs::msg::RegionOfInterest roi;
  const sensor_msgs::msg::Image& crop;
  const sensor_msgs::msg::Image& mask;
};
inline ImageObservationView imageView(const ObjectWithObservation& entry) {
  sanityCheckObjectObservation(entry);
  if (entry.observation.image.empty()) throw std::invalid_argument("Expected an image observation");
  const auto& state = entry.object.state;
  const auto& c = state.continuous_state;
  const auto& d = state.discrete_state;
  const auto& pixels = entry.observation.image.front();
  vision_msgs::msg::BoundingBox2D box;
  box.center.position.x = c[0]; box.center.position.y = c[1]; box.size_x = c[2]; box.size_y = c[3]; box.center.theta = c[4];
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = d[2]; roi.y_offset = d[3]; roi.width = d[4]; roi.height = d[5];
  return {state.header, state.sensor_id.front(), static_cast<uint32_t>(d[0]), static_cast<uint32_t>(d[1]), box, roi, pixels.crop, pixels.mask};
}
inline uint8_t bestClass(const ObjectState& state) {
  if (state.classifications.empty()) return ObjectClassification::UNCLASSIFIED;
  return std::max_element(state.classifications.begin(), state.classifications.end(),
                         [](const auto& a, const auto& b) { return a.probability < b.probability; })->type;
}
} // namespace perception_msgs::object_access
