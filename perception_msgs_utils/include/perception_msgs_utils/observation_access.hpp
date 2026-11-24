// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT
#pragma once

#include <perception_msgs_utils/class_taxonomy.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace perception_msgs::object_access {

inline bool isMetricModel(uint8_t id) {
  return id == EGO::MODEL_ID || id == EGORWS::MODEL_ID ||
         id == ISCACTR::MODEL_ID || id == HEXAMOTION::MODEL_ID ||
         id == TRAFFICLIGHT::MODEL_ID || id == TRAFFICSIGN::MODEL_ID;
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

inline ObjectWithObservation
toImageObservation(const ObjectWithEvidence &source) {
  if (source.evidence.size() != 1 ||
      source.evidence.front().image.size() != 1 ||
      !source.evidence.front().point_cloud.empty())
    throw std::invalid_argument(
        "Image conversion requires one source observation; cannot select a "
        "fused sensor implicitly");
  const auto &evidence = source.evidence.front();
  sanityCheckObjectEvidence(source);
  const auto &image = evidence.image.front();
  ObjectWithObservation result;
  result.object = source.object;
  auto &state = result.object.state;
  initializeState(state, BOX2D::MODEL_ID, CONTINUOUS_STATE_UNMEASURED);
  state.header = evidence.header;
  state.sensor_id = {evidence.sensor_id};
  state.continuous_state = {
      image.bounding_box.center.position.x,
      image.bounding_box.center.position.y, image.bounding_box.size_x,
      image.bounding_box.size_y, image.bounding_box.center.theta};
  state.discrete_state = {image.image_width,  image.image_height,
                          image.roi.x_offset, image.roi.y_offset,
                          image.roi.width,    image.roi.height};
  result.object.state_history.clear();
  result.object.state_predictions.clear();
  result.observation.image.emplace_back();
  result.observation.image.front().crop = image.crop;
  result.observation.image.front().mask = image.mask;
  sanityCheckObjectObservation(result);
  return result;
}

// Compatibility projection. New pipeline state and taxonomy remain independent
// of legacy models.
inline ObjectWithEvidence toLegacyImage(const ObjectWithObservation &source,
                                        uint8_t legacy_model, bool include_pixels = true) {
  sanityCheckObjectObservation(source);
  if (source.observation.image.empty())
    throw std::invalid_argument("An image observation is required");
  ObjectWithEvidence result;
  result.object = source.object;
  initializeState(result.object, legacy_model, CONTINUOUS_STATE_UNMEASURED);
  const auto &s = source.object.state;
  const auto &c = s.continuous_state;
  const auto &d = s.discrete_state;
  Evidence evidence;
  evidence.header = s.header;
  evidence.sensor_id = s.sensor_id.front();
  ImageEvidence image;
  image.image_width = d[0];
  image.image_height = d[1];
  image.bounding_box.center.position.x = c[0];
  image.bounding_box.center.position.y = c[1];
  image.bounding_box.size_x = c[2];
  image.bounding_box.size_y = c[3];
  image.bounding_box.center.theta = c[4];
  image.roi.x_offset = d[2];
  image.roi.y_offset = d[3];
  image.roi.width = d[4];
  image.roi.height = d[5];
  if (include_pixels) {
    image.crop = source.observation.image.front().crop;
    image.mask = source.observation.image.front().mask;
  }
  evidence.image.push_back(std::move(image));
  result.evidence.push_back(std::move(evidence));
  return result;
}

inline uint8_t legacyLightColor(uint8_t classification) {
  if (classification == 106)
    return TRAFFICLIGHT::STATE_GREEN;
  if (classification == 105)
    return TRAFFICLIGHT::STATE_YELLOW;
  if (classification == 103)
    return TRAFFICLIGHT::STATE_OFF;
  if (classification == 107)
    return TRAFFICLIGHT::STATE_RED_YELLOW;
  if (classification == 104 || (classification >= 108 && classification <= 114))
    return TRAFFICLIGHT::STATE_RED;
  return TRAFFICLIGHT::STATE_UNKNOWN;
}
inline uint8_t bestClass(const ObjectState &state) {
  if (state.classifications.empty())
    return ObjectClassification::UNCLASSIFIED;
  return std::max_element(state.classifications.begin(),
                          state.classifications.end(),
                          [](const auto &a, const auto &b) {
                            return a.probability < b.probability;
                          })
      ->type;
}
inline ObjectWithEvidence toLegacyLight(const ObjectWithObservation &source, bool include_pixels = true) {
  auto result = toLegacyImage(source, TRAFFICLIGHT::MODEL_ID, include_pixels);
  if (!isTrafficLightClass(bestClass(source.object.state)))
    throw std::invalid_argument("Expected traffic-light taxonomy");
  setTrafficLightState(result.object,
                       legacyLightColor(bestClass(source.object.state)));
  setTrafficLightType(result.object, TRAFFICLIGHT::TYPE_DEFAULT);
  return result;
}

inline Object toMetricObject(const Object &source) {
  if (source.state.model_id == BOX2D::MODEL_ID)
    throw std::invalid_argument(
        "Pixel state cannot be converted to metric without localization");
  if (source.state.model_id != TRAFFICLIGHT::MODEL_ID &&
      source.state.model_id != TRAFFICSIGN::MODEL_ID)
    return source;
  const auto &old = source.state;
  if (old.continuous_state.size() !=
          size_t(getContinuousStateSize(old.model_id)) ||
      old.discrete_state.size() != size_t(getDiscreteStateSize(old.model_id)))
    throw std::invalid_argument("Invalid legacy metric state");
  Object result = source;
  initializeState(result, HEXAMOTION::MODEL_ID, CONTINUOUS_STATE_UNMEASURED);
  auto &state = result.state;
  for (int i = 0; i < 3; ++i)
    state.continuous_state[i] = old.continuous_state[i];
  const std::vector<size_t> indices =
      old.model_id == TRAFFICSIGN::MODEL_ID
          ? std::vector<size_t>{0, 1, 2, 7, 8, 9, 14, 15}
          : std::vector<size_t>{0, 1, 2};
  for (size_t i = 3; i < indices.size(); ++i)
    state.continuous_state[indices[i]] = old.continuous_state[i];
  if (!old.continuous_state_covariance.empty()) {
    const size_t n = indices.size();
    if (old.continuous_state_covariance.size() != n * n)
      throw std::invalid_argument("Invalid legacy covariance");
    state.continuous_state_covariance.assign(256, 0.0);
    for (int i = 0; i < 16; ++i)
      state.continuous_state_covariance[i * 16 + i] =
          ObjectState::CONTINUOUS_STATE_COVARIANCE_INVALID;
    for (size_t i = 0; i < n; ++i)
      for (size_t j = 0; j < n; ++j)
        state.continuous_state_covariance[indices[i] * 16 + indices[j]] =
            old.continuous_state_covariance[i * n + j];
  }
  return result;
}

inline Object toLegacyMetricLight(const Object &source) {
  if (source.state.model_id == TRAFFICLIGHT::MODEL_ID)
    return source;
  if (source.state.model_id != HEXAMOTION::MODEL_ID ||
      source.state.continuous_state.size() != 16 ||
      !isTrafficLightClass(bestClass(source.state)))
    throw std::invalid_argument("Expected generic 3D traffic light");
  Object result = source;
  initializeState(result, TRAFFICLIGHT::MODEL_ID, CONTINUOUS_STATE_UNMEASURED);
  for (int i = 0; i < 3; ++i)
    result.state.continuous_state[i] = source.state.continuous_state[i];
  if (source.state.continuous_state_covariance.size() == 256) {
    result.state.continuous_state_covariance.resize(9);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        result.state.continuous_state_covariance[i * 3 + j] =
            source.state.continuous_state_covariance[i * 16 + j];
  }
  setTrafficLightState(result, legacyLightColor(bestClass(source.state)));
  setTrafficLightType(result, TRAFFICLIGHT::TYPE_DEFAULT);
  return result;
}
inline Object toLegacyMetricSign(const Object &source) {
  if (source.state.model_id == TRAFFICSIGN::MODEL_ID)
    return source;
  if (source.state.model_id != HEXAMOTION::MODEL_ID ||
      source.state.continuous_state.size() != 16 ||
      !isTrafficSignClass(bestClass(source.state)))
    throw std::invalid_argument("Expected generic 3D traffic sign");
  Object result = source;
  initializeState(result, TRAFFICSIGN::MODEL_ID, CONTINUOUS_STATE_UNMEASURED);
  const std::vector<size_t> indices{0, 1, 2, 7, 8, 9, 14, 15};
  for (size_t i = 0; i < indices.size(); ++i)
    result.state.continuous_state[i] =
        source.state.continuous_state[indices[i]];
  if (source.state.continuous_state_covariance.size() == 256) {
    result.state.continuous_state_covariance.resize(64);
    for (int i = 0; i < 8; ++i)
      for (int j = 0; j < 8; ++j)
        result.state.continuous_state_covariance[i * 8 + j] =
            source.state
                .continuous_state_covariance[indices[i] * 16 + indices[j]];
  }
  const auto semantic = bestClass(source.state);
  setTrafficSignType(result, semantic >= 130 ? semantic - 129 : 0);
  return result;
}
} // namespace perception_msgs::object_access
