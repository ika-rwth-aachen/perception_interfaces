// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

/**
 * @file image_access.hpp
 * @brief Image observations and metric-state availability
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <perception_msgs_utils/state_index.hpp>
#include <perception_msgs_utils/utils.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace perception_msgs {

namespace object_access {

  namespace image_detail {

    /**
     * @brief Read a vision_msgs pose center across supported ROS distributions.
     */
    template <typename T>
    auto centerX(const T& center, int) -> decltype(center.position.x) {
      return center.position.x;
    }

    /**
     * @brief Read the geometry_msgs pose center used by older vision_msgs versions.
     */
    template <typename T>
    auto centerX(const T& center, long) -> decltype(center.x) {
      return center.x;
    }

    /**
     * @brief Read a vision_msgs pose center across supported ROS distributions.
     */
    template <typename T>
    auto centerY(const T& center, int) -> decltype(center.position.y) {
      return center.position.y;
    }

    /**
     * @brief Read the geometry_msgs pose center used by older vision_msgs versions.
     */
    template <typename T>
    auto centerY(const T& center, long) -> decltype(center.y) {
      return center.y;
    }

    /**
     * @brief Check state storage before querying metric availability.
     */
    inline int stateSize(const ObjectState& state) {
      const int n = getContinuousStateSize(state.model_id);
      if (state.continuous_state.size() != static_cast<std::size_t>(n) ||
          (!state.continuous_state_covariance.empty() &&
           state.continuous_state_covariance.size() != static_cast<std::size_t>(n * n)))
        throw std::invalid_argument("Invalid state storage for metric availability check");
      return n;
    }

    /**
     * @brief Check a finite component and its covariance diagonal.
     */
    inline bool measured(const ObjectState& state, int n, int index) {
      if (state.continuous_state_covariance.empty()) return false;
      const double variance = state.continuous_state_covariance[n * index + index];
      return std::isfinite(state.continuous_state[index]) && std::isfinite(variance) && variance >= 0.0;
    }
  }  // namespace image_detail

  // --- metric availability ---------------------------------------------------

  /**
   * @brief Check if all position components are measured.
   */
  inline bool hasValidPosition(const ObjectState& state) {
    const int n = image_detail::stateSize(state);
    return hasX(state.model_id) && hasY(state.model_id) && hasZ(state.model_id) &&
           image_detail::measured(state, n, indexX(state.model_id)) &&
           image_detail::measured(state, n, indexY(state.model_id)) &&
           image_detail::measured(state, n, indexZ(state.model_id));
  }

  /**
   * @brief Whether an object's position is measured.
   */
  template <typename T>
  inline bool hasValidPosition(const T& object) {
    return hasValidPosition(object.state);
  }

  /**
   * @brief Whether any finite continuous-state component has a valid covariance diagonal.
   */
  inline bool hasMetricState(const ObjectState& state) {
    const int n = image_detail::stateSize(state);
    for (int i = 0; i < n; ++i)
      if (image_detail::measured(state, n, i)) return true;
    return false;
  }

  /**
   * @brief Whether an object has any measured metric estimate.
   */
  template <typename T>
  inline bool hasMetricState(const T& object) {
    return hasMetricState(object.state);
  }

  /**
   * @brief Whether no continuous-state component is measured.
   */
  inline bool isImageOnly(const ObjectState& state) { return !hasMetricState(state); }

  /**
   * @brief Whether an object has no measured continuous-state component.
   */
  template <typename T>
  inline bool isImageOnly(const T& object) {
    return isImageOnly(object.state);
  }

  // --- image checks ----------------------------------------------------------

  /**
   * @brief Check image geometry and payload layout without scanning pixels.
   */
  inline void sanityCheckImageObservation(const ImageObservation& image) {
    const auto& box = image.bounding_box;
    if (image.header.frame_id.empty() || image.header.stamp.sec < 0 || image.header.stamp.nanosec >= 1000000000u ||
        image.image_width == 0 || image.image_height == 0 || !std::isfinite(image_detail::centerX(box.center, 0)) ||
        !std::isfinite(image_detail::centerY(box.center, 0)) || !std::isfinite(box.center.theta) ||
        !std::isfinite(box.size_x) || !std::isfinite(box.size_y) || box.size_x <= 0.0 || box.size_y <= 0.0)
      throw std::invalid_argument("Invalid image observation header or bounding box");
    const auto& roi = image.roi;
    const bool rectangle = roi.width > 0 && roi.height > 0;
    if (roi.do_rectify ||
        (rectangle && (static_cast<uint64_t>(roi.x_offset) + roi.width > image.image_width ||
                       static_cast<uint64_t>(roi.y_offset) + roi.height > image.image_height)) ||
        (!rectangle && (roi.width || roi.height || roi.x_offset || roi.y_offset)))
      throw std::invalid_argument("Invalid source image ROI");
    const auto payload = [&](const sensor_msgs::msg::Image& pixels, bool mask) {
      if (pixels.data.empty()) {
        if (pixels.width || pixels.height || pixels.step)
          throw std::invalid_argument("Empty image payload has nonzero dimensions");
        return;
      }
      if (!rectangle || pixels.width == 0 || pixels.height == 0 || pixels.header != image.header ||
          pixels.is_bigendian > 1 || (mask && pixels.encoding != "mono8"))
        throw std::invalid_argument("Invalid crop or mask metadata");
      uint64_t pixel_bytes = 0;
      try {
        pixel_bytes = static_cast<uint64_t>(sensor_msgs::image_encodings::numChannels(pixels.encoding)) *
                      sensor_msgs::image_encodings::bitDepth(pixels.encoding) / 8;
      } catch (const std::exception&) {
        throw std::invalid_argument("Unsupported packed image encoding");
      }
      if (pixels.encoding == "nv12" || pixels.encoding == "nv21" || pixels.encoding == "nv24" || pixel_bytes == 0 ||
          pixel_bytes > std::numeric_limits<uint32_t>::max() / pixels.width || pixels.step < pixel_bytes * pixels.width ||
          static_cast<uint64_t>(pixels.step) * pixels.height != pixels.data.size())
        throw std::invalid_argument("Invalid crop or mask buffer layout");
    };
    payload(image.crop, false);
    payload(image.mask, true);
  }

  /**
   * @brief Validate observation identities and their contribution to sensor provenance.
   */
  inline void sanityCheckImageObservations(const ObjectState& state) {
    std::set<std::tuple<uint64_t, std::string, int32_t, uint32_t>> sources;
    for (const auto& image : state.image_observations) {
      sanityCheckImageObservation(image);
      if (std::find(state.sensor_id.begin(), state.sensor_id.end(), image.sensor_id) == state.sensor_id.end())
        throw std::invalid_argument("Image observation sensor is absent from state.sensor_id");
      if (!sources.emplace(image.sensor_id, image.header.frame_id, image.header.stamp.sec, image.header.stamp.nanosec)
               .second)
        throw std::invalid_argument("Duplicate image observation for the same source and acquisition");
    }
  }

  /**
   * @brief Validate image observations on an enclosing object.
   */
  template <typename T>
  inline void sanityCheckImageObservations(const T& object) {
    sanityCheckImageObservations(object.state);
  }

  // --- image getters and setters --------------------------------------------

  /**
   * @brief Get the observations without copying image buffers.
   */
  inline const std::vector<ImageObservation>& getImageObservations(const ObjectState& state) {
    return state.image_observations;
  }

  /**
   * @brief Get the image observations of an object.
   */
  template <typename T>
  inline const std::vector<ImageObservation>& getImageObservations(const T& object) {
    return getImageObservations(object.state);
  }

  /**
   * @brief Set observations and retain all contributing sensor IDs. Validate before changing the state.
   */
  inline void setImageObservations(ObjectState& state, std::vector<ImageObservation> images) {
    ObjectState candidate;
    candidate.sensor_id = state.sensor_id;
    candidate.image_observations = std::move(images);
    for (const auto& image : candidate.image_observations)
      if (std::find(candidate.sensor_id.begin(), candidate.sensor_id.end(), image.sensor_id) == candidate.sensor_id.end())
        candidate.sensor_id.push_back(image.sensor_id);
    sanityCheckImageObservations(candidate);
    state.image_observations = std::move(candidate.image_observations);
    state.sensor_id = std::move(candidate.sensor_id);
  }

  /**
   * @brief Set an object's image observations.
   */
  template <typename T>
  inline void setImageObservations(T& object, std::vector<ImageObservation> images) {
    setImageObservations(object.state, std::move(images));
  }

  /**
   * @brief Append a validated image observation and its sensor provenance.
   */
  inline void addImageObservation(ObjectState& state, ImageObservation image) {
    sanityCheckImageObservation(image);
    for (const auto& previous : state.image_observations)
      if (previous.sensor_id == image.sensor_id && previous.header == image.header)
        throw std::invalid_argument("Duplicate image observation for the same source and acquisition");
    const uint64_t sensor_id = image.sensor_id;
    state.image_observations.push_back(std::move(image));
    if (std::find(state.sensor_id.begin(), state.sensor_id.end(), sensor_id) == state.sensor_id.end())
      state.sensor_id.push_back(sensor_id);
  }

  /**
   * @brief Append an image observation to an object.
   */
  template <typename T>
  inline void addImageObservation(T& object, ImageObservation image) {
    addImageObservation(object.state, std::move(image));
  }

} // namespace object_access
} // namespace perception_msgs
