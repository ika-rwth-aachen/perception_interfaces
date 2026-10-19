// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <cmath>
#include <array>
#include <perception_msgs_utils/constants.hpp>

namespace perception_msgs {
namespace object_access {

/**
 * @brief Transform a stamped pose and rotate covariance while preserving validity markers.
 *
 * Marker entries are replaced with zero for numerical rotation, then propagated
 * to output entries that depend on them. An INVALID contribution makes an output
 * diagonal INVALID; otherwise, any INVALID or UNKNOWN contribution makes the
 * output entry UNKNOWN. INVALID is recognized only on the input diagonal, so
 * negative cross covariance, including -1, is rotated numerically.
 * Rotation coefficients with magnitude at most 1e-12 are ignored when propagating
 * markers to avoid rounding residue at exact axis rotations.
 *
 * @param[in] input Stamped pose and covariance in the source frame.
 * @param[out] output Transformed pose and covariance, with the transform's header.
 * @param[in] transform Transform from the source frame to the target frame.
 */
inline void transformPoseWithCovariance(
  const gm::PoseWithCovarianceStamped& input, gm::PoseWithCovarianceStamped& output,
  const gm::TransformStamped& transform)
{
  auto numerical = input;
  std::array<bool, 36> invalid{}, unknown{};
  bool has_markers = false;
  for (int k = 0; k < 36; ++k) {
    const double value = input.pose.covariance[k];
    invalid[k] = k % 7 == 0 && value == CONTINUOUS_STATE_COVARIANCE_INVALID;
    unknown[k] = value == CONTINUOUS_STATE_COVARIANCE_UNKNOWN;
    if (invalid[k] || unknown[k]) {
      numerical.pose.covariance[k] = 0.0;
      has_markers = true;
    }
  }
  tf2::doTransform(numerical, output, transform);
  if (!has_markers) return;

  tf2::Transform tf_transform;
  tf2::fromMsg(transform.transform, tf_transform);
  const auto& rotation = tf_transform.getBasis();
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      bool invalid_contribution = false, unknown_contribution = false;
      for (int k = 0; k < 3; ++k) {
        // Ignore rounding residue at exact axis rotations (e.g. pi/2).
        if (std::abs(rotation[i % 3][k]) <= 1e-12) continue;
        for (int l = 0; l < 3; ++l) {
          if (std::abs(rotation[j % 3][l]) <= 1e-12) continue;
          const int index = 6 * (3 * (i / 3) + k) + 3 * (j / 3) + l;
          invalid_contribution = invalid_contribution || invalid[index];
          unknown_contribution = unknown_contribution || unknown[index];
        }
      }
      if (i == j && invalid_contribution)
        output.pose.covariance[6 * i + j] = CONTINUOUS_STATE_COVARIANCE_INVALID;
      else if (invalid_contribution || unknown_contribution)
        output.pose.covariance[6 * i + j] = CONTINUOUS_STATE_COVARIANCE_UNKNOWN;
    }
  }
}

} // namespace object_access
} // namespace perception_msgs
