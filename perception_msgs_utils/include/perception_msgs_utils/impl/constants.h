// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

/**
 * @file constants.h
 * @brief Object state constants
 */

#pragma once

#include <cmath>
#include <limits>


namespace perception_msgs {

namespace object_access {

    const double CONTINUOUS_STATE_INIT = 0;

    const long int DISCRETE_STATE_INIT = 0;

    const double CONTINUOUS_STATE_COVARIANCE_INIT = 0;
    const double CONTINUOUS_STATE_COVARIANCE_INVALID = -1;
    const double CONTINUOUS_STATE_COVARIANCE_UNKNOWN = std::numeric_limits<double>::max();

} // namespace object_access

} // namespace perception_msgs
