// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

/**
 * @file constants.hpp
 * @brief Object state constants
 */

#pragma once

#include <cmath>
#include <limits>


namespace perception_msgs {

namespace object_access {

    const double CONTINUOUS_STATE_INIT = 0;

    const long int DISCRETE_STATE_INIT = 0;

    const double CONTINUOUS_STATE_COVARIANCE_INIT = ObjectState::CONTINUOUS_STATE_COVARIANCE_INIT;
    const double CONTINUOUS_STATE_COVARIANCE_INVALID = ObjectState::CONTINUOUS_STATE_COVARIANCE_INVALID;
    const double CONTINUOUS_STATE_COVARIANCE_UNKNOWN = ObjectState::CONTINUOUS_STATE_COVARIANCE_UNKNOWN;

} // namespace object_access

} // namespace perception_msgs
