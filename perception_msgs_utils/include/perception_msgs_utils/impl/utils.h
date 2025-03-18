/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

/**
 * @file utils.h
 * @brief Object state utility functions
 */

#pragma once

#include <cmath>

#include <perception_msgs_utils/impl/constants.h>


namespace perception_msgs {

namespace object_access {

  // --- state size ------------------------------------------------------------

  const std::string kExceptionUnknownModel = "Unknown model ID: ";

  /**
   * @brief Get the continuous state size for a given object state.
   *
   * @param state
   * @return int
   */
  inline int getContinuousStateSize(const ObjectState& state) {
    return state.continuous_state.size();
  }

  /**
   * @brief Get the continuous state size for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return int
   */
  template <typename T>
  inline int getContinuousStateSize(const T& obj) {
    return getContinuousStateSize(obj.state);
  }

  /**
   * @brief Get the continuous state size for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int getContinuousStateSize(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::CONTINUOUS_STATE_SIZE;
      case EGORWS::MODEL_ID:
        return EGORWS::CONTINUOUS_STATE_SIZE;
      case ISCACTR::MODEL_ID:
        return ISCACTR::CONTINUOUS_STATE_SIZE;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::CONTINUOUS_STATE_SIZE;
      case TRAFFICLIGHT::MODEL_ID:
        return TRAFFICLIGHT::CONTINUOUS_STATE_SIZE;
      default:
        throw std::invalid_argument(kExceptionUnknownModel + std::to_string(model_id));
    }
  }

  /**
   * @brief Get the discrete state size for a given object state.
   *
   * @param state
   * @return int
   */
  inline int getDiscreteStateSize(const ObjectState& state) {
    return state.discrete_state.size();
  }

  /**
   * @brief Get the discrete state size for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return int
   */
  template <typename T>
  inline int getDiscreteStateSize(const T& obj) {
    return getDiscreteStateSize(obj.state);
  }

  /**
   * @brief Get the discrete state size for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int getDiscreteStateSize(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::DISCRETE_STATE_SIZE;
      case EGORWS::MODEL_ID:
        return EGORWS::DISCRETE_STATE_SIZE;
      case ISCACTR::MODEL_ID:
        return ISCACTR::DISCRETE_STATE_SIZE;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::DISCRETE_STATE_SIZE;
      case TRAFFICLIGHT::MODEL_ID:
        return TRAFFICLIGHT::DISCRETE_STATE_SIZE;
      default:
        throw std::invalid_argument(kExceptionUnknownModel + std::to_string(model_id));
    }
  }

  /**
   * @brief Get the continuous state covariance size for a given object state.
   *
   * @param state
   * @return int
   */
  inline int getContinuousStateCovarianceSize(const ObjectState& state) {
    return state.continuous_state_covariance.size();
  }

  /**
   * @brief Get the continuous state covariance size for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return int
   */
  template <typename T>
  inline int getContinuousStateCovarianceSize(const T& obj) {
    return getContinuousStateCovarianceSize(obj.state);
  }

  /**
   * @brief Get the continuous state covariance size for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int getContinuousStateCovarianceSize(const unsigned char& model_id) {
    return std::pow(getContinuousStateSize(model_id), 2);
  }

  /**
   * @brief Set the continuous state covariance to unknown at (i,j) for a given object state.
   *
   * @param state
   * @param i
   * @param j
   */
  inline void setContinuousStateCovarianceToUnknownAt(ObjectState& state, const unsigned int i, const unsigned int j) {
    const int n = getContinuousStateSize(state);
    state.continuous_state_covariance[n * i + j] = CONTINUOUS_STATE_COVARIANCE_UNKNOWN;
  }

} // namespace object_access

} // namespace perception_msgs
