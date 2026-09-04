// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

/**
 * @file checks.h
 * @brief Object state sanity checks
 */

#pragma once

#include <perception_msgs_utils/impl/utils.h>


namespace perception_msgs {

namespace object_access {

  const std::string kExceptionInvalidStateSize = "Invalid continuous state size for model with ID: ";
  const std::string kExceptionInvalidDiscreteStateSize = "Invalid discrete state size for model with ID: ";
  const std::string kExceptionInvalidStateCovarianceSize = "Invalid continuous state covariance size for model with ID: ";

  /**
   * @brief Perform sanity check on continuous state size of given object state.
   * 
   * @param state 
   */
  inline void sanityCheckContinuousStateSize(const ObjectState& state) {
    int exp_state_size = getContinuousStateSize(state.model_id);
    int state_size = getContinuousStateSize(state);
    if (state_size != exp_state_size)
      throw std::invalid_argument(kExceptionInvalidStateSize + std::to_string(state.model_id) + ", " + std::to_string(state_size) + " != " + std::to_string(exp_state_size));
  }

  /**
   * @brief Perform sanity check on discrete state size of given object state.
   * 
   * @param state 
   */
  inline void sanityCheckDiscreteStateSize(const ObjectState& state) {
    int exp_discrete_state_size = getDiscreteStateSize(state.model_id);
    int discrete_state_size = getDiscreteStateSize(state);
    if (discrete_state_size != exp_discrete_state_size)
      throw std::invalid_argument(kExceptionInvalidDiscreteStateSize + std::to_string(state.model_id) + ", " + std::to_string(discrete_state_size) + " != " + std::to_string(exp_discrete_state_size));
  }

  /**
   * @brief Perform sanity check on continuous state covariance size of given object state.
   * 
   * @param state 
   */
  inline void sanityCheckContinuousStateCovarianceSize(const ObjectState& state) {
    int exp_state_cov_size = getContinuousStateCovarianceSize(state.model_id);
    int state_cov_size = getContinuousStateCovarianceSize(state);
    if (state_cov_size != exp_state_cov_size)
      throw std::invalid_argument(kExceptionInvalidStateCovarianceSize + std::to_string(state.model_id) + ", " + std::to_string(state_cov_size) + " != " + std::to_string(exp_state_cov_size));
  }

  /**
   * @brief Perform sanity check on continuous state of given object state.
   * 
   * @param state 
   */
  inline void sanityCheckContinuousState(const ObjectState& state) {
    sanityCheckContinuousStateSize(state);
  }

  /**
   * @brief Perform sanity check on continuous state of given template object that contains an object state.
   * 
   * @tparam T 
   * @param obj 
   */
  template <typename T>
  inline void sanityCheckContinuousState(const T& obj) {
    sanityCheckContinuousState(obj.state);
  }

  /**
   * @brief Perform sanity check on discrete state of given object state.
   * 
   * @param state 
   */
  inline void sanityCheckDiscreteState(const ObjectState& state) {
    sanityCheckDiscreteStateSize(state);
  }

  /**
   * @brief Perform sanity check on discrete state of given template object that contains an object state.
   * 
   * @tparam T 
   * @param obj 
   */
  template <typename T>
  inline void sanityCheckDiscreteState(const T& obj) {
    sanityCheckDiscreteState(obj.state);
  }

  /**
   * @brief Perform sanity check on continuous state covariance of given object state.
   * 
   * @param state 
   */
  inline void sanityCheckContinuousStateCovariance(const ObjectState& state) {
    sanityCheckContinuousStateCovarianceSize(state);
  }

  /**
   * @brief Perform sanity check on continuous state covariance of given template object that contains an object state.
   * 
   * @tparam T 
   * @param obj 
   */
  template <typename T>
  inline void sanityCheckContinuousStateCovariance(const T& obj) {
    sanityCheckContinuousStateCovariance(obj.state);
  }

  /**
   * @brief Perform sanity check on full state of given object state.
   * 
   * @param state 
   */
  inline void sanityCheckFullState(const ObjectState& state) {
    sanityCheckContinuousState(state);
    sanityCheckDiscreteState(state);
    sanityCheckContinuousStateCovariance(state);
  }

  /**
   * @brief Perform sanity check on full state of of given template object that contains an object state.
   * 
   * @tparam T 
   * @param obj 
   */
  template <typename T>
  inline void sanityCheckFullState(const T& obj) {
    sanityCheckFullState(obj.state);
  }

} // namespace object_access

} // namespace perception_msgs
