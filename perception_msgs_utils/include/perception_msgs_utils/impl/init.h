// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

/**
 * @file init.h
 * @brief Object state initializers
 */

#pragma once

#include <perception_msgs_utils/impl/constants.h>
#include <perception_msgs_utils/impl/convenience_state_setters.h>
#include <perception_msgs_utils/impl/utils.h>


namespace perception_msgs {

namespace object_access {
  
  /**
   * @brief This function initializes a given object state.
   * 
   * @param state 
   * @param model_id 
   */
  inline void initializeState(ObjectState& state, const unsigned char& model_id) {
    state.model_id = model_id;
    setContinuousState(state, std::vector<double>(getContinuousStateSize(model_id), CONTINUOUS_STATE_INIT));
    setDiscreteState(state, std::vector<long int>(getDiscreteStateSize(model_id), DISCRETE_STATE_INIT));
    setContinuousStateCovariance(state, std::vector<double>(getContinuousStateCovarianceSize(model_id), CONTINUOUS_STATE_COVARIANCE_INIT));
    setContinuousStateCovarianceDiagonal(state, std::vector<double>(getContinuousStateSize(model_id), CONTINUOUS_STATE_COVARIANCE_INVALID));
  }

  /**
   * @brief This function initializes a given template object that contains an object state.
   * 
   * @tparam T 
   * @param obj 
   * @param model_id 
   */
  template <typename T>
  inline void initializeState(T& obj, const unsigned char& model_id) {
    initializeState(obj.state, model_id);
  }

} // namespace object_access

} // namespace perception_msgs
