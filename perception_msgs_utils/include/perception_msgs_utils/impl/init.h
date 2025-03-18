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
