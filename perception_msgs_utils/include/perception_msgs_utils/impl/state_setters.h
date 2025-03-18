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
 * @file state_setters.h
 * @brief Setter functions for objects state members
 */

#pragma once

#include <perception_msgs_utils/impl/state_index.h>
#include <perception_msgs_utils/impl/checks.h>
#include <perception_msgs_utils/impl/utils.h>


namespace perception_msgs {

namespace object_access {

  /**
   * @brief Set the x-position for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setX(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexX(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the x-position for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setX(T& obj, const double val, const bool reset_covariance = true) {
    setX(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the y-position for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setY(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexY(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the y-position for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setY(T& obj, const double val, const bool reset_covariance = true) {
    setY(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the z-position for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setZ(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexZ(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the z-position for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setZ(T& obj, const double val, const bool reset_covariance = true) {
    setZ(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the longitdunial velocity for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setVelLon(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexVelLon(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the longitudinal velocity for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setVelLon(T& obj, const double val, const bool reset_covariance = true) {
    setVelLon(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the lateral velocity for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setVelLat(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexVelLat(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the lateral velocity for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setVelLat(T& obj, const double val, const bool reset_covariance = true) {
    setVelLat(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the longitudinal acceleration for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setAccLon(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexAccLon(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the longitudinal acceleration for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setAccLon(T& obj, const double val, const bool reset_covariance = true) {
    setAccLon(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the lateral acceleration for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setAccLat(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexAccLat(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the lateral acceleration for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setAccLat(T& obj, const double val, const bool reset_covariance = true) {
    setAccLat(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the roll for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setRoll(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    double capped_val = val;
    while (capped_val > M_PI) capped_val -= 2 * M_PI;
    while (capped_val < -M_PI) capped_val += 2 * M_PI;
    const int idx = indexRoll(state.model_id);
    state.continuous_state[idx] = capped_val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the roll for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setRoll(T& obj, const double val, const bool reset_covariance = true) {
    setRoll(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the roll-rate for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setRollRate(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexRollRate(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the roll-rate for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setRollRate(T& obj, const double val, const bool reset_covariance = true) {
    setRollRate(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the pitch for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setPitch(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    double capped_val = val;
    while (capped_val > M_PI) capped_val -= 2 * M_PI;
    while (capped_val < -M_PI) capped_val += 2 * M_PI;
    const int idx = indexPitch(state.model_id);
    state.continuous_state[idx] = capped_val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the pitch for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setPitch(T& obj, const double val, const bool reset_covariance = true) {
    setPitch(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the pitch-rate for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setPitchRate(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexPitchRate(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the pitch-rate for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setPitchRate(T& obj, const double val, const bool reset_covariance = true) {
    setPitchRate(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the yaw for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setYaw(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    double capped_val = val;
    while (capped_val > M_PI) capped_val -= 2 * M_PI;
    while (capped_val < -M_PI) capped_val += 2 * M_PI;
    const int idx = indexYaw(state.model_id);
    state.continuous_state[idx] = capped_val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the yaw for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setYaw(T& obj, const double val, const bool reset_covariance = true) {
    setYaw(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the yaw-rate for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setYawRate(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexYawRate(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the yaw-rate for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setYawRate(T& obj, const double val, const bool reset_covariance = true) {
    setYawRate(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the ackermann steering angle for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setSteeringAngleAck(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexSteeringAngleAck(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the ackermann steering angle for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setSteeringAngleAck(T& obj, const double val, const bool reset_covariance = true) {
    setSteeringAngleAck(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the ackermann steering angle rate for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setSteeringAngleRateAck(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexSteeringAngleRateAck(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the ackermann steering angle rate for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setSteeringAngleRateAck(T& obj, const double val, const bool reset_covariance = true) {
    setSteeringAngleRateAck(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the front wheel angle for a given object state.
   * 
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setSteeringAngleFront(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexSteeringAngleFront(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the front wheel angle for a given template object that contains an object state.
   * 
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setSteeringAngleFront(T& obj, const double val, const bool reset_covariance = true) {
    setSteeringAngleFront(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the rear wheel angle for a given object state.
   * 
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setSteeringAngleRear(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexSteeringAngleRear(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the rear wheel angle for a given template object that contains an object state.
   * 
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setSteeringAngleRear(T& obj, const double val, const bool reset_covariance = true) {
    setSteeringAngleRear(obj.state, val, reset_covariance);
  }
  
  /**
   * @brief Set the width for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setWidth(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexWidth(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the width for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setWidth(T& obj, const double val, const bool reset_covariance = true) {
    setWidth(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the length for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setLength(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexLength(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the length for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setLength(T& obj, const double val, const bool reset_covariance = true) {
    setLength(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the height for a given object state.
   *
   * @param state
   * @param val
   * @param reset_covariance
   */
  inline void setHeight(ObjectState& state, const double val, const bool reset_covariance = true) {
    sanityCheckContinuousState(state);
    const int idx = indexHeight(state.model_id);
    state.continuous_state[idx] = val;
    if (reset_covariance) setContinuousStateCovarianceToUnknownAt(state, idx, idx);
  }

  /**
   * @brief Set the height for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   * @param reset_covariance
   */
  template <typename T>
  inline void setHeight(T& obj, const double val, const bool reset_covariance = true) {
    setHeight(obj.state, val, reset_covariance);
  }

  /**
   * @brief Set the standstill indication for a given object state.
   *
   * @param state
   * @param val
   */
  inline void setStandstill(ObjectState& state, const bool val) {
    sanityCheckDiscreteState(state);
    const int idx = indexStandstill(state.model_id);
    state.discrete_state[idx] = val;
  }

  /**
   * @brief Set the standstill indication for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @param val
   */
  template <typename T>
  inline void setStandstill(T& obj, const bool val) {
    setStandstill(obj.state, val);
  }

  /**
   * @brief Set the traffic light state for a given object state.
   *
   * @param state
   * @param val
  */
  inline void setTrafficLightState(ObjectState& state, const uint8_t val) {
    sanityCheckDiscreteState(state);
    const int idx = indexTrafficLightState(state.model_id);
    state.discrete_state[idx] = val;
  }

  /**
   * @brief Set the traffic light state for a given template object that contains an object state.
   *
   * @tparam T
   *
   * @param obj
   * @param val
  */
  template <typename T>
  inline void setTrafficLightState(T& obj, const uint8_t val) {
    setTrafficLightState(obj.state, val);
  }

  /**
   * @brief Set the traffic light type for a given object state.
   *
   * @param state
   * @param val
  */
  inline void setTrafficLightType(ObjectState& state, const uint8_t val) {
    sanityCheckDiscreteState(state);
    const int idx = indexTrafficLightType(state.model_id);
    state.discrete_state[idx] = val;
  }

  /**
   * @brief Set the traffic light type for a given template object that contains an object state.
   *
   * @tparam T
   *
   * @param obj
   * @param val
  */
  template <typename T>
  inline void setTrafficLightType(T& obj, const uint8_t val) {
    setTrafficLightType(obj.state, val);
  }

} // namespace object_access

} // namespace perception_msgs
