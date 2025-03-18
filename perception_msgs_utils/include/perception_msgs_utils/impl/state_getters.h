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
 * @file state_getters.h
 * @brief Getter functions for objects state members
 */

#pragma once

#include <perception_msgs_utils/impl/state_index.h>
#include <perception_msgs_utils/impl/checks.h>


namespace perception_msgs {

namespace object_access {
  /**
   * @brief Get the x-position for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getX(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexX(state.model_id)];
  }


  /**
   * @brief Get the x-position for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getX(const T& obj) {
    return getX(obj.state);
  }

  /**
   * @brief Get the y-position for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getY(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexY(state.model_id)];
  }

  /**
   * @brief Get the y-position for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getY(const T& obj) {
    return getY(obj.state);
  }

  /**
   * @brief Get the z-position for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getZ(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexZ(state.model_id)];
  }

  /**
   * @brief Get the z-position for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getZ(const T& obj) {
    return getZ(obj.state);
  }

  /**
   * @brief Get the longitudinal velocity for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getVelLon(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexVelLon(state.model_id)];
  }

  /**
   * @brief Get the longitudinal velocity for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getVelLon(const T& obj) {
    return getVelLon(obj.state);
  }

  /**
   * @brief Get the longitudinal velocity for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getVelLat(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexVelLat(state.model_id)];
  }

  /**
   * @brief Get the longitudinal velocity for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getVelLat(const T& obj) {
    return getVelLat(obj.state);
  }

  /**
   * @brief Get the longitudinal acceleration for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getAccLon(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexAccLon(state.model_id)];
  }

  /**
   * @brief Get the longitudinal acceleration for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getAccLon(const T& obj) {
    return getAccLon(obj.state);
  }

  /**
   * @brief Get the lateral acceleration for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getAccLat(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexAccLat(state.model_id)];
  }

  /**
   * @brief Get the lateral acceleration for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getAccLat(const T& obj) {
    return getAccLat(obj.state);
  }

  /**
   * @brief Get the roll for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getRoll(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexRoll(state.model_id)];
  }

  /**
   * @brief Get the roll for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getRoll(const T& obj) {
    return getRoll(obj.state);
  }

  /**
   * @brief Get the roll-rate for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getRollRate(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexRollRate(state.model_id)];
  }

  /**
   * @brief Get the roll-rate for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getRollRate(const T& obj) {
    return getRollRate(obj.state);
  }

  /**
   * @brief Get the pitch for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getPitch(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexPitch(state.model_id)];
  }

  /**
   * @brief Get the pitch for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getPitch(const T& obj) {
    return getPitch(obj.state);
  }

  /**
   * @brief Get the pitch-rate for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getPitchRate(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexPitchRate(state.model_id)];
  }

  /**
   * @brief Get the pitch-rate for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getPitchRate(const T& obj) {
    return getPitchRate(obj.state);
  }

  /**
   * @brief Get the yaw for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getYaw(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexYaw(state.model_id)];
  }

  /**
   * @brief Get the yaw for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getYaw(const T& obj) {
    return getYaw(obj.state);
  }

  /**
   * @brief Get the yaw-rate for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getYawRate(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexYawRate(state.model_id)];
  }

  /**
   * @brief Get the yaw-rate for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getYawRate(const T& obj) {
    return getYawRate(obj.state);
  }

  /**
   * @brief Get the ackermann steering angle for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getSteeringAngleAck(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexSteeringAngleAck(state.model_id)];
  }

  /**
   * @brief Get the ackermann steering angle for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getSteeringAngleAck(const T& obj) {
    return getSteeringAngleAck(obj.state);
  }

  /**
   * @brief Get the steering angle rate for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getSteeringAngleRateAck(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexSteeringAngleRateAck(state.model_id)];
  }

  /**
   * @brief Get the steering angle rate for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getSteeringAngleRateAck(const T& obj) {
    return getSteeringAngleRateAck(obj.state);
  }

  /**
   * @brief Get the front wheel angle for a given object state.
   * 
   * @param state
   * @return double
   */
  inline double getSteeringAngleFront(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexSteeringAngleFront(state.model_id)];
  }

  /**
   * @brief Get the front wheel angle for a given template object that contains an object state.
   * 
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getSteeringAngleFront(const T& obj) {
    return getSteeringAngleFront(obj.state);
  }

  /**
   * @brief Get the rear wheel angle for a given object state.
   * 
   * @param state
   * @return double
   */
  inline double getSteeringAngleRear(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexSteeringAngleRear(state.model_id)];
  }

  /**
   * @brief Get the rear wheel angle for a given template object that contains an object state.
   * 
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getSteeringAngleRear(const T& obj) {
    return getSteeringAngleRear(obj.state);
  }

  /**
   * @brief Get the width for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getWidth(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexWidth(state.model_id)];
  }

  /**
   * @brief Get the width for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getWidth(const T& obj) {
    return getWidth(obj.state);
  }

  /**
   * @brief Get the length for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getLength(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexLength(state.model_id)];
  }

  /**
   * @brief Get the length for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getLength(const T& obj) {
    return getLength(obj.state);
  }

  /**
   * @brief Get the height for a given object state.
   *
   * @param state
   * @return double
   */
  inline double getHeight(const ObjectState& state) {
    sanityCheckContinuousState(state);
    return state.continuous_state[indexHeight(state.model_id)];
  }

  /**
   * @brief Get the height for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return double
   */
  template <typename T>
  inline double getHeight(const T& obj) {
    return getHeight(obj.state);
  }

  /**
   * @brief Get standstill indication for a given object state.
   *
   * @param state
   * @return true
   * @return false
   */
  inline bool getStandstill(const ObjectState& state) {
    sanityCheckDiscreteState(state);
    return state.discrete_state[indexStandstill(state.model_id)];
  }

  /**
   * @brief Get the standstill indication for a given template object that contains an object state.
   *
   * @tparam T
   * @param obj
   * @return true
   * @return false
   */
  template <typename T>
  inline bool getStandstill(const T& obj) {
    return getStandstill(obj.state);
  }

  /**
   * @brief Get the traffic light state for a given object state.
   *
   * @param state
   *
   * @return uint8_t
  */
  inline uint8_t getTrafficLightState(const ObjectState& state) {
    sanityCheckDiscreteState(state);
    return state.discrete_state[indexTrafficLightState(state.model_id)];
  }

  /**
   * @brief Get the traffic light state for a given template object that contains an object state.
   *
   * @tparam T
   *
   * @param obj
   *
   * @return uint8_t
  */
  template <typename T>
  inline uint8_t getTrafficLightState(const T& obj) {
    return getTrafficLightState(obj.state);
  }

  /**
   * @brief Get the traffic light type for a given object state.
   *
   * @param state
   *
   * @return uint8_t
  */
  inline uint8_t getTrafficLightType(const ObjectState& state) {
    sanityCheckDiscreteState(state);
    return state.discrete_state[indexTrafficLightType(state.model_id)];
  }

  /**
   * @brief Get the traffic light type for a given template object that contains an object state.
   *
   * @tparam T
   *
   * @param obj
   *
   * @return uint8_t
  */
  template <typename T>
  inline uint8_t getTrafficLightType(const T& obj) {
    return getTrafficLightType(obj.state);
  }

} // namespace object_access

} // namespace perception_msgs
