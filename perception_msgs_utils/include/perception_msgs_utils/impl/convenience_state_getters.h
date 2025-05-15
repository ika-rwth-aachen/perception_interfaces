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
 * @file convenience_state_getters.h
 * @brief Convenience getter functions for object state members
 */

#pragma once

#include <perception_msgs_utils/impl/checks.h>
#include <perception_msgs_utils/impl/state_getters.h>
#include <perception_msgs_utils/impl/state_index.h>

#include <algorithm>
#include <cmath>

namespace perception_msgs {

namespace object_access {

// --- full state/covariance -------------------------------------------------

/**
 * @brief Get the continuous state for a given object state.
 * 
 * @param state 
 * @return std::vector<double> 
 */
inline std::vector<double> getContinuousState(const ObjectState& state) {
  sanityCheckContinuousState(state);
  return state.continuous_state;
}

/**
 * @brief Get the continuous state for a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return std::vector<double> 
 */
template <typename T>
inline std::vector<double> getContinuousState(const T& obj) {
  return getContinuousState(obj.state);
}

/**
 * @brief Get the discrete state for a given object state.
 * 
 * @param state 
 * @return std::vector<long int> 
 */
inline std::vector<long int> getDiscreteState(const ObjectState& state) {
  sanityCheckContinuousState(state);
  return state.discrete_state;
}

/**
 * @brief Get the discrete state for a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return std::vector<long int> 
 */
template <typename T>
inline std::vector<long int> getDiscreteState(const T& obj) {
  return getDiscreteState(obj.state);
}

/**
 * @brief Get the continuous state covariance for a given object state.
 * 
 * @param state 
 * @return std::vector<double> 
 */
inline std::vector<double> getContinuousStateCovariance(const ObjectState& state) {
  sanityCheckContinuousState(state);
  return state.continuous_state_covariance;
}

/**
 * @brief Get the continuous state covariance for a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return std::vector<double> 
 */
template <typename T>
inline std::vector<double> getContinuousStateCovariance(const T& obj) {
  return getContinuousStateCovariance(obj.state);
}

/**
 * @brief Get the continuous state covariance entry (i,j) for a given object state.
 * 
 * @param state 
 * @param i 
 * @param j 
 * @return double 
 */
inline double getContinuousStateCovarianceAt(const ObjectState& state, const unsigned int i, const unsigned int j) {
  const int n = getContinuousStateSize(state);
  const std::vector<double> covariance = getContinuousStateCovariance(state);
  return covariance[n * i + j];
}

/**
 * @brief Get the continuous state covariance entry (i,j) for a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param i 
 * @param j 
 * @return double 
 */
template <typename T>
inline double getContinuousStateCovarianceAt(const T& obj, const unsigned int i, const unsigned int j) {
  return getContinuousStateCovarianceAt(obj.state, i, j);
}

/**
 * @brief Get the continuous state covariance diagonal for a given object state.
 * 
 * @param state 
 * @return std::vector<double> 
 */
inline std::vector<double> getContinuousStateCovarianceDiagonal(const ObjectState& state) {
  const int n = getContinuousStateSize(state);
  std::vector<double> diagonal(n);
  for (int i = 0; i < n; i++) diagonal[i] = getContinuousStateCovarianceAt(state, i, i);
  return diagonal;
}

/**
 * @brief Get the continuous state covariance diagonal for a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return std::vector<double> 
 */
template <typename T>
inline std::vector<double> getContinuousStateCovarianceDiagonal(const T& obj) {
  return getContinuousStateCovarianceDiagonal(obj.state);
}

// --- vector quantities -----------------------------------------------------

/**
 * @brief Get the position of a given object state.
 * 
 * @param state 
 * @return gm::Point 
 */
inline gm::Point getPosition(const ObjectState& state) {
  gm::Point position;
  position.x = getX(state);
  position.y = getY(state);
  position.z = getZ(state);
  return position;
}

/**
 * @brief Get the position of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::Point 
 */
template <typename T>
inline gm::Point getPosition(const T& obj) {
  return getPosition(obj.state);
}

/**
 * @brief Get the orientation of a given object state.
 * 
 * @param state 
 * @return gm::Quaternion 
 */
inline gm::Quaternion getOrientation(const ObjectState& state) {
  tf2::Quaternion q;
  double roll{0}, pitch{0}, yaw{0};
  if (hasRoll(state.model_id)) roll = getRoll(state);
  if (hasPitch(state.model_id)) pitch = getPitch(state);
  if (hasYaw(state.model_id)) yaw = getYaw(state);
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

/**
 * @brief Get the orientation of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::Quaternion 
 */
template <typename T>
inline gm::Quaternion getOrientation(const T& obj) {
  return getOrientation(obj.state);
}

/**
 * @brief Get the pose of a given object state.
 * 
 * @param state 
 * @return gm::Pose 
 */
inline gm::Pose getPose(const ObjectState& state) {
  gm::Pose pose;
  pose.position = getPosition(state);
  pose.orientation = getOrientation(state);
  return pose;
}

/**
 * @brief Get the object's geometric center position
 * 
 * @param state The state to get the center position from
 * @return gm::Point The position of the object's geometric center
 */
inline gm::Point getCenterPosition(const ObjectState& state) {
  gm::Point position = getPosition(state);
  const auto orientation = getOrientation(state);
  const gm::Vector3 offset_to_center = state.reference_point.translation_to_geometric_center;
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  const tf2::Vector3 offset_to_center_tf2(offset_to_center.x, offset_to_center.y, offset_to_center.z);
  const tf2::Vector3 rotated_offset_to_center = tf2::quatRotate(q, offset_to_center_tf2);
  position.x += rotated_offset_to_center.x();
  position.y += rotated_offset_to_center.y();
  position.z += rotated_offset_to_center.z();
  return position;
}

/**
 * @brief Get the object's geometric center position
 * 
 * @tparam T 
 * @param object The object to get the center position from
 * @return gm::Position The position of the object's geometric center
 */
template <typename T>
inline gm::Point getCenterPosition(const T& object) {
  return getCenterPosition(object.state);
}

/**
 * @brief Get the pose of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::Pose 
 */
template <typename T>
inline gm::Pose getPose(const T& obj) {
  return getPose(obj.state);
}

/**
 * @brief Get the pose covariance of a given object state.
 * 
 * @param state 
 * @return std::vector<double> 
 */
inline std::vector<double> getPoseCovariance(const ObjectState& state) {
  const int n = 6;
  const int model_id = state.model_id;
  std::vector<double> pose_covariance(n * n, 0.0);
  int ix, jx;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == 0 && hasX(model_id))
        ix = indexX(model_id);
      else if (i == 1 && hasY(model_id))
        ix = indexY(model_id);
      else if (i == 2 && hasZ(model_id))
        ix = indexZ(model_id);
      else if (i == 3 && hasRoll(model_id))
        ix = indexRoll(model_id);
      else if (i == 4 && hasPitch(model_id))
        ix = indexPitch(model_id);
      else if (i == 5 && hasYaw(model_id))
        ix = indexYaw(model_id);
      else
        continue;

      if (j == 0 && hasX(model_id))
        jx = indexX(model_id);
      else if (j == 1 && hasY(model_id))
        jx = indexY(model_id);
      else if (j == 2 && hasZ(model_id))
        jx = indexZ(model_id);
      else if (j == 3 && hasRoll(model_id))
        jx = indexRoll(model_id);
      else if (j == 4 && hasPitch(model_id))
        jx = indexPitch(model_id);
      else if (j == 5 && hasYaw(model_id))
        jx = indexYaw(model_id);
      else
        continue;

      pose_covariance[n * i + j] = getContinuousStateCovarianceAt(state, ix, jx);
    }
  }
  return pose_covariance;
}

/**
 * @brief Get the pose covariance of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return std::vector<double> 
 */
template <typename T>
inline std::vector<double> getPoseCovariance(const T& obj) {
  return getPoseCovariance(obj.state);
}

/**
 * @brief Get the pose with covariance of a given object state.
 * 
 * 
 * @param state 
 * @return gm::PoseWithCovariance 
 */
inline gm::PoseWithCovariance getPoseWithCovariance(const ObjectState& state) {
  gm::PoseWithCovariance pose_with_covariance;
  pose_with_covariance.pose = getPose(state);
  std::vector<double> covariance_vector = getPoseCovariance(state);
  gm::PoseWithCovariance::_covariance_type covariance;
  std::copy(covariance_vector.begin(), covariance_vector.end(), covariance.begin());
  pose_with_covariance.covariance = covariance;
  return pose_with_covariance;
}

/**
 * @brief Get the pose with covariance of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::PoseWithCovariance 
 */
template <typename T>
inline gm::PoseWithCovariance getPoseWithCovariance(const T& obj) {
  return getPoseWithCovariance(obj.state);
}

/**
 * @brief Get the velocity of a given object state.
 * 
 * @param state 
 * @return gm::Vector3 
 */
inline gm::Vector3 getVelocity(const ObjectState& state) {
  gm::Vector3 velocity;
  velocity.x = getVelLon(state);
  velocity.y = getVelLat(state);
  velocity.z = 0.0;
  return velocity;
}

/**
 * @brief Get the velocity of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::Vector3 
 */
template <typename T>
inline gm::Vector3 getVelocity(const T& obj) {
  return getVelocity(obj.state);
}

/**
 * @brief Get the velocity magnitude of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getVelocityMagnitude(const ObjectState& state) {
  gm::Vector3 vel = getVelocity(state);
  using namespace std;
  return sqrt(pow(vel.x, 2) + pow(vel.y, 2) + pow(vel.z, 2));
}

/**
 * @brief Get the velocity magnitude of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getVelocityMagnitude(const T& obj) {
  return getVelocityMagnitude(obj.state);
}

/**
 * @brief Get the acceleration of a given object state.
 * @param state 
 * @return gm::Vector3 
 */
inline gm::Vector3 getAcceleration(const ObjectState& state) {
  gm::Vector3 acceleration;
  acceleration.x = getAccLon(state);
  acceleration.y = getAccLat(state);
  acceleration.z = 0.0;
  return acceleration;
}

/**
 * @brief Get the acceleration of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::Vector3 
 */
template <typename T>
inline gm::Vector3 getAcceleration(const T& obj) {
  return getAcceleration(obj.state);
}

/**
 * @brief Get the acceleration magnitude of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getAccelerationMagnitude(const ObjectState& state) {
  gm::Vector3 acc = getAcceleration(state);
  using namespace std;
  return sqrt(pow(acc.x, 2) + pow(acc.y, 2) + pow(acc.z, 2));
}

/**
 * @brief Get the acceleration magnitude of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getAccelerationMagnitude(const T& obj) {
  return getAccelerationMagnitude(obj.state);
}

// --- alternative state entries ---------------------------------------------

/**
 * @brief Get the roll in degree of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getRollInDeg(const ObjectState& state) { return getRoll(state) * 180.0 / M_PI; }

/**
 * @brief Get the roll in degree of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getRollInDeg(const T& obj) {
  return getRollInDeg(obj.state);
}

/**
 * @brief Get the pitch in degree of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getPitchInDeg(const ObjectState& state) { return getPitch(state) * 180.0 / M_PI; }

/**
 * @brief Get the pitch in degree of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getPitchInDeg(const T& obj) {
  return getPitchInDeg(obj.state);
}

/**
 * @brief Get the yaw in degree of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getYawInDeg(const ObjectState& state) { return getYaw(state) * 180.0 / M_PI; }

/**
 * @brief Get the yaw in degree of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getYawInDeg(const T& obj) {
  return getYawInDeg(obj.state);
}

/**
 * @brief Get the velocity XYZ with covariance of a object state.
 * 
 * @param state 
 * @return gm::PoseWithCovariance 
 */
inline gm::PoseWithCovariance getVelocityXYZWithCovariance(const ObjectState& state) {
  gm::PoseWithCovariance vel_lon_lat, vel_xyz;
  vel_lon_lat.pose.position.x = getVelLon(state);
  vel_lon_lat.pose.position.y = getVelLat(state);
  vel_lon_lat.pose.position.z = 0.0;

  int ix, jx, n = 2;
  auto model_id = state.model_id;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == 0 && hasVelLon(model_id))
        ix = indexVelLon(model_id);
      else if (i == 1 && hasVelLat(model_id))
        ix = indexVelLat(model_id);
      else
        continue;

      if (j == 0 && hasVelLon(model_id))
        jx = indexVelLon(model_id);
      else if (j == 1 && hasVelLat(model_id))
        jx = indexVelLat(model_id);
      else
        continue;

      vel_lon_lat.covariance.at(6 * i + j) = getContinuousStateCovarianceAt(state, ix, jx);
    }
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, getYaw(state));
  gm::TransformStamped tf;
  tf.transform.rotation = tf2::toMsg(q);

#ifdef ROS1
  gm::PoseWithCovarianceStamped vel_lon_lat_stamped, vel_xyz_stamped;
  vel_lon_lat_stamped.pose = vel_lon_lat;
  tf2::doTransform(vel_lon_lat_stamped, vel_xyz_stamped, tf);
  vel_xyz = vel_xyz_stamped.pose;
#else
  tf2::doTransform(vel_lon_lat, vel_xyz, tf);
#endif

  return vel_xyz;
}

/**
 * @brief Get the velocity XYZ with covariance of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::PoseWithCovariance 
 */
template <typename T>
inline gm::PoseWithCovariance getVelocityXYZWithCovariance(const T& obj) {
  return getVelocityXYZWithCovariance(obj.state);
}

/**
 * @brief Get the velocity XYZ of a given object state.
 * 
 * @param state 
 * @return gm::Vector3 
 */
inline gm::Vector3 getVelocityXYZ(const ObjectState& state) {
  gm::Vector3 vel_lon_lat, vel_xyz;
  vel_lon_lat = getVelocity(state);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, getYaw(state));
  gm::TransformStamped tf;
  tf.transform.rotation = tf2::toMsg(q);
  tf2::doTransform(vel_lon_lat, vel_xyz, tf);
  return vel_xyz;
}

/**
 * @brief Get the velocity XYZ of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::Vector3 
 */
template <typename T>
inline gm::Vector3 getVelocityXYZ(const T& obj) {
  return getVelocityXYZ(obj.state);
}

/**
 * @brief Get the x-velocity of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getVelX(const ObjectState& state) { return getVelocityXYZ(state).x; }

/**
 * @brief Get the x-velocity of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getVelX(const T& obj) {
  return getVelX(obj.state);
}

/**
 * @brief Get the y-velocity of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getVelY(const ObjectState& state) { return getVelocityXYZ(state).y; }

/**
 * @brief Get the y-velocity of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getVelY(const T& obj) {
  return getVelY(obj.state);
}

/**
 * @brief Get the acceleration XYZ with covariance of a given object state.
 * 
 * @param state 
 * @return gm::PoseWithCovariance 
 */
inline gm::PoseWithCovariance getAccelerationXYZWithCovariance(const ObjectState& state) {
  gm::PoseWithCovariance acc_lon_lat, acc_xyz;
  acc_lon_lat.pose.position.x = getAccLon(state);
  acc_lon_lat.pose.position.y = getAccLat(state);
  acc_lon_lat.pose.position.z = 0.0;

  int ix, jx, n = 2;
  auto model_id = state.model_id;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == 0 && hasAccLon(model_id))
        ix = indexAccLon(model_id);
      else if (i == 1 && hasAccLat(model_id))
        ix = indexAccLat(model_id);
      else
        continue;

      if (j == 0 && hasAccLon(model_id))
        jx = indexAccLon(model_id);
      else if (j == 1 && hasAccLat(model_id))
        jx = indexAccLat(model_id);
      else
        continue;

      acc_lon_lat.covariance.at(6 * i + j) = getContinuousStateCovarianceAt(state, ix, jx);
    }
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, getYaw(state));
  gm::TransformStamped tf;
  tf.transform.rotation = tf2::toMsg(q);
#ifdef ROS1
  gm::PoseWithCovarianceStamped acc_lon_lat_stamped, acc_xyz_stamped;
  acc_lon_lat_stamped.pose = acc_lon_lat;
  tf2::doTransform(acc_lon_lat_stamped, acc_xyz_stamped, tf);
  acc_xyz = acc_xyz_stamped.pose;
#else
  tf2::doTransform(acc_lon_lat, acc_xyz, tf);
#endif
  return acc_xyz;
}

/**
 * @brief Get the acceleration XYZ with covariance of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::PoseWithCovariance 
 */
template <typename T>
inline gm::PoseWithCovariance getAccelerationXYZWithCovariance(const T& obj) {
  return getAccelerationXYZWithCovariance(obj.state);
}

/**
 * @brief Get the acceleration XYZ of a given object state.
 * 
 * @param state 
 * @return gm::Vector3 
 */
inline gm::Vector3 getAccelerationXYZ(const ObjectState& state) {
  gm::Vector3 acc_lon_lat, acc_xyz;
  acc_lon_lat = getAcceleration(state);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, getYaw(state));
  gm::TransformStamped tf;
  tf.transform.rotation = tf2::toMsg(q);
  tf2::doTransform(acc_lon_lat, acc_xyz, tf);
  return acc_xyz;
}

/**
 * @brief Get the acceleration XYZ of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return gm::Vector3 
 */
template <typename T>
inline gm::Vector3 getAccelerationXYZ(const T& obj) {
  return getAccelerationXYZ(obj.state);
}

/**
 * @brief Get the x-acceleration of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getAccX(const ObjectState& state) { return getAccelerationXYZ(state).x; }

/**
 * @brief Get the x-acceleration of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getAccX(const T& obj) {
  return getAccX(obj.state);
}

/**
 * @brief Get the y-acceleration of a given object state.
 * 
 * @param state 
 * @return double 
 */
inline double getAccY(const ObjectState& state) { return getAccelerationXYZ(state).y; }

/**
 * @brief Get the y-acceleration of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return double 
 */
template <typename T>
inline double getAccY(const T& obj) {
  return getAccY(obj.state);
}

// --- misc ------------------------------------------------------------------

/**
 * @brief Get the classification with highest probability of a given object state.
 * 
 * @param state 
 * @return ObjectClassification 
 */
inline ObjectClassification getClassWithHighestProbability(const ObjectState& state) {
  ObjectClassification highest_prob_class;
  auto highest_prob_class_it = std::max_element(
      state.classifications.begin(), state.classifications.end(),
      [](const ObjectClassification& c1, const ObjectClassification& c2) { return c1.probability < c2.probability; });
  if (highest_prob_class_it == state.classifications.end()) {
    highest_prob_class.probability = -1.0;
    highest_prob_class.type = ObjectClassification::UNKNOWN;
  } else {
    highest_prob_class = *highest_prob_class_it;
  }
  return highest_prob_class;
}

/**
 * @brief Get the classification with highest probability of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @return ObjectClassification 
 */
template <typename T>
inline ObjectClassification getClassWithHighestProbability(const T& obj) {
  return getClassWithHighestProbability(obj.state);
}

}  // namespace object_access

}  // namespace perception_msgs
