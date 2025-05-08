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
 * @file convenience_state_setters.h
 * @brief Convenience setter functions for objects state members
 */

#pragma once

#include <perception_msgs_utils/impl/checks.h>
#include <perception_msgs_utils/impl/state_index.h>
#include <perception_msgs_utils/impl/state_setters.h>

#include <array>

namespace perception_msgs
{

namespace object_access
{

// --- full state/covariance -------------------------------------------------

/**
 * @brief Set the continuous state of a given object state.
 * 
 * @param state 
 * @param val 
 */
inline void setContinuousState(ObjectState & state, const std::vector<double> & val)
{
  state.continuous_state = val;
  sanityCheckContinuousState(state);
}

/**
 * @brief Set the continuous state of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 */
template <typename T>
inline void setContinuousState(T & obj, const std::vector<double> & val)
{
  setContinuousState(obj.state, val);
}

/**
 * @brief Set the discrete state of a given object state.
 * 
 * @param state 
 * @param val 
 */
inline void setDiscreteState(ObjectState & state, const std::vector<long int> & val)
{
  state.discrete_state = val;
  sanityCheckDiscreteState(state);
}

/**
 * @brief Set the discrete state of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 */
template <typename T>
inline void setDiscreteState(T & obj, const std::vector<long int> & val)
{
  setDiscreteState(obj.state, val);
}

/**
 * @brief Set the continuous state covariance of a given object state.
 * 
 * @param state 
 * @param val 
 */
inline void setContinuousStateCovariance(ObjectState & state, const std::vector<double> & val)
{
  state.continuous_state_covariance = val;
  sanityCheckContinuousStateCovariance(state);
}

/**
 * @brief Set the continuous state covariance of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 */
template <typename T>
inline void setContinuousStateCovariance(T & obj, const std::vector<double> & val)
{
  setContinuousStateCovariance(obj.state, val);
}

/**
 * @brief Set the continuous state covariance entry at (i,j) of a given object state.
 * 
 * @param state 
 * @param i 
 * @param j 
 * @param val 
 */
inline void setContinuousStateCovarianceAt(
  ObjectState & state, const unsigned int i, const unsigned int j, const double val)
{
  sanityCheckContinuousStateCovariance(state);
  const int n = getContinuousStateSize(state);
  state.continuous_state_covariance[n * i + j] = val;
}

/**
 * @brief Set the continuous state covariance at (i,j) of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param i 
 * @param j 
 * @param val 
 */
template <typename T>
inline void setContinuousStateCovarianceAt(
  T & obj, const unsigned int i, const unsigned int j, const double val)
{
  setContinuousStateCovarianceAt(obj.state, i, j, val);
}

/**
 * @brief Set the continuous state covariance diagonal of a given object state.
 * 
 * @param state 
 * @param val 
 */
inline void setContinuousStateCovarianceDiagonal(ObjectState & state, const std::vector<double> val)
{
  const int n = getContinuousStateSize(state);
  for (int i = 0; i < n; i++) setContinuousStateCovarianceAt(state, i, i, val[i]);
}

/**
 * @brief Set the continuous state covariance diagonal of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 */
template <typename T>
inline void setContinuousStateCovarianceDiagonal(T & obj, const std::vector<double> val)
{
  setContinuousStateCovarianceDiagonal(obj.state, val);
}

// --- vector quantities -----------------------------------------------------

/**
 * @brief Set the position of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setPosition(ObjectState & state, const gm::Point val, const bool reset_covariance = true)
{
  setX(state, val.x, reset_covariance);
  setY(state, val.y, reset_covariance);
  setZ(state, val.z, reset_covariance);
}

/**
 * @brief Set the position of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setPosition(T & obj, const gm::Point val, const bool reset_covariance = true)
{
  setPosition(obj.state, val, reset_covariance);
}

/**
 * @brief Set the position of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setPosition(ObjectState & state, const std::array<double, 3> & val, const bool reset_covariance = true)
{
  setX(state, val[0], reset_covariance);
  setY(state, val[1], reset_covariance);
  setZ(state, val[2], reset_covariance);
}

/**
 * @brief Set the position of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setPosition(T & obj, const std::array<double, 3> & val, const bool reset_covariance = true)
{
  setPosition(obj.state, val, reset_covariance);
}

/**
 * @brief Set the orientation of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setOrientation(ObjectState & state, const gm::Quaternion & val, const bool reset_covariance = true)
{
  tf2::Quaternion q;
  tf2::fromMsg(val, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  if (hasRoll(state.model_id)) setRoll(state, roll, reset_covariance);
  if (hasPitch(state.model_id)) setPitch(state, pitch, reset_covariance);
  if (hasYaw(state.model_id)) setYaw(state, yaw, reset_covariance);
}

/**
 * @brief Set the orientation of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setOrientation(T & obj, const gm::Quaternion & val, const bool reset_covariance = true)
{
  setOrientation(obj.state, val, reset_covariance);
}

/**
 * @brief Set the orientation of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setOrientation(ObjectState & state, const std::array<double, 3> & val, const bool reset_covariance = true)
{
  if (hasRoll(state.model_id)) setRoll(state, val[0], reset_covariance);
  if (hasPitch(state.model_id)) setPitch(state, val[1], reset_covariance);
  if (hasYaw(state.model_id)) setYaw(state, val[2], reset_covariance);
}

/**
 * @brief Set the orientation of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setOrientation(T & obj, const std::array<double, 3> & val, const bool reset_covariance = true)
{
  setOrientation(obj.state, val, reset_covariance);
}

/**
 * @brief Set the pose of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setPose(ObjectState & state, const gm::Pose & val, const bool reset_covariance = true)
{
  setPosition(state, val.position, reset_covariance);
  setOrientation(state, val.orientation, reset_covariance);
}

/**
 * @brief Set the pose of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setPose(T & obj, const gm::Pose & val, const bool reset_covariance = true)
{
  setPose(obj.state, val, reset_covariance);
}

/**
 * @brief Set the pose of a given object state.
 * 
 * @param state 
 * @param xyz 
 * @param rpy 
 * @param reset_covariance 
 */
inline void setPose(
  ObjectState & state, const std::array<double, 3> & xyz, const std::array<double, 3> & rpy, const bool reset_covariance = true)
{
  setPosition(state, xyz, reset_covariance);
  setOrientation(state, rpy, reset_covariance);
}

/**
 * @brief Set the pose of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param xyz 
 * @param rpy 
 * @param reset_covariance 
 */
template <typename T>
inline void setPose(T & obj, const std::array<double, 3> & xyz, const std::array<double, 3> & rpy, const bool reset_covariance = true)
{
  setPose(obj.state, xyz, rpy, reset_covariance);
}

/**
 * @brief Set the pose covariance of a given object state.
 * 
 * @param state 
 * @param val 
 */
inline void setPoseCovariance(
  ObjectState & state, const gm::PoseWithCovariance::_covariance_type & val)
{
  const int n = 6;
  const int model_id = state.model_id;

  int ix, jx;
  for (int i = 0; i < n; i++) {
    bool i_valid = true;
    switch (i) {
      case 0: i_valid = hasX(model_id);  ix = i_valid ? indexX(model_id) : 0; break;
      case 1: i_valid = hasY(model_id);  ix = i_valid ? indexY(model_id) : 0; break;
      case 2: i_valid = hasZ(model_id);  ix = i_valid ? indexZ(model_id) : 0; break;
      case 3: i_valid = hasRoll(model_id);  ix = i_valid ? indexRoll(model_id) : 0; break;
      case 4: i_valid = hasPitch(model_id); ix = i_valid ? indexPitch(model_id) : 0; break;
      case 5: i_valid = hasYaw(model_id);   ix = i_valid ? indexYaw(model_id) : 0; break;
    }
    if (!i_valid) continue;

    for (int j = 0; j < n; j++) {
      bool j_valid = true;
      switch (j) {
        case 0: j_valid = hasX(model_id);  jx = j_valid ? indexX(model_id) : 0; break;
        case 1: j_valid = hasY(model_id);  jx = j_valid ? indexY(model_id) : 0; break;
        case 2: j_valid = hasZ(model_id);  jx = j_valid ? indexZ(model_id) : 0; break;
        case 3: j_valid = hasRoll(model_id);  jx = j_valid ? indexRoll(model_id) : 0; break;
        case 4: j_valid = hasPitch(model_id); jx = j_valid ? indexPitch(model_id) : 0; break;
        case 5: j_valid = hasYaw(model_id);   jx = j_valid ? indexYaw(model_id) : 0; break;
      }
      if (!j_valid) continue;

      setContinuousStateCovarianceAt(state, ix, jx, val[n * i + j]);
    }
  }
}
 /**
  * @brief Set the pose covariance of a given template object that contains an object state.
  * 
  * @tparam T 
  * @param obj 
  * @param val 
  */
template <typename T>
inline void setPoseCovariance(T & obj, const gm::PoseWithCovariance::_covariance_type & val)
{
  setPoseCovariance(obj.state, val);
}

/**
 * @brief Set the pose with covariance of a given object state.
 * 
 * @param state 
 * @param val 
 */
inline void setPoseWithCovariance(ObjectState & state, const gm::PoseWithCovariance & val)
{
  setPose(state, val.pose, false);
  setPoseCovariance(state, val.covariance);
}

/**
 * @brief Set the pose with covariance of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 */
template <typename T>
inline void setPoseWithCovariance(T & obj, const gm::PoseWithCovariance & val)
{
  setPoseWithCovariance(obj.state, val);
}

/**
 * @brief Set the velocity of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setVelocity(ObjectState & state, const gm::Vector3 & val, const bool reset_covariance = true)
{
  setVelLon(state, val.x, reset_covariance);
  setVelLat(state, val.y, reset_covariance);
}
 /**
  * @brief Set the velocity of a given template object that contains an object state.
  * 
  * @tparam T 
  * @param obj 
  * @param val 
  * @param reset_covariance 
  */
template <typename T>
inline void setVelocity(T & obj, const gm::Vector3 & val, const bool reset_covariance = true)
{
  setVelocity(obj.state, val, reset_covariance);
}

/**
 * @brief Set the velocity of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setVelocity(ObjectState & state, const std::array<double, 2> & val, const bool reset_covariance = true)
{
  setVelLon(state, val[0], reset_covariance);
  setVelLat(state, val[1], reset_covariance);
}

/**
 * @brief Set the velocity of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setVelocity(T & obj, const std::array<double, 2> & val, const bool reset_covariance = true)
{
  setVelocity(obj.state, val, reset_covariance);
}

/**
 * @brief Set the acceleration of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setAcceleration(ObjectState & state, const gm::Vector3 & val, const bool reset_covariance = true)
{
  setAccLon(state, val.x, reset_covariance);
  setAccLat(state, val.y, reset_covariance);
}

/**
 * @brief Set the acceleration of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setAcceleration(T & obj, const gm::Vector3 & val, const bool reset_covariance = true)
{
  setAcceleration(obj.state, val, reset_covariance);
}

/**
 * @brief Set the acceleration of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setAcceleration(ObjectState & state, const std::array<double, 2> & val, const bool reset_covariance = true)
{
  setAccLon(state, val[0], reset_covariance);
  setAccLat(state, val[1], reset_covariance);
}

/**
 * @brief Set the acceleration of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setAcceleration(T & obj, const std::array<double, 2> & val, const bool reset_covariance = true)
{
  setAcceleration(obj.state, val, reset_covariance);
}

// --- alternative state entries ---------------------------------------------

/**
 * @brief Set the roll in degree of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setRollInDeg(ObjectState & state, const double val, const bool reset_covariance = true)
{
  setRoll(state, val * M_PI / 180.0, reset_covariance);
}

/**
 * @brief Set the roll in deg of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setRollInDeg(T & obj, const double val, const bool reset_covariance = true)
{
  setRollInDeg(obj.state, val, reset_covariance);
}

/**
 * @brief Set the pitch in degree of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setPitchInDeg(ObjectState & state, const double val, const bool reset_covariance = true)
{
  setPitch(state, val * M_PI / 180.0, reset_covariance);
}

/**
 * @brief Set the pitch in degree of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setPitchInDeg(T & obj, const double val, const bool reset_covariance = true)
{
  setPitchInDeg(obj.state, val, reset_covariance);
}

/**
 * @brief Set the yaw in degree of a given object state.
 * 
 * @param state 
 * @param val 
 * @param reset_covariance 
 */
inline void setYawInDeg(ObjectState & state, const double val, const bool reset_covariance = true)
{
  setYaw(state, val * M_PI / 180.0, reset_covariance);
}

/**
 * @brief Set the yaw in degree of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param val 
 * @param reset_covariance 
 */
template <typename T>
inline void setYawInDeg(T & obj, const double val, const bool reset_covariance = true)
{
  setYawInDeg(obj.state, val, reset_covariance);
}

/**
 * @brief Set the velocity XYZ and yaw with covariance of a given object state.
 * 
 * @param state 
 * @param vel_xyz_in 
 * @param yaw 
 * @param var_vel_x 
 * @param var_vel_y 
 * @param cov_vel_xy 
 */
inline void setVelocityXYZYawWithCovariance(
  ObjectState & state, const gm::Vector3 & vel_xyz_in, const double yaw, const double var_vel_x,
  const double var_vel_y, const double cov_vel_xy)
{
  gm::PoseWithCovariance vel_lon_lat, vel_xyz;
  vel_xyz.pose.position.x = vel_xyz_in.x;
  vel_xyz.pose.position.y = vel_xyz_in.y;
  vel_xyz.pose.position.z = 0.0;

  vel_xyz.covariance.at(0) = var_vel_x;
  vel_xyz.covariance.at(1) = cov_vel_xy;
  vel_xyz.covariance.at(6) = cov_vel_xy;
  vel_xyz.covariance.at(7) = var_vel_y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, -yaw);
  gm::TransformStamped tf;
  tf.transform.rotation = tf2::toMsg(q);

#ifdef ROS1
  gm::PoseWithCovarianceStamped vel_lon_lat_stamped, vel_xyz_stamped;
  vel_xyz_stamped.pose = vel_xyz;
  tf2::doTransform(vel_xyz_stamped, vel_lon_lat_stamped, tf);
  vel_lon_lat = vel_lon_lat_stamped.pose;
#else
  tf2::doTransform(vel_xyz, vel_lon_lat, tf);
#endif

  setVelocity(state, {vel_lon_lat.pose.position.x, vel_lon_lat.pose.position.y}, false);
  setYaw(state, yaw, false);

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

      setContinuousStateCovarianceAt(state, ix, jx, vel_lon_lat.covariance.at(6 * i + j));
    }
  }
}

/**
 * @brief Set the velocity XYZ and yaw with covariance of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param vel_xyz_in 
 * @param yaw 
 * @param var_vel_x 
 * @param var_vel_y 
 * @param cov_vel_xy 
 */
template <typename T>
inline void setVelocityXYZYawWithCovariance(
  T & obj, const gm::Vector3 & vel_xyz_in, const double yaw, const double var_vel_x,
  const double var_vel_y, const double cov_vel_xy)
{
  return setVelocityXYZYawWithCovariance(obj.state, vel_xyz_in, yaw, var_vel_x, var_vel_y, cov_vel_xy);
}

/**
 * @brief Set the velocity XYZ and yaw of a given object state.
 * 
 * @param state 
 * @param vel_xyz 
 * @param yaw 
 * @param reset_covariance 
 */
inline void setVelocityXYZYaw(ObjectState & state, const gm::Vector3 & vel_xyz, const double yaw, const bool reset_covariance = true)
{
  gm::Vector3 vel_lon_lat, vel_xyz_in;
  vel_xyz_in = vel_xyz;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, -yaw);
  gm::TransformStamped tf;
  tf.transform.rotation = tf2::toMsg(q);
  tf2::doTransform(vel_xyz_in, vel_lon_lat, tf);
  setVelocity(state, vel_lon_lat, reset_covariance);
  setYaw(state, yaw, reset_covariance);
}

/**
 * @brief Set the velocity XYZ and yaw of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param vel_xyz 
 * @param yaw 
 * @param reset_covariance 
 */
template <typename T>
inline void setVelocityXYZYaw(T & obj, const gm::Vector3 & vel_xyz, const double yaw, const bool reset_covariance = true)
{
  setVelocityXYZYaw(obj.state, vel_xyz, yaw, reset_covariance);
}

/**
 * @brief Set the acceleration XYZ and yaw with covariance of a given object state.
 * 
 * @param state 
 * @param acc_xyz_in 
 * @param yaw 
 * @param var_acc_x 
 * @param var_acc_y 
 * @param cov_acc_xy 
 */
inline void setAccelerationXYZYawWithCovariance(
  ObjectState & state, const gm::Vector3 & acc_xyz_in, const double yaw, const double var_acc_x,
  const double var_acc_y, const double cov_acc_xy)
{
  gm::PoseWithCovariance acc_lon_lat, acc_xyz;
  acc_xyz.pose.position.x = acc_xyz_in.x;
  acc_xyz.pose.position.y = acc_xyz_in.y;
  acc_xyz.pose.position.z = 0.0;

  acc_xyz.covariance.at(0) = var_acc_x;
  acc_xyz.covariance.at(1) = cov_acc_xy;
  acc_xyz.covariance.at(6) = cov_acc_xy;
  acc_xyz.covariance.at(7) = var_acc_y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, -yaw);
  gm::TransformStamped tf;
  tf.transform.rotation = tf2::toMsg(q);

#ifdef ROS1
  gm::PoseWithCovarianceStamped acc_lon_lat_stamped, acc_xyz_stamped;
  acc_xyz_stamped.pose = acc_xyz;
  tf2::doTransform(acc_xyz_stamped, acc_lon_lat_stamped, tf);
  acc_lon_lat = acc_lon_lat_stamped.pose;
#else
  tf2::doTransform(acc_xyz, acc_lon_lat, tf);
#endif

  setAcceleration(state, {acc_lon_lat.pose.position.x, acc_lon_lat.pose.position.y}, false);
  setYaw(state, yaw, false);

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

      setContinuousStateCovarianceAt(state, ix, jx, acc_lon_lat.covariance.at(6 * i + j));
    }
  }
}

/**
 * @brief Set the acceleration XYZ and yaw with covariance of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param acc_xyz_in 
 * @param yaw 
 * @param var_acc_x 
 * @param var_acc_y 
 * @param cov_acc_xy 
 */
template <typename T>
inline void setAccelerationXYZYawWithCovariance(
  T & obj, const gm::Vector3 & acc_xyz_in, const double yaw, const double var_acc_x,
  const double var_acc_y, const double cov_acc_xy)
{
  return setAccelerationXYZYawWithCovariance(obj.state, acc_xyz_in, yaw, var_acc_x, var_acc_y, cov_acc_xy);
}

/**
 * @brief Set the acceleration XYZ and yaw of a given template object that contains an object state.
 * 
 * @param state 
 * @param acc_xyz 
 * @param yaw 
 * @param reset_covariance 
 */
inline void setAccelerationXYZYaw(
  ObjectState & state, const gm::Vector3 & acc_xyz, const double yaw, const bool reset_covariance = true)
{
  gm::Vector3 acc_lon_lat, acc_xyz_in;
  acc_xyz_in = acc_xyz;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, -yaw);
  gm::TransformStamped tf;
  tf.transform.rotation = tf2::toMsg(q);
  tf2::doTransform(acc_xyz_in, acc_lon_lat, tf);
  setAcceleration(state, acc_lon_lat, reset_covariance);
  setYaw(state, yaw, reset_covariance);
}

/**
 * @brief Set the acceleration XYZ and yaw of a given template object that contains an object state.
 * 
 * @tparam T 
 * @param obj 
 * @param acc_xyz 
 * @param yaw 
 * @param reset_covariance 
 */
template <typename T>
inline void setAccelerationXYZYaw(T & obj, const gm::Vector3 & acc_xyz, const double yaw, const bool reset_covariance = true)
{
  setAccelerationXYZYaw(obj.state, acc_xyz, yaw, reset_covariance);
}

}  // namespace object_access

}  // namespace perception_msgs
