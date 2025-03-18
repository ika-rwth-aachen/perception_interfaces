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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

using namespace perception_msgs;
using namespace perception_msgs::object_access;

std::uniform_real_distribution<double> uniform_distribution(-1, 1);
std::default_random_engine random_engine;

double randomValue() { return uniform_distribution(random_engine); }

TEST(perception_msgs, test_init)
{
  Object obj;
  initializeState(obj, EGO::MODEL_ID);

  EXPECT_EQ(getContinuousStateSize(obj), getContinuousStateSize((unsigned char)EGO::MODEL_ID));
  EXPECT_EQ(getDiscreteStateSize(obj), getDiscreteStateSize((unsigned char)EGO::MODEL_ID));
  EXPECT_EQ(
    getContinuousStateCovarianceSize(obj),
    getContinuousStateCovarianceSize((unsigned char)EGO::MODEL_ID));

  int n = getContinuousStateSize(obj);
  int m = getDiscreteStateSize(obj);

  std::vector<double> continuous_state = getContinuousState(obj);
  for (int i = 0; i < n; i++) EXPECT_DOUBLE_EQ(continuous_state[i], 0.0) << "i=" << i;

  std::vector<long int> discrete_state = getDiscreteState(obj);
  for (int i = 0; i < m; i++) EXPECT_EQ(discrete_state[i], 0) << "i=" << i;

  std::vector<double> continuous_state_covariance = getContinuousStateCovariance(obj);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j)
        EXPECT_DOUBLE_EQ(continuous_state_covariance[n * i + j], -1.0) << "i=" << i << ", j=" << j;
      else
        EXPECT_DOUBLE_EQ(continuous_state_covariance[n * i + j], 0.0) << "i=" << i << ", j=" << j;
    }
  }
}

TEST(perception_msgs, test_set_get_EGO)
{
  Object obj;
  initializeState(obj, EGO::MODEL_ID);

  double val;

  // set/getX
  val = randomValue();
  setX(obj, val);
  EXPECT_DOUBLE_EQ(val, getX(obj));

  // set/getY
  val = randomValue();
  setY(obj, val);
  EXPECT_DOUBLE_EQ(val, getY(obj));

  // set/getZ
  val = randomValue();
  setZ(obj, val);
  EXPECT_DOUBLE_EQ(val, getZ(obj));

  // set/getVelLon
  val = randomValue();
  setVelLon(obj, val);
  EXPECT_DOUBLE_EQ(val, getVelLon(obj));

  // set/getVelLat
  val = randomValue();
  setVelLat(obj, val);
  EXPECT_DOUBLE_EQ(val, getVelLat(obj));

  // set/getAccLon
  val = randomValue();
  setAccLon(obj, val);
  EXPECT_DOUBLE_EQ(val, getAccLon(obj));

  // set/getAccLat
  val = randomValue();
  setAccLat(obj, val);
  EXPECT_DOUBLE_EQ(val, getAccLat(obj));

  // set/getRoll
  val = randomValue();
  setRoll(obj, val);
  EXPECT_DOUBLE_EQ(val, getRoll(obj));

  // set/getPitch
  val = randomValue();
  setPitch(obj, val);
  EXPECT_DOUBLE_EQ(val, getPitch(obj));

  // set/getYaw
  val = randomValue();
  setYaw(obj, val);
  EXPECT_DOUBLE_EQ(val, getYaw(obj));

  // set/getYawRate
  val = randomValue();
  setYawRate(obj, val);
  EXPECT_DOUBLE_EQ(val, getYawRate(obj));

  // set/getSteeringAngleAck
  val = randomValue();
  setSteeringAngleAck(obj, val);
  EXPECT_DOUBLE_EQ(val, getSteeringAngleAck(obj));

  // set/getSteeringAngleRateAck
  val = randomValue();
  setSteeringAngleRateAck(obj, val);
  EXPECT_DOUBLE_EQ(val, getSteeringAngleRateAck(obj));

  // set/getStandstill
  setStandstill(obj, true);
  EXPECT_EQ(true, getStandstill(obj));

  // unknown covariance after setting state values
  std::vector<double> continuous_state_covariance = getContinuousStateCovariance(obj);
  int n = getContinuousStateSize(obj);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j)
        EXPECT_DOUBLE_EQ(continuous_state_covariance[n * i + j], std::numeric_limits<double>::max())
          << "i=" << i << ", j=" << j;
      else
        EXPECT_DOUBLE_EQ(continuous_state_covariance[n * i + j], 0.0) << "i=" << i << ", j=" << j;
    }
  }
}

TEST(perception_msgs, test_set_get_EGORWS)
{
  Object obj;
  initializeState(obj, EGORWS::MODEL_ID);

  double val;

  // set/getX
  val = randomValue();
  setX(obj, val);
  EXPECT_DOUBLE_EQ(val, getX(obj));

  // set/getY
  val = randomValue();
  setY(obj, val);
  EXPECT_DOUBLE_EQ(val, getY(obj));

  // set/getZ
  val = randomValue();
  setZ(obj, val);
  EXPECT_DOUBLE_EQ(val, getZ(obj));

  // set/getVelLon
  val = randomValue();
  setVelLon(obj, val);
  EXPECT_DOUBLE_EQ(val, getVelLon(obj));

  // set/getVelLat
  val = randomValue();
  setVelLat(obj, val);
  EXPECT_DOUBLE_EQ(val, getVelLat(obj));

  // set/getAccLon
  val = randomValue();
  setAccLon(obj, val);
  EXPECT_DOUBLE_EQ(val, getAccLon(obj));

  // set/getAccLat
  val = randomValue();
  setAccLat(obj, val);
  EXPECT_DOUBLE_EQ(val, getAccLat(obj));

  // set/getRoll
  val = randomValue();
  setRoll(obj, val);
  EXPECT_DOUBLE_EQ(val, getRoll(obj));

  // set/getPitch
  val = randomValue();
  setPitch(obj, val);
  EXPECT_DOUBLE_EQ(val, getPitch(obj));

  // set/getYaw
  val = randomValue();
  setYaw(obj, val);
  EXPECT_DOUBLE_EQ(val, getYaw(obj));

  // set/getYawRate
  val = randomValue();
  setYawRate(obj, val);
  EXPECT_DOUBLE_EQ(val, getYawRate(obj));

  // set/getSteeringAngleFront
  val = randomValue();
  setSteeringAngleFront(obj, val);
  EXPECT_DOUBLE_EQ(val, getSteeringAngleFront(obj));

  // set/getSteeringAngleRear
  val = randomValue();
  setSteeringAngleRear(obj, val);
  EXPECT_DOUBLE_EQ(val, getSteeringAngleRear(obj));

  // set/getStandstill
  setStandstill(obj, true);
  EXPECT_EQ(true, getStandstill(obj));

  // unknown covariance after setting state values
  std::vector<double> continuous_state_covariance = getContinuousStateCovariance(obj);
  int n = getContinuousStateSize(obj);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j)
        EXPECT_DOUBLE_EQ(continuous_state_covariance[n * i + j], std::numeric_limits<double>::max())
          << "i=" << i << ", j=" << j;
      else
        EXPECT_DOUBLE_EQ(continuous_state_covariance[n * i + j], 0.0) << "i=" << i << ", j=" << j;
    }
  }
}

TEST(perception_msgs, test_set_get_ISCACTR)
{
  Object obj;
  initializeState(obj, ISCACTR::MODEL_ID);

  double val;

  // set/getX
  val = randomValue();
  setX(obj, val);
  EXPECT_DOUBLE_EQ(val, getX(obj));

  // set/getY
  val = randomValue();
  setY(obj, val);
  EXPECT_DOUBLE_EQ(val, getY(obj));

  // set/getZ
  val = randomValue();
  setZ(obj, val);
  EXPECT_DOUBLE_EQ(val, getZ(obj));

  // set/getVelLon
  val = randomValue();
  setVelLon(obj, val);
  EXPECT_DOUBLE_EQ(val, getVelLon(obj));

  // set/getVelLat
  val = randomValue();
  setVelLat(obj, val);
  EXPECT_DOUBLE_EQ(val, getVelLat(obj));

  // set/getAccLon
  val = randomValue();
  setAccLon(obj, val);
  EXPECT_DOUBLE_EQ(val, getAccLon(obj));

  // set/getAccLat
  val = randomValue();
  setAccLat(obj, val);
  EXPECT_DOUBLE_EQ(val, getAccLat(obj));

  // set/getYaw
  val = randomValue();
  setYaw(obj, val);
  EXPECT_DOUBLE_EQ(val, getYaw(obj));

  // set/getYawRate
  val = randomValue();
  setYawRate(obj, val);
  EXPECT_DOUBLE_EQ(val, getYawRate(obj));

  // set/getWidth
  val = randomValue();
  setWidth(obj, val);
  EXPECT_DOUBLE_EQ(val, getWidth(obj));

  // set/getLength
  val = randomValue();
  setLength(obj, val);
  EXPECT_DOUBLE_EQ(val, getLength(obj));

  // set/getHeight
  val = randomValue();
  setHeight(obj, val);
  EXPECT_DOUBLE_EQ(val, getHeight(obj));
}

TEST(perception_msgs, test_set_get_HEXAMOTION)
{
  Object obj;
  initializeState(obj, HEXAMOTION::MODEL_ID);

  double val;

  // set/getX
  val = randomValue();
  setX(obj, val);
  EXPECT_DOUBLE_EQ(val, getX(obj));

  // set/getY
  val = randomValue();
  setY(obj, val);
  EXPECT_DOUBLE_EQ(val, getY(obj));

  // set/getZ
  val = randomValue();
  setZ(obj, val);
  EXPECT_DOUBLE_EQ(val, getZ(obj));

  // set/getVelLon
  val = randomValue();
  setVelLon(obj, val);
  EXPECT_DOUBLE_EQ(val, getVelLon(obj));

  // set/getVelLat
  val = randomValue();
  setVelLat(obj, val);
  EXPECT_DOUBLE_EQ(val, getVelLat(obj));

  // set/getAccLon
  val = randomValue();
  setAccLon(obj, val);
  EXPECT_DOUBLE_EQ(val, getAccLon(obj));

  // set/getAccLat
  val = randomValue();
  setAccLat(obj, val);
  EXPECT_DOUBLE_EQ(val, getAccLat(obj));

  // set/getRoll
  val = randomValue();
  setRoll(obj, val);
  EXPECT_DOUBLE_EQ(val, getRoll(obj));

  // set/getRollRate
  val = randomValue();
  setRollRate(obj, val);
  EXPECT_DOUBLE_EQ(val, getRollRate(obj));

  // set/getPitch
  val = randomValue();
  setPitch(obj, val);
  EXPECT_DOUBLE_EQ(val, getPitch(obj));

  // set/getPitchRate
  val = randomValue();
  setPitchRate(obj, val);
  EXPECT_DOUBLE_EQ(val, getPitchRate(obj));

  // set/getYaw
  val = randomValue();
  setYaw(obj, val);
  EXPECT_DOUBLE_EQ(val, getYaw(obj));

  // set/getYawRate
  val = randomValue();
  setYawRate(obj, val);
  EXPECT_DOUBLE_EQ(val, getYawRate(obj));

  // set/getLength
  val = randomValue();
  setLength(obj, val);

  EXPECT_DOUBLE_EQ(val, getLength(obj));
  // set/getWidth
  val = randomValue();
  setWidth(obj, val);
  EXPECT_DOUBLE_EQ(val, getWidth(obj));

  // set/getHeight
  val = randomValue();
  setHeight(obj, val);
  EXPECT_DOUBLE_EQ(val, getHeight(obj));
}

TEST(perception_msgs, test_set_get_TRAFFICLIGHT)
{
  Object obj;
  initializeState(obj, TRAFFICLIGHT::MODEL_ID);

  double val;

  // set/getX
  val = randomValue();
  setX(obj, val);
  EXPECT_DOUBLE_EQ(val, getX(obj));

  // set/getY
  val = randomValue();
  setY(obj, val);
  EXPECT_DOUBLE_EQ(val, getY(obj));

  // set/getZ
  val = randomValue();
  setZ(obj, val);
  EXPECT_DOUBLE_EQ(val, getZ(obj));

  // set/getTrafficLightState
  setTrafficLightState(obj, TRAFFICLIGHT::STATE_RED);
  EXPECT_EQ(TRAFFICLIGHT::STATE_RED, getTrafficLightState(obj));

  // set/getTrafficLightType
  setTrafficLightType(obj, TRAFFICLIGHT::TYPE_STRAIGHT);
  EXPECT_EQ(TRAFFICLIGHT::TYPE_STRAIGHT, getTrafficLightType(obj));
}

TEST(perception_msgs, test_convenience_set_get)
{
  Object obj;

  double val;

  // set/getContinuousState
  initializeState(obj, EGO::MODEL_ID);
  std::vector<double> state1, state2;
  for (int i = 0; i < getContinuousStateSize(obj.state.model_id); i++) state1.push_back(i);
  setContinuousState(obj, state1);
  state2 = getContinuousState(obj);
  for (int i = 0; i < getContinuousStateSize(obj.state.model_id); i++)
    EXPECT_DOUBLE_EQ(state1[i], state2[i]) << "i=" << i;

  // set/getDiscreteState
  initializeState(obj, EGO::MODEL_ID);
  std::vector<long int> discrete_state1, discrete_state2;
  for (int i = 0; i < getDiscreteStateSize(obj.state.model_id); i++) discrete_state1.push_back(i);
  setDiscreteState(obj, discrete_state1);
  discrete_state2 = getDiscreteState(obj);
  for (int i = 0; i < getDiscreteStateSize(obj.state.model_id); i++)
    EXPECT_EQ(discrete_state1[i], discrete_state2[i]) << "i=" << i;

  // set/getCovariance
  initializeState(obj, EGO::MODEL_ID);
  std::vector<double> state_cov1, state_cov2;
  for (int i = 0; i < getContinuousStateCovarianceSize(obj.state.model_id); i++)
    state_cov1.push_back(i);
  setContinuousStateCovariance(obj, state_cov1);
  state_cov2 = getContinuousStateCovariance(obj);
  for (int i = 0; i < getContinuousStateCovarianceSize(obj.state.model_id); i++)
    EXPECT_DOUBLE_EQ(state_cov1[i], state_cov2[i]) << "i=" << i;

  // set/getPoseWithCovariance
  //   set/getPose
  //     set/getPosition
  //     set/getOrientation
  //   set/getPoseCovariance
  //     set/getContinuousStateCovarianceAt
  initializeState(obj, EGO::MODEL_ID);
  gm::PoseWithCovariance p1, p2;
  p1.pose.position.x = 1.0;
  p1.pose.position.y = 1.0;
  p1.pose.position.z = 1.0;
  p1.pose.orientation.x = 0.5;
  p1.pose.orientation.y = 0.5;
  p1.pose.orientation.z = 0.5;
  p1.pose.orientation.w = 0.5;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) p1.covariance[6 * i + j] = 6 * i + j;
  }
  setPoseWithCovariance(obj, p1);
  p2 = getPoseWithCovariance(obj);
  EXPECT_DOUBLE_EQ(p1.pose.position.x, p2.pose.position.x);
  EXPECT_DOUBLE_EQ(p1.pose.position.y, p2.pose.position.y);
  EXPECT_DOUBLE_EQ(p1.pose.position.z, p2.pose.position.z);
  EXPECT_DOUBLE_EQ(p1.pose.orientation.x, p2.pose.orientation.x);
  EXPECT_DOUBLE_EQ(p1.pose.orientation.y, p2.pose.orientation.y);
  EXPECT_DOUBLE_EQ(p1.pose.orientation.z, p2.pose.orientation.z);
  EXPECT_DOUBLE_EQ(p1.pose.orientation.w, p2.pose.orientation.w);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++)
      EXPECT_DOUBLE_EQ(p1.covariance[6 * i + j], p2.covariance[6 * i + j])
        << "i=" << i << ", j=" << j;
  }

  // set/getVelocity
  initializeState(obj, EGO::MODEL_ID);
  gm::Vector3 vel1, vel2;
  vel1.x = 3.0;
  vel1.y = 4.0;
  vel1.z = 5.0;
  setVelocity(obj, vel1);
  vel2 = getVelocity(obj);
  EXPECT_DOUBLE_EQ(vel1.x, vel2.x);
  EXPECT_DOUBLE_EQ(vel1.y, vel2.y);
  EXPECT_DOUBLE_EQ(0.0, vel2.z);
  EXPECT_DOUBLE_EQ(std::sqrt(std::pow(vel1.x, 2) + std::pow(vel1.y, 2)), getVelocityMagnitude(obj));

  // set/getAcceleration
  initializeState(obj, EGO::MODEL_ID);
  gm::Vector3 acc1, acc2;
  acc1.x = 3.0;
  acc1.y = 4.0;
  acc1.z = 5.0;
  setAcceleration(obj, acc1);
  acc2 = getAcceleration(obj);
  EXPECT_DOUBLE_EQ(acc1.x, acc2.x);
  EXPECT_DOUBLE_EQ(acc1.y, acc2.y);
  EXPECT_DOUBLE_EQ(0.0, acc2.z);
  EXPECT_DOUBLE_EQ(
    std::sqrt(std::pow(acc1.x, 2) + std::pow(acc1.y, 2)), getAccelerationMagnitude(obj));

  // set/getRoll/Pitch/YawInDeg
  initializeState(obj, EGO::MODEL_ID);
  val = randomValue();
  setRollInDeg(obj, val);
  EXPECT_DOUBLE_EQ(val, getRollInDeg(obj));
  val = randomValue();
  setPitchInDeg(obj, val);
  EXPECT_DOUBLE_EQ(val, getPitchInDeg(obj));
  val = randomValue();
  setYawInDeg(obj, val);
  EXPECT_DOUBLE_EQ(val, getYawInDeg(obj));

  // set/getVelocityXYZ
  initializeState(obj, EGO::MODEL_ID);
  gm::Vector3 vel1_lon_lat, vel2_xyz, vel3_xyz;
  vel1_lon_lat.x = 3.0;
  vel1_lon_lat.y = 4.0;
  vel1_lon_lat.z = 5.0;
  setVelocity(obj, vel1_lon_lat);
  setYaw(obj, M_PI_2);
  vel2_xyz = getVelocityXYZ(obj);
  EXPECT_DOUBLE_EQ(-vel1_lon_lat.y, vel2_xyz.x);
  EXPECT_DOUBLE_EQ(vel1_lon_lat.x, vel2_xyz.y);
  EXPECT_DOUBLE_EQ(0.0, vel2_xyz.z);
  setVelocityXYZYaw(obj, vel2_xyz, M_PI_2);
  vel3_xyz = getVelocityXYZ(obj);
  EXPECT_DOUBLE_EQ(vel2_xyz.x, vel3_xyz.x);
  EXPECT_DOUBLE_EQ(vel2_xyz.y, vel3_xyz.y);
  EXPECT_DOUBLE_EQ(vel2_xyz.z, vel3_xyz.z);

  // set/getVelocityXYZWithCovariance
  initializeState(obj, ISCACTR::MODEL_ID);
  vel1_lon_lat.x = 3.0;
  vel1_lon_lat.y = 4.0;
  vel1_lon_lat.z = 5.0;
  setVelocity(obj, vel1_lon_lat);
  double var_lon = std::abs(randomValue()) + 0.5;
  double var_lat = std::abs(randomValue()) + 0.1;
  double cov_lat_lon = randomValue() * std::max(var_lon, var_lat);
  setContinuousStateCovarianceAt(
    obj, indexVelLon(obj.state.model_id), indexVelLon(obj.state.model_id), var_lon);
  setContinuousStateCovarianceAt(
    obj, indexVelLon(obj.state.model_id), indexVelLat(obj.state.model_id), cov_lat_lon);
  setContinuousStateCovarianceAt(
    obj, indexVelLat(obj.state.model_id), indexVelLon(obj.state.model_id), cov_lat_lon);
  setContinuousStateCovarianceAt(
    obj, indexVelLat(obj.state.model_id), indexVelLat(obj.state.model_id), var_lat);
  setYaw(obj, M_PI_2);
  auto vel2_xyz_with_cov = getVelocityXYZWithCovariance(obj);
  EXPECT_NEAR(vel1_lon_lat.x, vel2_xyz_with_cov.pose.position.y, 1.e-10);
  EXPECT_NEAR(-vel1_lon_lat.y, vel2_xyz_with_cov.pose.position.x, 1.e-10);
  EXPECT_NEAR(0.0, vel2_xyz_with_cov.pose.position.z, 1.e-10);
  EXPECT_NEAR(var_lon, vel2_xyz_with_cov.covariance.at(7), 1.e-10);
  EXPECT_NEAR(var_lat, vel2_xyz_with_cov.covariance.at(0), 1.e-10);
  EXPECT_NEAR(-cov_lat_lon, vel2_xyz_with_cov.covariance.at(1), 1.e-10);
  EXPECT_NEAR(-cov_lat_lon, vel2_xyz_with_cov.covariance.at(6), 1.e-10);
  EXPECT_NEAR(0.0, vel2_xyz_with_cov.covariance.at(2), 1.e-10);
  EXPECT_NEAR(0.0, vel2_xyz_with_cov.covariance.at(8), 1.e-10);
  EXPECT_NEAR(0.0, vel2_xyz_with_cov.covariance.at(12), 1.e-10);
  EXPECT_NEAR(0.0, vel2_xyz_with_cov.covariance.at(13), 1.e-10);
  EXPECT_NEAR(0.0, vel2_xyz_with_cov.covariance.at(14), 1.e-10);
  gm::Vector3 vel_xyz;
  vel_xyz.x = vel2_xyz_with_cov.pose.position.x;
  vel_xyz.y = vel2_xyz_with_cov.pose.position.y;
  vel_xyz.z = vel2_xyz_with_cov.pose.position.z;
  setVelocityXYZYawWithCovariance(
    obj, vel_xyz, M_PI_2, vel2_xyz_with_cov.covariance.at(0), vel2_xyz_with_cov.covariance.at(7),
    vel2_xyz_with_cov.covariance.at(1));
  auto vel3_xyz_with_cov = getVelocityXYZWithCovariance(obj);
  EXPECT_NEAR(vel2_xyz_with_cov.pose.position.x, vel3_xyz_with_cov.pose.position.x, 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.pose.position.y, vel3_xyz_with_cov.pose.position.y, 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.pose.position.z, vel3_xyz_with_cov.pose.position.z, 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(0), vel3_xyz_with_cov.covariance.at(0), 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(1), vel3_xyz_with_cov.covariance.at(1), 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(2), vel3_xyz_with_cov.covariance.at(2), 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(6), vel3_xyz_with_cov.covariance.at(6), 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(7), vel3_xyz_with_cov.covariance.at(7), 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(8), vel3_xyz_with_cov.covariance.at(8), 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(12), vel3_xyz_with_cov.covariance.at(12), 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(13), vel3_xyz_with_cov.covariance.at(13), 1.e-10);
  EXPECT_NEAR(vel2_xyz_with_cov.covariance.at(14), vel3_xyz_with_cov.covariance.at(14), 1.e-10);

  // set/getAccelerationXYZ
  initializeState(obj, EGO::MODEL_ID);
  gm::Vector3 acc1_lon_lat, acc2_xyz, acc3_xyz;
  acc1_lon_lat.x = 3.0;
  acc1_lon_lat.y = 4.0;
  acc1_lon_lat.z = 5.0;
  setAcceleration(obj, acc1_lon_lat);
  setYaw(obj, M_PI_2);
  acc2_xyz = getAccelerationXYZ(obj);
  EXPECT_DOUBLE_EQ(-acc1_lon_lat.y, acc2_xyz.x);
  EXPECT_DOUBLE_EQ(acc1_lon_lat.x, acc2_xyz.y);
  EXPECT_DOUBLE_EQ(0.0, acc2_xyz.z);
  setAccelerationXYZYaw(obj, acc2_xyz, M_PI_2);
  acc3_xyz = getAccelerationXYZ(obj);
  EXPECT_DOUBLE_EQ(acc2_xyz.x, acc3_xyz.x);
  EXPECT_DOUBLE_EQ(acc2_xyz.y, acc3_xyz.y);
  EXPECT_DOUBLE_EQ(acc2_xyz.z, acc3_xyz.z);

  // set/getAccelerationXYZWithCovariance
  initializeState(obj, ISCACTR::MODEL_ID);
  acc1_lon_lat.x = 3.0;
  acc1_lon_lat.y = 4.0;
  acc1_lon_lat.z = 5.0;
  setAcceleration(obj, acc1_lon_lat);
  var_lon = std::abs(randomValue()) + 0.5;
  var_lat = std::abs(randomValue()) + 0.1;
  cov_lat_lon = randomValue() * std::max(var_lon, var_lat);
  setContinuousStateCovarianceAt(
    obj, indexAccLon(obj.state.model_id), indexAccLon(obj.state.model_id), var_lon);
  setContinuousStateCovarianceAt(
    obj, indexAccLon(obj.state.model_id), indexAccLat(obj.state.model_id), cov_lat_lon);
  setContinuousStateCovarianceAt(
    obj, indexAccLat(obj.state.model_id), indexAccLon(obj.state.model_id), cov_lat_lon);
  setContinuousStateCovarianceAt(
    obj, indexAccLat(obj.state.model_id), indexAccLat(obj.state.model_id), var_lat);
  setYaw(obj, M_PI_2);
  auto acc2_xyz_with_cov = getAccelerationXYZWithCovariance(obj);
  EXPECT_NEAR(acc1_lon_lat.x, acc2_xyz_with_cov.pose.position.y, 1.e-10);
  EXPECT_NEAR(-acc1_lon_lat.y, acc2_xyz_with_cov.pose.position.x, 1.e-10);
  EXPECT_NEAR(0.0, acc2_xyz_with_cov.pose.position.z, 1.e-10);
  EXPECT_NEAR(var_lon, acc2_xyz_with_cov.covariance.at(7), 1.e-10);
  EXPECT_NEAR(var_lat, acc2_xyz_with_cov.covariance.at(0), 1.e-10);
  EXPECT_NEAR(-cov_lat_lon, acc2_xyz_with_cov.covariance.at(1), 1.e-10);
  EXPECT_NEAR(-cov_lat_lon, acc2_xyz_with_cov.covariance.at(6), 1.e-10);
  EXPECT_NEAR(0.0, acc2_xyz_with_cov.covariance.at(2), 1.e-10);
  EXPECT_NEAR(0.0, acc2_xyz_with_cov.covariance.at(8), 1.e-10);
  EXPECT_NEAR(0.0, acc2_xyz_with_cov.covariance.at(12), 1.e-10);
  EXPECT_NEAR(0.0, acc2_xyz_with_cov.covariance.at(13), 1.e-10);
  EXPECT_NEAR(0.0, acc2_xyz_with_cov.covariance.at(14), 1.e-10);
  gm::Vector3 acc_xyz;
  acc_xyz.x = acc2_xyz_with_cov.pose.position.x;
  acc_xyz.y = acc2_xyz_with_cov.pose.position.y;
  acc_xyz.z = acc2_xyz_with_cov.pose.position.z;
  setAccelerationXYZYawWithCovariance(
    obj, acc_xyz, M_PI_2, acc2_xyz_with_cov.covariance.at(0), acc2_xyz_with_cov.covariance.at(7),
    acc2_xyz_with_cov.covariance.at(1));
  auto acc3_xyz_with_cov = getAccelerationXYZWithCovariance(obj);
  EXPECT_NEAR(acc2_xyz_with_cov.pose.position.x, acc3_xyz_with_cov.pose.position.x, 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.pose.position.y, acc3_xyz_with_cov.pose.position.y, 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.pose.position.z, acc3_xyz_with_cov.pose.position.z, 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(0), acc3_xyz_with_cov.covariance.at(0), 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(1), acc3_xyz_with_cov.covariance.at(1), 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(2), acc3_xyz_with_cov.covariance.at(2), 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(6), acc3_xyz_with_cov.covariance.at(6), 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(7), acc3_xyz_with_cov.covariance.at(7), 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(8), acc3_xyz_with_cov.covariance.at(8), 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(12), acc3_xyz_with_cov.covariance.at(12), 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(13), acc3_xyz_with_cov.covariance.at(13), 1.e-10);
  EXPECT_NEAR(acc2_xyz_with_cov.covariance.at(14), acc3_xyz_with_cov.covariance.at(14), 1.e-10);
}

TEST(perception_msgs, test_getClassWithHighestProbability)
{
  Object obj;
  ObjectClassification dummy_class;
  ObjectClassification max_class;

  dummy_class.type = ObjectClassification::MOTORBIKE;
  dummy_class.probability = 0.2;
  obj.state.classifications.push_back(dummy_class);
  dummy_class.type = ObjectClassification::VAN;
  dummy_class.probability = 0.3;
  obj.state.classifications.push_back(dummy_class);
  dummy_class.type = ObjectClassification::CAR;
  dummy_class.probability = 0.4;
  max_class = dummy_class;
  obj.state.classifications.push_back(dummy_class);
  dummy_class.type = ObjectClassification::PEDESTRIAN;
  dummy_class.probability = 0.1;
  obj.state.classifications.push_back(dummy_class);

  ObjectClassification output_class = object_access::getClassWithHighestProbability(obj);

  EXPECT_DOUBLE_EQ(max_class.probability, output_class.probability);
  EXPECT_EQ(max_class.type, output_class.type);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
