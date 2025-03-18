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

#include <cmath>

#include <gtest/gtest.h>


using namespace perception_msgs;
using namespace perception_msgs::object_access;

static const double EPS = 1e-12;


TEST(tf2_perception_msgs, test_doTransform_Object_ISCACTR) {

  Object obj, obj_tf;
  initializeState(obj, ISCACTR::MODEL_ID);
  const int n = getContinuousStateSize(obj.state.model_id);
  setPosition(obj, {1.0, 2.0, 3.0});
  setVelocity(obj, {1.0, 2.0});
  setAcceleration(obj, {3.0, 4.0});
  setYaw(obj, M_PI);
  setYawRate(obj, -1.0);
  setWidth(obj, -1.0);
  setLength(obj, -1.0);
  setHeight(obj, -1.0);
  std::vector<double> covariance_diagonal(n);
  for (int i = 0; i < n; i++) covariance_diagonal[i] = i;
  setContinuousStateCovarianceDiagonal(obj, covariance_diagonal);

  gm::TransformStamped tf;
  tf.transform.translation.x = 10.0;
  tf.transform.translation.y = 20.0;
  tf.transform.translation.z = 30.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 1.0;
  tf.transform.rotation.w = 0.0;

  tf2::doTransform(obj, obj_tf, tf);

  // transformed state
  EXPECT_NEAR(getX(obj_tf), 9.0, EPS);
  EXPECT_NEAR(getY(obj_tf), 18.0, EPS);
  EXPECT_NEAR(getZ(obj_tf), 33.0, EPS);
  EXPECT_NEAR(getYaw(obj_tf), 0.0, EPS);
  EXPECT_NEAR(getVelX(obj_tf), 1.0, EPS);
  EXPECT_NEAR(getVelY(obj_tf), 2.0, EPS);
  EXPECT_NEAR(getAccX(obj_tf), 3.0, EPS);
  EXPECT_NEAR(getAccY(obj_tf), 4.0, EPS);

  // transform-invariant state
  EXPECT_NEAR(getVelLon(obj_tf), 1.0, EPS);
  EXPECT_NEAR(getVelLat(obj_tf), 2.0, EPS);
  EXPECT_NEAR(getAccLon(obj_tf), 3.0, EPS);
  EXPECT_NEAR(getAccLat(obj_tf), 4.0, EPS);
  EXPECT_NEAR(getYawRate(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getWidth(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getLength(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getHeight(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getHeight(obj_tf), -1.0, EPS);

  // transformed covariance
  std::vector<double> covariance_diagonal_tf = getContinuousStateCovarianceDiagonal(obj);
  for (int i = 0; i < n; i++)
    EXPECT_NEAR(covariance_diagonal[i], covariance_diagonal_tf[i], EPS) << "i=" << i;
}

TEST(tf2_perception_msgs, test_doTransform_Object_HEXAMOTION) {

  Object obj, obj_tf;
  initializeState(obj, HEXAMOTION::MODEL_ID);
  const int n = getContinuousStateSize(obj.state.model_id);
  setPosition(obj, {1.0, 2.0, 3.0});
  setVelocity(obj, {1.0, 2.0});
  setAcceleration(obj, {3.0, 4.0});
  setRoll(obj, M_PI);
  setPitch(obj, 0.0);
  setYaw(obj, 0.0);
  setRollRate(obj, -1.0);
  setPitchRate(obj, -1.0);
  setYawRate(obj, -1.0);
  setWidth(obj, -1.0);
  setLength(obj, -1.0);
  setHeight(obj, -1.0);
  std::vector<double> covariance_diagonal(n);
  for (int i = 0; i < n; i++) covariance_diagonal[i] = i;
  setContinuousStateCovarianceDiagonal(obj, covariance_diagonal);

  gm::TransformStamped tf;
  tf.transform.translation.x = 10.0;
  tf.transform.translation.y = 20.0;
  tf.transform.translation.z = 30.0;
  tf.transform.rotation.x = 1.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 0.0;

  tf2::doTransform(obj, obj_tf, tf);

  // transformed state
  EXPECT_NEAR(getX(obj_tf), 11.0, EPS);
  EXPECT_NEAR(getY(obj_tf), 18.0, EPS);
  EXPECT_NEAR(getZ(obj_tf), 27.0, EPS);
  EXPECT_NEAR(getRoll(obj_tf), 0.0, EPS);
  EXPECT_NEAR(getPitch(obj_tf), 0.0, EPS);
  EXPECT_NEAR(getYaw(obj_tf), 0.0, EPS);
  EXPECT_NEAR(getVelX(obj_tf), 1.0, EPS);
  EXPECT_NEAR(getVelY(obj_tf), 2.0, EPS);
  EXPECT_NEAR(getAccX(obj_tf), 3.0, EPS);
  EXPECT_NEAR(getAccY(obj_tf), 4.0, EPS);

  // transform-invariant state
  EXPECT_NEAR(getVelLon(obj_tf), 1.0, EPS);
  EXPECT_NEAR(getVelLat(obj_tf), 2.0, EPS);
  EXPECT_NEAR(getAccLon(obj_tf), 3.0, EPS);
  EXPECT_NEAR(getAccLat(obj_tf), 4.0, EPS);
  EXPECT_NEAR(getRollRate(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getPitchRate(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getYawRate(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getWidth(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getLength(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getHeight(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getHeight(obj_tf), -1.0, EPS);

  // transformed covariance
  std::vector<double> covariance_diagonal_tf = getContinuousStateCovarianceDiagonal(obj);
  for (int i = 0; i < n; i++)
    EXPECT_NEAR(covariance_diagonal[i], covariance_diagonal_tf[i], EPS) << "i=" << i;
}

TEST(tf2_perception_msgs, test_doTransform_Object_EGO) {

  Object obj, obj_tf;
  initializeState(obj, EGO::MODEL_ID);
  const int n = getContinuousStateSize(obj.state.model_id);
  setPose(obj, {1.0, 2.0, 3.0}, {M_PI_4, M_PI_4, M_PI_2});
  setVelocity(obj, {1.0, 2.0});
  setAcceleration(obj, {3.0, 4.0});
  setYawRate(obj, -1.0);
  setSteeringAngleAck(obj, -1.0);
  setSteeringAngleRateAck(obj, -1.0);
  std::vector<double> covariance_diagonal(n);
  for (int i = 0; i < n; i++) covariance_diagonal[i] = i;
  setContinuousStateCovarianceDiagonal(obj, covariance_diagonal);

  gm::TransformStamped tf;
  tf.transform.translation.x = 10.0;
  tf.transform.translation.y = 20.0;
  tf.transform.translation.z = 30.0;
  tf.transform.rotation.x = 1.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 0.0;

  tf2::doTransform(obj, obj_tf, tf);

  // transformed state
  EXPECT_NEAR(getX(obj_tf), 11.0, EPS);
  EXPECT_NEAR(getY(obj_tf), 18.0, EPS);
  EXPECT_NEAR(getZ(obj_tf), 27.0, EPS);
  EXPECT_NEAR(getRoll(obj_tf), -3 * M_PI_4, EPS);
  EXPECT_NEAR(getPitch(obj_tf), -M_PI_4, EPS);
  EXPECT_NEAR(getYaw(obj_tf), -M_PI_2, EPS);

  // transform-invariant state
  EXPECT_NEAR(getVelLon(obj_tf), 1.0, EPS);
  EXPECT_NEAR(getVelLat(obj_tf), 2.0, EPS);
  EXPECT_NEAR(getAccLon(obj_tf), 3.0, EPS);
  EXPECT_NEAR(getAccLat(obj_tf), 4.0, EPS);
  EXPECT_NEAR(getYawRate(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getSteeringAngleAck(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getSteeringAngleRateAck(obj_tf), -1.0, EPS);

  // transformed covariance
  std::vector<double> covariance_diagonal_tf = getContinuousStateCovarianceDiagonal(obj);
  for (int i = 0; i < n; i++)
    EXPECT_NEAR(covariance_diagonal[i], covariance_diagonal_tf[i], EPS) << "i=" << i;
}

TEST(tf2_perception_msgs, test_doTransform_Object_EGORWS) {

  Object obj, obj_tf;
  initializeState(obj, EGORWS::MODEL_ID);
  const int n = getContinuousStateSize(obj.state.model_id);
  setPose(obj, {1.0, 2.0, 3.0}, {M_PI_4, M_PI_4, M_PI_2});
  setVelocity(obj, {1.0, 2.0});
  setAcceleration(obj, {3.0, 4.0});
  setYawRate(obj, -1.0);
  setSteeringAngleFront(obj, -1.0);
  setSteeringAngleRear(obj, -1.0);
  std::vector<double> covariance_diagonal(n);
  for (int i = 0; i < n; i++) covariance_diagonal[i] = i;
  setContinuousStateCovarianceDiagonal(obj, covariance_diagonal);

  gm::TransformStamped tf;
  tf.transform.translation.x = 10.0;
  tf.transform.translation.y = 20.0;
  tf.transform.translation.z = 30.0;
  tf.transform.rotation.x = 1.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 0.0;

  tf2::doTransform(obj, obj_tf, tf);

  // transformed state
  EXPECT_NEAR(getX(obj_tf), 11.0, EPS);
  EXPECT_NEAR(getY(obj_tf), 18.0, EPS);
  EXPECT_NEAR(getZ(obj_tf), 27.0, EPS);
  EXPECT_NEAR(getRoll(obj_tf), -3 * M_PI_4, EPS);
  EXPECT_NEAR(getPitch(obj_tf), -M_PI_4, EPS);
  EXPECT_NEAR(getYaw(obj_tf), -M_PI_2, EPS);

  // transform-invariant state
  EXPECT_NEAR(getVelLon(obj_tf), 1.0, EPS);
  EXPECT_NEAR(getVelLat(obj_tf), 2.0, EPS);
  EXPECT_NEAR(getAccLon(obj_tf), 3.0, EPS);
  EXPECT_NEAR(getAccLat(obj_tf), 4.0, EPS);
  EXPECT_NEAR(getYawRate(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getSteeringAngleFront(obj_tf), -1.0, EPS);
  EXPECT_NEAR(getSteeringAngleRear(obj_tf), -1.0, EPS);

  // transformed covariance
  std::vector<double> covariance_diagonal_tf = getContinuousStateCovarianceDiagonal(obj);
  for (int i = 0; i < n; i++)
    EXPECT_NEAR(covariance_diagonal[i], covariance_diagonal_tf[i], EPS) << "i=" << i;
}


int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
