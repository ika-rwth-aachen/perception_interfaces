// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

namespace gm = geometry_msgs::msg;
using namespace perception_msgs::msg;

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

TEST(tf2_perception_msgs, transforms_prediction_without_covariance) {
  Object object, transformed;
  initializeState(object, ISCACTR::MODEL_ID);
  setContinuousStateCovarianceDiagonal(object, std::vector<double>(getContinuousStateSize(object), 0.1));
  setPosition(object, {1.0, 2.0, 3.0});
  object.state_predictions.emplace_back();
  object.state_predictions.back().states.push_back(object.state);
  object.state_predictions.back().states.back().continuous_state_covariance.clear();

  gm::TransformStamped transform;
  transform.transform.translation.x = 10.0;
  transform.transform.rotation.w = 1.0;
  tf2::doTransform(object, transformed, transform);

  EXPECT_NEAR(getX(transformed.state), 11.0, EPS);
  EXPECT_FALSE(transformed.state.continuous_state_covariance.empty());
  ASSERT_EQ(transformed.state_predictions.size(), 1u);
  ASSERT_EQ(transformed.state_predictions.front().states.size(), 1u);
  const auto& predicted = transformed.state_predictions.front().states.front();
  EXPECT_NEAR(getX(predicted), 11.0, EPS);
  EXPECT_TRUE(predicted.continuous_state_covariance.empty());
}

TEST(tf2_perception_msgs, rotates_covariance_markers_without_arithmetic) {
  for (const auto model : {EGO::MODEL_ID, EGORWS::MODEL_ID, ISCACTR::MODEL_ID, HEXAMOTION::MODEL_ID}) {
    for (const double marker : {CONTINUOUS_STATE_COVARIANCE_INVALID, CONTINUOUS_STATE_COVARIANCE_UNKNOWN}) {
      for (const double angle : {0.0, M_PI_2, M_PI_4}) {
        ObjectState state, transformed;
        initializeState(state, model);
        setPose(state, {1.0, 2.0, 3.0}, {0.0, 0.0, 0.0});
        setContinuousStateCovarianceDiagonal(state, std::vector<double>(getContinuousStateSize(state), 4.0));
        setContinuousStateCovarianceAt(state, indexX(model), indexX(model), marker);
        const auto original_covariance = state.continuous_state_covariance;
        gm::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.transform.translation.x = 10.0;
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, angle);
        transform.transform.rotation = tf2::toMsg(rotation);
        tf2::doTransform(state, transformed, transform);
        const auto covariance = getPoseCovariance(transformed);
        EXPECT_EQ(covariance[0], angle == M_PI_2 ? 4.0 : marker);
        EXPECT_EQ(covariance[7], angle == 0.0 ? 4.0 : marker);
        EXPECT_EQ(covariance[14], 4.0);
        if (angle == M_PI_4) EXPECT_EQ(covariance[1], CONTINUOUS_STATE_COVARIANCE_UNKNOWN);
        else EXPECT_NEAR(covariance[1], 0.0, EPS);
        EXPECT_EQ(covariance[6], covariance[1]);
        for (const double value : transformed.continuous_state_covariance) EXPECT_TRUE(std::isfinite(value));
        EXPECT_EQ(state.continuous_state_covariance, original_covariance);
        EXPECT_NEAR(getX(transformed), 10.0 + std::cos(angle) - 2.0 * std::sin(angle), EPS);
        EXPECT_NEAR(getY(transformed), std::sin(angle) + 2.0 * std::cos(angle), EPS);
        EXPECT_EQ(transformed.header.frame_id, "map");
        // The same guard must work for callers transforming in place.
        tf2::doTransform(state, state, transform);
        EXPECT_EQ(state.continuous_state_covariance, transformed.continuous_state_covariance);
      }
    }
  }
}

TEST(tf2_perception_msgs, preserves_negative_cross_covariance_and_rotates_finite_covariance) {
  ObjectState state, transformed;
  initializeState(state, EGO::MODEL_ID);
  setContinuousStateCovarianceDiagonal(state, std::vector<double>(getContinuousStateSize(state), 4.0));
  setContinuousStateCovarianceAt(state, EGO::X, EGO::Y, -1.0);
  setContinuousStateCovarianceAt(state, EGO::Y, EGO::X, -1.0);
  gm::TransformStamped transform;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);
  tf2::doTransform(state, transformed, transform);
  const auto covariance = getPoseCovariance(transformed);
  EXPECT_NEAR(covariance[0], 5.0, EPS);
  EXPECT_NEAR(covariance[7], 3.0, EPS);
  EXPECT_NEAR(covariance[1], 0.0, EPS);
}

TEST(tf2_perception_msgs, unknown_cross_terms_and_invalid_diagonals_follow_rotation) {
  gm::PoseWithCovarianceStamped pose, transformed;
  pose.pose.pose.orientation.w = 1.0;
  pose.pose.covariance[0] = CONTINUOUS_STATE_COVARIANCE_INVALID;
  pose.pose.covariance[7] = CONTINUOUS_STATE_COVARIANCE_UNKNOWN;
  pose.pose.covariance[2] = pose.pose.covariance[12] = CONTINUOUS_STATE_COVARIANCE_UNKNOWN;
  gm::TransformStamped transform;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);
  transformPoseWithCovariance(pose, transformed, transform);
  EXPECT_EQ(transformed.pose.covariance[0], CONTINUOUS_STATE_COVARIANCE_INVALID);
  EXPECT_EQ(transformed.pose.covariance[7], CONTINUOUS_STATE_COVARIANCE_INVALID);
  for (const int i : {1, 6, 2, 12, 8, 13})
    EXPECT_EQ(transformed.pose.covariance[i], CONTINUOUS_STATE_COVARIANCE_UNKNOWN);
  EXPECT_EQ(transformed.pose.covariance[14], 0.0);
}

TEST(tf2_perception_msgs, velocity_and_acceleration_covariance_getters_preserve_markers) {
  for (const double marker : {CONTINUOUS_STATE_COVARIANCE_INVALID, CONTINUOUS_STATE_COVARIANCE_UNKNOWN}) {
    ObjectState state;
    initializeState(state, EGO::MODEL_ID);
    setYaw(state, M_PI_4);
    setContinuousStateCovarianceDiagonal(state, std::vector<double>(getContinuousStateSize(state), 4.0));
    setContinuousStateCovarianceAt(state, EGO::VEL_LON, EGO::VEL_LON, marker);
    setContinuousStateCovarianceAt(state, EGO::ACC_LON, EGO::ACC_LON, marker);
    for (const auto& pose : {getVelocityXYZWithCovariance(state), getAccelerationXYZWithCovariance(state)}) {
      EXPECT_EQ(pose.covariance[0], marker);
      EXPECT_EQ(pose.covariance[7], marker);
      EXPECT_EQ(pose.covariance[1], CONTINUOUS_STATE_COVARIANCE_UNKNOWN);
      EXPECT_EQ(pose.covariance[14], 0.0);
    }
  }
}

TEST(tf2_perception_msgs, velocity_and_acceleration_covariance_setters_preserve_markers) {
  for (const double marker : {CONTINUOUS_STATE_COVARIANCE_INVALID, CONTINUOUS_STATE_COVARIANCE_UNKNOWN}) {
    ObjectState state;
    initializeState(state, EGO::MODEL_ID);
    gm::Vector3 vector;
    vector.x = 1.0;
    setVelocityXYZYawWithCovariance(state, vector, M_PI_4, marker, 4.0, 0.0);
    setAccelerationXYZYawWithCovariance(state, vector, M_PI_4, marker, 4.0, 0.0);
    for (const auto pair : {std::array<int, 2>{EGO::VEL_LON, EGO::VEL_LAT},
                            std::array<int, 2>{EGO::ACC_LON, EGO::ACC_LAT}}) {
      EXPECT_EQ(getContinuousStateCovarianceAt(state, pair[0], pair[0]), marker);
      EXPECT_EQ(getContinuousStateCovarianceAt(state, pair[1], pair[1]), marker);
      EXPECT_EQ(getContinuousStateCovarianceAt(state, pair[0], pair[1]), CONTINUOUS_STATE_COVARIANCE_UNKNOWN);
    }
  }
}

TEST(tf2_perception_msgs, nested_states_preserve_markers_under_three_axis_rotation) {
  Object object, transformed;
  initializeState(object, EGO::MODEL_ID);
  setContinuousStateCovarianceDiagonal(object, std::vector<double>(getContinuousStateSize(object), CONTINUOUS_STATE_COVARIANCE_UNKNOWN));
  object.state_predictions.emplace_back();
  object.state_predictions.back().states.push_back(object.state);
  gm::TransformStamped transform;
  tf2::Quaternion rotation;
  rotation.setRPY(0.3, 0.4, 0.5);
  transform.transform.rotation = tf2::toMsg(rotation);
  tf2::doTransform(object, transformed, transform);
  EgoData ego, transformed_ego;
  ego.state = object.state;
  ego.trajectory_planned = {object.state};
  ego.trajectory_past = {object.state};
  tf2::doTransform(ego, transformed_ego, transform);
  for (const auto& state : {transformed.state, transformed.state_predictions.front().states.front(),
                           transformed_ego.state, transformed_ego.trajectory_planned.front(),
                           transformed_ego.trajectory_past.front()}) {
    const auto covariance = getPoseCovariance(state);
    for (int i = 0; i < 6; ++i) EXPECT_EQ(covariance[7 * i], CONTINUOUS_STATE_COVARIANCE_UNKNOWN);
    for (const auto value : covariance) EXPECT_TRUE(std::isfinite(value));
  }
}


TEST(tf2_perception_msgs, transforms_ego_trajectories_with_optional_covariance) {
  for (const auto model : std::vector<unsigned char>{EGO::MODEL_ID, EGORWS::MODEL_ID}) {
    EgoData ego, transformed;
    initializeState(ego, model);
    setPose(ego, {1.0, 2.0, 3.0}, {0.0, 0.0, 0.0});
    setVelLon(ego, 4.0);
    setContinuousStateCovarianceDiagonal(ego, std::vector<double>(getContinuousStateSize(ego.state), 0.1));
    ego.trajectory_planned = {ego.state, ego.state};
    ego.trajectory_planned.front().continuous_state_covariance.clear();
    ego.trajectory_past = ego.trajectory_planned;

    gm::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.transform.translation.x = 10.0;
    transform.transform.translation.y = 20.0;
    transform.transform.rotation.z = 1.0;
    transform.transform.rotation.w = 0.0;
    tf2::doTransform(ego, transformed, transform);

    EXPECT_EQ(transformed.header.frame_id, "map");
    EXPECT_NEAR(getX(transformed.state), 9.0, EPS);
    EXPECT_NEAR(getY(transformed.state), 18.0, EPS);
    EXPECT_NEAR(getContinuousStateCovarianceAt(transformed.state, 0, 0), 0.1, EPS);
    for (const auto& trajectory : {transformed.trajectory_planned, transformed.trajectory_past}) {
      ASSERT_EQ(trajectory.size(), 2u);
      EXPECT_TRUE(trajectory.front().continuous_state_covariance.empty());
      EXPECT_EQ(trajectory.back().continuous_state_covariance.size(), ego.state.continuous_state_covariance.size());
      for (const auto& state : trajectory) {
        EXPECT_EQ(state.header.frame_id, "map");
        EXPECT_NEAR(getX(state), 9.0, EPS);
        EXPECT_NEAR(getY(state), 18.0, EPS);
        EXPECT_NEAR(std::abs(getYaw(state)), M_PI, EPS);
        EXPECT_NEAR(getVelLon(state), 4.0, EPS);
      }
      EXPECT_DOUBLE_EQ(getContinuousStateCovarianceAt(trajectory.front(), 0, 0), CONTINUOUS_STATE_COVARIANCE_INVALID);
      EXPECT_NEAR(getContinuousStateCovarianceAt(trajectory.back(), 0, 0), 0.1, EPS);
    }
    EXPECT_FALSE(ego.state.continuous_state_covariance.empty());
    EXPECT_TRUE(ego.trajectory_planned.front().continuous_state_covariance.empty());
    EXPECT_NEAR(getX(ego.state), 1.0, EPS);
  }
}


int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
