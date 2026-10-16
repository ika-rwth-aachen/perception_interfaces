// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <perception_msgs/msg/object.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tf2_perception_msgs {
  inline tf2::TimePoint stampToTime(const builtin_interfaces::msg::Time& t) {
    return tf2_ros::fromMsg(t);
  }
}

namespace tf2 {
  namespace gm = geometry_msgs::msg;
  using namespace perception_msgs::msg;
  using Time = tf2::TimePoint;
}

namespace tf2 {

  using namespace perception_msgs;
  using namespace perception_msgs::object_access;

  template <>
  inline void doTransform(const ObjectState& state_in, ObjectState& state_out, const gm::TransformStamped& transform) {
    state_out = state_in;
    state_out.header.stamp = transform.header.stamp;
    state_out.header.frame_id = transform.header.frame_id;

    if (state_in.continuous_state_covariance.empty()) {
      gm::PoseStamped xyz_rpy, xyz_rpy_tf;
      xyz_rpy.pose = getPose(state_in);
      doTransform(xyz_rpy, xyz_rpy_tf, transform);
      setPose(state_out, xyz_rpy_tf.pose, false);
    } else {
      gm::PoseWithCovarianceStamped xyz_rpy_cov, xyz_rpy_cov_tf;
      xyz_rpy_cov.pose = getPoseWithCovariance(state_in);
      transformPoseWithCovariance(xyz_rpy_cov, xyz_rpy_cov_tf, transform);
      setPoseWithCovariance(state_out, xyz_rpy_cov_tf.pose);
    }
  }

  template <>
  inline void doTransform(const Object& obj_in, Object& obj_out, const gm::TransformStamped& transform) {

    obj_out = obj_in;
    auto base_time = obj_in.state.header.stamp;
    doTransform(obj_in.state, obj_out.state, transform);

    // Currently commented out as we are not sure how the history will be filled/interpreted
    // for (int i = 0; i < obj_in.state_history.size(); i++){
    //   doTransform(obj_in.state_history[i], obj_out.state_history[i], transform);
    // }

    for (int i = 0; i < obj_in.state_predictions.size(); i++) {
      for (int j = 0; j < obj_in.state_predictions[i].states.size(); j++) {
        // The state prediciton is in the future, but we want to tranform it with the current tf
        auto future_time = obj_in.state_predictions[i].states[j].header.stamp;
        auto state_in = obj_in.state_predictions[i].states[j];
        state_in.header.stamp = base_time;
        doTransform(state_in, obj_out.state_predictions[i].states[j], transform);
        // Restore the timestamp
        obj_out.state_predictions[i].states[j].header.stamp = future_time;
      }
    }
  }

  template <>
  inline void doTransform(const ObjectList& obj_list_in, ObjectList& obj_list_out, const gm::TransformStamped& transform) {
    obj_list_out = obj_list_in;
    obj_list_out.header.stamp = transform.header.stamp;
    obj_list_out.header.frame_id = transform.header.frame_id;

    for (int i = 0; i < obj_list_in.objects.size(); i++) {
      doTransform(obj_list_in.objects[i], obj_list_out.objects[i], transform);
    }
  }

  template <>
  inline void doTransform(const EgoData& ego_in, EgoData& ego_out, const gm::TransformStamped& transform) {

    ego_out = ego_in;
    ego_out.header.stamp = transform.header.stamp;
    ego_out.header.frame_id = transform.header.frame_id;

    doTransform(ego_in.state, ego_out.state, transform);

    for (int i = 0; i < ego_in.trajectory_planned.size(); i++) doTransform(ego_in.trajectory_planned[i], ego_out.trajectory_planned[i], transform);

    for (int i = 0; i < ego_in.trajectory_past.size(); i++) doTransform(ego_in.trajectory_past[i], ego_out.trajectory_past[i], transform);

    for (int i = 0; i < ego_in.route_planned.size(); i++) {
      doTransform(ego_in.route_planned[i], ego_out.route_planned[i], transform);
    }
  }

  template <>
  inline Time getTimestamp(const ObjectState& state) {
    Time t = tf2_perception_msgs::stampToTime(state.header.stamp);
    return t;
  }
  template <>
  inline Time getTimestamp(const Object& obj) {
    Time t = tf2_perception_msgs::stampToTime(obj.state.header.stamp);
    return t;
  }
  template <>
  inline Time getTimestamp(const ObjectList& obj_list) {
    Time t = tf2_perception_msgs::stampToTime(obj_list.header.stamp);
    return t;
  }
  template <>
  inline Time getTimestamp(const EgoData& ego) {
    Time t = tf2_perception_msgs::stampToTime(ego.header.stamp);
    return t;
  }

  template <>
  inline std::string getFrameId(const ObjectState& state) {
    return state.header.frame_id;
  }
  template <>
  inline std::string getFrameId(const Object& obj) {
    return obj.state.header.frame_id;
  }
  template <>
  inline std::string getFrameId(const ObjectList& obj_list) {
    return obj_list.header.frame_id;
  }
  template <>
  inline std::string getFrameId(const EgoData& ego) {
    return ego.header.frame_id;
  }
}
