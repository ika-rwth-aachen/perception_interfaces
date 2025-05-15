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

#pragma once

namespace tf2 {

  using namespace perception_msgs;
  using namespace perception_msgs::object_access;

  template <>
  inline void doTransform(const ObjectState& state_in, ObjectState& state_out, const gm::TransformStamped& transform) {
    state_out = state_in;
    state_out.header.stamp = transform.header.stamp;
    state_out.header.frame_id = transform.header.frame_id;

    gm::PoseWithCovarianceStamped xyz_rpy_cov, xyz_rpy_cov_tf;
    xyz_rpy_cov.pose = getPoseWithCovariance(state_in);
    doTransform(xyz_rpy_cov, xyz_rpy_cov_tf, transform);
    setPoseWithCovariance(state_out, xyz_rpy_cov_tf.pose);
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
  #ifdef ROS1
  #define TF2_PERCEPTION_MSGS_TIME_TYPE const Time&
  #else
  #define TF2_PERCEPTION_MSGS_TIME_TYPE Time
  #endif
  template <>
  inline TF2_PERCEPTION_MSGS_TIME_TYPE getTimestamp(const ObjectState& state) {
    TF2_PERCEPTION_MSGS_TIME_TYPE t = stampToTime(state.header.stamp);
    return t;
  }
  template <>
  inline TF2_PERCEPTION_MSGS_TIME_TYPE getTimestamp(const Object& obj) {
    TF2_PERCEPTION_MSGS_TIME_TYPE t = stampToTime(obj.state.header.stamp);
    return t;
  }
  template <>
  inline TF2_PERCEPTION_MSGS_TIME_TYPE getTimestamp(const ObjectList& obj_list) {
    TF2_PERCEPTION_MSGS_TIME_TYPE t = stampToTime(obj_list.header.stamp);
    return t;
  }
  template <>
  inline TF2_PERCEPTION_MSGS_TIME_TYPE getTimestamp(const EgoData& ego) {
    TF2_PERCEPTION_MSGS_TIME_TYPE t = stampToTime(ego.header.stamp);
    return t;
  }

  #ifdef ROS1
  #define TF2_PERCEPTION_MSGS_FRAME_TYPE const std::string&
  #else
  #define TF2_PERCEPTION_MSGS_FRAME_TYPE std::string
  #endif

  template <>
  inline TF2_PERCEPTION_MSGS_FRAME_TYPE getFrameId(const ObjectState& state) {
    return state.header.frame_id;
  }
  template <>
  inline TF2_PERCEPTION_MSGS_FRAME_TYPE getFrameId(const Object& obj) {
    return obj.state.header.frame_id;
  }
  template <>
  inline TF2_PERCEPTION_MSGS_FRAME_TYPE getFrameId(const ObjectList& obj_list) {
    return obj_list.header.frame_id;
  }
  template <>
  inline TF2_PERCEPTION_MSGS_FRAME_TYPE getFrameId(const EgoData& ego) {
    return ego.state.header.frame_id;
  }
}
