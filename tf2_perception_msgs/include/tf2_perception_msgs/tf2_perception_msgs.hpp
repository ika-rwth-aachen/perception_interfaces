// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <perception_msgs/msg/object.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tf2 {
  namespace gm = geometry_msgs::msg;
  using namespace perception_msgs::msg;
  using Time = tf2::TimePoint;
#ifndef STAMP2TIME
#define STAMP2TIME
  inline Time stampToTime(const builtin_interfaces::msg::Time& t) {
    return tf2_ros::fromMsg(t);
  }
#endif
}

#include <tf2_perception_msgs/impl/tf2_perception_msgs.h>
