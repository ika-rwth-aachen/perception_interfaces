// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <perception_msgs/EgoData.h>
#include <perception_msgs/Object.h>
#include <perception_msgs/ObjectList.h>
#include <perception_msgs_utils/object_access.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2 {
  namespace gm = geometry_msgs;
  using Time = ros::Time;
#ifndef STAMP2TIME
#define STAMP2TIME
  inline const Time& stampToTime(const ros::Time& t) {
    return t;
  }
#endif
}

#include <tf2_perception_msgs/impl/tf2_perception_msgs.h>
