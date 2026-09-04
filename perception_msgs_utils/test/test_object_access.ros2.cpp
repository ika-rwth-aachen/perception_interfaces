// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object.hpp>

#include <perception_msgs_utils/object_access.hpp>

using namespace perception_msgs;
using namespace perception_msgs::msg;

namespace gm = geometry_msgs::msg;

#include <impl/test_object_access.cpp>
