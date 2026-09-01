// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

/**
 * @file object_access.hpp
 * @brief Main object-access header to include in ROS 2 projects
 */

#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Messages
#include <perception_msgs/msg/object.hpp>
#include <perception_msgs/msg/object_classification.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <perception_msgs/msg/object_reference_point.hpp>
#include <perception_msgs/msg/object_state.hpp>

// State Models
#include <perception_msgs/msg/ego.hpp>
#include <perception_msgs/msg/egorws.hpp>
#include <perception_msgs/msg/iscactr.hpp>
#include <perception_msgs/msg/hexamotion.hpp>
#include <perception_msgs/msg/trafficlight.hpp>

namespace perception_msgs {
    namespace gm = geometry_msgs::msg;
    using namespace msg;
}

#include <perception_msgs_utils/impl/object_access.h>
