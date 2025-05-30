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
