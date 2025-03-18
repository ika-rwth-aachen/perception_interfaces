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
 * @file object_access.h
 * @brief Main object-access header to include in ROS 1 projects
 */

#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Messages
#include <perception_msgs/Object.h>
#include <perception_msgs/ObjectClassification.h>
#include <perception_msgs/ObjectList.h>
#include <perception_msgs/ObjectReferencePoint.h>
#include <perception_msgs/ObjectState.h>

// State Models
#include <perception_msgs/EGO.h>
#include <perception_msgs/EGORWS.h>
#include <perception_msgs/ISCACTR.h>
#include <perception_msgs/HEXAMOTION.h>
#include <perception_msgs/TRAFFICLIGHT.h>

namespace perception_msgs {
    namespace gm = geometry_msgs;
}

#include <perception_msgs_utils/impl/object_access.h>