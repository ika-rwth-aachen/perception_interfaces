// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

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