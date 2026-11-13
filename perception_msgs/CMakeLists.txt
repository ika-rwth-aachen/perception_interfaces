cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

ament_export_dependencies(
    geometry_msgs
    std_msgs
    sensor_msgs
    vision_msgs
    rosidl_default_runtime
)

set(msg_files
    "msg/EgoData.msg"
    "msg/Object.msg"
    "msg/ObjectClassification.msg"
    "msg/ObjectList.msg"
    "msg/ObjectReferencePoint.msg"
    "msg/ObjectState.msg"
    "msg/ObjectStatePrediction.msg"
    "msg/EGO.msg"
    "msg/EGORWS.msg"
    "msg/ISCACTR.msg"
    "msg/HEXAMOTION.msg"
    "msg/TRAFFICLIGHT.msg"
    "msg/TRAFFICSIGN.msg"
    "msg/ImageObservation.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES geometry_msgs std_msgs sensor_msgs vision_msgs
)

ament_package()
