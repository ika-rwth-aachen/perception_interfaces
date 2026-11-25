cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_export_dependencies(
    geometry_msgs
    std_msgs
    sensor_msgs
)

set(msg_files
    "msg/BOX2D.msg"
    "msg/ImageObservation.msg"
    "msg/PointCloudObservation.msg"
    "msg/Observation.msg"
    "msg/ObjectWithObservation.msg"
    "msg/ObjectListWithObservation.msg"

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
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

ament_package()
