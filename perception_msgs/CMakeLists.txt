cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

ament_export_dependencies(
    geometry_msgs
    sensor_msgs
    std_msgs
    vision_msgs
    rosidl_default_runtime
)

set(msg_files
    "msg/ImageRegion.msg"
    "msg/ImageRegionArray.msg"
    "msg/ObjectAttribute.msg"
    "msg/Detection2DAttributes.msg"
    "msg/Detection2DAttributesArray.msg"
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
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES geometry_msgs std_msgs sensor_msgs vision_msgs
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_image_interfaces test/test_image_interfaces.py)
endif()

ament_package()
