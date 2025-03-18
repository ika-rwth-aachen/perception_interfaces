cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(perception_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

    find_package(ament_cmake REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(std_msgs REQUIRED)

    ament_export_dependencies(
        geometry_msgs
        std_msgs
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
    )

    rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES geometry_msgs std_msgs
    )

    ament_package()

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

    find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        message_generation
        std_msgs
    )

    add_message_files(DIRECTORY msg)

    generate_messages(
        DEPENDENCIES
            geometry_msgs
            std_msgs
    )

    catkin_package(
        CATKIN_DEPENDS
            geometry_msgs
            message_runtime
            std_msgs
    )

endif()
