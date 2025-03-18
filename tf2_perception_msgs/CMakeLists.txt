cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(tf2_perception_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

    find_package(ament_cmake REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(perception_msgs REQUIRED)
    find_package(perception_msgs_utils REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)

    add_library(${PROJECT_NAME} INTERFACE)

    target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
    ament_export_dependencies(perception_msgs perception_msgs_utils tf2 tf2_geometry_msgs tf2_ros)
 
    install(
        DIRECTORY include/
        DESTINATION include
    )

    install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include
    )

    if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        ament_add_gtest(${PROJECT_NAME}-test test/test_tf2_perception_msgs.ros2.cpp)
        target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        )
        target_include_directories(${PROJECT_NAME}-test PUBLIC test)
        ament_target_dependencies(${PROJECT_NAME}-test
            geometry_msgs
            perception_msgs
            perception_msgs_utils
            rclcpp
            tf2
            tf2_geometry_msgs
            tf2_ros
        )
    endif()

  ament_package()

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

    # add_compile_definitions(ROS1) must also be set in the CMakeLists.txt of the package that depends on this package!
    add_compile_definitions(ROS1)

    find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        perception_msgs
        perception_msgs_utils
        roscpp
        tf2
        tf2_geometry_msgs
    )

    catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
            geometry_msgs
            perception_msgs
            perception_msgs_utils
            roscpp
            tf2
            tf2_geometry_msgs
    )

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    catkin_add_gtest(${PROJECT_NAME}-test test/test_tf2_perception_msgs.cpp)
    if(TARGET ${PROJECT_NAME}-test)
        target_include_directories(${PROJECT_NAME}-test PUBLIC test)
    endif()

endif()
