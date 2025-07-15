cmake_minimum_required(VERSION 3.8)
project(perception_msgs_rendering)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})

add_compile_definitions(ROS_DISTRO_${ROS_DISTRO})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)
  
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  # uncomment the following section in order to fill in
  # further dependencies manually.
  # find_package(<dependency> REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(perception_msgs REQUIRED)
  find_package(perception_msgs_utils REQUIRED)
  find_package(rviz_common REQUIRED)
  find_package(rviz_ogre_vendor REQUIRED)
  find_package(rviz_rendering REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  if(NOT ROS_DISTRO STREQUAL "noetic"
      AND NOT ROS_DISTRO STREQUAL "humble"
      AND NOT ROS_DISTRO STREQUAL "jazzy")
    find_package(resource_retriever REQUIRED)
  endif()

  # Adds shallow folders containing ogre_media such as meshes and scripts to Rviz
  # so that they can be found at runtime. Requires rviz_rendering.
  register_rviz_ogre_media_exports(
    DIRECTORIES
    "meshes"
    "materials"
  )

  add_library(${PROJECT_NAME} SHARED
    src/rendering/object_state/object_state.cpp
    src/rendering/traffic_light/traffic_light.cpp
  )

  target_link_libraries(${PROJECT_NAME} PUBLIC
    rviz_ogre_vendor::OgreMain
    rviz_ogre_vendor::OgreOverlay
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${Qt5Widgets_INCLUDE_DIRS}
  )

  ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
    geometry_msgs   
    perception_msgs
    perception_msgs_utils
    rviz_common
    rviz_rendering
    tf2_geometry_msgs
  )
  if(NOT ROS_DISTRO STREQUAL "noetic"
      AND NOT ROS_DISTRO STREQUAL "humble"
      AND NOT ROS_DISTRO STREQUAL "jazzy")
    ament_target_dependencies(${PROJECT_NAME} PUBLIC resource_retriever)
  endif()

  ament_export_dependencies(
    geometry_msgs
    perception_msgs
    perception_msgs_utils
    rviz_common
    rviz_ogre_vendor
    tf2_geometry_msgs
  )

  # Export old-style CMake variables
  ament_export_include_directories("include/${PROJECT_NAME}")
  ament_export_libraries(${PROJECT_NAME})

  # Export modern CMake targets
  ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(
    DIRECTORY include/ meshes/ materials/
    DESTINATION include/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    #find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    #set(ament_cmake_cpplint_FOUND TRUE)
    #ament_lint_auto_find_test_dependencies()
  endif()

  ament_package(
    CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
  )

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

  # Currently no support for ROS1!
  # Specify that the directory should be excluded from the install target
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
          DESTINATION .
          EXCLUDE_FROM_ALL)

endif()
