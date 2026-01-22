cmake_minimum_required(VERSION 3.8)
project(perception_msgs_rviz_plugins)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

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
  find_package(perception_msgs_rendering REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rviz_common REQUIRED)
  find_package(rviz_default_plugins REQUIRED)
  find_package(rviz_ogre_vendor REQUIRED)
  find_package(rviz_rendering REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(map_msgs REQUIRED)
  
  # Conditional resource_retriever for ROS distro compatibility
  if(NOT ROS_DISTRO STREQUAL "noetic"
      AND NOT ROS_DISTRO STREQUAL "humble"
      AND NOT ROS_DISTRO STREQUAL "jazzy")
    find_package(resource_retriever REQUIRED)
  endif()

  set(display_headers_to_moc
    include/perception_msgs/displays/object_list/object_list_display.hpp
    include/perception_msgs/displays/ego_data/ego_data_display.hpp
    include/perception_msgs/displays/traffic_light/traffic_light_display.hpp
    include/perception_msgs/displays/attention_vector/attention_vector_display.hpp
    include/perception_msgs/displays/attention_grid/attention_grid_display.hpp
    include/perception_msgs/displays/attention_compass/attention_compass_display.hpp
    include/perception_msgs/displays/attention_hud/attention_hud_display.hpp
    include/perception_msgs/displays/attention_aura3d/attention_aura3d_display.hpp
    include/perception_msgs/displays/attention_cloud/attention_cloud_display.hpp
    include/perception_msgs/displays/attention_uncertainty/attention_uncertainty_display.hpp
    include/perception_msgs/displays/uncertainty_graph/uncertainty_graph_display.hpp
    include/perception_msgs/displays/vehicle_sensor_hud/vehicle_sensor_hud_display.hpp
    include/perception_msgs/displays/model_card/model_card_display.hpp
    include/perception_msgs/displays/data_card/data_card_display.hpp
  )

  foreach(header "${display_headers_to_moc}")
    qt5_wrap_cpp(display_moc_files "${header}")
  endforeach()

  set(display_source_files
    src/displays/object_list/object_list_display.cpp
    src/displays/ego_data/ego_data_display.cpp
    src/displays/traffic_light/traffic_light_display.cpp
    src/displays/attention_vector/attention_vector_display.cpp
    src/displays/attention_grid/attention_grid_display.cpp
    src/displays/attention_grid/attention_swatch.cpp
    src/displays/attention_compass/attention_compass_display.cpp
    src/displays/attention_hud/attention_hud_display.cpp
    src/displays/attention_aura3d/attention_aura3d_display.cpp
    src/displays/attention_cloud/attention_cloud_display.cpp
    src/displays/attention_uncertainty/attention_uncertainty_display.cpp
    src/displays/uncertainty_graph/uncertainty_graph_display.cpp
    src/displays/vehicle_sensor_hud/vehicle_sensor_hud_display.cpp
    src/displays/model_card/model_card_display.cpp
    src/displays/data_card/data_card_display.cpp
  )

  add_library(${PROJECT_NAME} SHARED
    ${display_moc_files}
    ${display_source_files}
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${Qt5Widgets_INCLUDE_DIRS}
  )

  target_link_libraries(${PROJECT_NAME} PUBLIC
    rviz_ogre_vendor::OgreMain
    rviz_ogre_vendor::OgreOverlay
  )

  pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

  ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
    geometry_msgs   
    rclcpp
    perception_msgs
    perception_msgs_rendering
    rviz_common
    rviz_default_plugins
    rviz_rendering
    tf2
    tf2_ros
    nav_msgs
    map_msgs
  )

  # Conditional resource_retriever target dependencies
  if(NOT ROS_DISTRO STREQUAL "noetic"
      AND NOT ROS_DISTRO STREQUAL "humble"
      AND NOT ROS_DISTRO STREQUAL "jazzy")
    ament_target_dependencies(${PROJECT_NAME} PUBLIC resource_retriever)
  endif()

  # Export modern CMake targets
  ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

  ament_export_dependencies(
    geometry_msgs
    perception_msgs
    perception_msgs_rendering
    rclcpp
    rviz_common
    rviz_default_plugins
    rviz_ogre_vendor
    tf2
    tf2_ros
    nav_msgs
    map_msgs
  )

  # Conditional resource_retriever export dependencies
  if(NOT ROS_DISTRO STREQUAL "noetic"
      AND NOT ROS_DISTRO STREQUAL "humble"
      AND NOT ROS_DISTRO STREQUAL "jazzy")
    ament_export_dependencies(resource_retriever)
  endif()

  # Adds shallow folders containing ogre_media such as meshes and scripts to Rviz
  # so that they can be found at runtime. Requires rviz_rendering.
  register_rviz_ogre_media_exports(
    DIRECTORIES
    "meshes"
  )

  # Install mesh files
  install(
    DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # RViz Display End

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
