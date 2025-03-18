# find package Qt5 because otherwise using the perception_msgs_rviz_plugins:perception_msgs_rviz_plugins
# exported target will complain that the Qt5::Widgets target does not exist
find_package(Qt5 REQUIRED QUIET COMPONENTS Widgets)