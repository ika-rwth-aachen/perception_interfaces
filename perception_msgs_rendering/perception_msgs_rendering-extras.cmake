# find package Qt5 because otherwise using the perception_msgs_rendering:perception_msgs_rendering
# exported target will complain that the Qt5::Widgets target does not exist
find_package(Qt5 REQUIRED QUIET COMPONENTS Widgets)