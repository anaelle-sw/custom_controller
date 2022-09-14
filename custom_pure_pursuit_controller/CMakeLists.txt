cmake_minimum_required(VERSION 3.5)
project(custom_pure_pursuit_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)
set(library_name pure_pursuit_controller)
set(dependencies
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    nav2_costmap_2d
    pluginlib
    nav_msgs
    nav2_util
    nav_2d_utils
    nav2_core
    tf2
    tf2_geometry_msgs
)

add_library(pure_pursuit_controller SHARED
    src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller
    rclcpp
    geometry_msgs
    nav2_costmap_2d
    pluginlib
    nav_msgs
    nav2_util
    nav2_core
    tf2
    tf2_geometry_msgs
)

add_library(progress_checker SHARED
    plugins/progress_checker.cpp)
ament_target_dependencies(progress_checker
    rclcpp
    rclcpp_lifecycle
    pluginlib
    geometry_msgs
    nav2_core
    nav2_util
    nav_2d_utils
    nav_msgs
    custom_msgs
)


install(TARGETS pure_pursuit_controller progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND FALSE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(pure_pursuit_controller progress_checker)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core custom_pure_pursuit_controller.xml)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_package()

