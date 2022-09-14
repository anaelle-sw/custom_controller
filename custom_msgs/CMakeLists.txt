cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

# For building custom messages
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_CXX_STANDARD 17)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProgressStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()

