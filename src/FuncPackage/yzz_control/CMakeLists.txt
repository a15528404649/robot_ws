cmake_minimum_required(VERSION 3.8)
project(yzz_control)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VideoStreamKey.msg"
)

install(PROGRAMS scripts/control.py DESTINATION lib/${PROJECT_NAME} RENAME control)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
