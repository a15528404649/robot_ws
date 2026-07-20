cmake_minimum_required(VERSION 3.8)
project(yzz_bunker_mini_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 本包直接生成 Bunker Mini 的 ROS 消息接口。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BunkerActuatorState.msg"
  "msg/BunkerRCState.msg"
  "msg/BunkerStatus.msg"
  DEPENDENCIES std_msgs
)

# ugv_sdk 是内部 C++ SDK，而不是独立 ROS 包。
add_subdirectory(src/ugv_sdk EXCLUDE_FROM_ALL)
set(CMAKE_CXX_STANDARD 14)

add_executable(${PROJECT_NAME}_node
  src/bunker_base_ros.cpp
  src/bunker_base_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node
  ugv_sdk
  "${cpp_typesupport_target}"
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
