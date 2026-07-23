cmake_minimum_required(VERSION 3.8)
project(yzz_gps)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gps.msg"
)

ament_python_install_package(yzz_gps_driver PACKAGE_DIR src/yzz_gps_driver)

install(PROGRAMS
  scripts/gps_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
