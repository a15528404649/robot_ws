cmake_minimum_required(VERSION 3.8)
project(yzz_transfer)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CombinedMapData.msg"
  "msg/WebsocketSignal.msg"
  DEPENDENCIES nav_msgs std_msgs
)

install(PROGRAMS
  scripts/map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_and_scan_forwarder
)
install(PROGRAMS
  scripts/heartbeat.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME heartbeat
)
install(PROGRAMS
  scripts/result.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME result
)
install(PROGRAMS
  scripts/env_data.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME env_data
)
install(PROGRAMS
  scripts/http_transmitter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME http_transmitter
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
