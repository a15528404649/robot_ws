cmake_minimum_required(VERSION 3.8)
project(yzz_detect)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Result.msg"
  "msg/ImageResults.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

install(PROGRAMS
  scripts/yolo_detect.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detect
)
install(DIRECTORY
  scripts/models
  scripts/utils
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/yolov5s.pt
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
