cmake_minimum_required(VERSION 3.8)
project(yzz_detect_pest)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/yolov5_det_trt.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yzz_detect_pest
)
install(DIRECTORY
  scripts/models
  scripts/utils
  lib
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
