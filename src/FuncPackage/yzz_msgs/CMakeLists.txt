cmake_minimum_required(VERSION 3.8)
project(yzz_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Battery.msg"
  "msg/GetHolder.msg"
  "msg/Imu.msg"
  "msg/Infrared.msg"
  "msg/Lifts.msg"
  "msg/PID.msg"
  "msg/Sensor.msg"
  "msg/Servo.msg"
  "msg/SetHolder.msg"
  "msg/Ultrasonic.msg"
  "msg/Velocities.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
