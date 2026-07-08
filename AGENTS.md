# AGENTS.md

## 项目概况

这是一个基于 NVIDIA Jetson AGX Orin 的 ROS 2 移动机器人项目。

当前开发环境：

- 计算平台：NVIDIA Jetson AGX Orin
- 操作系统：Ubuntu 24.04
- ROS 2 版本：Jazzy
- ROS 2 工作区：`~/robot_ws`
- 源码目录：`~/robot_ws/src`
- 构建工具：`colcon`

项目最终目标：

1. 接入激光雷达
2. 接入移动机器人底盘
3. 接入 IMU
4. 接入 GPS
5. 接入摄像头
6. 建立完整的机器人 TF 树
7. 完成机器人 URDF/Xacro 模型
8. 实现里程计
9. 实现场地建图
10. 实现定位
11. 使用 Nav2 实现自主导航
12. 完成多传感器和整机集成

## 当前进度

### 已完成

- YDLIDAR ROS 2 驱动已经放入工作区
- 雷达驱动已经成功编译
- 雷达可以通过以下命令启动：

```bash
ros2 launch ydlidar_ros2_driver ydlidar_4ros_view_launch.py