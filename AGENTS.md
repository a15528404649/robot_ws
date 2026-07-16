# AGENTS.md

# Bunker Mini ROS2 Robot Project

## 1. Project Overview

这是一个基于 NVIDIA Jetson AGX Orin 的 ROS 2 移动机器人项目。

当前开发环境：

- Hardware: NVIDIA Jetson AGX Orin
- OS: Ubuntu 24.04
- ROS Version: ROS 2 Jazzy
- Workspace: ~/robot_ws
- Source directory: ~/robot_ws/src
- Build system: colcon

工作空间结构：

~/robot_ws

├── build
├── install
├── log
├── maps
└── src


---

# 2. Source Packages


## yzz_robotlaunch

路径：

~/robot_ws/src/yzz_robotlaunch


作用：

机器人系统集成启动包。

负责将底盘、IMU、激光雷达、机器人模型、EKF、SLAM、Nav2等模块组合启动。


主要功能：

- 启动机器人硬件系统
- 启动robot_state_publisher
- 启动EKF(robot_localization)
- 启动SLAM
- 启动Nav2定位
- 启动Nav2导航


主要launch文件：

## robot_bringup.launch.py

作用：

启动机器人基础系统。

包含：

- Bunker Mini底盘驱动
- YDLIDAR驱动
- WIT IMU驱动
- robot_state_publisher
- EKF
- scan_sanitizer


启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch yzz_robotlaunch robot_bringup.launch.py
mapping.launch.py

作用：

启动slam_toolbox进行实时建图。

功能：

机器人运动过程中：

接收激光雷达数据
接收机器人运动信息
生成地图

启动命令：

source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch yzz_navigation2 mapping.launch.py
mapping_system.launch.py

作用：

一键启动建图系统。

启动命令：

ros2 launch yzz_robotlaunch mapping_system.launch.py
nav2_localization.launch.py

作用：

启动地图加载和AMCL定位。

输入：

地图yaml文件。

启动示例：

source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch yzz_navigation2 nav2_localization.launch.py \
map:=/home/orin/robot_ws/src/yzz_navigation2/maps/bunker_map.yaml
nav2_navigation.launch.py

作用：

启动Nav2导航系统。

包含：

planner_server
controller_server
bt_navigator
Nav2相关导航节点

启动命令：

source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch yzz_navigation2 nav2_navigation.launch.py
3. yzz_decription

路径：

~/robot_ws/src/yzz_decription

作用：

机器人模型描述包。

包含：

URDF
Xacro
STL模型
RViz模型显示配置

负责：

robot_description
TF静态关系

主要frame：

base_link

机器人主体坐标。

imu_link

IMU坐标。

laser_link

激光雷达坐标。

当前机器人坐标约定：

X轴：

机器人前方

Y轴：

机器人左侧

Z轴：

机器人上方

查看机器人模型：

cd ~/robot_ws

source install/setup.bash

ros2 launch yzz_decription display.launch.py
4. yzz_bunker_mini_base

路径：

~/robot_ws/src/yzz_bunker_mini_base

作用：

AgileX Bunker Mini ROS2底盘驱动。

主要功能：

CAN通信
接收速度控制命令
发布底盘状态
发布里程计

相关ROS包：

bunker_base

bunker_msgs

启动底盘：

ros2 launch bunker_base bunker_base.launch.py

CAN接口：

can_usb

CAN波特率：

500000

恢复CAN接口：

sudo ip link set can_usb down

sudo ip link set can_usb type can bitrate 500000

sudo ip link set can_usb up

不要随意修改该包源码。

5. ugv_sdk

路径：

~/robot_ws/src/yzz_bunker_mini_base/ugv_sdk

作用：

AgileX底盘SDK。

负责：

CAN协议通信
Bunker Mini底层通信支持

该包属于底层SDK。

除非确认底盘通信问题，否则不要修改。

6. yzz_imu

路径：

~/robot_ws/src/yzz_imu

作用：

WIT IMU ROS2驱动。

启动：

cd ~/robot_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch yzz_imu rviz_and_imu.launch.py

IMU话题：

/imu/data

IMU frame：

imu_link

查看IMU频率：

ros2 topic hz /imu/data

当前IMU安装方向：

X轴：

机器人前方

Y轴：

机器人左侧

Z轴：

机器人上方

7. yzz_lidar

路径：

~/robot_ws/src/yzz_lidar

作用：

YDLIDAR ROS2驱动。

当前雷达：

YDLIDAR TG30

设备：

/dev/ydlidar

话题：

/scan

frame：

laser_link

启动：

ros2 launch yzz_lidar ydlidar_4ros_view_launch.py
8. 摄像头

当前使用：

usb_cam

启动：

ros2 run usb_cam usb_cam_node_exe \
--ros-args \
-p video_device:=/dev/my_camera

查看图像：

ros2 run rqt_image_view rqt_image_view
9. RViz远程显示

虚拟机启动Orin上的RViz：

~/start_orin_rviz.sh
10. 地图保存

地图保存目录：

~/robot_ws/maps

保存地图：

source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

mkdir -p ~/robot_ws/maps


ros2 run nav2_map_server map_saver_cli \
-f ~/robot_ws/maps/bunker_map \
--ros-args -p save_map_timeout:=20.0

生成：

bunker_map.yaml

bunker_map.pgm

11. 导航流程
第一步：启动机器人基础系统
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch yzz_robotlaunch robot_bringup.launch.py

启动：

底盘
IMU
雷达
TF
EKF
第二步：启动定位系统
ros2 launch yzz_navigation2 nav2_localization.launch.py \
map:=/home/orin/robot_ws/src/yzz_navigation2/maps/bunker_map.yaml

检查：

ros2 lifecycle get /map_server

ros2 lifecycle get /amcl
第三步：启动导航
ros2 launch yzz_navigation2 nav2_navigation.launch.py

检查：

ros2 lifecycle get /planner_server

ros2 lifecycle get /controller_server
12. Development Rules for Codex

修改工程前：

必须先阅读：

AGENTS.md
README.md
package.xml
launch文件

修改原则：

不要删除已经工作的功能。
不要修改第三方驱动源码。
不要修改yzz_bunker_mini_base、ugv_sdk、yzz_lidar内部代码，除非明确要求。
修改前先分析依赖关系。
修改launch文件后检查：
package.xml
setup.py
install路径
launch引用

如果需要重命名文件：

必须同步修改所有引用。

13. Current Status

已经完成：

Bunker Mini底盘通信
CAN控制
YDLIDAR建图
IMU接入
URDF模型
TF
EKF
slam_toolbox建图
地图保存
AMCL定位
Nav2导航

后续开发：

GPS接入
摄像头应用
多传感器融合
工程整理
# ROS2 Python Coding Style


所有新开发ROS2 Python节点必须采用面向对象结构。


要求：

- 必须继承 rclpy.node.Node
- 必须使用class定义节点
- main函数只负责节点启动
- ROS接口必须在__init__中初始化
- callback函数独立定义
- 使用ROS2 parameter机制
- 不允许编写while True阻塞式ROS节点
- 保持节点模块化和可维护性


示例结构：

class ExampleNode(Node):

    def __init__(self):

        super().__init__('example_node')


    def callback(self,msg):

        pass


