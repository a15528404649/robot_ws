# bunker_mini_bringup

`bunker_mini_bringup` 是 BUNKER MINI 机器人整车集成与启动配置包。

这个包不负责底盘、雷达或 IMU 的底层驱动，而是把现有功能包统一组织起来，方便后续实现建图、定位和自主导航。

## 主要用途

该包后续用于统一管理：

- BUNKER MINI 底盘驱动
- IMU 驱动
- 激光雷达驱动
- URDF 与 robot_state_publisher
- TF 坐标变换
- robot_localization EKF 融合
- slam_toolbox 建图
- Nav2 定位与导航
- RViz 显示配置
- 地图文件

## 依赖的现有功能包

当前机器人工作空间中已有：

- `bunker_base`
  - 底盘驱动
  - 发布 `/odom`
  - 接收 `/cmd_vel`
  - 发布 `odom -> base_link`

- `bunker_mini_description`
  - 机器人 URDF/Xacro
  - 机器人模型
  - 传感器安装位置
  - 静态 TF

- `wit_ros2_imu`
  - IMU 串口驱动
  - 发布 `/imu/data`
  - 使用坐标系 `imu_link`

- `ydlidar_ros2_driver`
  - 激光雷达驱动
  - 发布激光扫描数据

- `robot_localization`
  - 融合底盘里程计与 IMU
  - 输出滤波后的里程计

- `slam_toolbox`
  - 二维激光建图

- `nav2`
  - 地图定位
  - 路径规划
  - 路径跟踪
  - 动态避障

## 目录结构

```text
bunker_mini_bringup/
├── config/
│   ├── ekf.yaml
│   ├── slam_toolbox.yaml
│   └── nav2_params.yaml
├── launch/
│   ├── robot_bringup.launch.py
│   ├── localization.launch.py
│   ├── mapping.launch.py
│   └── navigation.launch.py
├── maps/
│   ├── map.yaml
│   └── map.pgm
├── rviz/
│   └── bunker_mini.rviz
├── bunker_mini_bringup/
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
