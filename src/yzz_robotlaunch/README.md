# yzz_robotlaunch

`yzz_robotlaunch` 是 BUNKER MINI 机器人整车集成与启动配置包。

这个包不负责底盘、雷达或 IMU 的底层驱动，而是把现有功能包统一组织起来，提供机器人基础启动、建图与导航的一键启动命令。

## 主要用途

该包负责统一启动：

- BUNKER MINI 底盘驱动
- IMU 驱动
- 激光雷达驱动
- URDF 与 robot_state_publisher
- TF 坐标变换
- robot_localization EKF 融合
- slam_toolbox 建图
- Nav2 定位与导航

## 依赖的现有功能包

当前机器人工作空间中已有：

- `yzz_bunker_mini_base`
  - 底盘驱动
  - 发布 `/odom`
  - 接收 `/cmd_vel`
  - 发布 `odom -> base_link`

- `yzz_decription`
  - 机器人 URDF/Xacro
  - 机器人模型
  - 传感器安装位置
  - 静态 TF

- `yzz_imu`
  - IMU 串口驱动
  - 发布 `/imu/data`
  - 使用坐标系 `imu_link`

- `yzz_lidar`
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
yzz_robotlaunch/
├── launch/
│   ├── robot_bringup.launch.py
│   ├── mapping_system.launch.py
│   └── navigation_system.launch.py
├── yzz_robotlaunch/
│   ├── __init__.py
│   └── navigation_gate.py
├── resource/
│   └── yzz_robotlaunch
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

导航参数与地图位于 `yzz_navigation2`；机器人模型和 RViz 配置位于 `yzz_decription`。
这样该包只承担“整车方案启动”的职责。
