# AGENTS.md

# YZZ Bunker Mini ROS 2 Project

## 1. 开发环境

- 硬件：NVIDIA Jetson AGX Orin
- 系统：Ubuntu 24.04
- ROS：ROS 2 Jazzy
- 工作空间：`~/robot_ws`
- 构建工具：`colcon`

每次新开终端，先执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
```

## 2. 源码分类

```text
~/robot_ws/src/
├── DrvPackage/                         # 驱动包
│   ├── yzz_bunker_mini_base/            # 底盘驱动、bunker_msgs、ugv_sdk
│   ├── yzz_imu/                         # WIT IMU 驱动
│   └── yzz_lidar/                       # YDLidar 驱动与 scan_sanitizer
├── FuncPackage/                         # 功能包
│   ├── yzz_decription/                  # URDF、STL、静态 TF、RViz 模型配置
│   └── yzz_navigation2/                 # EKF、SLAM、Nav2 参数与地图
└── yzz_robotlaunch/                     # 方案启动包
```

`colcon` 会递归发现上述目录中的 ROS 2 包。目录分类名称不是 ROS 包名；启动时始终使用下面的 ROS 包名。

## 3. 功能包说明

### yzz_bunker_mini_base

- 路径：`src/DrvPackage/yzz_bunker_mini_base/yzz_bunker_mini_base`
- 底盘 CAN 驱动，发布里程计、接收 `/cmd_vel`。
- 同级 `bunker_msgs` 和 `ugv_sdk` 是底盘驱动的支持包，不要随意移动或修改。
- CAN 接口为 `can_usb`，波特率为 `500000`。

### yzz_imu

- 路径：`src/DrvPackage/yzz_imu`
- WIT IMU 串口驱动，发布 `/imu/data`，坐标系为 `imu_link`。
- 单独启动：`ros2 launch yzz_imu rviz_and_imu.launch.py`

### yzz_lidar

- 路径：`src/DrvPackage/yzz_lidar`
- YDLidar 驱动发布原始扫描；其中 `scan_sanitizer` 将 `/scan_raw` 重采样后发布为 `/scan`。
- 雷达设备使用 `/dev/ydlidar`。

### yzz_decription

- 路径：`src/FuncPackage/yzz_decription`
- 包含 URDF/Xacro、STL 网格、传感器安装位姿和 RViz 机器人模型配置。
- 负责发布 `robot_description` 及静态 TF。
- 主要坐标系：`base_link`、`imu_link`、`laser_link`。
- 单独查看模型：`ros2 launch yzz_decription display.launch.py`。

### yzz_navigation2

- 路径：`src/FuncPackage/yzz_navigation2`
- 保存 EKF、SLAM、Nav2 参数和地图。
- 地图目录：`src/FuncPackage/yzz_navigation2/maps/`。
- 参数目录：`config/` 和 `config/nav2/`。

### yzz_robotlaunch

- 路径：`src/yzz_robotlaunch`
- 系统集成启动包，统一启动底盘、IMU、雷达、机器人模型、EKF、SLAM 和 Nav2。
- `navigation_gate` 会在 RViz 使用 **2D Pose Estimate** 产生 `map -> odom` 关系后，再启动 Nav2 的规划与控制节点。
- 仅保留方案启动职责；地图、导航参数和 RViz 模型配置分别由 `yzz_navigation2`、`yzz_decription` 管理。

## 4. 常用命令

### 编译

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 基础系统

```bash
ros2 launch yzz_robotlaunch robot_bringup.launch.py
```

启动底盘、IMU、雷达、`robot_state_publisher`、EKF 和 `scan_sanitizer`。

### 一键建图

```bash
ros2 launch yzz_robotlaunch mapping_system.launch.py
```

### 保存地图

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot_ws/src/FuncPackage/yzz_navigation2/maps/<地图名> \
  --ros-args -p save_map_timeout:=20.0
```

该命令生成同名的 `.yaml` 与 `.pgm` 文件。

### 一键导航

使用默认地图 `bunker_map.yaml`：

```bash
ros2 launch yzz_robotlaunch navigation_system.launch.py
```

使用指定地图：

```bash
ros2 launch yzz_robotlaunch navigation_system.launch.py \
  map:=/home/orin/robot_ws/src/FuncPackage/yzz_navigation2/maps/<地图名>.yaml
```

导航启动后，先在 RViz 点击 **2D Pose Estimate** 设置机器人初始位置和朝向；看到 `navigation_gate` 提示 Nav2 已启动后，再使用 **2D Goal Pose** 下发目标点。

## 5. 修改注意事项

- 不要将 ROS 1 工程直接放入 `~/robot_ws/src`，避免被 colcon 误扫描。
- 修改包名或移动包目录后，检查 `package.xml`、`setup.py`/`CMakeLists.txt`、launch 引用和地图路径。
- 当前导航默认地图通过 `FindPackageShare("yzz_navigation2")` 自动定位，不依赖工作空间中的旧绝对路径。
- `yzz_camera` 尚未加入工作空间；以后加入时归入 `src/DrvPackage/`。
