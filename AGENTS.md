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
│   ├── yzz_bunker_mini_base/            # 底盘 ROS2 驱动、消息接口、内部 UGV SDK
│   ├── yzz_imu/                         # WIT IMU 驱动
│   ├── yzz_lidar/                       # YDLidar 驱动与 scan_sanitizer
│   └── yzz_gps/                         # GPS 串口驱动（室外导航数据源）
├── FuncPackage/                         # 功能包
│   ├── yzz_battery_adapter/              # 底盘电压到通用电池状态的适配
│   ├── yzz_ptz_camera/                  # 免驱 PTZ 相机功能包
│   ├── yzz_decription/                  # URDF、STL、静态 TF、RViz 模型配置
│   └── yzz_navigation2/                 # EKF、SLAM、Nav2 参数与地图
└── yzz_robotlaunch/                     # 方案启动包
```

`colcon` 会递归发现上述目录中的 ROS 2 包。目录分类名称不是 ROS 包名；启动时始终使用下面的 ROS 包名。

## 3. 功能包说明

### yzz_bunker_mini_base

- 路径：`src/DrvPackage/yzz_bunker_mini_base`
- 单一 ROS2 底盘驱动包，发布里程计、接收 `/cmd_vel`。
- `msg/` 与 `launch/` 同级，生成 `yzz_bunker_mini_base/msg/*` 消息接口。
- `src/ugv_sdk/` 是主包内部 C++ SDK，不是独立 ROS 包；不要单独用 colcon 编译它。
- CAN 接口为 `can_usb`，波特率为 `500000`。

### yzz_imu

- 路径：`src/DrvPackage/yzz_imu`
- WIT IMU 串口驱动，发布 `/imu/data`，坐标系为 `imu_link`。
- Python 源码位于 `src/yzz_imu_driver/driver.py`；ROS 可执行名仍为 `yzz_imu`。
- 单独启动：`ros2 launch yzz_imu rviz_and_imu.launch.py`

### yzz_lidar

- 路径：`src/DrvPackage/yzz_lidar`
- YDLidar 驱动发布原始扫描；其中 `scan_sanitizer` 将 `/scan_raw` 重采样后发布为 `/scan`。
- 雷达设备使用 `/dev/ydlidar`。

### yzz_gps

- 路径：`src/DrvPackage/yzz_gps`
- GPS 串口驱动，设备为 `/dev/gps_usb`，波特率为 `115200`，传感器坐标系为 `gps_link`。
- 发布：`/gps/fix`（`sensor_msgs/NavSatFix`）、`/gps/vel`（有效 RMC 时）和 `/gps/nmea`。
- 单独启动：`ros2 launch yzz_gps gps_driver.launch.py`。
- 在室内，`/gps/fix` 的 `status: -1` 和经纬度 `NaN` 表示尚未取得卫星定位；移至室外、天线无遮挡后应取得有效坐标。
- 当前室内建图/AMCL 导航仍使用现有里程计和激光定位，尚未与 GPS 融合；GPS-IMU 室外融合模式须在室外确认有效定位后再单独配置，以避免与 AMCL 的 `map -> odom` 冲突。

### yzz_battery_adapter

- 路径：`src/FuncPackage/yzz_battery_adapter`
- 将底盘 `/bunker_status` 的电压转换为 `/battery_state`（`yzz_msgs/Battery`），供状态上报和网页功能使用。
- 不参与底盘控制、建图或导航；百分比基于电压曲线估算，不是 BMS 直接提供的 SOC。

### yzz_ptz_camera

- 路径：`src/FuncPackage/yzz_ptz_camera`
- 通过系统 V4L2/UVC 接口使用免驱 PTZ 相机，发布 `usb_cam/image_raw`，并提供 `/SetHolder`、`/GetHolder` 云台控制接口。
- 虽归入功能包，仍依赖实际 `/dev/video*` 相机设备及其访问权限。

### yzz_webrtc_streamer

- 路径：`src/FuncPackage/yzz_webrtc_streamer`
- ROS2 WHIP/WebRTC 推流功能包；默认订阅 PTZ 图像 `/usb_cam/image_raw`，可通过启动参数改为原 ROS1 虫情相机话题 `/usb_cam_2/image_raw_2`。
- 监听 `/videostream_push_status`：`1` 启动推流，`0` 停止推流；空闲时不订阅图像、不连接公网。
- 使用平台 WHIP 地址和现有设备注册码，运行依赖为 Ubuntu 包 `python3-aiortc`、`python3-av`。
- 单独启动：`ros2 launch yzz_webrtc_streamer yzz_webrtc_streamer.launch.py`。

### yzz_waypoint_nav

- 路径：`src/FuncPackage/yzz_waypoint_nav`
- 运行在 Orin 上的 Nav2 多点巡航执行器；不直接发布 `/cmd_vel`，只逐点调用 Nav2 的 `navigate_to_pose` Action。
- 航点路线按地图保存到 `~/robot_ws/src/FuncPackage/yzz_waypoint_nav/data/waypoints/<地图名>/<路线名>.json`，与多点导航功能包集中管理。
- 保留 ROS1 巡逻参数：`rest_time`、`keep_patrol`、`random_patrol`、`patrol_type`、`patrol_loop`、`patrol_time`、`potrol_points_num`。其中 `potrol_points_num=0` 表示使用路线全部航点。
- 由 `yzz_web_mapping` 启动；空闲时不会发导航目标。加载地图并设置初始位姿后，才可从网页开始巡逻。

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

### GPS 驱动测试

```bash
ros2 launch yzz_gps gps_driver.launch.py
```

另开终端检查：

```bash
ros2 topic echo /gps/fix
```

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
- Jetson 内核升级后，CH340 的 `ch341` 外部内核模块需要按新内核版本重新编译并安装，否则 `/dev/gps_usb` 不会出现。

### yzz_web_mapping

- Path: `src/FuncPackage/yzz_web_mapping`.
- Local-LAN web console; it does not modify or auto-start the existing mapping/navigation launch files.
- Start: `ros2 launch yzz_web_mapping web_mapping.launch.py`; open `http://<Orin-IP>:8080` from Windows.
- It can start existing mapping/navigation launch files, save maps, publish initial poses and goals. Web teleoperation is disabled by default; explicitly use `allow_teleop:=true` to enable it.
- Saved maps use `src/FuncPackage/yzz_navigation2/maps/`, the same directory used by the existing navigation system.
